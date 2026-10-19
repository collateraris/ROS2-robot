"""Conservative 2D camera / planar LiDAR association, never a 3D-center estimator."""
from dataclasses import dataclass
import math
import numpy as np
from .calibration import PinholeCamera


@dataclass(frozen=True)
class FusionConfig:
    min_points:int=3
    depth_jump_m:float=.25
    max_neighbor_gap_m:float=.18
    min_foreground_contrast_m:float=.30
    max_depth_spread_m:float=.30
    bbox_margin_fraction:float=.03
    max_center_offset_fraction:float=.32
    max_sync_s:float=.12
    max_image_age_s:float=.5
    max_scan_age_s:float=.3
    min_score:float=.35
    min_uncertainty_m:float=.12

    def __post_init__(self):
        if self.min_points<3 or self.depth_jump_m<=0 or self.max_neighbor_gap_m<=0:
            raise ValueError("Fusion needs >=3 consistent points and positive distance thresholds")
        if not 0<self.max_sync_s<=.2 or not 0<self.max_image_age_s<=.5 or not 0<self.max_scan_age_s<=.5:
            raise ValueError("Fusion timestamp bounds are too permissive")
        if not 0<=self.bbox_margin_fraction<.25 or not 0<self.max_center_offset_fraction<=.5:
            raise ValueError("Invalid bounding-box margins")


def freshness_reason(image_ns,scan_ns,now_ns,config=FusionConfig()):
    if min(image_ns,scan_ns)<=0: return "missing_acquisition_timestamp"
    if image_ns>now_ns or scan_ns>now_ns: return "future_acquisition_timestamp"
    if (now_ns-image_ns)*1e-9>config.max_image_age_s: return "stale_image"
    if (now_ns-scan_ns)*1e-9>config.max_scan_age_s: return "stale_scan"
    if abs(image_ns-scan_ns)*1e-9>config.max_sync_s: return "unsynchronized_scan_image"
    return None


def scan_points(ranges,angle_min,angle_increment,range_min,range_max):
    r=np.asarray(ranges,dtype=float)
    if r.ndim!=1 or not np.isfinite([angle_min,angle_increment,range_min,range_max]).all() or angle_increment==0 or not 0<range_min<range_max:
        raise ValueError("Invalid LaserScan geometry")
    a=angle_min+np.arange(len(r))*angle_increment
    valid=np.isfinite(r)&(r>=range_min)&(r<=range_max)
    points=np.stack((np.where(valid,r,0)*np.cos(a),np.where(valid,r,0)*np.sin(a),np.zeros_like(a)),axis=-1)
    return points,valid,r,a


def projected_scan(points,valid,camera:PinholeCamera,camera_from_lidar):
    t=np.asarray(camera_from_lidar,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all(): raise ValueError("Invalid optical projection transform")
    p=np.asarray(points)@t[:3,:3].T+t[:3,3]
    positive=valid&(p[:,2]>.1)
    z=np.where(positive,p[:,2],1.)
    uv=np.stack((camera.fx*p[:,0]/z+camera.cx,camera.fy*p[:,1]/z+camera.cy),-1)
    inside=positive&(uv[:,0]>=0)&(uv[:,0]<camera.width)&(uv[:,1]>=0)&(uv[:,1]<camera.height)
    return uv,inside


def _cluster_indices(indices,points,ranges,config):
    clusters=[]
    for index in indices:
        if not clusters or index-clusters[-1][-1]!=1 or abs(ranges[index]-ranges[clusters[-1][-1]])>config.depth_jump_m or np.linalg.norm(points[index]-points[clusters[-1][-1]])>config.max_neighbor_gap_m:
            clusters.append([int(index)])
        else: clusters[-1].append(int(index))
    return clusters


def ground_detection(detection,*,ranges,angle_min,angle_increment,range_min,range_max,camera,camera_from_lidar,map_from_lidar,image_stamp_ns,scan_stamp_ns,now_ns,config=FusionConfig()):
    """Return (grounded object, rejection reason).

    A compact foreground cluster must have valid farther returns immediately on
    BOTH sides. A wall merely visible through/behind an RGB person box fails
    that test. This deliberately rejects many real but unresolvable detections.
    The output position is a planar laser-return centroid, not object center.
    """
    reason=freshness_reason(image_stamp_ns,scan_stamp_ns,now_ns,config)
    if reason: return None,reason
    label=str(detection.get("label","")); score=float(detection.get("score",0.))
    box=np.asarray(detection.get("bbox_xyxy",[]),dtype=float)
    if box.shape!=(4,) or not np.isfinite(box).all() or box[2]<=box[0] or box[3]<=box[1] or not np.isfinite(score): return None,"invalid_detection"
    if score<config.min_score: return None,"low_score"
    points,valid,r,a=scan_points(ranges,angle_min,angle_increment,range_min,range_max)
    # A full-circle driver may use 0..2pi, putting forward returns across the
    # array seam. Move that seam behind the camera before adjacency clustering.
    optical=np.asarray(camera_from_lidar,dtype=float)
    forward=optical[:3,:3].T@np.array([0.,0.,1.])
    forward_angle=math.atan2(forward[1],forward[0])
    order=np.argsort((a-forward_angle+np.pi)%(2*np.pi)-np.pi)
    points,valid,r,a=points[order],valid[order],r[order],a[order]
    uv,projectable=projected_scan(points,valid,camera,camera_from_lidar)
    w,h=box[2]-box[0],box[3]-box[1]; m=config.bbox_margin_fraction
    selected=np.flatnonzero(projectable&(uv[:,0]>=box[0]+m*w)&(uv[:,0]<=box[2]-m*w)&(uv[:,1]>=box[1]+m*h)&(uv[:,1]<=box[3]-m*h))
    if len(selected)<config.min_points: return None,"insufficient_projected_returns"
    candidates=[]
    width_bounds={"person":(.06,.90),"chair":(.06,1.20),"table":(.08,2.),"door":(.12,1.60)}
    for group in _cluster_indices(selected,points,r,config):
        if len(group)<config.min_points: continue
        lo,hi=group[0],group[-1]
        if lo==0 or hi==len(r)-1 or not valid[lo-1] or not valid[hi+1]: continue
        # Crop boundaries on a continuous background wall are not depth edges.
        if min(r[lo-1]-r[lo],r[hi+1]-r[hi])<config.min_foreground_contrast_m: continue
        if np.ptp(r[group])>config.max_depth_spread_m: continue
        span=float(np.linalg.norm(points[hi]-points[lo]))
        low,high=width_bounds.get(label,(.06,1.5))
        if not low<=span<=high: continue
        if abs(float(np.median(uv[group,0]))-(box[0]+box[2])/2)>config.max_center_offset_fraction*w: continue
        candidates.append(group)
    if not candidates: return None,"no_isolated_foreground_cluster"
    if len(candidates)!=1: return None,"ambiguous_foreground_clusters"
    group=candidates[0]; point=np.median(points[group],axis=0)
    t=np.asarray(map_from_lidar,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all(): return None,"invalid_map_transform"
    mapped=t[:3,:3]@point+t[:3,3]
    if not np.isfinite(mapped).all(): return None,"invalid_map_position"
    spread=float(np.median(np.linalg.norm(points[group]-point,axis=1)))
    # Motion of the robot is compensated by timestamped TF; object motion is not.
    object_speed=1.5 if label=="person" else .2
    temporal=object_speed*((now_ns-image_stamp_ns)+abs(scan_stamp_ns-image_stamp_ns))*1e-9
    uncertainty=max(config.min_uncertainty_m,spread)+temporal
    return {"label":label,"score":score,"position":{"x":float(mapped[0]),"y":float(mapped[1]),"z":float(mapped[2])},"uncertainty_m":float(uncertainty),"source":"camera_lidar","last_seen_ns":int(scan_stamp_ns),"image_stamp_ns":int(image_stamp_ns),"scan_stamp_ns":int(scan_stamp_ns),"bbox_xyxy":box.tolist(),"range_m":float(np.linalg.norm(point[:2])),"bearing_rad":float(math.atan2(point[1],point[0])),"support_points":len(group),"localization":"planar_lidar_cluster_not_3d_center"},None


def reject_conflicting_associations(observations,distance_m=.18):
    """One return cluster cannot establish two overlapping semantic objects."""
    rejected=set()
    for i,left in enumerate(observations):
        for j in range(i+1,len(observations)):
            right=observations[j]; a,b=left["position"],right["position"]
            if math.hypot(a["x"]-b["x"],a["y"]-b["y"])<distance_m:
                rejected.update((i,j))
    return [o for i,o in enumerate(observations) if i not in rejected],len(rejected)
