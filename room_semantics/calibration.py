"""Validated rectified pinhole camera + measured optical/LiDAR extrinsics."""
from dataclasses import dataclass
from pathlib import Path
import math
import numpy as np
import yaml


def transform_matrix(translation, quaternion_xyzw):
    t=np.asarray(translation,dtype=float); q=np.asarray(quaternion_xyzw,dtype=float)
    if t.shape!=(3,) or q.shape!=(4,) or not np.isfinite(t).all() or not np.isfinite(q).all():
        raise ValueError("Extrinsics need finite translation[3], quaternion_xyzw[4]")
    norm=float(np.linalg.norm(q))
    if abs(norm-1.)>1e-3:
        raise ValueError("Extrinsic quaternion must be normalized")
    x,y,z,w=q/norm
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    out=np.eye(4); out[:3,:3]=r; out[:3,3]=t
    return out


@dataclass(frozen=True)
class PinholeCamera:
    width:int
    height:int
    fx:float
    fy:float
    cx:float
    cy:float
    frame_id:str="camera_optical_frame"

    def __post_init__(self):
        if self.width<1 or self.height<1 or not self.frame_id:
            raise ValueError("Invalid camera shape/frame")
        if not np.isfinite([self.fx,self.fy,self.cx,self.cy]).all() or self.fx<=0 or self.fy<=0:
            raise ValueError("Invalid pinhole intrinsics")
        if not 0<=self.cx<self.width or not 0<=self.cy<self.height:
            raise ValueError("Principal point must lie in image")

    @classmethod
    def from_projection(cls,width,height,p,frame_id):
        p=np.asarray(p,dtype=float).reshape(3,4)
        if not np.isfinite(p).all() or not np.allclose(p[2],[0,0,1,0],atol=1e-6):
            raise ValueError("CameraInfo P must be a finite rectified pinhole projection")
        if not np.allclose([p[0,1],p[1,0],p[0,3],p[1,3]],[0,0,0,0],atol=1e-6):
            raise ValueError("Only zero-skew monocular rectified projection is supported")
        return cls(int(width),int(height),p[0,0],p[1,1],p[0,2],p[1,2],frame_id)


@dataclass(frozen=True)
class Calibration:
    camera_frame:str
    lidar_frame:str
    map_frame:str
    fixed_frame:str
    camera_from_lidar:np.ndarray
    verified:bool
    image_is_rectified:bool
    max_translation_error_m:float=.02
    max_rotation_error_rad:float=.03

    def verify_tf(self,actual):
        actual=np.asarray(actual,dtype=float)
        if actual.shape!=(4,4) or not np.isfinite(actual).all():
            raise ValueError("Invalid optical <- lidar TF")
        if not self.verified or not self.image_is_rectified:
            raise ValueError("Measured calibration + rectified/pinhole image must be explicitly verified")
        distance=np.linalg.norm(actual[:3,3]-self.camera_from_lidar[:3,3])
        r=self.camera_from_lidar[:3,:3].T@actual[:3,:3]
        angle=math.acos(float(np.clip((np.trace(r)-1)/2,-1,1)))
        if distance>self.max_translation_error_m or angle>self.max_rotation_error_rad:
            raise ValueError("Runtime optical/LiDAR TF disagrees with measured calibration")


def load_calibration(path,require_verified=False):
    value=yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value,dict) or value.get("schema_version")!=1:
        raise ValueError("Unsupported calibration schema_version")
    frames=[value.get(k) for k in ("camera_frame","lidar_frame","map_frame","fixed_frame")]
    if any(not isinstance(k,str) or not k or k.startswith("/") for k in frames):
        raise ValueError("Calibration needs nonempty ROS frame IDs without leading slash")
    if len(set(frames))!=4:
        raise ValueError("Camera, LiDAR, map and odom frames must be distinct")
    verified=value.get("verified")
    rectified=value.get("image_is_rectified")
    if type(verified) is not bool or type(rectified) is not bool:
        raise ValueError("Calibration verified/image_is_rectified must be boolean")
    extrinsics=value.get("camera_from_lidar",{})
    t=transform_matrix(extrinsics.get("translation_m"),extrinsics.get("quaternion_xyzw"))
    limits=value.get("tf_tolerance",{})
    translation=float(limits.get("translation_m",.02)); rotation=float(limits.get("rotation_rad",.03))
    if not 0<translation<=.05 or not 0<rotation<=.1:
        raise ValueError("Extrinsic TF tolerances must be positive and <=5cm / 0.1rad")
    c=Calibration(*frames,t,verified,rectified,translation,rotation)
    if require_verified: c.verify_tf(t)
    return c
