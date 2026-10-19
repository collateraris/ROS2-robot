from pathlib import Path
import math
import numpy as np
import pytest
from room_semantics.calibration import PinholeCamera,transform_matrix,load_calibration
from room_semantics.geometry import ground_detection,FusionConfig,freshness_reason,reject_conflicting_associations
from room_semantics.tracking import ObjectTracker

CAMERA=PinholeCamera(640,480,500.,500.,320.,240.)
OPTICAL=transform_matrix([0,0,0],[.5,-.5,.5,.5])


def fixture_scene():
    angles=np.linspace(-.3,.3,61); ranges=np.full(61,4.)
    ranges[np.abs(angles)<=.04]=2.
    return dict(ranges=ranges,angle_min=float(angles[0]),angle_increment=float(angles[1]-angles[0]),range_min=.05,range_max=12.,
                camera=CAMERA,camera_from_lidar=OPTICAL,map_from_lidar=np.eye(4),image_stamp_ns=1_000_000_000,
                scan_stamp_ns=1_010_000_000,now_ns=1_050_000_000)


def detection(label="person",bbox=(230,100,410,400)):
    return dict(label=label,score=.9,bbox_xyxy=list(bbox))


def test_foreground_cluster_gets_metric_laser_plane_point_and_source_stamps():
    result,reason=ground_detection(detection(),**fixture_scene())
    assert reason is None
    assert result["position"]["x"]==pytest.approx(2.,abs=.005)
    assert result["position"]["y"]==pytest.approx(0.,abs=.015)
    assert result["position"]["z"]==0.  # scan plane, never inferred person torso center
    assert result["support_points"]>=3
    assert result["source"]=="camera_lidar"
    assert result["last_seen_ns"]==1_010_000_000
    assert result["image_stamp_ns"]==1_000_000_000
    assert result["localization"]=="planar_lidar_cluster_not_3d_center"
    assert result["uncertainty_m"]>.12


def test_background_wall_inside_person_bbox_is_not_grounded():
    scene=fixture_scene(); scene["ranges"][:]=4.
    result,reason=ground_detection(detection(),**scene)
    assert result is None
    assert reason=="no_isolated_foreground_cluster"


def test_full_circle_scan_seam_at_forward_direction_is_supported():
    scene=fixture_scene(); a=np.arange(720)*2*np.pi/720
    scene.update(ranges=np.where(np.abs((a+np.pi)%(2*np.pi)-np.pi)<=.04,2.,4.),angle_min=0.,angle_increment=2*np.pi/720)
    result,reason=ground_detection(detection(),**scene)
    assert reason is None and result["position"]["x"]==pytest.approx(2.,abs=.005)


def test_two_supported_foreground_clusters_are_ambiguous():
    scene=fixture_scene(); scene["ranges"][:]=4.
    angles=scene["angle_min"]+np.arange(61)*scene["angle_increment"]
    scene["ranges"][(np.abs(angles-.08)<=.03)|(np.abs(angles+.08)<=.03)]=2.
    result,reason=ground_detection(detection(),**scene)
    assert result is None and reason=="ambiguous_foreground_clusters"


def test_missing_noisy_or_sparse_lidar_is_not_filled_in():
    scene=fixture_scene(); scene["ranges"][:]=np.inf
    assert ground_detection(detection(),**scene)[1]=="insufficient_projected_returns"
    scene=fixture_scene(); scene["ranges"][:]=4.; scene["ranges"][30]=2.
    assert ground_detection(detection(),**scene)[0] is None


def test_metric_position_uses_source_time_map_transform():
    scene=fixture_scene(); t=np.eye(4); t[:2,3]=[5.,-3.]; scene["map_from_lidar"]=t
    result,reason=ground_detection(detection(),**scene)
    assert reason is None
    assert result["position"]["x"]==pytest.approx(7.,abs=.005)
    assert result["position"]["y"]==pytest.approx(-3.,abs=.015)


def test_stale_future_or_unsynchronized_acquisition_rejected():
    c=FusionConfig()
    assert freshness_reason(1_000_000_000,1_010_000_000,1_600_000_000,c)=="stale_image"
    assert freshness_reason(1_000_000_000,1_200_000_000,1_220_000_000,c)=="unsynchronized_scan_image"
    assert freshness_reason(1_300_000_000,1_200_000_000,1_220_000_000,c)=="future_acquisition_timestamp"


def test_default_real_calibration_cannot_enable_metric_fusion():
    p=Path(__file__).resolve().parents[1]/"ros2_ws/src/room_perception/config/calibration.yaml"
    c=load_calibration(p)
    assert not c.verified and not c.image_is_rectified
    with pytest.raises(ValueError,match="verified"): c.verify_tf(c.camera_from_lidar)


def test_sim_calibration_matches_declared_base_frames_and_rejects_wrong_tf():
    p=Path(__file__).resolve().parents[1]/"weeks/week02/calibration.sim.yaml"
    c=load_calibration(p,require_verified=True)
    c.verify_tf(c.camera_from_lidar)
    # laser origin in base minus camera origin, rotated into optical axes.
    np.testing.assert_allclose(c.camera_from_lidar[:3,3],[0,-.02,-.085])
    invalid=c.camera_from_lidar.copy(); invalid[0,3]+=.10
    with pytest.raises(ValueError,match="disagrees"): c.verify_tf(invalid)


def test_camera_info_rejects_stereo_skew_or_invalid_pinhole():
    projection=[500,0,320,0,0,500,240,0,0,0,1,0]
    camera=PinholeCamera.from_projection(640,480,projection,"camera_optical_frame")
    assert camera.fx==500
    projection[3]=10.
    with pytest.raises(ValueError,match="monocular"): PinholeCamera.from_projection(640,480,projection,"camera_optical_frame")


def test_tracker_stable_ids_expire_on_source_stamp_not_inference_refresh():
    observation,_=ground_detection(detection(),**fixture_scene())
    tracker=ObjectTracker(); tracker.update([observation],1_050_000_000)
    first=tracker.scene(1_050_000_000)["objects"][0]
    second=dict(observation,last_seen_ns=1_110_000_000,scan_stamp_ns=1_110_000_000,image_stamp_ns=1_100_000_000)
    tracker.update([second],1_150_000_000)
    current=tracker.scene(1_150_000_000)["objects"][0]
    assert current["id"]==first["id"]
    tracker.update([second],1_750_000_000)
    assert len(tracker.scene(1_750_000_000)["objects"])==1
    assert tracker.scene(2_000_000_000)["objects"]==[]


def test_conflicting_semantic_boxes_cannot_both_claim_the_same_return():
    obj,_=ground_detection(detection(),**fixture_scene())
    assert reject_conflicting_associations([obj,dict(obj,label="chair")])==([],2)
