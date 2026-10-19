"""Week 3 cross-layer contracts using real pure modules, no ROS/Gazebo claims.

The input scan/image boxes below are controlled synthetic fixtures. Fusion,
tracking, publication JSON, parsing, symbolic planning and metric validation are
the production implementations. No detector weights or transport are mocked.
"""
import copy
import json
import math

import numpy as np
import pytest

from room_semantics.calibration import PinholeCamera, transform_matrix
from room_semantics.geometry import ground_detection, reject_conflicting_associations
from room_semantics.tracking import ObjectTracker
from room_planning.backends import make_plan
from room_planning.core import (MetricGoal, OccupancyGrid, PlanRejected,
                                parse_scene, standoff_goal, validate_grounding)


IMAGE_NS=10_000_000_000
SCAN_NS=IMAGE_NS+10_000_000
NOW_NS=IMAGE_NS+50_000_000
CAMERA=PinholeCamera(640,480,500.,500.,320.,240.)
CAMERA_FROM_LIDAR=transform_matrix([0,0,0],[.5,-.5,.5,.5])


def observation(*,angles=None,ranges=None,bbox=(230,100,410,400),
                map_from_lidar=None,offset_ns=0):
    """A box enclosing a supported near cluster against a farther wall."""
    if angles is None:
        angles=np.linspace(-.3,.3,61)
    if ranges is None:
        # Symmetric inclusive endpoints despite binary floating point linspace.
        ranges=np.where(np.abs(angles)<=.04+1e-12,2.,4.)
    value,reason=ground_detection(
        {'label':'person','score':.9,'bbox_xyxy':list(bbox)},
        ranges=ranges,angle_min=float(angles[0]),
        angle_increment=float(angles[1]-angles[0]),range_min=.05,range_max=12.,
        camera=CAMERA,camera_from_lidar=CAMERA_FROM_LIDAR,
        map_from_lidar=np.eye(4) if map_from_lidar is None else map_from_lidar,
        image_stamp_ns=IMAGE_NS+offset_ns,scan_stamp_ns=SCAN_NS+offset_ns,
        now_ns=NOW_NS+offset_ns)
    assert reason is None and value is not None
    return value


def published_scene(tracker,now_ns=NOW_NS):
    """Use the actual publisher payload including its ancillary evidence fields."""
    document=tracker.scene(now_ns,frame_id='map',status='ok',rejections={})
    return document,parse_scene(json.dumps(document,allow_nan=False))


def free_grid(data=None):
    # Map extents [-3,3] x [-3,3], with enough room for the full robot footprint.
    return OccupancyGrid(120,120,.05,-3.,-3.,0.,
                         tuple([0]*(120*120) if data is None else data))


def test_real_fusion_publication_to_symbolic_and_metric_navigation_goal():
    # A source-time map transform rotates laser +X into map +Y; numerical
    # coordinates must survive all interfaces rather than reverting to the box.
    map_from_lidar=transform_matrix([2.,-1.,0.],[0.,0.,math.sqrt(.5),math.sqrt(.5)])
    observed=observation(map_from_lidar=map_from_lidar)
    trusted,rejected=reject_conflicting_associations([observed])
    assert rejected==0
    tracker=ObjectTracker();tracker.update(trusted,NOW_NS)
    payload,scene=published_scene(tracker)
    assert payload['schema_version']==1 and payload['frame_id']=='map'
    assert payload['objects'][0]['source']=='camera_lidar'
    obj=scene.objects[0]
    assert obj.id==payload['objects'][0]['id']
    assert scene.stamp_ns==scene.image_stamp_ns==IMAGE_NS
    assert scene.scan_stamp_ns==obj.last_seen_ns==SCAN_NS
    assert payload['published_ns']==NOW_NS  # publication does not replace acquisition
    assert obj.x==pytest.approx(2.,abs=.005)
    assert obj.y==pytest.approx(1.,abs=.005)
    assert obj.z==0.  # position remains a LiDAR-plane return, not a torso center

    instruction='подъедь к человеку; подожди 1 секунду'
    plan=make_plan('rules',instruction,scene,set())
    assert plan.document()=={'steps':[{'action':'goto_object','object_id':obj.id},
                                    {'action':'wait','seconds':1.}]}
    assert validate_grounding(plan,instruction,scene,NOW_NS)
    goal=standoff_goal(2.,-1.,obj,.8)
    assert goal.x==pytest.approx(2.,abs=.005)
    assert goal.y==pytest.approx(.2,abs=.005)
    assert goal.yaw==pytest.approx(math.pi/2)
    assert math.hypot(goal.x-obj.x,goal.y-obj.y)==pytest.approx(.8)
    assert free_grid().check(goal,.23)


@pytest.mark.parametrize('occupancy',[-1,100])
def test_grounded_target_still_requires_free_metric_goal_footprint(occupancy):
    tracker=ObjectTracker();tracker.update([observation()],NOW_NS)
    _,scene=published_scene(tracker)
    plan=make_plan('rules','к человеку',scene,set())
    assert validate_grounding(plan,'к человеку',scene,NOW_NS)
    goal=standoff_goal(0.,0.,scene.lookup(plan.steps[0].object_id),.8)
    assert free_grid().check(goal,.23)
    cells=[0]*(120*120)
    # Occupy a neighbour touched by the footprint while leaving its center free.
    gx=math.floor((goal.x+3.)/.05)
    gy=math.floor((goal.y+3.)/.05)
    cells[gy*120+gx+3]=occupancy
    assert cells[gy*120+gx]==0
    with pytest.raises(PlanRejected,match='unknown/occupied'):
        free_grid(cells).check(goal,.23)


def test_delayed_scene_cannot_refresh_acquisition_with_publication_metadata():
    tracker=ObjectTracker();tracker.update([observation()],NOW_NS)
    payload,scene=published_scene(tracker)
    plan=make_plan('rules','к человеку',scene,set())
    delayed_ns=NOW_NS+1_200_000_000
    # A buffered payload may be republished with a new delivery/publication
    # field. Grounding must still use the original camera/scan/object stamps.
    payload['published_ns']=delayed_ns
    delayed=parse_scene(json.dumps(payload,allow_nan=False))
    with pytest.raises(PlanRejected,match='acquisition time is stale'):
        validate_grounding(plan,'к человеку',delayed,delayed_ns)
    assert delayed.image_stamp_ns==IMAGE_NS
    assert delayed.lookup(plan.steps[0].object_id).last_seen_ns==SCAN_NS


def test_repeated_scan_does_not_extend_target_lifetime_or_reuse_expired_id():
    observed=observation();tracker=ObjectTracker()
    tracker.update([observed],NOW_NS)
    _,initial=published_scene(tracker)
    old_id=initial.objects[0].id
    instruction=f'к {old_id}'
    old_plan=make_plan('rules',instruction,initial,set())
    # Repeated inference on one acquisition cannot keep a vanished person alive.
    for age in (.1,.3,.6):tracker.update([observed],NOW_NS+int(age*1e9))
    _,lost=published_scene(tracker,NOW_NS+900_000_000)
    assert not lost.objects
    with pytest.raises(PlanRejected,match='unknown or lost'):
        validate_grounding(old_plan,instruction,lost,NOW_NS+900_000_000)
    tracker.update([observation(offset_ns=1_000_000_000)],NOW_NS+1_000_000_000)
    _,reobserved=published_scene(tracker,NOW_NS+1_000_000_000)
    assert reobserved.objects[0].id!=old_id
    with pytest.raises(PlanRejected,match='unknown or lost'):
        validate_grounding(old_plan,instruction,reobserved,NOW_NS+1_000_000_000,
                           reference=initial)


def test_two_real_foreground_associations_require_an_explicit_observed_id():
    angles=np.linspace(-.4,.4,81)
    ranges=np.where((np.abs(angles-.18)<=.03)|(np.abs(angles+.18)<=.03),2.,4.)
    observations=[observation(angles=angles,ranges=ranges,bbox=box) for box in
                  ((175,100,285,400),(355,100,465,400))]
    trusted,rejected=reject_conflicting_associations(observations)
    assert len(trusted)==2 and rejected==0
    tracker=ObjectTracker();tracker.update(trusted,NOW_NS)
    payload,scene=published_scene(tracker)
    assert len(scene.objects)==2
    assert len({row['id'] for row in payload['objects']})==2
    assert all(obj.label=='person' for obj in scene.objects)
    with pytest.raises(PlanRejected,match='ambiguous person'):
        make_plan('rules','подъедь к человеку',scene,set())
    identifier=scene.objects[0].id
    instruction=f'к {identifier}'
    explicit=make_plan('rules',instruction,scene,set())
    assert explicit.steps[0].object_id==identifier
    assert validate_grounding(explicit,instruction,scene,NOW_NS)
    goal=standoff_goal(0.,0.,scene.lookup(identifier),.8)
    assert isinstance(goal,MetricGoal) and free_grid().check(goal,.23)


@pytest.mark.parametrize('source',['llm','synthetic','gazebo_ground_truth'])
def test_published_coordinates_cannot_be_relabeled_as_a_trusted_source(source):
    tracker=ObjectTracker();tracker.update([observation()],NOW_NS)
    payload,_=published_scene(tracker)
    modified=copy.deepcopy(payload)
    modified['objects'][0]['source']=source
    with pytest.raises(PlanRejected,match='trusted camera_lidar'):
        parse_scene(json.dumps(modified,allow_nan=False))
