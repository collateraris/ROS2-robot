import json
import math
from concurrent.futures import Future
import pytest

from room_planning.core import (GenerationGate,MetricGoal,OccupancyGrid,Plan,PlanRejected,
    Scene,Step,parse_command,parse_plan,parse_scene,planar_quaternion,standoff_goal,
    strict_json,validate_grounding,validate_path,validate_grid_path)


NOW=10_000_000_000


def scene_doc(objects=None):
    obj={"id":"person_0001","label":"person","score":.9,"position":{"x":2.,"y":1.,"z":.15},
         "uncertainty_m":.1,"source":"camera_lidar","last_seen_ns":NOW}
    return {"schema_version":1,"stamp_ns":NOW,"image_stamp_ns":NOW,"scan_stamp_ns":NOW,
            "frame_id":"map","objects":objects if objects is not None else [obj],"status":"ok"}


def scene(): return parse_scene(json.dumps(scene_doc()))


def plan(rows): return parse_plan(json.dumps({"steps":rows}),{"person_0001"},{"home"})


@pytest.mark.parametrize("text",[
    '{"steps":[],"steps":[{"action":"stop"}]}',
    '{"steps":[{"action":"wait","seconds":NaN}]}',
    '{"steps":[{"action":"wait","seconds":Infinity}]}',
    '{"steps":[{"action":"stop"}]} trailing',
    '```json\n{"steps":[{"action":"stop"}]}\n```',
])
def test_json_is_strict(text):
    with pytest.raises(PlanRejected): parse_plan(text,{"person_0001"},{"home"})


@pytest.mark.parametrize("rows",[
    [],[{"action":"stop"}]*9,
    [{"action":"goto_object","object_id":"hallucination"}],
    [{"action":"goto_waypoint","name":"unknown"}],
    [{"action":"goto_object","object_id":"person_0001","x":2}],
    [{"action":"goto_waypoint","name":"home","pose":[1,2,0]}],
    [{"action":"shell","command":"anything"}],
    [{"action":"wait","seconds":True}],[{"action":"wait","seconds":11}],
    [{"action":"wait","seconds":-1}],
    [{"action":"stop"},{"action":"goto_waypoint","name":"home"}],
    [{"action":"wait","seconds":10}]*4,
])
def test_plan_rejects_hallucinated_or_unbounded_actions(rows):
    with pytest.raises(PlanRejected): plan(rows)


def test_plan_known_object_and_waypoint_roundtrip():
    rows=[{"action":"goto_object","object_id":"person_0001"},{"action":"wait","seconds":2},
          {"action":"goto_waypoint","name":"home"},{"action":"stop"}]
    assert plan(rows).document()=={"steps":rows}


def test_correlated_command_strict():
    assert parse_command('{"request_id":"trial-1","instruction":"к человеку"}')==("trial-1","к человеку")
    assert parse_command("стоп")==("","стоп")
    with pytest.raises(PlanRejected):parse_command('{"request_id":"x","instruction":"stop","enable_execution":true}')


@pytest.mark.parametrize("change",[
    lambda d:d.update(frame_id="camera"),
    lambda d:d.update(schema_version=True),
    lambda d:d["objects"][0].update(source="llm"),
    lambda d:d["objects"][0].update(score=float("nan")),
    lambda d:d["objects"][0].update(uncertainty_m=-1),
    lambda d:d["objects"][0]["position"].update(x=True),
    lambda d:d["objects"].append(d["objects"][0].copy()),
])
def test_scene_requires_trusted_well_formed_metric_input(change):
    document=scene_doc();change(document)
    with pytest.raises(PlanRejected):parse_scene(json.dumps(document))


def test_grounding_unknown_stale_uncertain_moving_and_ambiguous():
    p=Plan((Step("goto_object",object_id="person_0001"),))
    assert validate_grounding(p,"к человеку",scene(),NOW)
    with pytest.raises(PlanRejected):validate_grounding(p,"к человеку",scene(),NOW+2_000_000_000)
    document=scene_doc();document["objects"][0]["uncertainty_m"]=.31
    with pytest.raises(PlanRejected):validate_grounding(p,"к человеку",parse_scene(json.dumps(document)),NOW)
    document=scene_doc();document["objects"][0]["position"]["x"]=2.4
    with pytest.raises(PlanRejected):validate_grounding(p,"к человеку",parse_scene(json.dumps(document)),NOW,reference=scene())
    document=scene_doc();other=dict(document["objects"][0]);other["id"]="person_0002";document["objects"].append(other)
    current=parse_scene(json.dumps(document))
    with pytest.raises(PlanRejected):validate_grounding(p,"к человеку",current,NOW)
    assert validate_grounding(p,"к person_0001",current,NOW)
    with pytest.raises(PlanRejected):validate_grounding(p,"к person_00010",current,NOW)


def test_slow_inference_uses_refreshed_same_id_but_not_moved_target():
    p=Plan((Step("goto_object",object_id="person_0001"),))
    document=scene_doc();later=NOW+20_000_000_000
    for key in ("stamp_ns","image_stamp_ns","scan_stamp_ns"): document[key]=later
    document["objects"][0]["last_seen_ns"]=later
    assert validate_grounding(p,"к человеку",parse_scene(json.dumps(document)),later,reference=scene())
    document["objects"]=[]
    with pytest.raises(PlanRejected):validate_grounding(p,"к человеку",parse_scene(json.dumps(document)),later,reference=scene())


def test_standoff_has_metric_distance_and_facing():
    obj=scene().lookup("person_0001")
    goal=standoff_goal(0,1,obj,.8)
    assert goal==MetricGoal(1.2,1,0)
    assert math.hypot(goal.x-obj.x,goal.y-obj.y)==pytest.approx(.8)
    assert standoff_goal(1.5,1,obj,.8) is None


def grid(data=None,yaw=0.):
    return OccupancyGrid(10,10,.2,0.,0.,yaw,tuple(data or [0]*100))


def test_goal_filter_rejects_outside_unknown_occupied_and_clearance_edges():
    assert grid().check(MetricGoal(1,1,0),.25)
    for goal in (MetricGoal(-.01,1,0),MetricGoal(.1,1,0),MetricGoal(1.9,1,0)):
        with pytest.raises(PlanRejected):grid().check(goal,.2)
    cells=[0]*100;cells[5*10+5]=-1
    with pytest.raises(PlanRejected):grid(cells).check(MetricGoal(1,1,0),.25)
    cells[5*10+5]=100
    with pytest.raises(PlanRejected):grid(cells).check(MetricGoal(1,1,0),.25)
    # Robot centre is free but its footprint intersects the neighbour.
    cells=[0]*100;cells[5*10+6]=100
    with pytest.raises(PlanRejected):grid(cells).check(MetricGoal(1.1,1.1,0),.25)


def test_rotated_map_origin_and_negative_cell_coordinates():
    assert grid(yaw=math.pi/2).check(MetricGoal(-1,1,0),.2)
    with pytest.raises(PlanRejected):grid(yaw=math.pi/2).check(MetricGoal(.01,1,0),.1)
    with pytest.raises(PlanRejected):OccupancyGrid(1,1,0,0,0,0,(0,))
    with pytest.raises(PlanRejected):OccupancyGrid(2,2,1,0,0,0,(0,))


def test_nav_path_requires_planar_valid_quaternion_and_grounded_endpoints():
    rows=[("map",0.,0.,0.,0.,0.,0.,1.),("map",1.,1.,0.,0.,0.,0.,1.)]
    assert validate_path(rows,"map",MetricGoal(0,0,0),MetricGoal(1,1,0))
    for modified in ([rows[0]],[("map",0,0,.1,0,0,0,1),rows[1]],
                     [("odom",0,0,0,0,0,0,1),rows[1]],
                     [("map",0,0,0,0,0,0,0),rows[1]],
                     [rows[0],("map",10,1,0,0,0,0,1)]):
        with pytest.raises(PlanRejected):validate_path(modified,"map",MetricGoal(0,0,0),MetricGoal(1,1,0))


def path_row(x,y): return ("map",x,y,0.,0.,0.,0.,1.)


def test_grid_path_checks_bends_and_free_segments():
    points=[path_row(.4,.4),path_row(1.,1.),path_row(1.6,.4)]
    assert validate_grid_path(points,grid(),.1)
    cells=[0]*100;cells[5*10+5]=100
    blocked=grid(cells)
    assert blocked.check(MetricGoal(.4,.4,0.),.1)
    assert blocked.check(MetricGoal(1.6,.4,0.),.1)
    with pytest.raises(PlanRejected):validate_grid_path(points,blocked,.1)


@pytest.mark.parametrize("value",[-1,100])
def test_grid_path_checks_sparse_long_segment_interior(value):
    # Both published endpoint poses are free. Sampling must still find the cell
    # between them: a sparse path cannot jump across a wall or unknown space.
    points=[path_row(.4,1.1),path_row(1.6,1.1)]
    cells=[0]*100;cells[5*10+5]=value
    blocked=grid(cells)
    assert blocked.check(MetricGoal(.4,1.1,0.),.1)
    assert blocked.check(MetricGoal(1.6,1.1,0.),.1)
    with pytest.raises(PlanRejected):validate_grid_path(points,blocked,.1)


def test_grid_path_sampling_work_is_bounded():
    with pytest.raises(PlanRejected,match="sampling budget"):
        validate_grid_path([path_row(.4,.4),path_row(1.6,1.6)],grid(),.1,max_samples=2)


class Handle:
    accepted=True
    def __init__(self): self.canceled=0
    def cancel_goal_async(self):self.canceled+=1


@pytest.mark.parametrize("phase",["planning","path_pending","path_running","nav_pending","navigating","waiting"])
def test_stop_cancels_generation_and_late_accepts_in_every_phase(phase):
    gate=GenerationGate();token=gate.begin();future=Future()
    gate.cancel() # STOP is independent of the pending future/phase.
    handle=Handle();future.set_result(handle)
    assert not gate.accept_handle(token,future.result())
    assert handle.canceled==1
    newer=gate.begin()
    assert gate.current(newer) and not gate.current(token)
    assert not gate.accept_handle(token,handle)
    assert gate.current(newer) # Late old result cannot invalidate a new task.
