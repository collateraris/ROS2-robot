"""Exercise actual async callbacks offline; ROS transport is explicitly stubbed.

These test cancellation/default gating, not ROS action wire compatibility.
"""
from concurrent.futures import Future
import importlib.util
from pathlib import Path
import sys
import types
from types import SimpleNamespace as NS
import pytest

from room_planning.core import GenerationGate,MetricGoal,Plan,PlanRejected,Scene,Step


@pytest.fixture
def module(monkeypatch):
    def message(**kwargs): return NS(**kwargs)
    definitions={
        "rclpy":{},"rclpy.action":{"ActionClient":object},
        "rclpy.clock":{"Clock":object,"ClockType":NS(STEADY_TIME=1)},
        "rclpy.node":{"Node":object},
        "rclpy.qos":{"QoSProfile":object,"ReliabilityPolicy":object,"DurabilityPolicy":object,"qos_profile_sensor_data":object()},
        "geometry_msgs.msg":{"PoseStamped":object},"nav_msgs.msg":{"OccupancyGrid":object},
        "nav2_msgs.action":{"ComputePathToPose":object,"NavigateToPose":object},
        "sensor_msgs.msg":{"Image":object},"std_msgs.msg":{"Bool":message,"String":message},
        "tf2_ros":{"Buffer":object,"TransformListener":object},
    }
    for name,values in definitions.items():
        item=types.ModuleType(name);item.__dict__.update(values);monkeypatch.setitem(sys.modules,name,item)
    name="_tested_room_tasks_coordinator"
    path=Path(__file__).parents[1]/"ros2_ws/src/room_tasks/room_tasks/coordinator.py"
    spec=importlib.util.spec_from_file_location(name,path);item=importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules,name,item);spec.loader.exec_module(item)
    return item


def node(module):
    obj=object.__new__(module.TaskCoordinator)
    obj.p={"shadow_only":True,"enable_execution":False,"planning_timeout_s":30.,"max_scene_age_s":1.,
           "max_uncertainty_m":.3,"minimum_score":.5,"max_target_movement_m":.3,"backend":"rules"}
    obj.gate=GenerationGate();obj.request_id="test";obj.instruction="home";obj.plan=None;obj.reference=None
    obj.goal=None;obj.anchor=None;obj.phase="idle";obj.step_index=0;obj.deadline=float("inf")
    obj.jobs=[];obj.scene=None;obj.action_handle=None;obj.path_handle=None;obj.logs=[];obj.contexts=[];obj.halts=0
    obj.now_ns=lambda:10_000_000_000
    obj.event=lambda event,status,details=None,**kw:obj.logs.append((event,status,details or {},kw))
    obj.context=lambda instruction,target:obj.contexts.append((instruction,target))
    obj.halt=lambda:setattr(obj,"halts",obj.halts+1)
    obj.plans=NS(publish=lambda message:None)
    return obj


def test_default_shadow_never_dispatches_nav_actions(module):
    obj=node(module);token=obj.gate.begin();obj.phase="planning"
    future=Future();future.set_result(Plan((Step("goto_waypoint",name="home"),)))
    obj.jobs=[module.InferenceJob(future,token,"test","home",Scene(obj.now_ns(),obj.now_ns(),obj.now_ns(),()),
                                 module.time.monotonic(),0)]
    called=[];obj.dispatch=lambda:called.append(True)
    obj.tick()
    assert called==[] and obj.halts==0
    assert any(event=="task_result" and status=="shadow" for event,status,*_ in obj.logs)
    assert not obj.gate.active


@pytest.mark.parametrize("shadow,enabled",[(True,False),(True,True),(False,False)])
def test_both_execution_switches_must_allow_actions(module,shadow,enabled):
    obj=node(module);obj.p.update(shadow_only=shadow,enable_execution=enabled)
    assert obj.shadow


def test_stop_while_model_future_is_blocked_is_immediate_and_discards_result(module):
    obj=node(module);token=obj.gate.begin();obj.phase="planning"
    future=Future();snapshot=Scene(obj.now_ns(),obj.now_ns(),obj.now_ns(),())
    obj.jobs=[module.InferenceJob(future,token,"test","home",snapshot,module.time.monotonic(),0)]
    obj.command(NS(data="стоп"))
    assert obj.halts==1 and not obj.gate.active and not future.done()
    future.set_result(Plan((Step("goto_waypoint",name="home"),)))
    obj.dispatch=lambda:pytest.fail("obsolete inference resumed movement")
    obj.tick()
    assert any(event=="plan" and status=="discarded" for event,status,*_ in obj.logs)


class Handle:
    accepted=True
    def __init__(self):self.canceled=0
    def cancel_goal_async(self):self.canceled+=1


@pytest.mark.parametrize("callback",["path_accepted","nav_accepted"])
def test_late_ros_action_accept_is_canceled_after_stop(module,callback):
    obj=node(module);token=obj.gate.begin();obj.gate.cancel()
    handle=Handle();future=Future();future.set_result(handle)
    getattr(obj,callback)(future,token,0)
    assert handle.canceled==1 and obj.action_handle is None and obj.path_handle is None


def test_stale_target_between_path_success_and_nav_dispatch_cancels_without_navigation(module):
    obj=node(module);obj.p.update(shadow_only=False,enable_execution=True)
    token=obj.gate.begin();obj.phase="path_running";handle=Handle();obj.path_handle=handle
    obj.plan=Plan((Step("goto_object",object_id="person_0001"),));obj.goal=MetricGoal(1,1,0)
    obj.movement_ready=lambda:MetricGoal(0,0,0)
    def lost(step): raise PlanRejected("target lost")
    obj.check_object=lost
    sent=[];obj.nav_client=NS(send_goal_async=lambda req:sent.append(req))
    future=Future();future.set_result(NS(status=4,result=NS(error_code=0,path=NS(poses=[1,2],header=NS(frame_id="map")))))
    obj.path_result(future,token,0,handle)
    assert sent==[] and obj.halts==1 and not obj.gate.active
    assert any(event=="task_result" and status=="rejected" for event,status,*_ in obj.logs)


def test_odom_health_loss_cancels_and_health_recovery_does_not_resume(module):
    obj=node(module);obj.p.update(shadow_only=False,enable_execution=True)
    obj.gate.begin();obj.phase="navigating"
    obj.health_cb(NS(data=False))
    assert obj.halts==1 and not obj.gate.active
    obj.dispatch=lambda:pytest.fail("health recovery automatically resumed movement")
    obj.health_cb(NS(data=True))
    obj.tick()
    assert obj.halts==1 and not obj.gate.active
