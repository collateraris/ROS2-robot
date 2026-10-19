"""Actual PCNode lifecycle methods with controlled scheduling, without ROS/DDS.

The nested class is extracted from ros_pc.py's AST so these checks exercise the
production callbacks rather than a copied action state machine. A strict fake
handle rejects invalid terminal transitions; a minimal Future is stepped by the
tests. This does not claim a Jazzy build, executor, DDS or hardware integration.
"""
from __future__ import annotations

import ast
from pathlib import Path
import threading
from types import SimpleNamespace
import uuid
import math
import traceback

import pytest


class FutureFixture:
    def __init__(self):
        self.value = None
        self.finished = False

    def done(self):
        return self.finished

    def set_result(self, value):
        assert not self.finished, "Result completed twice"
        self.value, self.finished = value, True

    def __await__(self):
        while not self.finished:
            yield self
        return self.value


class GoalHandleFixture:
    def __init__(self, key=1):
        self.goal_id = SimpleNamespace(uuid=key.to_bytes(16, "big"))
        self.request = SimpleNamespace(standoff_m=.8, timeout_s=30., avoid_obstacles=True)
        self.state = "accepted"
        self.transitions = []
        self.feedback = []

    @property
    def is_active(self):
        return self.state in {"accepted", "executing", "canceling"}

    @property
    def is_cancel_requested(self):
        return self.state == "canceling"

    def execute(self):
        assert self.is_active
        if not self.is_cancel_requested:
            self.state = "executing"

    def _finish(self, state):
        assert self.is_active, f"Illegal second terminal transition {self.state} -> {state}"
        if state == "canceled":
            assert self.is_cancel_requested
        self.state = state
        self.transitions.append(state)

    def abort(self):
        self._finish("aborted")

    def canceled(self):
        self._finish("canceled")

    def succeed(self):
        self._finish("succeeded")

    def publish_feedback(self, value):
        assert self.is_active
        self.feedback.append(value)


class PublisherFixture:
    def __init__(self, callback=None):
        self.messages, self.callback = [], callback

    def publish(self, message):
        self.messages.append(message)
        if self.callback:
            self.callback(message)


def make_node():
    source = Path(__file__).parents[1] / "room_world_model" / "ros_pc.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    cls = next(node for node in ast.walk(tree) if isinstance(node, ast.ClassDef) and node.name == "PCNode")
    module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    clock = SimpleNamespace(now=100.)
    namespace = {
        "Node": object, "threading": threading, "math": math, "uuid": uuid,
        "traceback": traceback, "args": SimpleNamespace(wm_trial=False),
        "time": SimpleNamespace(monotonic=lambda: clock.now), "Future": FutureFixture,
        "GoalResponse": SimpleNamespace(REJECT="reject", ACCEPT="accept"),
        "CancelResponse": SimpleNamespace(REJECT="reject", ACCEPT="accept"),
        "ApproachPerson": SimpleNamespace(Result=SimpleNamespace, Feedback=SimpleNamespace),
        "mode": "baseline", "model_hash": "", "device": "cpu",
        "c": {"perception": {"person_standoff_m": .8},
              "safety": {"task_timeout_s": 60., "max_inference_s": .3}},
        "task_to_ros": lambda value: SimpleNamespace(**value),
        "task_from_ros": lambda value: vars(value),
        "stop_to_ros": lambda reason, goal_id="": SimpleNamespace(reason=reason, goal_id=goal_id),
    }
    exec(compile(module, str(source), "exec"), namespace)
    node = namespace["PCNode"].__new__(namespace["PCNode"])
    node._condition = threading.Condition()
    node._latest = node._pending_task = node._pi_status = node._last_observation = None
    node._goal_id, node._lease, node._armed, node._status_received = "", "", False, 0.
    node._closed, node._active_action, node._action_reserved = False, None, False
    node._action_records = {}
    node.pub, node.stop_pub, node.task_pub = PublisherFixture(), PublisherFixture(), PublisherFixture()
    node.perception_pub, node.status_pub = PublisherFixture(), PublisherFixture()
    node.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    return node, namespace, clock


def status(goal_id, *, armed=False):
    return SimpleNamespace(schema_version=2, goal_id=goal_id,
        lease_id="valid-lease" if armed else "", armed=armed, reason="operator_disarm" if not armed else "armed")


def accept(node, key=1):
    handle = GoalHandleFixture(key)
    assert node.goal_request(handle.request) == "accept"
    node.goal_accepted(handle)
    return handle


def finish_coroutine(coroutine):
    with pytest.raises(StopIteration) as stopped:
        coroutine.send(None)
    return stopped.value.value


def begin_execute(node, handle):
    coroutine = node.execute_action(handle)
    yielded = coroutine.send(None)
    assert yielded is node._active_action["future"]
    return coroutine


def test_previous_task_status_never_arms_a_new_action_even_before_execute_starts():
    node, _, _ = make_node()
    node.install_task({"goal_id": "previous-manual-task", "kind": "manual", "direction": "forward"})
    node.status(status("previous-manual-task", armed=True))
    assert node._armed
    handle = accept(node)
    # A timer can run after handle.execute() queues the execute callback and
    # before that callback has installed the new task.
    node.action_tick()
    assert not node._active_action["was_armed"]
    coro = begin_execute(node, handle)
    assert not node._armed and node._lease == ""
    node.status(status("previous-manual-task", armed=True))
    node.action_tick()
    assert not node._armed and not node._active_action["was_armed"]
    node.status(status(node._active_action["goal_id"], armed=False))
    node.action_tick()
    assert handle.state == "executing" and node._active_action is not None
    assert handle.feedback[-1].phase == "waiting_for_arm"
    node.finish_action("test_cleanup")
    assert finish_coroutine(coro).reason == "test_cleanup"


@pytest.mark.parametrize("terminal", ["stop", "cancel", "supersede"])
def test_terminal_action_before_first_execute_returns_cached_result_without_second_transition(terminal):
    node, _, _ = make_node()
    handle = accept(node)
    if terminal == "cancel":
        assert node.cancel_request(handle) == "accept"
        handle.state = "canceling"  # rclpy applies CANCEL_GOAL after callback returns
        node.action_tick()
        expected_reason, expected_state = "action_cancelled", "canceled"
    elif terminal == "supersede":
        node.task(SimpleNamespace(goal_id="new-task", kind="text", text="стоп"))
        expected_reason, expected_state = "superseded_by_new_task", "aborted"
    else:
        node.finish_action("operator_stop")
        expected_reason, expected_state = "operator_stop", "aborted"
    assert node._active_action is None and not node._action_reserved
    installed_goal = node._goal_id
    result = finish_coroutine(node.execute_action(handle))
    assert result.reason == expected_reason and result.success is False
    assert handle.transitions == [expected_state]
    assert id(handle) not in node._action_records
    assert node._goal_id == installed_goal  # late execute cannot reinstall cancelled task
    assert not node.task_pub.messages


def test_cancel_sends_stop_before_result_and_waits_for_canceling_state():
    node, _, _ = make_node()
    handle = accept(node)
    coro = begin_execute(node, handle)
    action = node._active_action
    assert node.cancel_request(handle) == "accept"
    sent_stop = node.stop_pub.messages[-1]
    assert sent_stop.reason == "action_cancel_requested" and sent_stop.goal_id == action["goal_id"]
    assert not action["future"].done()
    # Self-delivery of the stop may precede the action server's state update.
    node.emergency_stop(sent_stop)
    assert node._active_action is action and not action["future"].done()
    handle.state = "canceling"
    node.action_tick()
    result = finish_coroutine(coro)
    assert result.reason == "action_cancelled" and handle.transitions == ["canceled"]


def test_completed_ack_count_is_deduplicated_and_bound_to_goal():
    node, _, _ = make_node()
    handle = accept(node)
    coro = begin_execute(node, handle)
    goal_id = node._active_action["goal_id"]
    def ack(goal, seq, confirmed=True):
        return SimpleNamespace(schema_version=2, goal_id=goal, has_interval=True,
            interval=SimpleNamespace(complete=True, actuation_confirmed=confirmed),
            boot_id=7, nonce=11, seq=seq)
    node.ack(ack(goal_id, 1))
    node.ack(ack(goal_id, 1))
    node.ack(ack("old-goal", 2))
    node.ack(ack(goal_id, 2, False))
    node.finish_action("goal_reached_stop", success=True)
    result = finish_coroutine(coro)
    assert result.success and result.completed_pulses == 1
    assert handle.transitions == ["succeeded"]


def test_success_observation_is_committed_before_pi_can_report_disarm():
    node, namespace, clock = make_node()
    handle = accept(node)
    coro = begin_execute(node, handle)
    goal_id = node._active_action["goal_id"]
    node.status(status(goal_id, armed=True))
    node.action_tick()
    assert node._active_action["was_armed"]
    proposal = {"goal_id": goal_id, "abort": True,
        "command": {"inference_ms": 0., "linear_mps": 0., "angular_rps": 0.},
        "diagnostics": {"reason": "goal_reached_stop", "filter_steps": 0,
                        "perception": {"person_visible": True, "person_distance_m": .7,
                                       "person_bearing_rad": .1}}}
    namespace["engine"] = SimpleNamespace(set_task=lambda _: None,
        process=lambda *args: proposal, fingerprint="f"*64)
    namespace["frame_from_ros"] = lambda _: ({"settled": True}, None)
    namespace["proposal_to_ros"] = lambda *args: SimpleNamespace(current_perception=None)
    namespace["planner_status_to_ros"] = lambda *args, **kwargs: SimpleNamespace()
    def immediate_pi_disarm(_):
        node.status(status(goal_id, armed=False))
        node.action_tick()
        node._closed = True
    node.pub.callback = immediate_pi_disarm
    node._latest = (object(), clock.now)
    node.run_worker()
    result = finish_coroutine(coro)
    assert result.success and result.reason == "goal_reached_stop"
    assert handle.transitions == ["succeeded"]


def test_closing_during_inference_discards_the_result_before_publication():
    node, namespace, clock = make_node()
    node.install_task({"goal_id": "text-task", "kind": "text", "text": "подъедь к человеку"})
    node.status(status("text-task", armed=True))
    proposal = {"goal_id": "text-task", "abort": False,
        "command": {"inference_ms": 0., "linear_mps": .05, "angular_rps": 0.},
        "diagnostics": {"reason": "baseline_visual_approach", "filter_steps": 0}}
    def infer(*_):
        node._closed = True  # shutdown's join timeout expires while CUDA was busy
        return proposal
    namespace["engine"] = SimpleNamespace(set_task=lambda _: None, process=infer, fingerprint="f"*64)
    namespace["frame_from_ros"] = lambda _: ({"settled": True}, None)
    namespace["proposal_to_ros"] = lambda *args: SimpleNamespace(current_perception=None)
    namespace["planner_status_to_ros"] = lambda *args, **kwargs: SimpleNamespace()
    node._latest = (object(), clock.now)
    node.run_worker()
    assert not node.pub.messages and not node.perception_pub.messages and not node.status_pub.messages
