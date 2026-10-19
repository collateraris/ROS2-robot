"""Run the real one-shot client main with bounded fake discovery, without ROS.

These tests check client ordering/cleanup only; they do not validate DDS delivery
or send anything to a robot. Typed conversion still runs through stop_to_ros.
"""
from types import ModuleType, SimpleNamespace
import sys

import pytest

from room_world_model import goal_client


class ClientHarness:
    def __init__(self, *, discover_after=3, interrupt_result=False):
        self.discover_after = discover_after
        self.interrupt_result = interrupt_result
        self.now = 0.
        self.spins = 0
        self.active = True
        self.log = []

    def install(self, monkeypatch):
        outer = self

        class Stop:
            pass

        class Future:
            def __init__(self, kind, value=None):
                self.kind, self.value = kind, value

            def result(self):
                return self.value

        class Handle:
            accepted = True

            def get_result_async(self):
                return Future("result", SimpleNamespace(result=SimpleNamespace(
                    success=True, reason="completed", completed_pulses=1)))

            def cancel_goal_async(self):
                outer.log.append(("request_cancel",))
                return Future("cancel")

        class Publisher:
            def get_subscription_count(self):
                return int(outer.discover_after is not None and outer.spins >= outer.discover_after)

            def publish(self, value):
                outer.log.append(("publish_stop", value.reason, outer.spins, self.get_subscription_count()))

        class Node:
            def __init__(self, name):
                outer.log.append(("node", name))

            def create_publisher(self, msg_type, topic, depth):
                assert msg_type is Stop and topic == "/roomwm/emergency_stop"
                return Publisher()

            def destroy_node(self):
                outer.log.append(("destroy_node",))

        class ActionClient:
            def __init__(self, node, interface, topic):
                assert topic == "/roomwm/approach_person"

            def wait_for_server(self, timeout_sec):
                outer.log.append(("discover_action", timeout_sec))
                return True

            def send_goal_async(self, goal, feedback_callback):
                outer.log.append(("send_goal", goal.standoff_m, goal.avoid_obstacles))
                return Future("goal", Handle())

            def destroy(self):
                outer.log.append(("destroy_client",))

        def spin_once(node, timeout_sec):
            outer.log.append(("spin_once", timeout_sec))
            outer.spins += 1
            outer.now += timeout_sec

        def spin_until_future_complete(node, future, timeout_sec=None):
            outer.log.append(("wait_" + future.kind, timeout_sec))
            if future.kind == "result" and outer.interrupt_result:
                raise KeyboardInterrupt

        def shutdown():
            outer.active = False
            outer.log.append(("shutdown",))

        rclpy = ModuleType("rclpy")
        rclpy.init = lambda **kwargs: outer.log.append(("init", kwargs))
        rclpy.ok = lambda: outer.active
        rclpy.spin_once = spin_once
        rclpy.spin_until_future_complete = spin_until_future_complete
        rclpy.shutdown = shutdown
        action = ModuleType("rclpy.action"); action.ActionClient = ActionClient
        node = ModuleType("rclpy.node"); node.Node = Node
        signals = ModuleType("rclpy.signals"); signals.SignalHandlerOptions = SimpleNamespace(NO="no_signal_handler")
        messages = ModuleType("roomwm_msgs"); messages.msg = SimpleNamespace(Stop=Stop)
        goal_action = ModuleType("roomwm_msgs.action")
        goal_action.ApproachPerson = SimpleNamespace(Goal=SimpleNamespace)
        for name, value in {"rclpy": rclpy, "rclpy.action": action, "rclpy.node": node,
                            "rclpy.signals": signals, "roomwm_msgs": messages,
                            "roomwm_msgs.action": goal_action}.items():
            monkeypatch.setitem(sys.modules, name, value)
        # Replacing this module's time binding avoids altering pytest/global time.
        monkeypatch.setattr(goal_client, "time", SimpleNamespace(monotonic=lambda: outer.now))


def test_stop_waits_for_discovery_before_its_only_publish(monkeypatch, capsys):
    harness = ClientHarness(discover_after=3)
    harness.install(monkeypatch)
    goal_client.main(["стоп"])
    publications = [event for event in harness.log if event[0] == "publish_stop"]
    assert publications == [("publish_stop", "explicit_stop", 3, 1)]
    assert not any(event[0] == "send_goal" for event in harness.log)
    assert "explicit_stop" in capsys.readouterr().out
    assert harness.log[-3:] == [("destroy_client",), ("destroy_node",), ("shutdown",)]


def test_no_subscriber_times_out_without_publish_or_success_report(monkeypatch, capsys):
    harness = ClientHarness(discover_after=None)
    harness.install(monkeypatch)
    with pytest.raises(SystemExit, match="No STOP subscriber discovered"):
        goal_client.main(["стоп"])
    assert 2.0 <= harness.now < 2.1
    assert not any(event[0] in {"publish_stop", "send_goal"} for event in harness.log)
    assert capsys.readouterr().out == ""
    assert harness.log[-3:] == [("destroy_client",), ("destroy_node",), ("shutdown",)]


def test_interrupt_publishes_stop_before_requesting_and_waiting_for_cancel(monkeypatch):
    harness = ClientHarness(discover_after=0, interrupt_result=True)
    harness.install(monkeypatch)
    goal_client.main(["подъедь к человеку"])
    names = [event[0] for event in harness.log]
    assert names.index("wait_result") < names.index("publish_stop") < names.index("request_cancel") < names.index("wait_cancel")
    assert [event for event in harness.log if event[0] == "publish_stop"] == [
        ("publish_stop", "goal_client_interrupted", 0, 1)]
    assert ("wait_cancel", 1.) in harness.log
    assert harness.log[-3:] == [("destroy_client",), ("destroy_node",), ("shutdown",)]
