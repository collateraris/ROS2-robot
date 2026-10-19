"""A typed ROS Action client with cancellation on Ctrl+C; never issues ARM."""
from __future__ import annotations
import argparse
import time


def main(argv=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from roomwm_msgs.action import ApproachPerson
    from .ros_interfaces import message_types, stop_to_ros
    from .commands import parse_command
    parser=argparse.ArgumentParser(description="Request approach through a ROS Action; ARM/deadman remain separate")
    parser.add_argument("text",nargs="?",default="подъедь к человеку и обходи препятствия")
    parser.add_argument("--standoff",type=float,default=1.)
    parser.add_argument("--timeout",type=float,default=120.)
    args,ros_args=parser.parse_known_args(argv)
    command=parse_command(args.text,args.standoff)
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=Node("roomwm_goal_client")
    stop=node.create_publisher(message_types().Stop,"/roomwm/emergency_stop",1)
    client=ActionClient(node,ApproachPerson,"/roomwm/approach_person")
    handle=None
    try:
        if not command.motion_requested:
            # A volatile sample published before DDS discovery is not replayed
            # to a later subscriber. Give a one-shot CLI STOP a bounded chance
            # to connect before sending it; do not report an unobserved send.
            discovery_deadline=time.monotonic()+2.0
            while rclpy.ok() and stop.get_subscription_count()==0 and time.monotonic()<discovery_deadline:
                rclpy.spin_once(node,timeout_sec=.05)
            if stop.get_subscription_count()==0:
                raise SystemExit("No STOP subscriber discovered; check the Pi/PC nodes and ROS network")
            stop.publish(stop_to_ros(command.reason))
            rclpy.spin_once(node,timeout_sec=.1)
            print(command.reason)
            return
        if not client.wait_for_server(timeout_sec=5.):
            raise SystemExit("ApproachPerson action server unavailable; start the PC node")
        goal=ApproachPerson.Goal()
        goal.standoff_m=args.standoff; goal.timeout_s=args.timeout; goal.avoid_obstacles=True
        def feedback(message):
            x=message.feedback
            print(f"{x.phase}: person={x.person_visible}, range={x.person_distance_m:.2f}m, confirmed pulses={x.completed_pulses}")
        future=client.send_goal_async(goal,feedback_callback=feedback)
        rclpy.spin_until_future_complete(node,future)
        handle=future.result()
        if not handle.accepted:
            raise SystemExit("Goal rejected: check standoff, timeout, active goal and controller mode")
        print("Goal accepted. Use the operator window for ARM and hold-to-run. Ctrl+C cancels.")
        result=handle.get_result_async()
        rclpy.spin_until_future_complete(node,result)
        value=result.result().result
        print(f"success={value.success}, reason={value.reason}, completed_pulses={value.completed_pulses}")
    except KeyboardInterrupt:
        if rclpy.ok():
            # STOP is independent of the action server's cancellation response.
            stop.publish(stop_to_ros("goal_client_interrupted"))
            if handle is not None and handle.accepted:
                cancellation=handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node,cancellation,timeout_sec=1.)
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        client.destroy(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()
