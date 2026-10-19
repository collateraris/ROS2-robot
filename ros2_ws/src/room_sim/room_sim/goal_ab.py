"""Two sequential Nav2 goals in map coordinates; contains no simulator truth input."""
import argparse
import math
import time
import signal
from .action_lifecycle import OwnedGoals
from .simulation_goal import SimulationMonitor, run_simulation_goal


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', type=float, nargs=3, action='append', metavar=('X','Y','YAW'),
                        help='repeat twice; coordinates must be clear measured map positions')
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--arm-simulation', action='store_true',
                        help='explicitly arm only after simulation marker and output mode are verified')
    options, ros_args = parser.parse_known_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    goals = options.goal or [[0.,0.,0.],[1.4,0.,0.]]
    if len(goals) != 2 or not all(math.isfinite(v) for goal in goals for v in goal):
        parser.error('provide two finite --goal X Y YAW values')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav2_msgs.action import NavigateToPose
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import Bool, String
    from geometry_msgs.msg import Twist
    from action_msgs.msg import GoalStatus
    # Keep ROS alive during Ctrl-C cleanup so cancellation and STOP can be sent.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = Node('room_sim_goal_ab', parameter_overrides=[Parameter('use_sim_time',value=True)])
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    owned = OwnedGoals()
    monitor = SimulationMonitor()
    node.create_subscription(Bool, '/room_sim/is_simulation',
        lambda msg: setattr(monitor, 'marker_at', time.monotonic() if msg.data else -math.inf), 1)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/room_sim/output_mode', lambda msg: setattr(monitor, 'output_mode', msg.data), qos)
    def on_health(msg):
        monitor.health, monitor.health_at = bool(msg.data), time.monotonic()
    def on_armed(msg):
        monitor.armed, monitor.armed_at = bool(msg.data), time.monotonic()
    def on_safe_command(msg):
        unused = (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)
        supported = all(math.isfinite(v) and abs(v)<1e-9 for v in unused)
        monitor.safe_command(msg.linear.x, msg.angular.z, time.monotonic(), supported)
    node.create_subscription(Bool, '/room_base/odom_healthy', on_health, 1)
    node.create_subscription(Bool, '/room_base/armed', on_armed, qos)
    node.create_subscription(Twist, '/cmd_vel_safe', on_safe_command, 1)


    def wait(future, timeout):
        end = time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.1)
        if not future.done():
            raise TimeoutError('ROS operation timed out')
        return future.result()

    stop = node.create_client(Trigger, '/room_base/stop')
    emergency = node.create_publisher(Bool, '/room_base/estop', qos)
    arm=node.create_client(Trigger,'/room_base/arm')
    parameters=node.create_client(GetParameters,'/room_sim_drive/get_parameters')

    def validate_simulation():
        if monitor.output_mode!='simulation' or not 0. <= time.monotonic()-monitor.marker_at <= .20:
            raise RuntimeError('no fresh simulator-only marker; refusing automatic ARM')
        providers = []
        for name, namespace in node.get_node_names_and_namespaces():
            services = node.get_service_names_and_types_by_node(name, namespace)
            if any(service == '/room_base/arm' for service, _types in services):
                providers.append((name, namespace))
        if providers != [('room_sim_drive', '/')]:
            raise RuntimeError(f'ARM provider is not the isolated simulator adapter: {providers}')
        if not parameters.wait_for_service(timeout_sec=3.):
            raise RuntimeError('simulation adapter parameter service unavailable')
        request=GetParameters.Request()
        request.names=['use_sim_time','simulation_autonomy']
        verified=wait(parameters.call_async(request),3.)
        if len(verified.values)!=2 or not all(value.type==1 and value.bool_value for value in verified.values):
            raise RuntimeError('automatic ARM requires simulator use_sim_time and simulation_autonomy true')
        if not arm.wait_for_service(timeout_sec=3.):
            raise RuntimeError('simulation ARM service unavailable')

    try:
        if not action.wait_for_server(timeout_sec=30.):
            raise RuntimeError('Nav2 action server not ready')
        if options.arm_simulation:
            deadline=time.monotonic()+3.
            while time.monotonic()<deadline and (monitor.output_mode!='simulation' or time.monotonic()-monitor.marker_at>.20):
                rclpy.spin_once(node,timeout_sec=.05)
        for label, (x,y,yaw) in zip(('A','B'),goals):
            if options.arm_simulation:
                # One explicit simulation session per new owned goal. Any previous
                # goal is already terminal; disarm while the next path is planned.
                validate_simulation()
                if not stop.wait_for_service(timeout_sec=3.):
                    raise RuntimeError('simulation STOP service unavailable')
                response=wait(stop.call_async(Trigger.Request()),3.)
                if not response.success:
                    raise RuntimeError('simulation STOP rejected: '+response.message)
            goal=NavigateToPose.Goal()
            goal.pose.header.frame_id='map'
            goal.pose.header.stamp=node.get_clock().now().to_msg()
            goal.pose.pose.position.x=x
            goal.pose.pose.position.y=y
            goal.pose.pose.orientation.z=math.sin(yaw/2)
            goal.pose.pose.orientation.w=math.cos(yaw/2)
            request=owned.submit(lambda: action.send_goal_async(goal))
            wait(request.acceptance,5.)
            owned.poll()
            handle=request.handle
            if handle is None or not handle.accepted:
                raise RuntimeError(f'goal {label} rejected')
            if options.arm_simulation:
                result=run_simulation_goal(request, monitor,
                    validate_simulation=validate_simulation,
                    arm=lambda: arm.call_async(Trigger.Request()),
                    spin_once=lambda timeout: rclpy.spin_once(node,timeout_sec=timeout),
                    running=rclpy.ok, timeout=options.timeout)
            else:
                result=wait(request.result,options.timeout)
            owned.poll()
            if result.status!=GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'goal {label} failed, status {result.status}')
            node.get_logger().info(f'Goal {label} reached at map ({x:.2f},{y:.2f})')
    finally:
        # Generation invalidation happens before any further callback can dispatch work.
        # Late-accepted goals are canceled by the same owned-goal callback during draining.
        owned.invalidate()
        # Disarm promptly even if action cancellation takes the full drain budget.
        # Queue this without blocking; the following drain also services its DDS reply.
        if stop.service_is_ready():
            try:
                stop.call_async(Trigger.Request())
            except Exception:
                pass
        settled = False
        try:
            settled = owned.cancel_and_drain(
                lambda timeout: rclpy.spin_once(node, timeout_sec=timeout),
                timeout=2., running=rclpy.ok)
        except (Exception, KeyboardInterrupt) as exc:
            node.get_logger().error(f'Action cleanup interrupted: {exc}')
        if not settled:
            # An unknown transport outcome cannot be made safe by a simple re-ARM.
            # Latch the adapter until Nav2 has been cleared and the operator resets it.
            node.get_logger().error('Owned Nav2 goal cancellation is unconfirmed; emergency stop latched. '
                                    'Clear/restart Nav2 before resetting the latch.')
            try:
                emergency.publish(Bool(data=True))
            except Exception:
                pass
        if stop.service_is_ready():
            try:
                wait(stop.call_async(Trigger.Request()),2.)
            except (Exception, KeyboardInterrupt):
                pass
        # Give the transient stop and service request a bounded opportunity to leave DDS.
        if rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=.05)
            except (Exception, KeyboardInterrupt):
                pass
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
