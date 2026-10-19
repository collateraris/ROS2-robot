"""Encoderless C1/RF2O -> validated odometry, Jazzy SLAM/Nav2 and one velocity chain."""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from room_nav.core import deep_merge


def _read(path):
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _apply_clock(value, use_sim_time):
    if isinstance(value, dict):
        for key, child in value.items():
            if key == 'ros__parameters':
                child['use_sim_time'] = use_sim_time
            else:
                _apply_clock(child, use_sim_time)


def _setup(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    truth = lambda key: arg(key).lower() in ('true', '1', 'yes')
    share = get_package_share_directory('room_nav')
    mode = arg('mode')
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    radius = float(arg('robot_radius'))
    if not 0.05 <= radius <= 2.0:
        raise ValueError('robot_radius must be a measured enclosing radius in [0.05, 2] m')
    clock = truth('use_sim_time')
    stock = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')
    params = deep_merge(_read(stock), _read(arg('nav_overrides')))
    for costmap in ('local_costmap', 'global_costmap'):
        p = params[costmap][costmap]['ros__parameters']
        p['robot_radius'] = radius
        p['inflation_layer']['inflation_radius'] = max(0.45, radius + 0.25)
    cm = params['collision_monitor']['ros__parameters']
    cm['StopZone']['radius'] = radius + 0.15
    cm['SlowZone']['radius'] = radius + 0.30
    cm['range']['enabled'] = truth('enable_range')
    _apply_clock(params, clock)
    temp_paths = []

    def temporary_yaml(document):
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', prefix='room_nav_', suffix='.yaml', delete=False) as out:
            yaml.safe_dump(document, out, sort_keys=False)
            temp_paths.append(out.name)
            return out.name

    nav_file = temporary_yaml(params)
    nodes = []
    if truth('start_base'):
        base_share = get_package_share_directory('room_base')
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'base.launch.py')),
            launch_arguments={'base_config': arg('base_config'),
                              'odom_guard_config': arg('odom_guard_config'),
                              'start_bridge': arg('start_bridge'),
                              'use_sim_time': str(clock).lower()}.items()))
    if truth('start_lidar'):
        lidar_params = _read(arg('lidar_config'))['sllidar_node']['ros__parameters']
        lidar_params.update({'serial_port': arg('lidar_port'),
                             'serial_baudrate': int(arg('lidar_baudrate')),
                             'frame_id': arg('laser_frame'), 'use_sim_time': clock})
        if lidar_params['serial_baudrate'] != 460800:
            raise ValueError('RPLIDAR C1 serial_baudrate must be 460800')
        nodes.append(Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
                          parameters=[lidar_params], remappings=[('scan', '/scan')], output='screen'))
    if truth('start_rf2o'):
        rf2o_params = _read(arg('rf2o_config'))['rf2o_laser_odometry']['ros__parameters']
        rf2o_params.update({'use_sim_time': clock, 'publish_tf': False, 'laser_scan_topic': '/scan',
                           'odom_topic': '/odom_rf2o', 'base_frame_id': 'base_link',
                           'odom_frame_id': 'odom', 'init_pose_from_topic': ''})
        nodes.append(Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
                          name='rf2o_laser_odometry', parameters=[rf2o_params], output='screen'))
    # No Nav2 node in this launch publishes to base hardware. Only CM may publish cmd_vel_safe.
    node_specs = [
        ('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', '/cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', 'smoother_server', []),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', '/cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', '/cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel_smoothed')]),
        ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', []),
    ]
    for package, executable, name, remaps in node_specs:
        nodes.append(Node(package=package, executable=executable, name=name,
                          parameters=[nav_file], remappings=remaps, output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_navigation', output='screen',
                      parameters=[{'use_sim_time': clock, 'autostart': True,
                                   'node_names': [spec[2] for spec in node_specs],
                                   'bond_timeout': 2.0}]))
    if mode == 'mapping':
        slam_share = get_package_share_directory('slam_toolbox')
        stock_slam = os.path.join(slam_share, 'config', 'mapper_params_online_async.yaml')
        slam = deep_merge(_read(stock_slam), _read(os.path.join(share, 'config', 'slam_overrides.yaml')))
        _apply_clock(slam, clock)
        slam_file = temporary_yaml(slam)
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': str(clock).lower(), 'slam_params_file': slam_file,
                              'autostart': 'true', 'use_lifecycle_manager': 'false'}.items()))
    else:
        map_file = os.path.abspath(os.path.expanduser(arg('map')))
        if not arg('map') or not os.path.isfile(map_file):
            raise ValueError('localization requires map:=/absolute/path/room.yaml from map_saver_cli')
        nodes.extend([
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=[nav_file, {'yaml_filename': map_file, 'use_sim_time': clock}]),
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[nav_file]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'autostart': True, 'use_sim_time': clock,
                              'node_names': ['map_server', 'amcl'], 'bond_timeout': 2.0}])])
    if truth('publish_laser_tf'):
        # Must match the scan header.frame_id. The values are measured extrinsics, metres/radians.
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='base_to_laser', output='screen',
            arguments=['--x', arg('laser_x'), '--y', arg('laser_y'), '--z', arg('laser_z'),
                       '--roll', arg('laser_roll'), '--pitch', arg('laser_pitch'), '--yaw', arg('laser_yaw'),
                       '--frame-id', 'base_link', '--child-frame-id', arg('laser_frame')],
            parameters=[{'use_sim_time': clock}]))
    if truth('start_mission'):
        nodes.append(Node(package='room_nav', executable='mission', name='room_mission', output='screen',
            parameters=[{'use_sim_time': clock, 'waypoints_file': arg('waypoints'),
                         'enable_waypoints': truth('enable_waypoints'),
                         'enable_external_goals': truth('enable_external_goals')}]))

    def cleanup(_context):
        for path in temp_paths:
            try:
                os.unlink(path)
            except OSError:
                pass
        return []
    nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    return [LogInfo(msg='room_nav: C1 /scan -> RF2O /odom_rf2o -> guarded /odom + TF. Nav2 -> smoother -> collision monitor -> /cmd_vel_safe. Drive starts disarmed.')]+nodes


def generate_launch_description():
    share = get_package_share_directory('room_nav')
    base_share = get_package_share_directory('room_base')
    defaults = {
        'start_mission': ('true', 'Start legacy named-waypoint mission; false with room_tasks coordinator'),
        'start_base': ('true', 'Include drive + validated RF2O odometry; false only when externally launched'),
        'start_bridge': ('true', 'Start Arduino UART drive bridge; false permits manual pushing with guard active'),
        'start_lidar': ('true', 'Start the RPLIDAR C1 driver; false for an externally supplied /scan'),
        'start_rf2o': ('true', 'Start patched RF2O; false for an externally supplied raw /odom_rf2o'),
        'base_config': (os.path.join(base_share, 'config', 'base.yaml'), 'Encoderless calibrated PWM bridge YAML'),
        'odom_guard_config': (os.path.join(base_share, 'config', 'odom_guard.yaml'), 'RF2O validity guard YAML'),
        'lidar_config': (os.path.join(share, 'config', 'rplidar_c1.yaml'), 'RPLIDAR C1 parameters'),
        'rf2o_config': (os.path.join(share, 'config', 'rf2o.yaml'), 'Patched RF2O parameters; output/TF ownership fixed by launch'),
        'lidar_port': ('/dev/ttyUSB0', 'C1 serial port; prefer unique /dev/serial/by-id path'),
        'lidar_baudrate': ('460800', 'RPLIDAR C1 required serial baudrate'),
        'mode': ('mapping', 'mapping (slam_toolbox) or localization (saved map + AMCL)'),
        'map': ('', 'Absolute map YAML path; required in localization'),
        'use_sim_time': ('false', 'Use /clock only for simulation/bag replay'),
        'nav_overrides': (os.path.join(share, 'config', 'nav2_overrides.yaml'), 'Small Nav2 override YAML'),
        'robot_radius': ('0.18', 'EXAMPLE enclosing radius metres; measure your entire robot before arming'),
        'enable_range': ('false', 'Enable an additional calibrated sensor_msgs/Range source on /range; stale source stops motion'),
        'publish_laser_tf': ('true', 'Publish measured base_link -> laser extrinsics only if driver does not'),
        'laser_frame': ('laser', 'Must exactly equal LaserScan.header.frame_id'),
        'laser_x': ('0.0', 'Measured laser x metres'),
        'laser_y': ('0.0', 'Measured laser y metres'),
        'laser_z': ('0.0', 'Measured laser z metres'),
        'laser_roll': ('0.0', 'Measured laser roll radians'),
        'laser_pitch': ('0.0', 'Measured laser pitch radians'),
        'laser_yaw': ('0.0', 'Measured laser yaw radians'),
        'waypoints': (os.path.join(share, 'config', 'waypoints.yaml'), 'Custom saved-map waypoint YAML'),
        'enable_waypoints': ('false', 'Set true only after replacing example waypoints with your own'),
        'enable_external_goals': ('false', 'Allow fresh metric map PoseStamped on /room_nav/goal'),
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in defaults.items()] + [OpaqueFunction(function=_setup)])
