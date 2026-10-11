from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('robot_web_teleop'))
    config = yaml.safe_load((share / 'config' / 'robot.yaml').read_text())
    namespace = LaunchConfiguration('namespace')
    nodes = [
        Node(package='robot_web_teleop', executable='camera_node',
             namespace=namespace, output='screen', parameters=[config['camera'], {
                 'device': ParameterValue(LaunchConfiguration('camera_device'), value_type=str)}]),
        Node(package='robot_web_teleop', executable='motor_bridge',
             namespace=namespace, output='screen', parameters=[config['motor'], {
                 'serial_port': ParameterValue(LaunchConfiguration('serial_port'), value_type=str)}]),
        Node(package='robot_web_teleop', executable='web_node',
             namespace=namespace, output='screen', parameters=[config['web']]),
    ]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='A robot node exited'))]))
        for node in nodes]
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        *handlers, *nodes,
    ])
