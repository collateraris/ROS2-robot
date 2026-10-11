"""USB camera + password-protected browser preview, without motor control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    camera = Node(
        package='robot_web_teleop', executable='camera_node',
        namespace=namespace, output='screen', parameters=[{
            'device': ParameterValue(LaunchConfiguration('camera_device'), value_type=str),
            'pixel_format': ParameterValue(LaunchConfiguration('pixel_format'), value_type=str),
            'width': 640, 'height': 480, 'fps': 30.0,
            'jpeg_quality': 70, 'preview_enabled': True,
        }])
    web = Node(
        package='robot_web_teleop', executable='web_node',
        namespace=namespace, output='screen', parameters=[{
            'view_only': True, 'bind_host': '0.0.0.0',
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
        }])
    nodes = [camera, web]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='A camera preview node exited'))]))
        for node in nodes]
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('pixel_format', default_value='MJPG'),
        DeclareLaunchArgument('port', default_value='8080'),
        *handlers, *nodes,
    ])
