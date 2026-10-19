"""Optional colcon entry point. Required config path is always explicit."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="Absolute path to project config/robot.yaml"),
        Node(package="room_world_model", executable="roomwm_pi", output="screen",
             arguments=["--config", LaunchConfiguration("config")]),
    ])
