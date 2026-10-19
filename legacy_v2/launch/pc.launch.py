"""Optional PC launch; shadow by default, no automatic arm or operator hold."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="Absolute path to project config/robot.yaml"),
        DeclareLaunchArgument("mode", default_value="shadow",
                              choices=["shadow", "teleop", "baseline", "wm"]),
        DeclareLaunchArgument("shadow_controller", default_value="baseline",
                              choices=["baseline", "teleop", "wm"]),
        Node(package="room_world_model", executable="roomwm_pc", output="screen",
             arguments=["--config", LaunchConfiguration("config"), "--mode", LaunchConfiguration("mode"),
                        "--shadow-controller", LaunchConfiguration("shadow_controller")]),
    ])
