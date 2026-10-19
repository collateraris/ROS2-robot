"""Choose the ML venv Python explicitly; ROS console-script shebang may be /usr/bin/python3."""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config=os.path.join(get_package_share_directory("room_perception"),"config","perception.yaml")
    calibration=os.path.join(get_package_share_directory("room_perception"),"config","calibration.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("python_executable",default_value=sys.executable),
        DeclareLaunchArgument("params_file",default_value=config),
        DeclareLaunchArgument("calibration_file",default_value=calibration),
        DeclareLaunchArgument("use_sim_time",default_value="false"),
        Node(package="room_perception",executable="perception",name="room_perception",output="screen",
             prefix=[LaunchConfiguration("python_executable")],
             parameters=[LaunchConfiguration("params_file"),{"calibration_file":LaunchConfiguration("calibration_file"),"use_sim_time":LaunchConfiguration("use_sim_time")}])])
