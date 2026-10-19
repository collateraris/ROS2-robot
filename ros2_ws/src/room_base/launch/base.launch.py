"""Guarded encoderless base: RF2O provides raw odometry in room_nav bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('room_base')
    return LaunchDescription([
        DeclareLaunchArgument('base_config', default_value=PathJoinSubstitution([
            share, 'config', 'base.yaml'])),
        DeclareLaunchArgument('odom_guard_config', default_value=PathJoinSubstitution([
            share, 'config', 'odom_guard.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        Node(package='room_base', executable='odom_guard', name='room_odom_guard',
             output='screen', parameters=[LaunchConfiguration('odom_guard_config'),
                 {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='room_base', executable='base_bridge', name='room_base',
             output='screen', condition=IfCondition(LaunchConfiguration('start_bridge')),
             parameters=[LaunchConfiguration('base_config'),
                 {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
