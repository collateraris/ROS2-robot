"""Jazzy + Gazebo Harmonic. Ground-truth pose is never an operational odometry input."""
import os
from pathlib import Path
import shlex
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    truth = lambda name: arg(name).lower() in ('true','yes','1')
    share = Path(get_package_share_directory('room_sim'))
    nav_share = Path(get_package_share_directory('room_nav'))
    world = Path(os.path.expanduser(arg('world'))).resolve()
    if not world.is_file():
        raise ValueError(f'Gazebo world file is missing: {world}')
    document = ET.parse(world).getroot()
    if document.find("./world[@name='room']") is None:
        raise ValueError('world must contain <world name="room">; use the supplied scenario generator')
    resource = str(share/'models')
    old_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if old_resource:
        resource += os.pathsep + old_resource
    gz_args = '-r -v 3 ' + ('' if truth('gui') else '-s --headless-rendering ') + shlex.quote(str(world))
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown':'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='room_sim_bridge', output='screen',
             parameters=[{'config_file':str(share/'config/bridge.yaml'), 'use_sim_time':True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='room_sim_state_publisher', output='screen',
             parameters=[{'robot_description':(share/'urdf/room_robot.urdf').read_text(), 'use_sim_time':True}]),
        Node(package='room_sim', executable='drive_adapter', name='room_sim_drive', output='screen',
             parameters=[{'use_sim_time':True, 'simulation_autonomy':truth('simulation_autonomy')}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share/'launch/bringup.launch.py')),
            launch_arguments={
                'mode':arg('mode'), 'map':arg('map'), 'use_sim_time':'true',
                'start_base':'true', 'start_bridge':'false', 'start_lidar':'false',
                'start_rf2o':'true', 'publish_laser_tf':'false',
                'start_mission':arg('start_mission'), 'robot_radius':arg('robot_radius'),
                'nav_overrides':arg('nav_overrides'),
                'enable_waypoints':'false', 'enable_external_goals':'false',
            }.items()),
        LogInfo(msg='Simulation: /scan -> RF2O -> odom guard -> /odom. Ground truth only /sim/odom. Explicit ARM required.'),
    ]
    if truth('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='room_sim_rviz',
                            arguments=['-d',str(share/'config/room.rviz')],
                            parameters=[{'use_sim_time':True}], output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('room_sim'))
    nav_share = Path(get_package_share_directory('room_nav'))
    defaults = {
        'world':(str(share/'worlds/room.sdf'),'SDF world path; world name room, model://room_robot'),
        'gui':('true','Gazebo GUI; false uses EGL headless rendering, still requires GPU rendering support'),
        'rviz':('true','Show map, scan, robot frames and camera'),
        'simulation_autonomy':('false','Simulation-only deadman bypass AFTER explicit ARM; false requires operator deadman'),
        'start_mission':('true','Old named-waypoint mission node; disable for room_tasks in weeks 3 and 4'),
        'mode':('mapping','mapping or localization'),
        'map':('','Saved YAML map for localization'),
        'robot_radius':('0.19','Conservative enclosing radius of the simulation robot'),
        'nav_overrides':(str(nav_share/'config/nav2_overrides.yaml'),'Nav2 parameter override YAML'),
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=description)
                              for name,(default,description) in defaults.items()]+[OpaqueFunction(function=_setup)])
