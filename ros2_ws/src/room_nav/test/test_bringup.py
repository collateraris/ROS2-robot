"""Offline launch construction checks. No claim of an executing ROS graph."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import yaml


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Configuration:
    def __init__(self, key):
        self.key = key

    def perform(self, context):
        return context[self.key]


def module(name, **attrs):
    value = types.ModuleType(name)
    value.__dict__.update(attrs)
    return value


class BringupTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1]
        self.tmp = tempfile.TemporaryDirectory()
        self.shares = {'room_nav': str(self.root),
                       'room_base': str(self.root.parent / 'room_base')}
        for package, path in [('nav2_bringup', 'params/nav2_params.yaml'),
                              ('slam_toolbox', 'config/mapper_params_online_async.yaml')]:
            share = Path(self.tmp.name) / package
            file = share / path
            file.parent.mkdir(parents=True)
            file.write_text('{}\n')
            self.shares[package] = str(share)
        modules = {
            'ament_index_python': module('ament_index_python'),
            'ament_index_python.packages': module('ament_index_python.packages',
                                                 get_package_share_directory=self.shares.__getitem__),
            'launch': module('launch', LaunchDescription=Action),
            'launch.actions': module('launch.actions', **{key: Action for key in
                ['DeclareLaunchArgument', 'IncludeLaunchDescription', 'LogInfo', 'OpaqueFunction', 'RegisterEventHandler']}),
            'launch.event_handlers': module('launch.event_handlers', OnShutdown=Action),
            'launch.launch_description_sources': module('launch.launch_description_sources', PythonLaunchDescriptionSource=Action),
            'launch.substitutions': module('launch.substitutions', LaunchConfiguration=Configuration),
            'launch_ros': module('launch_ros'),
            'launch_ros.actions': module('launch_ros.actions', Node=Action),
        }
        self.mocker = patch.dict(sys.modules, modules)
        self.mocker.start()
        spec = importlib.util.spec_from_file_location('room_bringup_offline', self.root / 'launch/bringup.launch.py')
        self.launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.launch)
        actions = self.launch.generate_launch_description().args[0]
        self.defaults = {a.args[0]: a.kwargs['default_value'] for a in actions if a.args}
        self.created_paths = []

    def tearDown(self):
        for path in self.created_paths:
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass
        self.mocker.stop()
        self.tmp.cleanup()

    def construct(self, **overrides):
        context = {**self.defaults, **overrides}
        actions = self.launch._setup(context)
        for action in actions:
            for parameter in action.kwargs.get('parameters', []):
                if isinstance(parameter, str) and os.path.basename(parameter).startswith('room_nav_'):
                    self.created_paths.append(parameter)
            args = action.kwargs.get('launch_arguments', [])
            for key, value in args:
                if key == 'slam_params_file':
                    self.created_paths.append(value)
        return actions

    def test_default_encoderless_graph_has_single_odom_tf_owner(self):
        actions = self.construct()
        nodes = {a.kwargs['name']: a for a in actions if 'name' in a.kwargs}
        rf2o = nodes['rf2o_laser_odometry'].kwargs['parameters'][0]
        self.assertEqual(rf2o['odom_topic'], '/odom_rf2o')
        self.assertFalse(rf2o['publish_tf'])
        self.assertEqual(rf2o['init_pose_from_topic'], '')
        self.assertEqual(rf2o['laser_scan_topic'], '/scan')
        driver = nodes['sllidar_node'].kwargs['parameters'][0]
        self.assertEqual(driver['serial_baudrate'], 460800)
        self.assertEqual(driver['scan_mode'], 'Standard')
        self.assertEqual(driver['frame_id'], 'laser')
        self.assertIn('base_to_laser', nodes)
        base = [a for a in actions if a.args and getattr(a.args[0], 'args', ()) and
                str(a.args[0].args[0]).endswith('base.launch.py')]
        self.assertEqual(len(base), 1)
        include_args = dict(base[0].kwargs['launch_arguments'])
        self.assertEqual(include_args['start_bridge'], 'true')
        self.assertIn('odom_guard_config', include_args)
        params = yaml.safe_load(Path(nodes['controller_server'].kwargs['parameters'][0]).read_text())
        self.assertEqual(params['velocity_smoother']['ros__parameters']['feedback'], 'OPEN_LOOP')
        self.assertEqual(params['bt_navigator']['ros__parameters']['odom_topic'], '/odom')
        self.assertEqual(params['collision_monitor']['ros__parameters']['cmd_vel_out_topic'], '/cmd_vel_safe')

    def test_push_mapping_keeps_guard_but_disables_bridge(self):
        actions = self.construct(start_bridge='false', lidar_port='/dev/serial/by-id/c1', use_sim_time='true')
        nodes = {a.kwargs['name']: a for a in actions if 'name' in a.kwargs}
        self.assertEqual(nodes['sllidar_node'].kwargs['parameters'][0]['serial_port'], '/dev/serial/by-id/c1')
        self.assertTrue(nodes['rf2o_laser_odometry'].kwargs['parameters'][0]['use_sim_time'])
        includes = [dict(a.kwargs['launch_arguments']) for a in actions if 'launch_arguments' in a.kwargs]
        self.assertEqual(next(p for p in includes if 'base_config' in p)['start_bridge'], 'false')

    def test_external_sensors_no_duplicate_nodes(self):
        actions = self.construct(start_base='false', start_lidar='false', start_rf2o='false', publish_laser_tf='false')
        names = {a.kwargs.get('name') for a in actions}
        self.assertNotIn('rf2o_laser_odometry', names)
        self.assertNotIn('sllidar_node', names)
        self.assertNotIn('base_to_laser', names)

    def test_invalid_c1_baud_and_missing_localization_map_rejected(self):
        with self.assertRaises(ValueError):
            self.construct(lidar_baudrate='115200')
        with self.assertRaises(ValueError):
            self.construct(mode='localization', map='')


if __name__ == '__main__':
    unittest.main()
