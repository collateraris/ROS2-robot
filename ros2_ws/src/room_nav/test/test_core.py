from copy import deepcopy
from pathlib import Path
import unittest
import yaml
from room_nav.core import deep_merge, parse_command, validate_waypoints


def points():
    return {'waypoints': {'home': {'pose': [0., 0., 0.], 'aliases': ['домой']},
                          'desk': {'pose': [1., 2., 0.], 'aliases': ['столу']}}}


class CoreTests(unittest.TestCase):
    def test_commands_exact_and_stop(self):
        wp = validate_waypoints(points())
        self.assertEqual(parse_command('Подъедь к столу!', wp), ('waypoint', 'desk'))
        self.assertEqual(parse_command('домой', wp), ('waypoint', 'home'))
        self.assertEqual(parse_command('СТОП!', wp), ('stop', None))
        for text in ('не стоп', 'к человеку', 'приезжай к столу или к окну'):
            self.assertEqual(parse_command(text, wp), ('unknown', None))

    def test_no_duplicate_alias_or_nan(self):
        data = points()
        data['waypoints']['desk']['aliases'] = ['домой']
        with self.assertRaises(ValueError):
            validate_waypoints(data)
        data = points()
        data['waypoints']['desk']['pose'][0] = float('nan')
        with self.assertRaises(ValueError):
            validate_waypoints(data)

    def test_merge_preserves_defaults_but_replaces_arrays(self):
        base = {'a': {'x': 1, 'y': 2}, 'plugins': ['old']}
        expected = deepcopy(base)
        result = deep_merge(base, {'a': {'y': 3}, 'plugins': ['new']})
        self.assertEqual(result, {'a': {'x': 1, 'y': 3}, 'plugins': ['new']})
        self.assertEqual(base, expected)

    def test_collision_monitor_chain_contract(self):
        path = Path(__file__).resolve().parents[1] / 'config' / 'nav2_overrides.yaml'
        doc = yaml.safe_load(path.read_text())
        cm = doc['collision_monitor']['ros__parameters']
        self.assertEqual(cm['cmd_vel_in_topic'], '/cmd_vel_smoothed')
        self.assertEqual(cm['cmd_vel_out_topic'], '/cmd_vel_safe')
        self.assertLessEqual(cm['source_timeout'], 0.3)
        for node in ('controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor'):
            self.assertIs(doc[node]['ros__parameters']['enable_stamped_cmd_vel'], False)
        self.assertIn('FootprintApproach', cm['polygons'])
        for name in ('local_costmap', 'global_costmap'):
            source = doc[name][name]['ros__parameters']['obstacle_layer']['scan']
            self.assertTrue(source['clearing'])
            self.assertTrue(source['marking'])


if __name__ == '__main__':
    unittest.main()
