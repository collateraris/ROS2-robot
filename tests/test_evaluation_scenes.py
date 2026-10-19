import math
import xml.etree.ElementTree as ET
from room_evaluation.scenes import generate_scene,world_sdf,save_scene
from room_evaluation.ros_runner import map_goal


def test_scene_generation_is_deterministic_and_split_disjoint():
    assert generate_scene('train',2)==generate_scene('train',2)
    scenes=[generate_scene(s,i) for s in ('train','dev','test') for i in range(5)]
    assert len({s['seed'] for s in scenes})==len(scenes)
    assert max(s['room']['width'] for s in scenes if s['split']=='train')<min(
        s['room']['width'] for s in scenes if s['split']=='dev')
    assert max(s['room']['width'] for s in scenes if s['split']=='dev')<min(
        s['room']['width'] for s in scenes if s['split']=='test')


def test_sdf_includes_real_robot_assets_and_world_systems():
    scene=generate_scene();root=ET.fromstring(world_sdf(scene));world=root.find('world')
    assert world.attrib['name']=='room'
    assert world.find('./include/uri').text=='model://room_robot'
    assert len(world.findall('plugin'))==5
    assert 'person_proxy' in [m.attrib['name'] for m in world.findall('model')]
    assert root.find('.//plugin[@name="gz::sim::systems::Contact"]') is not None


def test_gt_person_goal_has_exact_standoff_and_facing():
    scene=generate_scene('test',3);task=scene['tasks'][0];p=task['criteria']['target'];g=task['goal']
    assert math.isclose(math.hypot(g['x']-p['x'],g['y']-p['y']),1.)
    assert math.isclose(g['yaw'],math.atan2(p['y']-g['y'],p['x']-g['x']))


def test_known_waypoints_are_initial_map_coordinates_no_person_gt(tmp_path):
    import yaml
    scene=generate_scene();dest=save_scene(scene,tmp_path)
    public=yaml.safe_load((dest/'known_waypoints.yaml').read_text())
    assert 'person' not in public['waypoints']
    assert public['waypoints']['home']['pose']==[0.,0.,0.]
    task=scene['tasks'][1]
    assert public['waypoints']['table']['pose']==list(map_goal(scene,task['goal']).values())
