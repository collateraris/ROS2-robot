"""Offline contract checks: source construction and assets, never a running Gazebo claim."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml


class Action:
    def __init__(self,*args,**kwargs):
        self.args,self.kwargs=args,kwargs


class Config:
    def __init__(self,key): self.key=key
    def perform(self,context): return context[self.key]


def module(name,**attrs):
    result=types.ModuleType(name)
    result.__dict__.update(attrs)
    return result


ROOT=Path(__file__).resolve().parents[1]


class AssetsTests(unittest.TestCase):
    def test_sensor_extrinsics_match_urdf_and_native_model(self):
        sdf=ET.parse(ROOT/'models/room_robot/model.sdf').getroot().find('model')
        urdf=ET.parse(ROOT/'urdf/room_robot.urdf').getroot()
        for link,joint in [('laser','laser_fixed_joint'),('camera_link','camera_fixed_joint')]:
            origin=list(map(float,urdf.find(f"joint[@name='{joint}']/origin").attrib['xyz'].split()))
            pose=list(map(float,sdf.find(f"link[@name='{link}']/pose").text.split()))
            self.assertEqual(origin,pose[:3])
        lidar=sdf.find(".//sensor[@type='gpu_lidar']")
        self.assertEqual(lidar.findtext('gz_frame_id'),'laser')
        self.assertEqual(float(lidar.findtext('update_rate')),10.)
        scan=lidar.find('lidar/scan/horizontal')
        self.assertAlmostEqual(float(scan.findtext('min_angle')),-float(scan.findtext('max_angle')))
        camera=sdf.find(".//sensor[@type='camera']")
        self.assertEqual(camera.findtext('gz_frame_id'),'camera_optical_frame')
        self.assertEqual(camera.findtext('camera/camera_info_topic'),'/camera/camera_info')
        self.assertEqual(camera.findtext('camera/image/width'),'640')
        self.assertEqual(camera.findtext('camera/image/height'),'480')
        export=ET.parse(ROOT/'package.xml').getroot().find('export/gazebo_ros')
        self.assertEqual(export.attrib['gazebo_model_path'],'${prefix}/models')

    def test_bridge_does_not_feed_ground_truth_or_wheel_odometry_to_navigation(self):
        bridge=yaml.safe_load((ROOT/'config/bridge.yaml').read_text())
        channels={entry['ros_topic_name']:entry for entry in bridge}
        self.assertNotIn('/odom',channels)
        self.assertNotIn('/tf',channels)
        self.assertNotIn('/sim/wheel_odom',channels)
        self.assertEqual(channels['/sim/odom']['gz_topic_name'],'/sim/odom')
        self.assertEqual(channels['/sim/contacts']['ros_type_name'],'ros_gz_interfaces/msg/Contacts')
        self.assertEqual(channels['/sim/cmd_vel']['direction'],'ROS_TO_GZ')
        for topic in ('/scan','/camera/image_raw','/camera/camera_info'):
            self.assertEqual(channels[topic]['direction'],'GZ_TO_ROS')

    def test_world_robot_once_and_contact_bumper_extends_wheels(self):
        world=ET.parse(ROOT/'worlds/room.sdf').getroot().find("world[@name='room']")
        self.assertIsNotNone(world)
        includes=world.findall('include')
        self.assertEqual(sum(i.findtext('uri')=='model://room_robot' for i in includes),1)
        model=ET.parse(ROOT/'models/room_robot/model.sdf').getroot().find('model')
        drive=model.find("plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual(len(drive.findall('left_joint')),2)
        self.assertEqual(len(drive.findall('right_joint')),2)
        self.assertEqual(drive.findtext('odom_topic'),'/sim/wheel_odom')
        self.assertEqual(model.findtext("plugin[@name='gz::sim::systems::OdometryPublisher']/odom_frame"),'sim_world')
        contact=model.find("link[@name='base_link']/sensor[@type='contact']")
        self.assertEqual(contact.findtext('contact/topic'),'/sim/contacts')
        # Gazebo publishes contact messages only during physical contact. Wheel-floor
        # contacts give evaluator liveness during clean runs without fake heartbeats.
        wheel_contacts=[]
        for link in model.findall('link'):
            for sensor in link.findall("sensor[@type='contact']"):
                self.assertEqual(sensor.findtext('contact/topic'),'/sim/contacts')
                self.assertIsNotNone(link.find("collision[@name='%s']" % sensor.findtext('contact/collision')))
                if link.attrib['name'].endswith('_wheel'):
                    wheel_contacts.append(sensor)
                    self.assertEqual(float(sensor.findtext('update_rate')),20.)
        self.assertEqual(len(wheel_contacts),4)
        size=list(map(float,model.findtext("link[@name='base_link']/collision[@name='bumper_collision']/geometry/box/size").split()))
        self.assertGreaterEqual(.19,math.hypot(size[0]/2,size[1]/2))


class LaunchTests(unittest.TestCase):
    def setUp(self):
        shares={'room_sim':str(ROOT),'room_nav':str(ROOT.parent/'room_nav'),'ros_gz_sim':'/fake/ros_gz_sim'}
        modules={
            'ament_index_python':module('ament_index_python'),
            'ament_index_python.packages':module('ament_index_python.packages',get_package_share_directory=shares.__getitem__),
            'launch':module('launch',LaunchDescription=Action),
            'launch.actions':module('launch.actions',**{key:Action for key in ['DeclareLaunchArgument','IncludeLaunchDescription','OpaqueFunction','SetEnvironmentVariable','LogInfo']}),
            'launch.launch_description_sources':module('launch.launch_description_sources',PythonLaunchDescriptionSource=Action),
            'launch.substitutions':module('launch.substitutions',LaunchConfiguration=Config),
            'launch_ros':module('launch_ros'),
            'launch_ros.actions':module('launch_ros.actions',Node=Action),
        }
        self.mock=patch.dict(sys.modules,modules)
        self.mock.start()
        spec=importlib.util.spec_from_file_location('room_sim_launch_offline',ROOT/'launch/sim.launch.py')
        self.launch=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.launch)
        self.defaults={a.args[0]:a.kwargs['default_value'] for a in self.launch.generate_launch_description().args[0] if a.args}

    def tearDown(self): self.mock.stop()

    def test_sim_launch_uses_guard_and_never_starts_serial_driver(self):
        actions=self.launch._setup(self.defaults)
        includes=[dict(a.kwargs['launch_arguments']) for a in actions if 'launch_arguments' in a.kwargs]
        nav=next(x for x in includes if 'start_base' in x)
        for name,value in {'start_base':'true','start_bridge':'false','start_lidar':'false',
                           'start_rf2o':'true','publish_laser_tf':'false','use_sim_time':'true'}.items():
            self.assertEqual(nav[name],value)
        nodes={a.kwargs['name']:a for a in actions if 'name' in a.kwargs}
        self.assertEqual(nodes['room_sim_drive'].kwargs['parameters'][0],{'use_sim_time':True,'simulation_autonomy':False})
        self.assertIn('room_sim_state_publisher',nodes)
        self.assertFalse(any(a.kwargs.get('executable')=='base_bridge' for a in actions))

    def test_headless_forwards_mission_toggle_and_invalid_world_rejected(self):
        actions=self.launch._setup({**self.defaults,'gui':'false','rviz':'false','simulation_autonomy':'true','start_mission':'false'})
        includes=[dict(a.kwargs['launch_arguments']) for a in actions if 'launch_arguments' in a.kwargs]
        self.assertIn('--headless-rendering',next(x for x in includes if 'gz_args' in x)['gz_args'])
        self.assertEqual(next(x for x in includes if 'start_mission' in x)['start_mission'],'false')
        with tempfile.TemporaryDirectory() as tmp:
            invalid=Path(tmp)/'wrong.sdf'
            invalid.write_text('<sdf version="1.9"><world name="wrong"/></sdf>')
            with self.assertRaises(ValueError): self.launch._setup({**self.defaults,'world':str(invalid)})


if __name__=='__main__':
    unittest.main()
