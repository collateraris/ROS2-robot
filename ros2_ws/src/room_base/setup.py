from setuptools import setup
from glob import glob

setup(name='room_base', version='0.4.0', packages=['room_base'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/room_base']),
                  ('share/room_base', ['package.xml']),
                  ('share/room_base/config', glob('config/*.yaml')),
                  ('share/room_base/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Room robot project', maintainer_email='robot@example.invalid',
      description='RF2O guarded encoderless base bridge', license='MIT',
      entry_points={'console_scripts': [
          'base_bridge = room_base.encoderless_bridge:main',
          'odom_guard = room_base.odom_guard:main',
          'base_bridge_encoders = room_base.bridge:main']})
