from setuptools import setup
from glob import glob
setup(
    name='room_nav', version='4.0.0', packages=['room_nav'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/room_nav']),
                ('share/room_nav', ['package.xml']),
                ('share/room_nav/launch', glob('launch/*.launch.py')),
                ('share/room_nav/config', glob('config/*.yaml'))],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='Room robot project', maintainer_email='robot@example.invalid',
    description='Encoderless RF2O, RPLIDAR C1, room SLAM and Nav2', license='Apache-2.0',
    entry_points={'console_scripts': ['mission = room_nav.mission_node:main']},
)
