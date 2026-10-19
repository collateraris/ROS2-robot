from setuptools import setup
from glob import glob

setup(name='room_tasks',version='5.0.0',packages=['room_tasks'],
    data_files=[('share/ament_index/resource_index/packages',['resource/room_tasks']),
                ('share/room_tasks',['package.xml']),
                ('share/room_tasks/launch',glob('launch/*.launch.py')),
                ('share/room_tasks/config',glob('config/*.yaml'))],
    install_requires=['setuptools','PyYAML'],zip_safe=True,
    maintainer='Room robot project',maintainer_email='robot@example.invalid',
    description='Bounded local language/vision planner with guarded Nav2 execution',license='MIT',
    entry_points={'console_scripts':['coordinator = room_tasks.coordinator:main']})
