from glob import glob
from setuptools import setup

package_name = 'robot_web_teleop'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/web', glob('web/*.html')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Robot owner', maintainer_email='robot@example.invalid',
    description='Wi-Fi phone teleoperation, camera and serial Arduino bridge',
    license='MIT',
    entry_points={'console_scripts': [
        'camera_node = robot_web_teleop.camera_node:main',
        'motor_bridge = robot_web_teleop.motor_bridge:main',
        'web_node = robot_web_teleop.web_node:main',
    ]},
)
