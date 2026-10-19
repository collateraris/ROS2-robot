from pathlib import Path
from setuptools import setup

assets = []
for directory in ('launch', 'config', 'models', 'urdf', 'worlds'):
    groups = {}
    for path in Path(directory).rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:
            groups.setdefault(str(Path('share/room_sim') / path.parent), []).append(str(path))
    assets.extend(groups.items())
setup(name='room_sim', version='5.0.0', packages=['room_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/room_sim']),
                  ('share/room_sim', ['package.xml'])] + assets,
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Room robot project', maintainer_email='robot@example.invalid',
      description='Gazebo Harmonic room and guarded encoderless navigation', license='Apache-2.0',
      entry_points={'console_scripts': ['drive_adapter=room_sim.drive_adapter:main',
                                       'goal_ab=room_sim.goal_ab:main']})
