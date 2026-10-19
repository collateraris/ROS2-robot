from setuptools import setup
setup(name='room_eval',version='0.5.0',packages=['room_eval'],
      data_files=[('share/ament_index/resource_index/packages',['resource/room_eval']),
                  ('share/room_eval',['package.xml'])],
      install_requires=['setuptools'],zip_safe=True,
      maintainer='Room robot project',maintainer_email='robot@example.invalid',
      description='Simulation-only episode runner and GT metrics',license='MIT',
      entry_points={'console_scripts':['evaluate = room_eval.runner:main']})
