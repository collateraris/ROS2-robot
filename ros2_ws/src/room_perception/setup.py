from setuptools import setup
from glob import glob
setup(name="room_perception",version="0.5.0",packages=["room_perception"],
      data_files=[("share/ament_index/resource_index/packages",["resource/room_perception"]),
                  ("share/room_perception",["package.xml"]),
                  ("share/room_perception/launch",glob("launch/*.launch.py")),
                  ("share/room_perception/config",glob("config/*.yaml"))],
      install_requires=["setuptools"],zip_safe=True,
      maintainer="Room robot project",maintainer_email="robot@example.invalid",
      description="Camera / planar LiDAR semantic perception, no actuation",license="Apache-2.0",
      entry_points={"console_scripts":["perception = room_perception.node:main"]})
