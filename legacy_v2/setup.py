from setuptools import find_packages, setup
from glob import glob

setup(
    name="room_world_model", version="0.2.0", packages=find_packages(),
    description="Camera-only room navigation research with pulse-bounded actuation",
    license="MIT", python_requires=">=3.10",
    install_requires=["numpy>=1.24,<3", "PyYAML>=6", "Pillow>=10"],
    extras_require={
        "pi": ["pyserial>=3.5", "opencv-python-headless>=4.10,<6"],
        "ml": ["torch>=2.4,<3", "torchvision>=0.19", "opencv-python>=4.10,<6", "transformers>=4.45,<6", "ultralytics>=8.3,<9", "huggingface-hub>=0.25"],
        "test": ["pytest>=8"],
        "calibration": ["gpiozero>=2.0"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/room_world_model"]),
        ("share/room_world_model", ["package.xml"]),
        ("share/room_world_model/config", glob("config/*.yaml") + glob("config/*.json")),
        ("share/room_world_model/launch", glob("launch/*.launch.py")),
        ("share/room_world_model/firmware/roomwm_drive", ["firmware/roomwm_drive/roomwm_drive.ino"]),
    ],
    entry_points={"console_scripts": [
        "roomwm_pi=room_world_model.ros_pi:main",
        "roomwm_pc=room_world_model.ros_pc:main",
        "roomwm_operator=room_world_model.operator:main",
        "roomwm_goal=room_world_model.goal_client:main",
    ]},
)
