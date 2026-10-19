#!/usr/bin/env bash
# Official ROS Jazzy/Gazebo Harmonic pairing. Requires configured ROS apt repository.
set -euo pipefail
if [[ ${ROS_DISTRO:-jazzy} != jazzy ]]; then
  echo 'This project targets ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04.' >&2
  exit 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'Install ROS 2 Jazzy first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html' >&2
  exit 1
fi
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-gz ros-jazzy-robot-state-publisher ros-jazzy-rviz2
printf '%s\n' 'Simulation dependencies installed. Run the project build script, then source ros2_ws/install/setup.bash.'
