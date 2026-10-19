#!/usr/bin/env bash
set -euo pipefail
mode=${1:-sim}
if [[ "$mode" != sim && "$mode" != hardware ]]; then
  echo 'Usage: bash scripts/install_system.sh [sim|hardware]' >&2
  exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'Install ROS2 Jazzy on Ubuntu24.04 first; see docs/INSTALL_RU.md.' >&2
  exit 1
fi
packages=(git unzip build-essential python3-colcon-common-extensions python3-rosdep
  python3-venv python3-pip python3-yaml python3-serial python3-numpy python3-tk
  libeigen3-dev ros-jazzy-navigation2 ros-jazzy-nav2-bringup
  ros-jazzy-slam-toolbox ros-jazzy-tf2-ros ros-jazzy-nav2-collision-monitor
  ros-jazzy-rviz2 ros-jazzy-nav2-rviz-plugins)
if [[ "$mode" == sim ]]; then
  packages+=(ros-jazzy-ros-gz ros-jazzy-robot-state-publisher ros-jazzy-xacro)
fi
sudo apt update
sudo apt install -y "${packages[@]}"
printf 'Dependencies installed for %s. Next: rosdep update and build_ros.sh.\n' "$mode"
