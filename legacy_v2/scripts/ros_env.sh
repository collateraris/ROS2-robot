#!/usr/bin/env bash
# Source this file in each Pi and PC terminal. No persistent shell changes.
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is not installed at /opt/ros/jazzy.' >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash
roomwm_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -r "$roomwm_project_dir/ros_install/setup.bash" ]]; then
  source "$roomwm_project_dir/ros_install/setup.bash"
fi
unset roomwm_project_dir
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
# This does not open a firewall, provide authentication, or connect a VPN.
