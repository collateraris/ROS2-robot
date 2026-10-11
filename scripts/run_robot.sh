#!/usr/bin/env bash
# Run after colcon build. Arguments are forwarded to ros2 launch.
set -eo pipefail
ROBOT_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_PROJECT_DIR="$(dirname -- "$ROBOT_SCRIPT_DIR")"
if [[ -z "${ROBOT_PASSWORD:-}" ]]; then
    read -rsp 'Пароль для страницы робота (от 8 символов): ' ROBOT_PASSWORD
    printf '\n'
fi
export ROBOT_PASSWORD
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
source /opt/ros/jazzy/setup.bash
source "$ROBOT_PROJECT_DIR/ros2_ws/install/setup.bash"
exec ros2 launch robot_web_teleop robot.launch.py "$@"
