#!/usr/bin/env bash
# SOURCE this file in every interactive terminal; do not execute as a subprocess.
ROOM_PROJECT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export ROOM_PROJECT_ROOT
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$ROOM_PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
  echo 'ROS Jazzy and the project overlay must be installed/built first.' >&2
  return 1 2>/dev/null || exit 1
fi
_room_restore_nounset=false
case $- in *u*) _room_restore_nounset=true; set +u;; esac
source /opt/ros/jazzy/setup.bash
source "$ROOM_PROJECT_ROOT/ros2_ws/install/setup.bash"
export PYTHONPATH="$ROOM_PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [[ -f "$ROOM_PROJECT_ROOT/.venv/bin/activate" ]]; then source "$ROOM_PROJECT_ROOT/.venv/bin/activate"; fi
if [[ "$_room_restore_nounset" == true ]]; then set -u; fi
unset _room_restore_nounset
cd "$ROOM_PROJECT_ROOT"
