#!/usr/bin/env bash
set -euo pipefail
room_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-all}
if [[ -n "${VIRTUAL_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
  echo 'Build in a fresh terminal with system /usr/bin/python3, before activating a venv/Conda.' >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$room_root/ros2_ws"
available=4
if [[ -f "$room_root/WEEK_STAGE" ]]; then available=$(cat "$room_root/WEEK_STAGE"); fi
case "$available" in 1|2|3|4);; *) echo 'Invalid WEEK_STAGE' >&2; exit 2;; esac
case "$mode" in
  all) stage=$available;;
  week01) stage=1;; week02) stage=2;; week03) stage=3;; week04) stage=4;;
  hardware) stage=$available;;
  *) echo 'Usage: build_ros.sh [all|hardware|week01|week02|week03|week04]' >&2; exit 2;;
esac
if (( stage > available )); then echo 'Requested week is not included in this snapshot' >&2; exit 2; fi
packages=(room_base room_nav)
if [[ "$mode" != hardware ]]; then packages+=(room_sim); fi
if (( stage >= 2 )); then packages+=(room_perception); fi
if (( stage >= 3 )); then packages+=(room_tasks); fi
if (( stage >= 4 )) && [[ "$mode" != hardware ]]; then packages+=(room_eval); fi
colcon build --symlink-install --base-paths src --packages-up-to "${packages[@]}"
printf 'Build complete. Next: bash scripts/setup_python.sh base; source scripts/env_robot.sh\n'
