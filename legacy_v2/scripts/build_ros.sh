#!/usr/bin/env bash
# Run from the project checkout after activating its system-site-packages venv.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is required at /opt/ros/jazzy.' >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$project_dir"
# Exact paths are essential: the interface package is nested in an ament_python
# checkout and ordinary recursive discovery stops at the outer package.
python3 -c 'from colcon_core.command import main; raise SystemExit(main())' \
  --log-base "$project_dir/ros_log" build \
  --paths "$project_dir" "$project_dir/ros2/roomwm_msgs" \
  --build-base "$project_dir/ros_build" --install-base "$project_dir/ros_install" \
  --symlink-install --packages-select roomwm_msgs room_world_model \
  --event-handlers console_direct+
echo 'Build complete. Run: source scripts/ros_env.sh'
