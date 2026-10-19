#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
TASK_EIGEN=${1:-/usr/include/eigen3}
if [[ ! -f "$TASK_EIGEN/Eigen/Dense" ]]; then
  echo 'Usage: bash vendor/tests/check_rf2o_math.sh /path/to/eigen3 (Eigen 3.4 tested)' >&2
  exit 1
fi
TASK_BUILD=$(mktemp -d)
trap 'rm -rf -- "$TASK_BUILD"' EXIT
# Upstream includes eigen3/Eigen and eigen3/unsupported paths.
ln -s "$TASK_EIGEN" "$TASK_BUILD/eigen3"
${CXX:-c++} -std=c++14 -O1 -g0 \
  -I"$TASK_ROOT/vendor/tests/rf2o_host_stubs" -I"$TASK_BUILD" -I"$TASK_EIGEN" \
  -I"$TASK_ROOT/ros2_ws/src/rf2o_laser_odometry/include" \
  "$TASK_ROOT/ros2_ws/src/rf2o_laser_odometry/src/CLaserOdometry2D.cpp" \
  "$TASK_ROOT/vendor/tests/rf2o_math_host.cpp" -o "$TASK_BUILD/rf2o_math"
"$TASK_BUILD/rf2o_math"
