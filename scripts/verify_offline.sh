#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root:$project_root/ros2_ws/src/room_nav:$project_root/ros2_ws/src/room_base:$project_root/ros2_ws/src/room_sim:$project_root/ros2_ws/src/room_tasks:$project_root/ros2_ws/src/room_perception:$project_root/ros2_ws/src/room_eval${PYTHONPATH:+:$PYTHONPATH}"
compile_paths=(learning_nav robot_extras ros2_ws/src)
for room_module in room_semantics room_planning room_evaluation; do
  if [[ -d "$room_module" ]]; then compile_paths+=("$room_module"); fi
done
python3 -m compileall -q "${compile_paths[@]}"
test_paths=(tests learning_nav/tests)
for room_test_path in ros2_ws/src/room_*/test; do
  if [[ -d "$room_test_path" ]]; then test_paths+=("$room_test_path"); fi
done
python3 -m pytest "${test_paths[@]}"

firmware_build_dir=$(mktemp -d)
trap 'rm -rf -- "$firmware_build_dir"' EXIT
${CXX:-g++} -std=c++17 -Wall -Wextra -Werror \
  -Ifirmware/room_base_rf2o/tests firmware/room_base_rf2o/tests/firmware_test.cpp \
  -o "$firmware_build_dir/firmware_test"
"$firmware_build_dir/firmware_test"
rf2o_eigen_dir=${RF2O_EIGEN_DIR:-/usr/include/eigen3}
if [[ -f "$rf2o_eigen_dir/Eigen/Dense" ]]; then
  bash vendor/tests/check_rf2o_math.sh "$rf2o_eigen_dir"
else
  printf 'RF2O numerical host check skipped: install libeigen3-dev or set RF2O_EIGEN_DIR.\n'
fi
