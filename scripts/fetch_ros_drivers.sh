#!/usr/bin/env bash
# Fetch exact reviewed upstream source revisions into this ROS workspace.
# Existing directories are refused unless they carry our matching provenance marker.
set -euo pipefail
TASK_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
TASK_SRC="$TASK_ROOT/ros2_ws/src"
TASK_PATCH="$TASK_ROOT/vendor/patches/rf2o_jazzy_guard.patch"
RF2O_COMMIT=b38c68e46387b98845ecbfeb6660292f967a00d3
SLLIDAR_COMMIT=34300099fadfc772965962dec837bf436706188f
command -v git >/dev/null || { echo 'Install git first.' >&2; exit 1; }
command -v sha256sum >/dev/null || { echo 'sha256sum is required.' >&2; exit 1; }
TASK_PATCH_HASH=$(sha256sum "$TASK_PATCH" | cut -d ' ' -f 1)
mkdir -p "$TASK_SRC"
fetch_reviewed_source() {
  local name="$1" url="$2" revision="$3" patch_file="$4" expected_marker="$5"
  local destination="$TASK_SRC/$name" staging
  if [[ -e "$destination" ]]; then
    if [[ -f "$destination/ROOM_UPSTREAM_REVISION" ]] && [[ $(cat "$destination/ROOM_UPSTREAM_REVISION") == "$expected_marker" ]]; then
      echo "$name already prepared at the reviewed revision."
      return
    fi
    echo "Refusing to overwrite existing $destination. Move it aside deliberately before retrying." >&2
    return 1
  fi
  staging=$(mktemp -d "$TASK_SRC/.${name}.XXXXXX")
  if ! git -C "$staging" init -q ||
     ! git -C "$staging" remote add origin "$url" ||
     ! git -C "$staging" fetch --depth 1 origin "$revision" ||
     ! git -C "$staging" checkout --detach -q FETCH_HEAD; then
    rm -rf -- "$staging"
    return 1
  fi
  if [[ $(git -C "$staging" rev-parse HEAD) != "$revision" ]]; then
    rm -rf -- "$staging"
    echo 'Unexpected upstream revision.' >&2
    return 1
  fi
  if [[ -n "$patch_file" ]] && ! git -C "$staging" apply --check "$patch_file"; then
    rm -rf -- "$staging"
    echo 'RF2O patch does not match the reviewed source; no files installed.' >&2
    return 1
  fi
  if [[ -n "$patch_file" ]]; then
    git -C "$staging" apply "$patch_file"
  fi
  printf '%s\n' "$expected_marker" > "$staging/ROOM_UPSTREAM_REVISION"
  # Distribute upstream licences and source, without nested repository metadata.
  rm -rf -- "$staging/.git"
  mv -- "$staging" "$destination"
  echo "Prepared $name ($revision)."
}
fetch_reviewed_source rf2o_laser_odometry https://github.com/MAPIRlab/rf2o_laser_odometry.git \
  "$RF2O_COMMIT" "$TASK_PATCH" "${RF2O_COMMIT} patch_sha256=${TASK_PATCH_HASH}"
fetch_reviewed_source sllidar_ros2 https://github.com/Slamtec/sllidar_ros2.git \
  "$SLLIDAR_COMMIT" '' "$SLLIDAR_COMMIT"
echo 'Source ready. Install Jazzy dependencies with rosdep, then colcon build.'
