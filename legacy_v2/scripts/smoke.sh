#!/usr/bin/env bash
# CPU-only synthetic test. No ROS, camera, serial port or model download.
set -euo pipefail
ROOMWM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$ROOMWM_ROOT"
ROOMWM_SMOKE_DIR="${1:-runs/software_smoke}"
python -m room_world_model.smoke --output "$ROOMWM_SMOKE_DIR"
python -m pytest -q
