#!/usr/bin/env bash
set -euo pipefail
ROOM_W2_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ $# -ne 1 ]]; then echo "Usage: $0 /absolute/path/to/real-room-image.jpg" >&2; exit 2; fi
if [[ ! -x "$ROOM_W2_ROOT/.venv/bin/python" ]]; then echo 'Run scripts/setup_python.sh and install week02 requirements first.' >&2; exit 2; fi
cd "$ROOM_W2_ROOT"
export PYTHONPATH="$ROOM_W2_ROOT${PYTHONPATH:+:$PYTHONPATH}"
"$ROOM_W2_ROOT/.venv/bin/python" -m room_semantics.detect_image "$1" \
  --out "$ROOM_W2_ROOT/runs/week02/detections.json" \
  --annotated "$ROOM_W2_ROOT/runs/week02/detections.jpg" \
  --device "${PERCEPTION_DEVICE:-auto}"
