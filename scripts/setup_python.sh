#!/usr/bin/env bash
set -euo pipefail
room_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-base}
torch_mode=${2:-existing}
case "$mode" in base) extras=test;; perception) extras=test,perception,eval;; learning) extras=test,ml,eval;; all) extras=test,perception,ml,eval;; *) echo 'Usage: setup_python.sh [base|perception|learning|all] [existing|cpu]' >&2; exit 2;; esac
case "$torch_mode" in existing|cpu);; *) echo 'torch mode must be existing or cpu' >&2; exit 2;; esac
cd "$room_root"
if [[ ! -d .venv ]]; then /usr/bin/python3 -m venv --system-site-packages .venv; fi
.venv/bin/python -m pip install --upgrade pip
if [[ "$torch_mode" == cpu ]]; then
  .venv/bin/python -m pip install 'torch>=2.6,<3' --index-url https://download.pytorch.org/whl/cpu
elif [[ "$mode" != base ]]; then
  .venv/bin/python -c 'import torch; print("Existing PyTorch:",torch.__version__,"CUDA:",torch.cuda.is_available())' || {
    echo 'Install a suitable PyTorch build in .venv first, or choose cpu explicitly.' >&2
    exit 1
  }
fi
.venv/bin/python -m pip install -e ".[${extras}]"
printf 'Python environment ready. Run: source scripts/env_robot.sh\n'
