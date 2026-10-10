#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_isaac="${FARMILY_ISAAC_61_ROOT:-/root/isaac-sim-6.1.0}"
if [[ ! -x "$task_isaac/python.sh" ]]; then
  echo "Isaac Sim 6.1 python.sh를 찾지 못했습니다: $task_isaac" >&2
  exit 1
fi
export DISPLAY="${DISPLAY:-:0}"
export OMNI_KIT_ALLOW_ROOT=1
unset PYTHONHOME VIRTUAL_ENV PYTHONPATH
exec "$task_isaac/python.sh" --no-ros-env -u "$task_dir/harvest_s300/run_scene.py" "$@"
