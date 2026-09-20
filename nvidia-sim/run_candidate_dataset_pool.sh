#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_python="${FARMILY_ISAAC_PYTHON:-/root/isaaclab_env/bin/python}"
unset PYTHONHOME VIRTUAL_ENV
export PYTHONPATH="${task_dir}/rl"
export DISPLAY="${DISPLAY:-:0}"
exec "$task_python" -u "${task_dir}/rl/dataset_pool.py" "$@"
