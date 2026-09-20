#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_python="${FARMILY_ISAAC_PYTHON:-/root/isaaclab_env/bin/python}"
exec "$task_python" -u "${task_dir}/rl/pose_videos.py" "$@"
