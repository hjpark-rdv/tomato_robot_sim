#!/usr/bin/env bash
# Tomato_05 fixture demo: corrected ROI; prior [102,284,88,89] tracked Tomato_06.
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_output="${task_dir}/rl/runs/$(date +%Y%m%d_%H%M%S)_rgb_guard_tomato05"
exec "${task_dir}/run_ring_rl.sh" --mode rgb --rgb-action guard \
  --rgb-camera left --rgb-roi 194 197 80 86 --run-dir "$task_output" "$@"
