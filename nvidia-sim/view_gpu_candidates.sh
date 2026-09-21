#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export DISPLAY="${DISPLAY:-:0}"
exec "${task_dir}/run_gpu_candidate_dataset.sh" --num-envs 16 --candidates 16 --view-grid --keep-open "$@"
