#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export DISPLAY="${FARMILY_DISPLAY:-:0}"
exec "${task_dir}/run_ring_rl.sh" --mode elastic --elastic-action push "$@"
