#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
native_python="$repo_dir/mujoco-benchmark/.mujoco315-venv/bin/python"
scripts_dir="$repo_dir/mujoco-benchmark/scripts/s300_smoke"
data_dir="/root/docker_share/mujoko_debugging_data"
mode="${1:-replay}"
if (($#)); then shift; fi
case "$mode" in
  replay)
    trial_dir="$data_dir/20261012_s300_mujoco_full/campaign_final"
    if (($#)) && [[ "$1" != -* ]]; then trial_dir="$1"; shift; fi
    export DISPLAY="${DISPLAY:-:0}"
    exec "$native_python" "$scripts_dir/view_smoke.py" "$trial_dir" --gui --camera aisle "$@"
    ;;
  test)
    model_dir="${S300_MUJOCO_MODEL:-$data_dir/20261012_s300_mujoco_rebuilt/model}"
    output_dir="${S300_MUJOCO_OUTPUT:-$data_dir/$(date +%Y%m%d_%H%M%S)_s300_mujoco_harvest}"
    exec "$native_python" -u "$scripts_dir/run_harvest.py" "$model_dir" "$output_dir" \
      --hz 300 --free-hz 60 --windows 2 --max-cycles 2 --released-gutter-diagnostic "$@"
    ;;
  prepare)
    output_dir="$data_dir/$(date +%Y%m%d_%H%M%S)_s300_mujoco_assets"
    if (($#)) && [[ "$1" != -* ]]; then output_dir="$1"; shift; fi
    exec "$native_python" -u "$scripts_dir/prepare_harvest.py" "$output_dir" "$@"
    ;;
  *)
    echo 'Usage: run_s300_mujoco.sh replay [TRIAL] [viewer options] | test [test options] | prepare [OUTPUT] [source options]' >&2
    exit 2
    ;;
esac
