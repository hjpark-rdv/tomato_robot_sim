#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
native_python="$repo_dir/mujoco-benchmark/.mujoco315-venv/bin/python"
render_python="$repo_dir/mujoco-benchmark/.mujoco315-render-venv/bin/python"
scripts_dir="$repo_dir/mujoco-benchmark/scripts/s300_smoke"
data_dir="/root/docker_share/mujoko_debugging_data"
mode="${1:-replay}"
if (($#)); then shift; fi
case "$mode" in
  replay|render)
    trial_dir="$data_dir/20261012_s300_mujoco_full/campaign_final"
    if (($#)) && [[ "$1" != -* ]]; then trial_dir="$1"; shift; fi
    export DISPLAY="${S300_MUJOCO_RENDER_DISPLAY:-:0}"
    quality="${S300_MUJOCO_QUALITY:-ultra}"
    args=("$@")
    for ((arg_i=0; arg_i<${#args[@]}; arg_i++)); do
      if [[ "${args[$arg_i]}" == --quality ]] && ((arg_i+1<${#args[@]})); then quality="${args[$((arg_i+1))]}"; fi
      if [[ "${args[$arg_i]}" == --quality=* ]]; then quality="${args[$arg_i]#--quality=}"; fi
    done
    display_python="$native_python"
    if [[ "$quality" != fast ]]; then
      display_python="$render_python"
      if [[ ! -x "$display_python" ]]; then
        echo 'Prepare display environment: ./mujoco-benchmark/setup_s300_render.sh' >&2
        exit 1
      fi
    fi
    display_options=(--camera aisle --quality "$quality")
    if [[ "$mode" == replay ]]; then display_options+=(--gui); fi
    exec "$display_python" -u "$scripts_dir/view_smoke.py" "$trial_dir" "${display_options[@]}" "$@"
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
    echo 'Usage: run_s300_mujoco.sh replay|render [TRIAL] [--quality fast|balanced|ultra] [viewer options] | test [physics options] | prepare [OUTPUT] [source options]' >&2
    exit 2
    ;;
esac
