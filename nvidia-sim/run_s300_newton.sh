#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
scripts="$task_dir/harvest_s300"
python="${S300_NEWTON_PYTHON:-$task_dir/.newton-venv/bin/python}"
data_dir="${S300_NEWTON_DATA:-/root/docker_share/mujoko_debugging_data}"
mode="${1:-run}"
if (($#)); then shift; fi
if [[ "$mode" == '--help' || "$mode" == '-h' || "$mode" == 'help' ]]; then
  printf '%s\n' \
    'Usage: run_s300_newton.sh prepare [MODEL OUTPUT] [prepare options]' \
    '       run_s300_newton.sh run [INPUTS] [--gui] [--output OUTPUT] [runner options]' \
    '       run_s300_newton.sh check' \
    'prepare reuses the existing model and command sampler; it advances zero physics steps.' \
    'run defaults to Newton CPU: contact 400 Hz, free transfer 60 Hz. --gui adds the independent 15 FPS full-farm display.' \
    'GPU backend is diagnostic: single-world physics validity has not passed repeat validation.' \
    'CPU parallel trials use harvest_s300/benchmark_newton.py; see S300_NEWTON.md.' \
    'Defaults can be overridden with S300_NEWTON_MODEL, S300_NEWTON_INPUTS, S300_NEWTON_OUTPUT.'
  exit 0
fi
if [[ "$mode" == 'check' ]]; then
  exec "$task_dir/setup_s300_newton.sh" --check
fi
if [[ ! -x "$python" ]]; then
  printf 'Missing Newton Python: %s\nRun %s/setup_s300_newton.sh first.\n' "$python" "$task_dir" >&2
  exit 1
fi
export PYTHONNOUSERSITE=1
case "$mode" in
  prepare)
    model_dir="${S300_NEWTON_MODEL:-$data_dir/20261012_s300_mujoco_rebuilt/window_00}"
    output_dir="${S300_NEWTON_OUTPUT:-$data_dir/$(date +%Y%m%d_%H%M%S)_s300_newton_inputs}"
    if (($#)) && [[ "$1" != -* ]]; then model_dir="$1"; shift; fi
    if (($#)) && [[ "$1" != -* ]]; then output_dir="$1"; shift; fi
    exec env -u PYTHONPATH "$python" -u "$scripts/newton_inputs.py" \
      "$model_dir" "$output_dir" --hz 300 "$@"
    ;;
  run)
    input_dir="${S300_NEWTON_INPUTS:-$data_dir/20261016_s300_newton/inputs_render_separated}"
    output_dir="${S300_NEWTON_OUTPUT:-$data_dir/$(date +%Y%m%d_%H%M%S)_s300_newton_harvest}"
    if (($#)) && [[ "$1" != -* ]]; then input_dir="$1"; shift; fi
    if [[ ! -f "$input_dir/inputs.json" ]]; then
      printf 'Prepared inputs missing: %s/inputs.json\nUse the prepare command first.\n' "$input_dir" >&2
      exit 1
    fi
    export DISPLAY="${S300_NEWTON_DISPLAY:-:0}"
    exec env -u PYTHONPATH "$python" -u "$scripts/run_newton_harvest.py" \
      "$input_dir" --output "$output_dir" --backend cpu --hz 400 --free-hz 60 --viewer-fps 15 --native-observables-only "$@"
    ;;
  *)
    printf 'Unknown mode: %s\nUse %s --help.\n' "$mode" "$0" >&2
    exit 2
    ;;
esac
