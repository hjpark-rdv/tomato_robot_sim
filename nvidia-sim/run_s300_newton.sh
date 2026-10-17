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
    '       run_s300_newton.sh gpu [INPUTS] [--gui] [--output OUTPUT] [runner options]' \
    '       run_s300_newton.sh gpu-fast [INPUTS] [--gui] [--output OUTPUT] [runner options]' \
    '       run_s300_newton.sh gpu-realtime [INPUTS] [--gui] [--output OUTPUT] [runner options]' \
    '       run_s300_newton.sh check' \
    'prepare reuses the existing model and command sampler; it advances zero physics steps.' \
    'run defaults to Newton CPU: contact 400 Hz, free transfer 60 Hz. --gui adds the independent 15 FPS full-farm display.' \
    'gpu runs actual CUDA physics: cuSOLVER32, fixed tree/contact/force order, dense Hessian, 400/60 Hz, capacities128/256.' \
    'gpu-fast uses the separately calibrated 133-DOF plant and conservative fruit proxies (outward bound 0.1mm). This changes the physical representation.' \
    'gpu-fast inputs can be overridden with S300_NEWTON_FAST_INPUTS; gpu keeps the original model default.' \
    'gpu-realtime opts into the reviewed 133-DOF fruit-proxy model at 220/60 Hz and the exact belt-only 2mm grip/open-stroke guard; other pairs remain 1mm.' \
    'gpu-realtime is a changed physical approximation, not an identical-model acceleration or real-plant calibration.' \
    'Experimental --fused-sparse-factor remains opt-in and is not a GPU-mode default.' \
    'GPU single-world full GUI/headless equivalence passed; contact-phase realtime remains below target.' \
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
  run|gpu|gpu-fast|gpu-realtime)
    input_dir="${S300_NEWTON_INPUTS:-$data_dir/20261016_s300_newton/inputs_render_separated}"
    if [[ "$mode" == 'gpu-fast' ]]; then
      input_dir="${S300_NEWTON_FAST_INPUTS:-$data_dir/20261017_newton_solver_compare/inputs_coarse4_fruit_proxy_01}"
      printf '%s\n' 'Experimental GPU-fast: changed plant coordinates and conservatively enlarged fruit collision proxies; not an identical-model acceleration.' >&2
    fi
    if [[ "$mode" == 'gpu-realtime' ]]; then
      input_dir="${S300_NEWTON_REALTIME_INPUTS:-$data_dir/20261017_newton_solver_compare/inputs_coarse4_fruit_proxy_01}"
    fi
    if [[ "$mode" == 'gpu' || "$mode" == 'gpu-fast' || "$mode" == 'gpu-realtime' ]]; then
      output_dir="${S300_NEWTON_OUTPUT:-$data_dir/$(date +%Y%m%d_%H%M%S)_s300_newton_gpu}"
      runner_options=(--backend gpu --native-observables-only --native-control-callback
        --linear-algebra cusolver32 --hz 400 --free-hz 60
        --inertia-linear-algebra cusolver32 --tree-sums flat --canonical-contacts
        --deterministic RUN_TO_RUN --deterministic-scope solver --dense-hessian
        --fixed-solver-reductions --indexed-qfrc --native-integer-atomics --canonical-constraints
        --nconmax 128 --njmax 256 --viewer-fps 15)
      if [[ "$mode" == 'gpu-realtime' ]]; then
        runner_options+=(--hz 220 --contact-policy "$scripts/profiles/belt_strokes_2mm.json" --camera /World/HarvestCloseCamera)
        printf '%s\n' 'Experimental Newton GPU realtime profile: 133 DOF, conservative fruit proxies, 220/60Hz, belt-only 2mm grip/open strokes. Other pairs 1mm; original material and release coefficients retained.' >&2
      else
        printf '%s\n' 'Newton GPU original 400/60Hz preset: actual CUDA physics; contact-phase realtime target is not met by this preset.' >&2
      fi
    else
      output_dir="${S300_NEWTON_OUTPUT:-$data_dir/$(date +%Y%m%d_%H%M%S)_s300_newton_harvest}"
      runner_options=(--backend cpu --hz 400 --free-hz 60 --viewer-fps 15 --native-observables-only)
    fi
    if (($#)) && [[ "$1" != -* ]]; then input_dir="$1"; shift; fi
    if [[ ! -f "$input_dir/inputs.json" ]]; then
      printf 'Prepared inputs missing: %s/inputs.json\nUse the prepare command first.\n' "$input_dir" >&2
      exit 1
    fi
    export DISPLAY="${S300_NEWTON_DISPLAY:-:0}"
    exec env -u PYTHONPATH "$python" -u "$scripts/run_newton_harvest.py" \
      "$input_dir" --output "$output_dir" "${runner_options[@]}" "$@"
    ;;
  *)
    printf 'Unknown mode: %s\nUse %s --help.\n' "$mode" "$0" >&2
    exit 2
    ;;
esac
