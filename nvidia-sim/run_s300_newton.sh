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
    '       run_s300_newton.sh row-realtime [PREPARED_ROUTE] [row runner options]' \
    '       run_s300_newton.sh standalone [INPUTS] [--headless] [runner options]' \
    '       run_s300_newton.sh prepare-standalone [OUTPUT] [builder options]' \
    '       run_s300_newton.sh camera-patrol [ASSETS] [--headless] [patrol options]' \
    '       run_s300_newton.sh render-patrol RUN --output VIDEO.mp4 [--fps 6]' \
    '       run_s300_newton.sh prepare-patrol OUTPUT [--end-station METRES]' \
    '       run_s300_newton.sh prepare-route REFERENCE_ASSETS OUTPUT --end-station METRES' \
    '       run_s300_newton.sh prepare-scene-stream ASSETS OUTPUT [chunk options]' \
    '       run_s300_newton.sh prepare-geometric-patrol SOURCE_ASSETS NEW_ASSETS' \
    '       run_s300_newton.sh check' \
    'prepare reuses the existing model and command sampler; it advances zero physics steps.' \
    'run defaults to Newton CPU: contact 400 Hz, free transfer 60 Hz. --gui adds the independent 15 FPS full-farm display.' \
    'gpu runs actual CUDA physics: cuSOLVER32, fixed tree/contact/force order, dense Hessian, 400/60 Hz, capacities128/256.' \
    'gpu-fast uses the separately benchmarked 133-DOF plant and conservative fruit proxies (outward bound 0.1mm). This changes the physical representation.' \
    'gpu-fast inputs can be overridden with S300_NEWTON_FAST_INPUTS; gpu keeps the original model default.' \
    'gpu-realtime opts into the reviewed 133-DOF fruit-proxy model at 220/60 Hz and the exact belt-only 2mm grip/open-stroke guard; other monitored pairs remain 1mm, with the existing released-fruit exceptions preserved.' \
    'gpu-realtime is a changed physical approximation, not an identical-model acceleration or real-plant calibration.' \
    'gpu-realtime --gui waits for START after rendering, runs at1x and keeps the final scene open. Add --no-interactive-viewer for an automatic maximum-throughput GUI benchmark.' \
    'row-realtime reuses the one-bed route runner: current plant elastic, other plants fixed with colliders, released fruit dynamic; 220/60 Hz and unchanged contact policy. Segment preparation/loading is separate from loop RTF.' \
    'standalone uses the same GPU-realtime physics with a live Viser browser at port8091. Press START; repeat without reloading the scene. Minimal logs by default; --recording full enables optional state archives.' \
    'prepare-standalone rebuilds reusable local inputs from source USD/GLB using .asset-venv. No old trial states or NAS are required.' \
    'Experimental --fused-sparse-factor remains opt-in and is not a GPU-mode default.' \
    'GPU single-world full GUI/headless equivalence passed; the original 400/60 Hz presets remain below the contact-phase realtime target.' \
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
  render-patrol)
    exec env -u PYTHONPATH "$python" -u "$scripts/render_newton_patrol.py" "$@"
    ;;
  prepare-geometric-patrol)
    exec env -u PYTHONPATH "$python" -u "$scripts/newton_geometric_harvest.py" "$@"
    ;;
  prepare-patrol)
    asset_python="${S300_ASSET_PYTHON:-$task_dir/.asset-venv/bin/python}"
    for option in "$@"; do
      if [[ "$option" == --remove-lift-braces-from* ]]; then asset_python="$python"; fi
    done
    exec env -u PYTHONPATH "$asset_python" -u "$scripts/prepare_newton_patrol.py" "$@"
    ;;
  prepare-scene-stream)
    exec env -u PYTHONPATH "$python" -u "$scripts/newton_scene_chunks.py" "$@"
    ;;
  prepare-route)
    asset_python="${S300_ASSET_PYTHON:-$task_dir/.asset-venv/bin/python}"
    exec env -u PYTHONPATH "$asset_python" -u "$scripts/prepare_static_patrol_route.py" "$@"
    ;;
  camera-patrol)
    runtime="${S300_NEWTON_RUNTIME:-/root/farmily_runtime/newton}"
    patrol_assets="${S300_NEWTON_PATROL_ASSETS:-$runtime/assets/patrol_current}"
    if (($#)) && [[ "$1" != -* ]]; then patrol_assets="$1"; shift; fi
    exec env -u PYTHONPATH "$python" -u "$scripts/run_newton_camera_patrol.py" "$patrol_assets" \
      --output "$runtime/runs/$(date +%Y%m%d_%H%M%S)_camera_patrol" "$@"
    ;;
  prepare-standalone)
    runtime="${S300_NEWTON_RUNTIME:-/root/farmily_runtime/newton}"
    output_dir="$runtime/assets/$(date +%Y%m%d_%H%M%S)_rebuilt"
    if (($#)) && [[ "$1" != -* ]]; then output_dir="$1"; shift; fi
    asset_python="${S300_ASSET_PYTHON:-$task_dir/.asset-venv/bin/python}"
    exec env -u PYTHONPATH "$asset_python" -u "$scripts/prepare_newton_standalone.py" "$output_dir" "$@"
    ;;
  standalone)
    runtime="${S300_NEWTON_RUNTIME:-/root/farmily_runtime/newton}"
    input_dir="${S300_NEWTON_INPUTS:-$runtime/assets/current/inputs}"
    if (($#)) && [[ "$1" != -* ]]; then input_dir="$1"; shift; fi
    display_options=(--gui --interactive-viewer --session)
    forwarded=()
    for option in "$@"; do
      if [[ "$option" == '--headless' ]]; then display_options=()
      elif [[ "$option" == '--no-interactive-viewer' ]]; then display_options=(--gui --no-interactive-viewer)
      else forwarded+=("$option"); fi
    done
    export S300_NEWTON_OUTPUT="${S300_NEWTON_OUTPUT:-$runtime/runs/$(date +%Y%m%d_%H%M%S)_live}"
    exec "$task_dir/run_s300_newton.sh" gpu-realtime "$input_dir" \
      --viewer-backend viser --recording summary "${display_options[@]}" "${forwarded[@]}"
    ;;
  row-realtime)
    route_dir="${S300_NEWTON_ROUTE:-$data_dir/20261018_010218_newton_same_bed_continuous/native_piece_route}"
    if (($#)) && [[ "$1" != -* ]]; then route_dir="$1"; shift; fi
    printf '%s\n' 'Newton GPU one-bed active-window profile: current target plant elastic, neighboring plant collisions retained, detached fruit remains dynamic. Changed dynamics, not equivalent to all plants elastic.' >&2
    exec env -u PYTHONPATH "$python" -u "$scripts/run_newton_row_harvest.py" \
      --prepared-root "$route_dir" --campaign "active_window_$(date +%Y%m%d_%H%M%S)" \
      --active-window --hz 220 --free-hz 60 --nconmax 256 --njmax 512 \
      --dense-hessian-capacity 128 --dense-hessian-overflow-capacity 256 \
      --solver-tolerance 1e-8 --released-gutter-diagnostic \
      --local-model-cache "${S300_NEWTON_MODEL_CACHE:-/tmp/s300_newton_model_cache}" "$@"
    ;;
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
        for option in "$@"; do
          if [[ "$option" == '--gui' ]]; then runner_options+=(--interactive-viewer); break; fi
        done
        printf '%s\n' 'Experimental Newton GPU realtime profile: supplied prepared model (default 133 DOF), conservative fruit proxies, 220/60Hz, belt-only 2mm grip/open strokes. Other monitored pairs 1mm, preserving released-fruit exceptions; original material and release coefficients retained.' >&2
      elif [[ "$mode" == 'gpu-fast' ]]; then
        printf '%s\n' 'Newton GPU-fast changed-model 400/60Hz preset: actual CUDA physics; contact-phase realtime target is not met by this preset.' >&2
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
    if [[ "${S300_NEWTON_PRINT_ARGS:-0}" == '1' ]]; then
      exec "$python" -c 'import json,sys; print(json.dumps(sys.argv[1:]))' \
        "$input_dir" --output "$output_dir" "${runner_options[@]}" "$@"
    fi
    exec env -u PYTHONPATH "$python" -u "$scripts/run_newton_harvest.py" \
      "$input_dir" --output "$output_dir" "${runner_options[@]}" "$@"
    ;;
  *)
    printf 'Unknown mode: %s\nUse %s --help.\n' "$mode" "$0" >&2
    exit 2
    ;;
esac
