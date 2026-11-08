#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_isaac="${FARMILY_ISAAC_61_ROOT:-/root/isaac-sim-6.1.0}"
if [[ "${1:-}" == '--help' || "${1:-}" == '-h' ]]; then
  printf '%s\n' \
    'Usage: run_s300_harvest.sh [real-lift] [viewer options]' \
    '       run_s300_harvest.sh lift-test [motion options]' \
    '       run_s300_harvest.sh rgbd-test [motion options]' \
    '       run_s300_harvest.sh rgbd-gt [motion options]' \
    '       run_s300_harvest.sh camera-patrol [patrol options]' \
    '       run_s300_harvest.sh SCENE.usda --output OUTPUT [legacy viewer options]' \
    'Default: open the full farm with the side-mounted RB5 and real lift carriage.' \
    'lift-test: actual Isaac/PhysX raise, lower and return; GUI waits for START, with speed capped at 1x.' \
    'lift-test --headless --physics-only: bounded native check without rendering.' \
    'rgbd-test: same lift motion, portrait mounted RGB + metric depth saved at 2Hz.' \
    'rgbd-gt: RGB-D plus visible SIM-GT fruit/stem/leaf masks and recognition/depth overlays.' \
    'camera-patrol: actual camera scan, <=15cm forward steps, visible-truss queue and six distinct native harvest goal.' \
    'Camera patrol also publishes verified saved states to the harvest-replay history; S300_PUBLISH_REPLAY=0 disables publishing.' \
    'Override the corrected assembly/output using S300_ISAAC_ASSEMBLY and S300_ISAAC_OUTPUT.'
  exit 0
fi
if [[ ! -x "$task_isaac/python.sh" ]]; then
  echo "Isaac Sim 6.1 python.sh를 찾지 못했습니다: $task_isaac" >&2
  exit 1
fi
export DISPLAY="${DISPLAY:-:0}"
export OMNI_KIT_ALLOW_ROOT=1
unset PYTHONHOME VIRTUAL_ENV PYTHONPATH
if [[ "${1:-}" == 'camera-patrol' ]]; then
  shift
  task_assembly="${S300_ISAAC_ASSEMBLY:-/root/docker_share/mujoko_debugging_data/20261023_side_mount_camera_patrol/assembly_side_entry}"
  task_inventory="${S300_PATROL_INVENTORY:-/root/docker_share/mujoko_debugging_data/20261023_side_mount_camera_patrol/full_farm_inventory.json}"
  task_output="${S300_ISAAC_OUTPUT:-/root/docker_share/mujoko_debugging_data/$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_side_camera_patrol}"
  task_patrol_help=0
  for task_arg in "$@"; do
    if [[ "$task_arg" == '--help' || "$task_arg" == '-h' ]]; then task_patrol_help=1; fi
  done
  if [[ "$task_patrol_help" == '0' ]]; then
    if [[ -e "$task_output" ]]; then
      printf '기존 실행 결과를 덮어쓰지 않습니다: %s\n' "$task_output" >&2
      exit 1
    fi
    if [[ ! -f "$task_assembly/motion_manifest.json" || ! -f "$task_inventory" ]]; then
      printf '카메라 순회 assembly 또는 inventory를 찾지 못했습니다.\n' >&2
      exit 1
    fi
  fi
  if [[ "$task_patrol_help" == '0' && "${S300_PUBLISH_REPLAY:-1}" == '1' ]]; then
    task_collection="${S300_REPLAY_COLLECTION:-camera_patrol_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)}"
    mkdir -p -- "$(dirname -- "$task_output")"
    task_publish_log="${task_output}_web.log"
    "$task_dir/.asset-venv/bin/python" -u "$task_dir/../gripper-design/harvest-replay/publish_patrol.py" \
      --campaign "$task_output" --collection "$task_collection" --watch > "$task_publish_log" 2>&1 &
    printf 'Web replay publisher PID %s · log: %s\n' "$!" "$task_publish_log"
  fi
  exec "$task_dir/.asset-venv/bin/python" -u "$task_dir/harvest_s300/run_row_harvest.py" \
    "$task_assembly" "$task_inventory" - "$task_output" --camera-patrol --execute \
    --motion-arguments "$task_dir/harvest_s300/side_camera_patrol_arguments.json" \
    --python "$task_isaac/python.sh" --patrol-goal-trusses 6 --patrol-step-m .15 "$@"
fi
if [[ $# -eq 0 || "${1:-}" == 'real-lift' || "${1:-}" == 'lift-test' || "${1:-}" == 'rgbd-test' || "${1:-}" == 'rgbd-gt' ]]; then
  task_mode="${1:-real-lift}"
  if (($#)); then shift; fi
  task_assembly="${S300_ISAAC_ASSEMBLY:-/root/docker_share/mujoko_debugging_data/20261023_side_mount_camera_patrol/assembly_side_entry}"
  task_output="${S300_ISAAC_OUTPUT:-/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_isaac_${task_mode}}"
  if [[ ! -f "$task_assembly/composed.usdc" || ! -f "$task_assembly/real_lift_mount.json" ]]; then
    printf '수정된 리프트 장면이 없습니다: %s\nS300_ISAAC_ASSEMBLY로 새 assembly 경로를 지정하세요.\n' "$task_assembly" >&2
    exit 1
  fi
  if [[ "$task_mode" == 'lift-test' || "$task_mode" == 'rgbd-test' || "$task_mode" == 'rgbd-gt' ]]; then
    task_view=(--wait-for-start --keep-open --realtime)
    task_sensor=()
    if [[ "$task_mode" == 'rgbd-test' ]]; then task_sensor=(--capture-rgbd); fi
    if [[ "$task_mode" == 'rgbd-gt' ]]; then task_sensor=(--capture-gt); fi
    for task_arg in "$@"; do
      if [[ "$task_arg" == '--headless' ]]; then task_view=(); fi
    done
    exec "$task_isaac/python.sh" --no-ros-env -u "$task_dir/harvest_s300/run_motion.py" \
      "$task_assembly" --mode lift --output "$task_output" --hz 120 --solver TGS \
      --threads 16 --native-materialized-stage --gravity-neutral-warmup \
      --buffer-contact-events "${task_view[@]}" "${task_sensor[@]}" "$@"
  fi
  exec "$task_isaac/python.sh" --no-ros-env -u "$task_dir/harvest_s300/run_scene.py" \
    "$task_assembly/composed.usdc" --output "$task_output" "$@"
fi
exec "$task_isaac/python.sh" --no-ros-env -u "$task_dir/harvest_s300/run_scene.py" "$@"
