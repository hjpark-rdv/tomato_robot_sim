#!/usr/bin/env bash
# Reuse the native runner and measured-pose viewer in separate processes.
set -euo pipefail
if [[ $# -lt 1 || $# -gt 3 || "${1:-}" == --help ]]; then
  echo 'Usage: bash run_motion_live.sh ASSEMBLY [reference|interactive|240|480] [entry|belt|cycle|truss]'
  echo 'reference: PGS 120Hz; interactive: PGS 60Hz preview with different numerical dynamics.'
  echo 'entry: open-jaw entry/return; belt: close, opposed belts for 2s, open, withdraw (detachment not guaranteed).'
  echo 'cycle: start belts while open, close, fully reopen and close again with continuous belts; then stop/open/withdraw.'
  echo 'truss: repeat cycle on remaining measured fruit in one physical scene without resets.'
  echo 'Full plant, all colliders and self-collision remain. Contact guard defaults to 1mm.'
  echo 'Explicit variants: S300_SOLVER, S300_POSITION_ITERATIONS, S300_PHYSICS_THREADS, S300_PLANT_DYNAMICS, S300_CONTACT_LIMIT_MM, S300_FRUIT_CONTACT_LIMIT_MM, S300_FRUIT_FRUIT_LIMIT_MM.'
  echo 'Grasp motion: S300_GRASP_GAP_MM (default 28), S300_GRASP_HEIGHT_OFFSET_MM (default 0).'
  echo 'Yaw: S300_APPROACH_YAW_MODE=truss-axis aligns horizontal entry with attached-fruit XY direction; default robot-radial preserves prior motion.'
  echo 'Whole-truss budget: S300_MAX_TRUSS_CYCLES (default 12); all native contact guards remain.'
  exit 0
fi
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
assembly="$(realpath "$1")"
profile="${2:-reference}"
motion="${3:-entry}"
case "$motion" in
  entry) motion_args=(--entry-only) ;;
  belt|cycle|truss)
        harvest_profile=belt_spin_release
        [[ "$motion" == cycle || "$motion" == truss ]] && harvest_profile=belt_open_close_cycle
        motion_args=(--harvest-profile "$harvest_profile" --belt-speed-m-s "${S300_BELT_SPEED_M_S:-.15}" --belt-seconds 2
                    --close-seconds "${S300_CLOSE_SECONDS:-3}" --belt-ramp-seconds "${S300_BELT_RAMP_SECONDS:-0}"
                    --belt-infeed-speed-m-s "${S300_BELT_INFEED_M_S:-0}"
                    --grasp-gap-mm "${S300_GRASP_GAP_MM:-28}"
                    --grasp-depth-mm "${S300_GRASP_DEPTH_MM:-65}"
                    --grasp-height-offset-mm "${S300_GRASP_HEIGHT_OFFSET_MM:-0}") ;;
  *) echo 'Expected entry, belt, cycle, or truss motion' >&2; exit 2 ;;
esac
if [[ "$motion" == truss ]]; then motion_args+=(--whole-truss --max-truss-cycles "${S300_MAX_TRUSS_CYCLES:-12}" --truss-order "${S300_TRUSS_ORDER:-lowest-first}"); fi
if [[ "${S300_PREPARATION_HEIGHT_FIRST:-0}" == 1 ]]; then motion_args+=(--preparation-height-first); fi
if [[ "${S300_VERIFY_LEAF_CONTACT_AFTER_STEP:-0}" == 1 ]]; then motion_args+=(--verify-leaf-contact-after-step); fi
case "$profile" in
  reference) hz=120 ;;
  interactive) hz=60 ;;
  240|480) hz="$profile" ;;
  *) echo 'Expected reference, interactive, 240, or 480' >&2; exit 2 ;;
esac
if [[ "${S300_SCENE_CCD:-0}" == 1 ]]; then
  motion_args+=(--scene-ccd)
fi
solver="${S300_SOLVER:-PGS}"
iterations="${S300_POSITION_ITERATIONS:-64}"
physics_threads="${S300_PHYSICS_THREADS:-2}"
plant_dynamics="${S300_PLANT_DYNAMICS:-full}"
contact_limit="${S300_CONTACT_LIMIT_MM:-1}"
fruit_contact_limit="${S300_FRUIT_CONTACT_LIMIT_MM:-$contact_limit}"
fruit_fruit_limit="${S300_FRUIT_FRUIT_LIMIT_MM:-$contact_limit}"
motion_args+=(--contact-limit-mm "$contact_limit" --fruit-contact-limit-mm "$fruit_contact_limit" --fruit-fruit-limit-mm "$fruit_fruit_limit")
yaw_mode="${S300_APPROACH_YAW_MODE:-robot-radial}"
yaw_offset=30
if [[ "$yaw_mode" == truss-axis ]]; then yaw_offset=0; fi
motion_args+=(--approach-yaw-mode "$yaw_mode")
native_python="${S300_ISAAC_PYTHON:-/root/isaac-sim-6.1.0/python.sh}"
viewer_display="${S300_DISPLAY:-:0}"
asset_python="$task_dir/../.asset-venv/bin/python"
output="${S300_MOTION_OUTPUT:-/root/docker_share/mujoko_debugging_data/$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_s300_${profile}_live}"
if [[ ! -f "$assembly/scene.usda" || ! -x "$native_python" ]]; then
  echo 'Assembly or native Isaac Python is missing' >&2; exit 2
fi
mkdir -- "$output"  # Refuse to replace an earlier run.
output="$(realpath "$output")"
producer_pid=''; viewer_pid=''
cleanup() {
  for task_pid in "$producer_pid" "$viewer_pid"; do
    if [[ -n "$task_pid" ]] && kill -0 "$task_pid" 2>/dev/null; then
      kill -TERM -- "-$task_pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait_file() {
  local path="$1" task_pid="$2" i
  for ((i=0;i<1500;i++)); do
    [[ -f "$path" ]] && return 0
    kill -0 "$task_pid" 2>/dev/null || return 1
    sleep .2
  done
  echo "Timed out waiting for $path" >&2
  return 1
}
echo "Output: $output"
echo "Physics: ${plant_dynamics} / ${solver}${iterations} / ${hz}Hz / self-collision ON; motion: ${motion}."
echo "Contact limits: current-truss fruit/tool ${fruit_contact_limit}mm, same-truss fruit/fruit ${fruit_fruit_limit}mm, all other active pairs ${contact_limit}mm."
if [[ "$profile" == interactive ]]; then
  echo 'Interactive preview changes timestep from 120Hz to 60Hz; it is not numerically equivalent to the reference.'
fi
printf '%s\n' "$profile" > "$output/profile.txt"
cp -- "${BASH_SOURCE[0]}" "$output/launcher.sh"
motion_status="$task_dir/VIDEO_ENTRY_STATUS_KO.md"
if [[ "$motion" == cycle || "$motion" == truss ]]; then
  motion_status="$task_dir/CONTROLLED_BELT_HARVEST_KO.md"
fi
cp -- "$motion_status" "$output/MOTION_STATUS_KO.md"
setsid env DISPLAY="$viewer_display" OMNI_KIT_ALLOW_ROOT=1 \
  "$native_python" --no-ros-env -u "$task_dir/run_motion.py" "$assembly" \
  --output "$output/physics" --mode harvest --num-envs 1 \
  --preparation horizontal_ready "${motion_args[@]}" --approach-pitch-deg 0 --approach-yaw-offset-deg "$yaw_offset" \
  --headless --physics-only --device cpu --hz "$hz" --threads "$physics_threads" --performance optimized \
  --gravity-neutral-warmup --armature-setup authored --buffer-contact-events --record-contact-frames \
  --contact-backend callback --collision-filter pairs --plant-dynamics "$plant_dynamics" \
  --solver "$solver" --position-iterations "$iterations" --velocity-iterations 4 \
  --publish-live --live-fps 20 --realtime --start-gate "$output/start_gate.json" --gate-timeout 300 \
  > "$output/physics.log" 2>&1 &
producer_pid=$!
echo "Preparing physics and checking the full path; log: $output/physics.log"
if ! wait_file "$output/physics/live.json" "$producer_pid"; then
  tail -n 35 "$output/physics.log" >&2; exit 1
fi
capture_args=()
if [[ "${S300_CAPTURE_FRAME:-0}" == 1 ]]; then
  capture_args=(--capture "$output/final_measured_view.png")
fi
setsid env DISPLAY="$viewer_display" OMNI_KIT_ALLOW_ROOT=1 \
  "$native_python" --no-ros-env -u "$task_dir/view_motion_live.py" "$output/physics" \
  --camera entry --fps 15 --seconds "${S300_VIEW_SECONDS:-0}" \
  --ready-file "$output/viewer_ready.json" "${capture_args[@]}" > "$output/viewer.log" 2>&1 &
viewer_pid=$!
echo "Opening measured-state viewer; log: $output/viewer.log"
if ! wait_file "$output/viewer_ready.json" "$viewer_pid"; then
  tail -n 35 "$output/viewer.log" >&2; exit 1
fi
printf '{"released_by":"run_motion_live.sh","display_ready":true}\n' > "$output/start_gate.json"
echo 'Viewer ready. Starting actual physics; the viewer displays measured states and their realtime factor.'
status=0
wait "$producer_pid" || status=$?
producer_pid=''
if [[ "$motion" != entry && -f "$output/physics/summary.json" ]]; then
  "$asset_python" "$task_dir/analyze_belt_trial.py" "$assembly" "$output/physics" > "$output/belt_analysis.log"
fi
"$asset_python" "$task_dir/report_motion.py" "$output"
if [[ -f "$output/physics/summary.json" ]]; then
  "$asset_python" - "$output/physics/summary.json" <<'PY'
import json,sys
s=json.load(open(sys.argv[1]))
print('Completed:',s['completed'],'Stop:',s['stop_reason'])
print('Simulation: %.3fs; wall: %.3fs; speed: %.3fx' % (s['simulated_s'],s['physics_wall_s'],s['simulated_s']/max(s['physics_wall_s'],1e-9)))
PY
fi
echo "Report: $output/index.html"
echo 'Physics and state validation finished. Close the viewer window to exit.'
wait "$viewer_pid" || status=$?
viewer_pid=''
exit "$status"
