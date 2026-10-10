#!/usr/bin/env bash
# Reuse the native runner and measured-pose viewer in separate processes.
set -euo pipefail
if [[ $# -lt 1 || $# -gt 2 || "${1:-}" == --help ]]; then
  echo 'Usage: bash run_motion_live.sh ASSEMBLY [reference|interactive]'
  echo 'reference: PGS 120Hz; interactive: PGS 60Hz preview with different numerical dynamics.'
  echo 'Both retain full plant, all colliders, self-collision and the 0.5mm guard.'
  exit 0
fi
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
assembly="$(realpath "$1")"
profile="${2:-reference}"
case "$profile" in
  reference) hz=120 ;;
  interactive) hz=60 ;;
  *) echo 'Expected reference or interactive' >&2; exit 2 ;;
esac
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
echo "Physics: full plant / PGS64 / ${hz}Hz / self-collision ON; entry, hold and return only."
if [[ "$profile" == interactive ]]; then
  echo 'Interactive preview changes timestep from 120Hz to 60Hz; it is not numerically equivalent to the reference.'
fi
printf '%s\n' "$profile" > "$output/profile.txt"
cp -- "${BASH_SOURCE[0]}" "$output/launcher.sh"
cp -- "$task_dir/VIDEO_ENTRY_STATUS_KO.md" "$output/MOTION_STATUS_KO.md"
setsid env DISPLAY="$viewer_display" OMNI_KIT_ALLOW_ROOT=1 \
  "$native_python" --no-ros-env -u "$task_dir/run_motion.py" "$assembly" \
  --output "$output/physics" --mode harvest --num-envs 1 \
  --preparation horizontal_ready --entry-only --approach-pitch-deg 0 --approach-yaw-offset-deg 30 \
  --headless --physics-only --device cpu --hz "$hz" --threads 2 --performance optimized \
  --gravity-neutral-warmup --armature-setup authored --buffer-contact-events --record-contact-frames \
  --contact-backend callback --collision-filter pairs --plant-dynamics full \
  --solver PGS --position-iterations 64 --velocity-iterations 4 \
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
