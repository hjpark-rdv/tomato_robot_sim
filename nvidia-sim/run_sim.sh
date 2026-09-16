#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Deactivate active virtualenv or ROS pythonpath that can shadow Isaac Sim
unset VIRTUAL_ENV
unset PYTHONHOME
export PYTHONPATH="${SCRIPT_DIR}"
export DISPLAY="${DISPLAY:-:0}"

# Locate Python interpreter with working Isaac Sim SimulationApp
if /usr/bin/python3.11 -c "from isaacsim import SimulationApp" &>/dev/null; then
  PYTHON_EXE="/usr/bin/python3.11"
elif /root/isaaclab_env/bin/python3.11 -c "from isaacsim import SimulationApp" &>/dev/null; then
  PYTHON_EXE="/root/isaaclab_env/bin/python3.11"
elif python3.11 -c "from isaacsim import SimulationApp" &>/dev/null; then
  PYTHON_EXE="$(which python3.11)"
else
  echo "[ERROR] Could not find a Python interpreter with NVIDIA Isaac Sim installed." >&2
  exit 1
fi

usage() {
  echo "Usage: $0 [OPTIONS]"
  echo
  echo "Options:"
  echo "  (no args)           Run simulation in interactive GUI mode (using DISPLAY=${DISPLAY})"
  echo "  --headless          Run simulation in headless mode (no GUI)"
  echo "  --test              Run automated verification test suite"
  echo "  --steps N           Run simulation for N steps and exit"
  echo "  --capture PATH      Capture screenshot to image file"
  echo "  --rebuild           Re-generate robot URDF/USD and composite greenhouse stage"
  echo "  -h, --help          Show this help message"
  echo
}

mode="gui"
steps=0
capture=""

while (( $# > 0 )); do
  case "$1" in
    --headless)
      mode="headless"
      shift
      ;;
    --test)
      mode="test"
      shift
      ;;
    --rebuild)
      mode="rebuild"
      shift
      ;;
    --steps)
      steps="${2:-0}"
      shift 2
      ;;
    --capture)
      capture="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ "${mode}" == "rebuild" ]]; then
  echo "=== Re-generating Robot USD asset ==="
  "${PYTHON_EXE}" "${SCRIPT_DIR}/scripts/generate_robot_usd.py"
  echo "=== Re-generating Composite Greenhouse + Robot Scene ==="
  "${PYTHON_EXE}" "${SCRIPT_DIR}/scenes/create_composite_scene.py"
  echo "Rebuild complete!"
  exit 0
fi

if [[ "${mode}" == "test" ]]; then
  echo "=== Running Automated Verification Test ==="
  exec "${PYTHON_EXE}" "${SCRIPT_DIR}/scripts/test_robot_spawn.py"
fi

cmd=("${PYTHON_EXE}" "${SCRIPT_DIR}/scripts/run_farm_simulation.py")

if [[ "${mode}" == "headless" ]]; then
  cmd+=(--headless)
fi

if (( steps > 0 )); then
  cmd+=(--steps "${steps}")
fi

if [[ -n "${capture}" ]]; then
  cmd+=(--capture-frame "${capture}")
fi

echo "=== Starting Smart Farm Robot Simulation ==="
echo "Display: ${DISPLAY}"
echo "Command: ${cmd[*]}"
echo

exec "${cmd[@]}"

