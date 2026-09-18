#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Deactivate active virtualenv or ROS pythonpath that can shadow Isaac Sim
unset VIRTUAL_ENV
unset PYTHONHOME
export PYTHONPATH="${SCRIPT_DIR}"
export DISPLAY="${DISPLAY:-:0}"

# Locate Python interpreter with working Isaac Sim SimulationApp. Prefer the
# IsaacLab environment because its NumPy 1.26 build is compatible with Isaac
# Replicator writers (the system Python currently has incompatible NumPy 2.x).
if /root/isaaclab_env/bin/python3.11 -c "from isaacsim import SimulationApp" &>/dev/null; then
  PYTHON_EXE="/root/isaaclab_env/bin/python3.11"
elif /usr/bin/python3.11 -c "from isaacsim import SimulationApp" &>/dev/null; then
  PYTHON_EXE="/usr/bin/python3.11"
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
  echo "  --scale FACTOR      Set greenhouse scale factor (e.g. 1.0 for 100% original, 0.5 for 50%)"
  echo "  --steps N           Run simulation for N steps and exit"
  echo "  --capture PATH      Capture screenshot to image file"
  echo "  --no-ros2-camera    Disable D435 RGB-D ROS 2 topic publishers"
  echo "  --camera-width N    Camera image width (default: 640)"
  echo "  --camera-height N   Camera image height (default: 480)"
  echo "  --rebuild           Re-generate robot URDF/USD and composite greenhouse stage"
  echo "  -h, --help          Show this help message"
  echo
}

mode="gui"
steps=0
capture=""
scale=""
ros2_camera="enabled"
camera_width=""
camera_height=""

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
    --scale)
      scale="${2:-0.5}"
      shift 2
      ;;
    --steps)
      steps="${2:-0}"
      shift 2
      ;;
    --capture)
      capture="${2:-}"
      shift 2
      ;;
    --no-ros2-camera)
      ros2_camera="disabled"
      shift
      ;;
    --camera-width)
      camera_width="${2:-}"
      shift 2
      ;;
    --camera-height)
      camera_height="${2:-}"
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

if [[ -n "${scale}" ]]; then
  echo "=== Updating Composite Scene to Scale ${scale} ==="
  "${PYTHON_EXE}" "${SCRIPT_DIR}/scenes/create_composite_scene.py" --greenhouse-scale "${scale}"
fi

if [[ "${mode}" == "rebuild" ]]; then
  echo "=== Re-generating Robot USD asset ==="
  "${PYTHON_EXE}" "${SCRIPT_DIR}/scripts/generate_robot_usd.py"
  echo "=== Re-generating Composite Greenhouse + Robot Scene ==="
  "${PYTHON_EXE}" "${SCRIPT_DIR}/scenes/create_composite_scene.py" --greenhouse-scale "${scale:-0.5}"
  echo "Rebuild complete!"
  exit 0
fi

if [[ "${mode}" == "test" ]]; then
  echo "=== Running Automated Verification Test ==="
  exec "${PYTHON_EXE}" "${SCRIPT_DIR}/scripts/test_robot_spawn.py"
fi

if [[ "${ros2_camera}" == "enabled" ]]; then
  # Isaac Sim 5.1 ships a matching ROS 2 userspace. Configure its shared
  # libraries before Python starts so the bridge can load reliably.
  export ROS_DISTRO="${ROS_DISTRO:-humble}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  isaacsim_package_dir="$("${PYTHON_EXE}" -c 'import isaacsim; print(next(iter(isaacsim.__path__)))')"
  ros2_bridge_ext="${isaacsim_package_dir}/exts/isaacsim.ros2.bridge"
  if [[ -z "${ros2_bridge_ext}" || ! -d "${ros2_bridge_ext}/${ROS_DISTRO}/lib" ]]; then
    echo "[ERROR] Isaac Sim ROS 2 Bridge libraries were not found for ROS_DISTRO=${ROS_DISTRO}." >&2
    exit 1
  fi
  export LD_LIBRARY_PATH="${ros2_bridge_ext}/${ROS_DISTRO}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
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

if [[ "${ros2_camera}" == "disabled" ]]; then
  cmd+=(--no-ros2-camera)
fi

if [[ -n "${camera_width}" ]]; then
  cmd+=(--camera-width "${camera_width}")
fi

if [[ -n "${camera_height}" ]]; then
  cmd+=(--camera-height "${camera_height}")
fi

echo "=== Starting Smart Farm Robot Simulation ==="
echo "Display: ${DISPLAY}"
echo "Command: ${cmd[*]}"
echo

exec "${cmd[@]}"
