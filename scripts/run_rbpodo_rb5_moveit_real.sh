#!/usr/bin/env bash
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="${repo_dir}/../ros2_ws"
state_dir="${XDG_STATE_HOME:-${HOME}/.local/state}/farmily_tomato"
last_ip_file="${state_dir}/last_robot_ip"
default_robot_ip="192.168.99.196"

if [[ -s "${last_ip_file}" ]]; then
  default_robot_ip="$(<"${last_ip_file}")"
fi

# Robot workspace collision dimensions in metres, fixed in world.
# Keep these values aligned with run_rbpodo_rb5_moveit.sh so simulation and
# real-hardware planning use the same guard geometry.
wall_enabled=false
wall_left_x="-0.45"
wall_right_x="0.45"
wall_thickness="0.01"
wall_width="1.20"
wall_height="1.15"
ceiling_z="0.80"
ceiling_thickness="0.01"
ceiling_width="1.24"
ceiling_depth="1.20"

# The pedestal is centered below link0. Its top surface is at pedestal_top_z.
pedestal_size_x="0.60"
pedestal_size_y="0.60"
pedestal_height="0.01"
pedestal_top_z="-0.01"

launch_args=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_rbpodo_rb5_moveit_real.sh [--wall|--no-wall] [ROS launch arguments...]

Options:
  --wall       Enable the robot guard walls and base pedestal.
  --no-wall    Disable the robot guard walls and base pedestal (default).
  -h, --help   Show this help message.

Examples:
  ./scripts/run_rbpodo_rb5_moveit_real.sh --wall
  ./scripts/run_rbpodo_rb5_moveit_real.sh --no-wall
  ./scripts/run_rbpodo_rb5_moveit_real.sh --wall workspace_wall_height:=1.20
EOF
}

while (($#)); do
  case "$1" in
    --wall)
      wall_enabled=true
      ;;
    --no-wall)
      wall_enabled=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      launch_args+=("$1")
      ;;
  esac
  shift
done

source /opt/ros/humble/setup.bash
source "${ws_dir}/install/setup.bash"
set -u

export DISPLAY="${DISPLAY:-:0}"

if [[ ! -t 0 ]]; then
  echo "Refusing to start real hardware without an interactive terminal." >&2
  exit 2
fi

echo "REAL ROBOT MODE"
echo "  Robot: rb5_farmily"
echo
echo "Before continuing, clear the workspace, reduce the robot speed,"
echo "and keep the emergency stop within reach."
read -r -p "Robot IP [${default_robot_ip}]: " robot_ip
robot_ip="${robot_ip:-${default_robot_ip}}"

if [[ ! "${robot_ip}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
  echo "Invalid IPv4 address: ${robot_ip}" >&2
  exit 1
fi

for port in 5000 5001; do
  if ! timeout 3 bash -c 'exec 3<>"/dev/tcp/${1}/${2}"' _ "${robot_ip}" "${port}"; then
    echo "Cannot reach ${robot_ip}:${port}. Check power, cabling, and network settings." >&2
    exit 1
  fi
done

mkdir -p "${state_dir}"
printf '%s\n' "${robot_ip}" > "${last_ip_file}"
echo "Connected to ${robot_ip}; saved as the default for the next run."

exec /usr/bin/python3.10 /opt/ros/humble/bin/ros2 launch \
  rbpodo_moveit_config moveit.launch.py \
  model_id:=rb5_farmily \
  show_tomato_gripper:=true \
  robot_ip:="${robot_ip}" \
  use_fake_hardware:=false \
  fake_sensor_commands:=false \
  cb_simulation:=Real \
  activate_arm_controller:=true \
  publish_workspace_collisions:="${wall_enabled}" \
  workspace_left_wall_x:="${wall_left_x}" \
  workspace_right_wall_x:="${wall_right_x}" \
  workspace_wall_thickness:="${wall_thickness}" \
  workspace_wall_width:="${wall_width}" \
  workspace_wall_height:="${wall_height}" \
  workspace_ceiling_z:="${ceiling_z}" \
  workspace_ceiling_thickness:="${ceiling_thickness}" \
  workspace_ceiling_width:="${ceiling_width}" \
  workspace_ceiling_depth:="${ceiling_depth}" \
  robot_pedestal_size_x:="${pedestal_size_x}" \
  robot_pedestal_size_y:="${pedestal_size_y}" \
  robot_pedestal_height:="${pedestal_height}" \
  robot_pedestal_top_z:="${pedestal_top_z}" \
  "${launch_args[@]}"
