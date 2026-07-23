#!/usr/bin/env bash
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="${repo_dir}/../ros2_ws"
state_dir="${XDG_STATE_HOME:-${HOME}/.local/state}/farmily_tomato"
last_ip_file="${state_dir}/last_robot_ip"
default_robot_ip="192.168.222.196"

if [[ -s "${last_ip_file}" ]]; then
  default_robot_ip="$(<"${last_ip_file}")"
fi

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
  robot_ip:="${robot_ip}" \
  use_fake_hardware:=false \
  fake_sensor_commands:=false \
  cb_simulation:=Real \
  activate_arm_controller:=true \
  "$@"
