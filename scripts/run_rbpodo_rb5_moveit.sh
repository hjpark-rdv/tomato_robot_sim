#!/usr/bin/env bash
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="${repo_dir}/../ros2_ws"

source /opt/ros/humble/setup.bash
source "${ws_dir}/install/setup.bash"

# ROS Humble in this image is built for Python 3.10; the system default is
# Python 3.11.
export DISPLAY="${DISPLAY:-:0}"

# moveit.launch.py already starts both controller spawners. Starting them a
# second time here races the launch spawners and fails once they are active.
exec /usr/bin/python3.10 /opt/ros/humble/bin/ros2 launch \
  rbpodo_moveit_config moveit.launch.py \
  model_id:=rb5_farmily \
  use_fake_hardware:=true \
  fake_sensor_commands:=true \
  cb_simulation:=Simulation
