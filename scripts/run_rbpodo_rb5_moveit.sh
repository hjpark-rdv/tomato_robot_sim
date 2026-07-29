#!/usr/bin/env bash

set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="${repo_dir}/../ros2_ws"

# Robot workspace collision dimensions in metres, fixed in world.
# The vertical walls start at z=-0.05 and meet the bottom of the ceiling.
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
Usage: ./scripts/run_rbpodo_rb5_moveit.sh [--wall|--no-wall] [ROS launch arguments...]

Options:
  --wall       Enable the robot guard walls and base pedestal (default).
  --no-wall    Disable the robot guard walls and base pedestal.
  -h, --help   Show this help message.

Examples:
  ./scripts/run_rbpodo_rb5_moveit.sh --wall
  ./scripts/run_rbpodo_rb5_moveit.sh --no-wall
  ./scripts/run_rbpodo_rb5_moveit.sh --wall workspace_wall_height:=1.20
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
  cb_simulation:=Simulation \
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
