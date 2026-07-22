#!/usr/bin/env bash
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ws_dir="${repo_dir}/ros2_ws"

source /opt/ros/humble/setup.bash
if [[ ! -f "${ws_dir}/install/setup.bash" ]]; then
  echo "Workspace is not built: ${ws_dir}/install/setup.bash not found" >&2
  exit 1
fi
source "${ws_dir}/install/setup.bash"

exec ros2 launch rbpodo_tomato_harvest harvest_gui.launch.py "$@"
