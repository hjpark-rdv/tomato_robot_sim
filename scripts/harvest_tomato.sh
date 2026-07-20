#!/usr/bin/env bash
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ws_dir="${repo_dir}/ros2_ws"

usage() {
  echo "Usage: $0 TOMATO_NUMBER [--plan-only]" >&2
  echo "       $0                 # prompt for tomato number" >&2
}

if (( $# > 2 )); then
  usage
  exit 2
fi

tomato_number="${1:-}"
mode="${2:-}"

if [[ -z "${tomato_number}" ]]; then
  if [[ ! -t 0 ]]; then
    usage
    exit 2
  fi
  read -r -p "Tomato number [0-7]: " tomato_number
fi

if [[ ! "${tomato_number}" =~ ^[0-7]$ ]]; then
  echo "Invalid tomato number: ${tomato_number:-<empty>} (expected 0-7)" >&2
  exit 2
fi

if [[ -n "${mode}" && "${mode}" != "--plan-only" ]]; then
  usage
  exit 2
fi

source /opt/ros/humble/setup.bash
if [[ ! -f "${ws_dir}/install/setup.bash" ]]; then
  echo "Workspace is not built: ${ws_dir}/install/setup.bash not found" >&2
  exit 1
fi
source "${ws_dir}/install/setup.bash"
set -u

execute="true"
if [[ "${mode}" == "--plan-only" ]]; then
  execute="false"
fi

echo "Tomato ${tomato_number}: PICK_READY -> Cartesian approach (execute=${execute})"

exec ros2 run rbpodo_tomato_harvest tomato_harvest_test --ros-args \
  -p tomato_frame:="tomato_${tomato_number}_tf" \
  -p execute:="${execute}"
