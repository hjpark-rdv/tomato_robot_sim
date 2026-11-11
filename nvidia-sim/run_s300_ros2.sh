#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(dirname -- "$task_dir")"
ros_setup="${S300_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$ros_setup" ]]; then printf 'Missing ROS setup: %s\n' "$ros_setup" >&2; exit 1; fi
source "$ros_setup"
export ROS_DOMAIN_ID="${S300_ROS_DOMAIN_ID:-42}"
if [[ -f "$repo/ros2_ws/install/local_setup.bash" ]]; then source "$repo/ros2_ws/install/local_setup.bash"; fi
mode="${1:-check}"
if (($#)); then shift; fi
case "$mode" in
  build)
    cd "$repo/ros2_ws"
    exec /usr/bin/colcon build --packages-select farmily_newton_bridge --symlink-install "$@"
    ;;
  check)
    exec /usr/bin/python3 -c 'import rclpy,numpy; from sensor_msgs.msg import Image,CameraInfo,JointState; from tf2_msgs.msg import TFMessage; from rosgraph_msgs.msg import Clock; print("ROS 2 telemetry dependencies ready")'
    ;;
  bridge)
    # Source invocation also works before colcon install. No rclpy is imported
    # into the independent Python3.12 Newton process.
    export PYTHONPATH="$repo/ros2_ws/src/farmily_newton_bridge${PYTHONPATH:+:$PYTHONPATH}"
    exec /usr/bin/python3 -m farmily_newton_bridge.node "$@"
    ;;
  inspect)
    exec ros2 run farmily_newton_bridge inspect "$@"
    ;;
  *) exec ros2 "$mode" "$@" ;;
esac
