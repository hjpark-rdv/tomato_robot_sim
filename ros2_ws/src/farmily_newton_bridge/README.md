# farmily_newton_bridge

Live ROS 2 Humble telemetry and optional command authority for the existing Newton GPU simulator.

Start with `nvidia-sim/run_s300_ros2.sh build`, then run
`nvidia-sim/run_s300_newton.sh camera-patrol --ros2` from the repository root.
Use `nvidia-sim/run_s300_ros2.sh inspect` in another terminal to subscribe.

Use `nvidia-sim/run_s300_newton.sh ros-control` to command joints, rail velocity,
lift and gripper through ROS topics. Automatic patrol and ROS command authority
are separate launch modes.

See [setup, topics, encodings, QoS and validation](../../../nvidia-sim/S300_NEWTON_ROS2_KO.md).
The default DDS domain is 42. This package does not start hardware drivers.
Motion subscriptions are enabled only by the explicit ROS control launch mode.
