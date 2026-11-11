# farmily_newton_bridge

Live, read-only ROS 2 Humble telemetry for the existing Newton GPU camera patrol.

Start with `nvidia-sim/run_s300_ros2.sh build`, then run
`nvidia-sim/run_s300_newton.sh camera-patrol --ros2` from the repository root.
Use `nvidia-sim/run_s300_ros2.sh inspect` in another terminal to subscribe.

See [setup, topics, encodings, QoS and validation](../../../nvidia-sim/S300_NEWTON_ROS2_KO.md).
The default DDS domain is 42. This package does not start hardware drivers or
subscribe to motion commands.
