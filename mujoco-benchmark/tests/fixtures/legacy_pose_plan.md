# Frozen planner regression

`legacy_pose_plan.npz` was generated from `f24709c:nvidia-sim/rl/dataset_motion.py` using the deterministic kinematics/environment in `test_diagnostic_pose_planner.py` and `old_waypoints`. It contains607 commands and phase labels. The test compares arrays exactly. This small fixture validates legacy scheduling and interpolation; the separate server evidence also compares2,512 real planned commands against the original saved trace.
