# RBPoDo Tomato Harvest

This directory is the workspace root for the new Rainbow Robotics tomato
harvesting robot and gripper integration.

The upstream robot driver and descriptions are kept separately in
`../rbpodo_ros2` as a Git submodule. New robot, gripper, MoveIt, launch, and
harvesting code must be added below this directory instead of modifying
`tomato_moveit_teach` or the upstream submodule.

Planned package split:

- `rbpodo_tomato_description`: composed robot and gripper Xacro/meshes
- `rbpodo_tomato_moveit_config`: SRDF, kinematics, limits, and planners
- `rbpodo_tomato_bringup`: fake-hardware and real-hardware launch files
- `rbpodo_tomato_harvest`: tomato scene and harvesting behavior

The exact packages will be created after the robot `model_id` and gripper
interface/geometry are selected.

## Cartesian harvest test

From the repository root, select a tomato and start the real robot motion with
one command:

```bash
./scripts/harvest_tomato.sh 3
```

Omit the number to select it interactively, or append `--plan-only` to publish
the complete path in RViz without moving the robot:

```bash
./scripts/harvest_tomato.sh
./scripts/harvest_tomato.sh 3 --plan-only
```

With the RB5 MoveIt launch already running, plan a level Cartesian approach to
`tomato_3_tf` without moving the robot:

```bash
ros2 run rbpodo_tomato_harvest tomato_harvest_test
```

Every harvest starts from the `PICK_READY` named state captured from the real
robot. In plan-only mode RViz receives both trajectories in order:

1. Current joint state to `PICK_READY`
2. `PICK_READY` to the pre-approach point and tomato

The target keeps the tomato between `tomato_gripper_tip` and `main_vine_tf`,
places the tip 18 mm below the tomato center, preserves the current horizontal
approach direction, and corrects the gripper roll to its -90 degree ground-level
orientation. The Cartesian path first retracts 40 mm to a pre-approach point and
then advances to the tomato. Inspect the published path in RViz before explicitly
enabling execution:

```bash
ros2 run rbpodo_tomato_harvest tomato_harvest_test --ros-args -p execute:=true
```

Select another tomato or tune the offsets with ROS parameters, for example:

```bash
ros2 run rbpodo_tomato_harvest tomato_harvest_test --ros-args \
  -p tomato_frame:=tomato_5_tf \
  -p tip_standoff:=0.025 \
  -p tip_below_center:=0.018
```

## RB5 MoveIt quick start

From the repository root, run the setup once after a fresh clone:

```bash
./setup_rbpodo_rb5_moveit.sh
```

Then launch the `rb5_850e` fake-hardware MoveIt environment:

```bash
DISPLAY=:0 ./run_rbpodo_rb5_moveit.sh
```
