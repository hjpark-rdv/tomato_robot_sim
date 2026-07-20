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

## RB5 MoveIt quick start

From the repository root, run the setup once after a fresh clone:

```bash
./setup_rbpodo_rb5_moveit.sh
```

Then launch the `rb5_850e` fake-hardware MoveIt environment:

```bash
DISPLAY=:0 ./run_rbpodo_rb5_moveit.sh
```
