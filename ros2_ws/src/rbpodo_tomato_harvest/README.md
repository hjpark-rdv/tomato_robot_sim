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

## 카메라 검출 기반 토마토 TF 생성

MoveIt 실행 시 `tomato_tf_generator`가 함께 시작된다. 카메라 팀의 서비스는
한 번의 촬영에서 검출한 모든 토마토를 배열 메시지 하나로 발행한다. 좌표는
기본적으로 `d435_color_optical_frame` 기준이다.

```text
/tomato_detection/detections
farmily_tomato_interfaces/msg/TomatoDetectionArray
```

배열의 각 원소에는 토마토 ID, 중심점, 줄기 방향점이 들어간다. 새 배열이
들어오면 이전 `detected_tomato_*` 목록을 교체하고
`detected_tomato_0_tf`부터 다시 생성한다. 따라서 여러 번 촬영해도 TF가
누적되지 않으며, 운영 시 사용자는 카메라 팀이 제공하는 촬영 서비스만 호출한다.

카메라 서비스의 응답값은 그 서비스를 호출한 클라이언트만 받을 수 있으므로,
카메라 노드는 서비스 응답과 별개로 위 배열 토픽도 발행해야 한다. 각 토마토의
줄기 방향점은 중심과 다른 위치여야 한다.

기존 단일 중심점/줄기점 토픽과 `/tomato_tf_generator/create_tf` 서비스는
수동 디버깅 호환용으로 유지한다. 자동 생성은 launch 인자
`auto_create_detected_tomato_tf:=false`로 끌 수 있다.

### Fake 카메라 서비스

실제 카메라 서비스와 독립적으로 통신 흐름을 시험할 수 있도록 fake 카메라는
별도 launch로 제공한다. MoveIt/로봇 launch에는 포함되지 않으며 필요할 때만
다른 터미널에서 실행한다.

```bash
ros2 launch rbpodo_tomato_harvest fake_camera.launch.py
```

fake 카메라는 `tomato_0_tf`부터 `tomato_7_tf`까지 총 8개를 한 번에 검출하고
줄기 방향점으로 `main_vine_tf`를 사용한다. 다음 서비스를 호출하면 모든 TF를
카메라 좌표로 변환해 응답과 배열 토픽으로 내보내고, `tomato_tf_generator`가
각 토마토의 TF를 자동 생성한다.

```bash
ros2 service call /fake_tomato_camera/detect_tomatoes \
  farmily_tomato_interfaces/srv/DetectTomatoes '{}'
```

새 TF의 parent는 `d435_color_optical_frame`이다. 토마토 Z축은 `link0`의
수직 상향축과 일치하도록 카메라 좌표계에서 계산하고, X축은 지면에 투영한
토마토 중심→줄기 점 방향을 바라본다.

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
