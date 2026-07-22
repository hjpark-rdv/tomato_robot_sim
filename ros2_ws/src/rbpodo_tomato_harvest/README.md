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

검출 좌표의 입력 기준은 `d435_color_optical_frame`이지만, 새 TF를 생성할
때 좌표를 로봇 베이스 `link0`로 변환하여 parent로 사용한다. 따라서 검출 후
로봇 관절과 카메라가 움직여도 토마토 TF는 움직이지 않는다. 토마토 Z축은
`link0`의 수직 상향축과 일치하고, X축은 지면에 투영한 토마토 중심→줄기 점
방향을 바라본다.

## Cartesian harvest test

### 수확 작업 GUI

MoveIt과 fake/real 카메라 서비스를 각각 실행한 뒤, 별도 터미널에서 수확 GUI를
실행한다.

```bash
./scripts/run_harvest_gui.sh
```

동일한 GUI를 ROS launch 명령으로 직접 실행해도 된다.

```bash
ros2 launch rbpodo_tomato_harvest harvest_gui.launch.py
```

GUI에서는 다음 작업을 키보드 명령 없이 수행할 수 있다.

1. 카메라 서비스 호출 및 모든 검출 토마토 목록 확인
2. 수확할 `detected_tomato_N_tf` 선택
3. RViz에서 전체 궤적을 확인하는 Plan-only 실행
4. Plan-only 성공 후 실제 수확 모션 실행
5. 메인 토마토 줄기의 X/Y/Z 위치와 줄기 축 기준 회전 각도 조회 및 변경
6. PICK_READY 진입과 복귀 구간에 사용할 OMPL/CHOMP/PILZ LIN planner 선택

안전을 위해 새 검출 결과가 들어오거나 줄기 위치/회전을 변경하면 기존
Plan-only 성공 상태는 취소된다. 회전값은 `tomato_z_spin_deg`에 도 단위로
적용되어 메인 줄기 축을 기준으로 가지와 토마토 전체를 회전시킨다. 실제 수확
버튼은 현재 검출 결과에서 선택한 토마토의 Plan-only가 정상 완료된 경우에만
활성화된다. 줄기 위치나 회전을 바꾼 다음에는 GUI의 `토마토 촬영 / 검출`
버튼을 다시 눌러 TF를 갱신해야 한다.

토마토 장면의 기본 위치는 `[0.355, -0.375, 0.340]`, 메인 줄기 축 기준 회전은
`45°`이다. GUI에서 planner를 바꾸면 기존 Plan-only 성공 상태가 취소되며,
선택한 planner로 Plan-only를 다시 완료해야 실제 수확 버튼이 활성화된다.
PILZ LIN은 TCP 직선 이동 중 연속 IK가 존재해야 하므로 토마토 위치와 자세에
따라 `NO_IK_SOLUTION`으로 실패할 수 있다.

실제 카메라 팀의 서비스 이름이 다른 경우 launch 인자로 연결한다.

```bash
./scripts/run_harvest_gui.sh \
  camera_service:=/real_camera/detect_tomatoes
```

From the repository root, select a tomato and start the real robot motion with
one command. Run the camera detection service first so that the corresponding
`detected_tomato_N_tf` exists:

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
`detected_tomato_3_tf` without moving the robot:

```bash
ros2 run rbpodo_tomato_harvest tomato_harvest_test
```

Every harvest starts from the `PICK_READY` named state captured from the real
robot. In plan-only mode RViz receives the complete trajectory sequence in
order:

1. Current joint state to `PICK_READY`
2. Cartesian TCP position/orientation motion directly to the pre-approach pose
3. Cartesian motion from pre-approach through the tomato harvest sequence
4. Cartesian post-wait retreat
5. Selected planning pipeline return to `PICK_READY`

The planner selected in the GUI is used to reach `PICK_READY` and return to it.
From `PICK_READY` through the harvest and retreat, motion is constrained to TCP
Cartesian planning. The harvest target and local offsets are still defined at
`tomato_gripper_tip`, but every target pose is converted through the fixed
TCP-to-tip transform before MoveIt plans with `tcp` as its controlled link.
This keeps the physical tip goal unchanged while allowing wrist alignment to be
planned together with TCP translation instead of a separate in-place rotation
or a large artificial straight-line movement of the offset tip link.

The detected tomato TF's +X axis points toward the stem, so harvesting approaches
from its -X axis. The target keeps the tomato between `tomato_gripper_tip` and
the detected stem direction, places the tip 18 mm below the tomato center, and
corrects the gripper roll to its -90 degree ground-level orientation. After the
initial Cartesian approach, the complete sequence uses the local axes of
`tomato_gripper_tip`:

1. Move +50 mm along tip X.
2. Move +20 mm along tip Z.
3. Move -15 mm along tip X.
4. Move +10 mm along tip Z.
5. Hold for 2 seconds.
6. Move -30 mm along tip X.
7. Return to PICK_READY.

The dwell separates the Cartesian motion into pre-wait and post-wait
trajectories. Plan-only mode publishes the complete five-trajectory sequence to
RViz without waiting or moving the robot. Inspect it before explicitly enabling
execution:

```bash
ros2 run rbpodo_tomato_harvest tomato_harvest_test --ros-args -p execute:=true
```

Select another tomato or tune the offsets with ROS parameters, for example:

```bash
ros2 run rbpodo_tomato_harvest tomato_harvest_test --ros-args \
  -p tomato_frame:=detected_tomato_5_tf \
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
