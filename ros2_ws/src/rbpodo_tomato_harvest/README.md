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
6. wrist3 선회 후 pre-approach까지 사용할 Cartesian/OMPL/CHOMP/PILZ LIN 선택
7. 검출된 모든 토마토를 순서대로 Plan-only 검증 후 실제 연속 수확
8. 지정한 시작/종료/변화량으로 줄기 위치와 회전을 바꾸며 Plan-only 자동 테스트
9. 실패한 Plan의 마지막 유효 관절 자세를 RViz Query Goal State로 표시

안전을 위해 새 검출 결과가 들어오거나 줄기 위치/회전을 변경하면 기존
Plan-only 성공 상태는 취소된다. 회전값은 `tomato_z_spin_deg`에 도 단위로
적용되어 메인 줄기 축을 기준으로 가지와 토마토 전체를 회전시킨다. 실제 수확
버튼은 현재 검출 결과에서 선택한 토마토의 Plan-only가 정상 완료된 경우에만
활성화된다. 줄기 위치나 회전을 바꾼 다음에는 GUI의 `토마토 촬영 / 검출`
버튼을 다시 눌러 TF를 갱신해야 한다.

토마토 장면의 기본 위치는 `[0.355, -0.375, 0.340]`, 메인 줄기 축 기준 회전은
`45°`이다. GUI에서 planner를 바꾸면 기존 Plan-only 성공 상태가 취소되며,
선택한 planner로 Plan-only를 다시 완료해야 실제 수확 버튼이 활성화된다.
Plan-only가 끝나면 GUI 실행 로그에 최초 시작 관절값과 `PICK_READY`를 거쳐
pre-grasp에 도달할 때까지의 관절별 최소각, 최대각, 변화폭 및 trajectory point
수가 도 단위 표로 표시된다. 수확 동작과 복귀 trajectory는 이 통계에서 제외된다.
Pre-grasp 이전에 실패하면서 MoveIt 응답에 부분 trajectory가 있으면 그 구간까지
합산한다. trajectory point가 하나도 생성되지 않은 실패는 표시할 궤적이 없다고
기록한다.
Plan-only 실패 응답에 부분 trajectory가 있으면 GUI의
`실패 자세 → RViz Goal` 버튼이 활성화된다. 이 버튼은 마지막 유효 trajectory
point를 `/rviz/moveit/update_custom_goal_state`로 보내 MotionPlanning의
`Query Goal State`를 이동한다. 실제 IK가 존재하지 않는 실패 보간점 자체는
관절 상태로 표시할 수 없으므로 fraction이 `0.0`이면 실패 구간의 시작 자세가
표시된다. RViz 설정의 `MoveIt_Allow_External_Program`은 기본 활성화되어 있다.
기본 수확 순서는 `PICK_READY → wrist3 선회 → Cartesian pre-approach`이다.
`PICK_READY` 관절값은
`rbpodo_moveit_config/config/rbpodo.srdf`의
`PICK_READY/mainpulation` named state를 단일 원본으로 사용한다. 수확 planner와
fake hardware 초기 자세가 모두 이 값을 읽으므로 SRDF를 수정한 뒤 두 패키지를
다시 빌드하고 MoveIt을 재시작해야 한다.
PICK_READY의 TCP 자세에서 목표 pre-grasp TCP 자세까지의 회전 중 wrist3의
로컬 Y축 성분만 투영해 필요한 선회량을 계산한다. 이 사전 회전은 TCP 위치를
PICK_READY 위치에 고정한 Cartesian 경로로 수행한다. 따라서 OMPL 관절 목표를
사용하지 않으며, 회전 중 필요한 관절값은 MoveIt의 연속 IK가 계산한다.
이후 `Cartesian`은 선회가 끝난 상태에서 pre-approach까지 TCP 직선 경로를
계산한다. GUI의 기본 선택도 `Cartesian`이다.
사전 제자리 Cartesian 회전이 실패하면 동일한 제자리 회전을 OMPL로 반복하지
않는다. 대신 원래 PICK_READY 상태에서 최종 pre-grasp pose까지 OMPL
RRTConnect로 한 번에 계획하며, 이것도 실패할 때 전체 Plan을 실패 처리한다.
pre-approach, 수확 및 대기 후 후퇴 중 Cartesian 경로가 실패하면 해당 구간만
OMPL RRTConnect로 자동 재계획한다. 여러 waypoint가 포함된 수확 구간은 동작을
생략하지 않고 waypoint별로 OMPL을 순차 적용한다. 모든 OMPL fallback에는 각
단계 시작 자세 중심의 `±120°` 관절 path constraint가 동일하게 적용된다.
Cartesian 실패 후 OMPL fallback이 성공하면 전체 Plan은 성공으로 처리하고,
결과 로그의 `cartesian_fallbacks`와 단계별 기록에 전환 구간을 남긴다.
OMPL, CHOMP, PILZ LIN은 같은 pre-approach TCP pose를 각 planning pipeline의
목표로 전달한다. 최초 PICK_READY 진입은 OMPL RRTConnect를 사용한다.
Pre-approach 이후의 접근과 수확 동작은 우선 TCP Cartesian 경로를 시도한다.
PILZ LIN은 TCP 직선 이동 중 연속 IK가
존재해야 하므로 토마토 위치와 자세에 따라 `NO_IK_SOLUTION`으로 실패할 수 있다.
CHOMP는 pose goal을 직접 처리하지 못하므로 PILZ LIN으로 collision-free endpoint
관절 상태를 먼저 구한 뒤, 그 endpoint로 향하는 실제 궤적을 CHOMP가 최적화한다.
각 OMPL 단계에는 그 단계의 시작 자세를 중심으로 `base`, `shoulder`, `elbow`,
`wrist1`, `wrist2`를 `±120°`로 제한하는 path constraint가 적용된다.
`wrist3`는 이 제한에서 제외되며 기존 로봇 관절 범위를 사용한다.
Planner 기반 pre-grasp pose의 허용 오차는 위치 `5 mm`, 자세 축별 `0.05 rad`
(약 `2.86°`)이다. 이후 수확 목표까지는 Cartesian 경로가 정확한 pose로
보정한다.

`mainpulation` 그룹은 `rbpodo_moveit_config/config/ompl_planning.yaml`에
명시된 `RRTConnect` 설정을 사용한다. OMPL Pose goal은 기본 planning time
`2초`, attempts `2회`로 제한하며, 다음 ROS 파라미터로 조정할 수 있다.

- `ompl_pose_planning_time`: OMPL Pose goal 최대 계획 시간, 기본 `2.0`
- `ompl_pose_planning_attempts`: OMPL Pose goal 계획 시도 수, 기본 `2`

PICK_READY joint goal은 기존 `pick_ready_planning_time=10.0`,
`pick_ready_planning_attempts=5`를 유지한다. `±120°` constraint는 관절의
허용 범위를 제한하는 조건이며 최단 trajectory를 보장하는 품질 기준은 아니다.

Pre-grasp 접근 방향은 토마토 TF의 `-X`를 기본으로 하되 로봇 베이스
(`link0` 원점)를 향하도록 토마토 로컬 `+Y` 또는 `-Y` 중 가까운 쪽으로
필요한 만큼 회전한다. signed 회전각은 local `+Y` 방향이 양수, local `-Y`
방향이 음수이며 `-45°~+45°`로 제한한다. 기존 `-X` 방향과 로봇 방향의
차이가 `10°` 이내이면 회전하지 않는다. 원본 토마토 TF는 변경하지 않고 tip
target과 pre-grasp geometry만 회전한 뒤 기존과 동일하게 TCP pose로 환산한다.

- `adaptive_grasp_enabled`: 적응형 접근각 사용 여부, 기본 `true`
- `adaptive_grasp_max_rotation_deg`: 최대 회전각, 기본 `45.0`
- `adaptive_grasp_deadband_deg`: 기존 방향 유지 범위, 기본 `10.0`

Plan 결과의 `adaptive_grasp` 항목과 자동 테스트 CSV/JSONL에는 적용 회전각과
회전 전후 로봇 방향 오차가 기록된다.

`검출 토마토 전체 연속 수확`은 현재 검출 목록을 0번부터 순서대로 처리한다.
각 토마토마다 Plan-only를 먼저 수행하고 성공한 경우에만 실제 수확하며, 로봇은
매 수확 후 `PICK_READY`로 복귀한다. 계획에 실패한 토마토는 실제로 움직이지
않고 건너뛴 뒤 다음 토마토를 계속 처리한다. 실제 실행이 실패하거나 작업 도중
새 검출 결과가 들어오면 남은 수확은 실행하지 않고 즉시 중단한다.

RViz의 `HarvestPlanResults` 화살표는 성공·실패 여부와 관계없이 planner가
실제로 사용한 토마토 로컬 pre-grasp 진입 벡터를 표시한다. 보정이 없으면
토마토 로컬 +X축이다. local `+Y` 쪽으로 pre-grasp 위치가 보정되면 화살표는
`+X→-Y`, local `-Y` 쪽으로 보정되면 `+X→+Y` 방향으로 회전한다.
pre-grasp 위치 벡터와 화살표가 나타내는 토마토 방향 진입 벡터는 서로
반대이기 때문이다. 길이는 검출된 중심→줄기점
거리를 지면에 투영한 뒤 8 mm를
줄인 값을 사용한다. Plan-only 회전 없는 성공은 초록색, 적응 접근각이 적용된
성공은 하늘색, 실패는 빨간색이다.
결과가 바뀔 때만 Transient Local 마커를
발행하므로 깜빡이지 않고 유지된다. 새 검출 결과가 들어와도 기존 결과 마커는
유지되며, GUI의 `결과 마커 지우기` 버튼을 눌렀을 때 전체 마커를 삭제한다.
planner를 바꾸면 기존 계획 결과를 무효화하고 마커를 지운다.

오른쪽 `줄기 위치/회전 Plan 자동 테스트` 패널에는 X/Y/Z/회전의 시작값,
종료값, 변화량을 입력한다. `랜덤`을 체크한 항목은 각 케이스마다 시작~종료
범위에서 임의 값을 사용하고, 체크하지 않은 항목은 시작부터 종료까지 일정하게
변화량만큼 이동한다. 변경 중인 축 가운데 가장 먼저 종료값에 도달한 축을
기준으로 전체 케이스 생성을 종료한다. 랜덤 축의 변화량은 사용하지 않으며 종료
조건에서도 제외한다. 따라서 범위가 변하는 모든 축을 랜덤으로 설정한 경우에는
종료 기준이 될 비랜덤 변화 축을 하나 이상 지정해야 한다. 자동 실행 시 각
케이스마다 줄기 장면
적용과 촬영/검출을 수행한 뒤,
토마토 0번부터 7번까지 차례로 Plan-only를 수행한다. 회전 없는 성공은 초록색,
적응 접근각 성공은 하늘색, 실패는 빨간색
화살표로 표시하며, 각 화살표는 생성 당시 `link0` 좌표에 고정되어 다음 케이스로
줄기가 이동해도 기존 위치에 누적된다. 자동 테스트에서는 처리 부하를 줄이기 위해
`/display_planned_path` 궤적 애니메이션을 발행하지 않으며, 개별 Plan-only와 실제
수확에서는 기존처럼 전체 궤적을 발행한다. 또한 자동 테스트가 시작될 때 Planner
프로세스를 한 번만 생성하고 모든 토마토와 위치/회전 케이스에서 같은 MoveIt
액션 클라이언트를 재사용한다. 토마토마다 ROS 프로세스를 빠르게 생성·종료할 때
발생하던 DDS goal 응답 유실과 30초 타임아웃 지연을 방지한다.

자동 테스트 중에는 GUI 오른쪽에서 완료 수, 성공/실패 수, 성공률, 평균 Plan
시간과 실패 단계별 개수를 실시간으로 확인할 수 있다. 최근 100개 결과는 케이스,
토마토 번호, 결과, 실패 단계와 소요시간으로 표시된다. 결과는 각 Plan 직후
`~/farmily_tomato/harvest_results/<실행시각>/` 아래에 즉시 저장된다.

- `session.json`: 입력 범위, 변화량, 랜덤 seed, 실제 생성된 모든 케이스
- `results.jsonl`: 단계별 세부 진단을 포함한 원본 결과
- `results.csv`: 스프레드시트 분석용 요약 결과
- `summary.json`: 완료 시 성공률과 실패 단계별 합계

실패 단계는 `OMPL_PICK_READY`, `FK_PRE_ROTATION`,
`CARTESIAN_PRE_ROTATION`, `CARTESIAN_PREAPPROACH`,
`OMPL_PREAPPROACH`, `CHOMP_PREAPPROACH`,
`PILZ_INDUSTRIAL_MOTION_PLANNER_PREAPPROACH`, `CARTESIAN_APPROACH`,
`CARTESIAN_POST_WAIT`, `PILZ_PTP_RETURN_PICK_READY`, `TF_TARGET` 등으로
구분된다.
Cartesian 실패에는 경로 fraction과 MoveIt 오류 코드가, OMPL/CHOMP/PILZ
실패에는 pipeline, planner ID와 MoveIt 오류 코드가 기록된다.

마지막 복귀에는 OMPL과 Cartesian 대체 복귀를 사용하지 않는다. PILZ PTP가
실패하면 `failure_stage=PILZ_PTP_RETURN_PICK_READY`로 기록하고 전체 Plan을
실패 처리한다.

저장된 세션을 한눈에 확인할 수 있는 HTML 분석 보고서는 다음 명령으로 생성한다.

```bash
ros2 run rbpodo_tomato_harvest harvest_report \
  ~/farmily_tomato/harvest_results/20260723_124228_043000
```

세션 폴더에 `report.html`이 생성된다. 보고서에는 전체 성공률, 실패 단계와 이유,
토마토 번호별 성공률, 회전 구간별 성공률, 회전–Y 위치 분포와 Cartesian fraction이
낮은 실패 사례가 포함된다. 외부 웹 서버나 추가 파이썬 패키지는 필요하지 않다.

실제 카메라 팀의 서비스 이름이 다른 경우 launch 인자로 연결한다.

```bash
./scripts/run_harvest_gui.sh \
  camera_service:=/real_camera/detect_tomatoes
```

## 시뮬레이션 초기 자세

`run_rbpodo_rb5_moveit.sh`로 fake hardware를 실행하면 6개 관절이 SRDF에
등록된 `PICK_READY` 자세에서 시작한다. 기존 영점 자세로 시작해야 할 때는
launch 인자를 추가한다.

```bash
./scripts/run_rbpodo_rb5_moveit.sh start_at_pick_ready:=false
```

`run_rbpodo_rb5_moveit_real.sh`로 실제 로봇에 연결할 때는 이 설정으로 로봇을
움직이지 않으며, 하드웨어에서 수신한 현재 관절 자세를 그대로 사용한다.

## 로봇 작업 공간 가벽과 받침대

RB5 MoveIt launch는 기본적으로 `link0`에 고정된 작업 공간 collision을 생성한다.
두 개의 수직 가벽과 상단 가벽은 로봇의 좌우 및 상단 동작 범위를 제한하고,
로봇 바닥에는 베이스 하단 `z=0`에 맞닿는 받침대가 추가된다. 토마토와 줄기
collision은 별도 옵션이므로 가벽을 켜도 자동으로 활성화되지 않는다.

기본 가벽과 받침대를 사용하지 않으려면 다음과 같이 실행한다.

```bash
./scripts/run_rbpodo_rb5_moveit.sh --no-wall
```

가벽을 명시적으로 켜서 실행할 수도 있다.

```bash
./scripts/run_rbpodo_rb5_moveit.sh --wall
```

기본 크기는 `scripts/run_rbpodo_rb5_moveit.sh` 상단의
`Robot workspace collision dimensions` 설정 블록에서 변경할 수 있다.

직접 `ros2 launch`를 실행할 때는 다음 인자로 치수를 조정할 수 있다.

```bash
ros2 launch rbpodo_moveit_config moveit.launch.py \
  workspace_left_wall_x:=-0.55 \
  workspace_right_wall_x:=0.65 \
  workspace_wall_height:=1.15 \
  workspace_ceiling_z:=1.10 \
  robot_pedestal_size_x:=0.40 \
  robot_pedestal_size_y:=0.40 \
  robot_pedestal_height:=0.18
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
2. Selected Cartesian/OMPL/CHOMP/PILZ motion to the pre-approach TCP pose
3. Cartesian motion from pre-approach through the tomato harvest sequence
4. Cartesian post-wait retreat

The planner selected in the GUI controls the motion from `PICK_READY` to the
pre-approach pose. `Cartesian` preserves the original direct TCP path, while
OMPL, CHOMP, and PILZ LIN receive the same TCP pose as a planning-pipeline goal.
Entering `PICK_READY` uses OMPL RRTConnect. If any Cartesian segment fails, that
segment is retried with constrained OMPL RRTConnect. Multi-waypoint harvest
segments retain every waypoint and retry them sequentially. The harvest target
and local offsets are still defined at `tomato_gripper_tip`, but every target
pose is converted through the fixed TCP-to-tip transform before MoveIt plans
with `tcp` as its controlled link.

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
7. Return directly to all PICK_READY joint targets with PILZ PTP.

The dwell separates the Cartesian motion into pre-wait and post-wait
trajectories. Plan-only mode publishes the complete six-trajectory sequence to
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
