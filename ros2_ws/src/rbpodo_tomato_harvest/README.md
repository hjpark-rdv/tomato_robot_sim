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

배열의 각 원소에는 토마토 ID, 중심점, 줄기 방향점이 들어간다. 카메라 ID가
`...C0:T7`처럼 끝나면 마지막 `C0:T7`만 추출해 실제 수확 TF 이름으로 사용한다.
GUI 목록, 개별·스텝·자동 수확 및 전체 사전계획도 모두 같은 TF 이름을 전달한다.
이 형식이 없는 Fake·구형 입력만 `detected_tomato_N_tf` 이름을 사용한다. 새 배열이
들어오면 이전 수확 TF 목록을 교체하므로 여러 번 촬영해도 TF가 누적되지 않는다.

TF를 등록하기 전에 모든 중심점을 `world`로 변환한다. `C0:T7`에서는 `C0`을
클러스터 ID로 인식하며, 같은 `C#`의 토마토가 중간에 다른 클러스터와 섞이지
않도록 먼저 묶는다. 기존 `cluster_*` 경로 형식도 지원한다. 각 클러스터에 속한
토마토의 `world Z` 합계가 큰 클러스터부터 처리하고, 한 클러스터 안에서는 Z가
높은 토마토부터 정렬한다. 클러스터 합계가 같으면 클러스터 최고 Z와 클러스터
ID를 안정적인 tie-breaker로 사용한다. 클러스터 정보가 없는 원본 ID는 각각
독립 클러스터로 취급하므로 기존 입력은 전체 높이 내림차순을 유지한다. GUI 검출
목록과 TF generator가 동일한 정렬 함수를 사용한다. 운영 시 사용자는 카메라
팀이 제공하는 촬영 서비스만 호출한다.

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

Fake 카메라가 만드는 원본 검출 배열은 XY 거리가 기본 5 cm 이내인 토마토를
같은 세로 열로 묶어 위에서 아래로 나열한다. 하지만 최종
`detected_tomato_N_tf` ID는 실제 카메라와 동일하게 TF generator에서 `world` Z
높이 내림차순으로 다시 부여한다. Fake ID에는 명시적 `cluster_*` 성분이 없어서
각 토마토를 독립 클러스터로 취급한다. Fake 검출 배열의 열 판정 거리는 launch 인자
`vertical_column_xy_tolerance`로 조절할 수 있다.

```bash
ros2 service call /fake_tomato_camera/detect_tomatoes \
  farmily_tomato_interfaces/srv/DetectTomatoes '{}'
```

예를 들어 세로 열 허용거리를 7 cm로 변경하려면 다음처럼 실행한다.

```bash
ros2 launch rbpodo_tomato_harvest fake_camera.launch.py \
  vertical_column_xy_tolerance:=0.07
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

GUI는 `수확 작업`, `스텝 실행`, `접근 반복 테스트`, `자동 테스트`,
`장면 · 속도 · 리프트` 탭으로 구분되며,
실행 로그와 현재 상태는 어느 탭에서도 확인할 수 있도록 창 하단에 고정된다.
다음 작업을 키보드 명령 없이 수행할 수 있다.

1. Fake tomato 또는 실제 카메라 서비스를 선택해 검출하고 모든 토마토 목록 확인
2. 수확할 `detected_tomato_N_tf` 선택
3. RViz에서 전체 궤적을 확인하는 Plan-only 실행
4. Plan-only 성공 후 실제 수확 모션 실행
5. 메인 토마토 줄기의 X/Y/Z 위치와 줄기 축 기준 회전 각도 조회 및 변경
6. 검출된 모든 토마토를 순서대로 Plan-only 검증 후 실제 연속 수확
7. 지정한 시작/종료/변화량으로 줄기 위치와 회전을 바꾸며 Plan-only 자동 테스트
8. 실패한 Plan의 마지막 유효 관절 자세를 RViz Query Goal State로 표시
9. RB Speed Bar와 OMPL/Joint 계획 속도·가속도를 퍼센트 단위로 변경
10. 수확 동작 중 리니어모터 대기시간을 초 단위로 설정
11. 연속 수확에서 식물 바깥쪽 arc를 따라 다음 pre-grasp로 전환
12. 개별·전체·자동 수확 도중 현재 trajectory와 실제 RB5 모션 정지
13. UV 리프트 노드 실행, Bottom calibration, 현재 높이 확인과 목표 높이 이동
14. 토마토 높이보다 40 cm 낮게 리프트를 자동 배치하는 리프트 수확
15. 전체 토마토 trajectory를 먼저 계산한 뒤 저장된 경로만 실행하는 사전계획
16. RB DOUT8 릴레이 전원과 Arduino PIN8/9를 이용한 그리퍼 스트로크 제어
17. 검출된 토마토 중심을 RViz에 핑크색 구형 마커로 표시
18. 전체 수확 궤적을 한 번 계산한 뒤 실제 로봇을 한 단계씩 선택 실행
19. 접근 1~X단계를 선택해 연속 진입·역순 복귀하고 전체 토마토까지 순회하는 테스트
20. `/capture_camera` Trigger 서비스를 GUI에서 독립적으로 호출해 카메라 캡처
21. SRDF 저장 자세를 선택해 constrained OMPL로 Plan & Execute
22. 현재 Plan/스텝 대상의 비전 피드백 JSON을 로컬 TXT로 저장하고
    `DebugFrame.srv` 서비스로 전송

`수확 작업` 탭의 `카메라 캡처` 버튼은 `/capture_camera`
(`std_srvs/srv/Trigger`)를 호출한다. 이 버튼은 토마토 좌표를 갱신하는
`토마토 촬영 / 검출`과 별개이며, 서비스 응답의 성공 여부와 메시지를 하단 상태창과
실행 로그에 표시한다.

카메라 검출 영역의 `비전 피드백 JSON 저장/전송` 버튼은 실제로 계획 중인
`detected_tomato_N_tf`에 대응하는 카메라 검출 원본을 저장한다. 스텝 실행과
접근 반복 테스트의 Plan 대기·실행·일시정지 상태에서도 버튼을 사용할 수 있으며,
GUI에서 다른 행을 선택하더라도 실행 프로세스가 잡은 토마토를 기록한다. 파일에는
카메라 ID, 카메라 프레임 기준 토마토/줄기 X/Y/Z, 두 점 거리, 검출 순간의
실제 토마토/줄기점을 `link0`로 각각 TF 변환한 좌표, Planner가 계산한
Recommend/최종 pre-grasp와 접근각 보정 결과가 JSON으로
포함되고 기본 저장 경로는 `~/farmily_tomato/camera_target_records/`이다.
Recommend는 RViz 하늘색 화살표와 같은 검출 TF 원본 진입각(`0°`)이고,
Final은 실제 IK/도달성 보정을 거쳐 선택된 진입각이다.
`문제 유형`은 목록에서 고르거나 직접 입력할 수 있고 `비고`에는 영상 검토 요청을
자유롭게 적을 수 있다. 현재 `TomatoDetection` 인터페이스가 제공하지 않는 UV와
confidence는 임의 값을 만들지 않고 JSON에서 제외한다. Plan 결과가 아직 없는
시점의 로봇 계산값은 `null`로 저장된다. 서비스 전달에 불필요한 GUI 상태,
Planner 내부 설정, detection generation과 중복 진단 정보는 저장하지 않는다.
동일한 JSON 문자열은 `farmily_tomato_interfaces/srv/DebugFrame`의
`request_text` 필드에 그대로 담아 기본 `/debug_frame` 서비스로 전송한다.
서비스 이름은 GUI 노드의 `debug_frame_service` 파라미터로 변경할 수 있다.
서비스가 실행되지 않았거나 응답이 실패해도 로컬 TXT는 유지되며, 저장 성공과
서비스 전송 결과를 상태창과 실행 로그에 각각 표시한다.

Plan 완료 후 저장한 JSON은 별도의 좌표 그래프 프로그램으로 확인할 수 있다.

```bash
source ~/farmily_tomato/ros2_ws/install/setup.bash
ros2 run rbpodo_tomato_harvest vision_feedback_viewer
```

인자 없이 실행하면 가장 최근 피드백 파일을 열고, 파일 선택 버튼으로 다른 결과를
불러올 수 있다. 특정 파일을 바로 열 수도 있다.

```bash
ros2 run rbpodo_tomato_harvest vision_feedback_viewer \
  ~/farmily_tomato/camera_target_records/파일명_feedback.txt
```

그래프는 외부 plotting 패키지 없이 Tkinter로 동작하며, RViz를 위에서 본 것과
같은 `link0` 기준 X-Y 평면도를 표시한다. `link0 +X`는 그래프 위쪽(로봇 전방),
`link0 +Y`는 왼쪽, `link0 -Y`는 오른쪽에 표시한다. 따라서 RViz의 수평 진입
각도가 Y-Z 측면 투영으로 사라지지 않는다. 로봇 base, 토마토,
실제 검출 줄기점, Recommend 진입선과 최종 진입선을 색상으로
구분하고 base→tomato 및 tomato→vine 실제 3D 거리도 함께 표시한다. 진입선은
실제 pre-grasp 지점을 점으로 유지하면서 화살표 꼬리를 3배로 연장해 구별하기 쉽게
표시하고, 화살표 촉은 토마토 원의 바깥에서 멈춘다. 토마토와 줄기점 마커는
RViz 검출 마커의 실제 지름(17.5 mm, 6 mm)을 그래프 축척에 맞춰 표시하되,
줄기점은 전체 로봇 범위에서 사라지지 않도록 최소 반지름 6 px로 표시한다.
마커 중심 좌표는 확대와 무관하게 실제 좌표를 유지한다. 줄기점 마커는
`robot.vine_xyz`의 X-Y 실좌표에 그대로 표시하고, 겹침을 줄이기 위해 텍스트
라벨만 리더선으로 분리한다. 화면 하단에는 카메라 광학 프레임의
`ΔZ(vine-tomato)`와 어느 점이 카메라에 가까운지도 함께 표시한다.
하늘색 Recommend 화살표는 RViz 검출 마커와 동일하게 `robot.tomato_xyz`와
실제 `robot.vine_xyz` 벡터의 반대편에서 토마토로 진입하도록 표시하고,
주황색 Final 화살표는 Planner가 선택한 최종 pre-grasp 좌표를 표시한다. 기존
JSON에는 `link0` 기준 토마토/줄기 좌표가 없으므로 새 코드로 Plan한 뒤 JSON을
다시 저장해야 한다.

카메라 검출 영역의 `저장 자세` 콤보박스에서는 MoveIt SRDF에 등록된
`PICK_READY`, `PICK_READY_RIGHT`, `CAPTURE_LEFT`를 선택할 수 있다.
`Plan & Execute`를 누르면 현재 관절 자세에서 선택 자세까지 충돌 검사와 현재
자세 중심 ±120° joint constraint를 적용한 OMPL/RRTConnect 경로를 계획하고,
계획 성공 시에만 실제 trajectory를 실행한다. 실행 중에는 다른 수확 명령이
비활성화되며 기존 `모션 정지` 버튼으로 계획·실행 취소와 RB5 정지를 요청할 수
있다.

같은 탭의 `수확 옵션` 아래에는 `/tomato_vision/result_image`
(`sensor_msgs/msg/CompressedImage`)의 마지막 검출 결과를 표시한다. JPEG/PNG를
직접 디코딩하고 표시 영역에 맞춰 종횡비를 유지해 축소하며, 새 결과를 받기
전까지 마지막 이미지를 유지한다. 토픽은 서비스 호출 시점에만 발행되는
`RELIABLE`/`VOLATILE` 데이터이므로 GUI는 동일한 reliable QoS로 구독하지만,
GUI 실행 전에 발행된 과거 이미지는 표시하지 않는다.

검출된 토마토 목록에서 행을 선택하면 실행 로그 오른쪽의 측면 그래프가 즉시
갱신된다. 검출 직후에는 `link0`로 변환해 보존한 토마토 중심과 줄기점으로
Recommend 진입 방향만 표시한다. 해당 토마토의 Plan 결과가 있으면 Planner의
최종 pre-grasp를 주황색 Final 화살표로 함께 표시하며, 아직 Plan하지 않았거나
최종 접근 형상이 계산되기 전에 실패했다면 Final 화살표는 표시하지 않는다.

`스텝 실행` 탭에서는 토마토와 시작 자세를 선택하고 `스텝 Plan 생성`을 누른다.
이 시점에는 실제 로봇이 움직이지 않으며, 현재 자세→PICK_READY, pre-approach,
접근, 전진·상승·후퇴, 리니어모터 대기와 PICK_READY 복귀까지 총 10개 단계의
trajectory를 한 번 계산해 같은 프로세스에 보관한다. `실제 로봇 스텝 실행 허용`을
체크하고 최초 안전 확인을 통과하면 `다음 단계 실행` 또는 `선택 단계까지 실행`으로
완료되지 않은 단계를 순서대로 실행할 수 있다. `이전 단계 역순 실행`은 현재
위치에 바로 이어진 직전 단계 하나만 캐시된 joint trajectory의 역순으로
되돌린다. 같은 버튼을 반복하면 한 단계씩 더 되돌아갈 수 있고, 되돌린 단계는
다시 정방향으로 실행할 수 있다. 정방향과 역방향 모두 실행 직전에 실제 관절
자세와 캐시된 시작점의 최대 오차를 확인하며 `3°`를 넘으면 오래된 trajectory
실행을 차단한다. 중간 단계를 건너뛰는 실행은 허용하지 않는다.
`로봇 즉시 정지`는 MoveIt/controller goal 취소와 RB 정지를 요청하고 캐시를
폐기하므로, 정지 후에는 스텝 Plan을 다시 생성해야 한다.
수확 단계 목록 오른쪽의 `3~7단계 tip 로컬 XYZ 커스텀 (mm)` 표에서 각 단계의
X/Y/Z 이동량을 직접 입력할 수 있다. 기본값은 3단계 `(10, 0, 0)`, 4단계
`(40, 0, 0)`, 5단계 `(20, 0, 20)`, 6단계 `(0, 0, 20)`, 7단계
`(-50, 0, 0)` mm로 기존 동작과 같다. 각 축은 `-200~+200 mm` 범위에서
설정하며 모두 `tomato_gripper_tip` 로컬 좌표다. Plan 생성 시 입력값을 검증하고
trajectory에 고정하며, 해당 스텝 세션이 끝날 때까지 편집란을 잠근다. 역순 실행은
커스텀 좌표로 생성해 캐시한 동일 trajectory를 반대로 재생한다.
5단계에서 6단계로 전환할 때는 두 단계의 끝점을 직선으로 연결하지 않고,
tip 로컬 수평 진행 방향에서 6단계 이동 방향으로 접선이 변하는 3차 곡선을
7개 Cartesian waypoint로 계획한다. 단계별 최종 좌표는 변경하지 않으며,
6→5 역순 실행도 캐시된 동일 곡선 trajectory를 역재생한다.

`접근 반복 테스트` 탭은 전체 수확 중 1단계 현재 자세→PICK_READY부터 최대 6단계
tip 로컬 `+Z 20 mm` 2차 상승까지 계획한다. `마지막 단계 X`에서
1~6 중 하나를
선택하면 `1 → X 연속 진입`은 해당 단계까지 중단 없이 순서대로 실행한다. 이후
`X → 1 역순 복귀`를 누르면 별도 복귀 경로를
재계획하지 않고, 캐시된 각 joint trajectory의 구간 순서·point 순서·시간과
속도 방향을 뒤집어 정확히 같은 관절 경로로 원래 시작 자세까지 복귀한다. 복귀가
끝나면 같은 두 버튼을 반복해서 사용할 수 있다. 매 구간 직전 실제 관절 시작
오차가 `3°`를 넘거나 trajectory 실행이 실패하면 캐시를 폐기하고 새 Plan 생성을
요구한다. `4단계 진입 길이 (mm)`는 기본 `40`이며 `10~70 mm` 범위에서 직접
설정할 수 있다. 이 값은 4단계 tip 로컬 `+X` trajectory 계획과 화면의 단계
설명에 동일하게 적용되며, X가 4보다 작으면 해당 반복에서는 실행되지 않는다.
`전체 토마토 1 → X → 1`은 검출 순서대로 각 토마토의 Plan을 새로
계산하고 정방향 진입과 동일 trajectory 역순 복귀를 완료한 뒤 다음 토마토로
넘어간다. Plan 실패는 결과에 남기고 다음 토마토를 계속 시험하지만, 실제
trajectory 실행 실패는 로봇의 다음 시작 상태를 보장할 수 없으므로 전체 실행을
즉시 중단한다. 이 모드는 6단계 이후를 계획하지 않으므로 이후 수확 동작의 성공
여부와 무관하게 원하는 접근 구간만 독립적으로 시험할 수 있다.
`일시 정지`는 실행 중인 trajectory를 강제로 끊지 않고 현재 단계가 끝난 직후
다음 단계를 보류한다. 버튼이 `계속 실행`으로 바뀌며, 다시 누르면 같은 토마토의
저장된 진행 단계와 정·역방향 상태부터 이어서 실행한다. 전체 토마토 모드에서도
현재 토마토의 단계 사이, 정방향과 역방향 사이, 토마토와 다음 토마토 사이에서
동일하게 동작한다. 일시 정지 중 로봇 자세가 바뀌어 캐시 시작점 오차가 3°를
초과하면 기존 안전 검사에 의해 재개가 차단된다.

`리니어모터 대기시간`의 기본값은 `2.0초`이다. 실제 수확 시 pre-wait
Cartesian 동작이 끝난 뒤 입력한 시간만큼 자세를 유지하고 post-wait 후퇴를
시작한다. 입력값은 개별 수확, 전체 연속 수확과 실제 실행을 활성화한 자동
테스트에 동일하게 적용된다. Plan-only에서는 궤적만 계산하므로 실제로 기다리지
않는다.

수확 작업 탭의 `4단계 진입 길이 (mm)`는 기본 `40`이며 `10~70 mm` 범위에서
1 mm 단위로 설정한다. 개별 Plan-only·실제 수확과 전체 연속 Plan·수확의
4단계 tip 로컬 `+X` 이동에 적용된다. 개별 실행에서는 길이가 변경되면 기존
Plan-only 검증을 폐기하며, 전체 작업에서는 시작 시 선택한 길이를 모든 토마토에
동일하게 고정한다.

`토마토별 종료 단계`는 처리할 토마토 번호가 아니라 각 토마토에서 실행할 수확
단계를 제한한다. `3단계까지`는 모든 검출 토마토에서 `현재 자세 → PICK_READY`,
`PICK_READY → PRE_APPROACH`, `PRE_APPROACH → 접근 목표`까지만 실행한다.
`4단계까지`는 여기에 `접근 목표 → 앞으로 이동`을 추가한다. 제한 모드에서는
리니어모터 대기와 이후 수확·후퇴 동작을 실행하지 않는다. Arc를 체크하면 이전
토마토의 선택 종료 자세에서 식물 바깥 arc를 거쳐 다음 토마토로 이동하고, 전체
목록의 마지막 토마토가 끝난 뒤 선택된 `PICK_READY` 자세로 복귀한다. 리프트를
움직일 안전 후퇴 단계가 없으므로 단계 제한과 리프트 수확은 함께 사용할 수 없다.

`연속 수확: 식물 바깥 arc로 다음 pre-grasp 이동`은 기본 해제되어 있다. 체크하면
전체 연속 수확과 실제 실행 자동 테스트에서 첫 토마토는 `PICK_READY`로
시작하고, 이후 토마토는 `이전 post-wait 자세 → 식물 바깥 arc → 다음
pre-grasp`를 하나의 trajectory로 연결한다. TCP 시작·목표 자세 사이에 기본
7개 waypoint를 만들고, 직선 경로에서 식물 바깥 방향으로 최소 `0.12 m`, 최대
`0.25 m` 휘어진 경로를 생성한다. TCP 방향은 각 waypoint에서 부드럽게
보간한다. 먼저 충돌 검사를 포함한 Cartesian 경로를 시도하고, 일부 구간이
실패하면 해당 waypoint까지 시작 자세 중심 `±120°` constraint가 적용된 OMPL
RRTConnect로 대체한다. Cartesian이나 OMPL이 성공을 반환해도 Arc 전체에서
관절 하나의 span이 `continuous_arc_max_joint_span_deg` 기본 `120°`를 넘으면
대회전 경로로 판단해 폐기한다. 이 검사에는 `wrist3`도 포함한다. arc 전환
전체가 실패하면 현재 자세에서
`PICK_READY`까지 새로운 OMPL 우회 경로를 만들지 않는다. 대신
`PICK_READY` 이후 지금까지 성공한 연속 수확 trajectory 전체를 역순으로
재생하여 검증된 경로 그대로 `PICK_READY`에 복귀한 뒤, 다음 토마토에서 이미
독립 검증한 `PICK_READY → pre-grasp` trajectory를 재사용한다. 따라서 Arc
실패가 다음 토마토 자체의 성공 판정을 실패로 바꾸지 않는다. 역재생할 이력이
없으면 안전 복구 계획을 실패로 처리한다. 마지막 토마토 수확이 끝난 뒤에는
`PICK_READY`로 복귀한다.

`전체 모션 사전계획 후 저장 trajectory 실행`은 기본 해제되어 있다. 체크한 뒤
`전체 연속 수확`을 누르면 별도 planner 프로세스가 로봇을 움직이지 않은 상태로
모든 토마토의 전체 trajectory를 먼저 계산한다. 기본 수확에서는 이전 수확의
`RETURN_PICK_READY` 마지막 관절 상태를 다음 수확의 시작 상태로 사용한다. Arc
수확에서는 각 토마토를 먼저 `PICK_READY` 기준으로 독립 계획한다. 일시적인 OMPL
실패는 기본 3회까지 다시 시도하고, deadline·TF·geometry처럼 재시도로 바뀌지
않는 실패는 즉시 건너뛴다. 독립 계획에 성공한 토마토만 이전 post-wait 마지막
관절 상태와 TCP 자세에서 Arc 전환을 별도로 계산한다. Arc가 실패해도 토마토의
독립 성공 결과는 유지하며 cached reverse 복구와 독립 trajectory를 조합한다.
자세·각도·IK·경로 계획에 실패한 토마토는 실패 결과를 남기고 건너뛰며, 성공한
토마토의 저장 trajectory만 순서대로 실행한다. 마지막 번호의 토마토가 실패하면
마지막 성공 자세에서 저장된 성공 경로를 역재생하여 `PICK_READY`로 복귀한다.
모든 토마토가 계획에 실패한 경우에만 실행할 경로가 없으므로 종료한다. 사전계획
완료 후 실행 전 로봇 관절이 최초 계획 시작점에서 `3°` 이상 달라진 경우에도
캐시를 실행하지 않고 중단한다. 실제 trajectory 실행
실패는 안전상 전체 수확을 중단한다. 리프트 수확은 미래 리프트 높이의 planning
scene을 별도로 구성해야 하므로 현재 사전계획 옵션과 함께 사용할 수 없다.

수확 모션 영역의 `전체 연속 Plan` 버튼은 위 사전계획 절차만 수행하고 저장된
trajectory를 실제 로봇에 실행하지 않는다. 현재 선택한 시작 자세, 토마토별 종료
단계와 식물 바깥 Arc 옵션을 그대로 사용하며, 각 토마토의 성공·실패 결과와 실패
단계는 검출 토마토 목록과 실행 로그에 표시한다. 리프트 수확은 미래 리프트 높이별
planning scene을 한 번에 구성할 수 없으므로 이 버튼과 함께 사용할 수 없다.

식물바깥 Arc 전체수확은 Arc 실패 시 사용할 정확한 역방향 trajectory 이력을
보장하기 위해 리프트 수확이 아닐 때 사전계획 옵션을 자동 활성화한다. 복구가
발생한 계획에는 `recovery_stage=CACHED_TRAJECTORY_REVERSE_TO_PICK_READY`가
기록된다.

`리프트 수확: 토마토보다 40cm 낮게`를 체크하면 수확 계획 전에 검출 토마토의
`world` 기준 Z 높이를 조회하고 다음 식으로 Bottom 기준 목표 높이를 계산한다.

```text
목표 높이(mm) = clamp((토마토 world Z - 0.40m) × 1000, 0, 750)
```

계산값이 0 mm보다 작으면 0 mm, 750 mm보다 크면 750 mm를 사용한다. 실제
모드에서는 Bottom calibration과 리프트 노드 연결이 확인되어야 하며, 현재 높이가
목표의 기본 허용오차 `±1 mm`에 들어온 뒤 수확 계획을 시작한다. 시뮬레이션에서는
동일한 계산을 RViz lift joint에만 적용한다. 검출 토마토 TF와 토마토 장면은
`world`에 고정되므로 리프트가 움직여도 식물까지 로봇과 함께 이동하지 않는다.
자동 테스트에서 줄기 위치가 바뀐 직후에는 새 카메라 검출 중심과
`detected_tomato_N_tf`의 world 위치가 기본 `3 mm` 이내로 일치할 때까지
대기한다. 따라서 이전 테스트 케이스의 TF로 첫 토마토 리프트 높이나 수확 경로를
계산하지 않는다.

리프트 수확과 연속 수확을 함께 사용하면 중간 토마토 수확 후 로봇이 식물
바깥쪽으로 기본 `0.12 m` 안전 후퇴한 다음 리프트를 조정한다. 리프트 목표 도달
후 변경된 현재 상태에서 다음 pre-grasp arc를 새로 계획하므로, 리프트 이동 전에
계산한 trajectory를 재사용하지 않는다. 마지막 토마토 이후에는 기존과 같이
`PICK_READY`로 복귀한다. 자동 위치 테스트의 Plan-only에서는 시뮬레이션
모드일 때만 RViz 리프트를 이동한다. 실제 모드에서는 `실제 로봇 실행`까지
체크해야 물리 리프트가 움직인다.

`검출 토마토 전체 연속 수확`을 실행하면 각 토마토의 최종 결과가 오른쪽
`실시간 통계` 테이블에 즉시 누적된다. Plan-only가 실패한 토마토는 그 시점에
실패로 기록하고, Plan-only가 성공한 토마토는 실제 수확 실행까지 끝난 뒤 최종
성공 또는 실행 실패로 한 번만 기록한다. 계획 단계, Cartesian→OMPL 대체 여부,
계획 시간과 실제 전체 시퀀스 시간을 자동 테스트 결과와 같은 형식으로 표시한다.
전체 연속 수확 결과는 현재 화면 표시 전용이며 자동 위치 테스트의 CSV/JSONL
파일에는 추가하지 않는다.

GUI 하단의 `UV 리프트 제어`는 `farmily_uv_lift` 노드의 실행 여부를 ROS graph로
주기적으로 확인한다. `/lift_controller_node`가 없을 때만 `리프트 노드 실행`
버튼이 활성화되며, 버튼을 누르면 다음 launch를 실행한다.

```bash
ros2 launch farmily_uv_lift farmily_lift_controller_launch.py
```

`Bottom calibration 실행`은 `/lift_control/find_bottom_limit`에 `Bool(True)`를
발행한다. 완료 상태는 `/lift_status/bottom_limit_found`, 현재 높이는
`/lift_status/current_height`에서 받아 실시간으로 표시한다. Calibration이
완료된 뒤 목표 높이를 mm 단위로 입력하고 `높이 이동`을 누르면
`/lift_control/move_height`에 `Float64`로 발행한다. 목표 높이는 Bottom 기준
0~750 mm 범위여야 한다. `리프트 이동 정지`는 확인창 없이 즉시
`/lift_control/stop`에 `Bool(True)`를 발행하며, 수확 모션 실행 중에도 사용할 수
있다. Calibration 중 정지하면 해당 calibration은 실패 상태로 종료된다. 실제
모터가 움직이는 명령이므로 CAN 연결과 주변 안전을 먼저 확인해야 한다.

`rb5_farmily` 모델에서는 회색 리프트 프레임이 `world`에 고정되고, 초록색
`farmily_lift_platform`과 `link0` 이하 로봇 전체가
`farmily_lift_height_joint`를 따라 함께 승강한다. MoveIt launch가 함께 실행하는
`lift_joint_state_publisher`는 `/lift_status/current_height`의 mm 값을 m로 변환해
`/joint_states`에 발행한다. 리프트 노드를 실행하기 전에는 높이 `0 m`를 발행해
기존 시뮬레이션과 같은 기준 자세를 유지한다. 승강판 기본 치수와 위치, 리프트
최대 높이는 `rb5_farmily.urdf.xacro`의 `lift_platform_*`, `lift_max_height`
인자로 조정할 수 있다.

MoveIt을 `use_fake_hardware:=true`로 실행하면 lift joint bridge도 자동으로
시뮬레이션 모드가 된다. 이때 GUI는 실제 리프트 launch와 Bottom calibration을
비활성화하고, 목표 높이를 `/lift_simulation/control/move_height`에만 발행한다.
따라서 `/lift_control/move_height`에는 명령이 전달되지 않으며 실제 모터는
움직이지 않고 RViz의 초록색 승강판과 로봇 모델만 이동한다. 실제 모드에서는
시뮬레이션 토픽을 사용하지 않고 기존 리프트 높이와 calibration 상태를 따른다.

`장면 · 속도 · 리프트` 탭의 `그리퍼 스트로크 제어`는 실제 RB 제어기의
`/rbpodo_hardware/eval` 서비스와 `arduino_linear_motor` 노드를 함께 사용한다.
GUI의 `Arduino 노드 실행`은
`ros2 launch arduino_linear_motor pin89_serial.launch.py port:=/dev/ttyUSB0`
를 별도 프로세스로 시작한다. `/pin89_serial_node`가 이미 있으면 실행 버튼은
자동 비활성화되며, `/linear_motor/serial_status`의 `connected` 또는 TTY 오류를
상태 영역에 표시한다.

RB 제어기는 `set_dout_bit_combination(0,15,256,0)`을 사용해 DOUT8만 HIGH로
유지하고 나머지 DOUT을 모두 LOW로 만든다. 실제 모터 방향은 RB DOUT이 아니라
Arduino 토픽으로만 제어한다.

출력 조합은 다음과 같다.

- `늘림`: PIN8/9 모두 LOW 후 `/linear_motor/pin8=true`, `pin9=false`
- `줄임`: PIN8/9 모두 LOW 후 `/linear_motor/pin8=false`, `pin9=true`
- `정지`: `/linear_motor/pin8=false`, `/linear_motor/pin9=false`

Arduino TTY 연결, PIN8/9 토픽 구독, RB eval 서비스 및 DOUT8 초기화가 모두
완료되기 전에는 늘림·줄임·정지 버튼을 활성화하지 않는다. 늘림 또는 줄임은
자동으로 꺼지지 않으므로 원하는 위치에 도달하면 반드시 `정지`를 눌러 두 Arduino
핀을 LOW로 내려야 한다.

`로봇 이동 속도`의 기본값은 RB Speed Bar `10%`, OMPL/Joint 속도와
가속도 각각 `20%`이다. `속도 적용`을 누르면 계획 속도·가속도는 다음
Plan부터 개별 수확, 전체 연속 수확 및 자동 테스트에 모두 전달된다. RB Speed
Bar는 `/rbpodo_hardware/set_speed_bar` 서비스로 즉시 요청되며 실제 로봇의
모든 실행 구간에 영향을 준다. 하드웨어 노드가 없는 환경에서는 계획 속도만
적용된다. 이 경우 GUI는 시뮬레이션에서 정상적인 서비스 미연결임을 알리고,
`OMPL/Joint 적용`, `RB Speed Bar 미적용`, `Cartesian 미적용` 범위를 각각
상태와 로그에 명시한다. Cartesian 경로는
MoveIt Humble 서비스에서 별도 scaling 필드를 제공하지 않으므로 계획 배율은
OMPL/Joint 구간에 적용되고, 실제 Cartesian 실행 속도는 RB Speed Bar로
조절한다.

자동 테스트의 `자동 테스트 + 로봇 정지` 버튼과 수확 영역의
`현재 수확 모션 정지` 버튼은 남은 작업을 지우는 것에 더해 MoveIt
`/execute_trajectory`와 ros2_control joint trajectory controller의
활성 action goal을 모두 취소한다. 실제 로봇에서는 이어서
`/rbpodo_hardware/task_stop`도 호출한다. 시뮬레이션에서는 하드웨어 서비스가
없으므로 두 trajectory action 취소만 수행한다. GUI 로그에는 각 취소 요청의
전송 및 수락 여부와 실제 RB5 정지 결과가 별도로 기록된다. 이는 네트워크와
소프트웨어를 통한 운전 정지이며 안전등급 비상정지가 아니므로, 위험 상황에서는
항상 교시기 또는 설비의 비상정지를 사용해야 한다.

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
기본 수확 순서는 `PICK_READY → Cartesian pre-approach`이다.
접근 목표 이후 tip 로컬 수확 동작은 `+X 40 mm → (+Z 20 mm, +X 20 mm)
→ +Z 20 mm → -X 50 mm → -X 10 mm → 리니어모터 대기` 순서다.
괄호로 묶인 X/Z 변화량은 각각 하나의 Cartesian 대각선 이동으로 동시에 적용한다.
`PICK_READY` 관절값은
`rbpodo_moveit_config/config/rbpodo.srdf`의
`PICK_READY/mainpulation` named state를 단일 원본으로 사용한다. 수확 planner와
fake hardware 초기 자세가 모두 이 값을 읽으므로 SRDF를 수정한 뒤 두 패키지를
다시 빌드하고 MoveIt을 재시작해야 한다.
tip 기준으로 계산한 pre-grasp pose는 고정된 TCP-to-tip transform을 사용해
TCP 목표 pose로 환산한다. GUI는 PICK_READY의 TCP 자세에서 이 목표까지
위치와 자세를 한 경로에서 함께 변경한다. 별도의 TCP 제자리 회전 trajectory는
생성하지 않으며 `Cartesian 우선 + constrained OMPL fallback` 알고리즘으로
고정되어 있다.
이 direct Cartesian pre-approach가 실패하면 원래 PICK_READY 상태에서 같은
TCP 목표까지 constrained OMPL RRTConnect로 한 번에 재계획하며, 이것도 실패할
때 전체 Plan을 실패 처리한다.
pre-approach, 수확 및 대기 후 후퇴 중 Cartesian 경로가 실패하면 해당 구간만
OMPL RRTConnect로 자동 재계획한다. 여러 waypoint가 포함된 수확 구간은 동작을
생략하지 않고 waypoint별로 OMPL을 순차 적용한다. 모든 OMPL fallback에는 각
단계 시작 자세 중심의 `±120°` 관절 path constraint가 동일하게 적용된다.
Cartesian 실패 후 OMPL fallback이 성공하면 전체 Plan은 성공으로 처리하고,
결과 로그의 `cartesian_fallbacks`와 단계별 기록에 전환 구간을 남긴다.
최초 PICK_READY 진입은 OMPL RRTConnect를 사용한다.
Pre-approach 이후의 접근과 수확 동작은 우선 TCP Cartesian 경로를 시도한다.
각 OMPL 단계에는 그 단계의 시작 자세를 중심으로 `base`, `shoulder`, `elbow`,
`wrist1`, `wrist2`를 `±120°`로 제한하는 path constraint가 적용된다.
`wrist3`는 이 제한에서 제외되며 기존 로봇 관절 범위를 사용한다.
같은 시작 자세 중심 constraint는 `GetCartesianPath.path_constraints`에도
적용된다. Cartesian은 상대 jump threshold `2.0`, revolute 절대 jump threshold
`20°`를 사용한다. MoveIt이 성공을 반환하더라도 실행 전에 모든 trajectory를
다시 검사하여 base~wrist2의 관절 span이 `120°`, wrist3 span이 `180°`, 인접
point의 관절 변화가 `45°`를 넘으면 경로를 폐기한다. Cartesian 경로가 이 검사에
걸리면 기존 constrained OMPL fallback으로 전환하며, 캐시된 정방향·역방향
trajectory도 실제 실행 직전에 동일한 검사를 다시 수행한다.
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

Pre-grasp 접근 방향은 토마토 TF의 `-X`를 최우선으로 사용한다. 먼저 0°
pre-grasp의 IK를 충돌 검사와 PICK_READY 기준 `±120°` 관절 constraint를
포함해 검사하고, 불가능할 때만 토마토 로컬 `+Y`/`-Y` 방향의 보정각을
늘린다. 양쪽을 같은 절댓값 순서로 검사해 가능한 구간을 찾고 그 구간을 다시
좁혀, 가능한 자세 중 보정각이 가장 작은 방향을 선택한다. 같은 보정각이면
PICK_READY와 관절 이동량이 작은 IK 해를 우선한다. 이 후보 검사에서는 OMPL을
호출하지 않으며, 선택된 최종 자세에 대해서만 기존 Cartesian/OMPL 계획을
수행한다. signed 회전각은 local `+Y` 방향이 양수, local `-Y` 방향이 음수다.
원본 토마토 TF는 변경하지 않고 tip target과 pre-grasp geometry만 회전한 뒤
기존과 동일하게 TCP pose로 환산한다. `/compute_ik`를 사용할 수 없거나 유효한
후보가 하나도 없을 때만 기존 로봇 방향 기반 보정값으로 fallback한다.

수확 옵션의 `진입각: Recommend보다 로봇 방향 우선`을 체크하면 후보 탐색 순서를
반대로 적용한다. `최대 보정각` 입력값 안에서 먼저 로봇 방향에 가장 가까운 각도를
검사하고, 해당 IK가 불가능하면 Recommend 쪽으로 각도를 줄여가며 가장 로봇 쪽에
가까운 유효 경계를 선택한다. 로봇 방향이 최대각보다 가까우면 필요한 각도까지만
회전하며, 입력 범위는 `0~90°`, 기본값은 `45°`이다. 체크를 해제하면 기존처럼
유효한 최소 보정각을 선택한다. 이 두 값은 개별 수확, 전체 연속 Plan·수확,
스텝·접근 반복과 자동 테스트에 동일하게 적용된다.

추천 0° pre-grasp가 토마토 중심을 지나면서 토마토→로봇 베이스 방향에
수직인 deadline의 반대편에 있거나, 로봇 쪽이더라도 기본 15° 안전 영역을
확보하지 못하면 deadline guard가 활성화된다. 이 경우 deadline과 만나는
`ideal` 각도에 안전 여유각을 더한 값을 최소 보정각으로
사용하고, 로봇 쪽으로 향하는 동일 부호의 최소각~최대각 범위만 검사한다.
반대 방향과 ideal보다 추천 방향에 가까운 후보는 IK 가능 여부와 관계없이
제외한다. 설정된 최대각으로도 deadline을 지킬 수 없으면
`DEADLINE_REQUIRES_ANGLE_OVER_MAXIMUM`으로 즉시 실패한다.

- `adaptive_grasp_enabled`: 적응형 접근각 사용 여부, 기본 `true`
- `adaptive_grasp_max_rotation_deg`: 추천 진입각 기준 최대 보정각, 기본 `45.0`
- `adaptive_grasp_prefer_robot_direction`: `true`이면 최소 보정각 대신 허용 범위의
  유효 후보 중 로봇 방향에 가장 가까운 각도를 선택, 기본 `false`
- `adaptive_grasp_deadband_deg`: geometric fallback의 기존 방향 유지 범위, 기본 `10.0`
- `adaptive_grasp_ik_timeout_sec`: 후보 하나의 IK 제한 시간, 기본 `0.05`
- `adaptive_grasp_ik_service_wait_sec`: `/compute_ik` 연결 대기, 기본 `0.5`
- `adaptive_grasp_search_step_deg`: 최초 가능 구간 탐색 간격, 기본 `10.0`
- `adaptive_grasp_search_resolution_deg`: 최소각 경계 정밀도, 기본 `1.0`
- `adaptive_grasp_deadline_margin_deg`: ideal에서 로봇 쪽으로 더하는 안전 여유각,
  기본 `0.0`. 이에 따라 실제 3차원 `토마토→로봇 베이스` 방향을 중심으로
  `±90°`, 전체 180° 진입 영역을 허용한다. deadline 평면 뒤쪽의 진입은 계속
  금지하지만 deadline 안쪽의 추가 안전 여유각은 적용하지 않는다.

Plan 결과의 `adaptive_grasp` 항목과 자동 테스트 CSV/JSONL에는 적용 회전각과
회전 전후 로봇 방향 오차, IK 검사 횟수 및 각 후보 결과가 기록된다.

`검출 토마토 전체 연속 수확`은 `토마토별 종료 단계` 선택과 관계없이 현재 검출된
모든 토마토를 0번부터 순서대로 처리한다.
각 토마토마다 Plan-only를 먼저 수행하고 성공한 경우에만 실제 수확한다. Arc가
해제되어 있으면 매 토마토 후 `PICK_READY`로 복귀하고, Arc가 켜져 있으면 검출
목록의 마지막 토마토에서만 복귀한다. 계획에 실패한 토마토는 실제로 움직이지 않고
건너뛴 뒤 다음 토마토를 계속 처리한다. 실제 실행이 실패하거나 작업 도중 새
검출 결과가 들어오면 남은 수확은 실행하지 않고 즉시 중단한다.

RViz의 `HarvestPlanResults` 화살표는 성공·실패 여부와 관계없이 planner가
실제로 사용한 토마토 로컬 pre-grasp 진입 벡터를 표시한다. 보정이 없으면
토마토 로컬 +X축이다. local `+Y` 쪽으로 pre-grasp 위치가 보정되면 화살표는
`+X→-Y`, local `-Y` 쪽으로 보정되면 `+X→+Y` 방향으로 회전한다.
pre-grasp 위치 벡터와 화살표가 나타내는 토마토 방향 진입 벡터는 서로
반대이기 때문이다. 성공 화살표 길이는 검출된 중심→줄기점 거리를 지면에 투영한
뒤 8 mm를 줄인 값을 사용한다. 실패 화살표는 보정 후 최종
`preapproach_position → target_position`에서 방향을 직접 계산하고, 함께 표시되는
주황색 실제 접근 화살표와 동일한 길이를 사용한다. 최종 접근 geometry가 생성되기
전에 실패하면 하늘색 검출 화살표와 같은 60 mm 길이를 사용한다. 빨간색 화살표만
꼬리를 토마토 뒤쪽에 두고 화살표 촉이 토마토 중심을 바라보도록 배치한다.
Plan-only 회전 없는 성공은 초록색, 적응 접근각이 적용된 성공은 하늘색, 실패는
빨간색이다.
결과가 바뀔 때만 Transient Local 마커를
발행하므로 깜빡이지 않고 유지된다. 새 검출 결과가 들어와도 기존 결과 마커는
유지되며, GUI의 `결과 마커 지우기` 버튼을 눌렀을 때 전체 마커를 삭제한다.
planner를 바꾸면 기존 계획 결과를 무효화하고 마커를 지운다.

오른쪽 `줄기 위치/회전 Plan 자동 테스트` 패널에는 X/Y/Z/회전의 시작값,
종료값, 변화량을 입력한다. X/Y/Z/회전 모두 `랜덤`을 선택할 수 있다. 랜덤으로
선택한 항목은 생성되는 각 케이스마다 시작~종료 범위에서 임의 값을 사용하고,
변화량과 종료 조건에서는 제외한다. 체크하지 않은 X/Y/Z 항목은 시작부터
종료까지 일정하게 변화량만큼 이동하며, 가장 먼저 종료값에 도달한 축을 기준으로
XYZ 위치 생성을 종료한다. 비랜덤으로 변경되는 X/Y/Z 축이 없으면 랜덤 XYZ
위치 하나를 생성한다. 회전이 비랜덤이면 각 XYZ 위치에서 시작~종료 회전 범위를
모두 검사하고, 회전도 랜덤이면 각 XYZ 위치마다 회전값 하나를 임의로 추출한다.
따라서 Y와 Z를 동시에 랜덤으로 설정해도 자동 테스트를 시작할 수 있다. 자동 실행 시 각
케이스마다 줄기 장면
적용과 촬영/검출을 수행한 뒤,
토마토 0번부터 7번까지 차례로 처리한다. `실제 로봇 실행` 체크박스는 기본
해제되어 Plan-only로 동작한다. 체크하면 시작 전 안전 확인창을 표시하고,
Plan에 성공한 각 토마토 trajectory를 실제로 실행한 뒤 `PICK_READY`로
복귀한다. 계획 실패는 다음 토마토로 넘어가지만 실제 trajectory 실행이
실패하면 로봇 자세가 불확실할 수 있으므로 자동 테스트를 즉시 중단한다.
회전 없는 성공은 초록색,
적응 접근각 성공은 하늘색, 실패는 빨간색
화살표로 표시하며, 각 화살표는 생성 당시 `world` 좌표에 고정되어 다음 케이스로
줄기가 이동해도 기존 위치에 누적된다. 자동 테스트에서는 처리 부하를 줄이기 위해
`/display_planned_path` 궤적 애니메이션을 발행하지 않으며, 개별 Plan-only와 실제
수확에서는 기존처럼 전체 궤적을 발행한다. 또한 자동 테스트가 시작될 때 Planner
프로세스를 한 번만 생성하고 모든 토마토와 위치/회전 케이스에서 같은 MoveIt
액션 클라이언트를 재사용한다. 토마토마다 ROS 프로세스를 빠르게 생성·종료할 때
발생하던 DDS goal 응답 유실과 30초 타임아웃 지연을 방지한다.

자동 테스트 중에는 GUI 오른쪽에서 완료 수, 성공/실패 수, 성공률, 평균 Plan
시간과 실패 단계별 개수를 실시간으로 확인할 수 있다. 최근 100개 결과는 케이스,
토마토 번호, 결과, 실패 단계, 계획 소요시간과 실제 전체 시퀀스 시간으로
표시된다. Plan-only 결과의 실제 시퀀스 시간은 `-`로 표시하고, 실제 실행 시에는
PICK_READY 진입부터 수확·대기·후퇴·PICK_READY 복귀 완료까지 측정한다.
결과는 각 Plan 직후
`~/farmily_tomato/harvest_results/<실행시각>/` 아래에 즉시 저장된다.
세션 JSON/CSV에는 실제 실행 요청·시도·성공 여부와 실행 시간도 기록된다.

- `session.json`: 입력 범위, 변화량, 랜덤 seed, 실제 생성된 모든 케이스
- `results.jsonl`: 단계별 세부 진단을 포함한 원본 결과
- `results.csv`: 스프레드시트 분석용 요약 결과
- `summary.json`: 완료 시 성공률과 실패 단계별 합계

실패 단계는 `OMPL_PICK_READY`, `CARTESIAN_PREAPPROACH`,
`OMPL_PREAPPROACH`, `CHOMP_PREAPPROACH`,
`PILZ_INDUSTRIAL_MOTION_PLANNER_PREAPPROACH`, `CARTESIAN_APPROACH`,
`OMPL_CONTINUOUS_PREAPPROACH`,
`CARTESIAN_POST_WAIT`, `OMPL_RETURN_PICK_READY`, `TF_TARGET` 등으로
구분된다.
Cartesian 실패에는 경로 fraction과 MoveIt 오류 코드가, OMPL/CHOMP/PILZ
실패에는 pipeline, planner ID와 MoveIt 오류 코드가 기록된다.

수확 후 마지막 post-wait 자세에서 `PICK_READY` joint goal까지 OMPL로
복귀한다. 복귀 시작 자세를 중심으로 wrist3를 제외한 관절에 기존
`±120°` path constraint와 충돌 검사를 적용한다. 복귀 계획이 실패하면
`failure_stage=OMPL_RETURN_PICK_READY`로 기록하고 전체 Plan을 실패 처리하며,
Plan-only/RViz에서 검증된 동일 trajectory를 실제 수확 마지막에 실행한다.

저장된 세션을 한눈에 확인할 수 있는 HTML 분석 보고서는 다음 명령으로 생성한다.

```bash
ros2 run rbpodo_tomato_harvest harvest_report \
  ~/farmily_tomato/harvest_results/20260723_124228_043000
```

세션 폴더에 `report.html`이 생성된다. 보고서에는 전체 성공률, 실패 단계와 이유,
토마토 번호별 성공률, 회전 구간별 성공률, 회전–Y 위치 분포와 Cartesian fraction이
낮은 실패 사례가 포함된다. 외부 웹 서버나 추가 파이썬 패키지는 필요하지 않다.

`수확 작업` 탭의 `검출 소스`에서 다음 두 서비스를 즉시 선택할 수 있다. 소스를
바꾸면 서로 다른 카메라의 이전 좌표로 수확하지 않도록 기존 검출 목록과 Plan
상태를 초기화하며, 이후 개별 검출·전체 연속 수확·자동 테스트는 모두 선택된
서비스를 사용한다.

- `Fake tomato`: `/fake_tomato_camera/detect_tomatoes`
- `실제 /detect_tomatoes`: `/detect_tomatoes`

`검출 마커 표시`는 기본 체크되어 있다. 체크된 상태로 검출하면 메시지의
`header.frame_id`와 각 토마토의 `center`, `stem_point`를 사용해 다음 마커를
`/detected_tomato_markers`에 발행한다.

- 핑크색 구체: 지름 `0.0175 m`의 토마토 중심
- 초록색 구체: 지름 `0.006 m`의 줄기 좌표
- 하늘색 화살표: 줄기 반대편에서 토마토 중심으로 들어오는 `0.06 m` 진입 방향

카메라 검출 단계에서는 아직 Planner가 확정하지 않은 접근 자세를 예측하지
않으므로 주황색 접근 화살표를 발행하지 않는다. 하늘색 화살표는 검출된
`center→stem_point` 축을 반대로 연장한 비전 기준 진입 방향이다.
Plan을 실행해 정확한 접근 좌표가 계산되면 `/harvest_result_markers`에 주황색
화살표를 발행한다. Plan 결과 화살표의 시작점은 Planner가 계산한
preapproach 위치이다. 실제 접근 방향을 유지한 상태로 토마토 중심에 가장 가까운
지점까지 시각적으로 연장하여 핑크색 중심 마커와 떨어져 보이지 않게 한다. 이
연장은 마커 표시에만 적용되며 실제 trajectory의 목표 위치는 변경하지 않는다.
Plan이 목표 형상을 계산하기 전에 실패한 경우에는 Plan 결과 화살표를 표시하지
않는다.

새 검출은 이전 마커를 교체하며, 체크를 해제하거나 카메라 소스·장면 위치를
변경하면 즉시 삭제한다. MoveIt RViz 설정에는 `DetectedTomatoPreview`
MarkerArray display가 기본 등록되어 있다.

실제 `/detect_tomatoes` 서비스가 검출 결과를 서비스 응답으로만 반환해도 GUI가
그 결과를 `/tomato_detection/detections`에 다시 발행한다. 따라서 핑크색 마커와
`tomato_tf_generator`가 동일한 검출 스냅샷을 사용한다. 또한 MoveIt 런치는
기본적으로 `d435_link -> camera_link` 고정 TF를 발행하여 RealSense 드라이버의
`camera_color_optical_frame`을 로봇 TF 트리에 연결한다. 이미 외부에서 두 TF
트리를 연결한다면 `bridge_realsense_driver_tf:=false`로 비활성화할 수 있다.

기본 선택은 `Fake tomato`이다. GUI 시작 시 실제 카메라를 기본으로 선택하려면
다음과 같이 실행한다.

```bash
./scripts/run_harvest_gui.sh \
  default_camera_source:=real
```

Fake 또는 실제 카메라 팀의 서비스 이름이 다른 경우 각각 `camera_service`와
`real_camera_service` launch 인자로 변경할 수 있다.

```bash
./scripts/run_harvest_gui.sh \
  camera_service:=/simulation/detect_tomatoes \
  real_camera_service:=/real_camera/detect_tomatoes
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

RB5 MoveIt launch는 기본적으로 `world`에 고정된 작업 공간 collision을 생성한다.
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
2. Cartesian-first motion to the pre-approach TCP pose
3. Cartesian motion from pre-approach through the tomato harvest sequence
4. Cartesian post-wait retreat

The GUI fixes the motion from `PICK_READY` to the pre-approach pose to a
Cartesian-first path. Entering `PICK_READY` uses OMPL RRTConnect. If any
Cartesian segment fails, that segment is retried with constrained OMPL
RRTConnect. Multi-waypoint harvest
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

1. Move +70 mm along tip X.
2. Move +40 mm along tip Z.
3. Move -50 mm along tip X.
4. Move +10 mm along tip Z.
5. Hold for the configured `harvest_wait_sec` duration (GUI default: 2 seconds).
6. Move -10 mm along tip X.
7. Return directly to all PICK_READY joint targets with PILZ PTP.

The dwell separates the Cartesian motion into pre-wait and post-wait
trajectories. Plan-only mode publishes the complete trajectory sequence to RViz
without waiting or moving the robot. Inspect it before explicitly enabling
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
