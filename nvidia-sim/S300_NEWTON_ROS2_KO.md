# Newton GPU 시뮬레이터 ↔ ROS 2

현재 Newton GPU 시뮬레이터는 **ROS 명령 제어 모드**와 **자동 순회 + 상태 발행 모드**를 지원한다.
관절, 기체/리프트 위치, TCP, TF, 시뮬레이션 시각, 장착 카메라 RGB·깊이·GT 마스크를 수신할 수 있다.
실제 모터 드라이버는 실행하지 않는다. 아래 명령은 시뮬레이터의 실제 GPU actuator로 전달된다.

## ROS 명령으로 로봇 움직이기

자동 순회 대신 ROS가 명령을 결정하는 실행 모드다. 로봇암·기체·리프트·그리퍼를
ROS 프로그램에서 제어하며 기존 Newton GPU 솔버와 actuator/충돌 검사를 사용한다.
자동 수확 플래너가 외부 명령을 덮어쓰지 않는다.

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh ros-control
```

5905 화면에서 자세를 선택하고 **ROS 제어 시작**을 누른다. 준비가 끝나면
`명령 수신 중` 상태에서 기다린다. `--headless`는 화면 없이 바로 초기화한다.
명령이 없어도 물리와 상태/카메라 토픽이 계속 실행된다. 기본 종료 시간 제한은 없고,
`--seconds 60`은 준비 이후 wall-clock 60초 시험이다. `Ctrl+C` 또는 화면의
`ROS 제어 종료`로 종료한다. 다른 실행이 5905를 사용 중이면 먼저 종료한다.

다른 터미널에서 다음 명령들을 사용할 수 있다. 기본 domain42다.

```bash
cd /root/farmily_tomato

# 로봇암 손목을 지정 각도(rad)까지 2초 동안 이동. 기본 엘보우 위 자세의 작은 시험 동작.
./nvidia-sim/run_s300_ros2.sh topic pub --once \
  /farmily/sim/arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [wrist3], points: [{positions: [-1.17], time_from_start: {sec: 2}}]}"

# 레일 정방향으로 0.06m/s 전진. Ctrl+C로 발행을 끊으면 감속해 정지.
./nvidia-sim/run_s300_ros2.sh topic pub --rate 10 \
  /farmily/sim/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.06}}"

# 리프트 직동 관절의 절대 높이0.22m
./nvidia-sim/run_s300_ros2.sh topic pub --once \
  /farmily/sim/lift/command std_msgs/msg/Float64 "{data: 0.22}"

# 그리퍼 안쪽 간격96mm로 열기
./nvidia-sim/run_s300_ros2.sh topic pub --once \
  /farmily/sim/gripper/command std_msgs/msg/Float64 "{data: 0.096}"

# 진행 명령을 취소하고 정지 유지
./nvidia-sim/run_s300_ros2.sh topic pub --once \
  /farmily/sim/stop std_msgs/msg/Bool "{data: true}"

# 새 명령을 다시 허용. 취소된 이전 경로는 재개하지 않는다.
./nvidia-sim/run_s300_ros2.sh topic pub --once \
  /farmily/sim/stop std_msgs/msg/Bool "{data: false}"

# 수락/거절/완료/선점/타임아웃 및 실제 관절 피드백
./nvidia-sim/run_s300_ros2.sh topic echo /farmily/sim/command_feedback
./nvidia-sim/run_s300_ros2.sh topic echo /farmily/sim/joint_states
```

- `/arm_controller/joint_trajectory`는 `base, shoulder, elbow, wrist1, wrist2, wrist3` 중 이름으로 선택한 관절을 제어한다.
- `/farmily/sim/joint_trajectory`는 같은 메시지로 전체 10축 중 일부/전부를 제어한다.
  `root_joint`는 기체 레일 위치(m), `farmily_lift_height_joint`는 리프트(m), `jaw_L/R`은 좌우 직동 관절(m)이다.
  현재 관절 이름/위치는 `joint_states`를 기준으로 사용한다. 이름 순서와 positions 순서를 맞춰야 한다.
- 현재 기체는 레일1축 모델이다. `cmd_vel.linear.x` 양수=입구에서 전진, 음수=후진이며 월드Y 방향으로 움직인다.
  나머지 linear/angular 성분은0이어야 한다. 평면 회전/횡이동을 지원하는 이동체 모델은 아니다.
- `cmd_vel`은 기본 wall-clock0.5초 동안 새 메시지가 없으면 감속 정지한다(`--cmd-vel-timeout`).
  단일 arm trajectory는 발행 노드가 종료돼도 지정 경로를 완료한다. ROS 브리지 연결 자체가 끊기면 모든 축의 진행 명령을 취소하고 정지한다.
- 유효 범위: 기체0..등록 종점(기본3.85m), 리프트0..0.75m, 그리퍼 간격0.028..0.096m.
  기체/리프트 속도 최대0.12m/s, 가속도0.3m/s² 등 기존 시뮬레이션 명령 한도를 유지한다. 실제 기구 한도 인증값은 아니다.
- `JointTrajectory`는 positions 필수, 선택적 velocities/accelerations를 받는다. positions만 있으면 각 점에서 정지하는 quintic 보간,
  velocity까지 있으면 cubic, acceleration도 있으면 quintic이다. 전체 구간의 위치/속도/가속도 극값을 검사한다.
  잘못된 이름/NaN/범위 초과/너무 짧은 이동 시간/effort 명령은 거절한다. 마지막 점은 정지 상태여야 한다.
  최대512점/300초 경로, header stamp0은 즉시 시작이며 지정 stamp는 `/clock` 기준이다.
- 완료 판정은 보낸 목표만 보는 것이 아니라 실제 GPU 관절 위치·속도의 도달 여부를 확인한다.
  이 토픽은 충돌 없는 경로를 생성하는 OMPL/MoveIt 플래너 자체가 아니다. 주줄기/기체 접촉의 기존1mm 검사와 solver 유효성 검사는 유지된다.
- `command_feedback` JSON은 최근64개 이벤트를 포함한다. 이벤트 번호로 중복을 제거하면 된다.
  실행 폴더 `commands.jsonl`에는 전체 판정이 기록되고 `states.jsonl`에는 실제 관절 상태/목표가 기록된다.

카메라는 이 모드에서 기본 시뮬레이션1Hz(`--camera-hz`,0이면 끔)로 실제 RGB-D/GT를 촬영한다.
물리와 별도 작업 스레드로 촬영하며 노출 시각의 TF와 함께 발행한다. 렌더 지연이 있어도
`/clock`이나 최신 `joint_states`를 과거 시각으로 되돌리지 않는다. GUI는30Hz, 관절/TF는 기본20Hz 목표다.
물리 계산은220Hz이며 GPU 기록 버퍼는2스텝으로 재사용하므로 세션 길이에 비례해 GPU 메모리를 늘리지 않는다.
초기 자세는 모델의 시작 자세다. 자동 순회의 촬영 자세로 임의 이동하지 않으므로 처음에는
카메라에 로봇/구조물이 크게 보일 수 있다. 원하는 관측 자세도 ROS 관절 명령으로 지정한다.
`observations.timestamp_s`는 노출 시각, `current_timestamp_s`는 브리지가 알고 있는 최신 시뮬레이션 시각,
`age_s`는 두 시각의 차이다. `/clock`과 비교하면 구독 시점의 추가 지연도 확인할 수 있다.

ROS 제어 예제는 현재 위치를 확인한 뒤 사용한다. `JointTrajectory` 토픽 입력을 지원하며,
MoveIt/ros2_control의 `FollowJointTrajectory` 액션 서버나 실제 모터 드라이버 연결까지 구현한 것은 아니다.

## 자동 순회 + 상태 발행 실행

저장소 `/root/farmily_tomato`에서 최초 1회:

```bash
./nvidia-sim/run_s300_ros2.sh check
./nvidia-sim/run_s300_ros2.sh build
```

시뮬레이터 실행. 브라우저 `http://서버주소:5905`에서 **순회 시작**을 누른다.
이미 같은 포트로 실행 중이면 기존 실행을 종료하고 시작한다.

```bash
./nvidia-sim/run_s300_newton.sh camera-patrol --ros2 \
  --end-station 3.85 --max-harvests 1000 --max-decisions 10000
```

`--headless`를 추가하면 GUI 없이 자동 시작한다. 짧은 연동 확인은
`--end-station 0.5 --max-harvests 1 --max-decisions 100`으로 실행한다.
현재 기본 자산의 순회 등록 범위는 3.85m다.

**토픽 발행은 START 후 모델·카메라 준비가 끝나면 시작한다.** GUI의 자세 선택 대기 중에는 아직 발행하지 않는다.
순회가 끝나면 ROS 노드도 종료한다. GUI 최종 화면이 남아 있어도 과거 영상을 새 센서 데이터로 발행하지 않는다.

다른 터미널에서:

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_ros2.sh topic list
./nvidia-sim/run_s300_ros2.sh topic echo /farmily/sim/joint_states --once
./nvidia-sim/run_s300_ros2.sh inspect --seconds 60
```

`inspect`는 실제 DDS 토픽을 구독해 수신 개수를 출력한다. 선택적으로
`--output /tmp/farmily_ros_received`를 붙이면 수신 감사 JSON과 마지막 이미지 배열을 저장한다.
시험 전체를 받으려면 시뮬레이터 START 전에 실행하고 `--until-stopped --seconds 600`을 사용한다.

## 토픽

기본 namespace는 `/farmily/sim`, **ROS_DOMAIN_ID는 42**다.
helper는 현재 셸의 다른 domain 값과 관계없이 기본 42를 쓴다.
`--ros-domain-id 43`을 선택했으면 구독 helper에도 `S300_ROS_DOMAIN_ID=43`을 준다.
namespace 변경은 실행기 `--ros-namespace`, inspector `--namespace`로 맞춘다.

| 토픽 | ROS 타입 / 내용 |
| --- | --- |
| `/farmily/sim/joint_states` | `sensor_msgs/JointState`: 실제 관절 위치·속도. 기체 root_joint, 리프트, 6축, 양쪽 그리퍼 총 10개. 직동 m, 회전 rad |
| `/farmily/sim/tool_pose` | `geometry_msgs/PoseStamped`: `Tool_fixed`의 월드 자세 |
| `/tf` | `tf2_msgs/TFMessage`: 로봇·툴 및 카메라 optical frame 변환 |
| `/clock` | `rosgraph_msgs/Clock`: 물리 시뮬레이션 시각 |
| `/farmily/sim/camera/color/image_raw` | `sensor_msgs/Image`, `rgb8` |
| `/farmily/sim/camera/depth/image_raw` | `Image`, `32FC1`: optical Z 깊이, m, 유효하지 않은 픽셀 NaN |
| `/farmily/sim/camera/instance/image_raw` | `Image`, `32SC1`: 가시 객체별 GT ID. 아래 signed ID 규칙 참고 |
| `/farmily/sim/camera/semantic/image_raw` | `Image`, `mono8`: GT class ID |
| `/farmily/sim/camera/recognition/image_raw` | `Image`, `rgb8`: 기존 토마토·줄기 표시 오버레이 |
| `/farmily/sim/camera/camera_info` | `sensor_msgs/CameraInfo`: 실제 출력 크기와 K, 무왜곡 D, R, P |
| `/farmily/sim/camera/observations` | `std_msgs/String` JSON: 해당 노출의 가시 객체·숙도 proxy·깊이/마스크 정보·카메라 외부행렬 |
| `/farmily/sim/camera/labels` | `String` JSON: instance ID → 원본 ID, 객체/그루/송이/클래스/색 대응표 |
| `/farmily/sim/status` | `String` JSON: 현재 정차 위치, 실행 상태, 수확 동작/영역 수확 개수 |

카메라·관측·CameraInfo·clock은 **best effort**, 깊이 2다. 이미지 구독자는
`qos_profile_sensor_data` 또는 `BEST_EFFORT` QoS를 사용한다.
관절·TCP·TF는 reliable, labels/status/command_feedback는 reliable + transient local이다. 제어 명령 구독은 reliable/volatile이다.

```bash
./nvidia-sim/run_s300_ros2.sh topic echo /farmily/sim/camera/camera_info \
  --qos-reliability best_effort --once
```

이미지·CameraInfo는 같은 실제 노출 시각과
`farmily/sim/camera_optical_frame`을 사용한다. 해당 시각의 TF도 함께 발행한다.
optical 축은 x 오른쪽, y 아래쪽, z 전방이다. 외부 노드의 시뮬레이션 시각 사용은
`use_sim_time:=true`로 설정한다.
메시지 의미는 ROS 공식 [Image 정의](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)와
[CameraInfo 정의](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg)를 따른다.

자동 순회의 주기적 관절/TF 표본은 기본 시뮬레이션 10Hz(`--ros-state-hz`, 최대 설정 60Hz)이며
노출 시각과 구간 경계 표본이 추가된다. 최초 t=0은 입력 초기 자세이고, 이후 표본은 GPU 실행에서 읽은 실제 상태다.
계획·카메라 준비 때문에 물리가 멈춘 동안 clock을 임의로 전진시키지 않는다.

**자동 순회의 카메라는 기존 실제 촬영 이벤트에서만 발행한다. 고정 30FPS 스트림이 아니다.**
기존 순회의 이동 중 관측과 정차 촬영을 사용하므로 촬영 사이에는 이미지가 오지 않는다.
새 타임스탬프를 붙여 과거 프레임을 반복하지 않고 추가 렌더도 하지 않는다.
기본 출력은 240×320이며 실제 크기/K는 CameraInfo를 기준으로 사용한다.
같은 물리 시각의 이동 종단 촬영과 정차 재촬영이 존재할 수 있다.

### instance ID

원본 ID는 uint32 해시라 2³¹ 이상 값도 있다. ROS `32SC1`로 발행할 때
**비트는 그대로 두고 signed int32로 해석**하므로 음수 ID도 정상 객체다.
배경은 0이며, `instance > 0` 대신 `instance != 0`으로 객체를 구분한다.
`labels.instances`의 키와 `observations.objects[].instance_id`는 ROS signed ID다.
`source_instance_id`는 기존 uint32 값을 보존한다. semantic class 대응표도 labels에 포함된다.
이미지 색상으로 분류하거나 ML 모델을 실행하지 않고 기존 가시 GT 데이터를 전달한다.

## 구성과 실행 부담

```text
Newton GPU 순회 (기존 Python 3.12)
  → 읽기 전용 상태 표본 + 기존 실제 RGB-D/GT 노출
  → 개수 제한된 비동기 Unix socket 전송
  → ROS 2 Humble 발행 프로세스 (/usr/bin/python3, Python 3.10)
  → DDS 토픽 구독

ROS 제어 모드: DDS 명령 → 별도 ROS 프로세스 → 순서가 보존되는 제한 길이 명령 큐
  → 이름·범위·속도·가속도 검증 → 기존 GPU actuator → 실제 상태 피드백
```

rclpy를 Newton 가상환경에 넣지 않는다. 별도 ROS 프로세스가 DDS 처리를 맡는다.
느린 구독/전송 때문에 상태를 무한히 쌓지 않으며 데이터 종류마다 가장 최근 대기 표본만 유지한다.
합쳐진 표본 수는 실행 폴더 `ros2/transport.json`의 `dropped`에 기록한다.
자동 순회에서는 구독 지연·ROS 런타임 연결 종료를 경고/보고서로 남기고 GPU 물리를 계속 진행한다.
ROS 제어 모드에서는 브리지 단절 시 모든 명령을 취소하고 감속 정지 후 실행을 끝낸다.
ROS 초기화 자체가 실패한 경우에는 START 뒤 오류를 명시한다.
물리 솔버, 초기값, 입력 명령, 수확 판정, 충돌 정책은 바꾸지 않는다.

실행 폴더의 `ros2/config.json`, `bridge.log`, `publisher.json`, `transport.json`에서
domain, 발행 수, 오류, 종료 상태를 확인한다. 원시 rosbag은 자동 저장하지 않는다.
필요한 토픽만 선택해 별도로 `run_s300_ros2.sh bag record ...`를 사용한다.

다른 PC에서 직접 구독하려면 ROS 2를 준비하고 `ROS_DOMAIN_ID=42`를 맞춘다.
DDS discovery와 데이터 통신이 PC↔서버/컨테이너 사이에서 가능해야 한다.
이번 검증은 **같은 호스트에서 실제 DDS 구독**까지이며 외부 PC 네트워크 수신은 검증하지 않았다.
독립 시뮬레이션 여러 개를 동시에 실행할 때는 각각 다른 domain을 써서 전역 `/clock` 충돌을 피한다.

## 2026-11-11 ROS 명령 제어 검증

- 기존 GPU actuator를 통해 실제 관절을 구동했다. 목표를 화면에만 적용하거나 qpos를 강제로 쓰지 않았다.
- 최신 headless 시험: 여섯 관절 동시 이동, 리프트/그리퍼 동시 제어, 기체 전진/후진,
  속도 발행 중단 후 감속, 정지 유지/해제, 잘못된 이름/범위 거절, 실제 카메라 발행을 모두 통과했다.
  4,400스텝/19.9996sim초/20.0047wall초, 카메라20노출, 활성 관통0이었다.
- 앞선 headless45초와 GUI9초 시험에서도 같은 제어 항목을 통과했다(이때 arm 시험은 손목 한 축).
  명령 수락 관측은 각각 약43ms/145ms였다. DDS·GUI 부하에 따른 단일 표본이며 최악 지연 보장은 아니다.
- ROS 실시간 실행에서 기록한9,900스텝의 실제 actuator 명령을 기존 GPU 실행기에 다시 넣었을 때,
  4,950개 배치 경계의 qpos/qvel/time이 비트 단위로 일치했다.
  GUI/headless에서 wall-clock으로 보내는 명령의 도착 스텝은 달라질 수 있으므로 두 시험의 궤적이 같다고 주장하지 않는다.
- 실제 기체0.08m/s 이동 중 테스트 ROS 브리지 프로세스를 종료했다. 명령을 취소하고 감속하여
  최종 최대 관절 속도0.000154미만, `ros_disconnected`로 정상적으로 실행을 끝냈다. 활성 관통0.
- 시작 설정의 free-Hz가 다른 경우 온라인 버퍼를 과거 계획으로 재샘플링하려던 초기화 실패도 보존했다.
  입력/옵션 모두220Hz로 고정하고, 온라인 모드의 재샘플링을 초기 검사에서 거절하도록 수정했다.
- 초기 모델·카메라 로딩 시간은 위 실행 시간에 포함하지 않았다. ROS 명령 제어 시험이며 자동 순회의
  경로 계획 대기·전체 수확 속도를 해결했다는 결과가 아니다. 현재 물리/기하학적 수확 정책을 유지한다.
- 실제 하드웨어, 외부 PC DDS 통신, ROS 액션 서버는 이번 시험 범위가 아니다.
- 최종 회귀: native unittest37개와 ROS 메시지 unittest3개 통과. 로컬 colcon 빌드 및 의존성 검사 통과.
- 검증: `nvidia-sim/validation/s300_ros2_control_20261111.json`.
- 원자료: `/root/farmily_runtime/newton/runs/20261111_ros2_control`.
- 영상: `ros2_control_live.mp4`(실제 GUI wall-clock,13px 자막). 초기 로딩을 자른 원본 속도의9.84초이며
  원본 전체 기록은 `gui_video/page@3835a254accb64cd2e1caf26083aeeec.webm`이다.

## 2026-11-11 자동 순회 텔레메트리 검증

- 기존 작업 보존 커밋: `b35858a`.
- 동일 초기 모델·명령으로 headless ROS OFF/ON 각각 새 실행: 4개 구간,
  1회 수확 동작, 기존 기하학적 영역 수확 6개. 실제 과실 접촉 분리 시험은 아니다.
- 전체 기록 1,890상태, 상태·명령·접촉·말단 솔버 배열 224개가 정확히 동일했다.
- 실제 DDS 수신 관절 97개(입력 초기 상태 1개 포함), RGB·깊이·instance·semantic·overlay 각 7개.
  35개 이미지 payload가 저장 원본과 바이트 단위로 같다.
- CameraInfo 일치, TF로 복원한 카메라 외부행렬 최대 오차 6.67e-16,
  clock 단조 증가, 전송 누락/연결 오류 0.
- 12.027초 물리 실행의 촬영·계획·기록 포함 wall time은 OFF 17.348초, ON 17.994초였다.
  약 0.65초/3.7% 차이인 단일 짧은 표본이며 반복 성능 보장이나 실시간 달성 주장은 아니다. 초기 준비 별도.
- 최초 고해시 ID를 양수 int32로 바꾸던 오류는 비트 보존 변환으로 수정했다.
  실패 자료도 보존했고, ROS 프로세스가 종료되어도 GPU 수확이 완료되는 것을 확인했다.
- 새 GUI + ROS 실행에서도 1회/6개 영역 수확 완료. headless의 224개 물리 배열과 정확히 같다.
  초기 미리보기 포함 카메라 8세트/관절 98개 DDS 수신, 브라우저 오류 0. GUI 로딩 후 wall time 28.649초.
- native 회귀 36개, ROS 메시지 회귀 3개 통과.
- 검증 JSON: `nvidia-sim/validation/s300_ros2_bridge_20261111.json`.
- 원자료: `/root/farmily_runtime/newton/runs/20261111_ros2_bridge`.
