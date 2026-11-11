# Newton 순회 시뮬레이터 → ROS 2

현재 Newton GPU 순회 실행에 **읽기 전용 ROS 2 Humble 데이터 발행**을 연결했다.
관절, 기체/리프트 위치, TCP, TF, 시뮬레이션 시각, 장착 카메라 RGB·깊이·GT 마스크를 수신할 수 있다.
ROS에서 로봇을 명령하는 기능은 이번 범위가 아니다. 기존 하드웨어 드라이버는 실행하지 않는다.

## 실행

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
관절·TCP·TF는 reliable, labels/status는 reliable + transient local이다.

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

주기적 관절/TF 표본은 기본 시뮬레이션 10Hz(`--ros-state-hz`, 최대 설정 60Hz)이며
노출 시각과 구간 경계 표본이 추가된다. 최초 t=0은 입력 초기 자세이고, 이후 표본은 GPU 실행에서 읽은 실제 상태다.
계획·카메라 준비 때문에 물리가 멈춘 동안 clock을 임의로 전진시키지 않는다.

**카메라는 기존 실제 촬영 이벤트에서만 발행한다. 고정 30FPS 스트림이 아니다.**
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
```

rclpy를 Newton 가상환경에 넣지 않는다. 별도 ROS 프로세스가 DDS 처리를 맡는다.
느린 구독/전송 때문에 상태를 무한히 쌓지 않으며 데이터 종류마다 가장 최근 대기 표본만 유지한다.
합쳐진 표본 수는 실행 폴더 `ros2/transport.json`의 `dropped`에 기록한다.
구독 지연·ROS 런타임 연결 종료는 경고/보고서로 남기고 GPU 물리를 계속 진행한다.
ROS 초기화 자체가 실패한 경우에는 START 뒤 오류를 명시한다.
물리 솔버, 초기값, 입력 명령, 수확 판정, 충돌 정책은 바꾸지 않는다.

실행 폴더의 `ros2/config.json`, `bridge.log`, `publisher.json`, `transport.json`에서
domain, 발행 수, 오류, 종료 상태를 확인한다. 원시 rosbag은 자동 저장하지 않는다.
필요한 토픽만 선택해 별도로 `run_s300_ros2.sh bag record ...`를 사용한다.

다른 PC에서 직접 구독하려면 ROS 2를 준비하고 `ROS_DOMAIN_ID=42`를 맞춘다.
DDS discovery와 데이터 통신이 PC↔서버/컨테이너 사이에서 가능해야 한다.
이번 검증은 **같은 호스트에서 실제 DDS 구독**까지이며 외부 PC 네트워크 수신은 검증하지 않았다.
독립 시뮬레이션 여러 개를 동시에 실행할 때는 각각 다른 domain을 써서 전역 `/clock` 충돌을 피한다.

## 2026-11-11 검증

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
