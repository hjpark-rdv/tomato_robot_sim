# 원본 온실·송이에서 PICK_READY + 리프트 수확 학습

기본 학습 장면은 원본 온실과 수확 가능한 줄기·송이 USD를 참조합니다.
비전 없이 시뮬레이터 상태를 사용하며, 로봇과 고리의 접촉으로 실제 PhysX 조인트가 파손되어야 수확으로 판정합니다.

## 현재 시작 조건

- 온실/로봇 베이스 배치: `scenes/farmily_greenhouse_robot.usd`.
- 줄기/송이: `env_usd/tomato_stem_v8_with_rotated90_cluster_harvestable_FIXED.usd`의 열매 11개 전체.
- 학습용 송이 spawn 기본값: **`(-0.75, 0.55, 0.32)` m**, scale `0.5`, yaw `0°`.
  일반 `run_sim.sh`의 기본값은 기존 Y=1.20이며, 일반 실행에서 이 위치를 쓰려면
  `./run_sim.sh --spawn-stem -0.75 0.55 0.32`를 지정합니다.
- 팔 시작 자세: ROS GUI에서 `CAPTURE_LEFT`에 대응시키는 **`PICK_READY`**.
  `ros2_ws/src/rbpodo_ros2/rbpodo_moveit_config/config/rbpodo.srdf`를 직접 읽습니다.
  이번 확인에서 GUI가 읽는 설치본 SRDF와 저장소 파일의 내용이 같았습니다.
- 송이 높이: 열매 11개 과육 중심의 월드 Z 좌표 평균.
- 초기 높이: **로봇 장착면(`link0`)이 송이 중심보다 40cm 낮게**.
  현재 송이 중심은 약 **0.9492m**, 장착면은 **0.5492m**, 리프트 관절 변위는 **0.4742m**입니다.
  리프트 변위와 장착면의 월드 높이는 로봇 베이스 높이 때문에 다릅니다.
- 이후에는 정책이 리프트를 독립적으로 올리고 내립니다. 관절 범위는 **0~0.75m**이며,
  기본 최대 명령 속도는 **0.10m/s**입니다. 팔 제어는 움직이는 리프트 기준이므로
  팔을 유지하는 명령이 리프트의 상승을 상쇄하지 않습니다.

기존의 근거리 IK 시작 자세로 토마토 앞으로 이동시키는 초기화를 제거했습니다.
에피소드 리셋은 원래 송이 배치, ROS `PICK_READY`, 지정한 초기 리프트 높이를 복원합니다.

## 실행

저장소 루트에서 실행합니다. `nvidia-sim` 디렉터리에서는 경로 앞의 `nvidia-sim/`을 빼면 됩니다.
기존 Isaac Sim 5.1 / Isaac Lab 2.3.2와 `/root/isaaclab_env/bin/python`을 사용합니다.

```bash
# 새 시작 조건을 GUI로 확인
./nvidia-sim/run_ring_rl.sh --mode inspect --steps 1200

# 시작 자세·40cm 높이차·리프트 상하 이동·파손·송이 복원 검사
./nvidia-sim/run_ring_rl.sh --headless --mode test \
  --run-dir nvidia-sim/rl/runs/pick_ready_physics

# 새 행동/관측 구조로 PPO 학습
./nvidia-sim/run_ring_rl.sh --headless --mode train --timesteps 100000 \
  --run-dir nvidia-sim/rl/runs/pick_ready_train

# 새 모델 재생 (저장된 높이/배치 설정을 자동으로 읽음)
./nvidia-sim/run_ring_rl.sh --mode play --steps 1200 \
  --checkpoint nvidia-sim/rl/runs/pick_ready_train/policy.zip

# 새 모델에서 추가 학습
./nvidia-sim/run_ring_rl.sh --headless --mode train --timesteps 100000 \
  --checkpoint nvidia-sim/rl/runs/pick_ready_train/policy.zip \
  --run-dir nvidia-sim/rl/runs/pick_ready_continue
```

현재 원본 온실은 CPU physics, 환경 1개로 실행합니다. 목표는 기본 `Tomato_08`이며
`--target-fruit`로 선택합니다. `--stem-position X Y Z`, `--stem-yaw`, `--stem-scale`로
배치를 바꿀 수 있습니다. `--lift-start-below 0.4`는 초기 높이 차이,
`--lift-height-reference mount`는 로봇 장착면 기준입니다.
그리퍼 고리 중심을 기준으로 삼으려면 `--lift-height-reference ring`을 지정합니다.
요청한 높이를 리프트 이동 범위 안에서 만들 수 없으면 오류로 알립니다.
`--lift-speed 0.1`은 최대 명령 속도이며, 원본 관절의 최대 속도 0.25m/s 이하로 설정합니다.

## 행동·관측과 모델 호환성

행동은 **7개**입니다. 앞의 6개는 고리 좌표계의 위치/회전 변화이며,
마지막 1개는 리프트 상하 속도 명령입니다. 0은 현재 리프트 목표 높이를 유지합니다.
제어 주기는 60Hz, 물리 주기는 240Hz입니다. 팔 명령은 각 축 최대 1.5mm / 0.02rad입니다.

관측은 **39개**입니다. 고리 기준 과육 중심·꼭지 위치·방향(9), 팔 관절각/속도(12),
link6 월드 속도(6), 적용 행동(7), 삽입·접촉 이력과 접촉력(3), 리프트 변위/속도(2)입니다.

새 정책 형식은 `greenhouse-pick-ready-lift-v1`입니다.
**이전 고정 리프트 정책(6개 행동/36개 관측)은 직접 재생하거나 이어 학습할 수 없습니다.**
`runs/greenhouse_smoke`, `runs/greenhouse_resume_check`는 이전 형식입니다.
이전 단순 구형 토마토 정책 `runs/smoke_ppo`도 원본 온실 정책이 아닙니다.
해당 단순 장면은 명시적인 `--scene fixture` 옵션으로 보존되어 있습니다.

리프트를 포함한 준비 자세에서 접근할 수 있도록 에피소드 길이는 20초,
목표와의 거리 종료 기준은 2m로 설정했습니다. 접근 거리 보상의 길이 척도는 0.5m입니다.
`hook`/`inserted` 및 고정 진단 동작 `scripted`는 단순 장면 전용입니다.

## 물리와 성공 조건

원본 USD 파일에는 직접 쓰지 않고 학습 stage에 override를 추가합니다.
원본 열매의 시각 메시, 크기, 질량을 유지하며 고리 충돌 형상은 CAD의 열린 반원을 따르는
32개 캡슐로 구성합니다. 과육 표면~원래 attachment 사이에는 꼭지 접촉용 캡슐 근사를 추가합니다.
StaticPlant 메시 239개는 정적 삼각형 collider로 사용합니다.
붙어 있는 과육과 정적 줄기 사이 자기 충돌은 제외하고, 열매끼리 및 로봇과의 충돌은 유지합니다.

원래 attachment마다 kinematic 고정부와 분리 가능한 고정 조인트를 둡니다.
원본 파손 임계값 **3N / 0.08N·m**를 사용하며, `--break-force`/`--break-torque`를 명시하면 바뀝니다.
분리 조인트는 완성된 상태로 한 번에 반영하여 GUI에서 중간 속성 변경을 처리하면서
kinematic 고정부를 `wakeUp()` 하던 오류를 방지합니다.

성공은 삽입 이력, 고리와 목표 꼭지의 실제 접촉, 해당 조인트의 PhysX 파손이 이어져야 합니다.
다른 열매가 파손되면 실패입니다. 위치 조건만으로 조인트를 끊지 않습니다.
`invalid_breaks`는 잘못된 분리 횟수이며 엔진 오류 횟수가 아닙니다.
현재 고정 조인트는 분리 전 줄기 굽힘이나 과육 손상을 모델링하지 않습니다.
로봇은 이상적인 중력 보상을 가정하고, 열매에는 중력을 적용합니다.
시작 배치와 파손 임계값 무작위화는 아직 적용하지 않습니다.

## 검증 기록

실행 폴더의 `scene_provenance.json`에 송이 배치, ROS 준비 자세, 높이 기준, 계산된 리프트 변위,
관절 범위 및 정책 형식을 남깁니다. `physics_test.json`에는 초기 높이차와 실제 상하 이동량,
조인트 파손/복원 및 이웃 열매 실패 처리 검사 결과가 기록됩니다.

[새 시작 자세 미리보기](validation/pick_ready_overview.png).
이전 `validation/greenhouse_learning.json`은 고정 리프트/이전 배치의 기록이고,
`validation/learning.json`은 단순 장면의 기록입니다. 새 정책의 수확 성과와 구분해야 합니다.

[현재 조건의 검증 결과](validation/pick_ready_lift.json): 장착면 높이차 0.4000m,
리프트 약 2cm 상승/하강, 팔 유지, 원본 송이 파손·복원 3회, GUI PhysX 오류 0건을 확인했습니다.
새 형식의 PPO 2,048 스텝 학습과 저장 모델 재생도 확인했습니다. 수확 성공은 아직 0회입니다.
이 초기 모델은 `runs/pick_ready_smoke/policy.zip`에 있으며 새 형식으로 이어 학습할 수 있습니다.
