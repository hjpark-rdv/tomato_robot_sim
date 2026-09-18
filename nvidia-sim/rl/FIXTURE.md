> 이 문서는 원본 온실이 아닌 별도 단순 물리 테스트 장면의 기록입니다. 기본 실행 환경은 README.md를 보세요.

# 고리 그리퍼 수확 학습 — 첫 단계

비전/ROS 없이 RB5와 Ver.6 그리퍼로 단일 방울토마토의 꼭지를 분리하는 Isaac Lab 환경입니다.
원본 온실 USD, 로봇 USD, ROS GUI 파일은 수정하지 않습니다. 로봇은 원본 USD를 참조하는 학습 전용 override로 불러옵니다.

## 실행

저장소 루트 `/root/farmily_tomato`에서 실행합니다. 현재 설치된 Isaac Sim 5.1.0,
Isaac Lab 2.3.2.post1, Stable-Baselines3 2.8.0을 사용합니다.
Python은 `/root/isaaclab_env/bin/python`이며 `FARMILY_ISAAC_PYTHON`으로 바꿀 수 있습니다.

```bash
# 충돌 형상 생성 + 실제 물리 검증: 구멍, 테두리, 파손, 복원, 고리 수확
./nvidia-sim/run_ring_rl.sh --scene fixture --headless --mode test --num-envs 4 --rebuild

# 꼭지 근처에서 시작하는 첫 학습
./nvidia-sim/run_ring_rl.sh --scene fixture --headless --mode train --num-envs 4 \
  --curriculum hook --timesteps 100000 --run-dir nvidia-sim/rl/runs/first_run

# 저장 정책을 GUI로 재생 (사용 가능한 DISPLAY 필요)
./nvidia-sim/run_ring_rl.sh --scene fixture --mode play --steps 1200 \
  --checkpoint nvidia-sim/rl/runs/first_run/policy.zip

# GUI 없는 평가; 학습과 다른 seed 사용
./nvidia-sim/run_ring_rl.sh --scene fixture --headless --mode play --num-envs 4 --seed 43 \
  --steps 1200 --checkpoint nvidia-sim/rl/runs/first_run/policy.zip \
  --run-dir nvidia-sim/rl/runs/evaluation

# 학습 없이 접촉/분리를 살펴보는 진단 동작
./nvidia-sim/run_ring_rl.sh --scene fixture --mode scripted --curriculum hook --steps 600
```

이 버전은 **CPU physics**를 사용합니다. GPU direct API에서는 에피소드 중 USD 기반
조인트 재생성/몸체 갱신이 정상 동작하지 않아 `--device cuda:0`을 명시적으로 거부합니다.
CPU에서 4개 환경 병렬 실행과 PPO 학습을 검증했습니다. 대규모 GPU 학습은 별도의
분리·복원 구현과 검증이 필요합니다. 렌더링은 Isaac Sim GPU renderer를 사용합니다.

## 실제 구현한 물리와 성공 조건

- Ver.6 STL의 끝부분은 **열린 반원**입니다. 이를 닫힌 원으로 바꾸지 않았습니다.
  그리퍼 좌표계 중심 `(-0.10640287, 0.00616804, 0.00006305)` m, 테두리 중심선 반경
  27.5 mm를 따라 반경 1 mm의 캡슐 32개를 배치했습니다. CAD의 2 mm 두께 단면을
  원형 단면으로 근사합니다. 근위부는 별도의 convex hull이며 구멍을 덮지 않습니다.
- 현재 원본 로봇의 collision geometry에 빠진 CollisionAPI를 학습 override에서
  복원합니다. 원본 asset generator의 설정만으로 실제 collider 유무를 판단하지 않습니다.
- 토마토는 반경 11.5 mm, 질량 8 g의 구와 반경 1.5 mm, 길이 30 mm의 꼭지를 묶은 강체입니다.
  원본 식물의 복잡한 시각 메시를 복제한 것이 아닌 **학습용 단순 모델**입니다.
- 고정 줄기는 별도의 kinematic rigid body이며, 꼭지 끝에서 standalone fixed joint로
  열매에 연결됩니다. 초기 파손 임계값은 **3 N / 0.08 N·m**입니다.
  기존 온실 실행기의 1.0 N·m 덮어쓰기를 사용하지 않습니다.
- PhysX가 실제 접촉/하중으로 조인트를 끊습니다. 위치 조건으로 조인트를 끊거나,
  학습 중 열매를 그리퍼에 순간 부착하지 않습니다.
- 성공은 **삽입 이력 + 실제 고리/꼭지 collider 접촉 + 접촉 시점 근처의 목표 joint break**입니다.
  열매에 외력을 줘 떨어뜨린 경우는 실패이며, 이를 물리 테스트에서 확인합니다.
- 파손 후에는 해당 환경의 joint만 제거하고, 물체 pose/velocity를 복원한 뒤 새 joint를
  생성합니다. USD 좌표와 PhysX tensor state를 함께 맞추고 articulation FK를 갱신합니다.

현재 fixed joint는 분리 이전의 휘어짐을 표현하지 않습니다. 꼭지 접촉으로 발생한
힘/토크에 따른 분리를 근사합니다. 실제 줄기의 굽힘 강성, 파단각, 과육 변형/손상은
모델에 없습니다. 충격 감점은 과육 손상의 대리지표입니다. 또한 로봇은 이상적인 중력
보상을 가정해 로봇 몸체에만 중력을 끄고, 열매에는 정상 중력을 적용했습니다.
고정 베이스, 고정 리프트 목표, 기존 self-collision 비활성 설정을 사용하므로 이 초기 정책의
성과를 실기 수확률이나 온실 장애물 회피 성능으로 해석하면 안 됩니다.

## 관측·행동·보상

관측 36개: 고리 기준 열매 중심(3), 꼭지 접촉 목표점(3), 꼭지 방향(3), 팔 관절각/속도(12),
link6의 world-frame 선속도/각속도(6), 이전 적용 행동(6), 삽입/걸림 이력(2), 접촉력(1).
값은 모두 시뮬레이터의 상태를 사용합니다. 비전 인식 결과는 필요하지 않습니다.

행동 6개는 고리 좌표계의 위치/회전 변화입니다. 제어 주기 1/60초마다 각 위치축 최대
1.5 mm, 각 회전축 최대 0.02 rad를 명령하고, 240 Hz 물리 단계에서 끝점 오프셋을 반영한
Differential IK와 관절 PD 구동기를 사용합니다. 팔의 관절 목표 변화와 관절 한계를 제한합니다.

보상은 삽입/걸림 위치에 대한 진행량, 단계별 1회 보상, 유효 분리 +30, 잘못된 분리 -15,
비목표 접촉·충격·급격한 행동·시간 감점으로 구성됩니다. 성공, 잘못된 분리,
25 cm 작업 범위 이탈, 비정상 상태, 6초 시간 초과에서 에피소드가 끝납니다.

## 단계 확장

| 옵션 | 시작 상태 | 아직 학습해야 하는 동작 |
|---|---|---|
| `--curriculum hook` | 고리가 꼭지 가까이에 있음; 삽입 이력을 초기 조건으로 제공 | 걸림 접촉과 분리 |
| `--curriculum inserted` | 열매가 고리 내부에 있음; 삽입 이력을 초기 조건으로 제공 | 꼭지까지 이동, 걸기, 분리 |
| `--curriculum approach` | 열매가 고리 면에서 55 mm 떨어짐; 삽입 이력 없음 | 삽입부터 분리까지 |

`hook` 단계 성공은 삽입 행동을 학습했다는 뜻이 아닙니다. 다음 단계에서는 기존 checkpoint를
불러와 계속 학습하거나 새 정책과 비교할 수 있습니다.

```bash
./nvidia-sim/run_ring_rl.sh --scene fixture --headless --mode train --num-envs 4 \
  --curriculum inserted --timesteps 100000 \
  --checkpoint nvidia-sim/rl/runs/first_run/policy.zip \
  --run-dir nvidia-sim/rl/runs/inserted
```

기본 위치 무작위화는 각 축 ±1 mm (`--position-jitter`), 파손 임계값은 에피소드별 ±10%입니다.
기하 형상/질량은 `geometry.py`, 제어/보상은 `harvest_env.py`에서 설정합니다.
고리 형상 변경 후에는 `--rebuild`를 실행해야 합니다.
파손 임계값은 `--break-force 3 --break-torque 0.08`로 바꿀 수 있습니다.

생성 asset은 `generated/`, 모델/로그는 지정한 `--run-dir`에 저장됩니다.
`policy.zip`, `config.json`, `training_summary.json`, `evaluation_summary.json`,
`physics_test.json`으로 실행 설정과 결과를 확인할 수 있습니다. 생성 asset과 학습 모델은
Git에서 제외되지만 로컬 작업 공간에는 보관됩니다.

검증용 초기 정책은 `runs/smoke_ppo/policy.zip`에 있습니다. 4개 환경, `hook`, seed 42로
8,192 스텝 학습했고, 학습 중 완료 537개 중 유효 분리 535개였습니다.
별도 seed 43 평가에서는 완료 881개 중 881개가 유효 분리였습니다.
이는 위의 좁은 시작 분포에서의 초기 결과이며, `inserted`/`approach` 정책 학습은 아직 수행하지 않았습니다.
