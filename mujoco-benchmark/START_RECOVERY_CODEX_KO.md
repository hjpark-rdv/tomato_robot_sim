# Start recovery → D1 one-command handoff

기준 조상: `ecc6beafcc50e1c67116c470a1e6e2a44646edbc`  
브랜치: `codex/start-recovery-20260928`

## 목적

covered dense scene에서 확인된 시작 충돌:

- robot: `Hook/g410`
- obstacle: `neighbor_truss_collision_fruit_p19_t02_g052`
- penetration: 약 2.2344 mm

을 obstacle/mask/material/margin을 바꾸지 않고 **robot q만 변경해서** 해결한다.

중간 candidate/IK/path 실패마다 멈춰서 사용자에게 다시 묻지 않는다.

## 새 코드

- `initial_configuration_recovery.py`
  - deterministic expanding q search
  - axis + scrambled Sobol
  - joint bound / existing FCL self collision / native MuJoCo environment distance
  - finite search failure는 `impossible=null`
- `recover_initial_configuration.py`
  - covered MJB에서 canonical q 탐색
  - static non-overlap + 2초 hold validation
  - 원래 q와 모든 evidence 보존
- `materialize_new_start_run.py`
  - historical source run은 변경하지 않음
  - 새 initial q / planning_inputs / ring position을 가진 별도 snapshot 생성
  - **기존 candidate trace를 복사하지 않음**
  - 따라서 recorded prefix의 첫 q만 몰래 바꾸는 것이 불가능
- `run_new_start_d1.py`
  - 기존 D1 evidence-selected target/design을 그대로 사용
  - 모든 prefix를 new start에서 재계획
  - local suffix도 도달한 새 joint branch에서 재계획
  - 최대 기존 D1 planning query 수, physics 상한8 유지
  - whole-path audit PASS만 physics 실행
  - 실제 physics qpos가 있으면 MP4 생성 가능
- `run_start_recovery_pipeline.py`
  - start recovery → D1 planning → 유효 후보 physics를 한 명령으로 연결
  - start q를 못 찾은 경우에만 D1 전 단계에서 종료
  - 개별 query 실패는 다음 query로 계속 진행

## 서버 실행 예

경로는 기존 covered D1 server report 기준이다.

```bash
cd /root/farmily_tomato
git fetch origin
git checkout codex/start-recovery-20260928

PY=./mujoco-benchmark/.recovery-venv/bin/python
# 실제 planning env에 Torch/FCL/OMPL이 함께 있는지 먼저 확인한다.
PLAN_PY="$PY"
NATIVE_PY="$PY"

DENSE_RUN=/root/docker_share/mujoko_debugging_data/20260927_stem_obstacle_scene/robot_checks/Tomato_02
COVERED=/mnt/nas_rdv_md3/covered_d1_20260928/covered_scene
DATA=/root/docker_share/mujoko_debugging_data/20260928_multifamily_server
PREFIX=$PWD/mujoco-benchmark/validation/multifamily_server/prefix_comparison.json
OUT=/root/docker_share/mujoko_debugging_data/NEW_DATETIME_start_recovery_d1

"$PY" mujoco-benchmark/scripts/run_start_recovery_pipeline.py \
  --dense-source-run "$DENSE_RUN" \
  --covered-scene "$COVERED" \
  --campaign "$DATA/pilot" \
  --side-campaign "$DATA/side_repair" \
  --prefix-report "$PREFIX" \
  --output "$OUT" \
  --planning-python "$PLAN_PY" \
  --native-python "$NATIVE_PY" \
  --model-cache /mnt/nas_rdv_md3/mujoco_search_cache \
  --render-physics
```

planning Python이 별도 환경이면 `PLAN_PY`만 해당 Python으로 바꾼다. 기존 환경을 업그레이드하지 않는다.

## canonical start 선택

search radius는 normalized joint-space의 **탐색 범위**이지 안전 threshold가 아니다.

기본 탐색:

- original q를 증거로 먼저 평가
- normalized radius 0.01 / 0.025 / 0.05 / 0.1 / 0.2 / 0.4
- 각 radius에서 axis ± move 우선
- deterministic Sobol multi-joint samples
- 최대512 state candidate
- 최대12 static-valid candidate
- 상위6개 2초 hold 검사

선택 우선순위:

1. static native collision-free
2. existing FCL self-collision-free
3. hold physics validity PASS
4. original q에서 normalized joint change 최소
5. native obstacle clearance는 secondary tie-break

한 target마다 start q를 바꾸지 않는다.

## old prefix와 new prefix

old D1:

`original q → recorded prefix → local suffix`

new-start D1:

`canonical new q → newly planned collision-aware prefix → same intended handoff/staging geometry → newly planned local suffix`

기존 recorded prefix는 historical evidence로만 남고 새 trace에 복사되지 않는다.

## 결과

`pipeline_summary.json`은 현재 evidence에 따라 다음과 같이 구분한다.

- `CASE0_INITIAL_CONFIGURATION`
- `CASE1_APPROACH_PLANNING`
- `CASE2_LOCAL_MANIPULATION`
- `CASE3_LOCAL_PHYSICS_EVIDENCE_AVAILABLE`
- `CASE4_RUNTIME_VALIDATION_PENDING`
- `CASE4_PHYSICS_TRACKING_OR_CONTACT`

CASE3도 harvest success 선언이 아니다. contact/retention evidence를 별도로 본다.

## 금지

- g410 축소
- background obstacle 제거
- contact mask 확대/축소
- margin/gap 완화
- 물성/actuator 변경
- old trace 첫 q만 변경
- blocked path 강제 physics
- query 예산을 채우기 위한 추가 샘플
- bounded failure를 impossible=true로 변경
- start blocker 해결 전에 RL 실행
