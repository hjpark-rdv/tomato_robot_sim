# 여러 접근 경로·여러 장면을 실제 실행기에 연결하는 탐색

기준: `9ea72dd817a1b81c7a66b93ed3f6b74cce03fcda`. 기존 서버 실험을 포함한다.
작업 브랜치: `codex/multifamily-search-20260928`.

## 이번 작업의 중심

04번의 마지막 1.4mm만 맞추는 대신, 접근부터 확인 동작까지 다르게 만들어 여러 장면·열매에서 비교한다. 새 물리엔진이나 RL을 만들지 않는다. 기존 장면 생성기, scene-only 스냅샷, 경유점별 pose 계획, IK/FCL, 환경 검사, RobotEngine, 접촉 기록과 영상 렌더를 연결한다.

새 파일:
- `scripts/motion_family_search.py`: 5종 경로군과 Sobol 파라미터, 제한된 후속 탐색, 증거 수준 분류.
- `scripts/run_motion_family_search.py`: 여러 장면/타깃 배치 실행, 예산, 기록, HTML/CSV, 대표 영상.
- `scripts/prepare_motion_search_batch.py`: 기존 랜덤 GLB 생성기와 scene-only 수집기를 호출해 바로 실행 가능한 입력 manifest 생성.
- `scripts/target_truss_identity.py`: 여러 송이의 이름과 실제 바디 소속 확인.

기존 진단 실행기에 `--search-contact-policy`를 추가했다. 기본 모드는 유지한다. 기존 `dataset_motion.py`, 로봇 제어기, 물성, 충돌체, 생성기 자체, RGB-D 모델과 학습 데이터는 변경하지 않았다.

## 1. 달라진 경로

| family | 바뀌는 접근 |
|---|---|
| under_center | 하부 삽입 → 중심 통과 → 별도 안착 위치 |
| side_mouth | 고리를 기울여 측면의 열린 입구로 목표를 넣고 자세 전환 |
| flank_left | 왼쪽으로 휘어 들어오는 위치 경유점과 회전 변화 |
| flank_right | 오른쪽으로 휘어 들어오는 위치 경유점과 회전 변화 |
| pivot_sweep | 과실에 대한 상대 위치를 유지하도록 고리 위치와 회전을 함께 변경 |

방위각, 접근 고도각, 고리 로컬 x/z 기울기, 삽입 높이, 옆 변위, 우회 폭, 입구 회전, 꼭지 목표 지점, 확인 이동이 변한다. 같은 Sobol 표본을 경로군별로 교대로 배치한다. 기본 4표본 × 5경로군=20제안이며, 제안 자체가 유효하지 않으면 원인과 함께 별도 기록한다. 상한을 넘는 경유점을 잘라 성공처럼 사용하지 않는다.

완료/기하 후보 등 가능성이 보인 경로군 중 최대2개의 부모를 선택해 각각2개 후속 제안을 만든다. 이때 마지막 위치만이 아니라 접근 방위각·기울기·우회 폭을 바꾼다. 기본 전체 물리 예산은 타깃당12회이며 그 안에서 탐색과 후속 시도를 배분한다. 모든 제안이 실제 물리를 실행하는 것은 아니다.

**곡선은 현재 여러 Cartesian 구간으로 표현한다. 기존 계획기는 구간 사이에 감속/정지를 넣는다. 연속적인 사람 같은 관절 움직임을 이미 달성했다는 뜻은 아니다.** 초기 preapproach도 기존 관절 보간이다. 새 코드가 범용 장애물 우회 최적화기나 완전한 경로 탐색기는 아니다.

## 2. 접촉 규칙

사용자가 허용한 목표 과실과 해당 송이 중심가지 접촉을 새 탐색 모드에 반영했다. 실제 반원 와이어32개만 허용 도구로 사용하며, 목표 바디가 속한 `TRUSS_Truss_01_Peduncle_00` 아래의 정확한 Rachis 충돌체만 선택한다. `truss_01__` 같은 여러 송이 접두어를 구분한다.

목표 과실/자기 송이 Rachis/목표 꼭지의 가벼운 와이어 접촉은 접근·삽입·안착·유지·확인 단계에서 허용한다. 앞쪽 와이어나 접근 중 꼭지 접촉도 '허용 접촉'과 '뒤쪽 안착 증거'를 분리한다. 다른 과실, 다른 송이의 중심가지, 주줄기, 거터, 마운트나 팔 접촉은 이 권한에 포함하지 않는다.

허용 접촉의 힘을 버리지 않는다. 목표 과실+목표 꼭지+자기 Rachis의 접촉력 크기 합을 제한값에 비교한다. 기존 5N/20mm는 이번 파일럿의 시작 비교 조건으로 사용하되 손상 임계값/실물 안전값이 아니다. 비허용 접촉0.01N과 물리 관통0.5mm 조건은 그대로다. 과실과 중심가지를 실제로 관통하도록 collision mask나 물성을 바꾸지 않는다.

## 3. 결과 의미

- `contact_retention_evidence`: 현재 접촉/시간 기준의 강한 SIM 증거. 실물 수확 보증이 아니다.
- `geometric_candidate`: 실제 실행 중 기존 기하 안착 범위에 들어온 후보. 접촉력이0이어도 버리지 않는다. 모델 오차가 측정됐거나 실제 걸림이 입증됐다는 뜻은 아니다.
- `motion_completed_no_capture_evidence`: 동작 완주, 걸림 증거 없음.
- `ik_not_found`, `planning_rejected`, `path_blocked`: 이번 제안의 실패 단계와 충돌 쌍을 보존한다.
- `experimental_limit_stop`, `physics_invalid`, `execution_error`, `not_evaluated_budget`: 별도 분류한다.

**한 타깃에서 모든 제안이 실패해도 '수확 불가능'으로 확정하지 않는다.** 이번 범위에서 못 찾은 것과 물리적으로 불가능한 것은 다르다. 본 버전은 불가능성 증명기를 구현하지 않았고 `impossible=null`로 남긴다. 도달영역/연결성 등 별도 증명이 생긴 경우에만 후속 단계에서 확정 분류를 추가한다.

기존 1.5mm 기하 범위를 넓혀 성공으로 만든 것이 아니다. 반대로 접촉력0을 실물 실패로 단정하지 않는다. `hook_success=null`, `training_eligible=false`를 유지하며 기존 action14로 새 경로를 축약하지 않는다.

## 4. 서버에서 실행

기존 미커밋 변경과 원본 결과를 보존하고 별도 worktree를 사용한다. 사용자 서버의 native MuJoCo Python과 Torch/FCL planning Python을 구분한다. native 환경에는 기존 영상 의존성도 있어야 한다.

### 기존 랜덤 수집 폴더를 재사용

```bash
$PY mujoco-benchmark/scripts/run_motion_family_search.py \
  --discover-collection /실제/기존랜덤수집폴더 \
  --manifest-output /새로운/cases.json
```

또는 아래 형태로 실제 스냅샷 목록을 만든다. `source_run`에는 manifest/planning_inputs/replay_assets가 모두 있어야 한다. 기존 snapshot별 타깃 이름과 model hash를 검증한다.

```json
{"cases":[{"scene_id":"existing_scene","target":"Tomato_02","source_run":"/실제/대상별/physics","split":"pilot"}]}
```

### 새로운 랜덤 장면 준비

```bash
$PY mujoco-benchmark/scripts/prepare_motion_search_batch.py \
  --output "$PREP" \
  --source-dir /root/farmily_tomato/nvidia-sim/env_usd/tomato_rotate_glb \
  --planning-model /실제/기존/replay_assets/planning_model.pkl \
  --scenes 3 --trusses 1 --targets-per-scene 0 \
  --segment-min 6 --segment-max 10 --angle-min 0 --angle-max 180 --hz 240
```

이 wrapper는 기존 generate_random_glb_scenes.py와 candidate_experiment.py --scene-only를 호출한다. 장면 초기 검사 탈락도 기록하며 성공 장면으로 몰래 교체하지 않는다. 생성기의 기존 프로파일별 자산 보정/물성 설정은 scene metadata를 그대로 확인해야 한다. 주변 시각전용 형상을 실제 충돌체처럼 취급하지 않는다.

### 탐색·실행·비교

```bash
$PY mujoco-benchmark/scripts/run_motion_family_search.py \
  --manifest "$PREP/cases.json" --output "$OUT" \
  --planning-python /root/isaaclab_env/bin/python \
  --samples-per-family 4 --physics-per-target 12 --refine-parents 2 \
  --case-workers 2 --planning-workers 2 \
  --execute --max-target-force-n 5 --max-target-displacement-m 0.020 \
  --render-budget 5
```

`--execute`가 없으면 계획/조건부 검사만 한다. `--case-workers`는 CPU 프로세스 병렬이며 MJWarp GPU 병렬을 새로 구현한 것이 아니다. 메모리를 보고1부터 올린다. 각 child process에는 시간 예산이 있고 타임아웃은 성공불가 판정이 아니다.

출력: campaign.json / results.json / results.csv / index.html / case_result.json / 후보별 preflight·원자료 / video_queue.json 및 선택된 MP4. 영상은 기존 저장 qpos를 재생하며 새로운 물리를 몰래 계산하지 않는다. 실패 영상도 포함한다. `--resume`는 동일 설정·코드·입력의 저장된 사례 결과를 재사용한다. 기록이 끊긴 사례를 무작정 덮어 재실행하지 않으며 새 출력이나 별도 검토가 필요하다.

새 장면 준비는 RGB-D를 새로 촬영하지 않는다. 기존 collection 입력에는 observation_path를 보존한다. 새 장면의 RGB-D는 탐색 실험이 안정된 다음 기존 수집기로 연결하며 기존 포맷/장면 단위 분할을 유지한다. 이 탐색기는 SIM-GT를 쓰는 교사 탐색이지 단일 RGB-D 학생 모델 성능 개선 결과가 아니다.

## 5. Codex가 이번에 마칠 일

1. 위 브랜치를 별도 worktree로 받고 테스트한다. 04번의 마지막 자세 반복부터 시작하지 않는다.
2. 먼저 서로 다른 타깃2개로 smoke: 경로군당2제안, 물리 타깃당6회, 부모 추가탐색0, 대표영상3개. 가능하면 하나는 `truss_01__` 이름의 두 번째 송이로 선정해 매핑도 실제 자산에서 확인한다.
3. 제안→기존 IK/FCL→환경 검사→물리→집계→영상 연결 오류를 고친다. 오류를 IK불가/수확불가로 분류하지 않는다. 실제 후보의 접근 위치와 회전이 경로군별로 달라지는지 확인한다.
4. 연결이 정상일 때 새6장면에서 각3타깃, 타깃당20제안+최대4후속제안, 전체물리 타깃당12회 이하로 파일럿을 수행한다. 다음 라운드에서 모든 열매로 확장한다. 초기 장면/선택되지 않은 열매도 분모로 기록한다.
5. 기존 azimuth-only 경로가 있는 동일 타깃은 비교 기준으로 보존한다. 서로 다른 성공 기준으로 새 방법이 우월하다고 주장하지 않는다. 이번 최소 산출물은 경로군별 계획통과·완주·기하후보·접촉증거·중단원인 표와 영상이다.
6. 가장 유망한 복수 경로는 남은 예산 안에서만 다시 시도한다. 한 타깃의 미세위치 조정으로 전체 실험을 중단하지 않는다. 접촉증거가 없더라도 기하후보와 실제 영상을 보고하고 탐색 커버리지를 비교한다.
7. 물성/충돌체/기존 데이터셋/운영정책을 바꾸지 않는다. code hash와 입력 hash, 모든 제외·오류·미실행을 기록한다. 필요한 서버 연결 수정과 결과 요약을 별도 브랜치로 commit/push한다.

## 6. 검증 범위

로컬에서는 새 테스트45개 통과, MuJoCo가 필요한1개는 설치 부재로 건너뛰었다. CI에서는 기존 회귀와 새 native/기하/스케줄 시험을 함께 실행한다. CI 결과는 실제 로그로 별도 확인한다. 사용자 서버의 전체 온실 모델에서 새 탐색을 실행하거나 새로운 수확 성공 영상을 확보한 것은 아직 아니다. 파일럿 결과를 보기 전 성공률 개선을 주장하지 않는다.
