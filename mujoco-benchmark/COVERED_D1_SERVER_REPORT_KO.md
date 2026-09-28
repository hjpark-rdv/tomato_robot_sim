# 충돌 보강 후 D1 서버 검증 — 2026-09-28

## 결론

**PHASE 1 FAIL: 경로 실행 전의 로봇 시작 자세가 주변 과실과 겹친다.**

106개 주변 송이를 실제 충돌체로 추가하자, `Hook` 바디의 **마운트 충돌체 `g410`**이 배경 식물 19의 세 번째 송이(`truss_index=2`), **Tomato_05**와 처음부터 **2.2344mm** 겹쳤다. 이 과실은 선택한 목표 Tomato_02가 아니다. 목표 와이어 접촉 허용 정책으로 허용할 수 없다.

이는 “D1 11개 경로를 모두 시도했지만 실패”한 결과가 아니다. 사용자 지시의 **PHASE 1 FAIL이면 이후 실행 금지**에 따라 D1 계획 질의 **0/11**, 접근/수확 물리 **0/8**이다. 정지 검증만 원본/보강 각각 2초 실행했다. 정상 접근 또는 수확 성공을 확보하지 못했다.

좌표 변환을 잘못해서 엉뚱한 곳에 과실이 생긴 경우는 아니다. 원본 GLB 5종에서 재추출한 7,738개 충돌체의 위치를 전부 대조했고, 최대 재현 오차는 약 **1.7e-8m**였다. 마운트·기존 충돌체·관절·물성·actuator는 그대로다. `g410`의 기존 충돌 형상이 실제 마운트에 얼마나 정밀하게 맞는지는 별도의 모델 적합성 문제이며, 이번에 축소하거나 바꾸지 않았다.

## 브랜치와 범위

- 작업 브랜치: `codex/server-covered-d1-20260928`
- 입력 코드: `43cce2761eb9f33a531ac0529130a56358e7eceb` (`codex/obstacle-coverage-20260928`)
- 포함된 이전 서버 결과: `d1fdc7c`
- 기존 `mjlab-performance`, 원본 데이터, 사용자의 미추적 bundle은 변경하지 않았다.
- 이번 PHASE 1 대상은 기존 **dense gutter02 / Tomato_02** 환경이다. `d1fdc7c`의 별도 랜덤 4개 유효 장면/12타깃 전체를 검사한 것이 아니다.
- `training_eligible=false`, `hook_success=null`, `impossible=null`.

## PHASE 1 개별 판정

| 항목 | 결과 | 근거 |
|---|---|---|
| collision_coverage | PASS | 106송이, 과실1,060 / Rachis1,484 / pedicel4,664 / peduncle530. 주변 주줄기1,088 및 거터18 유지. 비활성 mask·누락 inventory 검사 |
| transform_consistency | PASS | 동일 source collider + 배치 재계산 7,738개 대조. 최대 약1.7e-8m. 기존 geom 위치·회전 차이0 |
| initial_robot_collision | **FAIL** | ready t=0에서 `g410` ↔ `neighbor_truss_collision_fruit_p19_t02_g052`, 실제 접촉 penetration2.234399mm / native signed distance−2.234417mm |
| target_plant_stability | PASS, 2초 범위 | 목표/활성 식물 최대 변위 원본·보강 **2.967781mm로 동일**. 5mm를 절대 gate로 사용하지 않음 |
| positive_control_detection | PASS | fruit / rachis / pedicel / peduncle 실제 source collider의 음수 거리 대조군4개 모두 해당 class로 차단 |
| negative_control_false_positive | PASS | 해당 collider와 양수 거리를 유지하는 표본 경로4개 모두 통과. 10mm를 강제하지 않음 |
| model_hash_consistency | PASS, PHASE 1 범위 | 입력/출력 MJB 해시 보존, 같은 covered MJB로 native 검사와 RobotEngine 정지 시험. D1 신규 trace에 대한 planner/runtime 검증은 아직 미실시 |
| runtime_validation | **FAIL, 기존 물리 유효성** | 로봇 관련 초기 관통이0.5mm 초과. NaN/Inf 상태·simulator warning은 없음 |

**고정된 배경끼리의 최대 겹침 수치로 scene 전체를 FAIL시킨 것이 아니다. 실제 로봇과 배경 과실 접촉이다.** 과실 접촉이 발생한 마운트를 cyan, 배경 과실을 pink로 표시한 실제 qpos 영상과 전체 시각 모델 사진을 저장했다.

양성/음성 대조군은 기존 `screen()`·native `mj_geomDistance()`를 사용한 **기하 probe fixture**다. 실제 와이어 32개를 기존 mapping으로 확인한 뒤 그중 하나를 probe로 사용했다. IK로 도달 가능한 로봇 동작을 입증하는 시험은 아니다.

초기 대조군의 정확히 동심인 fruit/probe 한 표본은 native 거리0을 반환했다. 이미 차단되었지만 “명백히 음수” 대조군에는 부적합하여, 원자료를 `controls_initial_concentric.json`에 보존하고 0.1mm 비동심 위치로 fixture만 변경했다. 최종4개 양성은 전 표본 음수다. 물리 모델이나 PASS threshold를 바꾼 것이 아니다. 최초 도구가 USD식 `segment_` 이름을 찾다가 실패한 오류도 보존했고, 기존 실제32-wire mapping을 재사용하도록 수정했다.

## 정확한 충돌 대상

- 배경 식물ID19, 송이index2(세 번째), 원본 `Tomato_05`
- GLB: `tomato_master_v10_cluster_curve_white.glb`
- source collider: `glb_col_Tomato_05`
- covered collider: `neighbor_truss_collision_fruit_p19_t02_g052`
- 로봇 collider: `g410`, body=`Hook`, path=`/World/envs/env_0/Robot/link6/tcp/tomato_gripper`
- 배경 과실 collider 중심: `[-0.4482404872, 0.7880243462, 1.0054362299] m`
- 초기 로봇 관절값(기존 joint 순서): `[0.5036082864, 3.2516102791, 0.1146665663, -1.6495294571, -0.1797474325, -1.3975170851, 0.6881734133]`

2초 동안 원본 로봇 바디 최대 이동0mm, 보강 로봇 **3.727016mm**. 새로운 초기 접촉이 실제로 로봇을 밀었다. 활성 식물은 원본과 같은 최대 변위다. 수확 중 target force/retention 결과는 없으며, 이 정지 영상은 접근 성공 영상이 아니다.

## 코드 수정·테스트

기존 `add_neighbor_truss_obstacles.py`, GLB 생성기, `RobotEngine`, `MuJoCoScene/screen`, 기존 target/32-wire mapping을 재사용했다.

- coverage가 이름만 보고 통과하던 허점 수정: 비활성/동적 모델과 mask가 호환되지 않는 collider 거부.
- 생성 inventory의 개별 collider 누락도 검사. 같은 종류 하나가 남아 있어도 다른 필수 과실 누락을 통과시키지 않는다.
- source geometry 재현/원본 물리 파라미터 불변 검사 도구, 정지 A/B, 실제 qpos 렌더 추가.
- background-only 겹침과 로봇/활성 식물의0.5mm 기준 분리. idle displacement를 universal gate로 사용하지 않는 회귀시험 추가.
- **308개 테스트 통과**, native·D1·OMPL 연결 회귀·기존 Torch/FCL pose planner 회귀 포함. 실제 온실에서 D1 경로가 통과했다는 뜻은 아니다.
- 격리 `.recovery-venv`: MuJoCo3.13.0 / OMPL2.0.1. 기존 Python 환경은 변경하지 않았다.
- 시간·native query 수는 JSON 보존. 이 실행은 성능 benchmark가 아니며 RSS는 측정하지 않았다.

## 결과와 대형 자산

Git에 [결과/영상 index](validation/covered_d1_server/index.html), [summary](validation/covered_d1_server/summary.json), [geometry](validation/covered_d1_server/geometry.json), [controls](validation/covered_d1_server/controls.json), [실제 qpos 영상](validation/covered_d1_server/idle_ab.mp4), 원자료/로그/상태 NPZ를 보존한다. 별도 첨부 없이 이 브랜치의 보고서와 증거를 검토할 수 있다.

서버 결과:
`/root/docker_share/mujoko_debugging_data/20260928_covered_d1_server/index.html`

호스트에서:
`/home/rdv/docker_share/mujoko_debugging_data/20260928_covered_d1_server/index.html`

Covered MJB/XML은 NAS에 보존:
`/mnt/nas_rdv_md3/covered_d1_20260928/covered_scene/`

MJB SHA256:
`6eaf5178c99be858a89a3c6f1d3dc39d578c9d5196a5a4e2be537d7f7543d724`

**838MiB MJB와308MiB XML 자체는 Git에 올리지 않았다.** 위치/바이트수/해시/[생성 입력](validation/covered_d1_server/covered_assets.json)을 보존했다. Git만 clone한 외부 머신에서 물리 재실행하려면 기존 GLB·원본 서버 자산과 이 대형 자산에 접근해야 한다. 서버 공유 결과 폴더의 `covered_scene`은 NAS 링크다.

## 재현 명령

기존 출력 폴더는 덮어쓰지 말고 새 경로를 사용한다.

```bash
cd /root/farmily_tomato
PY=./mujoco-benchmark/.recovery-venv/bin/python
SOURCE=/root/docker_share/mujoko_debugging_data/20260927_stem_obstacle_scene
LAYOUT=/root/docker_share/mujoko_debugging_data/20260927_robot_side_dense_trusses
COVERED=/mnt/nas_rdv_md3/covered_d1_20260928/covered_scene
OUT=/root/docker_share/mujoko_debugging_data/NEW_DATETIME_covered_check

# 생성 자체를 반복하려면 --output을 새 경로로 지정한다.
# $PY mujoco-benchmark/scripts/add_neighbor_truss_obstacles.py "$SOURCE" --layout "$LAYOUT" --output NEW_COVERED_PATH
$PY mujoco-benchmark/scripts/add_neighbor_truss_obstacles.py "$COVERED" --audit --require-neighbor-stems --require-gutter
$PY mujoco-benchmark/scripts/validate_covered_scene.py "$SOURCE" "$COVERED" "$LAYOUT" "$OUT"
$PY mujoco-benchmark/scripts/covered_idle_server_probe.py "$SOURCE" "$COVERED" "$SOURCE/robot_checks/Tomato_02" "$OUT"
$PY mujoco-benchmark/scripts/render_covered_idle.py "$OUT" "$SOURCE" "$COVERED"
```

렌더는 저장 qpos만 사용한다. collision 색/표시 group 변경은 렌더 프로세스 안에서만 수행하며 MJB/물리 결과를 바꾸지 않는다. 이 renderer의 강조 pair는 본 blocker 전용이다.

## d1fdc7c와 달라진 이유 / 다음 행동

`d1fdc7c`는 별도 랜덤 장면의 활성2송이에서 접근/삽입 차단을 측정했다. 이번106송이는 기존 밀집 배경에 보이던 추가 송이들이다. 두 실험은 같은 환경이 아니므로 성공률 증감으로 비교하지 않는다. 이번 변화는 **예전에는 검사하지 못했던 시작점의 마운트↔배경 과실 충돌이 드러난 것**이다.

다음은 RL·seat 미세조정·샘플 증액이 아니라 **정상 초기 로봇 자세/배치 계약의 수정**이다. 환경·타깃·허용 접촉을 유지한 상태에서 마운트를 포함해 시작 자세가 겹치지 않아야 한다. 지금 교사 탐색이 실패하는 것이 아니라 그 이전 전제가 깨졌다. 이번 한 장면의 결과로 다른 모든 장면/타깃도 불가능하다고 일반화하지 않는다.

초기 자세를 변경하면 기록 prefix의 시작 q와 달라진다. 기존 B의 recorded prefix를 새 초기값으로 덮어쓴 뒤 “같은 prefix 비교”라고 해서는 안 된다. 원본은 initial-invalid로 보존하고, 새 초기조건의 접근 실험을 명시적으로 별도 비교로 정의해야 한다. 이후 PHASE1이 통과하면 D1의 최대11질의/8물리 예산으로 자동 이어간다. 예산을 채우기 위해 막힌 경로를 실행하지 않는다.

현재로서는 CASE A/C/D를 판단할 실행 증거가 없다. **CASE B보다 앞단인 initial-state blocker**를 해소하는 것이 다음 작업이다. 수확 성공이나 local RL 필요성을 이번 결과에서 주장하지 않는다.
