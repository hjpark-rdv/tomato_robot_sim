# 접근 계획 결정 실험: 검증 및 Codex 전달문

## 실제 완료한 것

기능 커밋: `5739a7b9c8425be0515ad55b841fa143164afc15`.
기준: `d8db76d2d3a7e62081cf15c1f26a32c538676eed`.
브랜치: `codex/access-planning-decision-20260928`.

[GitHub Actions 36368542276](https://github.com/hjpark-rdv/tomato_robot_sim/actions/runs/36368542276)의 job `108759748107` 로그를 직접 확인했다.

- Python 3.11.16 / MuJoCo 3.13.0 / OMPL 2.0.1 / Torch 2.14.0+cpu.
- 새 route 시험 12개, native 시험 3개, 기존 진단 계획기 회귀 5개: **20 passed, 건너뜀 없음**.
- 실제 OMPL RRTConnect와 MuJoCo 거리 검사를 사용해 작은 2축 로봇의 직선 차단/우회 성공을 확인했다.
- 벽으로 막힌 작은 문제에서 근사 해를 실행 경로로 받아들이지 않는 것을 확인했다.
- 현재 선택한 IK branch에서 기존 `dataset_motion.plan`의 local suffix를 재계획하고 입력을 보존하는 합성 FK 시험을 통과했다.
- 기존 frozen legacy command 회귀도 통과했다.

이 시험의 일부 IK와 자기충돌 checker는 합성 구현이다. 실제 온실의 RB5/FCL/학습 자산을 이용한 CLI 전체 실행, 물리 추종, 새 고리걸기 영상은 **미검증**이다. 20개는 수확 성공 수가 아니다. 새 코드의 native API와 연결 계약을 검증한 수다.

요청된 `codex/server-multifamily-20260928`은 검토 시점의 연결에서 404였다. 서버 통계는 사용자가 전달한 보고이며 원자료와 영상의 독립 검토를 수행했다고 주장하지 않는다. 기능 커밋은 **새 파일 7개만 추가**한다. 서버에서 최신 작업 위에 cherry-pick하되 기존 파일을 되돌리지 않는다.

## 비교 해석에서 주의할 점

`legacy` 대 `multiseed-direct`는 IK seed뿐 아니라 이전 preapproach 경유점을 필수로 고정하지 않는 차이도 있다. 두 결과의 차이를 전부 IK seed의 효과라고 쓰지 않는다.
`multiseed-direct` 대 `ompl`은 같은 staging pose/seed 집합에서 직선 연결과 우회 연결을 비교한다. 다만 유효 경로를 찾는 순서에 따라 최종 선택한 IK 해가 달라질 수 있으므로 staging_q와 branch를 함께 보고한다.

`access-only`의 hold는 준비 위치 도달 시험이다. 기존 trial/evaluator가 기하 후보 값을 반환하더라도 수확/삽입의 긍정 라벨로 집계하지 않는다. 전체 삽입 동작과 서로 다른 분모로 관리한다.

현재 검사는 고정 초기 환경의 이산 기하 검사다. 허용된 과실/자기 송이 접촉을 제외하는 것은 가벼운 힘을 보장하는 것이 아니며 실제 물리는 같은 감시 조건으로 실행한다. 환경을 밀어야만 생기는 통로는 이번 free-space 접근 계획의 해결 범위가 아니다.

## Codex 전달문

이번에는 새 경로군이나 중단값을 더 늘리지 않는다. 기존 막힌 접근을 표준 관절공간 계획기로 해결할 수 있는지 한 라운드로 결정한다.

### 0. 작업 기반 보존

- 서버: rdv@192.168.222.27 / Docker humble_x64_env.
- 저장소: /root/farmily_tomato, 데이터: /root/docker_share/mujoko_debugging_data.
- 실제 서버 최신 branch/HEAD/dirty 상태부터 기록한다.
- `codex/server-multifamily-20260928`의 최종 결과가 원격에서 조회 가능한지 확인하고 정확한 SHA를 보고한다.
- 기존 서버 결과 위의 새 작업 branch 또는 worktree에 `5739a7b9c8425be0515ad55b841fa143164afc15`를 cherry-pick한다.
- 이 문서 커밋도 읽되 원본 서버 수정, policy lookup 최적화, MJB cache, robot-facing-mouth, 영상/로그를 덮어쓰지 않는다.
- 강제 reset/clean/checkout, 시스템 Python 변경을 하지 않는다.

### 1. 환경과 대표 사례 확정

`ACCESS_PLANNING_DECISION_KO.md`와 새 CLI를 읽는다.
계획 Python에는 기존 Torch/FCL, MuJoCo 3.13.0, OMPL 2.0.1이 함께 필요하다. 별도 가상환경에서 기존 의존성을 보존하고 설치한다. 패키지 import와 테스트를 먼저 실행한다.

새 테스트 15개와 기존 진단 계획기 5개를 실행한다. 서버의 기존 관련 회귀도 가능한 범위에서 확인하되 기능 연결 오류를 수확 불가능으로 기록하지 않는다.

기존 통과 4장면에서 최소 3개 다른 타깃을 포함하는 대표 후보 6개를 먼저 고정한다:
공통 준비 경로 차단 2, 더 늦은 접근 차단 2, 측면 자기충돌 1, 가장 늦게까지 진행한 1.
실제 기록과 이 분류가 다르면 추측하지 말고 실제 실패 단계로 선택 근거를 적는다.
새 장면을 만들거나 성공한 사례로 교체하지 않는다.

### 2. 같은 사례에서 세 가지 접근 비교

각 후보의 마지막 approach 위치/회전과 local suffix를 그대로 두고 아래를 비교한다.
- legacy: 기존 필수 경유점과 단일 IK 동작.
- multiseed-direct: 최대 8개 IK seed, 유효 endpoint 최대 3개, staging까지 관절 직선.
- ompl: 같은 IK seed/범위, 직선이 막히면 RRTConnect 우회.

각 mode에 `--access-only`를 적용해 준비 위치까지의 경로만 먼저 평가한다.

```bash
timeout 240s "$PLAN_PY" mujoco-benchmark/scripts/plan_access_candidate.py "$RUN" \
  --candidate "$CID" --base-policy "$BASE" --output "$OUT" \
  --mode ompl --access-only --ik-seeds 8 --max-branches 3 \
  --seconds 30 --solve-seconds 5 --seed 20260928
```

RUN은 기존 실제 replay_assets와 planning_inputs를 포함하는 run root다.
BASE는 확장된 trial policy가 아닌 기존 pedicel/rear-wire base policy다.
각 OUT은 새 폴더로 만들고 원본 run 아래에 쓰지 않는다.

OMPL이 선택한 staging_q, endpoint FK residual, 상태/거리 조회 수, 소요시간,
자기충돌과 환경충돌의 최초 대상, 접근 성공과 local 성공을 각각 기록한다.
시간초과는 아직 경로를 못 찾은 것이지 불가능 증명이 아니다.
OMPL 없는 환경에서 직선 대체를 성공처럼 표시하지 않는다.

### 3. 준비 위치 실제 도달 및 원래 local 동작 연결

전체 검사를 완료한 output run만 기존 `target_fruit_contact_trial.py`로 정상 초기 상태부터 실행한다.
`--search-contact-policy --execute --max-target-force-n 5 --max-target-displacement-m .020`을 사용한다.
실제 위치와 목표 staging 위치, 추종 오차, 정지 이유를 확인한다.
access-only는 별도 분모이며 아무 걸림 라벨도 얻었다고 집계하지 않는다.

최소 3개 타깃에서 실제 준비 위치 도달이 확인되면, 그 타깃들의 일부에서
`--access-only` 없이 원래 local suffix를 재계획한다. 준비 도달이 2개면 그 두 개까지만 탐색적 local 확인하고 확장 결정을 보류한다.
새 planner는 도착한 실제 계획 IK branch를 고정하고 suffix를 다시 계산한다.
이전 branch의 suffix trace를 단순히 이어 붙이지 마라.
새 output에 일반 plan_candidates.py를 다시 돌리면 옛 prefix로 재생성되므로 하지 않는다.

로컬 시작부터 기하가 막히면 어느 위치/회전/링·마운트·다른 물체 쌍인지 보고한다.
자기충돌을 없애려고 고리/link3 허용쌍을 추가하지 않는다.

### 4. staging pose 자체가 문제일 때만 저기울기 비교

기존 gate가 모든 IK branch에서 막히는 사례 최대 2개에만 access_sampling.py의
낮은 기울기 -45/0/+45도 heading 기준을 기존 make_candidate에 연결한다.
해당 helper는 자동 runner에 연결되어 있지 않으므로 작은 서버 orchestration에서 호출하고,
생성된 실제 pose_waypoints가 원래 후보와 어떻게 다른지 저장한다.

다음 기울기 시험은 one_factor_tilts로 한 항목만 변경하고 나머지는 고정한다.
이번 예산에서 새로운 전체 14차원 무작위 탐색으로 확대하지 않는다.
robot-facing-mouth의 서버 변경은 유지하고 실제 손목/마운트 자기충돌을 계속 검사한다.

### 5. 고정 예산과 종료 결정

계획은 18개 paired access + 최대 6개 local 연결 + 최대 6개 낮은 기울기 비교 = 최대 30개.
정상 시작 물리 실행은 access-only를 포함해 최대 8회다. 중도 중단도 실행 횟수에 포함한다.
실험 캠페인은 설정/의존성 준비 이후 최대 60분이며 개수/시간 중 먼저 도달하면 종료한다.
예상보다 느리면 소화 못 한 후보를 미평가로 남기며 같은 실패를 무한 재시도하지 않는다.

- 여러 타깃의 실제 접근이 확보되면 local 방법 비교 단계로 진행한다.
- 접근 0~1개, 대부분 timeout이면 표본/장면/물리 한도를 늘리지 않는다.
- 이 경우 느린 상태 검사, 잘못된 시작/목표, 유효 IK 부재를 구분해 다음 기술 선택을 보고한다.
- 접근은 되지만 모든 local 경로가 막히면 contact-aware 최적화 또는 local RL의 짧은 대등 비교가 다음이다.
- 이번 작업에서는 전체7관절 RL, 추가 힘 상향, 마지막1.4mm 보정, 전체80열매 확대를 시작하지 않는다.

### 6. 영상과 결과

동일 카메라에서 기존 막힌 명목 경로와 새 경로, 실제 접근/전체 실행을 비교한다.
최소 대표3개: 새로운 우회, 새로운 IK 팔 모양, 남은 실패를 보여준다.
명목 영상과 실제 물리 저장 qpos를 혼동하지 않는다.

최종 표는 candidate/mode/staging_pose/IK branch/FK residual/access plan pass/
actual staging reached/local plan pass/physics executed/geometry/contact evidence/
stop reason/state queries/wall time을 포함한다.

기하 후보, 접촉 유지 증거, 성공 미확인, 계획 실패, 실험 오류를 유지한다.
기하 간격만으로 실물 실패를 선언하지 말고 시뮬 부족을 이유로 실물 성공도 선언하지 마라.
재현 가능한 관절경로를 확보했는지가 이번 핵심 산출물이다.

서버 오류 수정이 필요하면 원인을 좁혀 테스트와 함께 commit/push한다.
최종 commit, 변경 목록, 전체 원자료 위치, 영상, 못한 작업을 보고한다.
원래 학습 데이터와 action14에 합치지 말고 `training_eligible=false`, `hook_success=null`, `impossible=null`을 유지한다.
