# ca83357 검토: 꼭지 기준 목표 자세와 다음 서버 실험

기준: `ca833579ef0141e999ab9628d527f4636a628a9e`.
작업 브랜치: `codex/hook-seating-goals-20260927`.
기존 운영 코드·물리·성공 라벨·저장 데이터·사전검사 정책은 변경하지 않는다.

## 1. 기존 결과에 대한 판단

원본 [서버 보고서](ENVIRONMENT_PREFLIGHT_SERVER_REREVIEW_RESULTS_KO.md),
[매핑](validation/preflight_rereview_20260927/mapping.json),
[gutter02 동일 시각 표본](validation/preflight_rereview_20260927/sync_gutter02/representative_samples.json),
새 진단 코드와 기존 경로 생성기를 대조했다.

- 7경로 모두 blocked라는 결과와, 전체 검사 완료/상세 생략의 구분은 유지한다.
- 양의 간격에서의 비목표 힘은 수치 표본으로도 확인된다. 예: 21.4s의 private D 거리 약0.869mm, 법선력 약0.734396N. 실제 직전 solver 힘과 재계산 힘을 구분한다.
- 5회 부분 중심 진입과 목표 안착0회는 서로 모순이 아니다. 완성된 수확 성공이나 물리적 불가능을 주장할 근거도 아니다.
- 여기서 새 온실 물리를 실행하거나 4개 MP4 전 프레임을 독립 재검증한 것은 아니다. 압축 원자료 전체의 새 재집계도 수행하지 않았다.

## 2. 소스에서 확인한 핵심 병목

`dataset_design.waypoints()`의 staged6d 분기는 `trajectory_search.waypoints()`로 간다.
그 함수는 `neck`를 받지만 위치 계산에서 사용하지 않는다. `hook_roll_deg`와
`approach_elevation_deg` 역시 현재 staged6d 계산에 반영되지 않는다.
과실 중심/반지름과 고정 삽입·상승 파라미터가 경로를 결정한다.
`dataset_motion.plan()`은 모든 waypoint에 같은 rotation을 사용한다.

따라서 dictionary에 Roll/Pitch 항목만 추가하거나 azimuth를 조금 변경하는 것은
각 꼭지와 뒤쪽 걸림면의 상대 자세를 직접 맞추는 작업이 아니다.
이는 구조적 한계이며 이번0/5의 모든 원인을 확정했다는 뜻은 아니다.

기존 `rear_capsule_geometry()`는 gap의 상한만 검사한다. 와이어를 관통해 gap이
음수인 자세도 기하 seated가 될 수 있다. 기존 `HookProbe.summary()`의
legacy_retention_proxy는 물리 유효성/비목표 접촉과 독립 필드이며, 진입 이력이나
기계적 이탈 저항을 증명하는 평가기도 아니다. `hook_success=null` 보존은 적절하다.
이 기존 함수/데이터를 덮어쓰지 않았다.

## 3. 이번에 추가한 코드

### `scripts/hook_seating_geometry.py`

- 실제 유한 꼭지 캡슐과 뒤쪽 와이어 캡슐을 입력으로 받는다.
- 꼭지 중심선과 와이어 중심선의 공통 법선을 이용해 지정한 표면 간격을 만드는
  RING 중심 위치와 회전을 계산한다. 고리 바디 원점과 RING 중심은 다르다.
- 제안 후 모든 뒤쪽 와이어와의 유한 선분 거리를 다시 검사한다.
- 기하 음성 대조: 입구 쪽, 너무 멀리 통과, 와이어 바깥, 와이어 관통,
  유한 중심선이 평면을 통과하지 않는 경우, 공면/평행의 모호한 경우.
- 유한 중심선 관통 조건은 보수적인 새 진단의 범위다. 끝단만 접촉하는 모든
  실제 걸림 가능성을 배제했다는 뜻은 아니다.

### `scripts/propose_hook_seating_goals.py`

- 현재 `load_engine`, `HookProbe`의 실제 target/rear ID 매핑과 `capsule_endpoints`를 재사용한다.
- 기존 명목 회전 주변의 local X/Z 기울기와 선택적인 SIM-GT 꼭지축 정렬을 사용한다.
- 목표 꼭지 내부 위치와 접촉할 뒤쪽 와이어를 바꾸면서 목표 자세들을 만든다.
- 출력 제한 때문에 모든 기울어진 자세가 사라지지 않도록 target/rotation family별로 순환 선택한다.
- `seating_goals.json`은 진단용 sidecar다. 기존 action14 형식이나 학습 데이터를 바꾸지 않는다.
- 모든 결과는 `diagnostic_only=true`, `offline_teacher_goal=true`,
  `physics_executed=false`, `training_eligible=false`, `hook_success=null`이다.
- 실제 로봇 IK, 고리의 나머지 형상(g410 등), 환경 충돌, 진입 경로, 식물 변형,
  안착/유지/당김은 아직 검증하지 않는다. 이 파일은 실행 가능한 trace가 아니다.

### 검증 범위

로컬에서는 NumPy/SciPy 기하·입력·좌표 변환·출력 제한 시험을 실행했다.
Native MuJoCo 시험과 기존 회귀시험은 `Hook seating geometry tests` Actions 결과로 확인한다.
양성 기하 fixture는 목표 생성기로 정답을 만든 것이 아니라 별도 알려진 좌표로 정의했다.
실제 native capsule distance와의 비교는 별도 구현의 교차검증이다.
정적 목표 자세 fixture는 자동 진입/수확 성공 표본이 아니다.

## 4. 서버에서 첫 실행

미커밋 변경을 보존하고 ca83357을 포함하는 이 브랜치를 별도 worktree/작업 브랜치에서
사용한다. reset --hard, clean, 원본 결과 덮어쓰기를 하지 않는다.

기존 gutter02 run에서 기하 목표만 내보내는 예:

```bash
RUN=/root/docker_share/mujoko_debugging_data/20260927_gutter_collision_scene/robot_checks/Tomato_02
OUT=/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_hook_seat_goals
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/propose_hook_seating_goals.py "$RUN" \
  --candidate predicted_00000 --output "$OUT" \
  --surface-gap-m 0.0004 --tilt-deg -15 0 15 --fraction 0.5 \
  --include-axis-aligned --max-goals 32
```

0.4mm와±15도는 오프라인 기하 탐색의 명시적 예제값이며 실물 안전 여유나 검증된
최적값이 아니다. 현재 native margin에서는 양의 gap에서도 힘이 발생할 수 있다.
이 값을 바꾸더라도 모델 margin/gap이나 기존 clearance를 바꾸지 않는다.
종료코드2는 이 제안군에서 유효 기하 목표를 내보내지 못한 것이며 수확 불가능이 아니다.
이 CLI를 실제 사용자 온실 run으로 끝까지 실행하는 검증은 서버에서 해야 한다.

## 5. Codex 다음 작업 지시

목표는 진단을 무한 확장하는 것이 아니라 **올바른 꼭지 안착을 만드는 경로1개와
검증 가능한 대조군**을 확보하는 것이다. 우선 Tomato_02 한 개, 이후1~3개만 진행한다.

1. 현재 소스/HEAD/작업 트리를 확인하고 이 브랜치의 테스트를 실행한다. 기존65시험과
   새 기하/native 시험의 pass/skip을 구분한다. 자산과 코드는 해시로 연결한다.
2. 실제 run에서 위 CLI를 실행한다. target_capsules와 rear_wire의 위치를 원본 CAD에
   겹쳐 보고 이름/반지름/축/RING 오프셋을 확인한다. 메타데이터가 일치하지 않으면 중단한다.
3. 기존 `HookProbe`와 새 기하 검사를 비교할 양성·음성 fixture를 먼저 만든다.
   아래 대조군 표를 따른다. 판정기를 통과하도록 정답을 그 판정기에서 역생성하지 않는다.
4. 목표 자세 중 일부를 기존 IK/FCL/환경 검사에 연결한다. 현재 plan의 단일 rotation
   제약을 먼저 확인한다. 필요한 경우 명시적인 waypoint별 pose를 처리하는 opt-in
   진단 경로만 추가하고, 기존 planner의 공통 제어/검증 부분을 재사용한다.
   기존 모드의 명령 배열이 바뀌지 않는 회귀시험을 추가한다.
5. 기존 준비/접근을 사용할 수 있는지 검사하되 충돌 경로를 고정 prefix로 강제하지 않는다.
   안전한 준비 → 삽입/회전 → 안착 목표 → hold → 작은 확인 동작을 연결한다.
   시작부터 걸어놓거나 저장 상태를 경유점으로 teleport한 실험은 진입 성공으로 세지 않는다.
6. 의도된 접촉은 target distal/terminal proximal과 실제 rear wire의 정확한 쌍 및
   seat/hold/확인 단계에만 한정한 **별도 진단 정책**으로 기록한다. 기존 정책은 보존한다.
   Rachis·다른 과실·다른 꼭지·주줄기·거터·마운트는 자동 허용하지 않는다.
   새 기하 predicate의 비관통 제한을 동적 soft-contact 물리 기준과 혼동하지 않는다.
7. 단계마다 결과를 분리한다: 기하 목표 있음 / IK 가능 / 전체 경로 검사 /
   실제 안착 / 유지 / 비목표 힘 / 물리 유효성. 실패 시 어느 단계에서 막혔는지 남긴다.
8. baseline1회 + 새 경로 최대12회의 오프라인 물리 예산으로 우선 비교한다.
   목표 종류와 기울기 다양성을 보존하고, 결과가 좋은 것만 보고하지 않는다.
   막힌 계획을 강제로 실행한 원인 조사 사례는 gate 통과 결과와 구분한다.
   새 동작의 유효 안착이 확인된 경우에만 작은 위치/자세 오차 시험을 붙인다.
9. 지금은 학습/RL/배경106송이 탄성화를 시작하지 않는다. GT 목표는 오프라인 교사
   탐색 전용이며 실제 단일 RGB-D에서 얻는 정보로 간주하지 않는다.
   새 출력은 waypoint별 회전이 달라질 수 있으므로 기존 action14로 축약해 학습하지 않는다.
10. 기존/새 동작/양성 대조/대표 음성 대조를 같은 시점/카메라로 MP4 저장한다.
    목표 꼭지, rear wire, Rachis를 구별하고 명목 경로와 실제 상태를 구분한다.
    자동 진입과 시작부터 안착한 fixture 영상을 명확히 다른 제목으로 표시한다.

### 양성·음성 대조군과 판정 분리

|대조군|기대 해석|
|---|---|
|알려진 rear 내부 자세의 목표 꼭지, 비관통|기하 안착 검사의 양성. 진입/유지 성공은 아님|
|허용된 목표쌍의 실제 힘과 안착, 정상 시작에서 진입 후 유지·확인 동작|기계적 유지 증거의 양성 후보. 독립 영상/물리 기준 함께 확인|
|처음부터 걸린 자세에서 정지|평가/접촉 제어 fixture일 뿐 자동 진입 성공 아님|
|과실 중심만 고리 안, 꼭지는 안착하지 않음|중심 진입 양성일 수 있지만 꼭지걸림 음성|
|고리 앞쪽/와이어 바깥/끝단만 가까움|rear 내부 안착 기준의 음성 또는 지원범위 밖|
|와이어 깊은 관통|기하 invalid. legacy seated=true여도 성공으로 처리하지 않음|
|다른 꼭지 또는 Rachis에 힘 발생|목표 정체성 조건 실패|
|gap 양수·contact 존재하지만 힘0|접촉력 증거 없음|
|잠깐 안착 후 hold/확인 동작에서 이탈|유지 실패|
|목표 없이도 같은 판정 양성|판정기 오류. 동작 최적화 전에 수정|

hold는 timestamp/최대 sample gap/실제 시간 범위로 확인하고 표본 개수만으로
1초를 주장하지 않는다. `HookProbe`는60Hz라서 짧은 접촉을 놓칠 수 있다.
핵심 접촉 이벤트와 물리 유효성은240Hz 또는 실제 physics 주기로 보존한다.
고리와 꼭지를 weld하거나 마찰/강성을 바꿔 유지 양성을 만들지 않는다.

최종 보고: commit/설정/대조군별 지표, 목표→IK→검사→실행 각 단계 개수,
전체 실패·보류, 상대 자세 잔차, 비목표 힘, 영상, 미검증 범위를 포함한다.
기하 helper가 생성한 자세를 그 helper로 재검사한 결과만으로 물리 성공을 선언하지 않는다.
