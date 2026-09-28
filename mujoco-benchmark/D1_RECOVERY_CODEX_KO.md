# Codex: d1fdc7c 기준 고정 비교 1라운드

## 목적

이번 브랜치는 실제 `d1fdc7c092fda78b66a6ed5e0da09522a94a6a55` 위에 추가했다. 이전 d8 기반 접근 연결기를 별도로 cherry-pick하거나 서버 수정사항을 되돌리지 않는다. 먼저 [검토 문서](D1_RECOVERY_REVIEW_KO.md)를 읽어라.

현재 실패를 한 종류로 취급하지 않는다. 원래 환경검사239개 중 최초 차단은 준비/접근139, 삽입/안착100이다. 네 경로군은48개의 공통 prefix를 공유한다. 접근이 이미 가능한 집단을 다시 접근 계획부터 고치지 않는다.

## 1. 적용·환경

- repo: `hjpark-rdv/tomato_robot_sim`
- 작업 기준 branch: `codex/d1-recovery-20260928`
- 필수 조상: `d1fdc7c092fda78b66a6ed5e0da09522a94a6a55`
- 서버: `rdv@192.168.222.27`, Docker `humble_x64_env`
- 기존 repo와 결과의 미커밋 변경을 보존하고 별도 worktree/작업 브랜치에서 진행한다.
- `git merge-base --is-ancestor d1fdc7c092fda78b66a6ed5e0da09522a94a6a55 HEAD`를 확인한다.
- 서버 후속 커밋이 더 있으면 이 브랜치를 일방적으로 덮어쓰지 말고 차이를 대조한다.
- 계획 환경에는 기존 Torch/FCL, 해당 MJB와 호환되는 MuJoCo, OMPL2.0.1이 함께 필요하다. 기존 가상환경을 임의로 업그레이드하지 말고 필요하면 격리 환경을 쓴다.
- 기존 native 회귀 + 별도 Torch/FCL 계획 회귀 + 새 테스트를 실행한다. CI 통과를 실제 온실 검증으로 간주하지 않는다.

## 2. 저장된 증거로 비교 목록 고정

예시:

```bash
DATA=/root/docker_share/mujoko_debugging_data/20260928_multifamily_server
OUT=/root/docker_share/mujoko_debugging_data/20260928_d1_recovery_round
# PLAN_PY는 위 의존성이 확인된 실제 계획용 Python 경로로 정한다.
"$PLAN_PY" mujoco-benchmark/scripts/prepare_recovery_batch.py \
  --campaign "$DATA/pilot" \
  --side-campaign "$DATA/side_repair" \
  --planning-python "$PLAN_PY" \
  --output "$OUT" \
  --run-plans
```

모델 반복 읽기가 병목이면 기존 hash 검증 캐시 디렉터리를 `--model-cache`로 지정한다. 캐시 크기와 RAM 여유를 확인한다. 최종 모델 링크가 NAS 원본으로 복원되는지 확인한다. OUT은 존재하지 않는 새 폴더여야 한다.

스크립트는 먼저 질의 목록을 파일로 저장하고 원본 prefix SHA를 대조한 다음 계획한다. 기본11질의:

A. 공통 prefix 차단4타깃: 고리 준비 위치·회전은 고정, 다른 IK branch와 우회 연결만 시험한다.
B. prefix 이후 차단2타깃: recorded / neutral / neutral_pitch15 각1개, 총6개. 원래 prefix가 재검사에서 통과해야 하며, 관절명령·phase·시간을 그대로 재사용한다. 이 집단에서는 OMPL로 접근을 변경하지 않는다.
C. 측면 자기충돌 대표1타깃: 기존 `robot_facing_mouth` 후보의 준비 목표를 고정하고 다른 관절 해/연결을 확인한다. 12개의 같은 표본 실패를 다시12개씩 실행하지 않는다.

neutral은 국소 roll/pitch/entry_twist를 모두0으로 한 단순화 대조이고, neutral_pitch15는 그 대조에서 pitch 하나만 바꾼다. 이는 원본과 한 변수만 다른 실험이라는 뜻이 아니다. 위 두 대조 사이의 차이는 pitch 하나다. 꼭지/와이어 관계 때문에 최종 위치도 달라질 수 있다. 기존 준비 구간은 이 네 경로군에서 이미 roll/pitch0이었으므로 낮은 기울기로 접근 자체를 고쳤다고 주장하지 마라.

계획 질의당 IK8개/최대3해/해당OMPL8초/전체120초/외부180초, 배치1500초를 유지한다. 중단·미평가를 불가능 판정으로 바꾸지 않는다. 추가 샘플을 무한 생성하지 마라.

## 3. 실제 실행은 최대8회

전체 경로가 통과한 결과를 먼저 실제 실행한다. `connection_result.json`의 `runnable_run`과 해당 candidate를 사용한다. 다시 기존 계획기로 Cartesian 후보를 계획하여 새 prefix를 지우지 마라.

전체 경로는 실패해도 별도로 검증된 `approach_only_runs`는 도달 확인용으로 사용할 수 있다. 이는 `manipulation_attempt=false`이며 걸림 시도나 성공 수에 넣지 않는다. 같은 타깃/같은 prefix의 도달 확인을 세 번 반복할 필요는 없다.

기존 실행 도구를 사용한다:

```bash
"$NATIVE_PY" mujoco-benchmark/scripts/target_fruit_contact_trial.py "$RUN" \
  --candidate "$CID" --base-policy "$BASE_POLICY" \
  --output "$TRIAL_OUT" --search-contact-policy --execute \
  --max-target-force-n 5 --max-target-displacement-m 0.02
```

실제 명령을 실행하기 전 RUN/CID/BASE_POLICY를 생성한 질의와 대조한다. 별도 물리 예산 장부를 만들고 접근-only와 중도 중단을 포함해 최대8회를 넘기지 않는다. 이번 배치 런처는 계획만 자동화하며 물리를 자동 실행하지 않는다.

목표 과실/자기 송이 Rachis/작업에 필요한 목표 꼭지의 정확한 고리 와이어 접촉 허용은 유지한다. 다른 과실·꼭지·송이, 주줄기·거터, 팔/마운트 접촉 허용을 넓히지 않는다. 물성·충돌체·margin/gap·마찰·강성·actuator·0.5mm 기준을 변경하지 않는다. 5N/20mm는 고정 실험 중단값이지 실물 안전 한계가 아니다.

## 4. 결과와 종료

집단 A/B/C 각각에 대해 다음을 표로 보인다:
원본 최초 차단 / 원본 prefix 재검사 / prefix 재사용 여부 / 유효 IK 해 수와 끝 자세 차단 원인 / 우회 연결 / 국소 계획 / 전체 검사 / 실제 실행 / 실제 도달과 단계별 접촉·변위 / 기하 후보 / 접촉 유지 증거.

- source의 모든 수정 코드·캐시·허용쌍 최적화가 유지됐는지 확인한다.
- B집단은 실제 prefix command SHA·phase·시간 동일성을 확인한다. suffix는 도달 해에서 재계획하므로 원본 전체와 bitwise 같다고 주장하지 않는다.
- C집단은 끝 자세 자체의 link3 충돌인지, 이동 중 충돌인지 구분한다. 단순 IK 미발견으로 구조적 불가능을 확정하지 않는다.
- 검증된 실제 qpos 영상3~4개를 우선 저장한다. 실제 물리가 없으면 명목 영상으로 그 사실을 표시한다.
- 기하상 근접 후보를 접촉0이라는 이유만으로 실물 실패라 하지 말고, 근접만으로 수확 성공도 선언하지 않는다.
- `training_eligible=false`, `hook_success=null`, `impossible=null`을 유지한다.

이 목록을 끝내면 한 라운드를 종료한다. 80열매 확장이나 새 장면 생성부터 다시 하지 마라. 새 해가 전혀 없으면 같은 방식을 증액하는 대신 실패 집단별 다음 선택을 보고한다.

A는 접근 개선, B는 국소 단순화 개선을 독립적으로 평가한다. A가 실패해도 이미 유효한 B의 국소 연구까지 대기시키지 않는다. 접근이 유효하지만 국소 동작이 계속 막히면 다음 단계로 같은 준비 상태에서 국소 경로 최적화와 국소 RL의 계산 예산을 맞춘 비교안을 제안한다. 이번 라운드 중 reward를 임의 변경하며 전체 RL을 시작하지 않는다.

기존 보고서, 이번 비교표, 원본/변경 명령 해시, 실행 로그와 대표 영상을 함께 보존한다. 필요한 서버 연결 오류만 테스트와 함께 수정하여 별도 작업 브랜치에 commit/push하고 정확한 SHA를 보고한다.
