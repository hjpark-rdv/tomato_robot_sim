# Codex 전달문: 접근 연결을 한 라운드로 검증하고 다음 방향을 결정

## 목적과 현재 판단

이번 목적은 후보 종류를 계속 늘리는 것이 아니라, 기존 준비/접근 보간을 실제 장애물 회피 계획으로 바꿨을 때 새 경로가 물리 실행에 도달하는지 확인하는 것이다. 04번 마지막 간격이나 힘 제한 상향으로 되돌아가지 않는다.

사용자 전달문 기준으로 새 240개 후보는 실제 물리 실행이 0회이며, 92개는 경로군이 갈라지기 전에 막혔다. 따라서 이번 결과만으로 RL이 필요 없거나, 반대로 RL이면 해결된다고 결론 내릴 수 없다. 우선 경로 접근성 문제와 국소 접촉 동작 문제를 분리한다.

## 소스 기준을 먼저 바로잡는다

- repo: hjpark-rdv/tomato_robot_sim
- 새 브랜치: codex/approach-connection-20260928
- 기능 커밋: 1be29eaa88fdfa7148496091b3c628ea4df5d0ba
- 실제 검토한 기반: d8db76d2d3a7e62081cf15c1f26a32c538676eed

현재 채팅의 연결된 GitHub API에서는 codex/server-multifamily-20260928 및 해당 보고서가 404였다. 서버 결과 숫자는 사용자 전달문에서 읽었고, 최신 원자료/영상/서버 수정 소스를 독립 검증한 것은 아니다.

먼저 서버의 실제 최신 branch/HEAD/remote를 확인하고, 위 서버 브랜치를 올바른 저장소로 push했는지 확인해라. 정확한 commit SHA와 보고서 URL을 남긴다. 기존 작업은 보존하고 최신 서버 브랜치 위 별도 worktree에 새 기능 커밋만 cherry-pick해라. 이전 기반 d8db로 서버를 되돌리지 마라. 새 커밋은 새 파일만 추가한다. 허용쌍 조회 최적화, MJB 캐시, robot-facing-mouth 등 서버 변경을 보존한다.

기존 미커밋 변경, 원본 결과, 물성을 지우거나 덮어쓰지 마라. reset --hard/clean/강제 checkout은 하지 마라.

먼저 읽을 문서: APPROACH_CONNECTION_REVIEW.md

## 구현된 것

- approach_connection.py: 기존 IK를 여러 seed로 시도하고 실제 FK 잔차/자기충돌/환경 충돌을 확인한 관절 해를 선택한다. 정규화된 관절 공간에서 OMPL RRTConnect를 호출한다.
- plan_approach_connection.py: 기존 preapproach/approach 전체 prefix를 교체한다. 도달한 관절 자세에서 원래 insert/seat/hold/verify를 기존 계획기로 다시 생성한다.
- 검색 중 기존 FCL 및 MuJoCo 형상 검사를 사용한다. OMPL의 근사 해는 통과시키지 않고, 경로 내부와 조합된 최종 trace를 다시 검사한다.
- 국소 suffix가 막혀도 독립적으로 검사한 approach_only_run을 남겨 실제 접근 도달 여부만 시험할 수 있다. 이는 manipulation_attempt=false이며 고리걸기 성공 분모에 넣지 않는다.
- 생산 경로 생성기, 물리, 접촉 정책, 성공 판정은 변경하지 않았다. 새 도구는 물리를 실행하지 않고, 새 trace를 기존 실행기에 전달하는 opt-in 연결기다.

## 1. 환경과 API 연결 점검

서버 rdv@192.168.222.27 / Docker humble_x64_env / /root/farmily_tomato.

기존 계획 환경(Torch/FCL)과 실제 MJB 버전에 맞는 MuJoCo에 ompl==2.0.1을 사용할 수 있는 격리 환경을 준비해라. 기존 작동 환경을 덮어 업그레이드하지 마라. OMPL 2.x allocState/direct callback API를 사용한다. 과거 OMPL 1.x 코드로 임의 변경하지 마라.

새 test_approach_connection.py와 기존 서버의 계획기 회귀 시험을 실행한다. native CI는 작은 검증 모델이며 전체 RB5/온실 통합 시험이 아니다. 이번 서버 연결에서 오류가 발견되면 원인을 좁혀 고치고 시험을 추가한다. 라이브러리 설치/인터페이스 오류를 수확 불가능으로 집계하지 않는다.

## 2. 대상과 비교 조건을 먼저 고정

이번에는 새 장면을 생성하지 않는다.

- 기존 prefix 단계에서 막힌 타깃 4개: 최소 2개 유효 장면, 가능하면 두 번째 송이 포함.
- 기존 삽입 이후까지 갔던 타깃 2개는 단계별 대조군으로 기록.
- prefix 타깃당 준비 자세 3개만 선정: 총 12개 준비 자세 질의.
- 가능하면 기존 후보 중 낮은 기울기를 먼저 선정한다. 그런 후보가 없으면 작은 기울기 기준 후보를 명시적으로 만들고 새로운 후보임을 기록한다.
- 각 질의는 원래 보간과 새 연결기가 동일한 목표 위치/회전/초기 상태/정책을 사용한다.
- 방위각, roll, pitch, 높이, standoff를 동시에 바꾸지 않는다. 새 connector 비교에서 어느 변경이 효과를 냈는지 분리한다.
- 4개 경로군의 같은 prefix를 4개의 독립 성공/실패로 세지 않는다. 동일한 준비 자세로 묶어서 집계한다.

## 3. 한 번의 실행 상한

질의당 IK seed 8개, 최대 관절 해 3개, 관절 해당 OMPL 8초, 전체 계획 120초다. 외부 subprocess watchdog도 180초로 둔다. 전체 새 물리 실행은 12회 이내이며 접근-only 실행과 중도 중단도 포함한다. 원래 준비 자세 질의 12개와 대조군의 기존 기록을 먼저 비교하고 무한 증액하지 마라.

```bash
timeout --kill-after=10s 180s "$PLAN_PY" \
  mujoco-benchmark/scripts/plan_approach_connection.py "$RUN" \
  --candidate "$CID" --base-policy "$BASE" --output "$OUT" \
  --ik-seeds 8 --max-branches 3 --solve-seconds 8 \
  --total-seconds 120 --seed 20260928
```

RUN은 실제 replay_assets/candidates.json/planning_inputs를 가진 기존 run이다. BASE는 해당 타깃의 기존 base_policy.json이며, 이미 확장된 contact_scope snapshot을 대신 넣지 마라. OUT은 기존 run 밖의 새 폴더다.

완주용 새 run은 전체 trace 검사를 통과했을 때만 OUT/run에 생성된다. suffix가 실패해도 branch_XX/approach_only_run이 있을 수 있다. 이 접근-only run은 준비 자세 도달 후 1초 hold이며, 고리 안착/수확 시도로 취급하지 않는다. 후보 ID는 _approach로 끝난다.

원래 Cartesian 후보 목록을 다시 plan_candidates에 넣어 prefix를 되돌리지 마라. 실행은 새 trace.json을 기준으로 한다. 전체/접근-only 각각의 manifest, params, plan, trace 해시를 보존한다.

## 4. 실제 물리 검증

기존 target_fruit_contact_trial 실행기를 --search-contact-policy와 동일한 5N/20mm 실험 제한으로 사용한다. 검사가 불충분하거나 실패한 trace를 억지로 실행하지 않는다. 목표 과실과 자기 송이 중심가지의 고리 와이어 접촉은 기존 허용 범위를 유지한다. 다른 과실/꼭지/송이, 주줄기/거터, 마운트/팔 접촉을 추가 허용하지 마라.

물성, collision mask, margin/gap, 마찰, 강성, actuator, 기존 0.5mm 수치 유효성 기준을 바꾸지 않는다. 5N/20mm는 손상/실물 안전 한계가 아니라 이번 비교의 중단값이다.

접근-only에서 확인할 것은 실제 TCP/FK 목표 오차, 계획과 추종 차이, 금지 접촉 여부, 유지 중 실제 위치다. 기하 prefix 통과와 실제 도달을 별도로 보고한다. 접촉 허용 때문에 식물이 이동했다면 그 상태 차이도 보존한다. 목표 걸림은 이후의 별도 지표다.

## 5. 이번 라운드 종료·전환 조건

4개 prefix 타깃 중 적어도 2개에서 검사 통과 경로와 실제 준비 위치 도달이 확인되면 접근 계획 보강을 멈추고, 그 도달 상태를 기준으로 국소 경로 3종만 비교하는 다음 단계로 간다. 2/4는 통계적 성공률 주장이 아니라 사전에 정한 실무적 진행 기준이다.

이 기준에 못 미치면 이번 라운드를 종료하고 아래 중 어느 실패인지 보고한다.
- 시작 관절 상태 자체가 현재 모델/정책에서 유효하지 않음
- 실제 FK/관절 범위에 맞는 준비 자세 IK를 못 찾음
- IK는 있으나 고리-팔/link3 또는 환경과 끝 자세가 충돌함
- 유효한 시작/끝은 있지만 정해진 시간에 연결 경로를 못 찾음
- 접근은 찾았지만 insert/seat에서 국소 경로가 막힘
- 명목 경로는 있으나 실제 추종/접촉/물리 유효성이 실패함

무조건 seed/time/각도 수를 계속 늘리지 마라. 마지막 1.4mm 조정이나 힘 제한 상향으로 돌아가지 않는다. 여전히 impossible=null이며, 유한 탐색 실패를 절대 불가능으로 쓰지 않는다.

## 6. RL을 검토할 조건

준비 위치까지 도달하는 경로가 확보됐는데, 여러 유효 준비 상태에서 국소 삽입/회전 동작이 반복 실패한다면 국소 task-space RL 또는 residual RL과 제한된 궤적 탐색을 비교하는 다음 실험을 제안해라.

그 비교에서는 동일한 초기 상태, 허용 접촉, 관측/제어 정보, 물리-step 예산을 사용하고 seed 3개를 둔다. 학습 wall time과 실제 physics step 수를 보고한다. 이번 작업 중 임의로 전체 관절 RL 학습을 시작하지 않는다.

SIM-GT 교사가 매 스텝 숨겨진 꼭지 위치를 알고 행동하는 성능을 초기 RGB-D 한 장 학생의 성능과 혼동하지 않는다. 형상 근접, 접촉 유지 증거, 수확 성공은 계속 분리한다. 부족한 접촉/평가 모델을 RL이 자동 보완한다고 가정하지 마라.

## 7. 최종 제출

타깃/준비 자세별 표:
원래 prefix 결과 / 유효 IK 해 수 / 끝 자세 충돌 / 새 prefix 발견 /
전체 trace 통과 / 실제 도달 / 이후 국소 결과 / wall time / 검사 수.

대표 4개 내외 영상: 원래 차단 계획, 새 계획, 실제 접근-only 도달, 실제 전체 동작이 있으면 포함한다. 명목 영상과 실제 qpos 영상을 분명히 구분한다. 성공한 것만 선별하지 않는다.

최신 서버 변경 위에 적용한 commit과 테스트 로그, 입력/정책/코드 해시, source 불변 검사, 남은 실패 원인을 commit & push한다. 80개 전체 열매 또는 새 장면 확장은 접근과 국소 실행이 실제로 여러 타깃에서 이루어진 뒤에 제안한다. training_eligible=false, hook_success=null을 유지한다.
