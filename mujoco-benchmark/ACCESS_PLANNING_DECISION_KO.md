# 접근 계획과 접촉 학습을 분리하는 1회 결정 실험

## 근거 및 소스 범위

2026-09-28 검토. 읽을 수 있는 기준은 `d8db76d2d3a7e62081cf15c1f26a32c538676eed`이다.
사용자가 전달한 서버 요약은 새 240제안 중 IK+로봇검사 239 통과, 환경검사 239 차단,
물리 실행 0회, 공통 prefix에서 92경로 차단이라고 보고한다. 이는 **사용자 전달 보고**이며,
이번 세션의 GitHub에서 `codex/server-multifamily-20260928` branch 및 파일 fetch가
404였다. 서버 보고서 원문/원자료/MP4를 독립 검증한 것으로 쓰지 않는다.
이번 변경은 조회 가능한 기준에서 **새 파일만 추가**한다. 서버 수정분을 대체하지 않는다.

조회한 `dataset_motion.plan`은 준비점 IK 하나를 현재 관절값에서 풀고 관절 직선으로
이동한다. 이후 Cartesian 구간도 이전 IK 해를 따라간다. FCL 자기충돌은 생성 후 검사하고,
환경 검사는 별도 사후 gate다. `motion_family_search`의 네 경로군은 같은 파라미터에서
같은 preapproach/approach를 쓴다. 따라서 이후 곡선 다양화만으로 막힌 prefix가 고쳐지지 않는다.

## 냉정한 결론

SIM 자동 탐색 -> 초기 RGB-D 행동 선택이라는 연구 방향을 폐기할 근거는 없다.
그러나 고정 경유점 제안 -> 충돌이면 탈락 -> 다른 템플릿 추가를 계속하는 방식은 중단한다.
이번의 0회 물리는 접촉 동작이나 RL의 우열을 시험하지 못했다.
공통 prefix는 잘못이 아니라 **아직 유효성이 확보되지 않은 공통 prefix**가 문제다.
유효한 접근을 찾으면 동일한 조건의 여러 local family에서 재사용할 수 있다.

## 변경: 범용 알고리즘은 직접 재구현하지 않는다

`access_route.py`: upstream OMPL 2.0.1 RRTConnect를 이용한다.
- 기존 목표의 마지막 approach 자세를 staging pose로 유지한다.
- 현재 팔 모양 하나만 고집하지 않고 제한된 IK seed 8개를 검사한다.
- 실제 FK, 관절 범위, 기존 FCL, 기존 MuJoCoScene 환경 형상으로 끝 자세를 거른다.
- 직선 연결부터 검사한다. 실패하면 OMPL이 관절 공간에서 우회 경로를 찾는다.
- 근사 해는 받지 않고 정확한 시작/끝 및 구간 중간을 재검사한다.
- 기존 prefix 경유점은 필수가 아닌 제안으로 취급한다. staging/local 목표는 변경하지 않는다.
- 선택한 관절 해에서 **local suffix를 다시 계획**한다. 다른 IK branch의 옛 suffix를 붙이지 않는다.
- 시간화한 최종 trace 전체를 같은 phase별 접촉 정책으로 재검사한다.
- source, 물성, 모델, 기존 planner 파일을 변경하지 않는다.

`plan_access_candidate.py`: 독립 실행 도구. 모드는 `legacy`, `multiseed-direct`, `ompl`이다.
`--access-only`는 접근+1초 hold만 생성한다. 이것을 삽입/걸림 성공으로 세지 않는다.
이 도구는 **계획만 생성**한다. 물리는 기존 `target_fruit_contact_trial.py`를 호출한다.
실제 추종/식물 변형은 계획 검사만으로 보장되지 않으므로 정상 시작 전체 물리 재생이 필요하다.

`access_sampling.py`: 기존 후보 생성기에 전달할 저기울기 기준 3개(-45/0/+45도 heading),
그리고 선택한 부모의 roll/pitch/elevation/entry twist를 **한 번에 한 항목만** +/-15도 바꾸는
표본을 제공한다. 기존 Sobol 기본값을 바꾸지 않는다. 측면 로봇-facing 서버 수정도 덮어쓰지 않는다.

## 실행

먼저 서버 작업 브랜치에서 이 커밋의 새 파일만 cherry-pick한다. 기존 파일이 이미 있다면
충돌 내용을 검토하고 강제 덮어쓰지 않는다. 현재 브랜치와 dirty 상태를 보존한다.
계획 Python에 기존 Torch/FCL과 MuJoCo 3.13.0, `ompl==2.0.1`이 함께 필요하다.
환경 버전을 몰래 업그레이드하거나 시스템 Python을 변경하지 않는다.

```bash
# RUN: 모델/계획 pickle/후보 목록을 포함하는 서버의 기존 immutable run root.
# BASE: 기존 정확한 pedicel/rear-wire base_policy.json (확장된 trial_policy 파일이 아님).
# 각 OUT은 새 폴더. 외부 timeout도 적용한다.
timeout 240s "$PLAN_PY" mujoco-benchmark/scripts/plan_access_candidate.py "$RUN" \
  --candidate "$CID" --base-policy "$BASE" --output "$OUT" \
  --mode ompl --access-only --ik-seeds 8 --max-branches 3 \
  --seconds 30 --solve-seconds 5 --seed 20260928
```

`--mode legacy` 및 `--mode multiseed-direct`로 같은 후보를 비교한다.
`--access-only`를 빼면 선택한 staging branch에서 원래 local suffix를 재계획한다.
출력 `access_report.json`에는 각 IK 해, 충돌 유형, 찾은 관절 우회, local 실패, 전체 검사를 남긴다.
`result.json`의 plan 통과는 물리 완료가 아니다. 전체 계획이 통과한 output run만 기존 도구로 실행한다.
출력 trace는 새 access backend의 결과다. 일반 `plan_candidates.py`를 다시 실행하면
옛 prefix로 재생성되므로 재계획하지 말고 아래 기존 물리 실행기로 읽는다.

```bash
"$NATIVE_PY" mujoco-benchmark/scripts/target_fruit_contact_trial.py "$OUT" \
  --candidate "$CID" --base-policy "$BASE" --output "$TRIAL_OUT" \
  --search-contact-policy --execute \
  --max-target-force-n 5 --max-target-displacement-m .020
```

5N/20mm는 이전 비교의 실험 중단값이다. 안전·손상 한계가 아니며 이번에 상향하지 않는다.
목표 과실/자기 송이 Rachis 허용은 기존 search policy 그대로다. 다른 꼭지/과실/송이/주줄기/
거터와 마운트/팔은 추가 허용하지 않는다. 현재 colliders에 없는 시각 물체는 여전히 검사되지 않는다.
정지 조건과 최종 `hook_success=null`, `training_eligible=false`, `impossible=null`도 유지한다.

## 서버 시험: 1라운드 상한과 판단 기준

기존 4개 유효 장면 중 6개 대표 후보를 **결과를 보고 바꾸지 말고 먼저 고정**한다.
공통 준비 구간 차단 2, 더 늦은 접근 차단 2, 측면 자기충돌 1, 가장 진행한 경로 1을 권한다.
이 분류가 실제 기록에서 확인되지 않으면 그 사실을 적고 선택 근거를 남긴다.

1. 원래/다중 IK 직선/다중 IK+OMPL의 access-only를 같은 6개에 비교한다(총 18개).
2. 접근이 나오는 타깃 최대 3개에서만 원래 local suffix를 재계획한다(최대 6개 추가).
3. 동일 scene/start/staging/branch/policy가 확인된 유효 access만 재사용할 수 있다.
4. 앞 단계에서 staging 자체가 안 맞는 사례 최대 2개에만 저기울기 기준을 추가한다
   (3개씩 최대 6개). 동시에 모든 각도/자세를 바꾸는 Sobol 확대는 하지 않는다.
5. 계획 총 30개, 정상 시작 실제 물리 최대 8회, 전체 캠페인 최대 60분(먼저 도달하면 종료).
   process hard timeout은 새 자식 프로세스에만 적용하고 사용자 다른 작업은 종료하지 않는다.

성공 기준은 **이번 접근 계획 개선의 진행/중단 기준**이며 일반화 통계가 아니다.
- 기존 막힘 중 최소 3개에서 access-only의 계획+실제 도달을 확인하면 local 비교를 진행한다.
- 접근 0~1개 또는 대부분 timeout이면 장면/표본을 늘리지 않는다. oracle 처리량, 시작 상태,
  staging의 물리적 위치와 IK 해 구성을 보고 아래 대안으로 전환한다.
- 접근은 되는데 suffix가 막히면 이번 문제가 일부 해결된 것이다. local 기하 통로 문제인지,
  접촉 중 적응 문제인지 구분해서 다음 방법을 정한다. 전체 수확 성공으로 선언하지 않는다.
- 우연히 한 경로가 완주하는 것보다 서로 다른 타깃의 실제 도달 수와 독립 경로 수를 본다.

## RL은 언제 쓰는가

RL 자체를 배제하지 않는다. 그러나 이번 자료는 새로운 접촉 구간을 **실행하지도 못한** 결과다.
이 단계에서 자유 7관절 RL로 바꾸면 우회/접촉/성공 판정/제어기/전이 문제를 한꺼번에 학습시킨다.
먼저 기하로 처리할 free-space 접근을 계획하고, 접촉 적응이 병목임이 확인되면 그 구간만
계획 기반 동작 + 제한된 6D residual RL 또는 짧은 궤적 최적화로 비교한다.
GT teacher는 가능하나 실물 actor가 매 순간 숨겨진 꼭지 GT를 받는 전제를 넣지 않는다.
초기 RGB-D만 사용하는 학생으로 옮기지 못하면 teacher 성능을 원래 목표 달성이라고 하지 않는다.

공통 평가를 고정한 작은 local 조건 3개에서 직접 궤적 탐색과 RL을 **같은 simulation-step 예산**으로
비교하고, reward 수정은 무제한 허용하지 않는다。 예산/분할/판정/중단 규칙을 실행 전에 저장한다.
가까이 도달한 기하 증거를 실패로 버리지 않되 물리 모델 불확실성을 성공 증거로 바꾸지도 않는다.

## 실패 시 선택할 다른 방향

- 상태 검사가 너무 느림: OMPL callback 비용을 측정하고 기존 FCL 환경 검사나 native/C++ 거리로
  옮긴다. 경로 샘플과 충돌 범위를 몰래 줄이지 않는다. GPU 이식은 측정 후 결정한다.
- 유효 staging을 못 찾음: 자세 기준과 robot base/lift 배치를 제한적으로 변경하는 문제다.
  바꾼 조건을 분리 기록하고 수확 불가능으로 성급히 라벨링하지 않는다.
- 접근은 되고 고리 삽입 통로가 계속 막힘: contact-aware local trajectory optimization/짧은 RL을
  동등한 조건에서 비교한다. 필요하면 고리/마운트 간섭의 기구적 한계를 별도 검사한다.
- 시뮬 접촉 유지가 결론을 좌우하지만 모델 신뢰성이 부족함: 소량의 정적 치수/힘-변위 측정 또는
  수동 지그 검증으로 불확실성을 좁힌다. 수천 개의 실물 성공 모션 라벨 수집을 요구하지 않는다.

## 연구 출처 (일반 방법 근거, 토마토에서의 성공 증거 아님)

- OMPL RRTConnect 및 validity: https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html
  https://ompl.kavrakilab.org/stateValidation.html
- MoPA-RL: 기하 계획과 local 제어를 결합하는 연구 https://arxiv.org/abs/2010.11940
- IndustReal: RL에 simulation-aware 업데이트/보상/커리큘럼이 함께 필요했던 접촉 조립 연구
  https://arxiv.org/abs/2305.17110
- SPARR: 실물 residual 학습을 실제 사용한다. 실물 trial을 거의 못 모으는 현재 조건의
  바로 적용 가능한 zero-real-data 대안으로 소개하지 않는다.
  https://research.nvidia.com/labs/srl/projects/sparr/

## 현재 검증 범위

로컬 순수 route/예산/분기/입력 보존 시험을 실행했다. 로컬 네트워크에서 패키지 설치가
실패해 OMPL/MuJoCo native는 CI에서 실행하도록 구성했다. CI 결과는 실행 완료 후 별도 기록한다.
작은 native 로봇의 우회 시험과 기존 planner 합성 FK 회귀는 실제 온실 성공이 아니다.
서버 최신 branch가 보이지 않는 문제 및 대형 MJB/계획 pickle 부재 때문에 전체 서버 실행은 미실시다.
