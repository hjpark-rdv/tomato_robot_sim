# 주변 식물 충돌 기준 이력과 Newton 속도 확인

2026-10-17. 브랜치 `codex/s300-newton-independent-tests-20261017`, 출발 `c80f93a`.
이번 변경은 선택형 시험 입력 생성기이며 기존 기본 실행의 충돌 정책은 변경하지 않았다.

## 기존 기준 확인

- 초기 기본형 A는 타깃 한 그루가 탄성이고 주변 식물은 고정 충돌체였다.
- 2026-10-11에는 A / 주변 한 그루 추가 탄성 B / 두 그루 추가 탄성 C / 타깃 송이·자기 주줄기만 식물 충돌 D를 비교했다.
- 이후 사용자 지시 **“타깃 송이·주줄기만 충돌 이것으로 선택해서, 로봇이 레일따라 이동하면서 토마토 수확 작업 해봐”**에 따라 D로 진행했다.
- 커밋 `ca3a716c6f37778580dfb81a6cfe7f8ea9aae48dadd3`의 HANDOFF에 사용자 선택과 창 전환 후 정책 인계가 기록되어 있다.
- Newton 도입 커밋 `f1941f5`와 `S300_NEWTON_RESULTS_KO.md`는 D_target_only를 이어받았다고 명시한다. Newton 성능 튜닝 중 임의로 새로 주변 충돌을 제거한 것은 아니다.
- 직전 답변에서 과거 기준이 없었던 것처럼 새로운 설계만 제안한 설명은 잘못이었다. 전체 농장 **표시**와 전체 농장 **충돌 계산**은 구분해야 한다.

옛 Isaac PhysX/CPU16/60Hz 기록은 A 0.890RTF, D 1.082RTF였다. B는0.670RTF에서 중단, C는0.670RTF 완주했다. D에서 비목표 식물 충돌체21,187개를 비활성화했다. 이 과거 수치를 현재 Newton GPU 결과와 직접 나눠 엔진 개선율로 해석하지 않는다.

## 이번 동일 조건 비교

현재133DOF + 기존 fruit proxy, 접촉220Hz/이동60Hz, 동일 초기상태·명령·벨트·분리조건을 사용했다.
기존 belt strokes2mm / 그 외1mm / 기존 낙과 예외를 유지했다.
기존 C_plus_two의 Plant_09와 Plant_28을 각각 고정 충돌체로 추가했다.
각143개(캡슐79+convex mesh64), 합계286개다. 잎의 기존 collision=false는 그대로다.
자유도는 모두133이며 주변 식물에 새 탄성을 추가하지 않았다.

| 조건 | 전체82.807초의 물리 계산시간 평균 | 평균RTF | 가장 느린 단계RTF | 기준 대비 시간 증가 |
|---|---:|---:|---:|---:|
| 현재 타깃만 식물 충돌 |21.383초|3.873|1.135|—|
| 주변1그루 고정 충돌 추가 |21.723초|3.812|1.138|1.59%|
| 주변2그루 고정 충돌 추가 |21.856초|3.789|1.113|2.22%|

각 조건2회, 화면 없이 순차 실행했다. 로딩/JIT/저장은 표의 물리 loop 시간과 별도다.
각9,127step 완주, 중단 없음, 연결 해제6개, finite/qacc/capacity 검사 모두 통과했다.
2회 반복 및 조건 사이에 저장된 전체 qpos/qvel/물체 pose/명령/phase/연결 상태가 정확히 같았다.
6개 연결 해제는 송이 전체 수확·포집 성공이 아니다.

별도 접촉 기록 실행에서286개 추가 collider가 관여한 접촉은0건이었다.
계측 실행도 비계측 실행과 저장 상태가 정확히 같았다. 계측 실행은 위 성능 평균에서 제외했다.
주줄기/중심가지/열매의 독립적인 CPU 기하 접촉 대조는 교차 검출/비교차 무접촉을 확인했다.
이 대조는 실제 수확이나 GPU 접촉 하중 성능을 대신하지 않는다.

## 적용 범위와 다음 판단

- **가까운 두 그루를 고정 충돌체로 포함하는 비용은 이번 경로에서 작았다.** 지금 성능만을 이유로 가까운 모든 주변 식물 충돌을 끌 필요가 있다는 증거는 없다.
- 이것은 옛 A의21,187개 전체 원본 triangle collider를 복원한 결과가 아니다. 기존 탄성 변환기에 이미 있는 capsule/convex 형상을 고정해서 재사용했다. PhysX의 triangle/convex cooking과 동일한 모델이라고 주장하지 않는다.
- 실제로 주변 식물에 부딪히는 경로, 주변 탄성, 다른 장면 및 GUI 성능은 이번에 측정하지 않았다. 두 번의 반복만으로2.2%를 일반적인 고정 비용이라고 해석하지 않는다.
- 다음으로 가까운 충돌체를 포함한 채 다른 송이 몇 개를 실행하고, 실제 접촉/경로 차단이 생기는 구간의 성능을 확인하는 것이 적절하다. 자동으로 기본값을 변경하거나 다른 물체 접촉을 허용하지 않았다.

## 구현·시험·자료

- `harvest_s300/newton_neighbor_obstacles.py`: 기존 변환 USD에서 활성 capsule/convex collider만 읽어 기존 prepared-input 포맷에 추가. 기존 실행기를 그대로 사용한다. triangle collider를 임의로 convex 취급하면 거부한다.
- 원래 body/joint/dof/equality/actuator/geom의 물리 필드를 이름으로 대조했다. XML에 없는 기존 MJB의 dof_length/runtime damping/prepared timestep을 보존했다. 명령/초기 상태 파일은 동일하다.
- 원본 입력과 결과, 사용자 gripper 변경을 보존했다. 데이터 폴더가 NAS로 이동하면서 과거 입력은 `/root/docker_share/mujoko_debugging_data_bak`에 남아 있어, 새 파생 입력의 경로만 그 위치로 연결했다.
- 준비 과정의 필수 certificate/proxy 파일 누락은 물리 시작 전 검사에서 발견하고 수정했다. 처음 실패한 출력과 `prephysics_input_repair.json`을 보존했다. 이 실패를 물리 실패나 수확 실패로 집계하지 않았다.
- 관련 회귀시험3개 통과: 원본 변환/잎 제외, triangle→convex 오해 거부, 기존 물성 변경 검출.
- 작은 결과: [validation/s300_newton_neighbor_collision_check.json](validation/s300_newton_neighbor_collision_check.json).
- 원자료: `/root/docker_share/mujoko_debugging_data/20261017_234049_newton_neighbor_collision_check`.
- `comparison.json`, `input_comparison.json`, `state_comparisons.json`, `neighbor_contact_summary.json`, `collider_controls.json`과 모든 실행 로그/실제 상태를 저장했다.
- 새 MP4는 생성하지 않았다. 이번 작업은 같은 동작의 화면 없는 속도/충돌 범위 비교다.

`training_eligible=false`, `harvest_success=null` 유지.

## 2026-10-18 후속: 두 위치와 주변4그루

사용자 승인 후 기존 Plant29/Truss00 및 다른 위치 Plant41/Truss00를 새로 실행했다.
별도 실행기나 새 수확 알고리즘을 만들지 않고 기존 USD 변환기, coarse4/fruit proxy 준비기,
Newton GPU 실행기와 실제 상태 렌더러를 사용했다. 위 10/17 절의 ‘다른 장면 미측정’ 이후의 결과다.

Plant29는 기존 준비 입력을 사용했다. Plant41은 기존 yaw−45°/얕은15mm 진입 계획을 그대로
사용해 현재 GPU 모델로 변환했다. 계획은53.34초 전체 명령이며, 옛 PhysX의36.8초 중단 상태를
그대로 재생한 것이 아니다. 각각 저장된 시작 상태에서 독립적으로 새 물리를 실행했다.
기존 Plant43/31의 계획 실패는 이번 새 physics 실패로 재집계하지 않았다.

### 범위와 성능

둘 모두133DOF, 접촉220/이동60Hz, 기존 벨트 양쪽 안쪽 구동/닫기·열기2회/분리 조건이다.
벨트↔목표 열매·꼭지의 지정 stroke2mm, 다른 감시 접촉1mm 및 기존 낙과 예외를 유지했다.
주변 식물은 기존 capsule/convex143개씩을 움직이지 않는 장애물로 추가했다. 잎은 비충돌이다.
전체 농장 외형은 표시하지만 전 농장의 모든 식물 충돌을 복원한 시험은 아니다.

| 대상/조건 | 실행 횟수 | sim초 | 물리 loop초 | RTF | 최저 단계RTF | 분리 개수 |
|---|---:|---:|---:|---:|---:|---:|
| Plant29, 주변09/28 |접촉계측1회|82.807|21.876|3.785|1.115|6|
| Plant41, 주변 없음 |비계측2회 평균|53.343|19.036|2.802|0.986|4|
| Plant41, 주변64/63 |비계측2회 평균|53.343|19.292|2.765|0.978|4|
| Plant41, 주변64/63/40/39 |비계측2회 평균|53.343|19.386|2.752|0.970|4|

최저 단계는belt_finish이며 반복 중 낮은 값을 표시했다. Plant41 주변2/4그루는 같은 타깃의
주변 없음 대비 loop시간+1.34/+1.84%다. Plant29 행은 계측 실행이므로 Plant41의 비계측 평균과
엔진 성능 비교를 하지 않는다. 앞선 Plant29의 비계측 주변2그루2회 평균은3.789RTF였다.
Plant41 주변2/4그루에 별도 전 physics-step 접촉계측을 각각1회 추가해, 이번 총 새 물리 실행은9회다.
계측 실행은 성능 평균에서 제외했다. 모든 실행에서 finite/qacc/capacity 및 선택 접촉정책PASS,
중도 중단0, 원래 명령 끝까지 완료했다.

초기 setup은Plant41 첫 실행20.30~23.56초, 같은 프로세스 반복8.60~16.90초였다.
NAS 읽기·입력 검증·준비가 포함되며 순수 JIT 시간으로 해석하지 않는다. 표의 loop에는 초기화,
저장, 영상 렌더를 포함하지 않는다. 주변 충돌 유무보다 초기 로딩/후처리가 사용자의 대기시간에
크게 느껴질 수 있다. 최저 단계가0.97배속이므로 모든 장면·모든 구간 실시간 달성을 주장하지 않는다.

### 접촉·재현 확인

- Plant29의 새 상태는 앞선 주변2그루 실행과 전 구간 정확 일치했다.
- Plant41의 주변0/2/4그루, 각 반복, 별도 계측 실행 사이에서 qpos/qvel/pose/연결/명령/phase/
  실제 timestamp가 전 구간 정확 일치했다.7개 전체 상태 비교가 모두PASS다.
- Plant29 9,127step, Plant41 각7,359step의 실제 접촉 버퍼를 검사했다.
  추가 주변 collider가 관여한 contact slot/활성 제약/힘은 전부0이다.
  따라서 충돌 검출 후보가 늘어나는 비용을 측정했지만 주변을 강하게 미는 상황은 미검증이다.
- Plant41의 주변2/4그루 명목 경로7,359샘플도 초기 식물 자세 고정 상태에서 교차0이었다.
  이는 저장 명령의 이산 기하 검사이지 연속 swept-volume 보장이나 실제 변형 검사 대체가 아니다.
- Plant41 감시 최대 침투0.982681mm, 개별 접촉 힘 벡터 크기의 전체 최댓값6.266621N이다. 이 힘은 타깃/구조물
  등을 포함한 기존 전체 지표이며 주변 식물 힘은0이다. Plant29의1.304378mm는 허용된 벨트↔꼭지
  stroke 범위이고 다른 감시 접촉 최대0.799010mm였다. 기존 낙과 예외의 침투는 별도 원자료에 남긴다.
- 접촉 기하/힘은 적분 전 solve값, 저장 witness timestamp는 해당 적분 후 clock이다.
  정확한 계산 시점은 각 `contact_witness_schema.json`을 따른다.
- Plant41 portable export가 이미 제외했던 과거 낙과12개는 새 타깃 과실 누락과 구분했다.
  NAS 이전 경로와 이전 단계 history까지 추적해12개 모두 checkpoint JSON/NPZ SHA 및 원래
  분리 이벤트를 검증했다(`historical_omission_lineage.json`). 제외된 과거 낙과를 이번에 물리로
  복원한 것은 아니며 전 농장의 연속 물리 상태라는 주장도 하지 않는다.

### 코드·입력·재실행

branch `codex/s300-newton-independent-tests-20261017`, 기준HEAD
`c80f93ad1f6755934238397f6e950fed64553265` + 미커밋 변경. 기존 gripper 사용자 변경은 보존했다.
원자료 `20261018_002607_newton_neighbor_multitarget`의 `code_provenance.json`과 `code_snapshot/`에
실제 사용 소스·SHA를 보존했다. 모델/명령/초기 상태 hash는 각inputs.json과summary.json에 있다.

- `newton_neighbor_obstacles.py`: 새로 만든 주변 mesh도 재구축 입력 해시에 포함.
- `motion_replay_clip.py`/`render_motion.py`: 옛 display USD의 ‘타깃만 충돌’ 표시보다 실제
  hash검증한 Newton 입력의 주변 충돌 범위를 우선. Newton 중단 이유도 자막에 표시.
- 관련19개 테스트PASS(변환/형상 보존14, 재생/자막5). 준비 단계 오류2개는 원인 수정 후 완료했고
  `prephysics_errors.json`에 기록했다. 이때 physics는0step이며 수확 실패로 집계하지 않는다.
- 기존 기본 실행의 충돌 범위를 자동 변경하지 않았다. 주변4그루 입력을 명시하면 아래로 실행한다.

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh gpu-realtime \
  /root/docker_share/mujoko_debugging_data/20261018_002607_newton_neighbor_multitarget/plant41_inputs_neighbors4 \
  --gui
```

로딩 후 START(1x)를 누르면 새 물리가 시작된다. 이번에는 새 장면의 GUI/headless A/B를 하지
않았으며 MP4는 실제 저장 상태 재생이다. 비계측 A/B 재실행은 원자료의 `reproduce.sh NEW_OUTPUT`.

### 영상과 결론

1배속·작은 자막·전체 농장 외형을 유지한 저장 실제 상태 영상이다. Plant29는35초부터의
접근/수확/후퇴 구간, Plant41은 시작부터 종료까지다. 전체 배경의 잎에 일부 동작이 가릴 수 있다.
Plant29의 최초 인코딩은 옛 충돌 OFF 자막이 남아 아래verified 파일로 정정했다.
원래 프레임과 물리 상태는 동일하고 `caption_correction.json`에 기록했다.

- [Plant29 영상](/root/docker_share/mujoko_debugging_data/20261018_002607_newton_neighbor_multitarget/plant29_neighbors/plant29_neighbors_actual_verified.mp4)
- [Plant41 주변4그루 영상](/root/docker_share/mujoko_debugging_data/20261018_002607_newton_neighbor_multitarget/plant41_neighbor4_witness/plant41_four_neighbors_actual.mp4)
- [대시보드](/root/docker_share/mujoko_debugging_data/20261018_002607_newton_neighbor_multitarget/index.html)
- [작은 결과 JSON](validation/s300_newton_neighbor_multitarget.json)

이번 두 위치에서는 가까운 주변 식물 고정 충돌을 포함하는 비용이 작았다. 따라서 현재 속도만을
이유로 주변을 모두 통과시키는 조건을 고집할 필요는 없다. 단, 주변 식물에 실제 접촉하는 경로,
주변 탄성, 더 많은 환경의 동시 실행은 별도 검증 대상이다.4/6개의 조인트 해제는 송이 전부
수확·포집·실물 손상 검증이 아니며 `training_eligible=false`, `harvest_success=null`을 유지한다.
