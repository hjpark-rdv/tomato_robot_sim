# 중앙 정렬을 유지한 벨트 부분 진입: 제어 옵션과 오프라인 검증

기준일: 2026-10-23. 이 문서는 코드·계획 검증과 **첫 후퇴5mm 실제 실행**을 기록한다.
`training_eligible=false`, `harvest_success=null`이며, 계획 통과와 실제 native 열매 분리를 구분한다.

## 해결한 문제

Plant25의 같은 yaw−30°/seed8 경로가 과거 2개 부품의 native cache에서는 환경 검사를 통과했지만,
40개 부품의 실제 SDK 형상을 연결한 뒤 주줄기에 막혔다. 이를 새 yaw 또는 좌표 변환 오류로 단정하지 않고 직접 비교했다.

- old/new source part249 vertex 차이: 최대 `6.25e−17 m`.
- collider→owner transform 차이: 최대 `2.22e−16`.
- 같은 관절 명령의 old/new world FK 차이: 최대 `0.000930 mm`.
- part249는 SDK가 반환한 convex **16개 모두**를 사용한다. source mesh보다 축별 AABB는 최대 `3.091 mm`, source convex hull 바깥으로는 최대 `5.841 mm` 확장돼 있었다.
- 첫 차단 접근 자세에서는 source envelope↔주줄기 간 거리가 `+3.054 mm`였지만 native piece12가 `0.172 mm` 교차했다. 최종 열린 진입 자세에서는 각각 `+1.295 mm`, `1.936 mm` 교차였다.

이번 증거에서는 같은 source 좌표에 이중 변환을 적용한 오류보다,
**기존 source envelope가 실제 cooked shape의 확장을 담지 못한 것**이 old PASS/new block의 원인이다.
SDK cooked shape가 source convex의 부분집합이라고 가정하지 않았다.
여기서 교차 깊이는 FCL 기하 진단이며 실제 접촉력·native physics 결과가 아니다.

## 추가한 작은 옵션

`run_motion.py --belt-entry-retract-mm 5`

기본값은 `0`, 범위는 `0..15 mm`다. 원래 집단 선정과 그리퍼 중앙 정렬을 유지하고,
선정 후 기존 `make_plan`의 TCP translation만 **실제 진입 방향의 반대로** 이동한다.
실제 preparation pose와 goal pose가 함께 이동하며 같은 yaw와 기존 IK·속도·self/environment 검사를 사용한다.
기체·식물·충돌체를 이동하거나 크기·물성·1mm 기준·분리 조건을 바꾸지 않는다.
기존 grasp depth의 `15..100 mm` 하한도 유지했다.

원래 full-containment 집단 선정 증거는 보존한다. 후퇴 후에는 실제 goal 좌표로 과실 중심·반경을 다시 변환해
`belt_entry_retract.entry_window`에 벨트 길이 구간과의 교차, depth 완전 포함, 폭, 높이 중심 조건을 따로 기록한다.
`all_selected_fruits_fully_inside_belt_depth_span`은 **depth 축만** 뜻하며,
부분 진입을 완전 포함·실제 접촉·열매 분리 성공으로 바꾸지 않는다.
선정 과실이 벨트 depth 구간과 전혀 겹치지 않으면 계획을 진행하지 않는다.

0일 때 기존 숫자 계산·명령 배열·계획 metadata를 유지한다.
새 후퇴값은 후속 계획 fingerprint에 포함되며, 독립 idle/cooking 관측에서는 제거된다.
고정 diagnostic plan을 불러오면서 nonzero 옵션을 요청해 실제 제어에 반영되지 않는 조합도 거부한다.

수정 범위는 `harvest_s300/belt_entry_retract.py`, `run_motion.py`,
`run_row_harvest.py`의 startup option 제거 한 줄과 관련 테스트다. 새 실행 파이프라인은 만들지 않았다.

## 한 자세의 후보 비교

동일 nominal−30°의 최종 열린 진입에서 관측된 part249의 모든 SDK piece와 주줄기11만 비교했다.
아래 endpoint 분리는 전체 경로 또는 실제 physics 통과를 뜻하지 않는다.

| TCP 후퇴 | 해당 쌍의 endpoint 결과 |
|---:|---:|
| 0 mm | piece12 교차 깊이 1.936 mm |
| 5 mm | 최소 거리 +3.016743 mm |
| 10 mm | 최소 거리 +7.969560 mm |
| 15 mm | 최소 거리 +12.922417 mm |

횡방향 +10mm 대조군은 열린 endpoint에서 +0.916mm 떨어졌고 self 검사는 통과했지만,
닫기·다시 닫기·열기 구간의 같은 주줄기 충돌 544표본으로 full environment 검사에 막혔다.
따라서 중앙을 유지하는 후퇴5mm 한 후보를 우선 전체 검사했다.

## 후퇴5mm 전체 계획 검증

원본은 `gt_row_maximum_local_heading_v5_campaign/windows/000_plant_25`이며,
정상 B의 실측 archive, nominal yaw−30°/seed8, 해당 source에 묶인 native40 cache를 그대로 사용했다.
원래 집단은 `Truss_00_Fruit_01/02`다. 기존 runner의 실제 `make_plan`을 재사용했다.

| 항목 | 결과 |
|---|---|
| 옵션0 재구성 | 원본 전체 phase/명령과 goal 정확히 동일 |
| 실제 goal 이동 | 반대 진입방향으로 5.000000mm, yaw 동일 |
| goal IK/FK 최대 성분 오차 | 1.55814e−8 |
| full self / 60Hz / 기존 2mm clearance | 3,185표본 PASS |
| full environment / 60Hz / native40 | 3,185표본 PASS, hard intersection 0 |
| 환경 distance queries | 47,158 |
| 전체 검사 완료·실행 자격 | complete=true, eligible=true |
| 원본 source 파일 | 전부 byte/SHA 불변 |
| 신규 native physics 실행 | 0회 |

과실 중심의 tool-X는 약 `±9.757 mm`로 중앙을 유지한다.
01번 depth sphere는 벨트 안에 있지만 02번은 `[134.644,161.356] mm`여서
기존 벨트 `[0,158] mm` 끝에 일부만 들어간다. 이 **부분 진입 상태에서 벨트가 실제로 끌어들이는지**가 다음 물리 검증 대상이다.

코드 회귀시험은 **232개 PASS**, 별도 독립 검토는 새 후퇴 시험 **7개 PASS**다.
기본 명령 불변, 실제5/10mm goal/preparation FK 변화, global yaw 1회 적용,
부분 진입 기록, 범위 밖 입력, startup 제거를 포함한다.

## 증거와 다음 실행의 경계

공유 데이터 기준 경로:
`/root/docker_share/mujoko_debugging_data/20261023_164703_maximum_camera_harvest/`

- `plant25_part249_geometry_comparison.json`: old/source/native 직접 비교 및 동일 FK 질의.
- `plant25_part249_backoff_pair_diagnostic.json`: endpoint0/5/10/15mm 한 쌍 비교.
- `plant25_lateral_endpoint_offline/full_path_report.json`: 횡방향 +10mm 실패 대조군.
- `plant25_retract5_full_offline/`: 단일 후보 plan, 원본 비교, fullself/fullenv, 입력·코드 해시.
- `plant25_retract5_offline.py`: 기존 API를 호출한 해당 단일 질의 재현 스크립트.
- `belt_entry_retract_validation.json/.log`: 232개 회귀시험과 최종 source hash.
- `belt_entry_retract_independent_validation.json/.log`: 별도 7개 시험·metadata 읽기 검토.

v5의 nominal+9개 대안은 모두 실제 수확 physics 시작 전 거절됐고 원본을 보존했다.
다음 Plant20은 준비 단계에서 controller만 안전 종료해 실제 child/native 실행이 없음을 별도 기록했다.
정상 B의 실제 상태는 아직 이 계획 검색으로 변경되지 않았다.

root 실행자의 `plant25_entry_retract5_native/plant25_entry_retract5_harvest`는
같은 모델·같은 B archive·같은 cache로 전체 검사를 다시 거쳐 실제 physics를 실행했다.
첫 진입은 `t=54.75s`까지 동작을 완주했고, `Truss_00_Fruit_01`이
`t=29.483333s / close`에 native break로 분리됐다. 이 1개를 새 분리 증거로 보존한다.

그다음03/04/05 집단의 suffix 계획은 `L/part_214 ↔ GutterSuspensionWire_005/Mesh`,
`prepare_joint_branch / suffix 상대6.85s`에서 차단됐다. 이 후보는 실행되지 않았고
물리는54.75초에서 더 진행하지 않았다. 원본 summary는 `gt_group_planning_inconclusive`로 보존했다.
이는 충돌이 난 물리 시도의 중단이 아니라, 알려진 조기 음성 감사 결과를 기존 증명기가
`complete=false`라는 이유로 미확정으로 처리해 검색 전체가 멈춘 연결 오류였다.

수정은 `unexecuted_planning_failure(..., allow_early_blocked_environment=False)`의
기본 동작을 유지하고 GT suffix에서만 opt-in을 사용한다.
`blocked/passed=false/native_runs=0/physics_unchanged`, 모든 model·plan·manifest·USDC 해시,
`stop_on_first_hard_hit=true/early_terminated=true`, 실제 first hard hit와
`first_recorded_hard_intersection`, 남은 pair/suffix의 `UNKNOWN` 증거가 모두 맞아야
**현재 후보만** 거절하고 다음 heading을 계획한다. 미검사 나머지는 UNKNOWN이며 실행 자격은 false다.
다른 불완전·오류·timeout·native 상태 생성은 계속 중단한다.

실제 원본 suffix 파일을 읽은 재현 시험에서 기본값은 여전히 증명하지 않고,
opt-in만 `nominal_environment_proposal_blocked_early_witness`를 반환했다.
원본 plan·검사·runtime error의 SHA는 변하지 않았다.
추가 회귀시험을 포함한 **236개 PASS**와 재현 증거는
`gt_early_environment_negative_validation.log`, `gt_early_environment_negative_reproduction.json`에 있다.

후속 실행은 과거 B로 되돌아가지 않고, 실제54.75초의 정확한 terminal checkpoint·명령·속도를
별도 검증한 후 연결해야 한다. 이 문서 작성 작업 자체는 새 영상·RGB-D·native 상태를 생성하지 않았다.

## 정확한 planning 경계에서 재개

`run_row_harvest.py --allow-planning-boundary-resume`는 기본값 false인 별도 opt-in이다.
기존 `validated_initial_checkpoint`의 모델·manifest·USDC·완료 cycle·actual self audit·controller 검증을 유지하고,
알려진 `gt_group_planning_inconclusive`의 **최종 실제 상태와 같은 완료 경계**만 제한적으로 승인한다.
검증 본문은 `harvest_s300/planning_boundary_resume.py`에 분리했고 기존 checkpoint loader와
완료-cycle 검증기, native controller resolver, 조기 음성 후보 증명기를 재사용한다.

실제25의 `cycle_checkpoints/000`과 최종 `route_checkpoint`는 모두54.75초다.
141몸체 pose/velocity, 10개 robot DOF의 q/qvel, 315개 plant DOF의 q/qvel/preload와
이름 순서를 포함한 **11개 배열이 float64까지 정확히 동일**하다.
native drive position/velocity targets와 분리 이벤트도 같다.
`states.npz` 마지막54.75초·hold·cycle0의 pose/q/command는 저장 float32 표현으로 정확히 일치하며,
첫 cycle 종료 frame3285 이후 새 physics 표본은 없다. 전체 actual self 감사3,286표본도 PASS다.
원본 summary는 계속 `completed=false`와 원래 stop reason을 유지한다.

추가 suffix는 같은 checkpoint·시간·frame을 사용하는 planning-only 기록이어야 한다.
실행 전 알려진 early-negative full environment 증거의 model/plan/checkpoint 해시와
분리 이력까지 다시 검사하며, 이 음성 결과를 경로 통과로 바꾸지 않는다.
최종 raw snapshot과 다른 이전 checkpoint, 물리·추종·열기·초기화 중단,
시간·배열·분리·controller 변조와 알 수 없는 planning 오류는 승인하지 않는다.
proof에는 `parent_motion_completed=false`, `original_stop_reason`,
`clean_physical_boundary=true`, `resuming_without_rollback=true`를 기록한다.

전용8개와 기존 row 회귀88개, 총 **96개 PASS**다.
실제 파일 대조에서도 기본값은 이전처럼 거절하고 opt-in만54.75초 경계를 승인했다.
원본 summary/states/완료CP/최종CP의 SHA는 전후 동일하다. 이 검증 자체는 native physics를 실행하지 않았다.

- `planning_boundary_resume_validation.json/.log`: 정확한96개 시험 명령·로그·코드 SHA.
- `plant25_retract5_terminal_boundary_identity_review.json`: 실제 최종 상태와 완료 경계 직접 대조.
- `plant25_planning_boundary_resume_actual_reproduction.json`: 기본 거절/opt-in 승인과 원본 SHA 불변.

v6 controller는 이 정확한25 상태를 초기 입력으로 시작했다.
그 이후 새 수확·이동 결과는 별도 실행 결과이며, 이 경계 승인만으로 수확 성공을 추가 집계하지 않는다.

별도 공동 회귀는 중복 없는20모듈 **314/314 PASS**이며,
`gt_retract5_boundary_joint_regression_r2.json/.log`에 명령·로그·source SHA가 있다.
최초 실행의 오래된 terminal fixture 오류 로그도 보존했고, fixture에 실제 이름의 rail DOF·native target을 추가한 시험 수정만 포함한다.

## 같은 assembly의 SDK 재초기화와 정적 관수관 진단

정확한54.75초 상태를 입력한 yaw0/seed5 후보는 self/full environment2623표본을 통과했지만,
threads16의 새 native SDK 초기화에서 SIGSEGV139로 끝났다. view 생성·실제 새 실행 상태 저장 전의
호환성 실패이며, 이 결과를 접근 불가능으로 집계하지 않았다.
읽기 전용 조사에서 이미 분리된 F01은 source USD의 부착 위치와 checkpoint 위치가1.15537m 달랐다.
그러나 F01의 incident joint는 단일 D6이고, 자신의 distal pedicel에 대한 명시적 filteredPair도 유지된다.
이 차이를 crash 원인으로 확정할 근거는 없으며, 사전 pose author 옵션은 구현하지 않았다.

같은 입력에서 threads만1로 바꾼 별도 실행은 SDK 초기화와 정확한 상태 복원을 통과했다.
robot/plant q와 qvel 오차는0, 최대 body 위치 오차는0.000292mm였다.
새 첫 cycle은45.4초에 후퇴까지 완료했고 F03/F05가 실제 native break로 추가 분리됐다.
다음 cycle에서 F04/F07도 분리됐지만64.7667초의 정적 관수관 접촉으로 중단됐다.
기존 F01은 imported prior이며 새 분리4개와 구분한다. 이 실행 전체를 완료로 표시하지 않는다.
`plant25_same_assembly_broken_joint_readonly_review.json`과 실제 실행의
`resume_native_validation.json`, `break_events.json`, `summary.json`이 근거다.

중단 pair는 `Truss_00_Fruit_04/Collider ↔ RowSupply_20mm/Mesh`다.
F04 native break64.0167초 이후이며 원시 최대 침투1.763595mm, 해당 최대점 힘0N도 그대로 보존한다.
실제 source 관수관은 House/Environment의 enabled 정적 Mesh이고, 모든 조상에 RigidBodyAPI나
시간표본이 없다. 기존 structural 역할 목록에 `RowSupply`만 빠져 진단 inventory에서 제외됐던 것이다.
기존 `--released-fruit-static-farm-support-diagnostic` opt-in의 역할 목록에 이 항목만 추가했다.
attached 과실·로봇·식물·동적/animated/unknown 지원물은 그대로 엄격하게 검사하며,
collision response·물성·1mm 기준·분리 조건은 바꾸지 않았다.

회귀134개 PASS이며 실제 원본1mm초과 접촉3개를 읽기 전용으로 재분류했다.
관수관2개가 추가된 inventory689개는 source geometry/transform SHA로 고정한다.
원본 scene/USDC/manifest/summary/states/contacts SHA는 전후 동일하고 원본 실패는 실패로 보존한다.
`plant25_released_rowsupply_static_inventory_review.json`,
`plant25_released_rowsupply_actual_reclassification.json`,
`plant25_released_rowsupply_validation.json`에 근거를 남겼다.
이는 기존 중단의 분류 수정이며, 아직 실행하지 않은 후속 suffix 완료나 추가 수확을 뜻하지 않는다.

## 원래 cycle 시계의 남은 구간만 연결하는 opt-in

`run_motion.py --mode recovery --resume-original-cycle-suffix`를 추가했다.
이 모드는64.7667초의 **마지막 실제 route checkpoint**만 입력으로 검증한다.
45.4초의 이전 완료 상태로 돌아가거나, 과실 위치/속도를 바꾸거나,
미완료 원본 summary를 완료로 고치는 기능이 아니다. 검증기는 원본 stop의 모든
미분류1mm 초과점이 이미 native 분리된 과실과 source-hash로 고정된 정적 RowSupply에
한정되는지, 최종 실제 pose/q/qvel/controller/분리 이력과 원본 시계를 대조한다.

원래 `cycles/002/plan.json` 전체 phase의 시간·waypoint·gap·belt를 보존하고,
source cycle45.4초부터 지난19.3667초를 건너뛰어 남은24.0333초만 같은 실행기로 계산한다.
원래 native tick 계산 `(3886 + local_tick) / 60 - 45.4`를 재현하므로
phase를 다시 보간하거나 새로운2초 settle을 넣지 않는다. 초기 belt controller
`[-0.05,+0.05]m/s`와 실제 drive target/속도를 복원하며 belt ramp0.5초도 고정한다.
원본 rail drive target은 보존하고 충돌 검사의 rail 위치만 현재 실측 station에 맞춘다.

기존 withdraw의96±0.5mm 열린 상태 검사는 그대로 유지한다.
원본 최종 `ready_close → hold`는44mm이므로 최종 영수증도 실제44mm를 검사한다.
이를96mm로 다시 열거나 새로운 수확 진입으로 집계하지 않는다.
완료 시 `terminal_original_cycle_suffix_completed` 영수증을 별도 저장하며
`harvest_attempt=false`, `new_approach=false`, `training_eligible=false`를 유지한다.
일반 harvest와 기존 reverse recovery는 새 opt-in 없이 동일한 명령 배열을 사용한다.

새 모드는 source-bound native tool cache를 사용한60Hz full environment 검사가 필수다.
실제 원본 terminal 상태에서 남은1,443표본을 전체 self/environment 검사했고,
native40 cache·28,072환경 query·hard hit0으로 통과했다. 이는 계획 검사 결과이며
native 후속 완료나 추가 분리의 증거를 대신하지 않는다.
자료는 `plant25_terminal_original_suffix_offline/{proof,plan}.json`과 해당 감사 파일이다.

영향 모듈의 최종 공동 회귀는 **28모듈/463개 PASS**, 실패·skip·중복0이다.
실제 runner AST는 남은 모든 tick의 arm/rail/jaw/belt ramp/phase를 원래 시계와 비교했고,
기존 기본 동작,44mm 정상 끝/96mm 오류 끝, 두 settle override 제외,
정확 초기 belt 복원과 명시적 terminal proof 연결을 포함한다.
시험 전후 runtime/시험 source SHA가 모두 같으며 native/GPU 시험은 이 회귀에서0회다.
`plant25_original_suffix_joint_validation.json/.log`에 기록했다.
root용 입력은 `plant25_original_suffix_case_proposed.json`이며 실제 source proof·CLI·동일 설정 검증을 통과했다.

이후 실제 native 후속 실행도 완료했다. 결과 경로는
`plant25_original_suffix_native/plant25_original_suffix_from64p766_threads1`이다.
정확한64.7667초 상태에서 남은 **24.0333초/1,442 physics step**을 실행했으며,
`completed=true`, `stop_reason=null`, 초기화와 실제1,443표본 전체 self 감사 모두 PASS다.
원래 cycle의 열기·재닫기·release relief·후퇴·복귀·44mm 닫기 곡선을 끝까지 연결했다.
`truss_sequence.termination=terminal_original_cycle_suffix_completed`로 기록했고,
이는 기존 진입의 미완료 suffix 영수증이며 새 접근 수확1회로 집계하지 않는다.
**새 native 분리는0개**다. 기존19/27B/25 누적15개의 분리 이력은 그대로 유지했다.
원본64.7667초 실행의 `completed=false`와 관수관 중단 사유는 수정하지 않았다.

독립 비교에서도 source 당시 일반 harvest AST, 현재 recovery AST, 공통 sampler를
전1,442개 실행 명령과 endpoint1개에 적용해7DOF arm·전체10DOF·rail·gap·belt ramp·phase의
차이가 모두0임을 확인했다. 근거는
`plant25_original_suffix_independent_command_review.json/.log`다.
정확한 후속 완료 상태에서 다음20/24/28/29/31 controller를 시작했지만,
그 새 station의 결과는 별도 실행 자료로 판단하며 아직 추가 성공을 기재하지 않는다.
