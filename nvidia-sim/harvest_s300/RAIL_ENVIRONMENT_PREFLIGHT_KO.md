# 레일 이동의 기존 FCL 환경 사전검사

## 2026-10-11 후속: 현재60Hz 수확 실행 연결

`run_motion.py`의 두 opt-in 환경 검사는 이제 단일60/120Hz 실행에서 실제 물리 주기와
같은 간격으로 검사한다. 외부 검사기에 `--hz`와 `--native-hz`를 함께 전달하며,
독립 검사기의 기본 native 주기는120Hz로 유지한다. 따라서 기존120Hz 실행을 대상으로
60Hz만 샘플링한 자료가 실행 승인으로 바뀌지 않는다. 준비 구간의 저밀도 검사는
계속 진단용이고, 불완전 검사/예산 소진/충돌은 실행을 막는다.

이 변경은 물리 Hz를 자동 변경하지 않는다. 현재 집단 수확 시험은 사용자가 승인한
공통1mm native 관통 중단값을 명시적으로 사용한다. 아래0.5mm/120Hz 기록은 이전
실험 당시 조건이다. 환경 교차 검사, 충돌체, 물성과 접촉 허용 범위는 유지한다.

2026-10-11. 수확이 끝난 실제 상태를 다음 레일 위치로 이동하기 전에, 기존 FK·FCL·120 Hz 경로 샘플러로 **첫 `rail_move`만** 검사하는 opt-in 어댑터를 추가했다. 새 계획기나 물리 실행기를 만들지 않았다. 실제 이동 성공은 아직 확인하지 않았다.

`rail_environment_preflight.py`는 S300 Carrier/차체·리프트·팔·그리퍼의 활성 충돌체를 모두 포함한다. 농장 충돌체는 원본 USD와 저장된 실제 바디 자세를 사용한다. 두꺼운 주줄기, 거터, 다른 송이의 열매·가지와의 교차는 차단한다. 부드러운 잎과 현재 허용 송이에 대한 그리퍼 접촉은 별도 진단으로 남기며, 이를 팔·리프트·차체 허용으로 확대하지 않는다. 원본 filteredPairs와 collision-group 관계를 재사용한다.

검사에는 물리 모델·충돌체·물성·actuator 변경이 없다. 환경 교차 검사는 추가 clearance 없이 원본 형상을 사용하고, 기존 로봇 자체 검사 2 mm와 실제 native 관통 기준 0.5 mm도 그대로 유지한다. SHA 불일치·미지원 형상·불완전 검사·query 예산 소진은 통과로 해석하지 않는다. 레일 이동이 없는 실행에서는 농장 전체를 로드하지 않고 `not_applicable`을 반환한다.

## 실제 상태의 200 mm 이동 제안 검사

정상 시작에서 얻은 9/10 분리 상태의 마지막 유효 checkpoint를 사용했다. 측정 레일 위치 1.6000002289 m에서 1.80000023 m까지, 기존 레일 helper가 만든 동일한 q·그리퍼 간격 44 mm의 이동 제안을 검사했다. 새 native 실행·상태 초기화·수확 credit은 없다.

| 검사 | 완료 | 강체 장애물 교차 | 기존 허용 수집면 진단 |
|---|---|---:|---:|
| 최초 어댑터 | 452/452 sample | 452 | 0 |
| 기존 접촉 정책 전체를 정확히 연결한 어댑터 | 452/452 sample | 0 | 452 |

최초 차단 pair는 이미 분리된 `Truss_02_Fruit_01` ↔ S300 Carrier의 `FlatRearDeck`이었다. 최초 어댑터가 기존 native 정책의 전체 `collection_colliders_from_stage` 대신 CAD subset만 사용하여 이 정확한 수집면을 빠뜨린 연결 오류였다. 원래 차단 결과와 실행 소스를 보존하고, 기존 정책의 전체 447개 정확한 collider를 연결한 새 결과를 저장했다. 추가 접촉 허용이나 물리 변경은 없다.

새 결과의 모든 pair 집계에서 452회 모두 이 동일한 분리 열매/수집판 진단이었으며, 다른 강체 장애물·바닥 지지·팔/차체 pair의 교차는 없었다. 모델·trace·checkpoint SHA와 초기 FK, sample timestamp는 두 검사에서 동일했다. 이동 형상 793개, 환경 형상 30,673개, swept 후보 130개를 포함하여 583 FCL query를 수행했다. 초기 FK의 최대 측정 위치 차이는 0.023 mm이며 레일 변환을 한 번만 적용했다. 함수 실행 시간 17.62초는 기능 검증 시간으로, 격리된 성능 benchmark가 아니다.

## 결과와 제한

새 결과는 `sampled_clear`, `complete=true`이다. 이는 이 저장 상태와 명목 레일 경로에서 강체 장애물 교차를 발견하지 않았다는 뜻이다. 실제 바디 추종·떨어진 열매의 움직임·잎 휘어짐·이동 후 settle은 native에서 별도로 확인해야 한다. 어댑터는 원본 CAD part의 보수적인 convex envelope를 사용하며 native cooked convex decomposition과 같다고 주장하지 않는다.

분리 열매의 수집판 접촉은 원자료에 남는다. `actual_arrival_proven=false`, `collection_verified=false`, `released_fruit_physics_verified=false`, `strict_whole_scene_physics_passed=null`, `training_eligible=false`를 유지한다. 기존 guarded native 이동을 시작하기 위한 명목 검사이며 실제 수확·운반 성공 판정이 아니다.

38개 집중 테스트가 통과했다. 실제 source/측정 상태 정합성, 레일 이중 이동 방지, 중간 구간 장애물, 전체 차체 포함, exact filter, 미완료/오류 검사 거부, 잘못된 owner와 clone 수집판·미분리 열매·팔/도구의 허용 거부, 이전 송이 분리 이벤트와 새 송이 이름의 alias 방지를 포함한다.

결과 root: `/root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/`

- 원래 차단 검사: `rail_environment_probe_200mm_latest_native/report.json`
- 기존 정책 일치 검사: `rail_environment_probe_200mm_existing_collection_scope/report.json`
- 같은 입력임을 검증한 요약: `rail_environment_probe_200mm_existing_collection_scope/paired_policy_validation.json`
- 실행 소스·명령·로그·테스트: 같은 새 결과 폴더
- Git 보존 요약: `validation/rail_environment_preflight_20261011.json`

runner의 opt-in `--verify-rail-environment-preflight`는 로봇 자체 검사 뒤, PhysX 초기화 전에 이 외부 asset-Python 검사를 호출한다. 이때 이미 사용 중인 `--released-fruit-ground-diagnostic`가 있어야 정확한 native 분리/수집면 진단 정책을 함께 사용한다. 기존 기본 실행의 명령 배열과 물리는 변하지 않는다.

## 준비 구간의 환경 검사 추가

이후 Plant22의 Truss01을 처음 접근한 native 시험에서, 로봇 자체 FCL은 통과했으나 `prepare_joint_branch` 9.658333초에 다른 식물의 잎과 접촉하여 중단됐다. 실제 계획은 선택한 `Truss_01_Fruit_01/02`를 향한 yaw 124.9445°·pitch 0°였다. 초기 로그의 yaw 163.9047°는 near-scope 선택 이전의 이전 송이 축이며 실행한 진입 방향이 아니다. 기존 manifest target `Truss_02_Fruit_11`은 이미 분리된 표시 기준이므로 그 target의 `native_target_detached=true`를 이번 수확 성공으로 읽으면 안 된다. 이번 시험의 새 분리는 0개, 가까운 두 송이 합계는 기존 9/20개였다.

접촉 상대 `Plant_34/Stem/mesh_132`는 SourceName `CompoundLeaf_02`이지만 **RigidBody가 없는 정적 triangle mesh**였다. 잎이라는 이름만으로 쉽게 밀리는 동역학을 가정할 수 없다. 상대 그리퍼 `L/part_209`는 `L_C13_outer_finger_cover`의 convexDecomposition이었다. native 원기록은 접촉 생성 시 −0.610191 mm·단일 접촉점 70.09 N이며, 저장된 solve 이후 정확한 CAD/잎 triangle surface는 1.780203 mm 떨어져 있었다. 보수적 CAD convex envelope는 교차했다. native cooked piece 검증이 없어 원기록을 실제 시각 표면 관통이라고 단정하지 않는다. 기존 중단과 유효하지 않은 terminal checkpoint는 그대로 보존했다.

같은 어댑터에 `scope='preparation'`을 추가했다. 첫 `approach` 전의 모든 기존 phase를 검사하고, joint 경로 중간에서 생기는 교차를 놓치지 않도록 기존 `check_plan` 샘플러를 두 번 재사용한다. 첫 번째는 모든 FK sample의 swept bounds, 두 번째는 동일 sample의 FCL이다. 끝점만으로 회전하는 팔의 영역을 추정하지 않는다. phase·timestamp·완료 여부·모든 pair의 집계를 남긴다.

새 preparation scope에서는 정적·kinematic·검증되지 않은 dynamic 잎도 장애물이다. 기존 manifest에 정확히 선언된 활성 dynamic body와 D6 joint를 갖는 compliant 잎만 진단으로 구분한다. 레일 scope의 기존 동작/검증 결과는 변경하지 않았다. 실제 정적 잎 접촉 중단을 레일 진단 허용과 혼동하지 않는다.

`audit_rail_transit(..., scope='preparation', hz=8)`은 제한된 후보를 거르는 coarse 진단이며 `eligible_for_native_execution=false`이다. 실제 실행 전에는 120 Hz 전체 준비 검사 완료가 필요하다. 외부 runner 경계인 `audit_external_rail(..., scope='preparation')`은 coarse 통과를 실행 승인으로 받아들이지 않는다. 이미 승인된 attachment transfer의 입력 checkpoint는 별도 opt-in `allow_attachment_transfer=True`와 기존 exact revision validator를 통해서만 재사용한다. 미지원 revision이나 SHA 불일치는 계속 차단한다.

48개 집중 테스트가 통과했다. 기존 38개 레일 테스트와 함께 정적 foreign 잎 반례, kinematic/fixed/미선언 잎 허용 거부, 끝점은 맑지만 중간 joint arc에서의 장애물 교차, 준비 phase 전체 timestamp, coarse 검사와 native 조건 분리, 기존 attachment-transfer 검증 재사용과 이전 native break identity 보존을 확인했다.

## 제한된 접근 연결 탐색에서의 상태 검사

기존 OMPL 접근 연결기가 사용할 `RailEnvironmentCheck.check_state(q, gap, rail_position_m=..., phase='prefix_search_state')`를 추가했다. 이전 명목 경로의 swept AABB나 sample index를 재사용하지 않고, 매 상태에서 전체 환경 inventory와 현재 FK 형상으로 검사한다. 현재 로봇의 전체 AABB를 이용한 정확한 broadphase 이후 기존 shape별 AABB·FCL·filteredPairs·접촉 대상 정책을 재사용한다. 탐색 상태가 기존 경로의 바깥쪽에 있어도 장애물을 놓치지 않는다.

반환값의 `complete`와 `hard_clear`가 모두 참이어야 연결기의 유효 상태로 사용할 수 있다. query 예산 소진·비수치 상태·검사 오류는 `inconclusive`이며 통과가 아니다. 탐색 query는 기존 sample timestamp와 pair 집계를 소비하지 않고, 같은 유한 FCL 예산을 사용한다. 상태의 `time_s=null`을 유지한다. 이미 checkpoint에 존재하는 정확한 native 분리 이벤트만 기존 수집면 정책의 lifecycle boundary에서 확인하며, 미래 분리나 예상 시간으로 허용을 만들지 않는다.

이 API는 경로·실행 성공 판정이 아니다. 연결 결과와 역방향 준비 복귀가 각각 실제 jaw gap에서 통과해야 하고, 최종 명목 경로는 기존 120 Hz 로봇 자체 검사와 준비 구간 환경 검사를 모두 완료해야 한다. 물리적 추종과 native 관통 감시는 그대로 남는다.

55개 집중 테스트가 통과했다. 추가 7개는 기존 sweep 바깥 장애물, sample ledger 불변, 44/96 mm 별도 검사, 정적 잎과 exact filter, checkpoint 분리/미래 분리 구분, 유한 예산과 비수치 입력, 현재 AABB prefilter와 전체 shape별 검사 결과의 일치를 검증했다. Git 요약은 `validation/environment_state_query_20261011.json`에 저장했다.

이후 정확히 선언된 `Leaf_V9_Additional_CompoundLeaf_07`의 sourceName 형식이 일반 CompoundLeaf 검사와 달라 구조물로 분류되던 연결 오류를 수정했다. 이름 추정 대신 **기존 manifest의 정확한 owner·mesh·활성 dynamic body·D6 joint**를 검증한다. 외부 동일 이름의 잎과 잘못된 mesh 매핑은 허용되지 않는다. 이 반례를 추가한 최신 집중 테스트 56개가 통과했다. 기존 source·물리·경로는 바꾸지 않았다.

## 원래 Truss01 준비 동작의 전체 환경 검사 결과

중단된 terminal을 시작 상태로 사용하지 않고, 원래 유효했던 입력 checkpoint와 실제 사용한 plan을 그대로 재검사했다. 120 Hz의 1,505개 sample을 모두 검사했고, 준비 경로는 차단됐다. 2,680개의 서로 다른 shape pair에서 16,444회 교차가 기록됐다. 정적 잎뿐 아니라 주줄기·다른 송이의 열매/가지·거터·구조물도 포함됐다. 따라서 잎 하나만 예외 처리하면 해결되는 경로로 보지 않는다.

실제 native가 중단된 `L/part_209 ↔ Plant34/Stem/mesh_132`도 명목 준비 경로 7.533~7.608초에 검사기가 차단했다. 계획 timestamp는 초기 native settle 2초를 제외하며, 실제 추종은 명목과 다르다. native 접촉 생성·보수적 source envelope·solve 후 실제 시각 표면의 측정 시점은 계속 구별한다.

이 결과는 환경이 검사를 완료했다는 증거이며 해당 준비 연결이 안전하다는 증거가 아니다. 기존 자체 FCL만으로 놓쳤던 환경 충돌을 실행 전에 차단할 수 있게 됐다. 다음 연결 탐색에서는 같은 시작/목표와 물리를 유지한 채 기존 접근 연결기에서 다른 joint 경유점을 검증한다. 결과는 `truss01_current_preparation_environment120/report.json`, Git 요약은 `validation/truss01_preparation_environment_20261011.json`에 보존했다.

## 실제로 밀릴 수 있는 근처 잎의 별도 물리 모델

새 Truss01 독립 실험에서 열린 그리퍼의 최종 준비 자세와 수평 진입 구간은 Plant20의 `CompoundLeaf_01/02`와 교차했다. 기존 모델에서 이 두 잎은 RigidBody 없이 고정된 triangle collider였다. 사용자 요구인 잎 밀림을 이 모델에서는 구현할 수 없다. 더 먼 250 mm 준비 위치가 맑아도 목표로 이동하며 같은 잎을 다시 지나므로, 준비 끝점만 통과한 것을 전체 진입 가능성으로 해석하지 않았다.

기존 `background_foliage.revise`로 이 정확한 두 canonical group만 별도 독립 모델에 추가했다. Leaf01의 mesh127~130과 Leaf02의 mesh131~134, 총 8개 형상을 그대로 두고 world-connected passive D6 leaf body 두 개를 만들었다. 기존 잎 가설의 stiffness 0.005·damping 0.0008을 재사용했다. 추가 질량은 약 5.61 g이며 원래 body의 질량과 로봇 articulation DOF는 바꾸지 않았다. 원본 농장/로봇/target/거터/주줄기/다른 송이와 filteredPairs는 보존했다.

이는 **잎 동역학과 collision proxy의 명시적인 변경**이다. 기존 static triangle을 동적으로 사용하기 위해 convexDecomposition으로 변경했으며, 점·triangle·world 위치 오차는 0이다. 잎 충돌을 무시하거나 관통 기준을 완화한 것이 아니다. 원래 모델과 이전 native 결과는 보존했다. source 시작에서 새로 시험하는 자산이므로 과거 9/10 분리를 이 모델의 결과로 가져오지 않는다.

새 자산은 `assembly_plant22_fresh_near20_truss01_passive_staging_leaves`이며 직접 materialized USDC를 사용한다. 전체 materialization 검증에서 32,649,127 vertices·32,142 colliders·279 rigid bodies의 형상/자세 오차가 0이었고, 기존 native scope의 정확한 2 body/8 collider CPU 등록을 통과했다. native 초기 안정성·실제 잎 밀림·120 Hz 전체 준비/진입 검사와 수확 동작은 별도 검증 대상이다. 요약은 `validation/staging_leaves_physical_hypothesis_20261011.json`에 저장했다.

## 2026-10-11 실제 이동 후 수확에서 발견한 차이

`--scope full` / 실행기의 `--verify-full-environment-preflight`를 추가했다.
기존 `rail`과 `preparation`의 범위는 유지한다. full은 같은 FCL/FK/형상/필터로
진입·닫기·재개방·후퇴까지 포함한 모든 명목 경유점을 검사한다. 자기 송이에 대한
그리퍼 접촉 범위만 기존대로 유지하며 다른 송이와 팔/기체 충돌은 허용하지 않는다.
4Hz 결과는 후보 선별용이고, 실제 실행 전에는 실제 native 60/120Hz와 같은 간격으로
완료한 검사만 인정한다. 움직일 식물은 초기/인계 상태에 고정한 명목 검사이므로
실제 변형·추종·낙하 물리를 대신하지 않는다.

원본 Plant29 경로는 명목 전체 검사도 통과했지만 실제 기체는 바닥 이음부에서
약798mm만 이동했다. 명령은1622mm였고 실제 오차는824mm였다. 이후 명령 위치를
기준으로 팔을 움직여 다른 Plant23의 꽃받침과 충돌했다. 바퀴–바닥 최대 단일
접촉점 힘은 약20.94kN, 그리퍼–다른 송이는 약2.51kN이었다. 실제 로봇의 손상
한계 측정값이 아니라, 레일로 높이가 고정된 차체가 턱에 걸린 수치적 제약력이다.
단순히 침투가 작다고 이동이 완료되는 것은 아니다.

- 정확한 네 바퀴와 GroundCover/CoverLap 사이에만 명시적 최대1mm 명목 겹침
  진단을 기록할 수 있다. 물리 충돌은 유지하며, 이것은 이동 완료 인증이 아니다.
  다른 robot/tool/거터 형상에 이 예외를 전파하지 않는다. 기본값은0이다.
- `rail_motion.rail_arrival_check`는 레일 뒤 첫 조작 명령 전에 측정된 rail DOF를
  확인한다. 목표와2mm 이내여야 진행하며, 실패하면 `rail_arrival_not_reached`다.
  2mm는 정차 위치 추종 허용값으로,1mm 접촉 유효성 기준과 다른 값이다.
- `rail_ground_alignment.ground_clearance`는 평판뿐 아니라 실제 CoverLap 이음부도
  포함한다. 설치 높이 수정은 기존 `revise --additional-installation-lift --lift-mm 2`로
  **별도 fresh 장면**에 적용했다. robot/tool/레일 지지 프레임만 함께 이동하며
  mesh/collider/질량/마찰/강성/actuator는 유지했다. 측정 route checkpoint를 임의로
  이동시키는 것은 거부한다. 원래 실패 실행은 보존한다.

추가 설치 보정의 실제 이음부 포함 명목 여유는약-0.75→+1.25mm다.
인스턴스화된 House의 CoverLap을 `TraverseInstanceProxies`로 포함해야 한다.
초기 설치 보고서는 이 인스턴스 안의 이음부를 놓쳐0.75→2.75mm로 표시했으므로,
원본은 보존하고 별도 `installation_floor_instance_review.json`으로 정정했다.
이 수정은 읽기 전용 높이 보고서의 범위 수정이며 장면/물리는 바꾸지 않았다.
새 장면도 실제 native 접촉과 측정 도착을 확인한 뒤에만 다음 행동으로 인계한다.
