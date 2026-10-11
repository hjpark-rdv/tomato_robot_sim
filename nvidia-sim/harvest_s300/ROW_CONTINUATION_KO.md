# S300 수확 상태를 보존한 송이·레일 연속 실행

## 타깃만 충돌 선택 후 얕은 진입 시험 (2026-10-11, 미완주)

결과: `/root/docker_share/mujoko_debugging_data/20261011_205000_target_only_row_harvest/index.html`.
요약: [validation/target_only_row_harvest_20261011.json](validation/target_only_row_harvest_20261011.json).
기준 HEAD `e47a04f`, 브랜치 `codex/s300-harvest-isaac-20261009`. 기존 미커밋 변경 보존.

이전 D의 Plant29 정상 완료 실제 checkpoint를 부모로 사용했다. 새 활성 창에서도
타깃 송이와 자기 주줄기만 식물 충돌을 유지하고, 다른 송이·이전 고정 식물은
충돌하지 않게 정확한 경로 목록을 다시 만든다. 원본 식물과 전체 농장 외형은 유지.
계획기와 native가 같은 저장된 collider 상태를 읽는다. 잎 무시/집·거터·로봇 유지.

Plant41/43/31의 기존 준비 후보를 새 실제 부모 자세에서 재검사했다. 초기에는
준비 자기충돌, 하우스 보강재, 자기 주줄기에 차단됐다. Plant31의 방향 부호가
출발 로봇 위치 기준이라 반대였던 구현 오류는 도착 위치 기준으로 바로잡았다.
그 뒤 IK는 통과했지만 주줄기, 그리고 집단 중앙의 거터 지지 와이어가 남았다.
이들은 다른 식물의 collider가 아니므로 사용자가 선택한 D에서도 계속 장애물이다.

사용자의 후속 지시로 자연 송이축을 유지하면서 실제 진입 방향에 별도 수평 yaw를
줄 수 있게 했다. `--truss-entry-yaw-offset-deg`는 기본0°, 유한±90°이며 Z높이축만
회전한다. 집단 선택 footprint와 실제 IK 경로에 같은 방향을 전달한다. `truss_axis`
는 자연축으로 보존하고 `approach_direction_world`가 실제 진입 방향이다. 재선택과
resume에서 offset을 중복 누적하지 않는다. 기존 mode와0°의 명령 배열 회귀시험 유지.

Plant31/41 각각±15/±30/±45°, 깊이15mm를 기존 IK/FCL state query로 비교했다.
Plant41−45°가 전체60Hz 명목 경로를 통과했다. 최초 native 초기화는 disabled
표시용 Skin의 dormant CollisionAPI 제거와 선언 검증의 호환 오류로 수확 전에 실패.
정확한 비목표 경로에 `collisionEnabled=false`가 남아 있으면 API 제거 후에도
비충돌임을 인정하도록 수정했다. 누락 prim/속성, true 값, 타깃·구조물 선언은 여전히
거부한다. 실제 활성 collider와 물성을 바꾸지 않았다. 경로 동일성 확인 후 새 출력에
재실행했으며, 실제 수확 rollout은 아래1회다.

| 실제 조건 | 결과 |
|---|---|
| Plant41/Truss00, 송이축 대비−45°, world yaw20.838° | 깊이15mm; 고정 평행 수평 진입 |
| 레일 실제 이동 | Y3.2223885→3.2937667m, +71.378mm |
| 도착 확인 오차 | 0.0757mm, 도착 후 조작 |
| 리프트 실제 범위 | 2.252~160.606mm |
| 익은 분리 | Fruit03/05, 총2개; 미숙0 |
| 닫기/열기 | 2회 중 마지막 열기36.8초에 중단; 후퇴 미완료 |
| 중단 | R 하판`part_244`↔자기 주줄기10, callback 겹침1.385mm |
| 계산 | 36.8sim /36.098loop초 =1.019×; 준비·초기화·검사·렌더 별도 |

깊이15mm는 집단 기준점의 설정이다. 실제 벨트 시작 시 선택 열매 중심은 앞끝에서
15.4~39.3mm 안쪽이었다. 모든 열매를 정확히15mm 넣었다는 뜻은 아니다.
분리 직전까지 actual tool 좌표의 안쪽 이동은03=45.5mm,05=68.2mm였다. 남은04는
최대110.5mm 들어왔다가 다시 나왔고, 주줄기10 중심도 약46.6mm 움직였다가 복원됐다.
그리퍼는 정지 grasp 구간에 명목 goal보다 약9.23mm 낮았다. 따라서 정적 source
기하에서의 통과를 변형 후 통과로 해석하면 안 되며, 추종과 탄성 복원 모두 검토해야
한다. `infeed_evidence.json`, `stem_tracking_evidence.json`에 실제 timestamp/pose를
보존했다. 단독 belt-off 대조가 없어 이 변위를 전부 벨트만의 효과로 귀속하지 않는다.

추가−60°/−75°를 검사했다. −60°는 최종 자기충돌, −75°는 최종 자세 통과 후
준비 자기충돌 여유2mm 미달이었다. 기존 회전수 정리, Cartesian 연결, IK seed6,
리프트50/100mm 대안도 보류됐다. 양수 clearance 미달은 관통과 구분한다. 이 추가
후보들은 실제 물리로 실행하지 않았다. 첫 실패의 분리 결과를 재사용한 성공도 없다.
177개 관련 자동시험PASS. 원본 실패·제안·계획·명목/실제 검사·코드 해시는 모두 보존.

전체 영상은 실제 저장 pose의 재생이며 물리 재실행이 아니다. 근접 영상도 전체 농장
geometry를 숨기지 않았다. 권장 영상은 전체/정면2개이며, 정면의 표시만 gamma2·
brightness0.05로 밝게 했다. 가림/어두운 카메라 점검 원본2개도 별도 보존했다.
작은 자막/1배속/6fps, 모든 MP4 전체 디코드와 프레임수 확인.
다음 송이 인계 가능한 완료 checkpoint를 확보하지 못했으므로 **연속 수확 완주가
아니다**. 포집 미검증, `training_eligible=false`, `harvest_success=null`을 유지한다.

## 전체 농장 식물 충돌·탄성 범위 비교 완료 (2026-10-11)

결과: `/root/docker_share/mujoko_debugging_data/20261011_194707_plant_interaction_comparison/index.html`.
작은 보존 요약: [validation/plant_interaction_comparison_20261011.json](validation/plant_interaction_comparison_20261011.json).
이번 작업 브랜치는 `codex/s300-harvest-isaac-20261009`, 기준 HEAD는
`e47a04f231efb8ac87e182ef7f8ea9aae48dadd3`이며 기존 미커밋 변경을 보존했다.

전체 온실 배경을 유지한 Plant29/Truss00의 동일 작성 장면과 동일 레일 이동/팔/벨트
명령을 네 조건에서 각각 새로 실행했다. 기존 full 식물 변환기, 계획기, 전체 환경
검사, `benchmark_motion`/`run_motion`, 실제 상태 렌더러를 재사용했다.
원본 `20261011_180823_continuous_harvest60/assembly_plant29`와 결과는 변경하지 않았다.

| 조건 | 식물 DOF | sim 초 | 물리 loop 초 | RTF | 익은 새 분리 | 결과 |
|---|---:|---:|---:|---:|---:|---|
| A 현재: 타깃 식물 탄성, 주변 고정 충돌 |315|82.800|93.008|0.890|5|완주|
| B 주변 Plant09도 탄성 |630|55.933|83.513|0.670|4|겹침 중단|
| C 주변 Plant09·28도 탄성 |945|82.800|123.600|0.670|4|완주|
| D 타깃 송이·자기 주줄기 외 식물 충돌 제외 |315|82.800|76.517|1.082|6|완주|

네 실행은 독립 비교이며 분리 개수를 합산하지 않는다. 미숙 새 분리는 A/B/C는0,
D는 Fruit09 1개다. 최초 본문의 모두0 표기를 원본 summary에 맞게 정정했다.
60Hz, CPU16, TGS64/4, 접근 전96mm 개방, 양 벨트 안쪽50mm/s, 닫기/열기2회,
공통1mm와 기존 낙과 접촉 정책을 유지했다. 전체 명목 경로·초기·실제 자기충돌
검사는 네 조건 모두 PASS. B는55.933초에 분리 Fruit03이 그리퍼 고정
`part_9` (`F01_rear_rail_backbone`)와 겹쳐 중단했다. raw callback3.187mm와
post-fetch convex SAT penetration upper bound2.157mm를 구분해 저장했다.
뒤쪽 프레임은 기존 하판/지지대 허용 목록이 아니며 기준을 높여 통과시키지 않았다.

B의 종료 시각이 다르므로 공통54.02sim초에서도 비교했다: A60.55wall초/0.892×,
B80.03/0.675×, C80.93/0.667×, D49.36/1.094×. 이 구간은 콘솔 시각0.01초
정밀도다. 완주 loop 기준 C는A보다33% 오래 걸렸고 D는18% 짧았다.
전체 native 프로세스는 각각426.47/376.70/428.49/287.71초이며 환경 검사만
155.29/155.97/147.69/75.70초다. 렌더와 자산 준비는 별도다. 따라서 D도
준비부터 결과 저장까지 실시간이라는 뜻은 아니다. 기존 route-window 생성244초
문제가 해소된 것도 아니다. 이번 파생 자산 복사/변환은1.61/25.68/48.65/3.80초였다.

### 구현과 검증 범위

- `plant_interaction_variants.py`는 새 폴더에만 자산을 만들고 원본 SHA를 재확인한다.
  추가 식물은 기존 `build_plant`/`compliant_foliage`와 잎 충돌 무시 정책을 적용해
  `/World/ElasticNeighbors/Plant_09`, `Plant_28`로 복사한다. 연결 관계, transform,
  물리 속성/토폴로지의 namespace 변환 검사를 저장한다. 각315DOF/116몸체다.
- `neighbor_plant_runtime.py`는 기존 고정 초기 자세 중력 preload를 재사용한다.
  native DOF/armature/초기·최종 q를 확인하고 body pose와 접촉을 기존 저장기에
  포함한다. 추가 식물 접촉은 일반 식물 guard로 분류한다. 추가 articulation이 있는
  checkpoint의 다음 창 인계, 환경 복제/GPU 병렬은 지원하지 않으며 명시적으로 거부한다.
- D는 비목표 식물 collider21,187개를 비활성화한다. 활성 collider23,220→2,033,
  계획기 환경 shape22,427→1,240이다. 같은 주줄기의 다른 송이 및 이전 Plant22
  낙과도 비목표 범위다. 거터·집·바닥·기체·팔·그리퍼와 타깃 송이/주줄기는 유지한다.
  시각 형상·관절·물성은 바꾸지 않았다. 비목표 낙과는 바닥도 통과할 수 있는 근사다.
- D 최초 실행은 기존 낙과 소속 확인기가 명시적으로 끈 collider를 지원하지 못해
  preflight에서 종료했다(native physics0회). `harvest_contact_scope.py`에 정확한
  비목표 소속·collider 목록·disabled 상태 검증을 연결했다. 실제 활성 충돌체의
  접촉 허용을 늘린 수정이 아니다. 새 `D_target_only_connected`에서 실행했으며
  최초 오류와 결과를 덮어쓰지 않았다. 이번 native 물리는 총4회다.
- 관련 자동 시험89개 PASS(`tests_connected.log`). 네 실행에서 동일 명목 phase hash,
  동일 command/belt 공통 prefix,60Hz 기록 간격, 유한 상태를 확인했다.
  실제 영상은 동일 카메라에서32.8초부터 끝까지1배속/6fps로 렌더했다. 전 농장 및
  잎을 숨기지 않았으며 작은 자막을 사용한다. 첫 가림 카메라 시도는 보존하되 보고서
  대표 영상에서 제외했다. 물리 측정과 영상 렌더를 동시에 실행하지 않았다.

### 해석상 제한과 다음 선택

추가 식물은 타깃과 가장 가까운 과실 중심으로 선정했다(Plant09 약243mm,
Plant28 약296mm). 실제 로봇↔추가 식물 접촉은0회다. 비잎 몸체의 최대 이동은
Plant09 0.074mm, Plant28 0.047mm로 작았다. 주변 식물을 강하게 밀 때의 부하나
전체 온실 탄성화의 속도는 아직 알 수 없다. 기존 full 변환기를 재사용했지만 고정
Mesh237개가 동적 Capsule79+convex Mesh64개로 바뀌므로 자유도만 바꾼 비교도 아니다.
주변 열매 부착은 기존 fixed3N/0.08Nm, 타깃은 기존 elastic 가설이며 물성 보정은 안 했다.

A/B/C는 기존 몸체의 실제 초기 pose가 같았다. D는 작성 배치는 같지만 충돌 제외 후
native 초기화에서 최대 위치5.282mm/자세0.04453rad 차이가 발생했다. 이를 숨기는
pose reset은 하지 않았다. `D_initial_state_difference.json` 참고. 또한 A도 동일
source SHA/명목 경로의 이전 실행4개와 이번5개 분리가 다르다. 조건당1회인 이번
수확 개수로 우열이나 재현 성공률을 주장하지 않는다.

현재 A를 전체 환경 충돌 검증 기준으로 유지한다. 빠른 수확 모션 개발에는 D가
유용한 opt-in 후보이며 실제 장애물 회피 검증으로 해석하지 않는다. 기본 충돌
설정을 D로 자동 변경하지 않았다. B/C는 필요한 주변 식물만 활성화하는 방법의
비용 측정으로 사용한다. 이후 우선 과제는 전체 대기시간에서 큰 환경 검사/장면 전환
비용을 줄이는 것이며 이번 결과만으로 주변 식물 전체를 탄성화할 근거는 없다.
`training_eligible=false`, `harvest_success=null`, 포집 미검증을 유지한다.

파생 장면 생성 예시(각 `OUTPUT`은 새 폴더):

```bash
cd /root/farmily_tomato
nvidia-sim/.asset-venv/bin/python nvidia-sim/harvest_s300/plant_interaction_variants.py \
  SOURCE_ASSEMBLY OUTPUT --target-only
# 주변 식물 탄성 비교는 --target-only 대신 정확한 source prim 경로 지정
nvidia-sim/.asset-venv/bin/python nvidia-sim/harvest_s300/plant_interaction_variants.py \
  SOURCE_ASSEMBLY OUTPUT --neighbors /World/Greenhouse/Plants/Plant_09
```

실행은 기존 `benchmark_motion.py`/`run_motion.py`에 파생 assembly와 동일 cases를
전달한다. 이번 정확한 인자와 정책은 결과 폴더의 `*_cases.json`, `*_policy.json`,
`run_comparison.py`, 각 trial 실행 로그에 보존했다.

## 비교 전에 정한 범위 (아래 네 조건의 실측 완료)

사용자 요청으로 정했던 네 비교 조건이다. 위 절에 새 실측 결과를 기록했으며,
현재 실행 기본값이나 기존 결과의 충돌 조건을 변경한 것은 아니다.

| 조건 | 변형·충돌 범위 |
|---|---|
| A. 현재 기준 | 현재 활성 식물은 탄성, 주변 식물은 고정 장애물 |
| B. 주변 1그루 추가 | A에서 로봇 주변 1그루도 탄성으로 계산 |
| C. 주변 2그루 추가 | A에서 로봇 주변 2그루도 탄성으로 계산 |
| D. 비목표 식물 통과 | 타깃 송이와 그 송이가 붙은 주줄기의 충돌만 유지. 그 밖의 줄기·송이는 표시하되 통과 가능 |

D는 같은 주줄기에 달린 다른 송이도 충돌 제외 대상이다. 정확한 plant/truss 소속으로
분리하며 거터·바닥·기체·팔·그리퍼의 기존 충돌은 유지한다. 잎 무시와 남은 충돌의
공통 1mm 기준도 유지한다. 실행 시 제외한 식물은 계획기 장애물 검사에서도 같은
범위로 제외해야 한다. 단순히 접촉 중단 판정만 무시하는 모드와 구분한다.

동일 초기 상태·명령·60Hz 설정에서 준비/환경검사/물리 루프 시간을 분리하고,
변형 대상 수·관절 수·실제 충돌체 수와 분리/관통 결과를 함께 기록한다.
D에서 비목표 식물의 동역학까지 줄일 경우에는 충돌 제외만의 효과와 구분한다.
배경은 모든 조건에서 유지하고, 영상은 전체 농장이 보이게 비교한다.
D의 통과 경로와 수확 결과는 주변 장애물을 피한 결과로 해석하지 않는다.
속도 및 수확 동작을 분리해 살펴보기 위한 후보로 구현·실측했으며, 결과는 위 절을 따른다.

## 최신: 새 접촉 정책의 60Hz 두 송이 연결 (2026-10-11 18시)

결과 `20261011_180823_continuous_harvest60`. 첫 번째는 기존174730 실행의 정상
최종 checkpoint를 그대로 사용했고 새 native 실행은 두 번째 송이1회다. 기존
`completed_final_checkpoint` / `validated_initial_checkpoint` / `switch_window` /
`prepare_harvest_group_policy` / `benchmark_motion` / `run_motion`을 재사용했다.
새 실행 엔진이나 별도 물리 모델은 만들지 않았다.

| 연결 대상 | 실제 레일 이동 | 새 ripe 분리 | 잔여 ripe | sim / loop 초 | RTF |
|---|---:|---:|---:|---:|---:|
| Plant22 / Truss02, 기존 부모 실행 | −20.000mm |5|2|57.000 / 81.012|0.704|
| Plant29 / Truss00, 새 실행 |+1,642.386mm|4|3|82.800 / 92.358|0.897|

연결된 고유 분리는9개다. 두 번째는 Fruit02/03/04/08이며 미숙 새 분리0.
남은 익은 Fruit01/05/07은 총3개지만, 현재 위치에서 기존 집단 선택기를 적용하면
그리퍼 영역에 한 번에 들어오는 익은 집단은 최대2개다. 다음 사이클 깊이50mm,
기존 송이 yaw/높이/시도 이력을 사용한 `remaining_group_evidence.json`을 보존했다.
최소3개를 만족하지 못해 추가 진입하지 않았다. 임계값을 낮추거나 세 개 모두가
한 집단이라고 가정하지 않았다. 이 결과는 절대 수확 불가능 판정도 아니다.

첫 실제 정상57초 상태의 로봇q/qvel/drive 목표, 식물 측정 변형, 분리5개 증거를
검증해 인계했다. 새 Plant29는 현재 실측 시작 자세에서 다시 계획하고 자기충돌 및
실제60Hz 간격의 전체 명목 환경검사를 통과한 뒤 실행했다. 레일 도착시점 측정오차
0.0113mm. 리프트 명목 준비q는18.921mm, 실제 시작168.921mm/동작 최저10.596mm/
최종160.606mm다. 하중/복귀 편차는 기록하고 사용자 요청대로 이를 독립 중단
조건으로 추가하지 않았다. 로봇 실제 자세 audit와 다음 정상 checkpoint 인계 검증도 PASS.

공통1mm, 기존 낙과 접촉 진단, 잎 무시, 접근 전96mm 개방, 양쪽 안쪽50mm/s,
닫기/열기2회 유지. 두 번째 깊이35mm/닫기6초는 기존 검증 경로 제안을 재사용한 값이다.
그 경로의120Hz 실행을60Hz 결과로 재분류하지 않고 새 실행했다. 앞선120Hz 두 송이
분리12개와 이번9개는 다르므로 물리적으로 동등한 속도 개선이라고 부르지 않는다.

두 loop 합139.8sim초/173.370wall초=0.806×다. 새 장면 생성 약244.6초,
두 번째 native 프로세스 전체402.6초(전체 환경검사154.4초, loop92.4초 포함),
영상은 별도다. 최종 audit 중 저장 상태 렌더 준비를 시작했지만 실제 physics loop와
렌더는 겹치지 않았다. 전체 반복 대기시간은 여전히 장면 전환·검사 비용이 크다.

인계·경로·접촉 관련90개 자동 테스트PASS. 작은 자막의 첫 송이 영상은 기존 raw
렌더 프레임만 재인코딩했고, 두 번째 전체 이동/수확 탑뷰는 저장 실제 상태를 렌더했다.
이전 식물의 붙어 있는 몸체는 측정 변형으로 고정, 분리 과실은 동적 유지, 새 식물만
활성 탄성인 기존 window 근사이며 PhysX solver는 경계에서 재시작한다.
전 농장 한 solver 연속 탄성/카메라 대상 판단/포집 검증은 아니다.
`training_eligible=false`, `harvest_success=null`, `collection_verified=false` 유지.

## 최신: 낙과 중단 범위 수정과 리프트 우선순위 (2026-10-11 17시)

분리된 과실↔정확한 그리퍼 하판/지지대 및 자기 송이 과실/가지 접촉은 물리 반응을
유지하며 진단으로 기록하고 계속한다. 새 row 기본 인자에 두 옵션을 포함했다.
기존 저장 cases는 자동 변경하지 않는다. 실제 native 분리 증거 및 식물·송이 소속을
확인하고, 인계된 과실도 기존 checkpoint/SHA/분리 이벤트 검사를 통과해야 한다.
다른 송이·주줄기·거터·팔/벨트 등으로 접촉 허용을 넓히지 않는다.

동일 Plant22 경로60Hz 재시험1회는57sim초/81.012loop초,0.704×로 완주했다.
익은5개 분리, 익은2개 남음. 이전60Hz 중단시점까지 실제 상태가 정확히 같다.
초기·명목 환경·실제 자기충돌PASS. 같은 경로120Hz의6개 분리보다1개 적으며,
이번은 새 정책의 단일 경로 회귀시험이다. 두 번째 식물60Hz 연속 검증은 아직 없다.
결과 `20261011_174730_released_same_truss_60hz`; [정책/한계](COMMON_CONTACT_LIMIT_KO.md).

사용자는 필요한 수확 높이에 맞춰 리프트를 조절하면 되므로 작은 복귀높이 편차를
소수 반복 시험의 선행 차단 사유로 삼지 않도록 했다. 아래 과거의 “리프트 복귀부터
수정” 우선순위를 대체한다. 측정 시작q와 다음 목표 높이를 기존 계획기에 전달하고
관절한계/실제 추종/충돌 검사는 유지한다. 제어기 자체 수정이나 무한 장기 누적 검증은 아니다.

## 최신: 실제 이동과 두 송이 수확 연결 (2026-10-11 15시)

결과 폴더: `/root/docker_share/mujoko_debugging_data/20261011_140011_move_harvest_repeat`.
기존 집단 선택기·계획기·환경 FCL·native 실행기·checkpoint 인계·재생기를 사용했다.
아래 두 실행은 정상 최종 상태를 인계한 한 시퀀스다. 원본 실패 시행과 독립 재시험은
따로 보존했고 실패 뒤 과거 checkpoint로 돌아간 결과를 연속 성공으로 합치지 않았다.

| 실행 | 실제 기체 이동 | 새 ripe 분리 | 남은 ripe | sim / 물리 루프 초 | RTF |
|---|---:|---:|---:|---:|---:|
| Plant22 / Truss02 | −20.000mm | 6 | 1 | 57.000 / 151.138 | 0.377 |
| Plant29 / Truss00 | +1,642.386mm | 6 | 1 | 82.792 / 165.526 | 0.500 |

두 실행 모두 초기 검사, 120Hz 전체 명목 환경검사, 실제 기록 자기충돌 검사를 통과했다.
공통 접촉1mm 유효성 중단 없이 접근 전96mm 개방, 집단 중앙으로 수평 진입,
양쪽 벨트 안쪽0.05m/s, 닫기/완전 열기2회, 후퇴·복귀를 마쳤다.
첫 경로 깊이50mm/닫기8초, 둘째 깊이35mm/닫기6초다. 물리120Hz CPU16 TGS64/4이며
충돌체·마찰·강성·actuator·물성은 바꾸지 않았다. 120Hz는 이번 별도 실행 옵션으로
사용했으며 모든 기존 실행의 기본Hz를 바꾼 것은 아니다.

실제 분리 증거는 첫 송이의 Fruit01/02/03/04/05/08,
둘째 송이의 Fruit02/03/04/05/07/08이다. 둘 다 현재 authored RGB 익음 규칙에 포함되고
미숙 대상 밖 새 분리는0이다. 남은 ripe Fruit07 / Fruit01은 각각1개로 최소집단3개
미만이므로 추가로 추적하지 않았다. 각 시행은1진입 예산이므로 남은1개가 새로
선택되는 두 번째 진입까지 실행한 시험은 아니다. 선택집단 밖 같은 송이 익은 열매의
동반 분리를 포함하지만 다른 시행의 분리 이벤트나 인계된6개를 다시 세지 않았다.

두 번째 실행은 첫 실제 최종57초의 `route_checkpoint`와 완료 실제 자세 audit를 검증해
`switch_window`로 이어받았다. 이전 줄기의 측정 변형 자세·분리된 열매·로봇 q/qvel·
drive 목표를 보존하고 새 Plant29만 active 탄성 식물로 구성했다. solver contact cache는
재시작한다. 전체 온실 식물 모두를 동시에 탄성화한 연속 단일 solver 시험은 아니다.

### 발견한 문제와 수정

원래 Plant29의 명령 이동은1.622m였지만 측정은0.798m였다. 고정 높이 rail joint의
기체 바퀴가 바닥 CoverLap 이음부에 걸렸고, 이후 팔만 명령 도착 위치를 기준으로
움직여 Plant23의 다른 송이 꽃받침과 충돌했다. 이를 명령 이동 완료로 잘못 설명했던
중간 해석은 정정했다. 명목 전체 경로 검사 통과가 실제 이동 완료는 아니다.

1. 별도 fresh 장면에서 robot/tool/레일 지지 프레임의 설치 높이를 함께2mm 올렸다.
   기존 측정 checkpoint를 이동시키는 것은 거부한다. 바닥 이음부 포함 여유는
   −0.75→+1.25mm다. initial report가 House instance 내부를 누락한 수치는 원본을
   보존하고 `installation_floor_instance_review.json`으로 정정했다.
2. 팔 동작 전 실제 rail 위치가 목표2mm 이내인지 확인한다. 2mm는 정차 위치 기준이며
   공통1mm 접촉 기준과 별개다. 이번 실제 도착 오차는0.0868mm / 0.00179mm였다.
3. 열린 상태로 팔을 움직이기 전에 정지 개방 유지0.5초를 넣었다. 실제 개방 피드백
   96mm±0.5mm 검사는 그대로 유지한다.
4. `--verify-full-environment-preflight`로 준비뿐 아니라 진입·닫기·열기·후퇴도 같은
   source 형상/FCL/접촉 정책으로 검사한다. 4Hz screen은 후보 선별용이며 실제 실행은
   native120Hz와 같은 간격의 완료 검사를 요구했다.

원본 실패2회는 `entry_open_not_reached` / `cycle_reopen_not_reached`, 새 분리0이다.
새 설치 장면의60Hz 시험2회는 각각6개/4개 분리 후 낙하한 열매와 그리퍼의 하판
`L/part_122` / 슬라이더 지지대 `L/part_117`의1.039mm / 1.120mm 실제 겹침으로 중단됐다.
60Hz 실패를 성공으로 재분류하지 않았다. 같은 두 번째 경로를120Hz로 실행한 뒤
첫 정상 완료가 나왔고, 이를 다음 송이로 이어갔다. 모든 실제6회는 `index.html`과
`validation_summary.json`에 포함한다. 앞선 IK/입력/초기화 오류는 physics 실패에 넣지 않는다.

### 시간·검증·남은 범위

RTF는 준비·계획·검사·저장·렌더 제외다. 전체 명목 환경검사만 별도로106.77초 /
295.60초가 걸렸다. 첫 실제57초와 다음82.79초의 loop 합은316.66초로 아직 실시간보다
느리다. 60Hz 실패는 일찍 중단돼120Hz의 완료 실행과 단순 속도비로 비교하지 않는다.
수확 종료 대기 lift q는177.236→168.769→160.301mm로, 회당 약8.47mm 낮아지는
복귀 높이 편차가 남았다. 앞선 rail-only 하중 보존 수정과 별개이며 긴 반복 전 해결해야 한다.

전체 offline808개 PASS 뒤 바닥 instance 검사5개 PASS(추가1개 포함).
native SDK 전용 기존2모듈은 offline 묶음에서 제외했다. 실제 물리 완료·중단·상태 인계는
위 별도 실행으로 확인했다. 영상3개는 저장 실제 body pose/q 재생, 1배속, 전체 디코드와
343/321/498 프레임 수 검사 PASS다. 관찰용 배경/잎 숨김은 재생 화면에만 적용된다.

`training_eligible=false`, `harvest_success=null`, `collection_verified=false` 유지.
12개는 native 조인트 분리이며 포집·상품성·실물 손상 검증이 아니다. 익음은 authored
RGB 규칙이며 카메라 detector는 아직 없다. 레일은 prismatic guide로 휠 구름·서스펜션·
슬립을 시험하지 않았다. 다음은 작은 추가 집단 시험 전 lift 복귀 목표 보정, 이후
장착 카메라 시야 안의 관측을 사용한 대상/정차점 선택 연결이다.

[Git 검증 요약](validation/move_harvest_repeat_20261011.json).
원본 로그·계획·상태·수정 patch·코드 hash·영상은 결과 폴더에 보존한다.

## 후속: 최신 집단 수확 동작으로 소량 연결 (2026-10-11)

최근 중앙 정렬·접근 전96mm 개방·닫기/완전 열기2회·양쪽 벨트 안쪽 구동을 사용해
레일 가까운 송이2개를 시험했다. 기존 `switch_window` / 집단 선택기 / `plan_motion` /
환경 FCL / `benchmark_motion` / native 실행·상태 저장을 재사용했다. 별도 실행 엔진은 없다.

| 후보 | 실제 기체 이동 예정 | 준비 거리 | 전체 준비 검사 | 실제 수확 실행 |
|---|---:|---:|---|---|
| Plant35 / Truss02 | +177.922mm |100mm|이웃 송이와 지지 와이어 교차|0|
| Plant37 / Truss00 | +450.956mm |100mm|주줄기 교차|0|
| Plant37 / Truss00 |동일|50mm|주줄기 교차 지속|0|

셋 모두 자기충돌 및 레일 환경 검사는 통과했지만, 팔 준비 구간과 끝 자세에서
장애물 교차가 남았다. 명목 형상 검사 결과를 실제 접촉력/침투 깊이로 해석하지 않는다.
기존 자기충돌만 통과한 station 제안은 환경 도달성 증명이 아니었다.
소수 경로에서의 실패이며 이 송이의 수확이 절대 불가능하다는 뜻도 아니다.

실제 레일 구동은 수확과 분리해 확인했다. 동일 측정 시작 상태에서451mm 이동의
독립 A/B2회 실행이며, 두 실행을90cm 연속 이동이나 두 송이 수확으로 합산하지 않는다.
첫 실행의 리프트 최대 변화8.370mm는 레일-only 단계가 기존 하중 보정 drive 목표를
측정q로 덮어쓰는 코드 오류였다. `run_motion.py`에서 route idle의 lift/arm 목표를
기존 값으로 유지하고 rail DOF만 이동하도록 고쳤다. actuator 물성은 그대로다.
재실행은 실제451mm 이동, 리프트 최대 변화0.000105mm 미만, 초기/실측 자기충돌
검사PASS, 관통 중단0, 새 열매 분리0. 11.45sim초/11.931loop초=0.960×.
장면 준비·로딩·계획·검사·저장·렌더 시간은 이 비율에서 제외한다.

새 active 식물을 만들 때 단순히 leaf-ignore manifest를 복사하면 새 잎 충돌체가
켜지는 문제도 수정했다. 기존 정확한 잎 식별 규칙으로 새 Plant35의232개,
Plant37의316개 잎 충돌체만 비활성화하고 비잎 충돌 상태는 보존한다. 환경 검사는
실제60/120Hz를 명시적으로 연결하며, 낮은 샘플률을 실행 통과로 재분류하지 않는다.
현재 native 공통 중단값은 사용자가 승인한1mm이며 아래0.5mm 기록은 과거 조건이다.
자동 테스트793개 PASS(기존 native SDK 전용2모듈 제외); 새 정지 리프트 명령 보존과
기존 harvest 보간,60Hz 충돌 양성/간격 불일치/저밀도 거부를 포함한다.

이동 전 장면은 이전 실제 최종 checkpoint를 보존한다. 완료된 이전 active 식물의
붙어 있는 가지는 측정 변형 자세로 고정하고, 이미 분리된 열매는 동적으로 이어받는다.
새 active 식물은 탄성 모델이다. PhysX solver cache는 재시작한다. 모든 줄기가 연속
탄성화된 모델 또는 실제 휠 slip/odometry 검증으로 확대 해석하지 않는다.

결과: `/root/docker_share/mujoko_debugging_data/20261011_131243_small_row_group_harvest/`.
`index.html`에 전체 후보와 실제 레일 영상, `validation_summary.json`에 측정값,
`work_final_changes.patch`와 소스 SHA, 실패 원본·실행 로그를 보존했다.
[Git 검증 요약](validation/small_row_group_harvest_20261011.json).
영상은 저장된 실제 poses/q이며, 수정 후 영상의 배경 식물/잎 숨김은 관찰용 재생 설정이다.

다음 최소 작업은 열린 그리퍼 외형과 주줄기를 고려해 집단의 준비·진입 위치를
고르는 것이다. 이동+높이 변경+수확을 소수 송이에서 먼저 연결한 뒤 카메라를 실제
장착 위치에 놓고 시야 안의 관측으로 대상을 고른다. 이번은 SIM-GT 위치 시험이며
카메라 선택·전체 레일 최적화·학습은 하지 않았다. `training_eligible=false`,
`harvest_success=null`, 포집 미검증을 유지한다.


이 문서는 기존 `run_motion.py`, `benchmark_motion.py`, `run_row_harvest.py`를 연결하는 상태 인계 규칙을 설명한다. 별도 수확 실행기나 새 물리 모델을 추가하지 않는다. SIM-GT 실험이며 학습 데이터에 넣지 않는다.

## 실제 마지막 상태에서 이어가기

`cycle_checkpoints/NNN`에는 실제 바디 위치·회전·속도, 로봇과 식물의 관절 위치·속도, preload, native 분리 이벤트, 로봇 drive의 위치·속도 목표를 저장한다. 측정된 관절 위치와 drive 목표는 서로 다른 값이며, 재개 시 둘을 각각 보존한다.

행동이 끝난 checkpoint를 만든 뒤 실행이 계속되어 물리 오류로 멈췄다면, 앞의 checkpoint는 독립 진단용으로 사용할 수 있다. 실제 레일 수확 이력의 인계에는 최종 정상 timestamp와 같은 checkpoint만 사용한다. 오류 뒤에 이전 상태로 돌아가 연속 성공으로 집계하지 않는다.

`run_row_harvest.py --initial-native-checkpoint`는 같은 assembly·manifest·USDC SHA, 정상 최종 summary, 최종 시각의 완료 cycle을 확인한다. 최종 summary의 `actual_self_collision_audit_passed=true`와 실제 저장 상태에 대한 완료 audit도 필요하다. audit의 model SHA와 입력 `states.npz` SHA를 함께 확인하므로, audit이 진행 중이거나 다른 상태 파일을 검사한 경우에는 인계하지 않는다. 새 plant window에서 초기 검사만 하고 실패한 경우에는 실제 이동·수확을 시작하지 않은 진단 시도로 기록한다. 이미 실제 이동이나 수확을 진행한 뒤 실패하면 그 측정 이력을 보존하고 캠페인을 중단한다.

최종 `route_checkpoint`도 실제 마지막 cycle의 `completed_cycle`·물리 유효성·완료 timestamp가 명시되고 최종 summary와 audit가 통과했을 때만 인계할 수 있다. 일반 idle, 중단 snapshot 또는 marker가 없는 snapshot은 완료 행동으로 바꾸지 않는다.

현재 줄기를 완료한 것으로 표시하고 중복 재실행을 제외할 때는 수확 범위에 포함되는 모든 열매의 정확한 native joint를 확인한다. Plant22에서는 Truss01·02의 20개가 대상이다. 한 송이의 10개만 분리되었거나, 다른 송이를 포함한 분리 이벤트가 20개 있다는 이유로 전체 완료로 판단하지 않는다. 실제 최종 summary가 20개 전체를 포함하고, 각 이름·바디·joint의 native 분리 증거와 일치할 때만 전체 native 분리로 기록한다. 이전 checkpoint는 다음 window의 측정된 물리 시작 상태로 그대로 사용한다. 이는 이미 분리된 열매의 중복 시도를 피하는 기능이며 실제 수집·수확 성공 판정은 아니다.

`run_row_harvest.py --defer-exhausted-current-plant`는 별도의 운영상 보류 옵션이다. 정상 최종 checkpoint·완료 실제 자세 audit·전체 20개 범위가 확인되고, 아직 붙어 있는 각 열매의 checkpoint와 summary 시도 횟수가 원래 예산과 일치하면서 모두 소진된 경우에만 현재 줄기의 제안을 보류한다. 예를 들어 실제 19개가 분리되고 Fruit10이 6회 시도 후에도 붙어 있다면, 그 열매를 **붙어 있는 미확인 대상**으로 보존한 채 다른 줄기를 시험할 수 있다. `all_eligible_native_detached=false`는 그대로 유지한다. 시도 횟수·분리 증거·최종 측정 상태를 초기화하지 않으며, 보류한 줄기는 원본으로 다시 만들어 재시도하지 않는다. 아직 시도 예산이 남은 열매나 한 송이만 포함한 summary에는 전체 줄기 보류를 적용하지 않는다.

## 같은 줄기의 다음 송이

이전 실행 코드에서 한 cycle의 끝이 전체 시간 예산과 정확히 같으면, 다음 loop 경계에 진입하지 않아 완료 marker만 누락될 수 있다. `terminal_cycle_checkpoint.py`는 이 경우의 호환 도구다. 정상 최종 summary와 실제 모든 자세 audit, 정확한 마지막 완료 cycle·timestamp·최종 poses/q가 기존 raw `route_checkpoint`와 일치할 때만 같은 trial의 새 cycle checkpoint로 발행한다. 원래 raw checkpoint와 결과는 그대로 두며, 모든 측정 배열·속도·preload·drive 목표·native 이벤트를 그대로 복사한다. 새 physics나 수확 결과를 만들지 않는다. 일반적인 중단·미완료·이전 timestamp·audit 진행 중 상태에는 사용할 수 없다.


현재 원본 Plant22는 Truss02에만 승인된 elastic fruit-neck 가설이 적용되어 있다. Truss01을 같은 가설로 시험하려면 다음 도구로 새 assembly를 만든다.

```bash
/root/farmily_tomato/nvidia-sim/.asset-venv/bin/python \
  /root/farmily_tomato/nvidia-sim/harvest_s300/eligible_attachment_transfer.py \
  SOURCE_ASSEMBLY LATEST_VALID_TERMINAL_CHECKPOINT \
  NEW_ASSEMBLY RAIL_NEAR_INVENTORY_JSON
```

도구는 원본 Plant22의 아직 분리되지 않은, 수확 범위에 포함되는 Truss01 연결만 기존 3 N / 0.008 Nm elastic 설정으로 바꾼다. Truss00, 기존 Truss02 연결 설정, 충돌 형상, 질량, 마찰, actuator와 0.5 mm 기준은 보존한다. 이미 native에서 분리된 Truss02 연결은 분리 증거와 비활성 선언을 유지한다. 이는 명시적인 연결 가설 이전이며, 결합된 식물 반응이 이전과 같다고 가정하지 않는다.

실행에는 기존 checkpoint와 `--allow-resume-attachment-transfer`를 함께 전달한다. `--near-trusses-metadata`와 `--initial-truss-first`를 사용하면 기존 목표 송이의 남은 열매를 먼저 확인한 뒤, 소진되었을 때 같은 실제 식물의 다른 허용 송이를 선택한다. 다음 송이에는 그 송이의 측정된 열매 방향을 사용한다. 다른 송이의 이전 yaw를 가져오지 않는다.

USD property 보존 검사와 계획 통과는 native 수확 성공이 아니다. native 초기 상태·DOF 복원, 실제 접촉과 분리, 저장된 모든 실제 로봇 자세의 검사를 별도로 통과해야 한다.

## 물리 중단 뒤의 독립 새 실험

2026-10-11의 Truss01 연속 실행은 `prepare_joint_branch` 9.658333초에서 다른 줄기 Plant34의 잎과 충돌해 중단했다. Truss01 진입 전이며 새 native 분리는 0개다. 이 물리 이력 뒤에는 이전 9/10 checkpoint를 가져와 연속 성공으로 진행하지 않는다.

다른 준비 경로를 비교할 때는 `eligible_attachment_transfer.revise_fresh(assembly, output, inventory_path)`로 **독립적인 정상 초기 상태 실험**을 만든다. 도구는 기존 `apply_transfer`, compact 저장과 USD property/geometry 보존 검사를 재사용한다. 원본의 정상 로봇 관절 상태·모든 열매의 활성 연결을 확인하고, 승인된 Truss01 연결 가설만 적용한다. 초기 목표는 Truss01이고 대상 범위는 가까운 쪽 Truss01·02의 20개다. Truss00은 바꾸지 않는다.

`fresh_campaign`은 checkpoint 미사용·이전 native 분리 0개·이전 실행과 독립임을 명시한다. 측정 상태를 가져온 assembly나 비활성 과실 연결이 있는 source는 거부한다. 새 실험의 열매를 이전 9개 분리와 합산하지 않는다. 준비 경로는 robot/tool 자기충돌 검사에 더해 농장 장애물 검사를 통과한 후보만 실제 native로 실행한다.

## 레일 이동

새 window의 기존 route 인계는 실제 이전 상태에서 새 plant를 활성화한다. 이후 이동은 world-Y prismatic joint를 구동한다. 로봇 바디 transform을 순간 이동시키지 않는다.

같은 assembly에서 이미 이동한 checkpoint를 재개하면 계획기와 FCL의 초기 원점은 저장된 실제 base 위치를 사용한다. native 초기 FK 비교도 이 시작 위치에서 수행한다. 새로 요청한 도착 위치와 혼동하지 않는다.

같은 assembly의 checkpoint에서 추가 레일 이동은 다음 명시 옵션을 요구한다.

```text
--resume-native-checkpoint CHECKPOINT
--allow-resume-rail-travel
--rail-to-y STATION_WORLD_Y_M
```

rail phase는 저장된 실제 시작 관절 상태에서 연결한다. 이전 native drive 목표와 측정된 base 위치를 별도로 보존하며, 설정한 속도를 실제 smoothstep command가 넘지 않도록 시간을 계산한다. 이후 cycle은 도착 위치를 유지한다. 전체 경로와 실제 physics 검사는 그대로 적용한다.

## 제한된 잎 초기 안정화 실험

`--condition-foliage-seconds`는 최대 3초의 별도 idle 초기화 실험이다. 정확히 식별된 잎 관련 raw 충돌은 원자료로 남긴다. 이후 최소 1초를 기존 0.5 mm 기준으로 연속 검사하며, 새로운 native 분리·비정상 수치·로봇 또는 잎 이외의 충돌 예외가 있으면 canonical 시작 상태로 인정하지 않는다.

정상 strict suffix, native 최종 상태, controller 목표, 실제 모든 저장 자세 검사와 source/code/model SHA가 확인된 경우에만 `conditioned_start.json`을 만든다. 이 checkpoint는 `canonical_conditioned_start`이며 `completed_cycle=false`다. 수확하거나 완료 행동을 실행했다고 기록하지 않는다.

재개에는 `--allow-resume-conditioned-start`가 필요하다. 같은 assembly만 허용하며 연결·충돌 proxy 수정과 합치지 않는다. 일반 idle 결과나 이전 실패 결과에는 이 옵션을 붙여도 통과하지 않는다. `run_row_harvest.py`는 여전히 실제 완료 cycle을 요구하므로, canonical idle 상태는 먼저 기존 `run_motion.py`에서 실제 수확을 진행하고 정상 최종 완료 checkpoint를 만든 뒤 인계한다.

## 결과를 읽을 때

- 과거 native 분리 이벤트는 `imported_native_prior`, `native_new_event=false`로 보존한다. 재개 후 새 callback과 구분한다.
- 초기 warmup 중 분리는 수확 횟수에 포함하지 않는다.
- FCL self 경로 통과, 환경 초기 통과, 실제 접근 완료, 열매 분리는 각각 별도 결과다.
- 제한된 계획 검색에서 후보를 찾지 못한 경우는 미발견으로 남긴다. 절대적인 수확 불가능으로 바꾸지 않는다.
- 새 엔진에서 상태를 복원할 때 PhysX contact cache까지 연속 재현한 것은 아니다. 실제 바디·DOF·속도·drive 복원 오차와 첫 solved physics 결과를 확인한다.

관련 검사: `test_motion_continuation.py`, `test_conditioned_start_checkpoint.py`, `test_eligible_attachment_transfer.py`, `test_run_row_harvest.py`. CPU 회귀 통과만으로 native 수확이나 새 window 초기 물리 통과를 주장하지 않는다.
