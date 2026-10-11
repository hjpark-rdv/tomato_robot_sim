# 공통 1mm 접촉 중단값과 집단 수확 속도 비교

## 최신: 분리된 열매와 같은 송이의 낙하 접촉

사용자는 분리된 열매가 굴러 같은 송이의 열매·가지에 닿는 상황도 수확 중단으로
처리하지 않도록 승인했다. `--released-fruit-same-truss-diagnostic`를 추가하고
새 레일 실행 기본 인자에 포함했다. 기존 cases와 직접 실행은 이 옵션을 명시한다.
아래 하판/지지대 및 기존 바닥/기체 정책과 독립적으로 선택할 수 있다.

- 한쪽 과실의 정확한 native 조인트 분리가 접촉 timestamp까지 확인되어야 한다.
  두 과실 중 부착 상태인 과실이 callback의 첫 번째로 전달돼도 분리된 반대편을 확인한다.
- 상대는 실제 manifest와 활성 USD Collider로 확인한 같은 송이의 과실·꼭지·중심가지·
  송이 부착 가지(peduncle)다. 주줄기·다른 송이·로봇은 추가 허용하지 않는다.
- 식물 namespace와 송이 번호를 함께 비교한다. 이전 식물의 `Truss_02`와 현재 식물의
  `Truss_02`는 다르다. 상태 인계된 열매는 기존 checkpoint/SHA/분리 이벤트/비활성
  조인트 검증을 통과한 경우에만 이전 자기 송이와 연결한다.
- 충돌 형상·필터·마찰·강성·Hz·drive·벨트·열매 분리 조건은 바꾸지 않는다.
  과실을 제거하거나 다른 위치로 보내지 않는다. 분리 후에도 그대로 충돌하고 움직인다.
- `released_fruit_same_truss_impacts`에 접촉력·겹침·시각을 별도 기록한다. 정밀 낙과
  물리나 포집/상품성 검증은 아니며 `training_eligible=false`, `harvest_success=null` 유지.

복귀 리프트 높이 편차는 사용자가 이번 소수 수확 시험의 blocker로 삼지 않도록 했다.
관측된 수치는 계속 남기되 임의의 복귀높이 오차만으로 실험을 중단하지 않는다.
다음 동작은 기존 계획기가 실제 시작 q와 다음 수확 목표에 맞춰 리프트·팔 자세를
계산한다. 실제 추종과 관절 한계·충돌 검사는 유지한다. 이번 변경은 리프트 제어기를
고쳤다는 뜻이 아니며, 모든 목표 높이에 자동으로 정확히 도달한다는 보장도 아니다.

검증 결과: `20261011_174730_released_same_truss_60hz`의 실제1회 실행은 이전과
같은 Plant22 경로/60Hz/CPU16/TGS64/4에서57초 전체 동작을 완료했다. 익은 과실
01~05의5개 분리, 익은07/08과 미숙09/10/11은 남았다. 1진입 예산으로 종료했으며
송이 전체 수확이나 포집 성공은 아니다. 같은 경로120Hz의6개 분리와도 결과가 다르다.
물리 loop81.012초,0.704×실시간(준비/검사/저장/렌더 제외); 같은57초 경로120Hz의
151.138초 대비46.4% 짧다. Hz가 달라 수치 응답까지 같은 최적화로 보지 않는다.
초기·전체60Hz 명목 환경·실제 자기충돌 검사PASS. 관련149개 자동 테스트PASS.
변경 전28.467초까지1,709시점 pose/q/명령/시각/phase 및 계획 phase 정확히 동일.

문제였던 분리 Fruit03↔Pedicel_proximal_09_2의28.467초 접촉을 정확히 새 범주로
기록했다(raw1.216mm; 이전 계산 후1.092mm). 하판/지지대3점, 자기 송이1점,
바닥10점의 기존 중단 대상 접촉이 진단 기록으로 남았다. 바닥 낙과 raw 최대2.115mm는
허용된 분리 후 물체의 진단 수치다. 이를 전체 장면1mm 이내나 정밀 낙과 물리 검증으로
설명하지 않는다. 이번 리프트 시작/복귀q는177.236→168.921mm이며 편차는 기록만 했다.
새 정책으로 두 번째 식물까지60Hz 연속 실행한 결과는 아직 없다.

## 후속: 분리된 열매의 그리퍼 하판·지지대 접촉 (2026-10-11)

사용자는 이미 분리된 열매가 하판·지지대에 부딪히는 현상 때문에 수확을 중단할
필요가 없다고 명확히 했다. `--released-fruit-gripper-support-diagnostic`로 다음
정확한4개 충돌체와 **해당 timestamp 이전 또는 같은 스텝에 native 조인트 분리가
확인된 과실**의 접촉만 기록 후 계속한다. collision response는 그대로 유지한다.

| 충돌체 | 원래 CAD 이름 |
|---|---|
| `/World/ActuatedTool/L/part_117` | `L_C01_carriage_saddle` |
| `/World/ActuatedTool/L/part_122` | `L_C02_lower_deck` |
| `/World/ActuatedTool/R/part_239` | `R_C01_carriage_saddle` |
| `/World/ActuatedTool/R/part_244` | `R_C02_lower_deck` |

실제 USD의 Mesh·sourceName·활성 collider·소속 jaw body를 확인하고 geometry hash를
기록한다. 붙어 있는 과실, 분리 시점보다 앞선 접촉, 다른 물체/벨트/상판/팔/거터/
주줄기는 기존 중단 기준을 유지한다. 분리 증거 없는 배경 과실도 포함하지 않는다.
NaN/Inf와 로봇 자기충돌 검사를 해제하지 않는다. 기존 바닥·기체 진단 옵션과 독립적이다.

새 `run_row_harvest.py` 기본 인자에는 이 옵션을 포함했다. 직접 `run_motion.py`를
사용하거나 과거에 저장한 cases.json을 재실행하면 옵션을 명시해야 한다. 옛 case를
자동 수정하거나 기존 실패 결과를 소급해서 성공으로 바꾸지 않는다. 환경 계획 검사에도
같은 옵션을 전달하되, 앞으로 분리될 것으로 예상한 과실을 미리 허용하지 않는다.

`contact_lifecycle_scope.json`의 `released_fruit_gripper_support_impacts`와 `records`에
접촉력/impulse, raw callback 겹침, 실제 분리 시각, 소속/형상 근거를 보존한다.
계산 후 형상 검사의 정확한 겹침은 기존 `leaf_contact_resolution.json`에 별도 보존한다.
`active_guard_exceedance_deferred=true`는 의도한 중단 제외가 있었다는 뜻이다.
기존 `strict_0_5mm_passed` 등의 이름은 옛 형식 호환 필드이며 현재1mm 전체 통과나
포집 성공으로 읽지 않는다. `collection_verified=false`, `released_fruit_physics_verified=false`,
`harvest_success=null`, `training_eligible=false`를 유지한다.

같은 fresh Plant22, 같은 경로/벨트/60Hz TGS64/4/CPU16에서 새 옵션만 추가한
native1회 결과: 원래27.5초의 Fruit02–왼쪽 saddle 접촉(계산 후1.120mm)을 기록 후
진행했다. raw callback1.444mm/3점, 단일 접촉점 힘 최대0.01471N이다.
기존27.5초까지1,651시점 pose/q/명령/시각/phase가 정확히 같고 계획 phase도 같다.
기존4개 분리에 Fruit05가 추가되어5개 분리됐다. 그러나28.467초에 분리 Fruit03이
같은 송이 `Pedicel_proximal_09_2`와 부딪혀 raw1.216mm/계산 후1.092mm로 다시
중단됐다. 이는 하판·지지대 밖의 접촉이므로 이번 정책으로 허용하지 않았다.
초기/실제 자기충돌 검사PASS,28.467sim초/41.420loop초=0.687×. 중도 종료 결과를
120Hz 완주 실행의 전체 시간과 비교해 속도 향상률로 보고하지 않는다.
관련 접촉/CLI/환경 연결106개와 레일 실행38개 테스트PASS. 전체 학습이나 추가 물리
모델 변경은 하지 않았다. 결과는 `20261011_170345_released_fruit_support_60hz`에 보존한다.

2026-10-11. 사용자 요청으로 현재 S300 Isaac 수확 실행기의 공통 겹침 중단값을
0.5mm에서 1mm로 변경한다. 앞선 10월 10일 시험의 현 송이 과실–그리퍼만 1mm인
선택형 설정과 다르다. 기존 실행 결과는 다시 판정하거나 덮어쓰지 않는다.

## 적용 범위

- `run_motion.py --contact-limit-mm 1`이 새 기본값이다.
- 로봇 팔·그리퍼·벨트·활성 식물과 관련된 기존 검사 대상에 공통 적용한다.
  거터·주줄기·다른 과실과의 접촉도 해당 활성 물체와의 접촉이면 같은 값을 쓴다.
- 초기 native 상태, 매 물리 스텝 callback/tensor 검사, opt-in 계산 후 형상 확인이
  같은 설정을 사용한다. 계산 전 접촉값과 계산 후 형상 증거를 모두 보존한다.
- 과실–그리퍼 및 현 송이 과실끼리의 개별 설정을 생략하면 공통 값을 상속한다.
  명시한 개별 override는 기존처럼 정확한 collider 매핑을 사용한다.
- GUI launcher는 `S300_CONTACT_LIMIT_MM=1`을 기본으로 사용하고, row runner의
  기본 인자에도 세 값 1mm를 명시한다. 실행 JSON과 HTML에 실제 값을 기록한다.

이 변경은 중단 판정의 허용치다. 물리 엔진의 충돌체, contact offset/margin, 마찰,
강성, 질량, actuator 및 native 파단값을 바꾸지 않는다. 계획기의 자기충돌/FCL
회피 여유도 변경하지 않는다. 1mm를 의도적으로 뚫고 가도록 경로를 만드는 기능이 아니다.

잎 제외와 이미 분리된 열매의 지정 바닥/기체 낙과 진단 예외는 기존 설정을 유지한다.
정적 배경끼리의 겹침 하나로 전체 장면을 중단시키는 새 검사는 추가하지 않는다.
과거 strict foliage-conditioning 진단만은 기존 계약대로 명시적 0.5mm가 필요하다.
과거 MuJoCo 학습/판정 코드와 원자료를 일괄 치환하지 않는다.

## 재현

현재 실행 명령에 `--contact-limit-mm 1`을 사용한다. 현재 과실별 override를
명시하지 않으면 모두 1mm다. 옛 명령에 `--fruit-contact-limit-mm .5`가 남아 있으면
그 특정 쌍은 계속 0.5mm이므로 삭제하거나 1로 바꿔야 한다.

엄격한 이전 기준 비교는 아래와 같다.

```text
--contact-limit-mm .5 --fruit-contact-limit-mm .5 --fruit-fruit-limit-mm .5
```

GUI launcher에서는 `S300_CONTACT_LIMIT_MM=.5`로 이전 공통 값을 선택한다.
별도의 `S300_FRUIT_CONTACT_LIMIT_MM` / `S300_FRUIT_FRUIT_LIMIT_MM` 환경 변수가
있으면 각각 우선 적용되므로 실행 시작에 출력되는 세 값을 확인한다.

## 실제 시험 자료

집단 수확 알고리즘 기준 커밋은 `4d45485`다. 모든 비교는 같은 잎 제외 모델,
full693 DOF, CPU16, 수평 송이 축 진입, 양쪽 안쪽 벨트 50/50mm/s,
최소 대상 수량 3개 정책을 사용한다. 새 학습 데이터로 쓰지 않는다.

- 알고리즘 기준: `/root/docker_share/mujoko_debugging_data/20261011_112538_ripe_group_harvest/`
- 0.5mm 속도 비교: `/root/docker_share/mujoko_debugging_data/20261011_113601_group_harvest_speed/`
- 공통 1mm 비교: `/root/docker_share/mujoko_debugging_data/20261011_114941_common_1mm_harvest_speed/`

각 폴더의 cases/command, 실제 상태, native 접촉/파단, 초기/실측 자기충돌 검사,
소스 사본과 해시를 보존한다. 중도 중단한 실행도 결과표에 포함한다.
속도는 시뮬레이션 초 / 실행 loop 초이며 초기 로딩·사후 검사·저장은 별도다.
여러 사이클 실행의 loop에는 사이클 사이 새 경로 계획 대기시간이 포함된다.
Hz/solver/iteration 변경은 수치 응답도 바꾸므로 동일 물리의 최적화라고 설명하지 않는다.

## 최종 결과

각 설정 1회이며, 같은 초기 경로에서 시작했다. 이후 분리/변형 결과에 따라
다음 집단을 선택하므로 전체 사이클 수가 다르다. 모든 실행의 초기 상태 및
저장된 전체 로봇 자기충돌 검사는 통과했다. 중단 사례는 속도 개선의 성공으로 세지 않는다.

| 설정 | 공통 mm | sim초 / loop초 | 배속 | 대상 분리 / 대상 밖 분리 | 결과 |
|---|---:|---:|---:|---:|---|
| 기준 TGS120/64 | 0.5 | 54 / 152.656 | 0.354 | 5/7 · 0 | 완료, 대상2개 남아 종료 |
| PGS120/64 | 0.5 | 23.742 / 55.012 | 0.432 | 0/7 · 0 | 겹침 중단 |
| PGS60/64 | 0.5 | 23.683 / 28.822 | 0.822 | 0/7 · 0 | 겹침 중단 |
| TGS120/32 | 0.5 | 54 / 109.592 | 0.493 | 6/7 · 0 | 완료, 대상1개 남아 종료 |
| TGS60/64 | 0.5 | 24.917 / 36.216 | 0.688 | 3/7 · 0 | 겹침 중단 |
| PGS120/64 | 1 | 24 / 55.258 | 0.434 | 0/7 · 0 | 겹침 중단 |
| PGS60/64 | 1 | 23.867 / 28.679 | 0.832 | 0/7 · 0 | 겹침 중단 |
| TGS60/32 | 1 | 54 / 62.681 | 0.862 | 5/7 · 0 | 완료, 대상2개 남아 종료 |
| TGS60/32 + 벡터 복사 | 1 | 54 / 59.955 | 0.901 | 5/7 · 0 | 완료, 대상2개 남아 종료 |
| TGS60/64 | 1 | 106 / 180.854 | 0.586 | 7/7 · 0 | 두 사이클 완료 |

TGS60/64의 181초에는 두 번째 집단 계획 대기25.373초가 포함된다.
빠른 TGS60/32 벡터 복사는 기존54초 동작의152.656초 대비 처리시간 약60.7% 감소다.
이 비교에는 Hz/iteration 변경이 포함된다. 접촉값 복사 옵션만 비교하면62.681→59.955초
(약4.35% 감소)이며 각1회이므로 시간 변동이 포함될 수 있다.

`--native-contact-vector-copy`는 기존 buffered recorder의 native Float3 복사를
선택하는 옵션이며 기본은 꺼져 있다. 켜기 전후 **3,241시점의277바디 pose,
로봇 q·명령·벨트 명령·phase·timestamp 및222,044개 접촉점 multiset이 완전 일치**했다.
callback 집계 시간은7.458→6.913초이며 simulate 시간에 포함된 값이다.

동일 solver/Hz의 0.5→1mm 변경도 원래 중단 시점까지 세 대조의 상태/명령/접촉이
완전 일치했다. TGS60/64는1,496시점/82,426접촉점, PGS60/64는1,422/76,604,
PGS120/64는2,850/150,485다. 물성 변경 없이 중단값만 바뀐 공통 구간 검증이다.

## 관찰해야 할 차이

- PGS120/64는 오른쪽 벨트–01번의 계산 후 겹침 상한1.020mm로 중단했다.
  PGS60/64는 오른쪽 벨트–01번1.161mm, 왼쪽 벨트–02번1.090mm로 중단했다.
  1mm를 추가로 올려 이 실행들을 통과시키지 않았다.
- TGS60/32의 계산 전 callback에는 `fixed/part_28`–01번의 순간6.635mm 겹침이 있다.
  같은 스텝의 계산 후 native convex 검사에서는 분리된 상태로 확인했다.
  25건의 계산 후 확인은 모두 통과했지만 **연속 시간 전체의1mm 이하 보증이 아니다**.
- TGS60/64도 callback 최대4.369mm와 계산 후 증거를 별도 보존했다.
  계산 후 확인10건 중 허용된 겹침 상한 최대0.220mm였다.
- 미수확09/10/11이 함께 밀리는 최대 이동은 기존 최대75.978mm,
  TGS60/32 최대91.714mm, TGS60/64 두 사이클 최대108.339mm였다.
  TGS60/32의 종료 이동은약24mm(기존약17mm)다. Hz/solver 수치 응답과 사이클 차이가
  있으므로 동일한 식물 반응이라고 말하지 않는다.
- 완료 사례에 NaN/Inf, 로봇 자기충돌, 과실 이외 조인트 파단은 없었다.
  7개 분리 사례의 실제 마지막 화면에서도 대상7개는 낙과, 대상 밖3개는 부착 상태다.
  포집 성공과 실물 안전성은 검증하지 않았다. GUI/병렬 수확 속도는 이번에 미측정이다.

**공통 허용치1mm는 기본 적용하고, 빠른 Hz/solver/복사 조합은 선택형으로 둔다.**
빠른 육안 동작 확인에는 유용하지만, 순간 접촉/변형의 정확도가 이전 설정과
동일하다고 확정한 최종 물리 설정은 아니다.

## 검사·재실행·영상

offline782개 테스트 통과(별도 native SDK 전용2모듈 제외), 실제 native 실행9회
(이 라운드 strict4 + 1mm4 + 복사 대조1), 이전 알고리즘 기준1회와 비교했다.
후처리에서는 최초 집단의 authored 좌표와 warmup 후 native 좌표를 혼동하던 오류를
수정했다. authored 입력은 manifest와 검증하고 native 차이는 별도 지표로 기록한다.
다음 사이클의 실제 측정 경계 검증1e-8m는 유지한다. 원본 trace는 변경하지 않았다.

빠른 설정을 새 폴더에 다시 실행:

```bash
cd /root/farmily_tomato
task_out="/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_group_fast_1mm"
/root/isaaclab_env/bin/python -u nvidia-sim/harvest_s300/benchmark_motion.py \
  /root/docker_share/mujoko_debugging_data/20261011_095724_inward_full_truss/assembly_leaves_visual_only \
  /root/docker_share/mujoko_debugging_data/20261011_114941_common_1mm_harvest_speed/copy_case.json \
  --output "$task_out" --timeout 1800
```

이 명령은 headless다. 기존 전체 자산과 해당 policy.json이 필요하며 Git만 clone하면
대형 자산/영상은 포함되지 않는다. 0.5mm 기준 명령을 새 코드로 재현하려면 앞 절의
세 strict 인자를 명시한다. 기존 결과 파일을 수정하지 않는다.

실제 저장 상태의 동일 `truss_top` 시점,1배속/6fps 영상 두 개:

- 빠른1사이클: `trials/tgs60_32_1mm/actual_motion.mp4` (325프레임)
- 대상7개 분리2사이클: `trials/tgs60_64_1mm/actual_motion.mp4` (637프레임)

관찰용으로 영상에서만 잎/배경 식물을 숨겼으며 실제 상태/물리 형상은 바꾸지 않았다.
전체 ffmpeg 디코드와 프레임 수 검사 통과. 결과 폴더 `index.html`에서 두 영상과
모든 실패/원자료를 볼 수 있다. 보존 요약은
`validation/common_contact_limit_20261011.json`이다.
