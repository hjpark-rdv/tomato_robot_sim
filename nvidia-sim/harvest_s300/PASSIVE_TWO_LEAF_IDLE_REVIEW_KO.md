# Plant20 주변 두 잎의 정지 시험과 충돌 형상 검토

2026-10-11 서버의 독립 fresh Truss01 시험이다. 기존 수확 상태를 초기화한 이어하기가 아니며 수확·레일 이동 실적으로 집계하지 않는다. `training_eligible=false`, `harvest_success=null`이다.

원본 결과는 `/root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/trials_fresh_truss01_two_passive_leaves_idle120/fresh_truss01_two_passive_leaves_strict_idle120_cpu16`에 보존했다. 분석과 원시 파일 SHA는 같은 결과 루트의 `fresh_two_leaf_idle_contact_review/summary.json`, 작은 저장소 요약은 [validation/passive_two_leaf_idle_20261011.json](validation/passive_two_leaf_idle_20261011.json)에 있다.

## 확인된 결과

120Hz/CPU16의 3초 idle은 기존 **활성 로봇·타깃** 초기 검사에서 완료됐고 새 열매 분리는 0개, 타깃 최대 변위는 0.451mm였다. 그러나 이것을 주변 두 잎의 정상 물리까지 검증한 결과로 읽으면 안 된다. 기존 buffered guard의 활성 경로 목록에는 새 `/World/BackgroundCompliantFoliage`가 없었다. 주변 잎끼리 또는 잎과 정적 농장 형상 사이의 원시 위반은 저장됐지만 판정기에 전달되지 않았고, `leaf_contact_resolution.json=[]`, `leaf_cooked_geometry.json={}`이었다. 원래 PASS 표시와 원시 자료를 수정하지 않고 이 범위 누락을 별도 보고한다.

| 항목 | 실제 기록 |
|---|---:|
| 물리 루프 | 57.043초 / 모의 3초, RTF 0.0526 |
| 접촉 콜백 | 22.558초 |
| simulate | 56.834초 — 콜백 시간이 포함되므로 합산 금지 |
| 결과 저장 | 별도 27.651초 |
| 전체 접촉점 | 1,199,596개 / 513개 collider pair |
| 새 두 잎 관련 접촉점 | 1,181,137개, 전체의 98.46% |
| 가장 깊은 원시 separation | Leaf02 mesh132 ↔ GutterSuspensionWire003, −17.196mm |
| 최대 단일 접촉점 힘 | Leaf02 mesh131 ↔ Plant20 Truss00 mesh71, 889.218N |

889N은 하나의 원시 접촉점 impulse/dt 값이다. 합력이나 실물 손상 한계가 아니다. 큰 힘이 기록된 일부 쌍은 저장된 post-fetch 자세에서 원본 표면 사이가 1.8–3.3mm 떨어져 있었다. 반면 가장 깊은 와이어 접촉은 저장 자세에서도 원본 삼각형 표면이 교차했다. 따라서 모든 원시 음수 값을 실제 표면 침투 깊이로 바꾸거나, 모든 접촉을 가짜 충돌로 치부할 수 없다.

접촉 콜백을 전부 없애는 비현실적인 가정을 해도 루프에 약 34초가 남는다. 기록 최적화만으로 해결될 상황이 아니다. 기존 3초 idle과 이 시험은 같은 모델의 격리 성능 A/B가 아니므로 속도 비율을 정밀한 회귀 수치로 주장하지 않는다.

## 앵커와 중력

두 잎은 정확한 world-connected standalone D6 몸체다. authored world/body pivot 일치 오차는 약 51nm이고 rest rotation 오차는 0°다. 잘못된 좌표 변환은 발견하지 못했다.

그러나 native에서 고정돼 있어야 하는 pivot의 위치 오차는 Leaf01 최대 16.1mm, Leaf02 최대 108.5mm였다. 몸체 중심 이동은 193.3/112.3mm, authored 자세에 대한 최대 회전은 114.3/30.2°였다. 현재 주변 잎 모델의 안정성이 입증됐다는 주장을 할 수 없다.

Standalone 잎은 기존 활성 articulation의 고정 authored-pose 중력 preload를 받지 않는다. 같은 방식의 **고정 초기 자세** torque를 계산하면 Leaf01 `[+0.002523, −0.000590, 0]Nm`, Leaf02 `[−0.000342, −0.003750, 0]Nm`다. 실제로 적용하지 않았다. zero-gravity SDK warmup에서도 이미 새 잎 관련 2,985개 접촉, −9.896mm 원시 separation, 최대 244.9N이 기록됐다. 중력 보상 누락만으로 이 초기 접촉 폭증을 설명할 수 없다.

## native 충돌 형상과 제한된 수정

같은 원본 모델의 별도 0.25초 native capture로 8개 잎 충돌체를 모두 읽었다. 127개 cooked convex 조각이 있었다. 이 capture와 앞선 3초 시험의 최초 0.25초 body pose/robot q/command는 bit identical이다.

Cooked support vertex 2,683개 중 2,637개가 원본 삼각형 표면에서 0.5mm 이상 떨어졌다. 거리 중앙값은 충돌체별 2.5–4.4mm, 최댓값은 4.8–8.15mm다. 원본 vertex 일부는 cooked 조각 합집합 밖에 있었다. 표면과 proxy의 근사 오차를 보여주는 진단이며, 0.5mm를 원본 근사도의 universal hard gate로 사용하지 않는다. 전체 삼각형·부피 포괄도 증명하지 않았다.

별도 `assembly_plant22_fresh_near20_truss01_passive_staging_leaves_accuracy_v2`를 생성했다. 바꾼 것은 정확한 Plant20 CompoundLeaf01/02의 8개 Mesh에 대한 convex decomposition API와 여섯 cooking 값뿐이다. 설정은 hull vertex limit64, max hulls128, voxel resolution5,000,000, error percentage0.5, min thickness1mm, shrinkWrapfalse다.

32,649,127개 world vertex, 279개 rigid pose, 32,142개 collider pose가 원본과 정확히 같고, 허용 cooking 필드를 되돌린 모든 effective property hash도 일치했다. 질량·관성·spring·damping·offset·filteredPairs·마찰·material·원본 형상·0.5mm 기준은 그대로다. 새 USDC SHA는 `c8b34e38a6f36e85a6e657a3828084e94d5349ddb5eb378ec4ea0e2fc4a0297b`다.

이 수정은 **물리 proxy가 달라지는 실험 가설**이다. L/part122의 native fixture 인증을 잎에 복사하지 않았다. 잎의 native cooking 양성/음성 대조, 주변 접촉 물리 유효성, pivot 안정성, 속도 개선은 모두 미검증이다. 자식 agent는 native를 실행하지 않았다.

신규 [background_leaf_proxy_accuracy.py](background_leaf_proxy_accuracy.py)는 기존 compact writer와 geometry/property audit를 재사용한다. 정확한 8개 매핑·몸체 외에는 수정할 수 없고, 숨은 material/filter/joint/offset/API 변경을 거부한다. 새 회귀 시험 6개가 통과했다.

다음 시험은 정확한 주변 잎 경로도 기존 물리 유효성 검사에 포함하고, 새 8개 cooked 형상을 읽어 동일 시작 상태에서 짧게 비교해야 한다. 원시 접촉은 유지하고 post-fetch 증거와 구분한다. 중력 preload가 필요하면 별도 가설로만 비교한다. 이런 정지 상태 확인 없이 긴 수확 실행을 시작하면 접촉 폭증과 모델 불안정을 수확 모션의 실패로 오해할 수 있다.

## 후속 native A/B: cooking 개선과 3초 안정성은 별개

부모 agent가 원본·accuracy_v2의 0.25초 capture와 accuracy_v2의 3초 idle을 실행했다. 8개 실제 cooked 조각은 127개에서 899개로 늘었고, support vertex와 원본 삼각형 표면 거리의 최댓값은 8.15mm에서 2.21mm로 줄었다. 중앙값도 충돌체별 2.5–4.4mm에서 0.61–1.14mm로 줄었다. 그러나 원본 vertex 일부는 여전히 조각 합집합 밖에 있었으므로 전체 source 표면·부피 포괄을 증명한 것은 아니다. 작은 요약은 [validation/passive_leaf_proxy_AB_20261011.json](validation/passive_leaf_proxy_AB_20261011.json)에 보존했다.

| 3초 idle 항목 | 원본 cooking | accuracy_v2 |
|---|---:|---:|
| 새 열매 분리 / 타깃 최대 변위 | 0개 / 0.451mm | 0개 / 0.451mm |
| 물리 루프 | 57.043초 | 30.849초 |
| 전체 접촉점 | 1,199,596 | 510,119 |
| 새 두 잎 관련 접촉점 | 1,181,137 | 491,660 |
| Leaf01 / Leaf02 최종 pivot 오차 | 15.03 / 108.51mm | 6.61 / 38.56mm |
| Leaf01 / Leaf02 마지막 0.5초 중심 이동 | 0.33 / 19.19mm | 23.60 / 14.05mm |

361개 timestamp에서 두 시험의 로봇 q·command와 주변 두 잎을 제외한 277개 tracked body pose는 bit identical이다. 접촉량과 기존 활성 타깃에 대한 영향은 비교할 수 있다. 루프 시간은 다른 작업이 동시에 실행된 기능 시험 값이므로 격리 성능 개선율로 주장하지 않는다.

accuracy_v2도 주변 잎이 안정적으로 정착하지 않았다. 마지막 0.5초 잎 중심의 평균 속도는 0.160 / 0.058m/s, 평균 각속도는 87.2 / 36.9°/s였다. 같은 구간의 주변 잎 접촉은 113,489개로, 0.5–1초의 55,162개보다 많았다. 마지막 구간 원시 separation 최솟값은 −3.166mm, 최대 단일 접촉점 힘은 339.39N이었다. 원래 활성 경로 guard의 범위 누락은 이 실행에도 있으므로 `completed=true`, `initialization_valid=true`를 주변 잎까지 유효한 canonical start로 승격하지 않는다. 수확·이동·conditioning 성공 실적은 0이다.

대표 쌍의 원본 삼각형을 authored rest, zero-gravity warmup 후 저장 t=0, 해당 접촉 이벤트의 post-fetch 상태에서 각각 확인했다. Leaf02 mesh132는 authored rest에서 자기 Plant20 Truss00 Fruit01 및 Truss02 Pedicel_proximal03과 실제로 교차했다. Truss02 Calyx03의 Sepal1/5는 authored rest에서 약 1.8mm 떨어져 있었지만 warmup 후 및 이벤트 저장 자세에서 교차했다. 이후 Truss00/02 Fruit03과도 교차했다. 반면 최대 344.98N의 Rachis_Trichomes 접촉은 저장 자세에서 원본 표면 간 0.142mm 간격이 있었다. 원시 pre-solve separation, 단일 impulse/dt, 저장 post-fetch source 표면 검사를 서로 바꾸어 해석하지 않는다.

두 잎을 고정된 자기 식물 과실·가지 사이에서 한 개씩의 rigid compound leaf로 회전시키는 현재 가설은 지속적인 접촉과 world-D6 위치 구속 오차를 보인다. 3초를 기다리기만 하면 안정화된다는 근거는 확보하지 못했다. 중력 preload만으로도 설명되지 않는 zero-gravity warmup 접촉이 이미 있었다. cooking 정확도 개선은 유용하지만, 더 긴 conditioning이나 추가 scope 확장 전에 이 모델의 정상 물리와 정확한 background guard 범위를 확인해야 한다. 다른 줄기·과실·필터·물성·0.5mm 기준은 변경하지 않았다.

새 원시 자료는 `trials_fresh_truss01_accuracy_v2_idle3_capture/fresh_truss01_two_passive_leaves_accuracy_v2_idle3_capture_cpu16`에 있으며, 분석은 `fresh_two_leaf_idle_contact_review/accuracy_v2_idle3_AB.json`, `accuracy_v2_idle3_contacts.json`, `accuracy_v2_idle3_source_pairs.json`에 원시 SHA·분석 코드와 함께 보존했다. 원래의 PASS 표기, 원시 접촉, 실패 근거는 덮어쓰지 않았다.
