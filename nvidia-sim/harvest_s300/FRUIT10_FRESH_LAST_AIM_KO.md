# 마지막 Fruit10의 실제 잡힘과 새 독립 송이 시험 제안

이 문서는 저장된 실제 native 상태를 재검토한 결과다. 기존 송이는 **9/10 분리, Fruit10 미분리**다. 여섯 번째 시도까지 마친 과거 경로를 되감아 성공으로 만들지 않는다. 아래 제안은 동일 자산의 정상 초기 상태에서 시작하는 **새 독립 전체 송이 시험**에만 적용한다.

실패의 직접 증거는 한쪽 벨트만 Fruit10에 닿았다는 것이다. 깊이65mm/높이−1.5mm에서 L 벨트의 목표 열매 접촉은 있었지만 R 벨트 접촉은 0개였다. 마지막 lateral−1mm 시험에서도 양측 동시 접촉은 0회였고, 기계 간격28mm 명령에 실제31.83mm에서 닫힘이 멈췄다. 그때 실제 벨트 안쪽 간격21.83mm에 열매 폭21.47mm가 들어왔지만 열매가 왼쪽으로 치우쳐 R 쪽에는 약0.96mm의 틈이 남았다.

R 벨트에는 이미 열매가 떨어진 11번/9번 꼭지 가지가 닿았다. R 상부 탈착 팔(`R_C10_removable_top_arm`, part304)과 상판(`R_C02_upper_deck`, part249)에는 7번 꼭지 가지 접촉도 기록됐다. L 벨트에는 목표10번 열매와 그 자신의 꼭지 가지가 함께 닿았다. 접촉점 최대 힘은 native 분리 조인트에 걸린 힘과 다르므로, L 쪽 3.014N 접촉점을 보고 분리 조건을 충족했다고 판단하지 않는다.

높이를+22mm로 올렸던 시험은 R 쪽의 다른 가지 접촉을 만들었고 실제 간격33.15mm, R 열매 접촉0회였다. 단순히 위로 올리는 방식은 개선 증거가 없었다. 마지막 시험의 lift 구동 목표와 실제 차이는 약1.83mm였으며, 이전 시험의8.48mm를 그대로 재사용해 높이를 보정해서는 안 된다. 정확한 과실 mesh bbox 중심은 body 원점과 일치하고 mesh centroid 차이도 약0.109mm다. 잘못된 큰 mesh 중심 오프셋이 원인은 아니다.

가장 작은 다음 시험은 첫9개까지의 기존 동작을 유지하고, **해당 송이의 native 미분리 열매가 정확히1개 남았을 때만 높이−8.5mm, 깊이80mm, lateral0mm**를 쓰는 것이다. 기계 간격28mm, 벨트0.05m/s, 닫기6초, 벨트 마무리2초, 기존 회전 방향과 물성은 유지한다.

저장된 실제 자세를 고정한 기하 투영에서 이 변경은 그리퍼를7mm 낮추고15mm 더 진입시킨다. 목표 열매의 전체 source mesh는 양측 벨트 높이 범위 안에 남는다. 기존7번 가지 접촉 시점의 R 탈착 팔 source bbox에는 최소 약4.07mm, R 상판 source bbox에는 약7.47mm 이상 분리 축이 생겼다. 9번 가지와 R 벨트의 authored primitive 간격도 기존 닫기 자세에서 약1.50mm로 벌어진다. 더 깊은 진입은 한쪽 벨트가 열매를 길이 방향으로30mm 이상 밀었을 때도 벨트 끝으로 빠져나갈 여유를 늘린다.

**11번 가지는 여전히 R 벨트 높이 범위 안에 있고, 목표10번 꼭지도 L 벨트 범위 안에 있다.** 따라서 새 동작이 성공한다고 보장할 수 없다. 이 두 가지의 실제 휨, R 벨트의 목표 열매 접촉, 실제 닫힘과 native 조인트 분리를 확인해야 한다. 이 투영은 authored source 형상의 분리 증거이며 native cooked proxy의 미래 접촉/유효성 판정은 아니다.

새 목표에 대한 기존 전체 SELF120 검사는6260표본 모두 PASS했다. 원래2mm 계획 여유는 유지했다. 시작 q는 기존 정상 source q, 목표 위치/방향은 마지막 실제 checkpoint의 Fruit10을 **오프라인 기하 참고**로만 사용했다. 그 checkpoint 자체의 lift0.143366m에서 과거−150mm 준비 재분배를 실행하면 원래 lift 제한 아래로 내려가므로, 정확한 checkpoint 시작 계획은 IK 이전에 거부됐다. checkpoint 복원이나 새 native 실행은 하지 않았다. 새 독립 시험에서 실제로 측정될 마지막 열매 상태는 별도로 다시 검사해야 한다.

결과 판정은 양측 벨트의 동일 timestamp 접촉, 실제 간격/열매 위치, 정확한 Fruit10 분리 조인트 이벤트를 함께 저장한다. 전체10개 조인트가 실제로 분리되어야 전체 송이 완료다. 기존0.5mm active physics 기준, 마찰, 강성, 힘 제한, collider와 필터는 바꾸지 않는다.

수치/해시 원자료: [validation/fruit10_fresh_last_aim_20261011.json](validation/fruit10_fresh_last_aim_20261011.json).
실서버의 전체 기하 원자료: `/root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/fruit10_candidate80_cad_obstacle_projection.json`, `fruit10_frozen_actual_capsule_capture_projection.json`, `fruit10_new_fresh_last_aim_normal_start_reference_self120/`.
