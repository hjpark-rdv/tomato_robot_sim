# 같은쪽 세 정차점 연속 수확: 독립 원자료 감사

실제 저장 상태를 CPU에서 읽고 좌표 변환만 재계산했다. 새 physics/GPU 실행은 **0회**다. 유효 경로는 `active_window_baseline/step_00_plant22 → active_window_followup/step_01_plant27 → step_02_plant29`이며,180Hz 실패/별도 시험은 합산하지 않았다.

세 구간이 모두 완주했고 현재 방문한 송이에서만 **17개 분리**됐다. 기존 색상 기준 익음 16개, 미숙1개(`Truss_02_Fruit_09`)다. 남은 열매13개는 익음5개·미숙8개다. 이는 관절 분리 수이며 회수·실물 수확 성공 수는 아니다.

|현재 식물|DOF|완주|새 분리(익음/미숙)|루프 RTF|벨트 구간 합산 RTF|최저 세부구간 RTF|동결 attachment|
|---|---:|---|---:|---:|---:|---:|---:|
|22|133|completed|6(5/1)|2.298|1.310|0.859 (belt_finish)|20|
|27|169|completed|5(5/0)|2.582|1.212|0.836 (belt_finish)|14|
|29|199|completed|6(6/0)|2.346|1.191|0.814 (belt_finish)|9|

전체 실제 physics 191.816895초 / 루프 79.461427초 = **2.414배속**. 기존3식물 탄성379DOF 비교군1.104배속 대비 루프는 **2.187배** 빨랐다. 현재 타깃만 탄성으로 바꾸고 다른 부착 식물은 측정 위치의 고정 장애물로 바꾼 결과이므로, 동일 물리 모델의 엔진 가속으로 해석하면 안 된다.

실제 setup 합은 **395.442841초**이고 setup+loop는 **474.904268초**(기존329.544611초)다. 최초 검증/모델 로딩/JIT/솔버 구성은 아직 비싸다. planning·export·render·GUI 대기는 이 합에도 포함되지 않는다. step1은 중복 모델 읽기 수정 전의 실제 시간이며, 뒤의 CPU 최적화 수치로 바꿔 쓰지 않았다.

독립 확인 결과:

- `all_segments_completed`: **True**
- `all_records_finite`: **True**
- `all_current_target_only_releases`: **True**
- `all_eq_events_match_saved_releases`: **True**
- `no_eq_reactivation`: **True**
- `all_frozen_body_poses_exact`: **True**
- `all_frozen_forces_torques_zero`: **True**
- `all_frozen_eq_attached`: **True**
- `all_previous_free_fruits_preserved`: **True**
- `all_prepared_named_mapping_exact`: **True**
- `all_canonical_bank_parent_terminal_matches`: **True**
- `all_transition_hashes_valid`: **True**
- `all_body_path_orders_identical`: **True**
- `all_boundary_clocks_identical`: **True**
- `all_policy_pass`: **True**

상태 전환은 단순 새 장면 시작이 아니다. 부모 실제 마지막 qpos/qvel/eq_active/ctrl/감쇠/warmstart를 공통379DOF 상태 은행으로 이름 매핑한 값이 다음 은행과 정확히 같다. 두 경계의 누적 시간 차이는0초이며 eq 재활성화는0회다. 세 모델의 body357개·geom2244개·eq30개는 유지되고 자유도만133→169→199로 바뀐다. 누적 분리 열매6개/11개는 다음 구간에서 free joint·비활성 attachment를 유지하여 실제로 움직이며 공중 고정되지 않는다.

|상태 경계|누적 시각 차이(s)|부모GPU→은행CPU 위치 최대오차(m)|실제 저장 영상 pose 경계 위치 최대오차(m)|은행→새모델 위치 최대오차(m)|
|---|---:|---:|---:|---:|
|22→27|0.0|8.628853645e-07|4.768371582e-07|2.220446049e-16|
|27→29|0.0|8.628853645e-07|9.536743164e-07|2.220446049e-16|

동결 식물의 attachment는 분리로 표시하지 않고 eq_active=True 그대로 보존했다. 동결 body의 저장 pose는 각 구간 전체에서 비트 단위로 동일하고 동결 attachment force/torque 최대는모두0이다. GPU의 static-static CONNECT 런치만 정확한 대상별로 제외했다. 향후 식물의 조기 분리는0개다. 기존 주변6그루/충돌체1150개와 시각적 농장 배경도 유지된다.

접촉 정책은 기존 선택된 현재 송이의 belt↔fruit/pedicel만 특정 닫기·열기 단계2mm, 나머지는1mm다. policy/finite/qacc/capacity는 모두통과했다. 총 접촉력 최대값은 특정 과실/비목표 힘으로 귀속하지 않는다.

한계:

- Joint release is not collection or a validated agricultural harvest success;17 releases include1 authored-color-ineligible fruit.
- Current plant and previously released free fruits remain dynamic; future/completed attached plant articulation is frozen. This changes physical response from full379 elastic baseline.
- Solver and contact caches are recreated at window transitions. This is state continuation, not a single persistent solver instance.
- Raw pose differences from CPU double/GPU float32 are reported; no1mm contact gate was relaxed by this audit.
- The existing exact belt fruit/pedicel2mm selected-phase stop exception remains; all other active pairs retain1mm. No global2mm policy.
- Minimum belt_finish phase RTF remains below1 at all3 stations despite average loop and combined belt phases exceeding1.
- Setup overhead remains high in actual sequence; later verification dedup/local-cache experiments do not retroactively change setup measurements.
- No new GUI/headless equivalence test is established by this saved-state audit; no contact witness proves a neighbor was physically contacted.
- Native convex gripper proxy approximation and fruit torsional separation model retain previous limitations.
- Color eligibility is existing authored RGB SIM-GT, not detector output or biological validation.

원자료:

- JSON: [/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/INDEPENDENT_THREE_STATION_AUDIT.json](/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/INDEPENDENT_THREE_STATION_AUDIT.json)
- 이전379DOF 비교: [/root/docker_share/mujoko_debugging_data/20261018_010218_newton_same_bed_continuous/report_native_metrics.json](/root/docker_share/mujoko_debugging_data/20261018_010218_newton_same_bed_continuous/report_native_metrics.json)
- Plant22: [/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_baseline/step_00_plant22/physics/summary.json](/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_baseline/step_00_plant22/physics/summary.json)
- Plant27: [/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_01_plant27/physics/summary.json](/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_01_plant27/physics/summary.json)
- Plant29: [/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_02_plant29/physics/summary.json](/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_02_plant29/physics/summary.json)

추가 교차검사: 누적 분리 event JSON이 이전 실제 event+현재 event와 정확히 일치한다. 모든분리 시각은 eq가 꺼진 실제 저장 시각과오차0초다(이벤트 solver_step은적분 전, 저장 solver_step은적분 후라1 차이). 향후식물분리0에는 동결 attachment의분리제외 정책이포함되며, 비목표접촉이물리적으로항상무해하다는증거는아니다.

실행한CPU감사스크립트동일바이트사본: [/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/audit_scripts/independent_three_station_audit.py](/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/audit_scripts/independent_three_station_audit.py)

후속 CPU 준비시간 개선(기존 실행과 분리): prepare의 호출 내 SHA 검증 중복을 없앴다. 동일169DOF 상태 은행으로 새 출력에 준비하는 단회시험은55.699→49.399초(11.31% 감소), rchar4.407→3.561GB(19.20% 감소)였다. input_scene.xml/MJB/source_export/proof/commands의 바이트와 초기·canonical 모든배열은수정전후동일하다. 원본480파일은각각SHA를실제로읽고, 이후955요청은파일identity/링크불변을검사해재사용했다. 입력변경거부/호출간재검증을포함한관련25테스트통과. Cold start 또는반복분포측정은아니고, 과거physics의실제setup395.443초를소급대체하지않는다.

CPU준비A/B근거: [/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/prepare169_scope_ab_comparison.json](/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/prepare169_scope_ab_comparison.json)
