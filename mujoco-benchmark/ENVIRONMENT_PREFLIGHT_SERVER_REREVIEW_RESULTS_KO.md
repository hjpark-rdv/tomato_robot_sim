# 환경 사전검사 재검토 서버 결과 — GPT-6 Pro 전달용

2026-09-27 KST. 기준 `bd59ea3`(필수 `1f5bc1f` 포함), 결과 브랜치 `codex/server-preflight-rereview-20260927`.

이번 제한된 검증은 완료했다. **7경로 모두 전체 스캔을 끝냈고 blocked였다. 양의 기하 간격에서도 비목표 접촉력이 실제 발생했다. 기존 부분 중심 진입을 목표 꼭지 걸림으로 해석할 수 없다.** 작은 경로 변경 5회에서는 걸림 개선을 확인하지 못했다. 사전검사 조건이나 물리 설정을 완화하지 않았다.

이 문서와 [증거 디렉터리](validation/preflight_rereview_20260927/README.md)를 함께 검토한다. 이전 1차 서버 결과는 [보존본](ENVIRONMENT_PREFLIGHT_SERVER_FIRST_RESULTS_KO.md)이며 이번 결과가 그 해석을 보완한다.

## 1. 변경 범위와 재현 정보

- 기존 `audit_environment_preflight.py`, `candidate_experiment.py`, `plan_candidates.py`, `RobotEngine`, Isaac의 `rear_capsule_geometry`를 재사용했다.
- 새 진단: `diagnose_contact_timing.py`(시점 정합성/힘), `hook_retention_diagnostic.py`(별도 걸림 지표), `render_contact_diagnostic.py`(저장 상태 비교 영상), 관련 테스트.
- MuJoCo **3.13.0**, Python **3.11.14**, **implicitfast / 240Hz**, 제어60Hz. 기존 모델·actuator·collision mask·margin/gap·학습기·생산 성공 판정을 변경하지 않았다.
- 전경 7건 진단 정책은 연산 예산만 `max_distance_queries: 2,000,000 → 12,000,000`, `timeout_s: 120 → 240`으로 변경. clearance=0, 샘플 간격, 허용 접촉 목록은 그대로다. 기본 정책 파일은 수정하지 않았다.
- 각 입력 model/trace/reference/manifest 및 코드 SHA256, seed, 정책 해시는 [provenance.json](validation/preflight_rereview_20260927/provenance.json). 입력 model/trace 해시는 실행 후에도 일치했다. 커밋 ID는 이 문서를 포함하는 Git 커밋으로 식별한다.
- 서버 원본: `/root/docker_share/mujoko_debugging_data/20260927_preflight_rereview_server`.
- 호스트 화면: `file:///home/rdv/docker_share/mujoko_debugging_data/20260927_preflight_rereview_server/index.html`.

## 2. 7경로 전체 스캔

모두 `complete=true`, `scan_stop_reason=end_of_path`, `full_path_requested=true`, `full_path_collected=true`다. 계산 예산 초과로 중단한 결과가 없다.

|경로|고유 pair/phase|상세 저장/생략|쿼리 수|실행 시간(s)|판정|
|---|---:|---:|---:|---:|---|
|original02|11|11 / 0|967,122|12.71|blocked|
|visual02|11|11 / 0|723,190|10.43|blocked|
|gutter02|11|11 / 0|4,223,302|27.93|blocked|
|gutter09|63|63 / 0|4,249,226|29.39|blocked|
|stem01|117|64 / 53|4,310,395|36.93|blocked|
|stem02|75|64 / 11|4,516,489|39.27|blocked|
|stem09|134|64 / 70|4,436,023|38.03|blocked|

[클래스별 최초·최악 위반 전체 표](validation/preflight_rereview_20260927/full_scan_tables.md)에 시각·phase·geom 쌍·거리, 종료 시각, 상세 잘림 여부를 저장했다. 주변 줄기뿐 아니라 이후 거터·과실·송이 가지 위반도 확인했다. stem01의 `other/g6`는 실제 `STEM_MainStem_01` 바디다. 분류 이름만으로 미지 장애물이라고 해석하지 않는다.

**명령 로봇 + 초기 식물의 음수 거리**와 **실제 움직인 로봇·식물의 침투량**은 다르다. 이 표는 명목 경로 기하 검사다. 일부 상세가 생략되어도 클래스별 최초/최악 집계는 보존됐다. 7건은 독립적인 무작위 성능 표본이 아니며, 이 결과로 전체 false-positive 비율을 계산하지 않는다.

## 3. 접촉 시점 정합성과 양의 간격에서의 힘

기존 `mj_step → mj_kinematics → callback`에서는 기하는 적분 후 상태이고 contact/force는 직전 solver 계산이다. live 데이터에 `mj_forward`를 추가하지 않았다. 전체 `MjData`를 `mj_copyData`로 별도 복사하고 그 복사본에서만 forward를 수행했다. 사용자 콜백 없음/플러그인0을 확인했다. **진단 전후 전체 physics step의 qpos+qvel 해시가 두 사례 모두 정확히 일치**했다.

- live 접촉력: 240Hz, `solve_time_s = poststep_time_s − dt`를 별도로 기록.
- 복사본 재계산 접촉력: 60Hz, 해당 post-step 시각의 상태와 ctrl/외력/warmstart 등을 복사해 계산. 실제 직전 solver 힘과 동일하다고 주장하지 않는다.
- A=명령 로봇+초기 식물, B=실제 로봇+초기 식물, C=명령 로봇+실제 식물, D=실제 로봇+실제 식물. A/B/C는 가상 기하 조합으로 힘을 해석하지 않는다.
- 모든 ABCD 행은 같은 시각에 계산했다. 각 열의 전체 최솟값은 발생 시각이 서로 다를 수 있으므로 그 차이를 인과 효과로 계산하지 않는다.

### gutter02: g390 ↔ glb_col_TRUSS_Rachis_05

해당 Rachis는 송이 중심가지 분절이다. Tomato_02의 목표 distal anchor는 **Attachment_01**, terminal proximal은 `TRUSS_Pedicel_proximal_02_02`다. Rachis 전체를 목표 허용 접촉에 넣지 않았다. [실제 모델 매핑](validation/preflight_rereview_20260927/mapping.json).

|측정|결과|
|---|---:|
|live 접촉 기록/활성 제약/양의 법선력 기록|1,998 / 1,522 / 1,471|
|양의 contact.dist인데 양의 힘이 있는 기록|1,471|
|선택 쌍 live 최대 법선력|0.734371 N|
|선택 쌍 60Hz 복사본 최대 법선력|0.734396 N|
|매 물리 스텝 선택 쌍 post-step 최소 기하 간격|+0.866435 mm|

두 geom의 margin/gap은 각각0.5/0.5mm, priority0이며 explicit pair는 없다. 기록된 `contact.includemargin`은1.0mm다. **겹치지 않았다는 사실은 접촉력 없이 지나갔다는 뜻이 아니다.**

같은 시각21.4000s에서 ABCD 거리(mm)는 `[+2.149974, +1.759561, +0.449746, +0.869011]`, D 재계산 법선력은0.734396N이다. 첫 명목 위반은15.238889s이고 가장 가까운 동기화 표본은15.233333s다. 둘을 정확히 같은 시각이라고 표현하지 않는다.

### stem01: g410 ↔ neighbor_stem_collision_18_05

g410은 Hook 바디의 mesh 형상으로, 고리 와이어 그 자체라고 부르면 안 된다. 상대 줄기는 고정 충돌체다.

|측정|결과|
|live 기록/활성 제약/양의 법선력|3,173 / 1,091 / 1,066|
|양의 거리에서 양의 힘|972|
|live 최대 법선력|314.486 N|
|60Hz 복사본 최대 법선력|268.573 N|
|최대 리프트 추종 오차|4.825 mm|
|최대 회전 관절 추종 오차|0.292913 rad|
|고리 중심 위치 오차 최대|233.036 mm|

4.133333s에서 A=C=−0.321959mm, B=D=+2.748066mm다. 고정 장애물이므로 A=C, B=D다. 이후 로봇이 장애물에 막히면서 명령과 크게 달라졌다. **큰 힘은 이 충돌·관통 실패 상황의 시뮬레이터 출력이며 실물 힘의 정확도나 안정한 동작을 보장하지 않는다.** 서로 다른 시각/계산 방식의 force 최대값을 동일 사건의 차이로 해석하지 않는다.

g410 margin/gap=0.5/0.5mm, 주변 줄기=0/0mm, priority는 둘 다0이다. 기록된 includemargin=0.5mm다. 자세 오차도 summary에 rad로 저장했다.

공식 MuJoCo **3.13** 문서는 margin을 힘이 작용하는 기하 팽창, gap을 추가 감지 구간으로 설명한다. 감지는 margin+gap까지, gap 구간은 비활성 접촉이다. 오래된 문서의 margin−gap 설명을 현재 버전에 대입하지 않았다. 서버 native fixture로도 양의 거리 활성 힘과 비활성 접촉을 구분해 확인했다. [3.13 XML reference](https://mujoco.readthedocs.io/en/3.13.0/XMLreference.html#body-geom-gap), [3.13 mj_forward](https://mujoco.readthedocs.io/en/3.13.0/APIreference/APIfunctions.html#mj-forward).

[동기화 요약 gutter02](validation/preflight_rereview_20260927/sync_gutter02/summary.json), [stem01](validation/preflight_rereview_20260927/sync_stem01/summary.json). 같은 폴더의 대표 표본 JSON과 전체 기록 `.json.gz`에 접촉 수·활성·힘·간격·시점을 분리했다. 집계 접촉 수는 시간에 걸쳐 반복되는 기록 수이며 독립 접촉 사건 수가 아니다.

## 4. 제한된 걸림 개선 파일럿: 개선 확인 못 함

한 라운드, **gutter02 / Tomato_02 / 5회**로 예산을 고정했다. 기존 azimuth-only 기준을 유지하고 삽입 깊이±2mm, 방위각+2/+4°만 변경했다. 기준각이−89.98944°여서 −2° 변형 대신 범위 안의+4°를 사용했다. −2° 초안은 실행하지 않았으며 집계하지 않았다. 기준 trace의 hold 전2,512개 관절 명령은 기존 trace와 정확히 동일했다. 전 시험에 같은1초 hold를 추가했다. 75° 상승·32.5mm 총 상승은 그대로다.

기존 IK/FCL 계획과 실제 물리 실행기를 사용했다. **5개 모두 명목 preflight blocked이지만, 원인 조사용으로 별도 오프라인 물리 실험을 수행했다.** 운영 gate를 통과한 결과가 아니다. 각 결과에 `diagnostic_only=true`, `training_eligible=false`, `offline_teacher_trial=true`를 기록했다. 학습·one-shot 성공 집계에 넣지 않는다.

|변형|기존 분류|물리 검사|목표 안착 표본|의도한 접촉 표본|유지 중 안착|비목표 힘 표본|
|---|---|---|---:|---:|---:|---:|
|baseline|partial_center_entry|valid|0|0|0|1,595|
|삽입−2mm|partial_center_entry|valid|0|0|0|1,583|
|삽입+2mm|partial_center_entry|valid|0|0|0|1,626|
|방위각+4°|partial_center_entry|valid|0|0|0|1,570|
|방위각+2°|partial_center_entry|valid|0|0|0|1,581|

`valid`는 기존 물리 관통 검사 통과를 뜻하며 무접촉을 뜻하지 않는다. 모든 시험에 hold60표본을 기록했다. `hook_success=null`, `legacy_retention_proxy=false`다. 잘못된 꼭지 안착+접촉 지표는0이지만 일반 비목표 접촉은 남았다.

재사용한 Isaac 기하로 **목표 capsule/고리 뒤쪽16와이어(g384..g399)의 안착**, 동시 의도 접촉,1초 유지, 마지막0.5초 상대 위치 안정성을 따로 기록했다. 비목표 힘 표본은 기존0.01N 기준으로 집계했으므로 그 이하의 힘이 전혀 없다는 뜻은 아니다. 이 proxy는 검증된 수확 성공 평가기가 아니다. 시간 연속적인 꼭지 통과·이탈 불가능성·파단 수확을 증명하지 않는다. proxy 자체와 물리 유효성·비목표 접촉은 별도 필드다.

이 장면은 기존 **거터 충돌 장면**이며 주변 주줄기/106송이는 시각 전용이다. 이번에 장애물을 제거한 것은 아니지만 현재 stem-obstacle 전체 환경과 다르다. 전체 환경 비접촉 성공으로 일반화할 수 없다. 작은 변경에서 성공을 못 찾은 것이 물리적 불가능의 증명도 아니다. 성공 동작을 확보하지 못했으므로 성공 동작의 오차 내성 검증은 미완료다.

## 5. 실행 gate / 자동 검증 / 영상

[gate_results.json](validation/preflight_rereview_20260927/gate_results.json)과 각 원본 result/preflight JSON을 실제로 포함했다. 이전 ZIP에서 누락된 문제를 보완했다.

- blocked: rollout0회, states 없음, physics_executed=false, training_eligible=false.
- inconclusive(max_samples=1): 동일하게 실행0회.
- 시작1초만 검사한 clear_prefix: rollout1회, states 저장. **전체 경로 통과가 아니며 miss**.
- 기존62개 + 새3개 = **65테스트 통과**. [테스트 로그](validation/preflight_rereview_20260927/tests.txt).
- 새3개는 실제 capsule 좌표 변환, 기존 안착 기하의 앞쪽/멀리 통과 배제, 양의 간격 활성/비활성 접촉 구분이다. 완성된 걸림 평가기의 정답률 검증은 아니다.

[영상 목록](validation/preflight_rereview_20260927/README.md): gutter02/stem01 각각 명목 계획 vs 새 물리로 기록한 상태, pilot baseline/+4° 비교. 전경/확대 및 같은 카메라, 타깃·phase·시간·힘 계산 시점 표시. 영상 작성은 **저장 qpos 재생**이며 렌더러는 physics를 새로 돌리지 않는다. 2배속/10fps 영상이므로 순간 접촉의 증거는240Hz/60Hz 수치 기록으로 판단한다.

## 6. GPT-6 Pro에 요청하는 다음 검토

1. 양의 간격 힘과 로봇 추종 실패를 고려하면, 기존 nominal gate의 겹침 검출을 단순 false positive로 완화할 근거가 없다. 이 해석과 같은 시각 ABCD 분해를 검토해 달라.
2. 중심 진입 성공이어도 현재 경로는 목표 꼭지의 뒤쪽 고리 안착을 만들지 못했다. 다음 단계는 단순 각도 재학습보다 **목표 꼭지와 고리 걸림면의 상대 자세/상승 종료 위치를 명시하는 국소 계획**이 필요한지 검토해 달라.
3. 허용 접촉은 목표 꼭지의 실제 geom 쌍·단계로 한정해야 한다. 현재 proxy를 기준 성공으로 승격하지 말고, GT fixture에서 의도한 걸림/스침/다른 꼭지/관통의 양성·음성 대조군을 먼저 정하는 것이 필요하다.
4. 그 뒤 동일 장면·동일 예산에서 기존 경로와 비교하고, 주변 주줄기 충돌이 있는 전체 환경에서 재검증해야 한다. 이번 결과만으로 전 열매의 비접촉 수확 가능성이나 수확 불가능성을 단정하지 않는다.

대규모 수집·RL·모델 변경·기준 완화는 수행하지 않았다. 요청된 측정 정합성1라운드와 제한된 동작 개선1라운드에서 종료했다.
