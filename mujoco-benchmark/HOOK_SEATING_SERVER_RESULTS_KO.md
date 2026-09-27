# 꼭지 안착 경로 서버 실험 결과

기준 `f24709c` / 코드 `bb38b35` / 이전 서버 검증 `ca83357`.
작업 브랜치 `codex/server-hook-seating-20260927`. 결과 커밋은 이 문서를 포함하는 Git 커밋이다.

**정상 시작에서 목표 꼭지에 안착·유지하는 동작은 이번 예산 안에서 확보하지 못했다.** 목표 자세 생성과 실제 waypoint별 위치·회전 계획은 연결했다. 끝 자세가 가능한 경우에도 그곳으로 올라가는 경로가 목표 과실과 충돌했다. 금지 접촉을 허용하거나 경로를 강제로 밀어붙이지 않았다.

## 수행 범위

- 원본 gutter02 / Tomato_02 / MuJoCo3.13.0 / implicitfast240Hz, 제어60Hz.
- 기존 학습 데이터, 물리 자산, collision mask, margin/gap, 마찰·강성·actuator, 관통0.5mm 기준, 기존 성공 판정과 운영 정책을 보존했다.
- `propose_hook_seating_goals.py`로320개 기하 제안 중283개 수용, 다양성을 유지해32개 내보냈다. gap0.4mm, local X/Z 기울기−15/0/+15°, fraction0.5, 꼭지축 정렬 포함.
- 정상 시작 물리 예산은 baseline1 + 새 경로 최대12였다. **실제 물리 실행은 baseline1, 새 경로0**이다. 새 경로는 모두 명목 전체 경로 검사에서 막혔다. 미실행을 물리 실패나 수확 불가능으로 집계하지 않는다.
- 물리 실행에 앞서 만든 계획은 최초12개 + 준비 우회10개 + 하부 삽입5개 = **27개**다. 계획 횟수와 물리 실행 횟수를 숨기거나 혼동하지 않는다. 하부 삽입 연결 이후 탐색을 종료했다.
- 모두 SIM-GT 오프라인 진단이며 `training_eligible=false`, `hook_success=null`. RL·재학습·대규모 수집은 실행하지 않았다.

자료: [증거 목록](validation/hook_seating_server_20260927/README.md), [단계별 전체 결과](validation/hook_seating_server_20260927/stage_results.json).

호스트 화면:

```text
file:///home/rdv/docker_share/mujoko_debugging_data/20260927_hook_seating_server/index.html
```

## 1. 실제 기하와 좌표 확인

실제 목표 capsule은 `glb_col_Attachment_01`, `glb_col_TRUSS_Pedicel_proximal_02_02`다. 뒤쪽 와이어는 g384..g399이고, `glb_col_TRUSS_Rachis_05`는 별도의 송이 중심가지다. 목표/Rachis를 합쳐 허용하지 않았다.

RING 중심은 Hook 바디 원점에서 local `[-0.10640287, 0.00616804, 0.00006305]`m 떨어져 있다. 생성기는 RING 중심 위치와 Hook 원점 위치를 각각 저장하며, 계획기는 RING 중심 pose를 받는다. 실제 IK 결과를 MuJoCo FK로 다시 계산했다. 선택한12개의 위치 잔차는 최대약3.95e−8m이고 새 안착 기하 조건은12개 모두 만족했다. **팔·전체 고리·환경까지 검사하면 끝 자세가 통과한 것은5개**다. 나머지는 과실/주줄기/마운트 등의 충돌 때문에 제외됐다.

[기하 목표](validation/hook_seating_server_20260927/seating_goals.json), [MuJoCo 실제 IK pose/잔차](validation/hook_seating_server_20260927/actual_ik_goal_geometry.json), [끝 자세 환경 검사](validation/hook_seating_server_20260927/endpoint_results.json).

영상에서 목표 꼭지는 노랑, 뒤쪽 와이어는 청록, Rachis_05는 분홍으로 표시했다. 실제 자산의 양성 fixture는 IK 끝 자세를 처음부터 놓은 **정적 기하 fixture**이며 자동 진입 성공이 아니다.

## 2. 판정 대조군

생성기의 목표를 정답으로 삼지 않고 독립적으로 정한 capsule 좌표와 실제 CAD rear-wire 좌표로 native MuJoCo 거리와 새 기하를 비교했다. 테스트 capsule 반지름1mm/무중력의 독립 fixture이며 운영 식물의 동적 물성 실험이 아니다. native 거리와 helper 거리는2e−8m 이내로 일치했다.

|대조군|새 목표 기하|legacy seated|해석|
|---|---|---|---|
|뒤쪽 내부, 비관통|true|true|기하 양성. 자동 진입/유지 성공 아님|
|과실 중심만 내부, 꼭지는 가운데|false|false|기존 과실 중심 영역은 true, 꼭지 안착은 false|
|앞쪽|false|false|뒤쪽 걸림면 조건 불충족|
|와이어 바깥쪽|false|false|간격이 가까워도 내부 안착 아님|
|와이어 관통 약1.001mm|false|true|legacy의 음수 gap 허용 문제 재현|
|다른 꼭지|false(정체성)|true|기하 배치와 목표 정체성을 별도 검사|
|Rachis|false(정체성)|true|비목표 중심가지 접촉은 허용하지 않음|

접촉력도 각 native fixture에 저장했다. 양성의 `intended_force`는 기존0.01N 표본 문턱에서는 false일 수 있다. 기하 양성이라는 이유로 힘·유지 양성을 만들지 않았다.

시간 판정은 별도 대조군으로 검사했다. 처음부터 안착된 정지는 자동 진입이 아니며, 중간 이탈·hold 중 누락 간격·짧은 hold·비목표 힘·물리 오류는 유지 증거를 거절한다. 표본 수가 많아도 실제 timestamp 범위가 부족하면 통과하지 않는다.

**시간 대조군은 합성 timestamp 시험**이며 실제 탄성 꼭지를 포획했다 빠뜨리는 물리 양성/음성 fixture를 확보했다는 뜻은 아니다. 전체 자산의 정적 안착 pose와 기존 중심 진입 상태를 같은 카메라 영상으로 별도 제공했다. 실제 걸림 유지의 동적 양성 대조는 미확보다.

## 3. 경유점별 pose를 실제 계획기에 연결

`dataset_motion.plan`에 `trajectory_mode=diagnostic_pose_waypoints_v1` opt-in만 추가했다. 각 waypoint의 RING 위치와 quaternion을 실제 endpoint IK·중간 Cartesian IK·회전 보간에 전달한다. 기존 FCL 자기충돌 검사, 관절/속도 제한 및 환경 사전검사를 재사용한다. hold는 명시적인 최소 시간도 받는다.

`diagnostic_only=true`, `training_eligible=false`를 요구하며 기존 action14 인코딩을 건너뛴다. 계획 실패에도 해당 태그를 보존하고, 실행기가 진단 결과를 학습 적격으로 되돌리지 않도록 했다. 기존 staged6d의 neck/roll/elevation 해석을 바꾸지 않았다.

회귀 확인:

- f24709c의 기존 planner로 만든 고정 fixture607개 명령/phase와 수정 후 결과가 동일.
- 실제 서버 baseline의2,512개 명령/phase가 원본과 정확히 동일.
- 새 경로의 서로 다른 waypoint 회전이 실제 FK 회전에 반영되고 hold가1.1초 이상 생성됨.
- 기존 FCL 충돌 거절이 opt-in 모드에도 유지됨.

## 4. 접근 경로에서 막힌 지점

|계획군|새 계획 수|IK·자기충돌·속도 통과|전체 환경 통과|최초 차단 단계|
|---|---:|---:|---:|---|
|목표 자세별 직접 준비|12|12|0|preapproach: 마운트 g410 대 다른 열매/가지|
|끝 자세 통과5개 × 준비 우회2종|10|10|0|insert: rear wire 대 목표 과실|
|하부 삽입 후 상승|5|5|0|seat: rear wire 대 목표 과실|

첫 준비 경로를 강제하지 않았다. 끝 자세5개만 남긴 후, baseline에서 충돌 전인 preapproach/entry 부분을 재사용하고 이후 삽입 경로를 새로 연결했다. 기존 blocked insert는 복사하지 않았다. 이를 통해 준비 충돌은 피했지만, 상승 중 과실을 통과하는 문제가 남았다.

최종5개에서 첫 위반은15.95~16.36s의 seat 단계, g387~g389와 `glb_col_Tomato_02`의 명목 겹침 약0.022~0.158mm였다. 이것은 **명목 경로의 최초 기하 위반**이며 실제 물리 침투량 또는 전체 경로 최대 겹침이 아니다. 기존 nominal clearance0 검사에서 차단했다. 물리 허용0.5mm보다 작다는 이유로 비허용 접촉을 허용하지 않았다.

검사 정책은 별도 파일이다. 두 목표 꼭지와 실제 뒤쪽 와이어32개 정확한 쌍을 **seat/hold/verify에만** 허용한다. 목표 과실, 다른 과실·꼭지, Rachis, 주줄기, 거터, 마운트는 허용하지 않았다. 운영 정책 파일은 변경하지 않았다. 새 동작의 유지·확인 단계는 계획에 존재하지만 물리적으로 도달하지 않았다.

끝 자세 검사 초안에서는 allowed phase 목록과 단일 seat 검사 phase가 맞지 않아 inconclusive가 나왔다. 이를 별도 seat-only 진단 정책으로 재검사한 `endpoint_results.json`/`endpoint_preflight_corrected.json`이 최종 결과다. 초기 보고서는 로컬에 보존했으며 통과 수에 넣지 않았다.

## 5. baseline 실제 실행

baseline은 기존 blocked 경로의 비교 재실행1회이며 운영 gate 통과 결과가 아니다. 원본 정상 초기 상태에서 기존 물리로 실행했다.

- 기존 분류: `partial_center_entry`, 기존 물리 유효성 검사 통과.
- 새 기하 안착0표본, legacy 안착0표본, 목표 꼭지 안착+의도 힘0표본.
- 최대 비허용 법선력 약**3.396873N**(모든 로봇–환경 쌍 중 private post-step 재계산 최대). 이전0.734N은 선택한 g390/Rachis 쌍의 값이므로 서로 다른 집계다.
- 최대 리프트 추종 오차3.665mm, 회전 관절0.008801rad.
- baseline에 hold/verify 단계가 없으므로 유지·확인은 **평가 구간 없음**이다. 0초를 유지 실패 실험과 혼동하지 않는다.

핵심 접촉은240Hz로 저장했다. live contact/force의 직전 solver 시각과 private MjData 전체 복사 후 forward 재계산 시각을 구분했다. 기하도 post-step240Hz로 평가했다. 실제 상태에 forward를 추가하거나 actuator를 바꾸지 않았다. private 재계산 힘은 직전 solver 출력과 동일하다고 주장하지 않는다.

[baseline 요약](validation/hook_seating_server_20260927/baseline/seating_summary.json), [240Hz 접촉 원자료](validation/hook_seating_server_20260927/baseline/contact_events_240hz.json.gz).

## 6. 검증과 영상

- 기존108개 + 시간/정체성8개 = **116 passed**, MuJoCo3.13 환경.
- Torch/FCL 계획 환경에서 opt-in/회귀5개 = **5 passed**. 두 실행 모두 skip 없음.
- 별도로 native 대조군7개, 합성 시간 대조4개 assertion 통과.
- 기존/새 최종 경로/전체 자산 양성 fixture/전체 자산 음성 fixture는 동일 전경·확대 카메라로 비교한다. 독립 capsule 양성·음성 영상도 제공한다.
- 새 경로 영상은 **차단된 명목 계획 시각화이며 물리 실행 영상이 아니다**. 정적 fixture 영상의 재생 시간은 실제 hold 성공 시간이 아니다.
- 입력·코드 SHA256, 명령, 전체 계획과 실패 목록, 명시 정책을 [증거 디렉터리](validation/hook_seating_server_20260927/README.md)에 보존했다.

## 7. 남은 병목

끝 자세의 IK와 일부 환경 적합성은 확인했지만, **과실을 통과하지 않고 그 자세까지 고리 전체를 이동시키는 연결**이 미해결이다. 작은 기울기와 단순 삽입→상승으로는 이번 연결을 찾지 못했다. 모든 가능한 진입이 불가능하다는 증거는 아니다.

다음 논의는 성공 기준이나 충돌체를 완화하는 대신, 목표 과실의 형상을 포함해 고리의 입구를 통과시키는 회전·이동 경유점을 어떻게 생성할지에 집중할 수 있다. 실제 접촉·유지 양성을 확보하지 못했으므로 위치/자세 오차 내성 시험은 수행하지 않았다. `contact_retention_evidence`도 검증된 최종 hook_success나 파단 수확 판정이 아니다.
