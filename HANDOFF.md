## 2026-11-13 커밋·푸시 저장

- 사용자 요청으로 V6 실제 농장 카메라 수확 루틴, 공유 CAD/센서 형상 및 이동 베이스 추종, 중앙 후퇴 스캔, 가시 과실 재시도·잔여1개 처리, 계획 중복 계산 감소, 송이 앞 정렬·직선 진입과 정렬 유지 보완을 함께 저장한다. 프로필·회귀시험·검증 요약도 포함한다.
- 현재 브랜치 `codex/s300-newton-standalone-20261108`을 기존 시뮬레이션 작업의 `tomato` 원격(`hjpark-rdv/tomato_robot_sim`)에 게시한다. 요청 시점 관련137회귀, 새 JSON 파싱, 쉘 문법 및 diff 공백 검사 통과. 기존 기록의 '커밋하지 않았다'는 각 과거 작업 시점의 상태이며, 아래의 검증 범위·실패 기록·미완료 범위는 그대로 보존한다.

## 2026-11-13 최신: 5906 중단 확인 및 진입 전 정렬 유지 보완

- 사용자의 '시작해놨어?' 확인 시, 기존 PID915744는 종료되어 있었다. `staged_entry_camera_20261113_214332`는 실제1.76838m/수확15회/영역 과실68개 후 step088의 실제 직선 이탈5.5366mm(기준5mm)로 감사에서 중단했다. 해당 미검증16번째 모션은 수확으로 세지 않았다. 기존 runtime 파일에는 `running:true`가 남아 있었으므로 실제 프로세스 종료·summary.error를 확인한 후 qpos/시계를 유지한 채 오류 상태로 바로잡았다. 이후 campaign 예외 시에도 runtime의 running을 false/phase를 error로 저장한다.
- 실패 궤적을 분석하니 transfer 말미 준비 위치 오차5.20mm가 정렬0.12초 뒤에도5.75mm로 남은 상태에서 진입이 시작됐다. 5906 정렬 유지 시간을0.12→0.30초(+0.18초)로 늘렸으며, 기존5mm 직선·도달 기준과 자세·충돌·제어설정은 변경하지 않았다.
- `staged_alignment_repro_300`: 실패 직전 step087의 **동일한 실제 qpos/qvel/시계/ctrl/eq/damping/warmstart**에서 이어서, 실패 계획의 관절점·속도 설정·모델을 유지하고 정렬 유지 시간만 바꿔 실제 Newton GPU로 재현했다. 전체1917상태/진입1130표본 검사 통과. 준비 위치 오차1.644mm, 직선 이탈 최대4.022mm, 수평 오차0.369°, 실제 진입각12.308°, 준비 간격104.319mm, 턱2회 닫기 및5mm/3° 목표 도달 통과. `reproduction.json`, `arrival_receipt.json`, `continuation_check.json` 보존. 과실 분리·접촉력 검증이나 전체3.85m 완주로 주장하지 않는다.
- 관련137회귀 및 compile/bash/diff 검사 통과. 커밋하지 않았다.
- 최종5906 PID953698, tmux `farmily_preview:viewer5906`, output `staged_aligned_camera_20261113_220113`, `--autostart`, end3.85m/max1000수확/max10000결정. 실제 시작 및 수확2회 완료 후에도 running=true를 확인했다. 새2회의 준비 위치 오차0.798/1.111mm, 직선 이탈4.204/3.041mm로 통과. `staged_alignment_live_validation.json` 및 체크인 `nvidia-sim/validation/s300_v6_staged_alignment_20261113.json`에 실패 사례 재현·목표 도달·실제 실행 증거를 기록했다. process/launch/start_5906_latest 및 hanging_preview 별칭 갱신, 이전 실패 기록 보존.

## 2026-11-13 최신: 송이 앞에서 정렬한 후 직선 진입

- 사용자 요청: 송이로 한 번에 가지 말고 조금 뒤쪽에 먼저 이동한 다음 직선으로 들어간다. 기존 V6의60mm 후퇴는 수확 중심 기준이라 롤러 입구는 이미 송이 표면에 가까워 보일 수 있었다. V6 농장 동작에서만 선택된 신선한 카메라 과실 추정 중심+이미 알려진 반경의 로봇 쪽 표면을 계산하고, 롤러 입구가 그 표면에서100mm 떨어진 준비 위치를 먼저 잡는다. 계획에는5mm 추종 여유를 추가한다. 숨은 과실 GT 중심/새 모델/물리·제어 변경은 없다.
- `side_camera_motion.py`: `transfer → pregrasp_align(0.12초) → approach → 잡기/놓기2회 → withdraw → return`. 준비 위치까지 이동/정렬/복귀할 때는 롤러가 해당 송이의 진입 전 경계면을 넘지 못하도록 IK/OMPL 전체 연결 경로에 추가 조건을 적용한다. 진입/후퇴는 동일 방향·직선 위에서 수행한다. 선택된 송이의 관측 반경과 실제 yaw 후보에 따라 진입 길이가 달라지며 검증된6회에서는133.8~183.0mm였다.
- `preview_side_search`/`approach_path`는 V6에만5mm 간격의 Cartesian IK 점을 사용한다(이전 preview 기본6개 표본은 유지). 계획의 조밀한 관절 보간에서도 수확 중심이 직선에서2mm 이상 벗어나거나 회전이3° 이상 달라지면 거절한다. 모든 실제 Newton 상태에서는 기존 수평1.5°/진입15°/카메라 위/기둥100mm/관절·충돌 조건에 추가로 직선5mm·방향3°를 확인하고, 준비 정렬이5mm 안에 도달한 뒤에 진입했는지 검사한다. 실제6회 검증 모두 준비 단계에서100mm 이상 떨어져 있었다.
- `LocalCheckedPlanner.constraint`는 상태 캐시를 사용하더라도 추가 경계 조건을 매번 적용한다. 전체/9축 선분 캐시 키에도 조건 식별자를 포함해, 일반 경로에서 검사한 선분을 준비 위치 제약이 있는 경로에 잘못 재사용하지 않는다. 새 회귀는 양 끝이 유효하더라도 가운데 준비 경계 조건을 위반하는 선분이 캐시를 통해 통과할 수 없는지 검사한다.
- 실제 `staged_entry_patrol_v1`: 수확6회/영역 과실24개/관측35회/전체 실측13424상태, 모두 도달·자세·가시성·연속 상태 보존 검사 통과. 독립적인 native FK 검사에서 롤러 최소 준비 간격101.045mm, 준비 위치 오차 최대4.653mm, 직선 이탈 최대4.842mm, 목표 방향 변화 최대0.605°. 실제 active penetration0m. 모델 SHA `4594f6cff5cf586396162c9bc4817a54b924d359bedffc0f1c091f7ba3d4107b` 유지.6회 예산이0.44524m에서 끝났으므로 기존6회/33개 기록과 수확량·처리 속도를 일대일 비교하는 실행은 아니다. 전체 농장 커버리지/물리 과실 분리·접촉력 검증으로 주장하지 않는다.
- `staged_entry_run_validation.json`은 숨은 위치 대신 선택된 관측값으로 준비 경계를 독립 재계산하고 실제 qpos로 직선/정렬/입구 간격을 확인한다. `staged_entry_physical_validation.json`은 실제 GPU RGB-D instance/depth23893건 및 수확 후 보이지 않는지384건, 기존 전 구간 연속 상태/모델 SHA/실제2회 닫기를 확인한다. 관련137회귀, compile/bash/diff 공백 검사 통과. 커밋하지 않았다.
- 실제5906 WebSocket QA `staged_entry_camera_qa_20261113_214123`: 초기 준비/시작·일시정지(qpos·시계 유지)·재개·새2단계 수확2회 통과. 실측 준비 간격102.3/103.3mm 및 직선 이탈4.20/3.11mm 확인. 체크인 증거 `nvidia-sim/validation/s300_v6_staged_entry_20261113.json`에 물리/관측/독립 FK/GUI 검증을 함께 기록한다.
- 최종5906 PID915744, tmux `farmily_preview:viewer5906`, output `staged_entry_camera_20261113_214332`, end3.85m/max1000수확/max10000결정. 기존 실행을 정상 닫기 뒤 교체했고, 이번에는 실제 시작 버튼도 눌러 실행 중임을 측정 상태로 확인했다. `staged_entry_deployment_validation.json` 기록. 첫 배포 시작 검사에서 짧은 초기 scan 뒤 계획 중에는 시뮬레이션 시계가0.3초 미만에 머무르므로6초 이내0.3초 조건은 timeout했지만, 시작 명령은 성공했고 재시작 없이 실제 진행된 qpos/시계/수확 상태를 별도 확인했다. process·launch·start_5906_latest 및 hanging_preview 별칭 갱신, 이전 기록 보존. 커밋하지 않았다.

## 2026-11-13 최신: 촬영 후 팔 동작 지연 계측 및 중복 계산 감소

- 사용자 피드백: 카메라 촬영 후 팔이 움직이기까지 오래 멈춘다. 실제 GPU 렌더는 평균 약0.105초이며, 신선한 관측의 후보 선택 → IK/전체 경로 검사 → 실행 입력 생성 → GPU 실행 준비가 직렬로 이어진다. 이전 모션의 전체 실측 상태 감사도 다음 촬영 전에 동기 실행한다. 사진 처리만의 지연으로 설명하지 않는다.
- `GripPlanner`는 관절 검사 후 `Planner`가 계산한 같은 FK에서 자세 검사를 수행하고, 충돌 검사 뒤에는 중복 geometry 검사 없이 하우징/기둥 등 나머지 조건을 검사한다. FK를 한 번만 계산하지만 모든 표본/충돌·관절·카메라 높이·기둥100mm 조건은 유지한다. 유한 lift IK의 SO(3) 오차는 SciPy 반복 변환 대신 기존 native arm planner와 같은 MuJoCo quaternion 연산을 사용한다.
- V6 `CameraClearance.enable_rigid_cache()`만 불변 형상에서 Tool과 같은 weld의 강체 거리, 검증된 두 slider weld의 정확한 턱 좌표별 거리를 재사용한다. 다른 팔·기둥·환경의 거리는 모든 검사 자세에서 새로 질의한다. 반올림한 자세 키/근사 성공 캐시는 없고, 재사용된 강체 거리에1e-10m 보수적 여유를 둔다. 기본 거리 질의는 기존처럼 캐시하지 않는다. 후보6개를 탐색하는 한 관측 안에서는 source manifest/정책을 한 번만 읽고 카탈로그 SHA 검증도 매 새 계획 호출마다 수행한다.
- `run_newton_side_camera_patrol.py`에 실제 camera RPC 완료 → 계획 준비 → 입력 준비 → GPU 준비 → 실제 팔 변화(명령과 측정 모두0.002rad 이상) 계측을 추가했다. 성공 계획만의 시간이 아니라 각 단계의 실제 벽시계를 남긴다. operator pause가 있으면 해당 지연에 포함될 수 있다. `camera_to_motion_latency`는 단순한 physics RTF와 구분한다.
- 실제 `latency_patrol_v1`: 같은0.5m/6회 수확 구간에서 기존33개 영역 수확 전부 보존, 총35회 실제 GPU 카메라 관측/13971개 실측 상태 검사 통과. 평균 경로 계산2.530→1.792초, 전체 벽시계181.083→153.432초. 두 실행의 선택된 세부 궤적과 시뮬레이션 시간은 조금 다르므로 동일 명령열의 엄밀한 속도 벤치마크로 주장하지 않는다. 단일 저장 관측의 프로파일에서는 실패 후보6개 검색3.005→1.729초, 성공 계획2.794→2.276초였다. 완전 즉각 응답/전체 실시간 달성은 아니다.
- 새 실행 최대 실제 진입각12.310°, 수평 오차0.747°, 영역 목표 위치 오차0.01674mm, active penetration0.00893mm. 모델 SHA `4594f6cff5cf586396162c9bc4817a54b924d359bedffc0f1c091f7ba3d4107b`와 물리·제어·5mm 도달 제한·두 번 닫기/열기·관측 깊이/픽셀/가시성 조건을 유지한다. 전체13971개 실제 상태에서 재사용한 카메라 간격과 독립적인 native 질의를 비교했고, 최대차1.000002e-10m/2mm 허용 판정 일치였다. `latency_run_validation.json`, `latency_physical_validation.json`에 기록한다.
- 관련135회귀 및 compile/bash/diff 검사 통과. 새 회귀는 native SO(3) 일치, FK 중복 제거 후 자세/충돌 거절 유지, 강체 거리 재사용 중 팔/턱 변화로 생기는 새 간섭 거절을 확인한다. ROS/5905/원본 형상과 물리 모델은 바꾸지 않았다. 커밋하지 않았다.
- 실제 WebSocket QA `latency_camera_qa_20261113_203057`: 초기 GPU 준비/시작·실측 시계와 qpos가 멈추는 일시정지·재개/수확2회 통과. 촬영 RPC 반환 후 실제 팔 동작까지3.626/4.437초였고, 앞선 실패 관측들에 소비한 시간까지 포함하는 전체 송이 대기시간은 더 길 수 있다. 즉각 수확 모션이라고 표시하지 않는다. `nvidia-sim/validation/s300_v6_camera_latency_20261113.json`에 headless/카메라 가시성/강체 거리 비교/실제 GUI 증거를 저장했다.
- 최종5906 PID768906, tmux `farmily_preview:viewer5906`, output `latency_camera_20261113_203256`, end3.85m/max1000수확/max10000결정. 사용자 시작을 기다리며, 원래 실행은 정상 닫기 뒤 교체했다. runtime/process·launch_command·start_5906_latest.sh 및 hanging_preview 별칭 갱신, 이전 실행 기록은 보존했다.

## 2026-11-13 최신: 보이는 익은 열매의 조기 포기 및 잔여 1개 보완

- 사용자 피드백: 분명 보이는 토마토를 많이 지나친다. 기존 live 기록 `level_camera_20261113_173032`의 약1.2m 시점에서 인식한 송이11개가 첫 잡기 후보/50mm 전진 재시도 두 번의 유한 실패 뒤 영구 operational blocker로 남아 있었다. 같은 송이의 다른 잡기 위치를 충분히 보지 않았고, 기존 최소2개 조건으로 남은1개도 큐에서 빠졌다. 일부는 실제 두꺼운 줄기/로봇 간섭이므로 검출 실패나 절대 도달 불가능으로 단정하지 않는다.
- V6 전용 `SideCoveragePatrol`/`side_camera_coverage.py`: 원래 공통 CameraPatrol의 기본 최소2개는 유지하면서 이번 V6만 보이는 익은1개를 정리 대상으로 유지한다. `CameraRegionSelector`의 명시적 최소1개+고정 로봇 진입 방향일 때만 singleton을 허용하며, 익음·12개 이상의 가시/깊이 픽셀·관측 신선도·정확한 source identity 조건을 유지한다. 단일 과실로 줄기 방향을 추정하거나 숨은 GT 중심을 사용하지 않는다. `select_group`의 새 옵션은 기본값을 보존한다.
- 매 관측에서 최대6개 카메라 footprint를 시도한다: 기존50mm 진입 후보가 막히면25/15mm 깊이와 다른 관측 과실 subset/남은1개를 검토한다. 실패한 계획은 physical attempt/수확 성공으로 세지 않는다. 과실별 실제 동작 시도는3회로 유한하게 제한한다. 기존 유한 보류 송이도 실제 차체가 마지막 실패 지점에서120mm 이상 전진했고, 신선한 현재 영상에 남은 익은 과실이 있으며, 근접 측면 작업영역 안일 때 한 번만 다시 검토한다. 이전 blocker/실패 증거는 이력에 보존하며 물리 실패·식물 quarantine은 재개하지 않는다. 모든 식물이 같은 고정 native 모델에 있으므로 V6에서 직전 plant에만 후보를 묶던 우선순위를 제거해 다른 가까운 송이도 처리한다.
- 개발 중 singleton 새 자세에서 손목/그리퍼의 아주 작은 명목상 겹침이 실제 servo를6.4mm 밀어 도달 실패를 만드는 것을 발견했다. 이를 허용오차 확대나 오래 기다리기로 해결하지 않았다. V6 grasp endpoint의 open/closed 실제 native contact에 최소0.5mm 여유를 추가하고, 수평 yaw 후보를±14→±12°로 줄여 기존15° 실측 제한 안에 추종 여유를 남겼다. 대체 후보와 singleton의 진입/후퇴는 최소1초로 검증한다. 기존 물리 모델/drive·관통 상한1mm·도달5mm/3°·수평1.5°·기둥100mm·카메라/관절/한바퀴 회전 금지는 유지한다. 두 번 잡기/놓기도 유지한다. v1/v2 개발 실행은 실측 자세 제한 위반으로 중단됐고 성공 실행으로 세지 않는다. v4는 준비 중 취소한 개발 실행이다.
- 최종 headless `coverage_patrol_v5`: 실제 Newton GPU/RGB-D, 주행0→.500464m/6회 예산 종료, 완료 수확 동작6회 모두 reached/두 번 닫힘 통과, 기하 수확33개. 최대 수평 기울기 .765410°, 진입 법선 각12.411518°, 실측13,807상태 검사. 새 시점 재검토5건. 이 중 이전 영상에서 실제 보였던 익은 과실28개를 포집 영역 안에서 확인했다. 이전 `level_scan_patrol_v5`의 같은 첫0.502m까지 기록은2회/기하12개/관측 익은12개였다. 새로 잡은 관측 익은 과실16개를 exact source key로 대조했다(`coverage_comparison.json`). 이전 긴 실행의 앞부분과 새0.5m 예산 실행의 비교이며 전체 농장/엄밀한 처리량 A/B/실시간 성능 검증은 아니다. 62.89050sim초/181.08324wall초로 더 많은 후보·동작에 따른 지연은 남는다. 물리 분리나 실제 수확력 검증이 아닌 기존 영역 수확 판정이다.
- 실제 GUI `coverage_camera_qa_20261113_175455`: GPU 준비 후 t=0 대기, 시작/일시정지 qpos·clock 고정/계속, 두 번의 수확 동작 중 두 번째가 이전 잔여1개를 정리하는 것까지 확인했다. 실측 화면/native 오차1.19e-16m. 패널에 보이는 익은 열매/수확 대기 송이/접근 보류 송이 수를 추가했다. API 기본 동작 보존·새 관측 재시도 한도·다른 후보/깊이·singleton의 익음/깊이 조건·정지 grasp contact 여유 등132회귀 통과. compile/bash/diff 검사 통과.
- 최종5906 PID623883, tmux `farmily_preview:viewer5906`, output `coverage_camera_20261113_175736`, 최대3.85m/1000동작/10000결정, native GPU 준비 후 시작 대기. 새로고침→**카메라 수확 시작**. 기존5905/ROS/원본 자산/모든 과거 결과는 그대로이며 커밋/푸시는 하지 않았다. 작은 근거 `nvidia-sim/validation/s300_v6_visible_coverage_20261113.json`. 실제 간섭으로 접근하지 못하는 과실은 여전히 보류하며 모든 보이는 과실의 수확을 보장하지 않는다.

## 2026-11-13 최신: 수평 진입·리프트 중심 스캔·대기 단축

- 사용자 피드백: 그리퍼가 기울어 진입하고, 동작/정지 대기가 느리며, 스캔 카메라가 작물에 너무 가까웠다. 최신 V6/5906만 수정했다. `profiles/v6_lift_centre_scan.json`의 native IK 준비 자세는 실제 optical X=.000113m, Y=.250091m, Z=.980128m로 리프트 폭 중심(X≈.001918m)까지 약303mm 후퇴했다. 카메라 CAD의 하향15° 장착은 그대로다. 초기 GPU 센서 영상에서 기존 왼쪽 송이를 볼 수 있고, 매 수확 후 이 스캔 자세로 실제 복귀한다. 원본 물리 MJB/식물/5905/ROS는 그대로다.
- 새 카메라 수확 탐색은 pitch/roll0°만 사용한다. 기존 ±14° pitch 대안을 제거했으며 수평 yaw0/±14°의 기존15° 진입 법선 범위는 유지한다. 모든 실측 진입·잡기·놓기·후퇴 상태에 그리퍼 상면의 수평 오차1.5° 제한을 추가했다. 기둥100mm·카메라 높이/하우징·기존 관절/회전·충돌 검사는 유지한다. 수평 도구 전면이 두꺼운 줄기에 과도하게 들어가지 않도록 보이는 과실의 진입 깊이를79→50mm로 정했고, 도달 기준5mm/3°나 수확영역 크기는 바꾸지 않았다. 서비스 중심의 차체 Y 보정을 .25→.40m로 맞췄고 실제 관측→실제 이동→재관측 규칙을 유지한다.
- V6 전용 명령 시간배율1.35(속도1.35배/가속도1.35²배, 생산 drive limit 변경 없음), 진입/후퇴 최소1.0/.85→.45/.45초, 삽입 대기 .45→.08초, 두 번의 닫기/열기는 유지하면서 hold .70→.30초/open hold .25→.08초로 단축했다. 두 번 잡는 삽입 구간은 기존약5.45→약3.14초다. 매 harvest에서 이전 성공 IK seed를 재사용하고, lift+arm IK를 먼저 시도하며, 이미 확인한 정확히 같은 상태/역방향 선분만 한 계획 호출 안에서 재사용한다. 반올림이나 인접 상태 재사용·실측 audit 생략은 하지 않는다. 같은 native ray를 방금 촬영했고 실제 스캔 자세가 같은 경우에는 불필요한 정지 스캔 구간 대신 그 관측으로 유한 재관측 정책을 갱신한다.
- `run_newton_harvest.run(...,ready_callback=None)`의 선택적 콜백을 CUDA 준비 후 첫 스텝 전에 추가했다. V6 시작 버튼은 이제 센서와 물리 GPU가 모두 준비된 뒤 활성화된다. t=0/초기 상태를 보존한 채 대기하므로 시작 클릭 뒤 GPU 업로드/graph 준비를 다시 기다리지 않는다. 기존 호출의 기본 동작은 같다. 새 실제 GUI 시작→첫 물리 상태 갱신0.73369초, 일시정지 qpos/clock 고정·계속·두 번 잡기·완료·종료 통과(`level_camera_controls_validation.json`).
- 실제 CUDA 실행 `level_scan_patrol_v5`: .962429m까지 수확 동작4회/기하 수확28개, 각 동작 두 번 닫힘·도달 통과. 실측 최대 수평 기울기 .659395°, 도달 위치오차 최대 .002611mm. 이는 수평 자세를 못 찾은 송이를 기울여 잡지 않고 유한 실패/다른 실제 위치에서 재탐색하는 모드다. 전체 루프는 여전히 실제 충돌/IK/연속 상태 검증 때문에 대기가 남아 있으며, 이 실행은 전체 루프 실시간이나 이전 실행 대비 처리량 향상 검증이 아니다. 개발 v1~v4는 다른 서비스/진입 설정의 결과로 보존했다.
- 최신 코드의 GUI QA `level_camera_qa_20261113_172852`: 수평 수확1회/6개, 최대 수평 오차 .527164°, 화면/native 위치오차3.60e-16m. 기본 탐색과 native exact-state/선분 캐시의 실제 장애물 거부·초미세 인접 상태 독립 검사·실측 검사 재실행까지126회귀 통과. 기존 카메라/물리 기록과 개발 결과는 덮어쓰지 않았다.
- 최종5906 PID583691, tmux `farmily_preview:viewer5906`, output `level_camera_20261113_173032`, 자산 `level_scan_patrol_v5/assets`. 새로고침 후 **카메라 수확 시작**. 시작 대기/최대3.85m·1000수확동작·10000결정이며 끝까지 도달·모든 송이 수확을 보장하지 않는다. 작은 근거 `nvidia-sim/validation/s300_v6_level_scan_20261113.json`. 새 커밋/푸시는 하지 않았다.

## 2026-11-13 최신: V6 농장을 실제 카메라 관측 순회로 연결

- 사용자가 기존 카메라 시야 확인→토마토 선택→수확 루틴을 최신 V6에도 적용하도록 요청했다. `farm-side`는 이제 `run_newton_side_camera_patrol.py`의 온라인 실행이다. 기존 좌표 기반 모션 시험은 `farm-side-motion`으로 남겼다. `CameraPatrol`/`CameraRegionSelector`와 실제 Newton GPU SensorCamera의 RGB·optical-Z depth·가시 SIM-GT 라벨을 재사용한다. 학습된 검출기는 아니며, 가려진 과실 중심을 목표 좌표로 사용하지 않는다.
- 새 `side_camera_assets.py`/`side_camera_scene.py`/`side_camera_motion.py`/`side_camera_viewer.py`가 최신 측면 장착·중앙 recessed 카메라·덮개 홈·컨베이어 CAD를 센서와 화면에 함께 적용한다. packed 시각 농장과 native 모델의 추가 fixed stem 때문에 과실 body ID가 달랐던 부분은 정확한 이름으로 매핑했다. 매 표시 상태에서 팔 native FK 오차를 검사한다. 원본 식물·물리 모델·5905/ROS는 유지한다.
- 기존 좌측 준비 자세를 독립 프로파일 `profiles/v6_left_camera_ready.json`로 저장했다. 파생 initial qpos/ctrl/pose metadata만 갱신하고 원본 MJB SHA `4594f6cff5cf586396162c9bc4817a54b924d359bedffc0f1c091f7ba3d4107b`를 보존한다. 기존 lift gravity feedforward가 .75m actuator 상한에서 잘리는 문제는 .740m 작업 상한으로 제어 여유를 남겨 해결했다. 중력·gain·물리 ctrlrange·기존 도달 허용오차 5mm/3°는 변경하지 않았다.
- 매 실행 구간은 직전 실제 qpos/qvel/ctrl/eq/damping/warmstart/clock을 검증하여 이어간다. 측정된 미세한 관절 한계 초과는 허용된 measured envelope의 짧은 연결 동작으로 복귀하며 실제 상태를 clamp하거나 순간이동하지 않는다. 유한 IK/직접 연결/필요시 OMPL로 계획하고, 실제 모든 저장 상태의 기둥100mm·카메라 높이·yaw 회전·진입15° 제한을 검사한다. 송이 진입 후 close/hold/open을 두 번 하고, 두 번째 hold의 실제 도달과 영역 내 과실만 기존 기하 수확 원장에 반영한다. 물리적 분리나 접촉 수확 성공 검증은 아니다.
- 최종 headless `runs/20261113_conveyor_bin/side_camera_patrol_v7`: 실제 카메라 관측18회, 수확 동작4회/기하 수확27개, 최종 주행 .527169m(4회 예산 종료). 실제11,419상태 검사, 최대 진입각14.21805°, 최대 capture 위치오차 .723685mm, 최대 활성 관통 .031880mm. 12,163개 관측 instance의 실제 mask/depth 픽셀 수를 JSON과 대조했고, 수확된 instance가 후속 센서 영상에서 사라지는169검사를 통과했다. 52.03959sim초/91.11140wall초이며, 전체3.85m 커버리지나 전체 루프 실시간 성능을 검증한 결과가 아니다. 이전 v5는1.2m까지4동작/29개였지만 서비스 위치 정책 변경 전의 별도 개발 결과다. 연결 끊김 시 미완료 v6 기록은 완료로 세지 않는다.
- 실제5906 GUI 시험은 `live_camera_qa_20261113_170747`: t=0/초기 센서1회/정지 시작, 시작 버튼, 실제 이동, 일시정지의 qpos/clock 고정, 계속, 두 번 그리핑/기하 수확7개, 종료까지 통과했다. 측정 화면/native 최대 위치 오차4.51e-16m. 실제 센서 초기 영상에서 왼쪽 송이가 롤러 사이에 보이는 것을 확인했다. `side_camera_live_controls_validation.json`과 `side_camera_run_validation.json`에 근거를 보존했다.
- 최종5906은 tmux `farmily_preview:viewer5906`, `live_camera_20261113_170940`에서 새 실행으로 시작 대기한다. 브라우저 새로고침 후 **카메라 수확 시작**. RGB/가시 토마토 영상·깊이 표시·일시정지/계속/순회 중지 지원. 최대3.85m/1000동작/10000결정으로 실행하며 개별 계획·물리 제약 때문에 끝까지 수확한다고 보장하지 않는다. `process.json`/`launch_command.json`/`start_5906_latest.sh`와 hanging preview alias를 갱신했고 이전 재생 실행/자산은 보존했다. 노트북 연결이 끊겨도 tmux 서비스는 유지된다. 112회귀 통과, 아직 새 커밋은 하지 않았다.
- 별도 실행: `./nvidia-sim/run_s300_newton.sh farm-side --prepared-assets /root/farmily_runtime/newton/runs/20261113_conveyor_bin/side_camera_patrol_v5/assets --end-station 3.85 --max-harvests 1000 --max-decisions 10000` (동일5906 서비스 중복 실행 금지). headless는 `--headless`. 작은 근거: `nvidia-sim/validation/s300_v6_live_camera_patrol_20261113.json`.

## 2026-11-13 최신: 농장 시작·주행·복귀를 좌측 촬영 자세로 수정

- 사용자 지적: 왼쪽 토마토 수확을 시작할 때 카메라가 왼쪽 작물을 볼 수 있어야 한다. 기존 V6 조립 확인용 parked 자세를 농장 시작/home으로 사용해 실제 광축이 `[+.129,+.966,-.224]`(진행 방향 +Y)을 보고 있었다. 이번에는 외형만 회전하지 않고 실제 native 시작 qpos와 actuator hold를 바꾸어 새 GPU 실행을 했다.
- `side_farm_motion.camera_ready_start()`는 첫 식물의 두 송이 실제 과실 중심과 카메라 extrinsic/K를 사용해 유한 IK 탐색으로 촬영 준비 자세를 정한다. 기둥100mm 이격/관절/카메라 상단 및 하우징 이격/회전 제한을 유지한다. 최초 실측 광축은 `[-.842338,+.420918,-.336592]`, 카메라 위치 `[-.302815,.244387,1.095053]m`. 두 송이20개 중심이 교정된 화면 안에 투영되며 평균 중심은 `[239.43,321.19]px`(480×640), 광학 깊이 .35619m다. 투영/광축 검사이며 차폐 검사나 새 RGB-D 검출 실행은 아니다.
- 새 home은 좌측 촬영 자세이며 최초1초 관측 대기/주행/매 수확 뒤 복귀에도 이를 사용한다. `side_farm.initial_for_plan()`은 새 qpos에 맞춰 ctrl/속도/몸체 pose metadata를 함께 초기화한다. 원본 자산/MJB는 수정하지 않고 새 실행의 initial_state만 파생한다. 새 `validate()`는 모든 실측 상태의 기존 자세 제약과 시작 투영, ready/rail/return 중 작물 면 광축을 검사한다.
- 새 실제 Newton GPU 실행: `runs/20261113_conveyor_bin/side_farm_left_camera_start_v1/physics`. 4지점×두 번 그리핑,11,331스텝/51.801624sim초/21.736779loop wall초(RTF2.383132), 최대 활성 관통 .104900mm, 비정상 종료0, 기존 이상적 영역 수확26개. 전체11,332상태 검사 통과; 준비·주행·복귀의 최소 좌측 수평 정렬 .893582(좌측 정면에서 최대약26.7°), 카메라 상단 최소13.573mm, yaw153.368~180.068°다. 원본 물리 모델 SHA4594f6c… 유지. 이전 GPU 기록/장면은 보존했다.
- 현재5906 PID471180, output `farm_gpu_left_camera_start_20261113_154821`, tmux `farmily_preview:viewer5906`, **새 시작 자세에서 일시정지**하여 사용자가 먼저 확인할 수 있게 했다. `시작 자세 보기` 버튼은 재생 없이 처음으로 돌아가며, `시점 → 시작 카메라 방향`은 시작 광축과 교정된 수직 FOV의 고정 시점이다(이동 중 카메라 추종이나 센서 영상은 아님). `수확 모션 재생`으로 새 모션 재생. 기존 장착부 좌표 오류에 대한 매 프레임 native/화면 일치 검사도 유지한다. 5905/ROS/원본 자산 불변.
- 관련32회귀/compile/diff 검사 통과. 작은 근거는 `nvidia-sim/validation/s300_side_farm_camera_start_20261113.json`, 전체 측정은 위 실행의 `camera_validation/validation.json`에 저장했다. 기준 커밋19e67f7 이후 작업이며 아직 커밋하지 않았다.
- 실제 브라우저 HTTP200/JS오류0, 시작 카메라 방향/수확부 확대 캡처, 최초1·2차 닫기/열기 및 좌측 카메라 방향 유지, `시작 자세 보기`로 t=0/정지 복귀 통과(`side_farm_left_camera_browser_validation.json`). `side_farm_left_camera_start_optical.png`에서 왼쪽 송이가 롤러 사이 정면에 보이는 것을 확인했다. 영상 캡처는 실제 시작 optical pose의 Viser 시점이며 새 센서 노출은 아니다. 진입/수확/후퇴 실측6,421상태의 최대 법선 이탈14.2185°로15° 제한 유지(`entry_angle_validation.json`). 최종5906은 t=0/정지/1배속이다.

## 2026-11-13 최신: 농장 화면의 로봇암·리프트 분리 표시 수정

- 사용자가 농장 화면에서 베이스 플랜지와 리프트 사이가 벌어지는 문제를 발견했다. 원인은 `preview_robot_mount.body_poses()`의 V6 장착 대상 좌표가 주행 0m의 CAD bounds에 고정돼 있었던 것이다. 차체·캐리지는 native 주행을 따르지만 팔·어댑터는 뒤에 남았다. 수정 전 주행 .163m에서 화면의 모든 팔/도구 몸체가 실제 native 위치보다 정확히 .163m 뒤에 있었다. 실제 GPU 모델의 root/platform 연결은 정상이다.
- V6 장착점에 월드 좌표의 주행축 변위를 적용하고 새 CAD 기준 bounds 생성 시 rail=0을 명시했다. 팔·베이스·어댑터·리프트가 함께 이동한다. 농장 뷰어는 매 렌더에서 표시된 모든 팔/도구 몸체를 실측 qpos의 native FK와 비교하여 1e-8m/회전행렬 오차 초과 시 거부하며, runtime에 오차와 장착점 좌표를 기록한다. 접속자별 `장착부 확대` 시점을 추가했다.
- 전체 GPU 저장 상태13,004개 × 팔/도구11몸체를 대조했다. 최대 위치 오차1.165e-15m, 회전행렬 오차1.333e-15, 베이스-캐리지 상대 이동2.221e-16m, 어댑터-캐리지 상대 이동0m. 마지막 상태에서 실제 CAD 플랜지-어댑터 면 간격−1.408e-8m, 어댑터-캐리지 간격0m. 근거: `runs/20261113_conveyor_bin/side_farm_mount_validation.json`. 원본 GPU 모델/상태 SHA 불변이며 물리를 재실행하지 않았다.
- 역방향/정방향 주행·리프트·다른 팔 자세와 회전된 차체의 주행축을 포함한 회귀2개 추가, 관련30시험/compile/diff 공백 검사 통과. 현재5906 PID453471, output `farm_gpu_mount_fix_20261113_153124`, tmux `farmily_preview:viewer5906`으로 교체했다. 기존5905와 원본 자산은 그대로다. 앞의 농장 동작 검증은 물리 기록을 검증한 것이며, 이번 수정으로 이동 중 표시 좌표까지 일치하게 했다.
- 실제 브라우저 HTTP200/JS오류0, 주행 .15743m 뒤 1차 유지 상태에서 `장착부 확대`를 선택해 CAD 플랜지·20mm 어댑터·캐리지가 틈 없이 이어진 화면을 확인했다. `side_farm_mount_browser_validation.json`/`side_farm_mount_connected.png`에 근거 보존. 최종은1배속·처음부터 재생 상태다.

## 2026-11-13 최신: V6 농장 투입 및 두 번 그리핑 GPU 시험

- 사용자 요청: 최신 V6 로봇을 기존 농장에 투입하고 토마토 위치에서 두 번 붙였다 떼는 동작을 보이게 한다. `side_farm.py`/`side_farm_motion.py`/`side_farm_viewer.py`/`run_newton_side_farm.py`와 `farm-side` 실행 모드를 추가했다. 현재5906을 V6 농장 GPU 기록 재생으로 변경했으며 기존 조립체/5905/원본 자산은 보존한다. 기준 커밋19e67f7 이후 미커밋 작업이다.
- 기존 source `patrol_reuse385m_20261111/up`의 전체 native 농장 마스크를 보존하고 V6 root 설치, 상판 제거,162개 기체/리프트/컨베이어 상자를 새 XML/MJB에 추가했다. 상자는 실제 Robot_world(차체 이동) 또는 platform(리프트+차체 이동)에 붙어 GPU에서도 따라간다. 원본 무충돌 부품은 다시 활성화하지 않는다. `side_farm_source_preservation.json`: 원래 농장 마스크/좌표·형상, 질량·관성·관절범위·마찰·solref/solimp·gap/margin·감쇠·actuator 파라미터 동일 확인. 과실/얇은 가지 무충돌, 굵은 줄기 충돌의 기존 geometric harvest approximation을 유지한다.
- 준비 변형 `side_farm_20261113_v1`은 비활성 로봇 geom에 bit4를 잘못 추가한 탐색 중간본이므로 사용하지 않는다. v2는 이를 고쳤으나 XML 기본 gap을 원본 binary와 맞춘 최종판은 **`/root/farmily_runtime/newton/assets/side_farm_20261113_v3`**이다. 기존 patrol_current는 변경하지 않았다. 카메라 Tool_fixed extrinsic과 V6 주석을 config에 넣었으나 이번 실행은 GT 좌표 모션 시험이며 새로운 RGB-D 노출이나 카메라 자율 검출을 수행하지 않는다.
- `side_farm_motion_v3`: 기존 inventory의 실제 농장 토마토 좌표에서 native FK/IK + OMPL 연결/리프트를 재계획했다. 선택된4지점은 Plant17/Truss1, Plant19/Truss1·2, Plant20/Truss1. 입구부터 약.796m까지 이동한다. 송이에60mm 진입하고.45s 대기, close→.70s hold→open→.25s open hold를 두 번(속도/가속도 제약으로 stroke당 약.775s) 실행 후 후퇴한다. 기존 .15s 짧은 접촉과 달리 진입 상태가 약5.45s 유지된다. 경로상태22403개 검사, 총59.7087초 계획. 실제 농장 상태에 맞는 source plant가 없는 inventory 항목은 시험하지 않는다.
- 실제 CUDA Newton/MuJoCo Warp 실행 **`side_farm_gpu_v4_physics`** 완료:13003스텝/59.719677sim초/23.810666루프wall초, RTF2.508106. 최대 활성 관통.081204809mm, broadphase406/contact1/constraint4, 비정상 종료·분리 이벤트0. modelSHA4594f6cff5cf586396162c9bc4817a54b924d359bedffc0f1c091f7ba3d4107b. 이전 v1/v3 GPU 시도는 업로드 기본 q=0의 초기 constraint workspace 부족으로 물리 스텝 전에 실패했다. 실제 준비된 시작 자세는 native 접촉0이다. 최종 njmax8192는 초기 업로드 공간만 확대한 것이며 충돌/관통 허용 기준은 바꾸지 않았다.
- 측정13004상태 모두 새 V6 posture/카메라/기둥 조건 재검사 통과. 카메라 높이13.367289mm/하우징 간격2.5mm, 기둥100mm, wrist3최대90.003964도. 실제 진입6027표본에서 최대14.377053도(15도 이내). 네 지점의8번 실제 닫힘 확인, 영역 기준26개 과실이 들어옴을 기록했다. 이는 물리 분리/접촉력 검증이 아닌 기존 영역 수확 판정이다. 측정 기록과 모델·계획 SHA를 일치시킨 후에만 재생한다.
- `side_farm_viewer.py`: 원본 가까운 농장 CAD/최신 V6/덮개 홈/카메라/컨베이어를 표시하고 실제 GPU qpos를 보간한다. 매번 재계획하지 않고 정지/계속/처음부터/배속 지원. CPU 검사지점 대신 실제 송이를 대상으로 보인다. 과실은 두 번째 유지 동작을 실제 실행하고 measured 영역 안에 들어온 경우에만 숨긴다(기존 ideal 수확 표시). default는 정방향1회 재생이며 끝나면 정지; 자동 반복 순간이동하지 않는다. 접속자별 시점/세계Z 위 방향 유지.
- 화면 준비를 줄이기 위해 실제 작업 구간 Y[-.4,1.446]m와 완전히 떨어진 plant visual geom20106개만 전송에서 제외한다. 구간 경계에 걸친 메시/근처 가지·과실/시설물은 원본 삼각형 그대로 유지하며 물리 농장 전체는 그대로다. 표시 시점 이름은 '작업 구간'. 이는 과실 좌표나 충돌체를 줄인 것이 아니다.
- 관련23회귀(두 번 닫기/열기와 유지 시간, 팔·리프트 고정, 표시 구간 경계 메시 유지 포함), compile/bash 문법/diff 공백 통과. `nvidia-sim/validation/s300_side_farm_20261113.json`에 실제 물리/상태 검증 요약. GUI WebSocket 실제1·2차 닫기/열기 및 정지·처음부터 재생 통과(`side_farm_controls_validation.json`). 서버 headless SwiftShader 브라우저의 전체/근거리 농장 캡처는 렌더 지연으로 timeout하여 별도 저해상도 QA 중; GPU 물리 실행 실패와 구분한다.
- 현재5906 PID436716, tmux farmily_preview:viewer5906, output `farm_gpu_replay_20261113_151446`, 모드는 `farm_gpu_replay`. process/launch/start_5906_latest.sh 및 hanging_preview/process.json 갱신, 기존 실행은 `*.before_farm_*` 보존. GUI의 '수확 모션 재생' 또는 '처음부터 재생'으로 즉시 재생한다. 새 시뮬레이션은 `./nvidia-sim/run_s300_newton.sh farm-side --prepared-assets /root/farmily_runtime/newton/assets/side_farm_20261113_v3 --headless --max-grips 4 --grip-cycles 2`로 별도 폴더에 실행 가능하다. '--headless'를 빼면5906 화면을 연다(기존5906은 먼저 종료 필요). 아직 커밋하지 않았다.

- 최종 화면 개선: `populate_scene(..., shadows=True)` 옵션을 추가하고 이번 농장 배경만 shadows=False로 표시한다. 원본 삼각형/재질/물리 모델과 기존 뷰어 기본값은 유지한다. 원거리 표시 제외와 함께 작업 구간의 농장 삼각형 약1565만개를 그대로 유지한다. 기본/확대 시점은 통로 안쪽, 시점 변경은 접속자별로만 적용한다.
- 최종 실제 브라우저 **HTTP200/JS오류0/새 접속 일시정지 유지/1·2차 그리핑 단계/2차 유지 장면** 통과(`side_farm_browser_validation.json`). `side_farm_second_grip.png`에서 최신 V6/중앙 카메라/기존 토마토 송이/컨베이어를 확인했다. QA 서버 Chromium은 SwiftShader로 느렸고 여러 초기 캡처 timeout·1회 launch SIGSEGV를 기록했다. 작은 QA viewport/DPR와 실제 입력값 기반 옵션 선택으로 최종 검사 통과. 이는 사용자 브라우저 설정을 바꾸지 않는다. 초기 overview는 로딩 중 밝게 보이는 프레임이며 실제 수확부 확인에는 second_grip 캡처를 사용한다. 마지막은1배속/처음부터 재생으로 둔다. 최종 관련26회귀와 compile/bash/diff 검사 통과.

## 2026-11-13 작업 저장

- 사용자 요청으로 미커밋 ROS2·OMPL 연동과 5906 V1~V6 형상/컨베이어/도달 검사/중앙 카메라 장착 작업을 함께 저장한다. Python 회귀68개, ROS2 메시지·플래너 회귀8개 및 쉘 문법/diff 공백 검사 통과.
- V6 카메라 덮개 홈의 boolean 연산에 필요한 `manifold3d==3.5.4`를 기존 Newton viewer 설치 목록에 추가했다. 현재 검증 환경과 같은 버전이며 실행 중인 5906 서비스는 유지한다.

## 2026-11-13 최신: V6 카메라 안쪽 삽입 및 승인된 덮개 홈

- 사용자가 카메라를 표시한 중앙 브래킷 쪽으로 최대한 넣도록 요청했다. 현재 덮개를 유지하면 약13.3mm만 더 들어갈 수 있었고, 목표 위치에서는 닫히는 윗덮개와 겹쳤다. 사용자가 명시적으로 **카메라 자리만큼 덮개에 홈을 내고 안쪽으로 장착**을 선택했다. V6/5906에서만 적용하며 원본 CAD/생산 물리/ROS/5905/V1~V5는 유지한다.
- `preview_gripper_camera.py`: 원래 카메라 크기/세로 설치/하향15도를 유지하고 Tool_fixed Y축으로67.5mm 후퇴, Z축으로18mm 상승했다. 하우징 AABB 하단 Y=-27.5mm/Z=78mm. 상승은 이동 경로에서 카메라가 수확 중심보다10mm 이상 위에 머물도록 하기 위함이다. 장착대 길이는92→22mm로 줄였다. mount schema `v6-gripper-recessed-portrait-rgbd-v2`, posture policy `v6-recessed-camera-no-yaw-turn-v5`.
- 두 윗덮개 g0307/g0428에만 원본 복사본의 manifold boolean 차집합을 적용한다. 완전히 닫힌 Tool 기준 홈 X±16mm/Y[-30,24]mm/Z[75,108]mm; 각 홈은 Tool_L/R의 실제 슬라이더와 함께 움직인다. 덮개 재료 제거 비율은 각각4.526%/3.813%, 남은 메시 watertight. `recessed_camera_cover_geometry.json`: 남은 CAD 정점+면 중심3260개가 보수적 거리 검사 상자들에 모두 포함됨을 확인했다. V1~V5에서는 손대지 않은 원본 덮개/카메라가 표시된다.
- CameraClearance는 해당 두 덮개와 카메라 사이의 거리만 홈을 제외한 보수적 상자7개로 검사한다. 기존 덮개 native collider/mask는 다른 팔·환경 충돌 검사에 그대로 남겨 보수성을 유지한다. 거리 질의용 비활성 두 geom 슬롯만 사용하며 실제 접촉/질량/BVH는 바꾸지 않는다. 같은 고정 Tool에 붙은 두 X 슬라이더임을 확인한 후 정확한 턱 좌표를 키로 거리 결과를 캐시한다(반올림 없음). 덮개가 없어졌다고 가정하거나 카메라 충돌을 통째로 제외하지 않는다.
- 회귀40개 및 컴파일/diff 공백 검사 통과. 새 회귀는 홈 밖의 재료 보존과 턱이 움직여 홈 가장자리가 카메라에 닿을 때 거절되는지를 확인한다. `camera_recess_scan*`/`camera_recess_candidate_checks*`는 설계 탐색 자료이고, 원래 source 설치 FK로 얻은 높이는 실제 V6 검증값이 아니다. 최종 검증에는 V6 설치 패치를 사용한다.

- `side_motion_recessed_camera_v1`: 기존2765자세/798선분/146cycle(오른쪽67/왼쪽79)을 유지하고182365개 정밀 표본 통과. trajectory SHA는 이전과 동일. 카메라 하단-수확 중심 최소14.014202mm, 하우징/남은 덮개 등과의 최소 간격2.5mm. yaw 한 바퀴 금지/기둥100mm/기존 관절범위 유지. `recessed_camera_recertification_v1.log` 참조. 도달 범위를 새로 탐색한 것이 아니라 기존 전체 동작을 새 장착에 재검사한 것이다.
- 5906 새 실행 PID375288, tmux `farmily_preview:viewer5906`, output `preview_recessed_camera_20261113_141456`, plan `side_motion_recessed_camera_v1/grip_motion.json`. 자동재생을 빼 장착 모습을 정적으로 확인할 수 있게 시작한다. process/launch/start_5906_latest.sh 및 hanging_preview/process.json 갱신. 이전 기록은 *_before_recessed_camera_*에 보존했다.

- 새 viewer의 fresh 경로 재검사 통과. `recessed_camera_preview_validation.json`/`recessed_camera_live_samples.json`: 실제 양쪽 진입·닫기·열기·후퇴, 광학 extrinsic/리프트·어댑터 추종 및30개 재생 표본 검사. live 하우징 간격2.5mm/카메라 높이29.257041mm/기둥104.519893mm, 원본 자산 SHA 동일. Newton 동역학이나 실제 제작 강성 검사는 수행하지 않았다.

- 브라우저 HTTP200/JS오류0/새 접속 정지 유지 통과(`recessed_camera_browser_validation.json`). 실제 정면/측면 렌더 `recessed_camera_mount.png`, `recessed_camera_mount_side.png`를 열어 카메라 후퇴와 중앙 장착을 확인했다. 첫 GUI QA는 CSS value 속성 선택자가 실제 입력값을 찾지 못해 timeout; 시점의 접근성 label/option으로 선택자를 고쳐 재검사 통과(제품 코드 변경 불필요). 최종 V6 시작 자세/일시정지/검사지점 숨김/1배속. 사용자 브라우저의 시점/마우스 orbit은 변경하지 않았다. 새 커밋/푸시는 하지 않았다.

## 2026-11-13 최신: V6 리얼센스 중앙 브래킷 이동

- 사용자 캡처의 파란 세로 박스: 손목 아래, 두 롤러 사이 중앙 브래킷에 기존 리얼센스 CAD를 옮긴다. 5906 V6만 적용. `preview_gripper_camera.py`가 원래 25×25×90mm 하우징을 재사용하고 기존 손목 장착대를 V6에서 숨긴다. 카메라는 Tool_fixed를 따라 실제 wrist3 yaw와 함께 회전한다. 세로 설치/아래15도 유지, 짧은 장착대는 가정 치수다. V1~V5 선택 시 원래 Robot_link5 카메라/장착대를 복원한다.
- 새 mount schema `v6-gripper-central-portrait-rgbd-v1`. Tool_fixed 기준 하우징 보수적 AABB X±12.5mm, Y40~87.473mm, Z60~153.335mm. 원래 광학 보정으로 CAD를 optical frame으로 변환해 새 장착 변환을 구한다. preview/runtime JSON에 `active_camera_body`, `world_from_camera_optical`를 기록한다. 원본 CAD/입력 MJB/initial/생산 RGB-D 보정 및 ROS/5905는 변경하지 않았다.
- 요청한 위치는 수확영역 상단과 일부 겹치므로 카메라 하단 높이 기준을 **수확영역 중심+10mm**로 바꿨고 사용자에게 설명했다(기존 전체영역 최상단+10mm와 다름). `v6-central-camera-no-yaw-turn-v4` 정책에 mount 전체를 포함해 구 mount 경로의 무검증 재사용을 차단한다. no-yaw-turn, wrist3±170도, 기둥100mm, elbow20~140도, wrist1±150도, 전완 상향10도 조건 유지.
- 카메라는 원본 물리 모델에 충돌 geom이 없는 시각 부품이었다. `CameraClearance`가 private native MuJoCo 비활성 geom 슬롯을 OBB 거리 질의에만 사용한다. contact mask/body/BVH를 바꾸지 않고 같은 fixed body의 부품도 명시적으로 검사하며 활성 팔/그리퍼/보관함·차체 장애물과2mm 이상 여유를 요구한다. 실제 카메라 질량이나 Newton 동역학을 추가한 것은 아니다. 장착대의 강성·시야 가림·실제 RGB-D 프레임은 검증 범위 밖이다.
- `side_motion_central_camera_v2`는 기존 v3의2765저장 자세/798선분/146수확 cycle(오른쪽67/왼쪽79)을 그대로 재검사했다. trajectory SHA `99937c34d6c83e361c042b367a84af768c998cadec17f5a1014f87aa20e71aa2` 동일. 정밀182365표본 통과, 카메라 높이 최소11.104868mm/하우징 간격5.499998mm, world yaw 약0~180도/손목3최대90.027971도. `central_camera_recertification_v2.log` 참조. 중간 v1는 카메라 거리 검사 추가 전에 중단했으므로 배포하지 않는다.
- 관련38회귀(카메라 같은-body 간섭/Tool yaw 추종/legacy optical 유지/새 높이 기준 포함), 파이썬 컴파일 및 diff 공백 검사 통과. `시점 → 카메라 장착부`는 선택 시 한 번만 확대하며 기존 마우스 orbit을 추적 덮어쓰기 하지 않는다.
- 5906 새 실행 PID350300, tmux `farmily_preview:viewer5906`, `preview_central_camera_20261113_135012`, `--side-plan .../side_motion_central_camera_v2/grip_motion.json`. process/launch/start_5906_latest.sh 및 hanging_preview/process.json 갱신, 이전 실행은 *_before_central_camera_*에 보존. 배포 후 전체 fresh 경로 검사와 실제 GUI 점검 결과는 아래에 기록한다.

- 배포 viewer fresh 경로 검사 통과. `central_camera_preview_validation.json`/`central_camera_live_samples.json`: HTTP200, 양쪽 낮은 진입·닫기·열기·후퇴와 실제30프레임 검사. 광학 world transform은 Tool FK×새 optical extrinsic과 일치, 리프트.417325~.75m 추종. live 카메라 높이최소13.668584mm/하우징간격5.500759mm/기둥104.519893mm, source hash동일. 이 표본 최소값과 전체 경로 최소값은 구별한다.

- `central_camera_browser_validation.json`: 실제 브라우저 HTTP200/JS오류0/새 접속의 일시정지 유지. `central_camera_mount.png`에서 기존 카메라가 손목 위가 아닌 두 롤러 사이 중앙 위쪽에 있는 것을 확대 확인했다. 첫 headless Chromium 실행은 페이지 접속 전 SIGSEGV로 종료되었고 재실행은 정상 통과했다. QA 브라우저만 기존 SwiftShader 설정을 사용했다. 최종 상태는 V6 시작 자세/일시정지/검사지점 숨김/1배속으로 장착 위치를 보기 쉽게 두었으며, 사용자 브라우저는 `시점 → 카메라 장착부`로 확대 가능하다. 커밋/푸시는 하지 않았다.

## 2026-11-13 최신: V6 그리퍼 yaw 한 바퀴 회전 차단 완료

- 사용자 추가 조건: 그리퍼가 yaw 방향으로 한 바퀴 돌아서는 안 된다. 5906 V6에만 적용했다. `side_yaw_diagnostic/diagnostic.json`으로 기존 v2 저장 경로에서 실제 Tool_fixed +Y 세계 yaw가 약360도 도는 연결4개를 확인했다. 손목3 관절만 제한해도 해결되지 않는 결합 회전이었다(일부 wrist3는-270도 동등회전 분기도 사용).
- `preview_posture.side_posture_policy`를 `v6-camera-up-no-yaw-turn-v3`로 변경: world heading -30~210도(오른쪽0/시작90/왼쪽180), forward 수평 투영>=.25, 실제 wrist3 범위±170도. 금지된 뒤쪽 sector를 넘어 한 바퀴 돌거나 수직 방향에서 yaw 정의를 우회할 수 없다. 표시각만 wrapping하지 않으며 모든 IK/OMPL 상태·shortcut·저장 선분·역재생·fresh 로딩에 적용한다. 카메라10mm상단/기둥100mm/엘보우20~140/wrist1±150/전완상향10/진입15도 조건은 그대로다. 기존 V5 정책/원본 모델·CAD/5905/ROS는 유지했다.
- 이전 경로는 재생하지 않고 `side_motion_v3`를 새로 생성했다. v2 좌표는 IK 씨앗으로만 사용하고 새 working bounds 안의 동등 회전을 고른 뒤 모든 경로를 새 검사한다. 실제 재생 각도를 순간 정규화하지 않는다. 회전 동등값이 범위에 없으면 명시적으로 거절한다. viewer에 'yaw 한 바퀴 회전 금지 · 이동 경로 전체 검사'를 표시한다.
- 새450점 중 local164개, 완전연결146개(오른쪽67/왼쪽79; 이전152개보다6개 감소). 양쪽55cm 낮은 지점은 유지했다. 경로 독립 collision/posture 정밀 표본 182,365개 통과. 카메라 최소 119.3006mm, wrist3 절대최대 90.027971도, 최대 진입편차 14.001576도, 최대 표본 관통 0.876900mm. 제한된 탐색 실패는 기하학적으로 불가능하다는 증명이 아니다.
- 별도 `side_yaw_path_validation.json`: 같은 저장 선분 182,365표본의 native FK를 다시 계산하고 unwrap한 실제 tool yaw를 검사. 정·역 전체 yaw 범위 [-0.0005068361113377491, 180.00029513316917], 한 구간 최대 yaw 폭 90.000017도,360도 구간0, 정역 순회 net0. `side_yaw_summary.json`에 높이별/기존비교/전체 제약 결과 기록.
- 35개 회귀 통과: endpoint는 같지만 내부에서360도 도는 경로 차단, 좌우180도 전환/역방향 허용, 수직yaw우회 차단, wrist3±360 alias명령 거절, yaw검사 없는 구V6계획 재계획 요구. GUI 파이썬 컴파일 및 diff 공백 검사 통과.
- 현재5906 PID325455, tmux `farmily_preview:viewer5906`, 출력 `preview_side_yaw_20261113_132539`, `--side-plan .../side_motion_v3/grip_motion.json --autoplay`. process/launch/start_5906_latest.sh 및 hanging_preview/process.json 갱신, 이전 기록 *_before_side_yaw_*에 보존. 새 viewer 로딩에서 전체 선분을 다시 검사했다.
- 실제 GUI `side_yaw_preview_validation.json`/`side_yaw_live_samples.json`: 양쪽 낮은 진입·닫기·열기·후퇴/리프트·카메라·기둥·yaw·실제손목각 검사, 29개 재생 표본. 브라우저 HTTP200/JS오류0/새 접속 일시정지 보존, `side_yaw_lower_pose.png` 화면 확인. 마지막은 검사지점 표시 켬/1배속/처음부터 재생. 새 Newton 동역학·토마토 접촉력·실기 검증이나 커밋/푸시는 하지 않았다.

## 2026-11-13 최신: V6 카메라 상단 유지와 하단 도달 개선 완료

- 사용자 요청: 카메라가 그리퍼 아래로 내려가는 자세 금지, 리프트와 조합해 표시 영역 하단 도달 개선. 5906 V6만 변경했다. V1~V5/5905/ROS/원본 CAD·물리 자산은 유지한다. V6 리프트 모터/상판은 이전처럼 숨김이다.
- `preview_posture.side_posture_policy(scene)`는 실제 link5 카메라 하우징 CAD `s300_g0092`의 보수적 body AABB를 기록한다. 카메라 최하단이 수확 영역 최상단보다10mm 이상 높아야 하며 모든 저장·보간·진입·턱 동작·후퇴·연결 상태에 적용한다. 기존 기둥100mm/엘보우20~140도/wrist1±150도/전완상향10도 조건 유지. V5 POSTURE_POLICY는 그대로이고 V6는 `v6-camera-above-grip-and-mast-clearance-v2`로 이전 V6 계획의 재사용을 거절한다. 이전80개 cycle 중18개는 카메라가 아래인 자세였다(`side_camera_diagnostic/camera_diagnostic.json`).
- 기존 chassis mesh g0045/g0050/g0052는 떨어진 휠·데크·콘솔 부품들을 합친 convex hull이라 빈 공간까지 막았다. `preview_side_collision.py`: V6 private collision 복사본에서만 연결된 CAD 부품별 보수적 AABB로 대체한다. 복잡한 부품은 최소 추가 체적 병합으로64박스 이하. L자 리프트 받침2개는25mm 높이 단위로 triangle edge를 clipping해 빈 모서리 공간을 보존한다. 모든 박스에1mm 여유를 더하며 CAD를 삭제·축소하지 않는다. V6 장애물162개/native slots224개. 새 박스가 원래 CAD의666,303개 정점+면 중심을 모두 포함함을 `side_camera_diagnostic/cad_coverage_validation.json`에 확인했다.
- `preview_side_search.py`: 같은 엘보우 부호에서 실제 리프트0~.75m와 진입 pitch/yaw0/±14도를 명시적으로 탐색하고 전체 local cycle까지 검사한다. 기존350점에60/65cm 높이100점을 추가해450점 검사. `side_motion_v2/grip_motion.json`: local153개, 연결포함152개(오른쪽73/왼쪽79), 동일 기존350점에서는80→116개. 지면 기준 수확 중심55cm에서 양쪽3개씩,60cm 양쪽8개씩,65cm 양쪽10개씩 통과.40cm 구간은 통과 경로를 못 찾음(보관함 벽/롤러/기둥 간섭 등). 유한 후보 탐색이며 빨강은 기하학적 불가능의 증명이 아니다.
- 최종203,480개 정밀 표본 감사 통과. 카메라 최소 높이 여유10.2325mm, 기둥100mm 조건, 전완상향 최대9.904413도, elbow47.8997~139.9184도, wrist1최대149.79572도, 최대 관통.8769mm(1mm 기준 미만), 진입각14.001576도, 리프트.30~.75m. 전체 생성581.636초. 보고서 `side_camera_summary.json`; 원본 모델/initial/CAD SHA 동일, 기존 V5 collision patch SHA 동일. 관련30개 회귀 통과.
- 현재5906 PID293154, tmux `farmily_preview:viewer5906`, 출력 `preview_side_camera_20261113_125258`. `process.json`/`launch_command.json`/`start_5906_latest.sh`와 hanging_preview/process.json 갱신. `--side-plan .../side_motion_v2/grip_motion.json --autoplay` 실행. 이전 실행 기록은 *_before_side_camera_20261113_125258.*에 보존. 로딩 시에도203,480개 상태를 새로 검사해 output/side_validation/loaded_plan_validation.json 기록.
- 실제 GUI의 '오른쪽 아래 보기'/'왼쪽 아래 보기' 버튼을 추가했다. `side_camera_preview_validation.json`/`side_camera_live_samples.json`: 양쪽 낮은 목표에서 진입·닫기·열기·후퇴, 정지·홈·재생 및29개 실제 재생 표본을 독립 native FK/geomDistance로 검사. 카메라 여유 최소130.3675mm, 기둥 최소104.5199mm, 실제 리프트.417325~.75m; 이는 live29개 값이고 전체 경로 경계값은 앞 항목과 다르다. 어댑터 위치 검사는 Viser position setter의 np.allclose(rtol1e-5/atol1e-8) 생략 허용오차를 사용한다. 처음1e-10으로 검사했을 때7.4µm 표시 갱신 생략 때문에 QA가 실패했으나 코드의 알려진 표시 허용오차로 확인했다.
- 브라우저 `side_camera_browser_validation.json`, 스크린샷 `side_camera_lower_pose.png`: HTTP200/JS오류0, 왼쪽 낮은 close 자세 표시. 첫 브라우저 QA는 실행 상태가 바뀌어 정지 보존 assertion에 실패했으며 원인을 단정하지 않는다. 재검사에서는 새 브라우저 정지 보존/낮은 자세 표시를 통과했다. headless QA만 소프트웨어 렌더링, 사용자 GPU 설정은 그대로다. 마지막은 검사지점 켬/1배속/처음부터 재생이다.
- 재생은 저장된 `--side-plan` 사용. 같은 탐색을 다시 만들 때 `--side-workspace --side-seed-coverage .../side_motion_v1/grip_coverage.json`으로 이전 결과를 IK 씨앗으로만 쓸 수 있다(목표와 모든 경로 새 검사). 실제 실행한 오프라인 생성 스크립트 `/tmp/plan_side_motion_v2.py`. 새 GPU 동역학·접촉력·실제 과실 파지 검증은 하지 않았다. 새 커밋/푸시는 하지 않았다.

## 2026-11-13 최신: V6 모터 제거 및 양쪽 수확 모션

- 사용자 요청: V6의 리프트 모터를 일단 제거하고 모션을 다시 테스트한다. 이전 턴의 정지 형상만 표시 제약은 이번 요청으로 해제됐다. 5906만 변경한다. V1~V5 형상/기존 V5 계획/5905·ROS·production·원본 CAD는 유지했다.
- `preview_robot_mount.py`: V6에서만 모터와 수평 상판을 숨긴다. 형상용 q가 native chassis hull과 겹쳐 시작 q를 [-60,0,60,-60,90,0]deg로 바꿨다. 이전의 기둥10cm 조건을 충족하도록 어댑터 두께12→20mm(8mm 증가)로 조정했다. `--side-workspace`는 새350점 검사, `--side-plan`은 V6 전용 계획 로딩/fresh 감사 및 수확/턱/리프트 재생이다. 원래 반대편 경로를 재활용하지 않았다. 어댑터는 매 프레임 리프트를 따라가고 시작 자세 보기/양쪽 구간 선택/정지/재개를 지원한다. V6 기본 카메라는 기존 보관함 사선 시점, 세계Z up 유지.
- `preview_workspace_motion.collision_model(...,layout_key='side')`: private 복사본에서 제거된 수평 상판의 native collision proxy도 끈다. `preview_grip_workspace.layout_boxes(...,layout_key='side')`: 모터 AABB를 빼고 실제 이동 캐리지와20mm 어댑터 AABB를 리프트 추종 장애물로 추가한다. side도 기존 POSTURE_POLICY(기둥.1m,엘보우20–140,wrist1±150,forearm상향≤10)를 적용한다. 생성·독립 저장 경로 감사·fresh 로딩 모두 같은 설치와 장애물 구성을 사용한다.
- 최종 `side_motion_v1`: 같은 로봇 양옆 영역350점 중 자세118개(오른쪽49/왼쪽69), 연결→40mm 진입→닫기→열기→후퇴 전체 cycle80개(오른쪽34/왼쪽46). **132,467개 정밀 표본** 통과, 최대 표본 관통.888666mm(기존1mm 미만), 진입 방향14.000109도, 리프트.342765~.75m, elbow24.1355~139.9175도/wrist1최대140.0886도/forearm상향최대6.81693도. 전체 생성583.756초. FK 표시/충돌 구성 오차4.44e−16. 유한 탐색에서 찾은 통과점이며 나머지 점의 기하학적 불가능/영역 전체 보장은 아니다.
- 관련16개 테스트 통과. 새 회귀1개는 원래 상판과 접촉하던 native probe가 side 복사본에서만 더 이상 접촉하지 않고 원본/기존 버전에서는 그대로 접촉함을 확인한다. `side_motion_legacy_patch_check/validation.json`에서 기존 V5 충돌 패치 SHA가 정확히 같았다. `side_motion_probe/probe.json`에는 최초 형상용 q 충돌도 기록했고 safe_start.json은 새 시작 자세 탐색 기록이다. 소스 원본 물리/초기 상태/packed CAD SHA도 그대로다.
- 현재5906 PID260832, tmux `farmily_preview:viewer5906`, 출력 `preview_side_motion_20261113_122110`. `process.json`/`launch_command.json`/`start_5906_latest.sh`와 기존 hanging_preview/process.json 갱신. --layout side --side-plan .../side_motion_v1/grip_motion.json --autoplay로 실행한다. 기존 실행 기록은 *_before_side_motion_20261113_122110.*에 보존했다. 최종 로딩에서도132,467개 표본을 다시 검사했고 output/side_validation/loaded_plan_validation.json에 통과 기록이 있다.
- `side_motion_preview_validation.json`/`side_motion_live_samples.json`: 실제 WebSocket GUI 양쪽 진입/닫기/열기/후퇴, 턱 이동, 정지/재개, 초기 자세/재생을 확인했다. 91개 실제 재생 표본의 native geomDistance로 기둥 최소.104519893m, forearm상향 최대−46.63075도를 별도로 측정했다. 이91개는 양쪽 첫 cycle이므로 리프트.75m 고정이다. 리프트 추종은 별도로 `side_motion_lift_validation.json`에서2배속 재생 중.75→.717454101m 이동한 뒤 정지하여 **실제 송신 Scene SetPosition/Visibility 메시지**의 캐리지·베이스·어댑터가 FK와1e−9m 이내로 맞고 모터·상판이 숨김임을 확인했다.
- `side_motion_summary.json`은 수치 요약이다. `/tmp/verify_side_motion_browser.py`에서 최종 실제 브라우저 HTTP200/JS오류0/정지 보존과 왼쪽 close 화면을 확인했고 `side_motion_left_pose.png`/`side_motion_browser_validation.json`에 남긴다. 검사 후 검사지점 켬·1배속·처음부터 재생으로 남긴다. headless QA 브라우저만 SwiftShader를 사용한다. 새 GPU 동역학/실제 토마토 접촉력 시험은 하지 않았다. 이번 결과는 native CPU FK/IK/OMPL·표본 간섭 검사 및 관절 재생이다. 커밋은 요청되지 않아 수행하지 않았다.

## 2026-11-13 최신: V6 리프트 옆면 장착 정지 형상

- 사용자 최신 요청: 다음 버전은 수평 리프트 상판을 제거하고 로봇 베이스를 리프트 옆면에 붙인다. **이번에는 로봇 형상만, 모션을 실행하거나 새 경로/도달/간섭을 검증하지 않는다.** 기존 버전은 보존하며 5906 미리보기만 변경한다.
- `preview_robot_mount.py`에 `side`/`V6 · 리프트 옆면 장착`을 추가했다. 이동 캐리지 CAD 경계를 직접 측정해 열린 방향(world −Y)의 측면에 가정 어댑터(폭200×높이220×두께12mm)를 붙인다. 베이스의 원래 밑면을 세계−Y로90도 돌려 어댑터 외측에 정확히 맞췄다. V6에서만 원래 상판 메시를 숨기며, 예전 상판 높이/설치방향 선택은 숨긴다. 캐리지/모터/베이스는 같은 리프트 기준으로 배치되며 모터는 어댑터 아래15mm에 붙는다. 카메라/90도 툴 브래킷/그리퍼, 원본 CAD 색상 및 20cm 높이 밀착 컨베이어 보관함은 유지한다.
- 정지 형상용 임의 q(deg)=[−90,30,60,−90,90,0], 리프트 설정.75m, 베이스 장착면 중심의 지면높이1.396215642m. 이 값은 형상을 보기 위한 배치이지 도달/충돌/기존 V5 정책 검증 결과가 아니다. 새 모션·IK·OMPL·GPU 물리 시험을 실행하지 않았다. 검사점/자동재생/배출 재생 없이 정지 화면이다.
- 현재 5906 PID232791, tmux `farmily_preview:viewer5906`, 출력 `/root/farmily_runtime/newton/runs/20261113_conveyor_bin/preview_side_mount_20261113_115316`. runtime의 `process.json`/`launch_command.json`/`start_5906_latest.sh`, 기존 hanging_preview/process.json도 갱신했다. 이번 실행에는 workspace-motion/계획 파일/배출 파일/autoplay 플래그가 없다. **V1~V5도 정지 형상 비교로 선택 가능**하며 예전의 실제 V5 모션은 저장된 실행 인자/계획을 사용해 복구할 수 있다. 이전 전체 모션 실행 인자와 스크립트는 `*_before_side_mount_20261113_115056.*`, 중간 정지 실행은 같은115316 접미사로 보존했다. 5905/ROS/production 자산은 수정하지 않았다.
- `side_mount_layout_validation.json`: V1~V6 실제 WebSocket GUI 전환에서 상판/어댑터/보관함 표시가 맞고 V6로 복귀했다. 원본 물리/초기 상태/packed CAD 해시는 이전과 같으며 이전 V5 시작 q 및 실제 리프트.673075m의 베이스/팔/캐리지/상판/모터/기둥 변환은 오차0이다. 이전 preview.json의 lift_height_m은 슬라이더.75로 기록되지만 실제 motion 시작은 parked_pose.json의 reference_lift_m을 사용하므로 비교에도 후자를 사용했다. 단순 정지 CAD 부착면/축 일치를 확인했으며 충돌 검사는 하지 않았다.
- `side_mount_static_validation.json`: 최종 PID/HTTP200, side mount, 상판숨김/어댑터표시/컨베이어표시, 모션 플래그 없음, running=false/physics_steps=0 확인. 브라우저 렌더링은 `side_mount_v6.png`, `side_mount_browser_validation.json`에 보존한다. headless QA 브라우저만 SwiftShader를 사용한다. py_compile 및 git diff --check 통과, 새로운 테스트/커밋은 추가하지 않았다.

## 2026-11-13 최신: 작업영역을 넓히는 V5 시작 자세 비교

- 사용자가 빨간 작업영역을 최대한 커버하는 시작 자세를 요청했다. 형상/보관함/기둥을 옮기거나 충돌 조건을 풀지 않고, 같은350개 작업점에서 초기 추정·리프트·엘보우 가지·연결 경로를 비교했다. V5의 기존 기둥100mm, 엘보우20~140도, wrist1 ±150도, forearm 위쪽10도 제한은 유지했다. 목표 진입 방향은 시작 자세와 분리한 세계 좌표 회전으로 저장하고 yaw/기울기를 합친 진입 방향도15도 원뿔 안으로 검사한다. 그리퍼 접근축 둘레0/180도 roll은 기존처럼 후보이며 기구학/간섭 검사이지 파지력·실기 검증은 아니다.
- `preview_start_search.py`: 실제 리프트0~.75m의 여러 높이와 양쪽 엘보우에서 native Jacobian IK를 수행하고,40mm 진입·닫기·열기·후퇴가 통과한 국소 후보만 그래프에 넣는다. 그리퍼만의 목표 자세가 이미 보관함/기둥과 깊게 겹치는 경우 private MjData의 정확한 native 거리로 먼저 제거한다(6mm 여유를 두고 prune, 채택 후보는 전체 native 검사). 예: X=.495,Y=-.1,Z=.4m의 한 수평 자세는 Tool_fixed와 보관함 옆벽이58mm 겹쳤다. 이를 시작 자세로 해결됐다고 표시하지 않는다.
- 초기 `start_search_v2`는 같은350점/32,702개 IK probe에서 엘보우 가지별133개 국소 cycle을 확보했다. `--resume-candidates`는 저장된 후보로부터 국소 동작을 다시 검사하므로 전체 IK 그리드를 반복하지 않는다. ±360도 표현을 일관되게 정리하고 엘보우 가지마다 어깨 기준도 분리한다. 베이스의 다른 실제 회전 표현은 별도 노드로 유지하며, 각 연결은 실제 중간 상태를 전부 검사한다. coarse 검사는 검색용이고 채택 연결은0.002rad/리프트0.5mm 표본으로 다시 검사한다. `require_continuous_segments`가2π 차이의 순간 점프도 거부한다.
- 최종 `start_search_v4`는 관절 회전 표현 변형을 포함한235개 시작 구성 후보를 비교했다. 서로 연결된 고유 작업점 수를 우선하고, 동률에서는 양쪽 균형과 가장 긴 연결 경로 시간을 비교한다. **오른쪽27/175·왼쪽24/175, 총51곳**으로 기존32곳을 모두 유지하고19곳을 더 확보(+59.375%). 반대 엘보우의 최선 연결 후보는50곳(오른쪽26·왼쪽24)이므로51곳 쪽을 선택했다. 유한 그리드/시간 제한 탐색의 최선 후보이며 전역 최적이나 연속 부피 전체의 도달 증명이 아니다.
- 선택 시작 리프트=.673075m(슬라이더 설정67.3cm, 지면 기준 TCP 높이가 아님), 팔 각도[base,shoulder,elbow,wrist1,wrist2,wrist3]도=[-26.3133,118.5378,-100.3915,-108.1481,90.0001,-26.3130]. 같은 새 연결 그래프에서는 기존 시작도51곳에 연결되며, 선택 시작은 최장 연결 시간 추정34.984→31.361sim초로 줄였다. 평균은17.612→17.825sim초로 약간 늘어, 모든 경로/실행속도가 빨라졌다는 주장은 하지 않는다. **32→51 개선은 탐색·연결 보완 효과를 포함하며 초기q 변경만으로 기구의 물리 도달 범위를 넓혔다고 해석하지 않는다.**
- `start_search_v4/grip_motion.json`/`grip_coverage.json`/`grip_trajectory.npz`/`parked_pose.json`/`start_candidates.json` 및 상위 `start_search_summary.json`. 독립58,505개 표본과 서버 fresh 재검사 통과, 최대 표본 관통.625172mm(기존1mm 미만), yaw14.000177도,3D 진입 방향14.041075도, 리프트.425~.75m, 엘보우84.952~139.764도, wrist1 최대124.642도, forearm pitch 최대-51.651도. 완료 목표의 높이는 오른쪽.55~1.0m/왼쪽.70~1.0m이며 영역 상하단 전부를 커버했다고 표시하지 않는다. 색상은 이전과 같은 초록=전체 연결 cycle, 파랑=자세 후보, 빨강=제한 탐색에서 해 못 찾음이다.
- `preview_grip_workspace.py`의 최종 export/audit를 공통 함수로 분리하고 목표 회전·3D 진입·구간 연속성도 저장/로딩 시 검사한다. `--conveyor-pose-file`의 reference_lift_m와 nominal_tool_rotation_world를 V5에 적용해 V1/V4 시작 설정을 바꾸지 않는다. GUI에32→51 및 시작 리프트를 표시하고 **선택한 시작 자세 보기** 버튼으로 초기 자세를 고정할 수 있다. GPU 배출 재생에서 돌아와도 같은 자세로 복귀한다.
- 현재5906 PID209690/tmux farmily_preview:viewer5906, 출력 `preview_start_search_20261113_113012`. 실행 기록은 conveyor_bin의process.json/launch_command.json 및 기존 hanging_preview/process.json, 실행 스크립트start_5906_latest.sh. 기존 실행 정보는*_before_start_search.json에 남겼다. 5905/ROS/production 모델/물리 백엔드와 원본 CAD 및 V1 전체 배열은 유지했다.
- 회귀46개 통과. `start_search_preview_validation.json`/`start_search_preview_samples.json`: HTTP200,58,505개 fresh 검사, 양쪽 GUI 진입·닫기·열기·후퇴/실제 턱 변화, 시작 버튼의 관절·리프트 초기값 일치, 정지·재개, 별도 GPU 기록120개 배출 및 배출→시작 자세 복귀를 확인했다. 재생149표본은 별도 native geomDistance로 재측정해 최소 기둥 간격0.401390m(이149표본에만 해당), 원본 CAD/V1 배열은 정확히 같았다. GUI 관측 첫 시도는2배속의.357wall초 진입이.5초 기록 사이에 빠졌다(`start_search_gui_sampling_failure.json`). 저속 재시도의 배출 시간 제한도짧아 기록했고(`start_search_preview_slow_unload_timeout.log`), 최종 검사는 그리핑.25배속/배출2배속으로 관측했다. 제품 궤적/조건을 바꿔 통과시킨 것이 아니다.
- 실제 브라우저 HTTP200/JS오류0, 초기 자세 고정과 양쪽 화면을 `selected_start_pose.png`/`start_search_right_pose.png`/`start_search_left_pose.png`로 확인했다(`start_search_browser_validation.json`). QA 브라우저만 기존 SwiftShader를 사용했고 사용자 그래픽 설정을 변경하지 않았다. `start_coverage_comparison.png`/`.svg`는 같은350개 점의 전후 완료 동작 수 비교다. GPU 물리 전체 시험/새 영상/커밋은 수행하지 않았다. 중간start_search_v1~v3 기록은 보존하며 배포는v4만 사용한다.

- 시작 자세 고정 후 두 번째 창이 연결되면 on_connect의 중복 autoplay가 다시 재생시키는 문제를 재현했다(`start_home_connection_before_fix.json`). 초기 autoplay는 이미 select_layout에서 설정하므로 접속 때 강제로 재개하던 코드만 제거했다. 최종 실행의 두 WebSocket 클라이언트와 실제 새 브라우저에서 정지/elapsed=0 유지 확인(`start_home_connection_validation.json`, `start_home_browser_validation.json`, JS오류0). 최종 화면은 시작 리프트.673075m/왼쪽 시작 자세/elapsed=0/일시정지로 남겼다. 수정 전 실행/검증은*_before_home_pause_fix.json 및 before_home_pause_fix_*에 보존했다.

## 2026-11-13 최신: V5 기둥 이격과 과도하게 접히는 관절 자세 제한

- 사용자가 캡처의 깊게 접힌 팔/손목을 위로 치켜드는 자세와 리프트 기둥에 가까이 붙는 자세를 금지했다. 5906 V5에만 작업 자세 제약을 추가하고 V1~V4의 이전 형상/기록과 5905/ROS/production 물리는 유지했다. 제조사 한계를 변경한 것이 아닌 시뮬레이션 작업 범위 가정이다.
- `preview_posture.py`의 정책 `v5-mast-clearance-and-open-elbow-v1`: 기존 팔/툴 native 충돌체와 보수적인 고정 기둥 CAD AABB 사이 최소100mm, 엘보우 절댓값20~140도, wrist1 -150~150도, 엘보우→첫 손목의 위쪽 기울기 최대10도. 기둥 proxy에 positive-distance contact margin/gap을 추가해 관통하지 않더라도 가까운 자세를 거부한다. 링크/툴 전체에 이격을 적용하며 부착된 고정 상판 자체를 이격 대상 팔로 오판하지 않는다. 실제 CAD 정밀 충돌/하드웨어 인증이 아닌 native proxy와 표본 경로 검사다.
- `GripPlanner`는 IK/7축 OMPL/연결/진입/닫기/열기/후퇴 및 독립 재검사 모두 같은 제약을 사용한다. FK로 금지된 위쪽 기울기를 먼저 거르고, IK의 리프트 여유축으로 엘보우가 위에 있도록 유도한다. 수렴했으나 충돌하는 같은 후보를 반복 검사하던 탐색을 조기 종료했다. 기존 unsafe 궤적을 단순 재생하지 않고 350개 작업점을 새로 풀었다. 이전 자세는 새 충돌/정책 검사에 통과해야만 초기 추정으로 재사용된다. V5 로더는 현재 정책이 없는 예전 계획을 거부하고 실제 저장 경로를 다시 검사한다.
- 최종 `/root/farmily_runtime/newton/runs/20261113_conveyor_bin/near_posture_v3`: 오른쪽25/175·왼쪽28/175 자세 해, 전체 그리핑 경로는 **오른쪽17·왼쪽15, 총32곳**. 이전49곳과 동일한 목표14곳의 자세/경로를 다시 풀었고, 기존에 전체 경로를 찾지 못했던18곳도 통과했다. 제한된 탐색 실패는 불가능의 증명이 아니다. 33,788개 독립 경로 표본 및 실행 시 fresh 재검사 모두 통과. 표본 최대 관통0, yaw14.000177도, 리프트.616372~.75m. 실제 경로 엘보우86.852~137.596도, wrist1 절댓값 최대120.088도, forearm 위쪽 기울기 최대-51.651도(전 구간 아래 방향), 기둥 최소100mm 기준 통과.
- 기존에156도 접힌 시작 자세를 V5에 재사용하지 않도록 `--conveyor-pose-file .../near_posture_v3/parked_pose.json`을 추가했다. V1/V4 `--pose-file`은 유지한다. 배출 기록을 볼 때도 새 V5 시작 자세로 고정한다. GUI 상태에 기둥10cm/자세 제약을 표시한다. 현재 tmux `farmily_preview:viewer5906`, PID35058, 출력 `preview_guarded_20261113_082330`. 실행 인자/프로세스는 conveyor_bin의 `launch_command.json`/`process.json`과 이전 hanging_preview의 process.json에 기록한다. `start_5906_latest.sh`도 새 계획/포즈를 사용한다.
- 관련40개 회귀 통과. 새6개는 비관통7cm 간격의 거부,10cm 초과 허용, 위로 치켜드는 팔,156도 엘보우/손목 뒤집힘의 거부, 기둥 없는 검사 실패, 예전 V5 계획 로딩 차단을 확인한다. `guarded_preview_validation.json`/`guarded_preview_samples.json`에서 HTTP200, 양쪽 실제 GUI의 이동·진입·닫기·열기·후퇴 및 턱 변화/정지·재개를 확인했다. 움직이는65표본을 독립 `mj_geomDistance`로 다시 측정해 최소.288727m(이 값은 이65표본에만 해당), pitch 최대-67.649도였다. 전체 경로의 이격 보장은 위100mm 표본 기준으로 구분한다. 원본 자산 해시/CAD 전체 배열/V1 전체 궤적 배열은 정확히 같고 별도 GPU 배출 기록120개 재생도 유지했다. GPU 물리 전체 시험을 다시 실행한 것은 아니다.
- 실제 브라우저에서 양쪽 정지 그리핑 자세를 렌더링해 `guarded_right_pose.png`/`guarded_left_pose.png`로 확인했다. `guarded_browser_validation.json`: HTTP200, JS오류0, 정책 문구 표시, 양쪽 화면 확인 후 V5 오른쪽 처음부터 재생으로 복귀했다. headless QA 브라우저만 이전과 같은 SwiftShader 소프트웨어 렌더링을 사용했으며 사용자 그래픽 설정/물리 백엔드는 바꾸지 않았다.
- 중간 실패/중단된 near_posture_v1/v2는 보존했고 배포하지 않는다. 기구학/간섭 검사이며 파지력·토마토 접촉이나 실기 동작을 새로 검증한 것은 아니다. 새 커밋/영상 저장은 하지 않았다.

## 2026-11-13 최신: 완료된 V5 뷰어 실행 복구

- 사용자 요청으로 이전 작업 완료 여부를 확인했다. V5 소스/형상/경로/GPU 배출 검증 결과는 모두 저장되어 있었다. 이전 PID555121은 없었고5906 포트도 닫혀 있었다. 이전 뷰어 로그에 traceback은 없으며 현재 컨테이너 PID1 시작 시각이 이전 실행 이후다. 종료 원인을 별도의 로그 없이 단정하지 않는다. 제품 기능의 미완료가 아니라 실행 프로세스 부재를 복구했다.
- 검증된 launch_command.json의 옵션과 원래 자산/포즈/V1·V4·V5 궤적/배출 기록을 재사용한다. 현재 실행 정보는 `/root/farmily_runtime/newton/runs/20261113_conveyor_bin/process.json`과 기존 hanging_preview/process.json에 있다. 출력은 `preview_recovered_20261113_080156`, 로그는 같은 상위폴더의 동명.log다. `tmux -L farmily_preview attach -t viewer5906`으로 접속할 수 있으며 터미널 연결과 분리해 실행한다. 컨테이너가 종료·재생성되는 경우까지 tmux가 유지해 주는 것은 아니다.
- 관련34개 회귀를 다시 실행해 모두 통과했다. 재시작 과정에서77,975개 경로 표본을 다시 native 충돌 검사했고, 원본 모델/초기상태/CAD 해시도 이전과 같았다. HTTP200, 양쪽 진입·닫기·열기·후퇴와 실제 턱 위치 변화, 정지·재개, GPU 시험 기록의 문/벨트 동작 및120개 배출, 그리핑 재생 복귀를 실제 GUI WebSocket 경로로 재확인했다. 결과 `viewer_recovery_validation.json`/`viewer_recovery_samples.json`/`recovery_validation.log`. 전체 GPU 물리 시험을 재실행한 것이 아니라 기존에 검증된 기록의 재생을 확인한 것이다.
- 현재5906은 `V5 · 밀착 컨베이어 보관함`의 그리핑 재생으로 복귀시켰다. 브라우저를 새로고침해 사용할 수 있다. 이번 복구에서는 시뮬레이션 소스나5905를 변경하지 않았다.

## 2026-11-13 최신: V5 틈 없는 20cm 컨베이어 보관함과 기체 바로 옆 그리핑 검사

- 사용자가 보관함 받침발/틈 제거, 전체 바닥 컨베이어·롤러, 뒤쪽 개방 배출문, 높이 약20cm와 기체 바로 옆 작업영역 검사를 요청했다. `V5 · 밀착 컨베이어 보관함` (`--layout conveyor`)을 추가하고 V1~V4는 유지했다. 현재5906 출력은 `/root/farmily_runtime/newton/runs/20261113_conveyor_bin/preview_conveyor_v2`; PID/실행 인자는 같은 상위폴더 및 이전 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/process.json`에 기록한다. 농장/5905/ROS/production 충돌 정책은 수정하지 않았다.
- `preview_conveyor_bin.py`의 공유 가정 형상: 외형560×1100×200mm, 하단Z=.314m/상단=.514m, 받침발0·차체 틈0. 원래 리프트의 앵커 발은 밀폐 하우징 내부에 남으며 새로운 보관함 받침발은 없다. 하부 구동부와 원래 앵커를 덮는 벨트 바닥Z=.400m이므로 **내부 유효 깊이는114mm**이며 외형200mm와 구분한다. 벨트516×1087mm/횡롤러31개/반경16mm, 출구는 컨트롤박스 반대쪽 세계Y-의 짧은 면이다. 위 힌지 문이 바깥쪽/위로110도 열리며 벨트 위112mm 구간을 개방한다. 닫힌 문은 벨트와2mm 이격하고 구동 하우징 아래쪽은 고정한다.
- 첫 배출 시험에서 출구 앞 고정 프레임의 비구동 턱 위에 마지막 열매6개가 남았다. 벨트 시작Y=-.455m로 고정 프레임 외곽-.450m보다5mm 돌출시키고 끝 롤러를 옮겼다. 닫힌 문이 벨트와 겹치지 않게 문 하단을 정리했다. 이 형상을 기준으로 팔/배출 시험을 다시 수행했다. 바닥·벽·리브·문·롤러 하부의 보수적 envelope만 팔 장애물이며 열린 보관함 내부를 통째로 막는 상자는 없다.
- `preview_grip_workspace.py`의 새 near 프로필은 그리퍼 수확영역 중심 기준 X[.30,.50]/[-.50,-.30], Y[-.15,.80], Z[.30,1.30]m다. 벽 본체에서 명목상2~22cm 옆이다. 각 면175개, 총350개를 검사했고 리프트+팔7축 IK/여러 엘보우 시드/그리퍼 접근축 둘레0·180도 roll/좌우±15도 제한을 사용한다. 실제 관절 한계와 기존1mm 충돌 기준은 넓히지 않았다. 이전 자세는 새 형상에서 매번 다시 검사하는 초기 추정으로만 재사용하며 이전 궤적을 새 형상의 검증으로 복사하지 않았다. 180도 roll을 포함한 캐시 자세의 방향 복원도 수정했다.
- 최종 `near_v3/grip_motion.json`, `grip_coverage.json`, `grip_trajectory.npz`: 오른쪽70/175·왼쪽71/175 자세 해, 전체 이동→40mm 진입→96mm에서36mm로 닫기→열기→후퇴는 **오른쪽23곳·왼쪽26곳, 총49곳**. 독립 경로77,975표본, 최대 관통.871155mm, 최대 방향14.000426도, 실제 리프트.27765~.75m. 완료 목표 높이는 오른쪽.70~1.28m/왼쪽.85~1.28m다. 낮은.40/.55m 표본은 해를 못 찾았다. 실패는 제한된 IK/2.5초 연결 탐색에서 해를 못 찾은 것이며 불가능의 증명은 아니다. 요약 `near_reach_summary.json`. 기구학/단순 충돌체 검사이며 토마토 접촉력·파지력·실기 검증은 아니다.
- `run_conveyor_unload.py`는 **별도의 보관함 단독 Newton → MuJoCo Warp CUDA 물리 시험**이다. GPU에 모터가 달린 문1축/롤러31축, 전체 벨트의 prescribed surface velocity, 반경18mm·20g 강체 구120개를 사용한다. 원래 로봇 전체/실제 송이/과육 손상·적재 안정성 모델은 아니다. 최종 `unload_v7/unload_report.json`: Newton solver/접촉6차원/1ms, 문110도/벨트.25m/s, 벨트 끔4.000sim초에0개 배출, 켬6.439716sim초에120개 전부 출구 통과. 출구 이전 바닥 누수0, bin 접촉 최대.045753mm, 비유한 상태/버퍼 넘침/반복 경고0. 전체 최대 관통2.174694mm는 배출 후 지면/구끼리 접촉을 포함하며 bin의1mm 기준과 구분한다. 테스트 RTF는 off .0424/on .0260으로 느리며 실시간 가속 달성으로 보고하지 않는다. 이미 출구를 통과한 뒤 차체 아래로 되굴러온 구는 바닥 누수로 오판하지 않는다. 모두 배출된 시점에 시험을 끝내며 외부 더미의 장시간 안정화는 범위 밖이다. 이전 invalid/부분 실행 `unload_v1`~`v6`는 사용하지 않는다.
- 5906의 `GPU 배출 결과 보기`는 위 실제 GPU 상태 NPZ의 **기록 재생**이다. 그동안 팔은 초기 자세로 고정하며 robot dynamics를 실행했다고 표시하지 않는다. 문/롤러/벨트/구120개의 상태를 보간하고 실제 누적 출구 통과 수를 표시한다. `load_unload_record`는 backend/물리 valid/형상 일치/SHA/배열 모양·유한성·시각/배출 수를 확인해 실패 기록을 차단한다. 오른쪽/왼쪽 구간 버튼으로 그리핑 재생에 복귀한다. 배속/정지/재개/처음부터와 V1~V4 선택을 유지한다. 원본 약660mm 상판/매달림 설치/모터 연결/바퀴 재질은 유지한다.
- 실제 HTTP200/브라우저 JS오류0, 양쪽 턱 위치 변화와 진입·닫기·열기·후퇴, V1/V2/V4/V5 전환, 정지·재개, 문/벨트/120개 배출 재생을 확인했다. `conveyor_browser_validation.json`/`conveyor_browser_samples.json`, 렌더 `conveyor_preview_final.png`/`conveyor_floor_full.png`/`conveyor_door_open.png`/`conveyor_unloaded.png`. 이 컨테이너의 기본 headless Chrome은 WebGL 초기화에서 멈췄다. **QA 브라우저만** VK_ICD_FILENAMES와 VK_DRIVER_FILES를 Chromium의 vk_swiftshader_icd.json으로 지정하고 LD_LIBRARY_PATH/DISPLAY를 제거했다. 소프트웨어 렌더링이므로 동작은 실제 GUI와 같은 WebSocket 명령으로 검사하고 정지 장면을 캡처했다. 로봇/물리 백엔드를 CPU로 바꾸지 않았다. 사용자 노트북 브라우저의 그래픽 설정이나 서버 패키지는 변경하지 않았다.
- 최종 문구 수정 후 `preview_conveyor_v2`를 다시 실행하고 `conveyor_deployed_smoke.json`에서 HTTP200/JS오류0/120개 배출 재생과 그리핑 복귀를 재확인했다. 새 실제 렌더 `conveyor_preview_final.png`/`conveyor_discharge_front.png`도 확인했다. 전체 물리/궤적은 앞선 시험과 같으며 새 실행에서도77,975개 경로 표본과 원본 CAD/V1 배열 보존을 다시 확인했다.
- 관련34개 회귀 통과(새 보관함5개 포함). 원본 모델/초기상태/CAD 정점·면·정점색 및 V1 전체 궤적 배열이 직전과 정확히 같다(`conveyor_source_preservation.json`). 새 영상/커밋은 만들지 않았다. 실행은 기존 preview-hanging에 `--layout conveyor --conveyor-plan .../near_v3/grip_motion.json --unload-report .../unload_v7/unload_report.json`을 추가한다. 현재 실행은 이전 V1 workspace-motion/left/autoplay와 V4 storage-plan도 포함한다. 큰350점 계획/배출 물리 시험은 오프라인에서 끝냈으며 접속/구간 선택 시 다시 실행하지 않는다. 계획 로딩 시 native 경로 재검사는 수행한다.

## 2026-11-12 최신: V4 보관함 배치 양쪽 그리핑 영역 검사

- 사용자가 V4 형상을 채택하고 탑뷰의 양쪽 빨간 영역에서 그리핑을 확인하도록 요청했다. V4 기둥/원래660mm 상판/보관함/색상을 유지한다. 그림의 작업점을 X[.30,.78] 및[-.78,-.30], Y[-.15,.80]m로 해석하고 기존 시험 높이Z[.30,1.04]m를 사용한다. 좌표는 실제 그림을 측정한 값이 아닌 가정 범위이며 그리퍼 수확영역 중심 기준이다.
- `preview_grip_workspace.py`는 기존 `collision_model`/native FK·Jacobian IK/OMPL/검증된 경로 재생기를 재사용한다. 각 면60개, 총120개 표본의 자세를 여러 리프트 높이와 양쪽 엘보우 시드에서 탐색했다. 오른쪽26/60·왼쪽23/60에서 충돌 기준을 만족하는 자세를 찾았고, 그중 각8개 총16개 지점의70mm 진입→닫기→열기→후퇴와 연결 이동을 채택했다. 실패 표본은 제한된 탐색에서 해를 못 찾았다는 뜻이며 도달 불가능의 증명은 아니다. 표본 높이는.40/.65/.90/1.02m이고 영역 상단1.04m 자체는 유지한다.
- 새 장애물은 보관함의 열린 내부를 유지하는35개 바닥/벽/테두리/리브/발 상자, 리프트 원본CAD5개 부품의 보수적AABB, 상판을 따라 움직이는 모터AABB다. 숨겨진 식물의 기존 BVH가 있는 비활성 지오메트리 슬롯을 private model에서만 사용한다. native 원래 로봇/기체 충돌을 유지하면서 새 상자와 움직이는 팔·베이스·상판 간 접촉만 추가한다. 부착된 차체와 보관함 자체의 접촉은 팔 간섭으로 오판하지 않는다. 새 지오메트리의 AABB/BVH와 body mask cache도 갱신하며 모터는 실제 검사 리프트 높이에 맞춘다. 이 단순화된 리프트 외곽의 보수성을 정확한 CAD 메시 충돌 검증으로 표현하지 않는다.
- 초기 반대편 전환은 짧은 관절각 연결/단시간RRT로 통과하지 못했다. 기존 오른쪽 연결을 역순으로 따라 접은 자세로 복귀한 뒤 기체 앞쪽으로 베이스를+180도 돌리는 검사된 연결로 수정했다. 실제 한계 안에서 동등한2π 관절 회전을 선택하며 한계를 늘리지 않는다. 최초 독립 검사에서는 상단1.04m 목표가 IK의2mm 종료 오차로 영역 바깥에 측정되어 실패했다. 목표를1.02m 안쪽으로 옮기고 전체를 재계획했으며 판정 영역/1mm 기준은 넓히지 않았다. 실패 `storage_grip_scan_v1`~`v5`도 보존한다.
- 최종 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/storage_grip_scan_v6`: `grip_motion.json`/`grip_coverage.json`/`grip_trajectory.npz`. 전체29,493개 표본 재검사에서 최대 관통.544291mm(<1mm), 최대 팔 간격.001999787rad/리프트.5mm/턱.5mm, 진입·그리핑·후퇴 최대 방향오차.002247도, 리프트.50~.75m. 전체 전진 재생 약154.2초, 연결된 정·역 반복 약306.6초다. 동역학/접촉력/토마토 유지력/실기 검증은 하지 않았다. 주황색 구는18mm반경의 시연 표식이며 실제 열매 접촉·수확 성공을 주장하지 않는다. 턱은 열린96mm에서36mm 간격까지 움직인다.
- 현재5906은 `preview_storage_grip_v1`, 프로세스는 상위 `process.json` 참조. 실행은 기존 `preview-hanging`에 `--layout storage --storage-workspace` 또는 `--storage-plan .../storage_grip_scan_v6/grip_motion.json` 추가. 현재 실행은 기존 `lower_pose.json`/`--workspace-motion --workspace-side left --autoplay`도 지정해 V1 기존 동작을 보존한다. 계획 파일 로딩은 원본 모델/초기상태/CAD/배치 patch/장애물/파일SHA를 확인하고29,493개 표본을 다시 native 충돌 검사한 후 허용한다(`storage_validation/loaded_plan_validation.json`). V4 설치 제어는 검사 중 고정하며 `오른쪽 구간 보기`/`왼쪽 구간 보기`는 미리보기 재생 시각을 이동한다. 새 `양쪽 검사` 시점과 양쪽 빨간 영역·표본 점(초록=전체 동작/파랑=자세만/빨강=해 못 찾음)을 표시한다.
- 실제 브라우저HTTP200/JS오류0에서 양쪽 진입·닫기·열기·후퇴와 native 턱 위치 변화, 일시정지/구간 이동, V2 정적 전환/V1 기존 모션 재개/V4 복귀, 카메라 유지 확인. `storage_grip_browser_validation.json`/`storage_grip_browser_samples.json`, 렌더 `storage_grip_viewer_final.png`/`storage_grip_left_closing.png`/`storage_grip_right_closing.png`/`storage_grip_bilateral_top.png`. 원본 모델·초기상태·450개CAD/색상 해시, 보관함 형상, V1 전체 궤적 배열이 직전과 같다(`storage_grip_source_preservation.json`). 새 장애물 활성화/모터 이동/BVH/재생5개 포함 관련29회귀 통과. 새 동영상/커밋은 만들지 않았다. 농장/5905/production 물리 자산은 수정하지 않았다.

## 2026-11-12 최신: 5906 V4 기본 상판과 낮고 긴 토마토 보관함

- 사용자 최종 지시는 상판 길이를 정상화하고, 차체 위 가로로 긴 빨간 영역에 하나의 큰 토마토 보관함을 만드는 것이다. `V4 · 기본 상판 + 토마토 보관함` (`--layout storage`)을 추가했다. 기둥은 V2의 컨트롤박스 밀착 위치를 유지하고, 로봇암 장착 위치도 V2로 복원한다. V3의 .80m 추가 돌출 및 약.766m 연장판은 V4에서 사용하지 않으며 원래 CAD 상판 길이659.9996mm를 유지한다. 중간에 취소된 기둥 전방 이동은 적용하지 않았다. 기존V1/V2/V3 선택은 유지한다.
- 새로운 보관함은 실측 부품이 아닌 가정 치수 길이1100×폭560×높이280mm의 윗면이 열린 단일 통이다. 세계 좌표 외곽 X[-.28,.28]/Y[-.45,.65]/Z[.36,.64]m, 바닥18mm/벽12mm/상단 테두리와 보강 리브가 있다. 네 받침발이 차체 상면.314m에서 지지하며 보관함은 차체에 고정된다. 기존 리프트 발을 피해 바닥을46mm 올렸다. 앞쪽100mm 돌출은 배치 가정이며 적재강성 검증 결과가 아니다. V4에서 기존 파란 타워 틀은 숨기고 `토마토 보관함 표시`로 새 통을 켜고 끈다.
- 현재5906 출력 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/preview_storage_bin_v2`, PID/출력은 같은 상위폴더 `process.json`에 기록한다. 기본 시점 `보관함 배치`, 카메라는 계속 Z위쪽/접속자별 유지한다. V4는 정적 형상 비교이며 수동 리프트 조절만 가능하다. 토마토 투입/수확 경로/보관함 충돌/적재하중은 검증하지 않았다. 기존 농장/5905/원본 물리 자산은 수정하지 않았다. 새 영상은 만들지 않았다.
- 실제 브라우저HTTP200/오류0에서 V4 높이.75→.55 변경 시 상판·모터·로봇 동시 이동/보관함 고정, 보관함 토글, V4→V2→V3→V1→V4 전환, V1 기존 모션 재개, 카메라 불변을 확인했다. 첫 시험에서 높이 조절 뒤 버전 선택 시 Viser가 높이 초기화 콜백을 같은 스레드에서 동기 호출하여 일반 Lock이 교착되는 문제를 재현했다. RLock으로 수정하고 같은 조작을 포함해 전체 재검증했다. 실패 출력 `preview_storage_bin_v1`도 보존한다.
- 증거 `storage_layout_validation.json`/`storage_browser_validation.json`/`storage_source_preservation.json`, 최종 렌더 `layout_v4_storage_final.png`/`layout_v4_storage_top.png`. 리프트0/.375/.75에서 V4의 전체 몸체 변환이 V2와 정확히 같고 V3도 보존된다. 기본 자세에서 보관함 외곽 안 원래 CAD 정점0/팔 최저점과 통 상단 사이195mm를 확인했다(정확한 메시 충돌·전체 모션 검증 아님). 원본 MJB/초기 상태/450개 CAD 형상·색상 해시와 V1 전체 궤적 배열이 직전V3 출력과 정확히 같다. 관련 회귀24개 통과. 커밋은 요청되지 않았다.

## 2026-11-12 최신: 5906 V3 로봇암 외측 배치

- 사용자가 V2 트레이타워 앞 빨간 영역으로 로봇암을 빼도록 지시했다. V1/V2를 유지하고 `V3 · 로봇암 외측 배치` (`--layout outboard`)를 추가했다. 리프트·모터·트레이 공간은 V2 위치 그대로 두며 RB5 베이스/6축 팔/브래킷/그리퍼/카메라만 세계Y- 방향으로 기본.80m 추가 이동한다. 화면의 `로봇암 돌출거리 (m)`에서0..1.20m 조절 가능하다. 관절 각도/한계는 바꾸지 않는다.
- 기존 상판 끝에서 새 베이스까지 이어지는 약.766m 길이의 연장 상판을 형상으로 추가했다(기존 판과20mm 겹침, 베이스 중심보다110mm 앞쪽 여유). 상판 두께/폭은 기존10mm/약230mm에 맞춘 배치 참고용이며 제작 강성/처짐/전도 검증을 한 것은 아니다. 높이 변화에 따라 연장판과 팔이 함께 이동하고 모터는 원래 상판 아래 장착을 유지한다.
- 기존 아래쪽 후보 자세/리프트.75m에서 전체 팔·툴 시각 메시Y최댓값=-.383637m, 트레이 예정 공간 앞면Y=-.33m로 약53.64mm 분리된다. 공간 안 팔 정점0. 이는 해당 자세의 분리이며 전체 작업영역·이동경로·실제 타워 선반 충돌 검증이 아니다. 리프트0/.375/.75m에서 팔만.8m 이동, 다른 몸체/방향 유지, 연장 상판의 베이스 지지면 연결 확인. 기존V2 몸체 변환도 그대로 유지. 증거 `outboard_layout_validation.json`.
- 현재5906 출력 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/preview_outboard_layout`. 새 `외측 배치` 시점은 Z위쪽 회전 기준을 유지하고 초기 화면에 전체 돌출 배치가 들어오도록 설정했다. V1은 기존 모션을 유지하며 V2/V3는 정적 형상 비교다. 원본 농장/5905/물리 자산은 수정하지 않았다. 커밋은 요청되지 않았다.
- 실제 브라우저HTTP200/오류0에서 V3→V2→V3→V1→V3 선택, 돌출.8→.9→.8m에 팔만 이동/연장판 길이 연동, 높이.75→.35m에 팔·모터·상판 동시 이동, 배치틀 토글, V1 모션 재개, 카메라 유지 확인. `outboard_browser_validation.json`, 최종 `layout_v3_outboard_final.png`를 실제 렌더로 확인했다. 기존 CAD/색상 해시 및 V1 전체 궤적 배열도 V2 출력과 정확히 같다. 새 영상은 만들지 않았다.

## 2026-11-12 최신: 5906 배치 V1/V2 선택

- 사용자 요청으로 5906 조립체 미리보기에 `배치 버전`을 추가했다. `V1 · 기존 배치`는 기존 좌표/모션을 유지하고, `V2 · 리프트 밀착 배치`는 리프트를 컨트롤박스 쪽 세계Y+ 방향으로.51136263465m 이동한다. 실제 CAD의 캐비닛 앞면과 리프트 고정부 외곽 사이20mm를 기준으로 구했다. 기둥/원래 지지대5개 메시를 별도 `Preview_lift_frame`으로 분리하고 상판·모터·RB5·툴을 같은 이동량으로 함께 옮긴다.
- 트레이타워 실치수가 아직 없어 앞쪽 상판의500×400×900mm 가정 공간을 파란 선반 틀로 표시한다. V2에서 표시 토글 가능. 제작용 형상/새 물리 충돌체가 아니다. V2는 정적 배치 비교와 수동 리프트 높이 조절을 제공하며, 기존 검증 궤적을 변경 배치에서 재생하지 않는다. 기존 모션은 V1에서 유지한다. 관절 한계/기구학 원본/농장·5905 실행 자산은 변경하지 않았다.
- 기본 `--layout original` 유지, 새 배치로 시작하려면 `preview-hanging --layout compact`. 현재5906은 기존 pose-file/workspace-motion/left/autoplay와 함께 compact로 띄운 `preview_layout_versions_v2` 출력이다. 시점은 접속자별로 유지하고 새 `배치 비교` 시점도 Z 위쪽/회전축을 유지한다. 배치 변경 시 카메라를 재설정하지 않는다.
- 실제 CAD450개 형상의 정점/면/정점색 및 V1 세계 변환이 이전 버전과 정확히 같음(최대 변환 오차0)을 확인했다. 리프트0/.375/.75m에서 이동/고정 그룹 및 모터-상판 상대변환 검증 통과. 증거 `compact_layout_check/layout_validation.json`. V1 전체 팔/리프트 궤적 배열도 이전 `preview_camera_orbit_fix`와 정확히 같다. 기존 관련 회귀24개 통과. 새 배치 동작·간섭·적재하중 검증 완료를 주장하지 않는다.
- 실제 브라우저HTTP200/오류0에서 compact→original→compact 전환, V2 높이.75→.35m의 팔/상판/모터 동시 이동, 배치틀 표시 토글, V1 기존 동작 재개, 카메라 유지 확인. `layout_versions_browser_validation.json`, 화면 `layout_v1_original.png`/`layout_v2_compact_final.png`/`layout_v2_lowered_lift.png`. 새 영상은 만들지 않았다.
- 배치 비교에서 주의할 실제 발견: V2 리프트.75m/현재 아래쪽 후보 자세에서 Robot_link2/3의 시각 메시 정점11,895/40,788개가 가정 트레이타워 공간 안에 있다. 팔꿈치 부근과 예정 공간의 겹침이 확인되며, 실제 선반/프레임 충돌이나 전체 가동 범위를 검증한 것은 아니다. 타워 실치수·높이 또는 팔 자세의 추가 설계가 필요하다. 증거 `compact_tray_envelope_check.json`. 이 사실을 사용자 최종 보고에 알린다.

## 2026-11-12 최신: 미리보기 카메라 회전 기준 복원

- 사용자 지적의 직접 원인은 직전 '위에서' 시점의 `client.camera.up_direction=(0,1,0)` 설정이었다. 이 값이 orbit 축까지 세계Y로 바꿨다. 기존 공유 시점 드롭다운은 테스트/다른 접속자의 선택을 모든 접속자 카메라에 강제 적용하기도 했다.
- 모든 시점의 위쪽/회전 기준을 세계Z `(0,0,1)`로 고정했다. 완전한 수직 탑뷰의 특이점을 피하려고 탑뷰 오프셋만 `[0,-.25,2.6]`으로 조금 기울였다. 시점 드롭다운은 `client.gui`로 이동해 값/카메라를 접속자마다 분리했다. 한 접속자가 시점을 바꾸거나 새로 들어와도 다른 화면은 재설정하지 않는다.
- 실제 브라우저2개/6개 시점에서 마우스 드래그로 유효한 위치 변화, Z위쪽 유지, 다른 접속자의 카메라/선택값 불변, JS오류0 확인했다. 증거 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/camera_orbit_validation.json`, 캡처 `camera_orbit_fixed.png`. 새 영상은 생성하지 않았다.
- 현재5906 출력 `preview_camera_orbit_fix`. 이전 `preview_opposite_workspace`와 CAD/색상 해시 및 전체 팔/리프트 궤적 배열이 정확히 같다. 기존 반대편24개 목표 기구학 미리보기를 유지하며 카메라만 수정했다. 새 커밋은 요청되지 않았다.

## 2026-11-12 최신: 반대편 작업 영역 접근 미리보기

- 사용자 탑뷰의 빨간 영역을 기존 녹색 박스의 차체 중심선 반대편 X=-.78..-.35/Y=-.34..-.06/Z=.30..1.04m로 대략 해석했다. `preview-hanging --workspace-motion --workspace-side left --autoplay --port 5906`으로 선택한다. 기존 `right` 기본은 유지한다. 기존 CAD/형상 색상/리프트/매달림 설치와 충돌 마스크·1mm 기준을 재사용한다.
- 실제 IK로 반대편을 향하는 자세를 구하며 관절각을 단순 반전하지 않는다. 기존 자세에서 리프트.75m로 반대편 X=-.46/Y=-.2/Z=.95m까지 4.102초의 검사된 관절 연결 후, 반대편 목표24개를 연속 방문/역순 반복한다. 영역 안의 방향은 반대편 정면 기준±15도, 작업면을 바꾸는 최초 연결에서만180도 회전을 허용한다. 연결에서도 충돌 검사는 유지한다. 새 '위에서' 시점을 추가했다.
- 전체 채택 경로14,291개 표본에서 기존 충돌체 관통0, 팔 최대표본.00199978rad/리프트.499693mm, 영역 안 최대방향14.92845도. 리프트.40821..75m 이동. 실제 방문한 영역 X=-.76393..-.37487/Y=-.30241..-.07682/Z=.55824...99962m. 낮은 일부 목표는 차체 간섭/도달 실패로 제외하며 빨간 박스 전체가 도달 가능하다는 주장은 아니다.
- 현재5906 출력 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/preview_opposite_workspace`, 원자료의 `workspace_motion.json`/`workspace_trajectory.npz`. 초기 독립 계획도 `opposite_plan_check`에 보존한다. 형상/FK/충돌 표본 기반의 기구학 미리보기이며 Newton GPU 동역학·수확·실기 실행 검증은 아니다. 새 커밋은 요청되지 않았다.
- 실제 브라우저HTTP200/오류0,5개 시점 모두 반대편X<-.35m와 위치 변화 확인, 일시정지/재개 통과. `opposite_workspace_browser_validation.json`, 정면/탑뷰 캡처 `opposite_workspace_front.png`/`opposite_workspace_top.png`,25초1배속 실제 브라우저 영상 `opposite_workspace_live.mp4`(13px 자막). 원본webm도 `opposite_browser_recording`에 보존한다.

## 2026-11-12 최신: 휠·조작부 색상 복원

- 사용자 지적의 원인은 차체 `s300_g0046`/`chassis_11` 한 메시(787,707정점)의31가지 원본 정점 색을 변환 시 첫 색 `[1,1,.2157]` 하나로 축약한 것이었다. 이 때문에 바퀴뿐 아니라 화면/버튼도 모두 노랗게 보였다.
- `robot_vertex_colors.py`가 기존 로컬 USD의 원본 색을 읽고 source/compiled 정점 및 면 인덱스 일치를 검증한 뒤 packed 파일에 정점별RGBA를 보존한다(좌표 오차6.87e-8m). `PackedScene`/`pack_scene`/`newton_viser.geom_mesh`와 정적 배치 경로도 이를 유지한다. 미리보기 준비에서 .asset-venv의 일회성 색 복원 후 최종 packed 메시를 사용한다.
- 원본 노란 CAD 색 부분은 형상을 분리해 확인한 고무 롤러32개이므로 검정 계열 `[7,9,12]`로, 휠의 푸른 CAD 금속 부품은 회색 `[160,166,174]`으로 변경했다. 화면/비상정지는 원본의 검정/빨강을 복원한다. 최종30색, 롤러 정점35,760개/회색 금속140,808개, 휠 노란 정점0. 형상·토폴로지와 이전 리프트/팔 궤적·native collision patch는 배열/파일 해시로 정확히 같다.
- 현재5906 출력 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/preview_wheel_materials`. 원본 USD/MJB 및 기존 농장 실험 자산은 유지한 미리보기 색상 변경이다. 검증 `wheel_materials_validation.json`, 출력의 `robot_vertex_colors.json`. 색상 유지3회귀+기존 뷰어/장면 패킹 포함20개 통과. 새 커밋은 요청되지 않았다.

## 2026-11-12: 리프트와 로봇암 동시 랜덤 동작

- 이전 랜덤 미리보기는6축 팔만 명령했고 리프트는0.75m에 고정했다. 모터를 상판에 연결한 직전 변경만으로는 리프트가 자동으로 움직이지 않았다. 사용자 지적에 따라 `--workspace-motion` 기본을 리프트1축+팔6축 연동으로 수정했다. `--fixed-lift`는 이전 고정 높이 비교 모드다.
- 목표의 높이에 맞춰 리프트를 정하고, 각 Cartesian 구간을 리프트 높이와 함께 보간하며 기존 Jacobian IK를 푼다. 혼합 단위는 별도로 처리한다(리프트 표본<=0.5mm, 팔<=.002rad). 시간은 기존 리프트.12m/s/팔 속도 한도로 정한다. 표본별 관절 한계/기존1mm 간섭/녹색 영역/좌우15도 조건을 확인한다.
- 24개 목표/15,087개 검사 표본, 관통0, 리프트.341558..75m(약40.84cm 이동), 최대 좌우각14.9293도. 별도120개 시점에서 표시 FK와 검사 모델 오차<=1.2e-15, 모터-상판 상대 변환 오차<=5.6e-17, 두 부품 이동량 일치. 회귀4개 통과. 증거 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/coordinated_lift_path_validation.json`.
- 현재5906 출력 `preview_lift_arm_motion`. 화면의 리프트 표시도 실제 재생 높이를 따라 바뀐다. 저장 궤적은 `joint_positions_rad`(6축)와 `lift_positions_m`를 분리한다. 모델/메시/모터 연결은 그대로이며 원본 MJB는 수정하지 않았다. 이번에도 기구학 미리보기이며 Newton 동역학 실행이 아니다.
- 실제 브라우저HTTP200/오류0,35초 중5초 간격7표본에서도 리프트 약26cm 이동을 확인했고 상판/모터의Z 변화량과 정확히 같았다. 일시정지는 리프트/팔을 모두 정지시키고 재개했다. `coordinated_lift_browser_validation.json`,35초1배속 영상 `lift_arm_random_motion_live.mp4`(13px 자막), 화면 `coordinated_lift_preview.png`를 같은 원자료 폴더에 저장한다. 새 커밋은 요청되지 않았다.

## 2026-11-12: 리프트 모터를 상판 아래에 연결

- 사용자 지시에 따라 5906 조립체 미리보기의 기존 리프트 모터/감속기 및 부속40개 CAD 메시를 고정 기체 그룹에서 분리했다. 기둥/하부 지지대는 유지하고 `Preview_lift_motor`를 움직이는 상판에 고정 연결한다. 모터 상단은 상판 아랫면보다10mm 아래이며 원래 CAD 방향/XY 배치는 유지한다.
- `preview_robot_mount.py`가 상판 좌표계의 고정 장착 변환을 사용한다. 리프트0/.375/.75m와 이전/변경 받침판 높이의6개 조합에서 모터 이동량=상판 이동량, 상대 변환 및10mm 간격 일정, 하단의 기존 모터 중복0 확인. 증거 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/motor_mount_check/motor_mount_validation.json`.
- 현재5906은 기존24개 랜덤 동작을 유지한 `preview_moving_motor` 출력으로 재실행한다. 이전 수확용 원본 USD/MJB 및 물리 질량/접촉 정책은 변경하지 않은 형상 수정이다. 기존 CAD 모터는 시각 형상이며 새 물리 충돌체를 추가하지 않았다.

## 2026-11-12: 5906 작업 영역 내 무작위 관절 동작

- 사용자 표시 영역을 대략 X=0.35..0.78/Y=-0.34..-0.06/Z=0.30..1.04m의 얕은3D 박스로 해석했다. 기준점은 기존96×158×84mm 수확 영역의 중심이다. 상단 받침판/리프트.75m/매달림 자세를 유지하고24개 무작위 목표로 연속 이동한 뒤 같은 경로를 역순으로 반복한다. 기체/리프트는 이동하지 않는다.
- 실행 `./nvidia-sim/run_s300_newton.sh preview-hanging --port 5906 --pose-file /root/farmily_runtime/newton/runs/20261112_hanging_preview/lower_pose.json --workspace-motion --autoplay`. 브라우저 접속 시 재생, 시작/계속·일시정지·처음부터·배속 선택. 기본 작업 영역 시점은 정면이다. 현재 출력 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/preview_workspace_motion_v2`.
- 기존 Jacobian IK를 재사용한 **기구학 관절 궤적 미리보기**다. Newton GPU 동역학/수확/실기 구동 검증이 아니다. 원본 물리 파일은 유지하고 개인 복사 모델에서 베이스/받침판 충돌체를 실제 표시 위치로 옮겨 검사했다. 숨겨진 농장은 검사에서 제외한다. 로봇 자체·리프트·기체의 기존 마스크/제외 정책과1mm 기준은 유지한다.
- 변경한 로컬 충돌체의 BVH는 mj_setConst만으로 갱신되지 않아 직접 재계산했다. 실제 충돌 검출 회귀와 루프 연결 회귀2개 통과. 채택 경로17,572개 표본(최대.002rad)에서 관통0. 반복 전 구간30Hz5,308프레임의 영역/관절 한계 확인, 최대 좌우각14.9293도. 도달/경로 검사 실패 후보는 제외하며 박스 전체의 도달 가능성을 증명한 것은 아니다. 방문 높이.407..1.000m.
- 원자료 `workspace_path_validation_v2.json`, 현재 출력의 `workspace_motion.json`/`workspace_trajectory.npz`. 모델 전체559MB를 반복 저장하지 않고 약간의 설치 변경 배열과16KB 궤적만 저장한다. 이전 프로토타입의 이번 작업용 모델 복사 캐시는 정리했고 기존 사용자 자산/정적 모델은 유지했다.
- 브라우저HTTP200/오류0, 실제 위치 변화와 일시정지·재개 확인(`workspace_browser_validation_v2.json`). 실제 브라우저35초·1배속 녹화 `workspace_random_motion_live.mp4`, 화면 `workspace_motion_preview_v2.png`. 화면 구성 변경 전후 궤적/충돌 변경 배열 해시가 정확히 같다. 자막13px. 새 커밋은 요청되지 않았다.

## 2026-11-12: 매달림 받침판을 상단으로 이동

- 사용자 표시 위치에 맞춰 5906 형상 미리보기의 받침판을 기존 위치보다 0.35m 올렸다. 판 아랫면은 리프트0.75m에서1.446m. 리프트 기둥/뒤 캐리지는 유지하고 받침판·매달린 RB5 베이스·팔·브래킷·그리퍼·카메라를 함께 올린다. 팔 자세/방향은 직전 아래쪽 후보와 같다.
- `preview-hanging` 기본 상단 배치, `--plate-rise-m .35`로 오프셋 지정. 화면의 '받침판 위치'에서 이전 하단/변경 상단을 비교할 수 있다. 현재 실행은 기존 `lower_pose.json`을 함께 지정했고 출력은 `/root/farmily_runtime/newton/runs/20261112_hanging_preview/preview_upper_plate_v2`다.
- 정적 배치 확인: 모든 팔/툴/베이스 이동량 정확히350mm, 방향과 기둥 유지, 표시 받침판1개, 원본 MJB/초기 상태 해시 동일. 원자료 `upper_plate_layout_check.json`. 이번 요청은 형상 변경이며 물리 모델/수확 실행은 바꾸지 않았다. 이전 낮은 받침판에서의 정적 간섭 확인을 새 배치의 검증으로 표시하지 않는다.

## 2026-11-12: 매달림형 그리퍼 아래쪽 자세 확인

- 사용자가 표시한 차체 위 아래쪽 공간으로 그리퍼를 내릴 수 있는지 정적 IK를 확인했다. 매달림 변환을 native 모델의 base parent frame과 link0 collider에 적용한 FK는 기존 형상 미리보기와 최대1.1e-16 일치. 물리 적분/이동 경로 시험은 하지 않았다.
- 같은XY Tool_fixed 목표 [.15,-.2,.42/.45/.48]m는 관절 범위 내 해가 있으나 기존 차체/일부 팔-그리퍼 간섭이 있었다. 목표Z=.54m 후보는 관통0. 팔꿈치-156.38도(모델 한계-165도까지8.62도), 손목2+90도, 손목3+71.55도. 실제 표시 박스 좌표를 측정한 것은 아니며 화면의 아래쪽 영역을 대략 해석한 후보다.
- 5906에 `--pose-file /root/farmily_runtime/newton/runs/20261112_hanging_preview/lower_pose.json`으로 실행했다. '로봇 자세'에서 기존/그리퍼 아래쪽 후보 비교 가능. 정적 간섭 확인은 매달림/리프트.75m 기준이며 높이/설치방향 변경 시 재검사된 것으로 표시하지 않는다. 현재 실행 출력 preview_lower_pose_v2. 원본 자산/수확 모델은 유지한다.
- 증거 lower_target_search.json, lower_clearance_search.json, hanging_static_check.mjb, lower_pose.json. 형상 캡처 lower_gripper_preview.png. 케이블/실기 구동/연결 이동 경로 검증 아님.

## 2026-11-12 最新: 5906 매달림 로봇 형상 미리보기

- 사용자 요청으로 ROS/자동수확 실행 대신 현재 CAD 조립체의 매달림형만 표시한다. 실행 `./nvidia-sim/run_s300_newton.sh preview-hanging --port 5906`. 기본 리프트0.75m, 기존 상판 아랫면1.096m에 RB5 베이스를 붙이고 원래 팔·90도 브래킷·그리퍼·카메라를180도 회전한다. 베이스 하우징도 함께 회전하며 리프트/기체는 정립 상태다.
- `preview_robot_mount.py`는 기존 검증된 packed CAD 형상·색상과 native FK를 재사용한다. 설치 방향(매달림/기존 정립), 리프트 높이, 전체/정면/측면/확대 시점 선택 가능. 농장 배경 대신 기체·리프트·로봇 조립체만 표시. 원본 물리/농장 자산과 현재 patrol_current는 수정하지 않는다.
- 형상 확인 요청에 따라 물리 스텝/모션/충돌 검증0. 브라우저에서 실제 형상 렌더와 HTTP200 확인. 원자료 `/root/farmily_runtime/newton/runs/20261112_hanging_preview`, 최종 미리보기 preview_v2, 화면캡처 hanging_preview_final.png. 정적 형상이라 동영상은 만들지 않았다. 이 변형을 실행 가능한 물리 모델로 보고하지 않는다.

## 2026-11-11 최신: ROS 목표 → OMPL 경로 계획 → Newton GPU 실행

- 사용자 요청으로 실제 OMPL2.0.1 RRTConnect를 연결했다. 실행 `./nvidia-sim/run_s300_newton.sh ros-control --planner ompl`, 기본5905/domain42. `setup_s300_newton.sh --planner`는 기존 물리 버전 핀을 유지한다. 이번 변경은 미커밋 상태이며 직전 ROS 제어 커밋843b066 위에서 작업했다.
- `/farmily/sim/planner/joint_goal`(JointState,팔6축 subset), `/planner/pose_goal`(PoseStamped,world의 Tool_fixed 원점), `/planner/status`, `/planner/trajectory` 추가. 별도 Python3.12 worker가 현재 MJB를 한 번 로딩해 CPU FK/충돌/OMPL 검색; 실제 궤적은 기존 Newton GPU actuator로 실행. 자유 직선은 먼저 검사하고 막히면 RRTConnect로 우회한다. 기존 직접 trajectory 토픽은 OMPL을 거치지 않는다.
- 모델 SHA·정지 상태·전체10축 시작 명령을 실행 직전에 원자적으로 확인한다. 계획 경로 실행 동안 기체/리프트/그리퍼 포함 다른 움직임을 거절하며 stop은 허용한다. 실제 도달 q/v로 완료 판정. 피드백 중단/프로세스 종료 감시, 상세 계획/경로/실측 로그 기록. 한도 기본2초, 정확한 경로만 허용,1mm 및 기존 plant collision 정책 유지, 경로 표본<=.002rad.
- DDS+GPU headless55초 시험:6축/Cartesian 목표, 실행 중 리프트 거절, 잘못된 목표 거절, 취소·재계획 통과. 간단한 목표 계획57.6/60.5ms, 실제 오차.00064rad/1.17mm,12100스텝/활성 관통0. GUI 원본 농장 팔↔그리퍼 간섭 경로 우회 및 복귀 모두 완료: 계획.8878/.9008초,1774스텝8.064sim/8.131wall초, 활성 관통0, 브라우저 오류0.
- 첫 GUI 우회 성공 직후 복귀가 레일 실측-0.000608mm를 관절 범위 이탈로 거절했다. 실패를 보존하고 측정 위치/연결 구간에 기존 제어기 추종 범위만 적용하도록 수정했다. 실제 좌표를 clamp하지 않고 충돌 검사하며 명령 시작/목표/경로 한계와1mm 기준은 엄격하게 유지한다. 동일 실패 상태 재시험 및 새 GUI 왕복 통과. native29+ROS8회귀 통과.
- 실행/토픽 문서 `nvidia-sim/S300_NEWTON_ROS2_KO.md`, 검증 `nvidia-sim/validation/s300_ros2_ompl_20261111.json`, 원자료 `/root/farmily_runtime/newton/runs/20261111_ros2_ompl`. 작은13px 자막 실제wall-clock 영상 `ompl_detour_live.mp4`, 수정 전 `ompl_detour_before_bounds_fix.mp4`, 전체원본 gui_video_v2/gui_video 보존.5905는 새 OMPL 연결 시작 대기 화면.
- 기존 자동수확 순회 알고리즘을 OMPL로 교체한 것은 아니다. ROS 목표 계획 연결이며 MoveIt 액션 서버/실제 하드웨어 드라이버는 미구현. 초기 농장·카메라 로딩은 개선하지 않았다. CPU 경로 계획을 GPU 물리 가속이라고 보고하지 않는다.

## 2026-11-11 최신: ROS 토픽으로 로봇암·기체·리프트·그리퍼 제어

- 사용자의 추가 요청에 따라 `./nvidia-sim/run_s300_newton.sh ros-control`을 구현했다. 자동 순회 `camera-patrol --ros2`와 별도 명령 권한 모드다. 기본5905/domain42, 시작 자세 선택 후 **ROS 제어 시작**. 실제 장비 드라이버는 실행하지 않는다.
- `arm_controller/joint_trajectory`/`joint_trajectory`(JointTrajectory), `cmd_vel`(Twist.linear.x 레일1축), `lift/command`/`gripper/command`(Float64), `stop`(Bool) 구독. 이름·위치·속도·가속도 극값/시각을 검증하고 실제 GPU actuator에 전달한다. qpos 강제 대입 없음. 완료는 실제 q/v로 판정하며 정지 해제는 이전 취소 경로를 재개하지 않는다. cmd_vel0.5wall초 timeout, 브리지 단절 시 전체 감속 정지.
- 기존 Newton GPU 모델/주줄기 충돌/1mm 가드 유지, 물리220Hz·2스텝 CUDA graph 버퍼 반복으로 세션 길이에 GPU 메모리가 비례하지 않는다. 상태20Hz, 실제 카메라 기본1simHz는 별도 스레드. 지연된 영상은 노출 시각 TF를 유지하고 clock/최신 joint_states를 되감지 않는다. observations.age_s는 최신 시뮬레이션 시각과 실제 노출 차이를 표시한다.
- 최신 실제 DDS+GPU: 여섯 암 관절 동시 이동/리프트/그리퍼/기체 전진·후진/timeout/stop/잘못된 명령 거부/카메라 통과. 4400스텝19.9996sim초/20.0047wall초,6축 최대 목표 오차0.0001473rad, 활성 관통0. 앞선45초headless/9초GUI도 통과. 녹화 GUI는 손목 한 축 시험이며 최신6축 검증은 headless다.
- ROS 실시간 실제 명령9900스텝을 기존 GPU 실행기로 다시 실행하여4950배치의 qpos/qvel/time 비트 일치. GUI/headless의 wall-clock 명령 도착은 달라질 수 있으므로 독립 입력의 궤적 일치 주장 아님. 연결 단절 시험은 실제0.08m/s 주행 중 브리지 종료 후 최대 속도0.000154미만/관통0/ros_disconnected 종료.
- 온라인 입력free-Hz 정리 중 기본 프리셋60Hz가 과거 계획 재샘플링을 유발한 초기화 실패를 보존했다. 명시적220Hz 옵션과 조기 온라인 버퍼 검증으로 수정했다. 실험 `final_six_axis` 실패를 `final_six_axis_v2` 성공으로 덮어쓰지 않았다.
- 원자료 `/root/farmily_runtime/newton/runs/20261111_ros2_control`, 검증 `nvidia-sim/validation/s300_ros2_control_20261111.json`, 실행/토픽 문서 `nvidia-sim/S300_NEWTON_ROS2_KO.md`. 영상 `ros2_control_live.mp4`(13px,실제wall-clock9.84초)와 전체원본 `gui_video/page@3835a254accb64cd2e1caf26083aeeec.webm`.5905는 ROS 제어 시작 대기 화면으로 전환했다.
- 이 결과는 초기 로딩/자동 순회 플래너 대기 시간 해결이나 실제 모터 제어 완료가 아니다. ROS 액션 서버/MoveIt/하드웨어 드라이버/외부PC DDS는 별도 연결 단계다. 초기 수동 자세에서는 카메라에 로봇/구조물이 크게 보일 수 있으며 촬영 자세 역시 ROS 명령으로 지정한다.

## 2026-11-11: Newton 순회 ROS 2 토픽 연동

- 요청대로 기존 누적 작업 먼저 `b35858a`에 커밋했다. 같은 브랜치 `codex/s300-newton-standalone-20261108`에서 ROS 연동을 추가했다. 푸시는 요청하지 않아 하지 않았다.
- `camera-patrol --ros2`: Python3.12 Newton GPU 물리는 그대로, 별도 Python3.10 ROS2 Humble 프로세스가 실제 관절/리프트/기체/TCP/TF/clock/RGB/depth/GT mask/CameraInfo/가시 객체/상태를 발행한다. domain42, namespace `/farmily/sim`. 제어 구독이나 하드웨어 드라이버 실행 없음.
- `nvidia-sim/run_s300_ros2.sh build/check/topic/inspect` 및 `ros2_ws/src/farmily_newton_bridge` 추가. 카메라는 기존 촬영 시점만 발행하며 고정 영상 FPS가 아니다. nominal 상태10Hz + 촬영/구간경계 표본, exposure-time TF, 깊이32FC1 m/NaN, instance uint32 bit-pattern을32SC1로 보존한다. 음수도 유효 ID이고 `source_instance_id`로 원본 ID가 보존된다. 느린 DDS는 최신 대기 표본으로 합치고 런타임 ROS 단절이 물리를 중단하지 않는다.
- 실제 새 headless ROS OFF/ON 각각4개 구간/1회 동작/6개 영역 수확. 기록1,890상태와224개 물리·명령·접촉 배열이 정확히 같다. ROS 실제 관절97개/카메라7세트 수신,35개 이미지 바이트 동일, TF 오차6.67e-16, 전송 오류/누락0. 12.027sim초 대비 OFF17.348/ON17.994wall초(초기 별도): 짧은 단일 표본의3.7% 차이이며 가속/실시간 달성 주장이 아니다.
- 새 GUI+ROS도224개 물리 배열 동일,1회/6개 영역 수확,관절98개/초기 미리보기 포함 카메라8세트 수신,브라우저 오류0. 마지막5905는 ROS옵션/3.85m/1000회 한도로 입구 시작 대기 화면을 다시 띄웠다. START 뒤 초기 준비 후 토픽이 나타나며 종료하면 발행도 종료한다.
- 전체3.85m ROS 반복/외부PC DDS 네트워크는 미검증. 카메라30FPS 스트림이나 ROS제어를 구현했다고 해석하지 않는다. 상세 `nvidia-sim/S300_NEWTON_ROS2_KO.md`, 검증 `nvidia-sim/validation/s300_ros2_bridge_20261111.json`, 원자료 `/root/farmily_runtime/newton/runs/20261111_ros2_bridge`. 새 영상 저장 없이 실제 GUI와DDS로 검증했다. native36개/ROS3개 회귀 통과.

## 2026-11-11 최신: 순회 시작 범위와 짧은 종점 감속 오류 수정

- 사용자 마지막 명령 `camera-patrol --end-station 3.85 --max-harvests 1000 --max-decisions 10000`은 기본 자산이0.85m를 가리켜 START 뒤 실패했다. 기존3m 농장 자산에 이미 있는4그루/40열매 ID와64개 주줄기 collider를 추가 등록하여3.85m로 확장했다. 새 메시/식물 조인트0, 준비46.42초. `patrol_current` → `/root/farmily_runtime/newton/assets/patrol_reuse385m_20261111`. 실제 실행 중 농장을 새로 변환하지 않는다.
- `preflight_patrol`은 GUI 형상 로딩 전에 선택 가능한 up/down 구간을 확인한다. 잘못된 범위는0.46초에 실패하며 launch/summary에 원인을 남긴다. 모델/카메라 초기화 예외도 error.json에 기록한다. 시작 화면에 실제 순회 종점을 표시한다.
- 새 GUI 검증에서0.8482916m 지점의 전체 주행 계획에1.708mm 잔여 구간이 생겨 기존 고정0.12m/s 경계속도로 정지할 수 없는 추가 오류를 발견했다. `cruise_route`는 남은 정지거리와 단계별 가속거리로 다음 경계속도를 제한한다. 속도0.12m/s·가속도0.3m/s²·카메라 관측 간격·충돌 규칙은 유지한다. 기록된 실패 위치와10µm 잔여 구간/1cm 이동 간격을 포함한 경계속도 연속성·단조 이동·속도/가속도 회귀33개 통과.
- 원자료 `/root/farmily_runtime/newton/runs/20261111_route_start_fix`. 첫 주행의 별도 감속 실패는 `failed_tail_gui`에 보존했다. 새 GUI/GPU 전체 재실행은3.850217m 도착,19회 수확 동작/18송이에서126개 영역 수확, 모든 물리 구간 완료/활성 위반0/진입각 통과. 169.975sim초/360.310wall초(RTF0.472), 초기23.871초 별도. 실행 속도 전체를 개선한 작업이나 물리 분리 검증은 아니다. 새 headless 대조/엘보우 아래 전체 검증은 하지 않았다. 검증 `nvidia-sim/validation/s300_route_start_fix_20261111.json`, 최종 영상 `route385m_live.mp4`(실제 browser wall-clock,13px 자막), 실패 영상 `failed_tail_route385m_live.webm`.5905는 새3.85m 입구/시작 대기 상태로 재실행한다.
- 실행: `./nvidia-sim/run_s300_newton.sh camera-patrol --end-station 3.85 --max-harvests 1000 --max-decisions 10000` (포트5905). 새 기본 자산의 등록 한도는3.85m이며 더 긴 구간은 기존 농장 재사용 `prepare-route`로 미리 등록한다. 수확 횟수 옵션을 생략하면 기존 기본6회 한도는 유지된다.

## 2026-11-11 최신: 이동을 막지 않는 장면 선로딩

## 2026-11-11 최신: 기존 농장 재사용3m 순회

- 사용자 지적대로 불필요한 식물별 탄성 변환을 중단했다. `prepare-route` → `prepare_static_patrol_route.py`: 기존 전체 농장 메시·색상·배치/instance catalog 재사용, 추가7그루130열매 ID와112개 주줄기 capsule만 등록. 새 메시/식물 조인트0. 기존 로봇/카메라/1mm/진입±15도/84mm 영역 유지. 준비45.79초, 시각31,468형상/3,471메시의 초기 world 형상·색상 정확히 동일 확인.
- 자산 `/root/farmily_runtime/newton/assets/patrol_reuse3m_20261111` (현재 기본 `patrol_current`0.85m는 유지). 실행: `./nvidia-sim/run_s300_newton.sh camera-patrol /root/farmily_runtime/newton/assets/patrol_reuse3m_20261111 --end-station 3 --max-harvests 1000 --max-decisions 1000`.
- 실제 GUI/GPU 3.000205m 도착,16회 동작/15송이에서107개 영역 판정 수확. 전체 송이 완전 수확/물리 분리 증거 아님. 모든 물리 구간 완료/활성 위반0/진입각 통과. 138.06sim초/286.59wall초, 초기23.34초 별도. 실행의 경로·카메라 대기는 남음. 아래 자세3m 전체 및 새 GUI/headless 대조 미실시.
- 포트5905 최종 화면 유지. `geometric_fruit_names`는 새 물리 attachment 없이 정적 열매를 수확/숨김하는 목록이며 `fruit_catalog`와 기존 robot assembly를 구분한다. 회귀30개, row target12개 통과. 검증 `nvidia-sim/validation/s300_reused_farm_3m_20261111.json`, 원자료 `/root/farmily_runtime/newton/runs/20261111_patrol_3m`, 영상 `route3m_live.mp4`(wall-clock, 작은 자막). 중단된 예전 전체 변환 `/root/farmily_runtime/newton/assets/patrol_route3m_20261111`는 보존하되 사용하지 않는다.


- 사용자 요청은 다음 구간 선로딩 때문에 별도로 기다리지 않도록 하는 것이다. `newton_scene_chunks.py`에서 오프라인 구간 파일을 만들고 `newton_scene_stream.py`가 별도 프로세스에서 다음 카메라 GPU 모델·BVH·첫 렌더를 준비한다. 현재 장면만 최초 준비 장벽이며 다음 장면을 기다린 뒤 출발하지 않는다. 구간 경계에서 READY만 확인하고, 미완료면 기존 장면의 유효 겹침 범위를 사용한다. 범위를 소진하면 명시적 오류이며 가짜 관측/충돌 생략으로 진행하지 않는다.
- GUI 정적 장면도 별도 스레드에서 준비한다. 전환은 표시 토글뿐이고, 이전 메시 삭제/GPU 프로세스 join은 백그라운드에서 한다. 현재/다음 두 worker로 제한하고 기존 물리 적분·실측 상태·수확 ledger를 그대로 유지한다. 내용이 같은 구간은 GPU 모델을 재사용한다.
- `patrol_current` → `/root/farmily_runtime/newton/assets/patrol_prefetch_20261111`. 기존 up/down 모델·초기 상태·물성·충돌 정책을 symlink로 그대로 공유하고 시각 구간 파일만 추가했다. `camera-patrol`의 기본 `--scene-loading auto`가 이를 사용한다. `--scene-loading full`은 비교 기준, `prepare-scene-stream PROFILE PROFILE/scene_stream`은 새 자산의 일회성 변환이다. 현재5m 농장은 센서 범위 안에 전부 들어가서 기본 시각 geom31,468개를 줄이지 않았다.
- 실제 선로딩 부하 시험은30cm 구간에서 `--scene-reload-identical`로 동일 내용 재사용을 의도적으로 끄고 수행했다. 다음 GPU 장면 준비8.5~8.7초를 기존 실행과 겹쳤고, 최종 GUI 두 번의 전환0.31/0.47ms·로딩 대기0초·coverage miss0이었다. 입구0..0.85m에서5회 진입/33개 영역 수확, 전체15개 물리 구간 상태·접촉·명령과24회 RGB·depth·mask·관측이 전체 장면 기준 및 GUI/headless에서 정확히 같다. 실제 열매 분리나6송이 전체 수확이 아니다.
- 37개 회귀 통과. 단위 시험에 늦은 preload 비차단, GPU만 준비되고 파일이 미완료인 경우, 이전 장면 정리 지연, 유효 범위 소진, 실제 카메라 위치 이탈, 원본 변경 검출, 먼 식물10→100개 증가 시 첫 구간4개 유지 등을 포함했다. 초기 catalog의 미라벨 형상0 ID 검증 오류는 원자료를 보존하고 수정했다.
- 기본 headless는 초기 준비15.60→11.88초(카메라12.73→8.97초). 로딩 후 전체는54.46→55.39초/42.77sim초로 거의 같다. **경로 검사/기록/관측의 기존 정지는 미해결**이며 전체 실행 가속이나 무지연을 달성했다고 보고하지 않는다. 시각 선로딩만 구현했고 대규모 물리 창 이동/기존0.85m를 넘는 전체 레일 수확은 이번 범위가 아니다. 오프라인 최초 변환은 여전히 원본 전체를 읽는다.
- 원자료 `/root/farmily_runtime/newton/runs/20261111_scene_prefetch`, 검증 `nvidia-sim/validation/s300_scene_prefetch_20261111.json`. 영상 `prefetch_final_live.mp4`와 정리 스레드 개선 전 `prefetch_live.mp4`는 모두 실제 브라우저 wall-clock 녹화이며 기존 계획 대기를 포함한다(13px 자막). 이전 미커밋 작업·데이터 보존. 새 커밋/푸시는 요청되지 않았다. 검증용5907은 시험 후 종료했고, 기존 사용자5905 실행은 중단하지 않았다.

## 2026-11-11 이전: 이동 중 관측·짧은 수확 동작·GUI 일정 배속

- 사용자 요청은 주행의 반복 정지와 수확 시작 지연 제거다. 기본 `camera-patrol --motion-profile fluid`로 변경했다. 이동 중 실제 장착 카메라를 약15cm 간격으로 확인하며, 중간 관측마다 속도를0으로 내리지 않는다. 관측이 수확/정밀 확인을 요구하면 측정 상태에서 감속하고 정차 시야를 다시 확인한다. 실제 GPU 명령으로 움직이며 좌표를 순간이동하거나 화면만 이어 붙이지 않는다.
- 기존 IK 경로의 이동량에 맞춰 시간을 정하고 반복 대기/복귀 후 닫기를 제거했다. 그리퍼는 수확 사이에 열린 상태를 유지한다. 1회 기존24.35초→이번5.62~6.85초. 주행 명령 최대0.12m/s·가속도0.3m/s²는 시뮬레이션용 한도이며 실제 RB5/기체 안전 한도의 증거는 아니다. `--motion-profile legacy`는 기존 타이밍이다.
- GPU 엔진/kinematics CUDA graph/고정 deterministic adapter/검증된 정적 metadata를 재사용한다. 동일 명령의 기존5개 구간은 최적화 전후 모든 물리 상태·접촉 배열이 완전히 같다. 동적 관측 종단은 원래 전체 계획과 실행된 정확한 prefix를 함께 저장하고 다음 구간에서 검증한다. GUI는 기본1배속·30Hz 목표, `--display-rate 0`은 최대 처리량이다. batch는 GUI/headless 공통2로 설정한다.
- 확대84mm 영역/촘촘한 송이/주줄기 충돌/±15도/엘보우 위/1mm 기준을 그대로 두고 입구0..0.85m에서5회 동작·33개 영역 수확을 확인했다. 실제 과실 분리나6송이 전체 수확은 아니다. 새 headless와 카메라 회전/확대를 포함한 새 GUI의 전체15개 구간 상태·접촉·수확 처리가 배열 단위로 같다.
- **무지연 목표는 아직 완전히 충족하지 않았다.** 정상 구간 연결도 준비·기록·관측·계획 시간이 남고, 충돌 경로를 여러 개 검사할 때 기존 GUI 검증에서는 최대6.84초 멈췄다. 초기 camera/model 준비 약16초, 첫 GPU 준비 약6초도 별도다. 깨끗한 headless는42.771sim초/54.727wall초(RTF0.782, 최초 준비 별도). 수확 타이밍을 줄여 총 작업 시간을 줄인 것이므로 예전 긴 타이밍의 RTF와 직접 엔진 가속률로 비교하지 않는다.
- 실제 기록 영상은 `/root/farmily_runtime/newton/runs/20261110_fluid_patrol/fluid_harvest_actual.mp4`, 검증은 `nvidia-sim/validation/s300_fluid_patrol_20261111.json`이다. 영상은1배속 시뮬레이션 시간이며 wall-clock 계획 대기는 포함하지 않는다. 테스트88개 고유 항목 통과, plan adapter6통과/2환경skip. `patrol_current`는 기존84mm 자산 그대로,5905는 새 코드의 시작 대기 화면으로 교체한다. 이전 미커밋 작업 보존, 새 커밋/푸시는 요청되지 않았다.

## 2026-11-10 최신: 수확 영역 높이48→84mm

- 사용자 녹색 표시처럼 롤러 끝 캡까지 포함하도록 실제 영역 판정을 확장했다. CAD 캡 범위는 Tool_fixed Z=-4..79mm, 새 박스는 X=-48..48/Y=0..158/Z=-5..79mm, 크기96×158×84mm다. 기존37mm 높이 중심을 유지한다. 화면 노란 박스/저장 영상 박스/카메라 집단 선택/실측 도달 위치의 열매 중심 판정이 같은 선언 범위를 사용한다.
- `HarvestGroupSelector`는 명시한 `tool_height_interval_m`을 사용한다. Newton 영역 모드에서 현재 자산의 선언값을 전달하며 기존 물리 수확 선택기의 기본13..61mm는 유지한다. 이전 자산·기록도 선언한48mm 박스로 재생/재시험할 수 있게 `volume_bounds`를 사용한다.
- 새 자산 `/root/farmily_runtime/newton/assets/patrol_capture_height84_20261110`으로 `patrol_current`를 교체했다. `newton_geometric_harvest.py SOURCE OUTPUT --volume-only`는 검증한 물리 파일을 공유하고 영역 메타데이터/서명만 새로 만든다. 두 엘보우 모드의 MJB/XML/초기 상태/명령 바이트가 직전 촘촘한 송이 자산과 정확히 같다. 농장/로봇/충돌체/접촉 기준은 바꾸지 않았다.
- 새 GPU 샘플2회 진입/13개 영역 수확 정상 완료, 그중7개가 이전 Z=13..61mm 바깥에 있어 확대 영역의 실제 적용을 확인했다. 6개 물리 구간 모두 유한 상태/버퍼/접촉 검사 통과, 활성 관통0mm다. 시뮬레이션62.343초/로딩 후 전체42.942초(RTF1.452)이며 실제 물리 열매 분리 증거는 아니다.
- 관련 회귀66개 통과. asset Python에서 선택/카메라45개, native Python에서 영역/GPU 순회21개를 검사했다. 최초 잘못 묶은 native 환경에서는 USD `pxr` 부재로3개가 실패했으며 올바른 두 환경으로 전부 재검증했다. 원자료 `/root/farmily_runtime/newton/runs/20261110_capture_height84`, 검증 `nvidia-sim/validation/s300_capture_height84_20261110.json`, 실제1배속/13px 영상 `height84_harvest_actual.mp4`다.5905는 새 박스의 시작 대기 화면으로 재실행한다.

## 2026-11-10 최신: 방울토마토 송이를 촘촘하게 수정

- `dense_cherry_v1`을 새 농장 생성의 기본으로 적용했다. 전체 68그루/106송이/1,060개 열매를 다시 구성했다. 열매 지름 약20~27mm·형상·색·ID는 유지하고 송이 안 배치를 압축한다. 실제 삼각형 표면의 최근접 간격은2.30~5.46mm이며, 모든 열매 쌍의 겹침이 없다. 대표 직선 송이의 열매 중심 높이 범위190.07→127.22mm, 중심줄기251.37→162.95mm, 주줄기 연결가지109.66→76.76mm다. 주줄기·잎 템플릿은 정점/면/색이 원본과 정확히 같다.
- 원본 GLB에 덮어쓰지 않고 `dense_truss.py`에서 열매/꽃받침을 강체 이동하고 연결 가지의 중심선을 함께 옮긴다. `latest_farm.build_latest(..., truss_profile='reviewed_original')`로 이전 형상도 새로 생성할 수 있다. `prepare_dense_patrol.py`는 기존 생성기/변환기를 연결하며, 중단된 plant19 압축은 보존 후 재생성하고 완료된17/18단계를 SHA 검증해 재사용했다.
- 현재 자산은 `/root/farmily_runtime/newton/assets/dense_trusses_20261110/patrol`이며 `patrol_current`를 이곳으로 교체했다. 낮춘 평판 리프트/70mm gripper inset/손목 위 세로 하향15도 카메라/두 엘보우 모드/주줄기만 충돌/1mm/1024버퍼를 유지했다. 활성90개 열매의 manifest·native·visual 중심 오차는 최대4.41e-12m이며 전체1,060개 GT ID/숙도 색이 같다.
- 새 모델로 입구부터 실제 Newton GPU·장착 카메라 시험: 2그루의3송이를 대상으로6회 진입/21개 영역 수확,16개 물리 구간·22,273상태 정상 완료, 활성 관통0mm, 최대 후보572/1024. 시뮬레이션175.223초/촬영·계획·저장 포함114.755초(RTF1.527), 최초 준비15.716초 별도다. 다른 작업량인 이전6회 시험과 직접 속도 향상 비교하지 않는다. 기존 영역 수확 정책의 결과이며 실제 과실 분리/송이 전체 완료를 뜻하지 않는다.
- 시험 중4회15개 뒤 낮은 기억 송이가 현재 카메라에 안 보일 때 동일 정차 촬영을 반복하는 Newton 연결 누락을 발견했다. 기존 native 실행기의 `record_local_failure` 처리와 같은 정책으로 새 관측 실패를 기록하고,5cm 전진 재관측/2개 실제 시점 후 유한 보류한다. 해당 송이를 지우거나 수확 처리하지 않는다. 수정 전 진단은 의도적으로 종료하고 원본·중단 사유를 보존했다. 수정 후 독립 재시험에서 다음 그루 수확까지 완료했으며, 수정 개입 이전11개 물리 구간의 모든 상태 배열이 진단과 정확히 같다.
- 회귀56개 중55통과/구형 상판 카메라1skip. 검증 `nvidia-sim/validation/s300_dense_trusses_20261110.json`, 원자료 `/root/farmily_runtime/newton/runs/20261110_dense_trusses`. 영상 `dense_trusses_harvest_actual.mp4`는 새 측정 상태의1배속/13px 자막이며 PIP는 시각을 표기한 직전 정차 관측이다.5905는 새 모델의 시작 대기 화면으로 실행한다. 기존 미커밋 변경을 보존했고 이번 요청에서는 커밋/푸시하지 않았다.

## 2026-11-10 최신 수정: 연속 수확 5번째 GPU 후보 버퍼 초과

- 사용자 5905 실행은 4회/12개 영역 수확 뒤 5번째 준비 구간에서 `solver_capacity_overflow`로 중단됐다. 순회 실행기가 `nconmax=256`을 고정했고, broadphase 후보가 257개부터 초과했다. 실제 접촉은 최대 1개이며 활성 관통 초과는 없었다. 원본 `20261110_roller_projection/gui_preview`의 중단과 상태를 보존했다.
- 순회 기본 `--nconmax`를 1024로 수정했다(`--njmax` 1024 유지). 접촉 후보도 같은 버퍼를 쓰므로 실제 접촉 개수만으로 용량을 정하면 안 된다. 실행 요약에 후보/접촉/제약 최대치와 각 버퍼 한도를 추가했다. 물리 모델·충돌 정책·1mm·솔버·시간 간격은 그대로다.
- 원래 실패 008구간을 직전 정상 007말단에서 독립 재검증했다. 수정 전 유효 prefix 394상태가 완전히 같고, 전체 완료/후보 최대 310이다. 새 입구부터 headless/GUI 각각 6회·18개 영역 수확(2송이 대상), 전체 10구간 완료/활성 관통 0mm/최대 후보 639를 확인했다. GUI 카메라 회전·확대 5회 포함, 두 실행의 전체 상태·접촉 metrics·수확 처리가 배열 단위로 같다. 6송이 수확이나 실물 분리는 아니다.
- 회귀 18개 통과. 원자료 `/root/farmily_runtime/newton/runs/20261110_patrol_capacity_fix`, 검증 `nvidia-sim/validation/s300_patrol_capacity_fix_20261110.json`. 정상 5·6번째 실제 영상은 `fifth_sixth_harvest_fixed_actual.mp4`다. 5905는 수정된 시작 대기 화면으로 재실행했다. 더 큰 모델의 무조건 성공을 보장하지 않으며 기존 overflow 중단 검사는 유지한다.

## 2026-11-10 이전 샘플 수확: 보정한 그리퍼로 첫 진입 완주

- `patrol_roller_projection_20261110`의 엘보우 위 시작 상태에서 새 Newton GPU/장착 카메라 순회를 실행했다. 입구0→0.15→0.30→0.45m 이동 후 Plant17/Truss02 첫 수확 동작1회와 복귀를 완료했다. 기존 영역 수확 규칙으로 Fruit01/02 두 개를 처리했으며 실제 열매 분리는 아니다.
- 5개 물리 구간 모두 완료, 활성 관통0mm, 유한 상태·버퍼·실제 진입각/엘보우 조건 통과. 목표 위치오차0.0198mm, 최대 진입 yaw0.00793도. 원래 주줄기/로봇 충돌과1mm 기준을 유지했다.
- 시뮬레이션37.99초 / 촬영·계획·저장을 포함한 실행32.85초(RTF1.16), 초기 준비15.79초 별도. GPU 계산만7.19초(RTF5.29), 수확 구간24.35초/계산3.33초(RTF7.32). 접촉 수확·전체 레일 성능으로 일반화하지 않는다.
- 원자료 `/root/farmily_runtime/newton/runs/20261110_roller_harvest_sample/up`, 검증 `nvidia-sim/validation/s300_roller_harvest_sample_20261110.json`. 영상은 상위 폴더의 `entrance_to_harvest_actual.mp4`와 `harvest_closeup_actual.mp4`이며 동일 실제 상태의1배속 재생이다. 카메라 PIP는 표기된 직전 정차 관측이다. 시각 확인용5905 미리보기는 별도로 유지했다.

## 2026-11-10 최신 Newton 형상: 롤러 돌출 위치 보정

- 사용자가 직전 120mm 후퇴가 과하다고 수정 요청하여, 그 위치에서 그리퍼 전체를 **앞으로 50mm** 보정했다. 원래 기준 inset은 70mm, flange/tool 원점 수평 간격은 80mm다. 90도 어댑터 연결판·조인트·충돌체·질량도 함께 재생성했다. 손목 위 세로/하향15도 카메라의 고정 변환과 낮춘 평판은 유지했다.
- `patrol_current` → `/root/farmily_runtime/newton/assets/patrol_roller_projection_20261110`. 포트5905 시작 전 미리보기를 새 모델로 다시 띄웠다. 두 엘보우 모드·개폐·손목±15도 검사10통과/구형 카메라1skip, 기존 대비 정확히 tool +Y 50mm 이동 및 카메라 extrinsic 불변 확인.
- 새 GPU 리프트/손목 시험16.50초 완주, RTF3.72, 활성 관통0mm. 전체 수확 순회를 재검증한 결과는 아니다. 원자료 `/root/farmily_runtime/newton/runs/20261110_roller_projection`, 검증 `nvidia-sim/validation/s300_roller_projection_20261110.json`. 이전 모델·미커밋 변경 보존.

## 2026-11-10 이전 Newton 형상: 안으로 당긴 그리퍼·손목 위 카메라

- 그리퍼 전체를 손목 쪽으로 120mm 이동했다. 고정 90도 어댑터를 유지하면서 수평 flange/tool 간격 150→30mm, 모든 tool body/joint/collider 및 어댑터 질량을 함께 갱신했다. 낮춘 평판 상판은 그대로다.
- 카메라는 상판에서 손목 link5의 상단 cap 위로 옮겨 세로·하향 15도로 고정했다. 실제 하우징 최하단 기준으로 받침 높이를 맞췄다. 위/아래 모드는 동일 extrinsic이며 아래 모드 카메라 정렬은 실제 base pan으로 수행한다. 정면 보정에 명시적 하향 목표각을 추가하고 원래 광축 오차 검사를 유지했다.
- 현재 `patrol_current` → `patrol_inset_wrist_camera_v3_20261110`. 5905에 새 시작 전 미리보기를 띄웠다. CAD/FK 10통과·구형 카메라 검사1skip, pan18/보정27시험 통과. 새 GPU 리프트/손목 및 레일 이동 2회 완주·활성 관통0mm, 카메라 익음 proxy 빨강43개 관측. 수확 실적이나 전체 순회 완료를 뜻하지 않는다.
- 원자료 `/root/farmily_runtime/newton/runs/20261110_inset_wrist_camera`, 검증 `nvidia-sim/validation/s300_inset_wrist_camera_20261110.json`. 이전 모델·중간 오류/진단 자료 및 미커밋 작업 보존. 새 commit/push 요청은 없었다.

## 2026-11-10 이전 Newton 형상: 낮춘 상판·중앙 카메라

- 브랜치 `codex/s300-newton-standalone-20261108`. 기존 미커밋 작업과 결과를 보존했다. 사용자가 지정한 낮은 위치로 상판/수직 RB5를 300mm 내려 삼각 cantilever를 없애고 660×230.5×10mm 가정 평판을 적용했다. 이동 조인트·충돌체·질량/관성 추정도 같은 형상으로 변경했다.
- 이전에 요청한 손목 고정 90도 브래킷과 위/아래 시작 모드를 유지한다. 카메라는 팔에서 분리하여 낮춘 상판의 작물 쪽 변 중앙에 세로로 고정했다. RGB-D/GT 광학 프레임과 실제 시각 장착을 함께 바꿨다.
- 기본 `patrol_current` → `patrol_lower_plate_camera_20261110`. 실행 `./nvidia-sim/run_s300_newton.sh camera-patrol`, 포트 5905. 새 GUI는 시작 전 미리보기 상태로 띄웠다.
- 8개 CAD/FK 회귀 통과. 새 GPU 리프트/손목 시험 16.50초·RTF3.72, 별도 리프트/레일 촬영 이동 14.50초·RTF3.87, 모두 활성 관통 0mm. 새 촬영 위치에서 익음 proxy 빨강 30개 관측·로봇 자체 가림 0픽셀. 전체 수확 순회 완료나 실물 분리 성공의 증거가 아니다.
- 상세 `nvidia-sim/S300_NEWTON_CAMERA_PATROL_KO.md`, 검증 `nvidia-sim/validation/s300_lower_plate_camera_20261110.json`, 원자료 `/root/farmily_runtime/newton/runs/20261110_lower_plate_camera`. 최신 요청은 형상 수정이며 이번 변경을 새 commit/push하지 않았다.

## 2026-11-08 커밋·푸시용 작업 상태 보존

- 사용자가 지금까지의 작업 전체를 커밋·푸시하도록 요청했다. 현재 브랜치는 `codex/s300-side-mount-patrol-20261023`이며 카메라 순회·수확, 측정 상태 재개, 웹 PIP·구간 선로딩, 관련 회귀시험과 검증 문서를 함께 보존한다.
- 이번 검증은 실제 물리 실행을 새로 시작하지 않은 오프라인 회귀시험이다. 수확 관련 69개 모듈에서 947회 실행/930통과/환경 의존 17skip, 웹 Python 48통과, JavaScript 7개 시험 파일 통과. 오래된 AST 조건문 탐색과 누락된 CLI 기본값 때문에 실패하던 테스트 3곳을 현재 실행기 구조에 맞췄다. 결과는 `nvidia-sim/validation/s300_commit_snapshot_20261108.json`에 기록했다.
- 마지막 저장 문서의 실적은 **5송이에서 27알 분리(익은25·우발적 미숙2)**다. 6송이 목표, 송이 전체 수확, 포집 성공으로 승격하지 않는다. P22의 고정 연결과 기존 탄성 연결 차이 및 후속 이식·수확 시험은 미완료 상태로 남긴다. 아래 과거 기록의 ‘진행 중’ 표현은 해당 기록 시점의 상태다.
- 현재 `/mnt/nas_rdv_md3` 원자료 저장소가 연결되지 않아 실제 USD·체크포인트·동영상은 이번에 재검증하거나 Git으로 전송하지 않았다. 로컬 렌더 가상환경과 Git 백업 bundle도 Git 대상에서 제외하고 그대로 보존한다.

## 2026-10-24 최신: 실제5송이27알, P22 연결모델 차이 조사 중

- 실제 native 분리 **5송이·27알 = 익은25 + 우발적 미숙2**. GPU 복구 이후 새분리는 Plant20/Truss01의 익은7알이다. 이 송이의 초기 익은7/7은 분리됐고 미숙3알은 남는다. 전역6송이·송이전체10알·포집 성공은 아직 달성하지 않았다.
- **P22 실제 수확동작은 정상 완주했지만 분리0**. 현재 시야로 선택한1회 진입에서 닫기/열기2회와 후퇴를 실제49.1333초/2949상태/99RGBD·GT 노출로 저장했다. 초기검사·실제자가충돌 PASS/native 종료0이며 동작 완료를 분리 성공으로 세지 않는다. 최신 수락된 말단은 `/root/docker_share/mujoko_debugging_data/20261024_013419_gpu_restored_camera_harvest/p22_camera_harvest_from_retained_lift04/trials/p22_resume_camera_harvest/route_checkpoint`다. 다음 시험은 이 말단·현재 접촉 증거를 기준으로 하고 이전004/초기 상태로 롤백하지 않는다.
- 중요 모델 차이: 실제 P22/T01 열매연결은 FixedJoint·3N·0.08Nm였고, 기존 elastic·0.008Nm 설정은 T02에만 적용돼 있었습니다. 따라서 C6 무분리를 위치·진입만의 문제로 단정하거나 의도한 elastic·0.008Nm와 동일조건 비교라고 해석할 수 없습니다. 대상별 연결모델과 attachment transfer 적용 범위를 조사하며 C7 실제 실행은 보류합니다. C6 원본·영상·native분리0 기록은 그대로 보존하고 물리모델을 바꿔 성공을 만들지 않았습니다.
- 추가 모델감사: P22는 과거 미수확archive의 원래Fixed 연결을 보존했고 T01에 attachment transfer가 적용되지 않았습니다. 이를 임의 구현오류로 단정하지 않습니다. 읽기전용 실제USD 감사에서 이전 Plant20/T01 익은7개 성공은 세 실행 모두 d0378cef…의 PhysicsJoint·3N·약0.008Nm/±1° spring 설정임을 확인했습니다. 현재 P22/T01 Fixed·3N·약0.08Nm와 다른 모델입니다. 다음은 기존 eligible_attachment_transfer를 명시 적용하는 별도3N/.008 모델로, 깊이15mm를 유지하고25mm안은 보류합니다. 원본과 같은physics라고 부르지 않습니다. 근거 successful_T01_and_P22_actual_USD_joint_comparison.json/md.

- 앞서 T00의19계획거절(닫기wire11/접근주줄기4/예산2/경로미발견2)과 T02의 집단창 부족(간격57/58mm>48mm)은 다른 병목이다. 관측0진입을 물리실패·수확불가능으로 바꾸지 않았다. P22 시작의 import 오류와 기존 retain-lift-drive-reference 누락 통제중단도 원본·증거를 보존했다. 통제중단 초기화의 내부 진단스텝 가능성은 명시하고 그 상태는 정규 연속경로로 수락하지 않았다. 이후 같은 P22 자산에서 해당flag를 보존한 실제1cm 상승/freshRGBD를 정상 검증했다.
- 실제 native capture41형상/part163의15조각을 확인했다(sourceUSDC5f74ed6c…/capture31e345ba…는 T00 원자료). 현재 native조각의 FCL 계획교차도 실제 PhysX 접촉과 구분한다. 과거 누락cache의 단일 CAD envelope/source mesh6.9103배는 역사적 proxy 오차이며 SDK hull 부피비가 아니다. wire native cooking은 별도미검증/FCL depth는 물리침투·힘이 아니다. 근거 `part163_actual_native_capture_publication_receipt.json`.
- 웹은 이전 canonical6 + fresh30 = **36개 실제 구성구간·678개 RGBD/GT 노출**이다. 정상종료기록29개(스캔·이동 포함)와 복구된 원래 중단prefix1개를 구분한다. NameError partial5초는 별도진단·정확말단 인계를 표시하고, 원래 completed=false/native_running 표시를 덮어쓰지 않았다. 기본 웹 선택도 보존했다.
- 전체17개 영상은 `published_review_videos.md/json`에 전체 절대경로로 정리했다. 새 P22 영상은 실제 이동3회·리프트·추가1cm준비·진입을7구간/201노출/96.7833초로 연결한다(30fps 경계반올림 영상96.834초). T01 익은7개 영상은108.8초/220노출이다. 모두1배속·12px·전체 decode/입력SHA 검증이며 영상 재편집으로 새수확을 더하지 않는다.
- 실행은 기존 native PhysX CPU60Hz/threads1/TGS64·4 + RTX 카메라2Hz다. 전체농장 시각과 타깃송이·자기주줄기 중심의 기존충돌범위를 유지했다. 게시 과정에서 충돌응답·1mm·actuator를 바꾸지 않았으나 자산별 native분리조건이 모두 동일했다는 주장은 철회한다. 현재 P22/T01은 FixedJoint·3N·0.08Nm이며 기대한 elastic·0.008Nm 동일조건이 아니다. Newton GPU physics 성공을 뜻하지 않는다. 각 프로세스의 solver cache는 재시작하므로 bitwise 엔진 연속성은 주장하지 않는다. `training_eligible=false`, `harvest_success=null`.

## 2026-10-24 최신: 정확한 말단 후속 완주, 다음5송이v7 진행

- 현재 정상 물리 말단은 `plant25_original_suffix_native/plant25_original_suffix_from64p766_threads1/route_checkpoint=24.033333초`다. 원본25실제64.7667초 stop에서 원래cycle clock을이어실행했고원본completed=false/stop은그대로다. 새결과completed=true/stopNone/init·actualselfPASS/새분리0. 끝간격43.999966mm, 후퇴전96mm가드PASS.
- actual1442명령/full10DOF/jaw/belt가원래frozenrunner AST와float32완전동일하다. 기존분리15개 모두유한/정상floor지원유지, 새구간raw1mm초과0. 반면최종lift실제0.133435m vscommand0.139692m의−6.256834mm 정적추종오차가있다. 명령누락/완벽추종으로해석하지않고현재실측q와native제어target을각각인계한다.
- source는현재도v5window25(c718…USDC)이며, 이최종completedCP의default `validated_initial_checkpoint` PASS 증거는 `plant25_original_suffix_final_checkpoint_proof.json`이다. 이전64.766/45.4/54.75/B27로돌아가지않는다.
- 새 `gt_row_maximum_next5_threads1_v7_campaign` 실행중. 다음은20/Truss02→24→28→29→31, 현재20창생성. current25는native5분리/익은02·08미해결상태로명시적budget-unselected에남겼고전체송이완료로세지않는다. 충돌정책타깃송이·주줄기,원래물성/60Hz/1mm/native40, threads1현재성공설정을유지한다.
- canonicalunique분리15(익은14+미숙1), 분리가발생한송이3개, 정상완료진입6회+같은진입의분할후속완료1건이다. 독립재시도/A원본/Frozen재초기화를합산하지않는다. freshRGBD/실제camera순회/NewtonGPU/포집성공은아직아니다. GPU EPERM/nativeCPU SIM-GT교사범위유지.

## 2026-10-24 현재 물리 말단: Plant25 단일 스레드 후속64.7667초

- 최신 trial은 `plant25_resume_threads1_native/plant25_resume_yaw0_seed5_threads1_harvest`, 최종 `route_checkpoint=64.7666667초`다. 이전45.4초 완료CP 또는54.75초 parent로 돌아가 연속이라 집계하지 않는다. 현재 SDK/native 프로세스는 종료됐고 정확한 이 말단의 forward suffix 복구를 구현 중이다.
- v6 yaw0/seed5는 전체self/environment2623표본 PASS 뒤 SDK첫initialize simulate에서segfault139(복원전, 내부결과unknown). 원본격리 후 같은54.75정상parent의 명시적새복구시험에서threads16→1만 변경해 exactrestore/init를 통과했다. 다중스레드가원인이라고단정하지않는다.
- 실제 첫45.4초 수확/후퇴완료: 새F03/F05분리. 두번째집단07/08은wire차단후다른heading/seed를fresh감사통과해 실행했고F04/F07도native분리됐다. 현재25누적F01/03/04/05/07=5개,19→27B→25누적15개(익은14+미숙1), 완료진입6회/분리가발생한송이3개다. 전체송이완료·포집성공이아니다.
- 전체trial completed=false/penetration_over_contact_policy. 실제stop은이미분리F04↔RowSupply_20mm 정적관수관 raw1.763595mm/0N, 기존 --released-fruit-static-farm-support-diagnostic 역할누락이었다. 정적/무시간표본/원본sourcehash 증명 후 RowSupply역할1개만기존scope에추가했고134시험PASS/원본raw와stop불변. 부착/로봇/식물/moving·unknown물체 예외없다.
- 두agent가 기존recovery/명령샘플러/감사/실행기를연결해원본cycleclock을그대로유지하는남은cycle_open→reclose→open→후퇴를준비한다. 원본cycle19.366667초에서이어가며부분열기구간을새smoothstep으로재보간하지않는다. exact64.7667만대상/원본physvalidfalse는보존하고모든stop의별도재분류증명필수다.
- 공동회귀314PASS,새native0현재. GPU EPERM/freshRGBD불가/nativeCPU60Hz/SIM-GT교사범위는유지한다. 수확뒤다음cohort는20T02/24/28/29/31이며current25를명시적예산보류로뒤에둬초기그룹을재생성하지않을계획이다.

## 2026-10-23 현재 물리 말단: Plant25 t54.75초, 계획 연결 수정 후 이어가기

- 이전27B 이후 실제25가54.75초 진행했다. 현재 source는 `gt_row_maximum_local_heading_v5_campaign/windows/000_plant_25`, trial은 `plant25_entry_retract5_native/plant25_entry_retract5_harvest`, 정상 완료 cycle CP000과 terminal route_checkpoint는 같은54.75초 상태다. **새 물리를27B 또는 이전 CP에서 시작해 연속으로 집계하지 않는다.**
- 5mm 실제 TCP 후퇴로 native40 도구↔주줄기 전체 경로 검사 통과 후 첫 진입/닫기·열기2회/후퇴를 완료했다. Fruit01이29.4833초 close 중 native 분리됐다. 누적19→27B→25는11개(익은10+미숙1), 완료 진입5회, 분리가 발생한 송이3개다. 전체 송이 완료·포집·카메라 순회 성공이 아니다.
- 다음 집단03/04/05의 계획에서 GutterSuspensionWire_005 접촉을 찾고 조기 종료한 audit(complete=false)를 전체 unknown 오류로 처리해 `gt_group_planning_inconclusive`로 멈췄다. 첫 cycle 이후 물리는 진행되지 않았다. 원본 전체 trial completed=false/stop은 그대로 보존한다.
- 조기 음성을 검증된 현재 후보 거절로만 처리하는 GT opt-in 연결 수정236시험 PASS. 나머지 미검사 경로 UNKNOWN은 유지한다. 정상 완료 cycle과 실제 terminal의11배열·제어기·분리이력·최종trace가 정확히 같을 때만 software-only stop 재개를 허용하는 별도 opt-in을 검증 중이다. default stopped-run 거부와 더 이른 CP rollback 금지는 유지한다.
- 실제 self3286표본 PASS, 주줄기↔공구 raw 접촉0. 이전10분리 과실은 모두 정상 바닥 높이를 유지했다. raw1mm초과1점은 분리 후 Fruit01↔GroundCover 접촉1.2865mm이며 기존 released-support 진단에 기록됐다. 부착 기준/물성/충돌체/분리조건은 바꾸지 않았다.
- GPU EPERM 때문에 fresh RGB-D는 불가하며 nativeCPU60Hz/SIM-GT 교사 시험이다. 다음은 현재25 남은 집단의 방향/IK 유한 탐색 후20/Truss02→24→28→29→31. 이전 내용보다 이 물리 말단이 최신이다.

## 2026-10-23 최신: 분리 과실 충돌 수정 후27B 실제 재검증 완료

- 아래27A 바닥 누락 오류는 원본을 보존하고 수정27B로 검증했다. 기존 정상19 CP001=93.3167초에서 새 모델/새 출력으로 재실행했다. frozen native-released 과실만 collider ON을 유지하며 attached 식물/다른 송이 ghost 정책과 물성/1mm 부착 접촉 기준은 유지한다.
- `gt_row_floor_retained27_b_campaign`:127.7333초 정상완료, 두 번 진입·후퇴, 새 익은 과실4개 분리, 실제 자기충돌 감사 PASS. 이전19 과실6개는7,665개 전체 frame에서 바닥 통과 없이 유지됐고 실제 바닥 접촉·힘이 기록됐다. 원본A와B는 독립 분기이므로 더하지 않는다.
- A/B의 로봇/도구/현재식물/Frozen19 56,084spec·218,213field를 비교해 물리 필드 차이가 정확6개 기존 분리 과실 collisionEnabled False→True뿐임을 확인했다. source19 CP의 관절/몸체/속도/이력 인계는 동일하다. 311회귀+별도9시험 PASS.
- 검증된19→27B 계보는 실제 분리10개(익은9+미숙1), 두 송이에서 진입·후퇴4회, local sim 시간 합221.05초다. 전체 송이 수확·전체식물 완료·포집·실물 손상 안전성을 의미하지 않는다. 이미 분리된 열매의 바닥/등록지지물 접촉은 원래 물리 응답을 유지하는 진단 범위다.
- 근거는 최대수확 원자료 폴더의 `completed19_to27b_harvest_chain.json`, `plant27b_terminal_resume_proof.json`, `released_fruit_lifecycle_actual_validation_b.json`, `plant27_floor_retained_ab_independent_review.json`이다. 다음 정상 source는 `gt_row_floor_retained27_b_campaign/windows/000_plant_27`, 최종CP는 해당trial의001이다. 잘못된27A 말단으로 돌아가지 않는다.
- 다음 유한 대상은25→20/Truss02→24→28→29→31이다. GT group heading suffix search opt-in은225회귀+독립8시험을 통과했고 새v5 캠페인을 실행 중이다. GPU EPERM으로 fresh RGB-D가 불가하므로 SIM-GT 교사 실행과 카메라 순회 성공을 구분한다.

## 2026-10-23 추가 발견: 진단27A의 이전 분리 과실 바닥 충돌 누락

- 중요: 아래의19→27 측정 상태 인계/새4개 분리는 실제 기록이지만,27A의 전체 환경은 정상으로 판정하면 안 된다. target-only 창 전환이 이미 분리된 Frozen 과실 collider까지 꺼서 이전19 과실6개가 바닥을 통과했고,127.7333초에서Z약-80,042m까지 낙하했다. 원본states/영상/모델은 수정하지 않는다. 포집/전체환경 유효성 미검증을 단순 문구로 처리할 문제가 아니라 실제 lifecycle 오류다.
- 새25 v4 준비는 native 실행 전에 중단했다. exact native 분리·완료CP·원래joint·Frozen body에 묶인 과실만 collider ON을 보존하고, 다른 부착 식물의ghost 정책은 유지하도록 수정 중이다. 물성/형상/1mm 기준/pose clamp/강제분리로 고치지 않는다.
- 수정 검증은 정상19 CP001에서 새27B로 분기해 실제 재실행한다. 이미-80km가 된27A 상태를 바닥으로 순간 이동하거나 되돌린 뒤 연속이라 주장하지 않는다. A를 보존하고 명시적인 수정 A/B 시험으로 구분한다.
- 최신 상태: 원자료 progress_status.json/current_blocker. GPU EPERM으로 새 카메라는 여전히 불가, current task는nativeCPU/SIM-GT 교사탐색이며학습자료에섞지않는다.

## 2026-10-23 최대 수확 후속: 완료된 1송이 2회 진입과 다음 창 연결

- 아래 기존 Run09의 보류/질문 상태보다 이 항목이 최신이다. 사용자 최대 수확 요청에 따라 기존 실행기·실제 native PhysX CPU60Hz·부착 과실1mm 기준으로 작업한다. GPU 장치 open EPERM 때문에 새 RTX RGB-D는 불가하며, 이번 SIM-GT 위치·색상 교사 탐색을 카메라/뉴턴 GPU 검증으로 표시하지 않는다.
- 원자료 `/root/docker_share/mujoko_debugging_data/20261023_164703_maximum_camera_harvest/index.html`. 독립 trial 재시도는 합산하지 않는다. strict7 trial은 두 송이에서7개 분리/3회 동작 완료 후 실제 개방 문제로 중단했다.
- 최신 `plant19_target_truss01_support_native/plant19_truss01_static_support_harvest`:93.3167초, 두 번 진입·각 두 번 열닫·후퇴 정상 완료, 실제 자기충돌검사 PASS. 분리6개는 익은01/02/03/05/07 + 미숙09이다. 익은04/08은 남았으며 기존 최소2개 footprint를 찾지 못했다. 전체 송이 수확·포집 성공이 아니다.
- 이 trial은 기존 선택 정책인 타깃 송이+자기 주줄기만 충돌한다. 전체 농장 시각 형상 유지. 이미 native 분리된 과실이 등록된 정적 재배판/거터 지지물에 닿는 경우는 원래 충돌 응답·힘·침투를 보존하며 진단 기록한다. 부착 과실/로봇/미등록·동적 물체의1mm 가드는 유지한다.
- 원본 중단과 비교해 첫25.1333초/1509표본 전체 state·command·phase·belt는 bitwise동일,39606접촉점도 동일하다. 원본 stop을 삭제하지 않았다. 최종 CP001=93.3167초의 모델/몸체/관절/기존 분리/실제 audit를 검증했고 더 이른 상태로 롤백하지 않았다.
- `--target-truss-windows`로 기존 다음 식물 창 전환을 재사용한다. near-truss 강제 옵션만 생략하고 정확한 plant/truss/body/joint를 검증한다. 첫 후속25창은 실제 skin 변형이 session layer에 있어 root-spec guard에 걸렸다(native0). 기존 Flatten→Stage.Open으로 합성 형상 보존 후 엄격한 편집 검사를 적용했고 재현/원본불변 시험 포함112개 PASS. 이전 오류 결과 보존.
- `gt_row_target_only_next6_v2_campaign`은 위 정확한 최종 CP에서25/24/27/28/29/31 순서를 시도한다. 현재 진행 상태는 row_campaign.json 참조. source19 다른 두 송이와 미선택 식물을 완료로 세지 않는다. 측정 상태·제어기 전달이며 PhysX 솔버 캐시는 새 프로세스에서 재시작한다.
- 전체 웹 재생 `http://lab.farmily.ai:8090/gripper-design/harvest-replay/?run=max_teacher_plant19_truss01_6detach_20261023`의5600프레임/270몸체/전체54.2M삼각형 검증 PASS. 새 영상은 실제 저장 상태 발췌이며 새 물리·새 RGB-D가 아니다. 영상 전체 목록은 원자료 published_review_videos.json/md.
- 관련 회귀246개 PASS와 창 전환112개 PASS 로그를 각각 보존했다. `training_eligible=false`, `harvest_success=null`. 기존 대량 dirty 작업 보존, 이 요청에서 새 commit/push하지 않았다.

## 2026-10-23: RB5 측면 장착과 카메라 증분 순회 (실제 2열매 분리, 6송이 미완료)

- 작업 브랜치 `codex/s300-side-mount-patrol-20261023`, 시작 HEAD `74a9098915fd4d052716ce7e7ecb82a03bd87c89`. 이전 미커밋 변경과 원자료를 보존했다.
- 사용자 표시의 돌출 상판/삼각 받침(CAD4/10)을 제거하고 실제 이동 carriage 수직판에 RB5 전체 체인/베이스 조인트/그리퍼를 함께 옮겼다. 실제 리프트 유지. 장착판 질량은 제거 형상을 제외한 CAD 체적 비율 추정이며 하드웨어 강도 인증은 아니다. link1 고정 세로 RGB-D는 새 ready FK에서 왼쪽 작물을 정면으로 본다.
- 새 기본 `bash nvidia-sim/run_s300_harvest.sh camera-patrol`: 레일 입구0m부터 실제 카메라/깊이/GT 관측을 사용해 보이는 익음 proxy 집단을 기억하고 최대15cm(명령 상한20cm)씩 전진한다. 전체106송이/68주 인벤토리는 소속/한쪽 줄 정책에만 사용하며 숨은 과실 중심으로 목표를 정하지 않는다. 같은 식물의 미해결 관측 집단을 먼저 처리한다.
- 기준은 기존 실제 native PhysX CPU60Hz, RGB-D2Hz,1mm 관통/실제 벨트2회 닫기·열기/잎 무시/기존 주변 충돌체다. 이번은 Newton GPU 성능 검증이 아니다. 측면 장착 전용 수확 동작의 실제6송이 검증은 진행 중이며 성공으로 보고하지 않는다.
- 보이는 집단의 새 IK 경로가 저장된 계획/장면SHA에 묶인 충돌 증거로 거부될 때만 같은 집단·방향에서5/8 IK branch를 추가 검사한다. 원래 최대12제안과 기존 전체 경로 검사 유지. 실제 물리 중단은 재시도/롤백하지 않는다.
- 초기 배치 오류는 로봇 접근 실패와 분리한다. 독립 idle/warmup에서 이미 관절이 끊어지는 full-plant는 이 캠페인에서 운영 보류하고 다른 미시도 송이도 같은 모델을 반복 시작하지 않는다. 원본 증거와 관측은 유지하며 불가능/독립 타깃 실패/수확으로 세지 않는다. 이전 식물은 저장 실제 상태로 동결하고, 새로 관측된 미수확 식물만 제한적으로 재활성화한다.
- 웹 재생은 다음 구간 모델/상태/카메라와 일부 PIP 프레임을 미리 읽는다. 실제 브라우저에서 다음 구간 첫 프레임의277바디 좌표 오차0, 시점/PIP 시각/공유 모델1회 다운로드/자동 연결 확인. 독립 재시험이나 중단 구간은 자동 연결하지 않는다. 동일 모델의 새 저장상태는 정확한 모델/바디순서SHA 확인 후 GLB만 공유하고 각 구간을 독립 검증한다.
- 최신 Run09는 Plant20/Truss02의 첫 카메라 제안에서 기존 준비 연결·전체 self/environment 검사를 통과해 실제로 접근/벨트/닫기를 실행했다. 선택한 Fruit01은23.8167초, Fruit02는24.3333초에 native joint가 새로 분리됐다. 그러나24.55초에 이미 분리된 Fruit02↔같은 주줄기의 다른 Truss00 Fruit09 접촉(전체4접촉점 최대3.7332mm/0.07478N)으로 기존1mm 규칙이 중단시켰다. 실제 분리2개와 수확 구간 완주는 다르므로 완료 송이 집계는0/6이다. 영상·기준을 바꿔 성공으로 처리하지 않았다.
- Run09의 실제 마지막 상태는 `patrol_run_09/trials/segment_001_harvest/segment_001_harvest/route_checkpoint`의24.55초다. 완전한 native q/qvel/몸체 속도/plant preload/제어기 목표와2개 분리 이벤트가 있다. 정상5초의 `cycle_checkpoints/000`으로 되돌려 연속 실행했다고 주장하면 안 된다. 분리된 과실↔다른 송이 열매 접촉을 별도 진단으로 기록하며 계속할지 사용자에게 확인 중이며, 미응답을 허가로 간주하지 않는다.
- 별도6개 관측 기반 얕은 진입 오프라인 질의(35/20/15mm)는 전체 경로 통과0개다. 일부는 준비 자세 자기충돌, 일부는 같은 주줄기의 다른 송이 과실/중심가지에서 막혔다. 실제 physics는0회이며 특정 유한 표본 실패를 수확 불가능으로 바꾸지 않는다. 원자료 `shallow_observed_probe/index.html`에 실패도 모두 보존했다.
- 웹7개 JavaScript 회귀 모음은 통과했다. 앞선 실제14개 prefetch 브라우저 항목도 통과했지만, 최신 중단 Run09의 별도 소프트웨어 브라우저 검사는120초 안에 ready에 도달하지 못해 **미확정**이다(page JavaScript 오류 기록0). 이 항목을 통과로 보고하지 않는다. 실제 상태 MP42개는 첫/중간/끝 모두 검은 화면 아님과277바디 위치·회전 행렬 오차0을 독립 확인했다.
- 실제 결과/로그/소스 스냅샷: `/root/docker_share/mujoko_debugging_data/20261023_side_mount_camera_patrol/index.html`. 실행별 `controller_source/`, `camera_patrol.json`, `camera_patrol_queue.json`, prior-campaign SHA체인에 보존한다. `training_eligible=false`, `harvest_success=null`. 현재 동작의 단계별 실제 수확 집계와 영상은 해당 인덱스가 기준이다.
- 조립 웹: http://lab.farmily.ai:8090/gripper-design/robot-assembly/ ; 실제 재생: http://lab.farmily.ai:8090/gripper-design/harvest-replay/?collection=camera_side_patrol_20261023 . 상세 실행/보류/재개/웹 자동등록: `nvidia-sim/harvest_s300/CAMERA_PATROL.txt`.

## 2026-10-23: 정면 카메라·전체 기록 탐색 및 지나친 송이 원인 확인

- 기존 정차점 순회는 카메라 감지 즉시 정차하는 알고리즘이 아니다. 실제 예시22→25는0.762m,25→27은0.499m,27→29는0.570m이며 후속28→20 재시험에는약1.998m역주행도 있었다. 기본 시험은 정차당 실제 수확1회 제한이었다. 전역 이동간격 제한·관측 송이 대기열·자동 수확 정차점 최적화는 이번에 구현 완료하지 않았다.
- `run_motion.py --camera-harvest`의 스캔 기본은 `--camera-scan-view row-front`: 카메라 고정 장착을 유지하고 실제 base yaw로 레일에 수직인 작물 방향을 본다. `target-point`는 이전 사선 방식을 재현한다. 수확 중에는 link1과 함께 회전하며 독립 짐벌이 아니다. 원본 카메라 영상/행렬을 수정하지 않았다. 실제 스캔의 정면 대비 yaw45.4462°→-0.00308° 확인. 새 시험 정차는1.600→1.873662m,약27.4cm이며 전역20cm정차 규칙이 아니다.
- 웹에 '전체 기록' 시간 막대 추가. 기존14구간499.5초(8분19.5초)를 구간을 넘겨 탐색하며 실제 source clock과카메라를 함께 전환한다. 독립 재시험의 자동 정지·수동 선택 안내 유지. 묶음 전체를 하나의 연속 물리 실행으로 해석하지 않는다. 실제 브라우저에서55초 선택→두번째 구간0.6초 및GT카메라decode PASS.
- 새 원자료 `/root/docker_share/mujoko_debugging_data/20261023_front_camera_review/`. 첫 정상 시험은13.633초/819상태,정면스캔완료 후고정팔자세19계획(15self여유거리,4IK)거절,수확0. 시야86열매중14개만후보:다른식물56,등록된현재대상외10,익음정책제외6. '화면에 보이는 모든 송이 자동 수확'은 미완료이며 제외된 송이를 수확 불가능으로 분류하지 않는다.
- 추가 독립 시험은 기존 자동IK·OMPL·실제cooked tool기하를 재사용했다. 6번째 후보의 전체경로 검사를 통과하고 실제 접근/닫기까지 진행,37.5초에Truss_02_Fruit_02의native joint분리1건.37.7667초 분리열매↔RB5 link3 접촉의겹침5.35158mm로기존1mm규칙에따라중단.2267상태/77RGB-D프레임. 동작 완주·포집·6송이 완료 아님. 접촉/물성/충돌체/중단기준을완화하지않았다. 이번은native PhysX CPU60Hz+RGBD2Hz이고NewtonGPU속도검증이아니다.
- 앞선첫시작은사라진`/root/docker_share/mujoko_debugging_data_bak`경로때문에검사inconclusive였다. NAS의기존동일SHA자료를확인하고원래경로의symlink만복구했다. 추가OMPL의첫CLI시도는필수cooked-shape옵션누락으로실행전차단되어로그보존;실제2회와구분한다.
- 새웹묶음 `camera_front_review_20261023`은독립초기상태2개기록이며기존기본5/6송이묶음을바꾸지않았다. 두원본`rgbd/analysis.mp4`와웹export/모든실패로그보존. 결과 `nvidia-sim/validation/s300_front_camera_review.json`. Python90PASS,마지막FK오류메시지수정후해당17회귀재검사PASS(중복합산하지않음),기존JS56항목+새전체시간막대모듈PASS. 개별웹export는원본전체상태/카메라/모델로독립검증한다.

## 2026-10-23: 카메라·웹 재생 기록의 무손실 저장 최적화

- 최근 수확 원자료 18.810→12.231GB, 웹 재생 자산 15.072→5.563GB, 초기 mounted-GT 전체 0.966→0.474GB. 총34.848→18.269GB, 약16.579GB(47.6%) 감소. GB는10진수이며 웹 hardlink의 동일 inode를 한 번만 센 크기다. 열려 있던 `lift_gt_final`은461.65→224.27MB. 별도 감사 로그 용량은 결과 디렉터리에서 확인한다.
- `recording_storage.py`: depth/renderer-instance/instance/semantic을 원본 NPY 바이트 그대로 `.npy.gz`에 저장한다. dtype/NaN payload/shape를 포함해 무손실이다. 동일 renderer label은 RGB-D 디렉터리 내부 상대 링크로 공유한다. 새 촬영은 최대4개 파일 I/O 작업만 병렬화하고 전부 완료 후 프레임을 공개한다. 물리·렌더러를 별도 스레드에서 실행하지 않는다.
- `compact_camera_storage.py`: 기본 dry-run, `--apply`로 완료된 결과만 이관. 모든 기존 NPY는 압축을 다시 읽어 원본 전체 바이트 일치 확인 후 교체하며 `storage.json`에 원본/압축 SHA를 남긴다. 원본 frames.jsonl/camera.json/objects.json·실제 상태·결과·이미지·영상은 변경하지 않는다. 예전 로그의 .npy 논리 경로는 `load_array`에서 gzip을 찾는다. 외부에서 np.load(path)만 쓰던 코드는 gzip.open을 사용하거나 `gzip -dk`/`--restore-arrays`로 복원해야 한다.
- 새 웹 export는 SHA를 검증한 `scene.glb.gz`만 유지하며 같은 웹 자산만 `assets/_shared` hardlink로 공유한다. 원본 NAS와 웹 파일은 hardlink하지 않는다. 다른 모델을 동일 모델로 합치지 않았다. 공유 자산은 불변으로 취급하고 수정 시 atomic replace한다.
- Python87개, 실제 JS함수56항목 통과. 실제 리프트·수확2개 기록 및 새 exporter의 임시 전체 농장 export를 독립 검증했다. 새 export 임시 파일은 검사 후 제거했다. 실제 브라우저에서 gzip 장면 로딩·RGB/GT/Depth PNG decode·10초 이동과47프레임/2761상태 확인. 소프트웨어 WebGL screenshot은 시간초과로 미확정이며 캡처 성공으로 보고하지 않는다. 초기 브라우저 검사 RAF 대기 문제는 polling=100ms로 검사기를 수정했고 원래 시도 로그도 보존했다.
- NAS 동시 이관 부하 중 교대6회 비교에서4개 카메라 배열+label 저장 중앙값은 기존0.520초→새0.299초. 파일 I/O 시험이며 전체 물리 RTF 개선 근거가 아니다. 새 native physics는 실행하지 않았다.
- 결과/복원 안내: `gripper-design/harvest-replay/README.txt`, `nvidia-sim/validation/s300_recording_storage.json`. 실제 감사·원본 SHA·metadata backup·테스트 로그는 `/root/docker_share/mujoko_debugging_data/20261022_storage_compaction/`에 보존한다. 기존 미커밋 작업을 보존했으며 이 요청에서는 commit/push하지 않았다.

## 2026-10-22: 카메라 영역 선택·연속 수확·웹 PIP (5/6송이, GPU 접근 복구 대기)

- **현재 결론:** 웹 오른쪽 RGB/GT/Depth PIP와 실제 저장 상태 선택/재생은 구현했다. 카메라 관측으로 선택한 **서로 다른5송이에서 익은 열매11개 native joint 분리**를 확인했다. 요청한 최소6송이에는 아직1송이가 부족하며, 송이 전체 수확·포집·실물 성공으로 보고하지 않는다. 아래 중간 진행 기록은 시간순 원인 분석이며, 최신 상태는 이 항목을 우선한다.
- 24번 후속의 새 OMPL 경로는 fine edge 검사와 **전체60Hz self/environment 검사6,723표본 COMPLETE PASS** 뒤 실제 실행됐다. 그러나 준비 이동16.1초에 측정 열림폭95.0825mm가 기존96mm±0.5mm guard를 통과하지 못해 정지했다. 첫 두 경계는95.8558/95.5073mm로 통과했고, 이 이동 구간의 ActuatedTool 접촉은0건이다. 손목 기울기에 따른 기존 jaw drive의 중력 탄성 오차와 맞는다. 단순 대기/감속으로 해결된다고 단정하거나 gate를 완화하지 않는다. 수확 동작·분리는 아직0건이며 경로의 기하 통과와 실제 추종 실패를 분리한다.
- 최신 실제 중단 상태는 `continuous_camera_connector24/.../route_checkpoint`의16.1초/966step, canonical SHA `f6c0afd62935b4d413eaf48e5ee32e519d457ecf6883a719305cce48bd770093`다. 완전한 q/qvel·controller·식물 preload를 보존했다. **이전 정상8.5초 checkpoint로 조용히 되돌리지 않는다.** 새 실행 자체는 앞선88표본 소프트웨어 오류 뒤 정상24에서 시작한 명시적 독립 분기이며 웹 이력에도 표시한다.
- 정지 결과 저장 **이후** Vulkan `ERROR_OUT_OF_HOST_MEMORY`가 발생했고, 현재 `/dev/nvidiactl`, `/dev/nvidia0`, `/dev/nvidia-uvm` open은 mode666에서도 `EPERM`, `nvidia-smi`는 NVML UnknownError, 최소 `cuInit(0)`은100(NO_DEVICE)이다. 호스트 가용RAM 약97GiB/cgroup OOM0이므로 RAM 소진으로 단정하지 않는다. GPU PCI 활성/driver-library570.211.01 일치지만 호스트 Xid·GPU 건강은 미확인이다. 도커 device/cgroup 접근 복구를 요청했으며 GPU reset/도커 재시작/host-key 우회는 수행하지 않았다. 이 자원 오류를 이미 저장된 열림 guard 정지의 원인으로 바꾸지 않는다.
- 중단 상태의 실제 준비 이동337개 측정 표본(16.1→10.5초)을 역순으로 만드는 **진단용** 복구 자료를 보존했다. 기존 정상 상태로의 rollback이나 실제 복구 실행이 아니다. 기존 recovery 제어기가 이번 고정 lift bias8.3152987mm를 보존하지 않으므로 실행을 코드에서 차단했다. 움직이는 terminal의 목표−측정8.3580464mm를 새 보정값으로 삼지 않는다. 다음 실제 시험 전 이 연결을 먼저 검증해야 한다. 순수FK로 본 충돌 통과 경로의 이후 구간은 jaw축이 최대약79° 기울어 기존 drive에서 약4.95mm 정적 열림 손실이 예상된다. 이는 근사 진단이며, 다음 수정은 guard 완화보다 손목/그리퍼 자세의 실행 가능성을 계획에 포함하는 방향이다.
- 최종 CPU 회귀410개 중408PASS/2SKIP(asset Python의 Torch 부재), 해당 실제 Torch2회귀는 기존 별도 환경에서 PASS. 웹 실제 함수56개 및 JS syntax PASS. 수확5송이와 마지막 중단 기록의 source summary/배열 SHA, 전체60Hz timestamp, 종료시각, actual-self 검사도 독립 대조PASS했다. 현재 GPU 장치 차단 이후 새 physics 성공이나 새 GUI/headless 동등성을 주장하지 않는다.
- 최종 웹 기본 모음은 `camera_region_route_20261022`이며 **5/6송이·목표 미완료·11개 분리**를 표시한다. 실제 정상/중단 경로14개와 독립 진단16개를 따로 선택한다. 실제 카메라 MP4는31개(수확5, 이동·스캔·중단9, 독립 진단15, 미검증 초기 원본2)이며 원본 전체경로 목록과 로그 묶음을 결과 폴더에 둔다. 88표본 오류는 완성된 카메라 MP4가 없어 실제 일부 상태3D만 제공한다. 기본 catalog와 두 보고서 HTTP200 확인.
- GPU 차단 뒤 최종 실제 브라우저 검사는 명시 SwiftShader 소프트웨어 렌더러로만 수행했다. 기본5/6 모음/처음22 정지/전체 농장/오른쪽 RGB·GT선택영역·Depth PIP/22→다음 기록 자동 전환은 실제 PASS. 이후 독립 분기에서 자동 정지·수동24선택·최종 모델캐시 검사는 CPU 렌더링 시간한도로 **미확정**이며 전체 브라우저 검사를 PASS로 바꾸지 않는다. 이 동작의 실제 소스 함수56회귀는 PASS이고, 이전 하드웨어 브라우저22→28 선택/카메라 검증도 보존한다. 마지막 소프트웨어 브라우저·driver 등 소유 프로세스10개 종료 확인. 증거 `final_software_web_qa.json`과 `20261022_final_camera_web_check/final_incomplete_default_swiftshader/default5of6_recognition_pip.png`.

- 브랜치 `codex/s300-isaac-real-lift-20261022`, 기반 HEAD `74a9098915fd4d052716ce7e7ecb82a03bd87c89`. 기존 미커밋 실제 리프트/카메라 작업을 보존했다. 이번 사용자 요청은 웹 오른쪽 카메라 PIP, 카메라로 집단 선택, 최소6송이 실제 시험 및 모든 이력 선택/재생이다.
- 기존 native `run_row_harvest.py`/`run_motion.py`/checkpoint/웹 exporter를 연결한다. **현재 실행은 Isaac6.1 native PhysX CPU60Hz, RGB-D/GT nominal2Hz**이며 이전 Newton GPU 속도 결과와 구분한다. 전체 농장 외형·현재 장면의 주변 충돌체를 유지한다. 실제 리프트/link1 세로 카메라 입력은 `20261022_065240_shoulder_rgbd/assembly_exposure`에서 이어진 모델이다.
- `--camera-harvest`: 실제 준비·개방·레일 이동·base yaw 촬영 뒤 현재 보이는 빨간 열매 최소2개를 선택한다. 기존 익음 정책∩visible GT∩valid depth∩현재 식물/허용 송이∩미분리로 제한한다. 표면 중앙값+알려진 반경은 근사 중심이다. visible Rachis/PCA 방향의 부호는 카메라→관측 중심으로 정하고, 수직에서15° 안의 가지는 관측 중심 radial yaw를 제안한다. 관측 기준0,+15,-15,+30,-30°마다 footprint와 중앙 보정을 다시 구한다. 실제 IK/자가충돌/환경 검사 뒤 기존 수평 벨트 동작을 실행한다.
- 원본 RGB/depth/instance/semantic은 보존하고, 선택한 열매의 현재 보이는 픽셀에만 얇은 분홍 테두리를 파생 표시한다. 신경망 인식·가려진 열매 중심·실물 숙도 판정이 아니다. 지도는 정차 제안/촬영 준비용이며 이동 순서 최적화는 아니다.
- `mounted_rgbd.py`/`mounted_gt.py`: RGB·depth·camera parameter·instance/label을 같은 렌더 묶음의 CPU 소유 복사본으로 읽는다. borrowed texture를 파일 저장 뒤 늦게 읽던 문제를 수정했다. 잘못된 payload는 원본/labels/오류를 보존하고 같은 물리 시각에서만 최대3회 전체 묶음을 재촬영한다. 미지 ID를 배경으로 바꾸거나 매핑/좌표 기준을 완화하지 않는다. 이전20의0.5초 센서 실패와 수정 후3초7프레임 대조는 겹치는 전체 native 상태가 bit-exact이며 별도 진단이다.
- 카메라 초기 스캔은 기존 ready/open/선택 ready-lift/rail/pan 명령만 생성한다. 실제 촬영에 쓰지 않을 수확 staging IK 때문에 정상 스캔이 차단되던 연결 오류를 제거했다. 실제 카메라 수확은 이후 전체 IK를 수행한다. 기존 비카메라 명령과 수확 phase/물성/1mm 기준은 유지한다.
- 카메라 경계에서 `camera_boundary_rail_state`가 현재 completed-cycle checkpoint를 검증하고 기존 `measured_resume_rail_origin`/`attach_resume_rail_hold`를 재사용한다. **기하 원점은 실제 측정 레일 위치, 제어는 native getter의 원래 목표값**이다. 20번에서 명목 목적지를 현재 위치로 쓰던101µm 오류를 수정했다. 같은0.1mm 수치 검사가 수정 뒤 실제1.05µm 이하로 통과한다. 원본 경로/오류와 실제 후속 후보의 다른 송이 과실 충돌을 구분해 보존한다.
- 환경 검사의 독립 jaw_L/R 측정 위치 및 고정 rail_anchor 정렬도 바로잡았다. 카메라별 현재 checkpoint/정확한 송이로60Hz 전체 환경 검사를 한다. complete PASS만 실행 가능하다. 첫 금지 충돌로 조기 거절한 suffix는 UNKNOWN이다. 22의2537표본 전체 경로 검사는 **사후 명목 검사**이며 당시 새 camera-boundary gate가 실행됐다고 소급하지 않는다.
- 현재 실제 완료된 집단 진입은 **22/25/27/29/28의5개 송이, 빨간 열매11개 native joint 분리, 미숙 분리0개**다. 각 송이 진입1회/닫기·열기2회를 검증했으며 익은 열매24개가 남아 송이 전체 완료가 아니다. 20은45.9667초 이동·스캔과8.5초 재스캔을 정상 완료했으나 수확 경로11개씩이 거절돼 수확 실적0이다. 이어30도 최신20 상태에서56.1초 실제 이동·스캔을 정상 완료했으나 기존 범위11후보는 IK/자가충돌로 거절됐다. 다음 실행의 부모는 이 최신30 normal checkpoint이며 이전 좋은 상태로 되돌리지 않는다.
- 30의 추가 오프라인 유한 검사에서는 카메라 yaw+60도 경로의 전체 자가충돌2537표본이 통과했지만 접근 중 그리퍼 오른쪽 덮개↔주줄기5 충돌이 남았다. 기존 native cooking 도구로 실제 덮개16조각을 모두 읽어 동일 경로를 재검사해도 막혔다. 별도0.01667초 idle 형상 캡처는 실제 연속 경로 상태나 수확 실적에 포함하지 않는다. Cartesian 두 연결도 준비 중 실제 리프트 받침대 간섭으로 거절됐다. 기준/형상을 바꾸지 않고 다른 높이의 송이 촬영을 준비한다.
- 지도 기반 수확 경로가 차단됐다는 이유로 촬영 자체를 막지 않도록 행 실행기에 명시적인 `--allow-camera-scan-only-proposals`를 추가했다. 원래 차단 상태/해시를 보존하고 새 장면 초기 검사·실제 스캔·카메라 수확 경로의 기존 전체 검사를 각각 요구한다. 23은 초기화 중 외부 송이↔주줄기 간섭으로2개 native joint가 조기 분리돼 **미사용 장면**으로 제외했다. 19는 먼저 C++ 초기화 exit139가 발생했고, 동일 모델의16→8스레드 대조는 초기화를 지나갔지만 최대10.27mm/약3984N 접촉과17개 조기 분리로 제외됐다. 어느 것도 수확이나 실제 이동으로 세지 않으며 마지막 유효 부모는30 normal56.1초다. 17은 주줄기↔외부 송이 source 검사는 맑았으나, 실제 첫 idle 스텝에서 낮은 비선택Truss00 Fruit10↔CoirSlab 9.4077mm 및 Fruit10/08 조기 분리로 미사용 제외됐다. 이 범위 누락을 반영해24의 전체10과실을1097개 구조물(coir/growbag/wire/gutter 포함), 자체/외부 과실·주줄기와 비교했다. 요청 범위의 source 겹침 증거0으로24의 기존 행 실행을 진행 중이며, 새 native 초기 유효성은 별도 요구한다.
- 연속 운전의 공통 제어 문제도 발견했다. 8개 정상 실행의 getter 목표값/실제q/명령을 대조하면 매 scan 종료에서 lift가 원래 drive 목표보다 약8.315mm 낮고, 다음 구간/수확 suffix가 그 측정값을 새 목표로 재사용한다. 따라서 22 시작0.177236m→30 종료0.027565m로 정지 높이가 누적 하강했다. 고의 리프트 이동이나 물리 관통이 아니라 반복된 setpoint 재기준화 문제이며, 원본은 보존했다. `--retain-lift-drive-reference` opt-in이 검증된 getter 목표−측정 위치 차이를 lift 명령에 한 번만 더하고, 별도 `desired_command`와 실제 `command`를 저장한다. 이 모드는 물성/강성/초기 상태를 바꾸지 않는다. 기존24 이동·스캔은 변경 전 코드이며, 새 실제 후속에서 보정·물리 결과를 검증 중이다.
- 24는 초기3초 native 검사와35.833초 실제 이동·스캔을 정상 완료했지만 수확 후보11개는7 IK 미발견/3 준비 중 양의 간격 부족/1 주줄기 woody stub 충돌로 막혔다. 최신 정상 부모는24 CP000(`45dc0dd6…`)이다. depth35→15mm의 단일 얕은 진입은 전체 self2537표본 PASS 뒤 덮개↔주줄기로 막혔다. 후속 native에서 L/part_209·R/part_331의 실제 볼록 조각32개를 모두 읽었다. 동일 원본/reader SHA에 결합된 이 형상으로 재검사해도 위쪽 집단의 덮개↔주줄기 충돌은 남는다. source envelope의 거짓 차단이라고 단정하지 않는다. 물리 충돌체/마진/1mm 기준은 변경하지 않았다.
- `continuous_camera_native_covers24`의 새 lift-reference 실행은 **소프트웨어 오류로1.45초/88실제 표본 뒤 중단**됐다. CPU `torch.as_tensor(q0)`와 NumPy 초기 q0가 메모리를 공유하여 보정값을 매번 누적했다. 세 생성 경로의 opt-in 버퍼에 `q0.copy()`를 적용했고 실제 CPU Torch 반복쓰기2회귀를 통과했다. 원본 부분 trace·오류·해시는 그대로 보존한다. 이 실패에는 완전한 native qvel checkpoint가 없으므로 후속을 그 마지막 상태에서 연속 재개했다고 주장하지 않는다. 재시도는 마지막 정상24 CP에서 분기하는 **별도 진단 시험**으로 표시하고 웹에도 분기를 공개한다. 아직 수정 후 native 완료 검증이나 여섯 번째 송이 실적은 없다.
- `--camera-ik-seed-indices`는 기존9개 IK seed 중 명시 목록만 같은 총12제안 예산 안에서 유한 순회한다. heading/seed별 거절 집합·fingerprint를 분리하며 기본 단일 seed 순서는 보존한다. 추가 대안들은 실패도 모두 보존했다. 1mm **계획 여유** counterfactual도 이후0.879mm로 막혔고, 해당 준비 끝점은 실제 자기 교차가 있어 운영2mm 계획 기준을 완화하지 않았다.
- 정상25→준비 중 옆24의 woody pruned stub가 턱을96→86.914mm로 밀어 진입 전에 정지한 이력도 보존한다. 잎으로 취급하지 않았다. `terminal25_recovery`가 그 마지막16.5초 상태의 실제 경로를 역행해11.5167초 정상 복귀했다. 이전 좋은 checkpoint로 되감지 않았다.
- 31/21의 미사용 장면 초기 실패는 source 과실↔와이어/다른 과실의 실제 초기 겹침과 대응한다. startup CLI 오류·센서 오류·IK/자가충돌/환경 충돌·불완전 검사를 구분하며, finite failure를 절대 수확 불가능으로 바꾸지 않는다. unused probe는 실제 수확/이동 이력의 상태로 채택하지 않는다.
- 웹 `http://lab.farmily.ai:8090/gripper-design/?tab=replay`의 오른쪽 PIP RGB/GT/Depth는 실제 촬영 timestamp를 사용하고 미래 프레임을 표시하지 않는다. 시험 묶음/구간/순차 재생/실패 로그 첨부를 지원한다. 전체 농장 model cache는 활성+현재 loading만 보존한다. **웹은 실제 저장 상태 재생이며 새 physics가 아니다.** 카메라2Hz와 실제 상태60Hz를 구분한다.
- 상태 인계는 실제 로봇·제어기·분리 과실을 이어받고, 지나간 부착 식물은 측정 변형 상태로 동결하는 기존 active-window 근사다. PhysX 인스턴스/접촉 캐시는 구간마다 재시작한다. 하나의 persistent solver 실행이나 전체 주변 식물 탄성화를 주장하지 않는다.
- 원자료 `/root/docker_share/mujoko_debugging_data/20261022_075300_camera_region_campaign`. `camera_campaign_report.py DATA_ROOT`로 전체 실제/실패 JSON·HTML을 갱신한다. `final_web_drafts`의 목표/영상/아카이브는 **실제6개 실적과 모든 실행 terminal**이 확인된 뒤 게시한다. 큰 자산/배열/영상은 NAS·웹 assets에 보존하며 Git 제외다.
- alias 수정 뒤 현재 관련 회귀385실행/383통과/Torch 없는 asset환경2skip(`post_alias_core_regressions.log`), 실제 CPU Torch에서는 해당2개 PASS(`real_torch_lift_reference_alias_regression.log`)다. 이전 다른 모듈 집합371실행/364통과/Isaac 전용7skip 기록도 보존한다. 반복 검사는 고유 테스트 수로 합산하지 않는다. PIP/선택/순차재생/분기표시 JavaScript38검사 PASS. 실제 브라우저 예비7항목도 PASS(오른쪽PIP·RGB/GT/Depth·실제22→28전환·한 모델 cache·오류0). 최종 기본 시험 묶음·순차 재생 검사는 실제6송이와 모든 physics 종료 뒤 수행한다.
- 현재25의 실제50.5sim초 루프는372.45wall초(0.136RTF)이며 planning124.04초+촬영183.07초가82.5%다. simulate/fetch54.18초만 따로 보면0.932RTF다. 이 수치는 전체 운영 실시간 달성이나 이전 Newton GPU 성능으로 해석하지 않는다.
- `training_eligible=false`, `harvest_success=null`; native 연결 분리·송이 전체 수확·포집·실물 성공을 구분한다. 이번 변경은 아직 커밋하지 않았다.

# Farmily Tomato 작업 인수인계

## 2026-10-22: 카메라에 보이는 모델 GT 마스크·인식/깊이 표시

- 사용자가 이제 모델의 알려진 객체 정보를 이용한 마스크 표시를 요청했다. 기존 RGB-D 전용 저장은 유지하고 `--capture-gt` / `./nvidia-sim/run_s300_harvest.sh rgbd-gt`를 추가했다. 분류 신경망 없이 Replicator visible instance ID를 실제 USD mesh/sourceName에 연결한다. 기존 원본 scene/물리/카메라/재질은 편집하지 않는다.
- `mounted_gt.py`: 전체 농장 instanced mesh와 활성 식물의 변형 skin을 함께 읽는다. `semantic.npy`(uint8 종류), `instance.npy`(uint32 안정 객체 ID), renderer 원본 ID/경로 대응, `objects.json`, `gt_catalog.json` 저장. 열매는 plant/truss/Fruit ID로 구별하고, 주줄기·Rachis·pedicel·peduncle·calyx·잎·거터·레일·구조물·로봇을 구분한다.
- RGB/Recognition/Depth Overlay3화면을 `analysis.png`/`analysis.mp4`/`rgbd/index.html`로 저장한다. 작은 제목·열매 corner box, 줄기 초록색, 깊이 고정0.1~2m 색상표시(원본0.1~5m)를 사용한다. 과실 색상은 USD authored RGB proxy이며 생물학적 숙도나 수확 가능성 판단이 아니다.
- 카메라에 실제 보이는 픽셀만 저장한다. 가려진 부분을 채우지 않으며 박스는 보이는 부분의 외곽이다. 3D 좌표는 visible depth 표면 중앙값이지 전체 열매 중심이 아니다. 수확 영역 선택/도달성/동작 순서는 아직 결정하지 않는다. `training_eligible=false`, `harvest_region_selected=false`.
- 회귀9개와 1m/2m 깊이·객체종류·완전가림·부분가림·화면밖 대조PASS. 원자료 `/root/docker_share/mujoko_debugging_data/20261022_072000_mounted_gt/index.html`. 최종 GT 승강23초47쌍도 기존 RGB-D 실행과 전체 상태·명령 정확일치,47프레임 전체 픽셀 ID/클래스/카메라시각 대조PASS. 후처리 벡터화는 동일 프레임 출력 정확일치,2.081→0.221초(물리 전체 가속 수치가 아님). 최종 native loop130.98초/0.176RTF이며 촬영·파일저장 평균1.35초/프레임이 남는다. 최종 자료는 `lift_gt_final/rgbd/`, 작은 검증은 `nvidia-sim/validation/s300_mounted_gt.json`.
- 현재 native Isaac 센서 시험이며 Newton GPU 실행기의 GT 연결과 카메라 기반 수확 알고리즘은 미검증이다. 기존 웹 조립/과거 재생 및 기존 수확 학습 데이터는 변경하지 않았다.

## 2026-10-22: RB5 하단 관절 세로 RGB-D 카메라

- 사용자 표시 위치인 `link1` 바깥쪽(+Y) 원형 커버에 원본 D435 외형을 세로로 부착했다. 베이스 yaw/리프트/기체 이동을 따르고 어깨 pitch에는 돌아가지 않는다. 실제 준비 자세에서 왼쪽 작물을 바라본다. 장착 브래킷·카메라는 visual-only이며 기존 물리 바디·조인트·충돌체2630개 속성/변환 동일; 카메라 하드웨어 무게/충돌 인증은 아니다.
- `DISPLAY=:0 ./nvidia-sim/run_s300_harvest.sh rgbd-test` → 로딩 후 START. 기존 native Isaac 승강 실행기에 촬영을 opt-in 연결했다. GUI의 `RGB-D view` 버튼으로 Robot/RGB-D 시점을 선택한다. `--headless`로 화면 없이도 RGB-D를 렌더/저장한다. `--physics-only`와 촬영을 함께 쓰면 거부한다. 새 기본 장면은 `20261022_065240_shoulder_rgbd/assembly_exposure`이며 기존 경로는 `S300_ISAAC_ASSEMBLY`로 지정 가능하다.
- `mounted_rgbd.py`: 기존 69.4° landscape RGB FOV 근사를 세로480×640으로 회전, 이상적인 aligned RGB/optical-Z depth(m,float32,무효NaN)를 저장한다. depth 범위0.1~5m, 별도 segmentation/mask 없음. RGB는 렌더러 ISO100→400(+2EV), depth는 동일이다. 프레임마다 K/실제 world_from_camera_optical/로봇q/physics step/timestamp를 저장한다. 실제 D435 stereo matching·noise·노출시간 모션블러·장치 보정은 미구현이다.
- PhysX Fabric이 움직이는 부모 밑 USD 카메라 좌표 갱신을 놓치는 문제를 검출했다(8.47mm). 실제 측정된 link1 자세×고정 장착변환으로 world 렌더 카메라를 동기화하며 GT 타깃을 향해 카메라를 임의 회전하지 않는다. Replicator `delta_time=0` 촬영 전후 physics step/time 동일을 강제한다. 카메라 좌표를 렌더러와 매 프레임 대조한다.
- 23초/120Hz 승강을 실제 Isaac6.1 PhysX CPU로 완료, 2Hz RGB-D47쌍(0~23초), 전체 native 상태/명령/timestamp가 카메라 없는 이전 시험과 정확 일치했다. 렌더 카메라 변환오차 최대4.44e−16, 유효 depth 비율 최소99.595%。1m/2m 기준판 깊이 및 RGB 방향/위치 검사PASS, 관련10개 회귀PASS.
- 최종 loop118.58초에는 최초 센서 캡처46.06초가 포함됐다. 이후46회 캡처 합11.42초; 최초 캡처를 뺀 루프 약0.317RTF. 이 native RGB-D 시험은 기존 Newton GPU 수확 속도 결과를 대체하지 않는다. Newton GPU 실행기 센서 연결/카메라 기반 수확 알고리즘/ROS 전송은 이번에 검증하지 않았다.
- 원자료·미완료 시험·영상: `/root/docker_share/mujoko_debugging_data/20261022_065240_shoulder_rgbd/index.html`. 최종 `lift_rgbd_final/rgbd/`, 작은 증거 `nvidia-sim/validation/s300_mounted_rgbd.json`. 기존 수확 데이터에 섞지 않았고 `training_eligible=false`다.

## 2026-10-22: 실제 리프트 수정 모델을 Isaac 실행기에 연결

- 브랜치 `codex/s300-isaac-real-lift-20261022`, 시작점 `74a9098915fd4d052716ce7e7ecb82a03bd87c89`. 웹과 같은 실제 받침판·뒤쪽 승강판·삼각 브래킷이 RB5·그리퍼와 함께 움직이고 기둥·기체는 고정된다. 새 `motion_assets.build()`에도 동일 수정기를 연결했다. 기존 수확 결과와 Newton GPU 입력은 덮어쓰지 않았다.
- `DISPLAY=:0 ./nvidia-sim/run_s300_harvest.sh lift-test`로 전체 농장 안에서 실제 Isaac 승강을 확인한다. 로딩 뒤 `START`를 눌러 시작하고 완료 후 창은 유지된다. 새 결과 폴더에 저장한다. `real-lift`는 수정 모델 보기, `lift-test --headless --physics-only`는 화면 없는 검사다. 환경변수 `S300_ISAAC_ASSEMBLY`/`S300_ISAAC_OUTPUT`으로 경로를 지정할 수 있다.
- 최종 사용 장면은 이전 native 수확 완료 이력이 있는 **전체 농장 외형 + 활성 식물1개**의 수정본 `20261022_062713_isaac_real_lift/assembly_verified_scene`이다. 처음 사용한 탄성 식물3개 장면은 Plant29 줄기↔Plant30 송이의 초기 겹침1.383mm로 중단했고, 그 실패도 별도 보존했다. 임계값을 늘리거나 동일 작업량인 것처럼 비교하지 않았다.
- Isaac Sim6.1 / PhysX CPU TGS /120Hz/16threads에서 상승→유지→하강→복귀23초를 headless·GUI 각각 완료했다. 두 실행의 전체 저장 timestamp/pose/q/명령과 분리 기록이 정확히 일치한다. 감시 대상 관통0, 새 분리0, 사전·실제 팔 자기충돌 검사 통과. 실제 승강량 약41.5~691.5mm, 기존 유한 구동기의 정지 높이오차 약8.5mm는 남는다. 기구 검증이며 수확 성공 시험은 아니다.
- 이번 native 속도는 headless0.430RTF, GUI0.149RTF다. 이전 Newton GPU 속도 개선 결과를 대체하지 않으며 **수정 모델의 Newton GPU 수확 입력 재생성/경로 재검증은 미완료**다. 기둥 고정/받침대 연동을 검증했지만 나머지 리프트 CAD는 이전처럼 visual-only이고 실물 스트로크·질량 인증도 아니다.
- native materialized USD 선택, 원본 SHA가 같은 아카이브 자산 경로, 이웃 관절 armature 판독을 수정했다. 실제 USD 값을 읽을 뿐 물성은 바꾸지 않는다. 관련24개 회귀 통과. 작은 결과는 `nvidia-sim/validation/s300_isaac_real_lift.json`, 전체 로그·상태·영상은 `/root/docker_share/mujoko_debugging_data/20261022_062713_isaac_real_lift/`에 있다.

## 2026-10-21: 농장·수확 동작 웹 재생 탭

- 브랜치 `codex/s300-farm-replay-web-20261021`. 기존 미커밋 리프트/그리퍼 작업을 보존했다. 웹 세 번째 탭 `http://lab.farmily.ai:8090/gripper-design/?tab=replay`에서 실제 저장된 Isaac 실행을 선택·재생한다. 시간 탐색, 일시정지, 0.25~2배속, 자유 시점·로봇/송이/농장/탑뷰 제공.
- `gripper-design/harvest-replay/export_replay.py`가 기존 USD 메시·재질 판독기, `MeasuredPoseDisplay`, `transmission_transforms`와 같은 스킨 공식을 재사용한다. 농장31,538개 가시 메시 인스턴스/53,997,552삼각형(복제 포함)을 유지하고 반복 형상은 GPU instancing, 가지는 glTF skin으로 표시한다. 웹에서는 새 physics를 실행하지 않는다.
- 등록 예제는 **Isaac Sim 6.1 / PhysX CPU TGS 60Hz**의 실제 완료 기록2개: `isaac_truss_57s`(3,421프레임×277바디/57초/분리5개), `isaac_move_83s`(4,969×136/82.8초/레일 약1.6424m 이동/분리4개). 원본은 로컬 `/root/docker_share/mujoko_debugging_data_bak`에 있으며 NAS의 같은 이름 백업과 다르다.
- 모든 timestamp·pose·분리 이벤트와 실제 runtime composed USD SHA를 독립 비교했다. 추가 pose 정밀도 손실0, 스킨 계산 오차 최대2.86e−7m. 변조 음성 대조5개 검출. 코드/작은 증거는 `harvest-replay/checks/`, 대형 웹 번들은 `harvest-replay/assets/`이며 Git 제외이므로 머신 이동 때 별도 보존한다.
- 두 기록은 최근 실제 리프트 수정 이전 모델이다. 기록 당시 형상을 재생하며 새 장착 형상으로 바꿔 과거 결과를 보여주지 않는다. 최신 로봇 조립 탭은 수정된 실제 리프트를 계속 사용한다.
- 새 기록 추가는 `./nvidia-sim/.asset-venv/bin/python gripper-design/harvest-replay/register_run.py ASSEMBLY TRIAL --id NEW_ID --label "이름"`. 원본 결과는 덮어쓰지 않는다. 형식·재현은 `harvest-replay/README.txt`.

## 2026-10-21: 실제 리프트 받침대에 RB5 연결

- 브랜치 `codex/s300-real-lift-20261021`. 기존 미커밋 그리퍼 웹 작업을 보존했다. 새 웹 조립 탭: `http://lab.farmily.ai:8090/gripper-design/?tab=robot`.
- 가상 `.30×.95m` 상판만 승강하던 오류를 수정했다. 원본 assy_lift의 받침판·뒤쪽 승강판·삼각 브래킷을 기존 `farmily_lift_height_joint`의 움직이는 바디에 연결했다. 기둥·모터·외측 바닥 지지대는 고정. 원본 리프트 삼각형120,881개 보존, 가상 판 제거, RB5 장착높이−2.931mm 보정.
- `nvidia-sim/harvest_s300/real_lift_mount.py`는 새 assembly를 만들며 원본을 덮어쓰지 않는다. 기존 관절·drive·질량 추정 유지. 가상 상판 collider는 실제 받침판 hull로 교체; 나머지 리프트 CAD는 이전처럼 visual-only이며 전체 기구 충돌 인증이 아니다.
- 결과 `/root/docker_share/mujoko_debugging_data/20261021_230320_real_lift_mount/assembly/composed.usdc`. 농장 배경을 포함한 새 조립이다. **기존 저장된 수확 MJB/trace/간단 실행기는 원본 상태로 보존**했다. 새 장착 형상으로 수확을 하려면 새 입력과 경로 검증이 필요하다.
- 로봇 단독 승강23초를 native MuJoCo CPU와 Newton/MuJoCo Warp GPU에서 각각 완료, 관통0. 기존 finite PD의 정지 높이오차 약8.51mm는 남는다. CAD·장착·FK 회귀6개 통과. 전체 농장 수확 재시험 및 실물 질량/강도/스트로크 인증은 미완료.
- 웹에서 `리프트 상승량`과 `승강 보기`로 실제 받침대·RB5·그리퍼가 함께 움직이는 것을 볼 수 있다. 웹 조작은 기구 미리보기이며 라이브 물리가 아니다. CPU/GPU 결과와 브라우저 검증은 `gripper-design/robot-assembly/checks/real_lift_validation.json`, 재현 방법은 `gripper-design/README.txt`.

## 2026-10-18: 한쪽 베드 Newton GPU 속도 개선 및 추가 대안 검증

- 현재 작업 브랜치 `codex/s300-newton-active-window-20261018`. 기존 선택기·계획기·GPU 실행기를 재사용해 Plant22→27→29 실제 연속 이동/수확3구간을 완료했다. 반대편Plant41 제외. 새 간단 실행: `./nvidia-sim/run_s300_newton.sh row-realtime` (화면 없는 새 캠페인).
- 현재 타깃 식물만 탄성 계산하고 다른 식물은 실제 마지막 자세의 **고정 충돌체로 유지**, 이미 분리된 과실은 계속 free body다. 주변6그루1150충돌체·전체 농장 외형 유지. 자유도133→169→199. 세 식물 동시 탄성379DOF와 동일 물리 모델의 솔버 가속이라고 해석하지 않는다.
- 물리191.8169초를79.4614초에 실행: **전체2.414RTF**, 벨트구간1.19~1.31RTF(이전0.50~0.58). 가장 느린마지막 유지0.814~0.859RTF는 미해결이다. 최종one-command 재실행도 전상태exact·2.411RTF·17개 분리다. setup합395.44→165.20초였지만 계획/모델축소까지 전체3구간584.90초(약9분45초)+최초검증 대기여서 운영전체 실시간은 미달이다. 후속 동일 준비 A/B55.70→49.40초는 별도 단회 결과다.
- 연결 해제17개(색상상 익음16+미숙1), 남음13개. 이전379DOF는18개였으며 P27Fruit02가 추가로 남았다. 전체수확/포집/실물성공 아님. 두 경계 actual clock과상태·분리이력 연속성 독립검증PASS. `training_eligible=false`, `harvest_success=null`.
- GUI 표시 약6.76→14.83FPS. 첫Plant22 전체8,221기록의 새 headless/GUI 적분이 byte exact다. GUI새 측정 RTF1.897/headless2.313;3정차점전체GUI·cold-start동등성은 미검증이다.
- 개별 Hz/solver/cache/CCD/형상저장 대안을 비교했다.220/60Hz 유지. 마지막hold만160/180Hz로 낮추는 대안도 다른 hardware↔가지1mm 초과로 중단해 미채택. 기본1mm/기존 정확belt↔목표fruit·pedicel의 지정5phase만2mm(닫힌유지belt_finish포함),물성·분리조건 유지.
- 최종 중복제거 회귀395PASS/25SKIP,별도표시15PASS. 큰 자산/배열/영상은NAS,작은검증·코드·결과보고서는Git. 사용자gripper-design 별도변경은보존/커밋제외.
- [상세 결과·실행·영상 목록](nvidia-sim/S300_NEWTON_ACTIVE_WINDOW_SPEED_KO.md), [검증 요약](nvidia-sim/validation/s300_newton_active_window_speed.json).
- 결과: `/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/index.html`. 모든 후보의 완료/중단·최초접촉·실제 qpos영상 포함. 다음 큰 병목은 구간별 준비/재로딩과 마지막접촉단계이며 GPU병렬 확대는이번에새로검증하지않았다.

## 2026-10-18: 한쪽 베드3곳 연속 이동·수확 완주, native 충돌체 수정

- 같은 거터 X=-0.775m의 **Plant22/Truss02 → Plant27/Truss00 → Plant29/Truss00** 세 방문을 모두 완료했다. 반대 베드 Plant41 제외. 실제 레일 정차1.580→2.653→3.222m와 리프트 이동 뒤, 각 방문에서 두 번 닫기·열기 및 후퇴를 수행했다.
- `native_piece_route/sequence_native`: 각6개씩 **총18개 연결 해제=색상상 익음17+미숙1(Plant22 Fruit09), 미방문 조기 분리0**. 각 송이4개씩 남았다(익음4+미숙8). 송이 전체 완료·선택적 수확·포집·실물 성공은 아니다.
- 공통 모델은3개 탄성 식물379DOF/과실30개/2244geom, 주변 고정6그루1150collider와 전체 농장 배경이다. 기존 단일133DOF와 작업량이 다르다. `run_newton_row_harvest.py`가 기존 선택기·IK·GPU 실행기를 연결하며, 두 인계 경계 qpos/qvel/eq/poses/time 차이0이다. solver instance/접촉 캐시는 구간마다 새로 생성하므로 한 번 로딩하는 persistent solver는 아니다.
- 최초 GPU tolerance1e-6의 미방문 과실 힘 스파이크는 force 읽기 오류가 아니라 수렴 문제였다. 명시 `--solver-tolerance 1e-8`을 업로드 clamp 후 실제 GPU에 반영하는 opt-in을 적용했다. **기존 기본 호출은 보존**한다.
- 이전 `sequence_precise`는17개 분리 후 Plant29의 Tool_L `L/part_161`↔남은 꼭지1.071557mm에서 중단했고, depth35→25mm 대조도 같은 쌍으로 막혔다. 원본 빈 공간을 메우던 단일 hull만 원본 PhysX 기본 native16조각으로 수정하고 새 MJB/정상 초기 상태에서 전부 재실행했다. 비선택 형상·물성·명령·초기 상태 등71,813개 불변성 검사PASS. 이전 결과·대조는 보존하고 합산하지 않는다.
- native 근사에도 원본에 없는0.209~0.279mm 거짓 overlap과 표본 최대0.635mm 덮임 부족이 남았다. strict 음성 기하 대조FAIL과 기존1mm 감시 통과를 구분한다. 원본 CAD 전체 coverage 인증은 아니다. 감시 기준·물성·분리 한계는 올리지 않았다.
- 전체191.816895sim초/173.742993loop초=평균1.104RTF, setup155.801618초 별도다. 최저 접촉 단계0.269RTF이므로 **접촉 실시간은 미해결**이다. 기타 쌍1mm/정확한 현재 목표 belt 쌍2mm 범위를 유지했다. `--released-gutter-diagnostic`는 명시 opt-in이며 최종 세 방문에서 해당 접촉0건이었다. 주변 식물의 실제 pair 접촉0을 뜻하지 않는다.
- [상세·실행 명령](nvidia-sim/S300_NEWTON_SAME_BED_CONTINUOUS_KO.md). 준비 입력은 `/root/docker_share/mujoko_debugging_data/20261018_010218_newton_same_bed_continuous/native_piece_route`, 결과 루트 `index.html`·`native_video_list.json`·`all_videos.json`에 실제 영상 목록을 보존한다. 고유 회귀78개PASS. 새 GUI/headless 전체 비교·FCL 환경 사전 인증·runtime phase guard는 미완료. `training_eligible=false`, `harvest_success=null`. 이 변경은 위 active-window 속도 개선 작업과 함께 보존한다.

## 2026-10-18: 주변 고정 충돌을 포함한 두 위치의 GPU 수확 시험

- 기존 Plant29/Truss00와 다른 Plant41/Truss00의 명령을 재사용해 새 physics9회 실행, 모두 완료. 각각 연결 해제6개/4개이며 독립 reset 시험이다. 연속 이동 수확이나 전체 송이 수확 성공으로 합산하지 않는다.
- Plant41의 주변0/2/4그루 조건을 각2회 비교: 평균2.802/2.765/2.752RTF, 물리시간19.036/19.292/19.386초(53.343sim초). 주변4그루 시간 증가1.84%, 최저단계0.970RTF. 초기화·저장·렌더 별도.
- Plant29 주변09/28, Plant41 주변64/63 및40/39. 기존 변환 capsule/convex를 고정해서286/572개 추가, 잎 충돌 제외. 실제 주변 접촉0건, Plant41 모든 A/B·반복·접촉계측의 전체 저장 상태 정확 일치. 강하게 밀리는 주변 탄성/전 농장 충돌 검증은 아니다.
- 기존 기본 충돌 범위는 자동 변경하지 않았다. 실제 입력 메타데이터를 읽어 영상의 잘못된 ‘주변 충돌 OFF’ 자막을 바로잡고 새 mesh도 입력 hash에 포함했다. 관련19개 테스트PASS.
- [상세 결과·GUI 명령·영상](nvidia-sim/S300_NEWTON_NEIGHBOR_COLLISION_CHECK_KO.md#2026-10-18-후속-두-위치와-주변4그루), 원자료 `/root/docker_share/mujoko_debugging_data/20261018_002607_newton_neighbor_multitarget/index.html`.
- `training_eligible=false`, `harvest_success=null`. 사용자 gripper 변경 및 기존 결과 보존. 이번 후속 변경은 아직 미커밋이다.

## 2026-10-17: 주변 식물 충돌 이력 확인 및 고정1·2그루 비용 비교

- 이전 기본형은 주변 고정 충돌 포함이었고,10/11 사용자 선택으로 D_target_only를 사용했다. Newton도 이를 이어받았다. 전체 배경 표시는 전체 식물 충돌 계산을 뜻하지 않는다.
- 현재133DOF/220·60Hz/동일 명령에서 주변0/1/2그루 고정 충돌을 각2회 새 실행: 평균3.873/3.812/3.789RTF, 시간 증가1.59/2.22%. 가장 느린 단계도1.113RTF 이상이다.
- 기존 변환 Plant09/28의 capsule+convex143/286개 추가. 전체21,187개 원본 triangle collider 복원이 아니며, 주변 탄성도 추가하지 않았다. 기존 기본값은 유지했다.
- 모두9,127step 완주·연결 해제6개, 저장 상태 정확 일치. 실제 추가 주변 접촉0건이어서 강하게 밀 때의 비용은 미측정. 관련 테스트3개 통과.
- [기준 이력·결과·한계](nvidia-sim/S300_NEWTON_NEIGHBOR_COLLISION_CHECK_KO.md). 결과 `/root/docker_share/mujoko_debugging_data/20261017_234049_newton_neighbor_collision_check`.

## 2026-10-17: Newton GUI 로딩 후 START, 1배속 실행

- `./nvidia-sim/run_s300_newton.sh gpu-realtime --gui`는 초기 장면 렌더 완료 후 **Harvest control → START (1x)**를 누를 때까지 물리0초에서 대기한다. 완료 후에도 창을 유지한다.
- headless 처리량 측정은 기존 그대로. GUI 자동 시작/최대 처리량 측정에는 `--no-interactive-viewer`를 추가한다. 의도적인1배속 대기 시간을 계산 성능 저하로 해석하지 말 것.
- 실제 버튼 클릭·전체9,127step/82.807초 완주·기존 headless 전체 상태 정확 일치 확인. 관련 테스트20개 통과. 물리/접촉/분리 조건은 유지했다.
- [명령·검증 기록](nvidia-sim/S300_NEWTON_INTERACTIVE_VIEWER_KO.md), 결과 `/root/docker_share/mujoko_debugging_data/20261017_230835_newton_interactive_validation`.

## 2026-10-17: Newton GPU 개별11대안·조합, 접촉 실시간과 GUI 일치 확보

- 브랜치 `codex/s300-newton-independent-tests-20261017`, 출발 `4112bda`.
  11개 대안을 개별 시험하고 제한된 조합을 실제 서버에서 실행했다. 기존 사용자 gripper 변경·대형 자산·학습 데이터는 보존했다.
- 새 opt-in 명령: `./nvidia-sim/run_s300_newton.sh gpu-realtime --gui`.
  실제 Newton→MuJoCo Warp CUDA 물리 + 별도 Isaac 전체 농장 표시다. 기존 `gpu`/`gpu-fast` 기본값은 유지했다.
- 권장 조건은 **133DOF+보수적fruit proxy,접촉220/이동60Hz,정확한벨트↔열매/꼭지만2mm,나머지1mm**다.
  기존 분리 열매의 바닥·기체·자기 송이 접촉 예외는 유지하므로 그 예외까지1mm였다는 뜻은 아니다.
  물리 loop 전체 약3.86–3.87RTF/최저phase1.13–1.14, GUI 전체3.37–3.38/최저0.98.
  190DOF/200Hz 대조도 완주하며 headless최저1.01,GUI최저0.89다.
  두 모델 및 새 명령의 새 headless/GUI 전체 상태·접촉·힘·분리 시계열 정확 일치. 저장 영상 재생만으로 비교한 것이 아니다.
- 133DOF는 기존 내부55pair가 추가로 억제되는 표현 변경이고 proxy외향상한0.1mm다. 원본385DOF 그대로의 가속이라고 주장하지 않는다.
  2mm는 닫기/닫힌압력유지/열기stroke의 벨트 눌림 근사 판정이며 실제 탄성 벨트를 계산한 것이 아니다. 재질 수치는 유지하나 Hz에 따라 유효 접촉행 계수는 달라진다.
- 동일133DOF 입력의 CPU16thread 전체5.55RTF로 **단일 환경 CPU가 더 빠르다**. GPU 우위로 포장하지 않았다.
  권장 명령 cold 준비/loop약8.88/21.40초, GUI약28.92/24.48초.3.9RTF를 로딩·저장까지 포함한 운영 처리량이라고 해석하지 않는다.
- GPU 배치는 canonical 단일 환경과 다른 native 수치 경로이며, 별도 N1 기준으로 모든 환경의 전체 기록을 비교한다.
  N1/2/4/8/16/32의 합산RTF1.91/3.41/5.70/8.46/11.21/13.13, N32는16,575MiB다. 모든 world가 기준N1과 정확히 일치했다.
  N40/128은4096 capacity 초기화 제한이며, N1/96을 먼저 대조한 N40/96은13.73합산RTF/20,447MiB로 완주했다.
  단96은 broadphase 여유0이어서 고정 입력의 용량 탐색이다. 새 장면 파일럿은32/128부터 시작하고 기본128은 유지한다.
  N32 환경당 전체0.410/마지막벨트0.081RTF이므로 합산처리량을 각 환경의 실시간이라고 해석하지 않는다.
  [배치 보고서](nvidia-sim/harvest_s300/newton_gpu_batch_independent_results_ko.md)에서 환경당/합산 처리량·메모리·용량 제한을 확인한다.
- 5–6개 연결 해제는 송이10개 전체 수확 성공이 아니다. 분리 열매의 기존 바닥/기체/자기 송이 접촉 예외를 유지해 바닥13.9/20.2mm 겹침이 남는다.
  정확한 낙하·수거 검증이나 실물 물성 보정은 미완료. `training_eligible=false`, `harvest_success=null`. 새 RL/수확 알고리즘은 추가하지 않았다.
- [종합 결과·명령·전체 영상 목록](nvidia-sim/S300_NEWTON_INDEPENDENT_RESULTS_KO.md),
  [개별11항목 표](nvidia-sim/S300_NEWTON_INDEPENDENT_TEST_MATRIX_KO.md),
  [단일 검증 JSON](nvidia-sim/validation/s300_newton_canonical_realtime.json).
  원자료/대시보드 `/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/index.html`.

## 2026-10-17: Newton GPU 솔버·식물 표현 비교

- 브랜치 `codex/s300-newton-solver-compare-20261017`, 시작 `7930cb0`, 구현 `507653e`.
  원본 물리/제어/자산과 미커밋 gripper 작업을 보존하고, 공식 Isaac importer·XPBD·VBD·양방향 ADMM·식물 좌표/충돌 표현을 실제 시험했다.
- 원본 GPU 전체1.556배속 → 133DOF+보수적0.1mm 열매 proxy 약2.50배속. 접촉400Hz/이동60Hz·활성1mm·3N/.008Nm 분리 조건 유지.
  벨트 마지막 단계는 약0.62배속으로 **접촉 실시간 목표 미달성**. 6개 연결 해제이며 전체 송이 수확 성공이 아니다.
- 변경 모델은 질량/관성/외부 collider를 보존하지만 독립 잎/막대 굽힘 자유도를 줄이고 새 강체 내부의 기존 충돌 가능55pair가 억제된다.
  동일 모델의 속도 향상이라고 해석하지 말 것. 정적 compliance와 동일 CPU의 밀기/복원/회전 반응을 별도 비교했다. 실물 보정은 미완료.
- `./nvidia-sim/run_s300_newton.sh gpu-fast --gui`는 opt-in 변경 모델. 기존 `gpu`는 원본 유지.
  133DOF 원형 후보의 실제 카메라 이동 GUI/headless 전체 상태·접촉·힘·분리 정확 일치. 최종 proxy 반복/GUI 증거는 보고서/검증 JSON 참조.
- 같은133DOF 원형의 실제GPU1/2/4/8환경 합산처리량1.229/2.187/3.629/5.356. 모든 환경의 전체 시간열·힘·분리 이벤트가 단일 환경과 정확 일치.
  이는 서로 다른 장면 시험이나 메모리 최대 N 검증이 아니며 환경당 속도는 N 증가 시 감소한다.
- [솔버·표현 결과 보고서](nvidia-sim/S300_NEWTON_SOLVER_COMPARISON_KO.md),
  [작은 검증 요약](nvidia-sim/validation/s300_newton_solver_comparison.json).
  원자료/실제 상태 영상 `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/index.html`.
  `training_eligible=false`, `harvest_success=null`. RL/새 수확 알고리즘/대규모 수집은 하지 않았다.

## 2026-10-17: Newton GPU 전체 평균 실시간 초과, 접촉 구간은 미달

- 브랜치 `codex/s300-newton-gpu-realtime-20261016`, 출발 `1868610`.
  기존 동일 입력/명령·400/60Hz·활성1mm·분리 규칙·전체 Isaac 농장 외형을 유지했다.
- CUDA graph 나머지 스텝 포함, sparse 합산 색인, 불필요한 결정성 처리 제거,
  제약 생성 후 행/포인터 정렬 적용. 실제 GPU 전체 실행 **83.983→53.052초(1.561×)**,
  카메라 이동 GUI **95.780→61.267초(1.351×)**. 각6개 분리·활성 최대.794207mm.
- 전체13,806step의 상태/힘/접촉/분리 기록은 이전 GPU와 정확히 같고 새 GUI 쌍도 정확 일치.
  그러나 belt_finish는 headless.382×/GUI.328×로 **접촉 실시간 목표 미달**, 실제N환경 수확 미실행.
  대표 접촉 스텝 GPU solver4.921ms로400Hz 예산2.5ms 초과. 평균 속도로 목표 완료를 주장하지 않는다.
- `./nvidia-sim/run_s300_newton.sh gpu --gui`에 검증한 설정 적용.
  CPU 비교 모드는 그대로 유지. 단일 환경만 검증, `training_eligible=false`, `harvest_success=null`.
- 결과 [S300_NEWTON_GPU_REALTIME_KO.md](nvidia-sim/S300_NEWTON_GPU_REALTIME_KO.md),
  검증 [s300_newton_gpu_realtime.json](nvidia-sim/validation/s300_newton_gpu_realtime.json).
  원자료/영상 `/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/`.
  폴더는 작업 시작일10/16 표기이며 최종 실행/검증은10/17 KST에 완료했다.

## 2026-10-16: 실제 Newton GPU 실행·전체 GUI 일치 확보, 접촉 속도 미달성

- 브랜치 `codex/s300-newton-gpu-20261016`, 출발 `f1941f5`. 기존 CPU 결과와 사용자 미커밋
  gripper-design 자산은 보존했다. 이번 물리는 **Newton → MuJoCo Warp CUDA**이며,
  별도 Isaac 프로세스가 전체 농장과 실제 측정 상태를 표시한다. CPU fallback은 없다.
- 같은 132바디/385DOF/활성901geom/10열매 입력과 명령, 접촉400Hz/이동60Hz,
  활성 관통1mm·3N/.008Nm 분리·분리 열매 접촉 예외·타깃 송이/자기 주줄기 충돌 정책 유지.
  CUDA graph, cuSOLVER32 factor, 고정 순서 tree/contact/force reduction, 동등한 Hessian 행렬곱을 적용했다.
- 최종 headless/실제 카메라 이동 GUI를 각각 새 초기화해 전체13,806step+초기 상태 비교:
  q/v·body pose·명령·접촉·힘·분리 이벤트 **bit-exact**. 각6열매 분리, 활성 최대.794207mm.
  저장 영상 재생을 GUI/headless 검증으로 대신하지 않았다. `harvest_success=null`, `training_eligible=false`.
- 속도: 82.793sim초를 headless83.983초(**.986×**), GUI95.780초(**.864×**).
  headless 닫기.333×/재닫기.420×/벨트 끝.244×로 **실시간 목표 미달성**.
  단일 환경 속도 조건 미충족이므로 실제 N환경 수확 확장은 하지 않았다. N4 행렬 대조 시험은 수확 성능이 아니다.
- 모델 한 번 로딩5.259초, 반복 로딩0초. 두 번째 reset.978초, 첫 reset.010초.
  초기 setup과 physics loop는 분리했지만 JIT만의 독립 시간은 미측정이다. 미기록 물리 warmup은0step.
- 실행: `./nvidia-sim/run_s300_newton.sh gpu --gui` (화면 없이 실행은 `--gui` 생략).
  한 프로세스 반복/GUI 대조: `gpu --repeats 2 --gui-repeat-index 1 --camera-test-motion`.
  CPU 비교 모드 `run`은 보존한다. 기존 CPU 결과를 GPU 목표 달성으로 사용하지 않는다.
- 보고서 [S300_NEWTON_GPU_RESULTS_KO.md](nvidia-sim/S300_NEWTON_GPU_RESULTS_KO.md),
  작은 검증 자료 [s300_newton_gpu_recovery.json](nvidia-sim/validation/s300_newton_gpu_recovery.json).
  원자료·로그·실제 상태 영상: `/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_recovery/`.
  최종 대표 `repro_fixed_reductions_session400`, 이전 재현성 통과 `repro_dense_hessian_session400`.
- Float32 합산 순서를 바꾸면 유효한 실행끼리도5/6개 분리 차이가 있다. CPU와 GPU trajectory 동일성이나
  PhysX와의 동일성을 주장하지 않는다. pinned backend/GPU의 GUI 대조 결과다. 전체10열매·포집 성공 미검증.

## 2026-10-16: S300 Newton 단일 실행·GUI 일치·CPU 병렬 검증

- 브랜치 `codex/s300-newton-20261016`. Isaac 원본 `ca3a716`과 기존 결과 보존,
  기존 MuJoCo portable 자산·sample_plan·제어·분리·영상 경로 재사용.
- 별도 Newton1.7.0 /MuJoCo·MJWarp3.15.0 /Warp1.17.0. **권장 CPU는 Newton framework +
  native MuJoCo CPU**이며 GPU 가속 결과로 설명하지 않는다. 실제 Newton GPU도 시험했지만
  재검증 단일 실행이 활성 겹침 기준 초과로 중단되어 N8/32는 미평가다.
- 기준 CPU400Hz 접촉/free60Hz 이동: 82.795sim /66.733loop초=1.241×,
  실제 Isaac GUI 표시68.037초=1.217×. 13,806스텝+초기 상태 전체가 명령·q/v·pose·
  equality·접촉·분리 이벤트까지 정확히 같다. GUI는 별도 표시 전용 프로세스다.
- 한 번 진입/두 차례 개폐에서6열매 분리, 활성 최대 겹침0.681mm. **전체10열매 수확 아님**.
  전체 평균은 실시간 이상이나, 접촉 구간은 약0.30~0.65×로 아직 느리다.
- CPU1/8/24/48 독립 환경 총81회 완주·동일 저장 상태. 합산1.249/9.673/26.272/35.907
  sim초/실제초. 24환경 환경당1.095×,48환경.748×. 최대RSS75.71GiB.
  `cpu_scaling/evidence_identity.json`에서 상태NPZ·매step physics 지표NPZ의 SHA256이 각각1종임을 확인.
  같은 장면/명령 복제 처리량이며 다양한 장면·정책 검증이 아니다.
- 결과가 정확히 같은 선택 최적화: native-observables1.306×,
  비충돌 시각geom3,819개만 물리에서 분리1.336×. 활성901개와 질량·관절·물성 유지,
  정적12,370항목·전체 실행 상태/접촉 동등성 확인. 최적화 GUI도1.304×·전체13,806step
  정확 일치라 현재 기본은 `inputs_render_separated` +native-observables. 원본 입력도 보존한다.
- 120/240/300Hz·CG는1mm 초과 중단, dense는5개 분리로 동작 달라 기본 채택 안 함.
  분리3N/.008Nm·활성1mm·전체 농장 외형·타깃 송이/자기 주줄기만 충돌 정책 유지.
- 실행 `./nvidia-sim/run_s300_newton.sh run --gui` (headless는 `--gui` 생략).
  매번 새 출력, source/cache 덮어쓰기 없음. 자세한 명령 [S300_NEWTON.md](nvidia-sim/S300_NEWTON.md),
  결과 [S300_NEWTON_RESULTS_KO.md](nvidia-sim/S300_NEWTON_RESULTS_KO.md).
- 원자료 `/root/docker_share/mujoko_debugging_data/20261016_s300_newton/`.
  실제 상태1배속 영상2개는 `cpu400_render_separated_gui/newton_actual_entry.mp4`,
  `newton_actual_truss_top.mp4`. 후자는 재생 화면만 배경/잎133경로를 숨긴 접촉 탑뷰다.
  `training_eligible=false`, `harvest_success=null`. 전체행 수확·포집·RGB-D 판단·GPU 병렬은 미완료.

## 2026-10-12: S300 전체 농장·연속 수확 MuJoCo 3.15 연결 완료

- `codex/s300-mujoco-belt-smoke-20261012`, Isaac 보존 `ca3a716`, 이전 소규모 시험 `51e22eb` 후속.
- 전체 농장 원본 외형 + S300/리프트/RB5/동적 턱/안쪽 벨트, 241바디/739DOF. pose 강제 이동 없음.
- 레일 총 약1.714m 이동, 두 송이의 선정 집단03/04/05 모두 포함해 **빨간색 판정 대상9개 분리**.
  원본 policy의07/08도 빨간색 대상. 진입 전96mm 개방, 수평 진입, 닫기/열기2회, 후퇴·다음 위치 완료.
  잔여 대상1/4개 중 한 번에 잡을 집단 최대1/2개라 최소집단3개 미달로 남김.
- 최종 **300Hz 접촉/60Hz 이동/worker0**: 136.137sim/174.589loop초=**.780×**.
  활동 겹침.983mm, 1mm 중단0. 낙하 후 최대15.37mm는 별도 진단이며 전체 물리 정확도 통과 아님.
- 전체 반복/worker4 전 구간/4 접촉 구간의 저장 배열 완전 일치. worker4는 전체 실행에서 더 느려 기본0 유지.
- 타깃 송이·자기 주줄기만 식물 충돌, 잎 무시, 나머지 식물 외형 유지. 구조물은 거터18개+바닥.
  opt-in 낙하 과실↔거터 진단 예외, 바닥 top-plane 사용. 부착 과실/로봇 거터 기준 유지.
- 접촉 margin 이식 오류와 마지막 열기 60Hz 누락, 검은 영상 camera/light 갱신, 캐시 원본 보호 수정.
- 명령: `./mujoco-benchmark/run_s300_mujoco.sh replay`(실제 기록1× GUI), `test`(새 물리), `prepare`(새 변환).
- 결과 `/root/docker_share/mujoko_debugging_data/20261012_s300_mujoco_full/index.html`, 대표 `campaign_final`.
- 자세한 이식/속도/영상/검사/한계: [S300_MUJOCO_MIGRATION_KO.md](mujoco-benchmark/S300_MUJOCO_MIGRATION_KO.md).
- RGB-D 기반 선택·전체 행로·GPU 병렬·수거·실물 손상 기준은 미완료. `harvest_success=null`, `training_eligible=false`.

## 2026-10-12: MuJoCo 3.15 단일 송이 시험

- Isaac 기존 작업 `ca3a716` 보존 후 `codex/s300-mujoco-belt-smoke-20261012`에서 시험.
- 최신 안정판 3.15.0은 별도 `mujoco-benchmark/.mujoco315-venv`; 기존 환경 보존.
- 주줄기+1송이+기록된 그리퍼 pose, 로봇 팔/농장 제외. 120Hz 36.8초를 약24.5초(1.50배) 처리.
- 안쪽 벨트 03/04/05/07 분리, belt-off 02/05 분리. 120Hz 두 반복 상태 완전 일치.
- 60Hz는 약3배지만 1mm 기준 초과·9개 분리로 채택 불가. **전체 이식/검증된 수확 성공 아님**.
- prescribed 턱이 접촉력에 양보하지 않아 분리 후 최대315N/바닥30.4mm 겹침. 동적 턱 및 정확한 충돌 분해가 다음 단계.
- 보고서: [S300_MUJOCO_SMOKE_KO.md](mujoco-benchmark/S300_MUJOCO_SMOKE_KO.md).
- 데이터/영상: `/root/docker_share/mujoko_debugging_data/20261012_mujoco_belt_smoke/index.html`.
- 이전 Isaac GUI 갑작스러운 동시 파단 원인은 미확정 상태 유지.

## 2026-10-12 MuJoCo 소량 비교 전 Isaac 상태 보존

사용자 요청으로 아래 Isaac 연속 수확/식물 충돌 범위/얕은 진입 작업을 먼저 커밋하고,
새 브랜치에서 기존 MuJoCo 코드와 현재 벨트 그리퍼·송이 하나의 소량 비교를 진행한다.
사용자 작성 중인 `gripper-design` 변경과 로컬 Git bundle은 별도 보존한다.

직접 GUI 재시험 `20261011_234926_536009_rail_harvest_gui`는 시작 준비 약165초,
18.4167sim/38.727loop초(0.476배속), 접근 중 조인트10개 동시 파단/겹침 중단이다.
원래 headless 시험과18.400초까지1,105개 표본의 pose/q/명령이 bit-exact이며,
다음 한 스텝에서만 급변한다. 사용자는 카메라만 조작했다고 확인했다.
GUI가 원인이라고 확정하지 않는다. 화면 갱신/앱 구동 차이의 정확한 원인은 미확정.
MuJoCo 시험은 별도 모델·출력이며 기존 Isaac 결과나 학습 데이터에 섞지 않는다.

## 2026-10-11 타깃만 충돌: 레일 이동·얕은 진입 실제 시험 (미완주)

사용자가 D(타깃 송이·자기 주줄기만 식물 충돌)를 선택했다. 이전 Plant29 D의 정상
완료 실제 상태에서 새 창을 구성. 비목표 식물 충돌 제외 정책이 창 전환 후 새 타깃에
정확히 재연결되도록 수정했다. 전체 농장 외형·로봇·집·거터 충돌은 유지한다.
사용자 후속 지시대로 주줄기를 피하는 수평 yaw와 얕은 진입/벨트 인입을 시험했다.

- Plant41 송이축 대비−45°, 기준점 깊이15mm, 동일 열매 집단03/04/05. 실제 레일71.38mm
  이동 후 닫기/열기2회 중03/05의 익은2개 분리, 미숙0. 36.8초의 마지막 열기에서
  하판`R/part_244`↔자기 주줄기10 callback 겹침1.385mm로 중단. 기준1mm 유지.
  후퇴·다음 송이 이동 미완료이며 정상 완료 checkpoint로 승격하지 않았다.
- 36.8sim/36.098loop초=1.019×. 준비·검사·native 초기화·렌더는 별도다.
  분리 전 안쪽 이동03=45.5mm/05=68.2mm. 벨트 단독 원인 비교는 아니다.
  주줄기 중심 약46.6mm 이동/복원과 명목 대비TCP 높이−9.23mm를 함께 확인했다.
- Plant41/43/31의 차단된 계획도 보존. 도착 전 로봇 위치로 송이 PCA 부호를
  고르던 오류를 도착 위치 기준으로 수정. Plant31은 거터 지지 와이어가 집단 중앙을
  지나 얕은 yaw 대안도 보류. Plant41의−75° 최종 자세는 가능했지만 준비 경로와
  회전수 정리/Cartesian/IK branch/리프트 대안은 자기충돌 또는2mm 계획 여유 미달.
  양수 여유 미달을 실제 관통으로 부르지 않는다. 추가 물리 실행0회.
- `--truss-entry-yaw-offset-deg` 선택형±90° 추가. 자연 송이축과 실제 진입 방향을
  분리해 집단 footprint/경로에 동일하게 적용하고 반복 시 각도를 누적하지 않는다.
  기본0° 명령은 유지. 표시용 disabled Skin의 CollisionAPI 정리 호환 오류도 수정.
  첫 native 초기화 오류는 수확 동작 전이었고 성공으로 세지 않는다. 관련177시험PASS.

결과 `/root/docker_share/mujoko_debugging_data/20261011_205000_target_only_row_harvest/index.html`.
작은 요약 `nvidia-sim/harvest_s300/validation/target_only_row_harvest_20261011.json`.
실제 상태1배속/작은 자막/전체 배경의 권장 영상2개(전체/정면 밝기보정).
가림/어두운 카메라 점검 원본2개도 index에 별도 보존. 포집·완주·전체 레일 수확
미검증, `training_eligible=false`, `harvest_success=null`. 미커밋 작업이며 기존 변경 보존.
다음에는 정적 진입각뿐 아니라 약9mm의 추종 오차와 당김 후 주줄기 복원을 고려해야
한다. 중단 상태를 숨기거나 이전 초기 상태로 되돌려 연속 성공처럼 집계하지 말 것.

## 2026-10-11 전체 농장 식물 충돌·탄성 속도 비교

Plant29 동일 명목 이동·수확/60Hz/CPU16을 네 조건에서 새 실행. 현재 탄성1그루는
82.8sim/93.01loop초=0.890×, 주변1그루 추가0.670×(55.93초 중단), 주변2그루 추가
82.8/123.60=0.670×, 타깃 송이·자기 주줄기 외 식물 충돌 제외82.8/76.52=1.082×.
각 익은 새 분리5/4/4/6, 미숙0/0/0/1(D Fruit09). 독립 시행이므로 합산 금지.
최초 미숙0 표기는 원본 summary에 맞게 정정했다. 주변1그루 조건은 분리된
Fruit03↔그리퍼 뒤 프레임2.157mm 겹침으로 중단. 기준1mm/물성은 유지했다.
추가 주변 식물↔로봇 접촉0회이므로 강하게 밀 때의 부하는 미측정. 충돌 제외는
21,187 collider를 끄며 전체 시각 배경·거터·로봇·타깃은 유지한다. D의 실제 초기
자세는 collision 변경 후 최대5.282mm 차이가 있어 동일 결과 최적화가 아니다.
전체 native 프로세스287.71~428.49초, 환경 검사75.70~155.97초로 준비/검사 비용이 남았다.
기존 기본 충돌 정책은 유지하고 선택형 파생 자산 생성기만 추가했다. 추가 탄성 식물의
checkpoint 다음 창 인계는 명시적으로 미지원. 관련89개 테스트PASS.
결과 `/root/docker_share/mujoko_debugging_data/20261011_194707_plant_interaction_comparison/index.html`.
동일 시점·전체 배경·실제 상태·작은 자막1배속 영상4개. 상세/명령/한계는
[ROW_CONTINUATION_KO.md](nvidia-sim/harvest_s300/ROW_CONTINUATION_KO.md).
`training_eligible=false`, `harvest_success=null`, 포집 미검증. 이번 비교는 커밋 전 작업이다.

## 2026-10-11 60Hz 두 송이 연속 이동·수확 완료

기존 `20261011_174730_released_same_truss_60hz`의 Plant22 실제 완료 상태(5개 분리)를
인계해 Plant29/Truss00을 새60Hz/CPU16/TGS64/4 실행으로 이어갔다. 실제 이동1.642386m,
도착오차0.0113mm. 리프트168.921→최저10.596→복귀160.606mm. 접근 전96mm 개방,
수평 진입, 양쪽 안쪽50mm/s, 닫기/열기2회·후퇴·복귀 완료. 새 익은02/03/04/08의
4개 분리, 앞선5개와 연결 합계9개이며 첫 송이는 재실행하거나 초기화하지 않았다.
두 번째 잔여 익은01/05/07의3개를 실제 마지막 자세에서 기존 집단 선택기로 재평가:
한 번에 들어오는 집단 최대2개라 최소3개 기준 미달, 추가 진입 없이 남김. 미숙 새 분리0.
두 번째82.8sim/92.358loop초=0.897×. 첫 송이 포함139.8sim/173.370loop초=0.806×.
장면 준비 약244.6초, 두 번째 native 프로세스 전체402.6초(환경검사154.4초 포함)는
별도다. 전체 작업이 실시간이라는 뜻은 아니다. 초기/60Hz 전체명목/실제 자기충돌PASS,
인계 관련90개 테스트PASS, 다음 인계 가능한 정상 완료 checkpoint까지 검증.
이전 붙은 식물은 측정 변형대로 고정, 분리 열매는 동적으로 유지하는 기존 window 방식.
카메라 판단·전체 레일·포집은 미검증. `training_eligible=false`, `harvest_success=null`.
결과 `/root/docker_share/mujoko_debugging_data/20261011_180823_continuous_harvest60/index.html`.
작은 자막의 실제 상태1배속 영상: 첫 송이/두 번째 이동 전체/두 번째 수확 탑뷰3개.

## 영상 표시 선호 (2026-10-11)

사용자가 자막이 화면을 너무 많이 가린다고 지적했다. 이후 S300 영상은
`render_motion.py`의 작은 기본 자막(`FontSize=10`, 얇은 외곽선, 하단 여백6)을 사용한다.
진단 문구는 유지하며 실제 스타일을 render JSON에 기록한다. 기존 MP4는 덮어쓰지 않는다.

## 2026-10-11 분리 후 같은 송이 접촉 허용, 60Hz 완주

사용자 승인에 따라 native 분리된 열매↔자기 송이의 과실·꼭지·중심가지·peduncle
접촉은 기록 후 계속한다. 실제 namespace/manifest/collider/native break 시각으로
확인하며 다른 송이·주줄기·팔/벨트 등은 기존 검사를 유지한다. 새 row 실행의 기본
인자에 반영했다. 직접/기존 case 실행은 `--released-fruit-same-truss-diagnostic` 명시.
같은 Plant22 경로1회: 이전28.467초 중단을 통과해57초 전체 동작·후퇴·복귀 완료.
익은 열매5개 분리, 익은2개/미숙3개 남음. 1진입 예산이며 송이 전체 수확은 아니다.
60Hz/CPU16/TGS64/4,57sim초/81.012loop초=0.704×. 같은 경로120Hz는151.138초,
이번 loop는46.4% 짧지만 분리 개수도6→5로 달라져 물리 동등 최적화는 아니다.
이전60Hz 중단28.467초까지1,709시점 pose/q/명령/시각/phase 정확히 일치.
초기·전체 명목 환경·실제 자기충돌PASS, 관련149테스트PASS. 물성·형상·1mm 유지.
낙과 접촉 예외로 전체 geometry의1mm 이하를 보증하지 않는다. `harvest_success=null`,
`training_eligible=false`, 포집 미검증. 리프트 복귀 편차는 사용자 요청으로 소수 시험의
blocker가 아니며 실제 시작q/목표 높이에 맞춰 계획한다. 제어기 수정은 하지 않았다.
결과 `/root/docker_share/mujoko_debugging_data/20261011_174730_released_same_truss_60hz/index.html`.
상세 [공통 접촉 기준](nvidia-sim/harvest_s300/COMMON_CONTACT_LIMIT_KO.md).
아래 같은 송이 접촉 중단 기록은 변경 전 역사다. 새 정책60Hz의 두 번째 식물 연속 실행은 미검증.

## 2026-10-11 분리 후 그리퍼 하판·지지대 접촉 중단 제외

사용자 요구에 따라 정확한 native 분리 후 좌우 하판/슬라이더 지지대4개와의 접촉은
별도 옵션으로 기록 후 계속한다. collision/물성/1mm 기준은 그대로이며 붙어 있는
과실 및 다른 접촉은 기존 검사를 유지. `run_row_harvest` 새 기본 인자에도 반영했고,
직접 실행/기존 cases에는 `--released-fruit-gripper-support-diagnostic`를 명시한다.
같은 Plant22 경로60Hz 재시험: 기존 중단27.5초를 통과, 추가1개 포함5개 분리 후
28.467초에 분리 Fruit03↔같은 송이 Pedicel_proximal_09_2 접촉으로 중단.
마지막 raw 겹침1.216mm/계산 후1.092mm, 아직 완주 아님. 하판·지지대 이외의
낙하 후 접촉 허용은 이번 검증 범위에 포함하지 않았다.
27.5초까지1,651시점의 pose/q/명령/시각/phase가 원본60Hz와 정확히 같음.
28.467sim/41.420loop초=0.687×(중도 종료). 초기/실제 자기충돌PASS, 관련144테스트PASS.
결과 `/root/docker_share/mujoko_debugging_data/20261011_170345_released_fruit_support_60hz/index.html`.
정책 상세는 [공통 접촉 기준](nvidia-sim/harvest_s300/COMMON_CONTACT_LIMIT_KO.md).

## 2026-10-11 이동 → 집단 수확 → 다음 송이 실제 연결 완료

`e47a04f` 이후 미커밋 작업. 별도 fresh 설치 장면에서 Plant22/Truss02 수확 후,
정상 최종 native checkpoint를 인계해 Plant29/Truss00까지 실제 이동·수확·후퇴 완료.
첫 실제 이동−20.000mm/새 분리6개, 다음+1,642.386mm/새 분리6개. 두 실행 모두
초기/전체 명목 환경/실제 자기충돌 검사 통과, 공통1mm 중단 없음. 익은 열매는 각1개
남고, 대상 밖 미숙 열매의 새 분리는0. 실패 재시험의 열매 수는 합산하지 않는다.
120Hz CPU16 TGS64/4:57sim/151.138loop초=0.377×,
82.792sim/165.526loop초=0.500×. 준비·계획·검사·렌더 제외이며 실시간 아님.
원본 바퀴가 바닥 이음부에 걸려 명령1.622m 중 실제0.798m만 움직인 문제를 발견했다.
별도 fresh 설치 높이2mm 보정, 실제 rail 도착 확인, 정지 개방 유지0.5초,
전체 진입/후퇴 환경검사 연결로 수정. 충돌 형상/물성/공통1mm는 유지했다.
수확 대기 자세 복귀 후 리프트 높이 편차가 회당 약8.47mm 남아 긴 반복 전 수정 필요.
전체808개 및 후속 바닥 검사5개(추가1개 포함) PASS. 영상3개 전체 디코드/프레임검사 PASS.
카메라 대상 선택/포집 성공/바퀴 구름·슬립은 미검증. `training_eligible=false`,
`harvest_success=null`. 상세는 [연속 실행 기록](nvidia-sim/harvest_s300/ROW_CONTINUATION_KO.md).
결과 `/root/docker_share/mujoko_debugging_data/20261011_140011_move_harvest_repeat/index.html`.
요약 `nvidia-sim/harvest_s300/validation/move_harvest_repeat_20261011.json`.

## 2026-10-11 소량 레일 이동·집단 수확 연결 시험

현재 `e47a04f` 이후 작업. 새 송이2개/준비 경로3개를 검사했다. Plant35/Truss02는
이웃 송이·지지 와이어, Plant37/Truss00는 주줄기 교차로 모두 실행 전 차단.
Plant37 대기 거리100→50mm도 차단되어 새 송이 수확 physics는0회다.
별도 레일 단독 실제 이동451mm를 확인했다. 첫 실행에서 리프트 하중 보정 목표가
측정q로 덮어써져8.370mm 처짐을 발견·수정했다. 같은 시작 상태의 독립 A/B 재실행:
최대 리프트 변화0.000105mm 미만, 실제 자기충돌PASS/중단 없음/새 분리0.
수정 후11.45sim초/11.931loop초=0.960×. 준비·장면 로딩·검사·영상은 별도.
새 active 식물의 잎 무시 정책 상속과60Hz 환경검사 연결도 수정. offline793개 PASS.
소스 물성/충돌체/공통1mm 유지. 기존 gripper-design 사용자 변경은 별도 보존.
이동+리프트 높이 변경+수확의 연속 완주는 아직 아니다. 두 레일 시험을90cm 연속 이동으로
합산하지 않는다. 새 카메라 판단은 미구현; 다음 병목은 그리퍼 전체와 주줄기를 고려한
준비·진입 위치 선정이다. `training_eligible=false`, `harvest_success=null` 유지.
결과/실제 이동 영상: `/root/docker_share/mujoko_debugging_data/20261011_131243_small_row_group_harvest/index.html`.
검증 요약: `nvidia-sim/harvest_s300/validation/small_row_group_harvest_20261011.json`.
기존 [레일 연결 문서](nvidia-sim/harvest_s300/ROW_CONTINUATION_KO.md)에 범위·한계 기록.

## 2026-10-11 수확 집단 중앙으로 접근 위치 보정

`HarvestGroupSelector`가 선택된 열매들의 측정 중심 평균을 두 벨트 사이에 맞추도록
그리퍼 국소X 방향만 보정한다. 기존 집단 선택 우선순위/송이 yaw/진입 깊이·높이를
유지하며, 보정 후 모든 선택 과실의 반경 포함 범위 및 대상 밖 과실 유입을 재검사한다.
식물 위치/물성을 바꾸지 않는다. 원래 기준점·이동량·보정 전후 오차는 선택 이력에 저장.
동일Truss_02 한 번 진입 native시험: 계획15.920mm 옆으로 이동, 캡처16.333초의 실제
집단 중심 좌우15.797→−0.122mm, 진입 끝−0.103mm. 두 닫기/완전 열기와 후퇴 완료.
대상01~05의5개 분리, 대상07/08 및 대상 밖09/10/11 남음. 초기/실측 자기충돌PASS.
53.5sim초/79.660loop초. 이전4개 분리 대비 이번5개이나 한 장면으로 수율 향상 일반화 금지.
offline789개 PASS. `training_eligible=false`, `harvest_success=null`, 포집 미검증.
결과: `/root/docker_share/mujoko_debugging_data/20261011_124843_centered_group_approach/`.
요약: `nvidia-sim/harvest_s300/validation/centered_group_approach_20261011.json`.
앞선 개방·2회 반복 변경과 함께 코드·문서·검증 요약을 커밋에 포함한다.
사용자 gripper-design 변경은 별도 보존하며 영상·대형 상태 기록은 공유 폴더에 유지한다.

## 2026-10-11 한 번 진입에서 닫기→완전 열기2회

`belt_open_close_cycle`은 진입 후 벨트 연속 회전 상태로 닫기/열기2회를 완료한다.
마지막 열림도 벨트 구동에 포함하고 개방 유지0.5초 동안 감속한다. 실제96mm 개방
확인 후 벨트 정지·열린 상태 후퇴. 이전처럼 마지막 열림 전에 벨트를 끄지 않는다.
접근 전 정지 개방, 동일 수평 접근/벨트 속도/공통1mm/물성은 유지한다.
native1진입 시험: 두 완전 개방 피드백95.999/95.999mm, 초기/실측 자기충돌PASS,
53.5sim초/77.179loop초, 대상4개 분리·3개 남음·대상 밖 분리0.
이번 실행은1진입 예산으로 종료했으며 전체 송이 완료가 아니다. 턱 반복2회와
새 집단으로 재진입하는 횟수를 혼동하지 말 것. 닫힘 중 실제 폭약33~34mm는 과실
접촉을 받는 상태이며28mm 명령에 완전 도달했다는 뜻은 아니다.
offline785개 PASS. 기존 명령 해시4개는 의도한 패턴 변경에 맞춰 갱신, 초기 로그 보존.
결과/영상: `/root/docker_share/mujoko_debugging_data/20261011_123435_two_close_open_cycles/`.
요약: `nvidia-sim/harvest_s300/validation/two_close_open_cycles_20261011.json`.
이번 및 바로 앞 접근 전 개방 수정은 위 중앙 정렬 수정과 함께 커밋에 포함한다.

## 2026-10-11 접근 전 그리퍼 완전 개방

수평 진입은 출발 자세에서 먼저96mm 개방한 뒤 리프트/팔 준비 이동·진입을 한다.
`--preparation-opening ready`가 기본이며, 기존44mm transfer 옵션은 이 모드에서
적용하지 않고 요청값/실제 적용값을 기록한다. 옛 동작 재현은 `legacy-staging` 명시.
각 이동 직전 실제 개방 피드백을 확인한다. 자세/방향/벨트/물성과 공통1mm 기준 유지.
같은 Truss_02/TGS60/64 실제1회: 개방 중 팔 명령 변화0, 이동 중 실측95.984~96.000mm.
53sim초/78.720loop초, 대상5개 분리 후 남은2개로 최소3개 규칙 종료, 대상 밖 분리0.
초기/전체 실측 자기충돌 검사PASS, offline784개 PASS. 다음 사이클도 코드상 같은
개방/검사 순서를 사용하나 이번 native 실행에서는 두 번째 사이클이 발생하지 않았다.
결과/영상: `/root/docker_share/mujoko_debugging_data/20261011_122521_open_before_approach/`.
문서: [HARVEST_GROUP_POLICY_KO.md](nvidia-sim/harvest_s300/HARVEST_GROUP_POLICY_KO.md).
요약: `nvidia-sim/harvest_s300/validation/open_before_approach_20261011.json`.
코드·문서·검증 요약은 위 중앙 정렬 수정과 함께 커밋에 포함한다.

## 2026-10-11 공통 겹침1mm 기본 적용 · 집단 수확 속도 비교

사용자 요청으로 현재 Isaac S300 수확 실행기의 공통 active-contact 중단값을1mm로
변경했다. 초기/실행/계산 후 형상 확인에 같은 값을 적용하고, 과실별 override를
생략하면 상속한다. 물성·충돌체·계획 충돌 여유는 유지한다. 과거0.5mm 결과 보존.
`--contact-limit-mm .5`로 이전 공통 값 선택 가능; 명시적 과실 override도 확인할 것.

알고리즘 커밋4d45485 이후: TGS60/32 + native 접촉 벡터 복사에서54sim초/59.955loop초
(0.901배속), 대상5개 분리/2개 남아 최소3개 기준 종료. TGS60/64는 두 사이클
106sim초/180.854loop초로 대상7개 모두 분리. 대상 밖3개 분리는 둘 다0.
빠른 설정의 미수확 과실 최대 이동약92mm(기존76mm)와 계산 전6.635mm 순간 겹침은
별도 기록한다. 계산 후 검사는 통과했지만 연속 비관통/물리 응답 동등성을 보증하지 않는다.
PGS60/120은 계산 후1.020~1.161mm 겹침으로 중단돼 채택하지 않았다.

782 offline 검사PASS. 복사 전후3,241시점·222,044접촉점 완전 일치.
빠른 solver/Hz/복사는 선택형이며 GUI/병렬 성능은 이번 미측정이다.
문서: [COMMON_CONTACT_LIMIT_KO.md](nvidia-sim/harvest_s300/COMMON_CONTACT_LIMIT_KO.md).
결과/실제1배속 영상2개: `/root/docker_share/mujoko_debugging_data/20261011_114941_common_1mm_harvest_speed/index.html`.
기존 알고리즘과 물리 실행/저장/재생기를 재사용했다. 아래0.5mm 표기는 당시 기록이다.

## 2026-10-11 최소 수량 기반 집단 수확 정책

`--harvest-group-policy`로 동일 송이의 대상 열매가 그리퍼 범위 안에3개 이상인
집단을 높은 쪽부터 선택하고, 실제 사이클 후 남은 수량/위치로 재평가한다.
고정 위/아래2회나 남은1개까지 개별 수확하는 정책이 아니다. 원래 모드는 보존했다.
현재 대상 표시는 자산의 빨간 RGB를 이용한 SIM-GT 대용 기준이며 카메라 숙도 인식은 아니다.
Truss_02 한 사이클: 대상7개 중5개 분리, 남은2개로 종료, 대상 밖3개 분리0.
초기/실측 자기충돌 검사PASS, 54sim초/152.656loop초. offline773테스트 PASS.
상세: [HARVEST_GROUP_POLICY_KO.md](nvidia-sim/harvest_s300/HARVEST_GROUP_POLICY_KO.md).
결과: `/root/docker_share/mujoko_debugging_data/20261011_112538_ripe_group_harvest/`.
학습 데이터 아님, 분리와 포집 성공은 구별한다. 앞선 미커밋 수확 기반 작업을 함께 보존한다.

## 2026-10-11 잎 접촉 제외 후 Truss_02 전체 10개 분리 확인

사용자 확정: 잎 때문에 회피하거나 수확을 중단하지 않는다. `leaf_contact_policy.py`로
별도 `assembly_leaves_visual_only`를 만들었다. 원본 잎 외형은 유지하고 정확한 잎 충돌체
8,152개만 비활성화했다. 잎의 저항을 생략하는 근사이며 실제 잎 접힘 모델은 아니다.
원래 주줄기·잘린 가지·꼭지·과실·로봇·거터 속성과 활성 접촉 0.5mm 기준은 유지한다.
모든 비잎 속성 일치 검사, 잎 제외/거터 유지 등 자동 테스트3개 통과. 원본 자산 보존.

정상 초기 상태에서 양쪽 안쪽50/50mm/s로9개 분리. 11번만2회 미분리였으며,
262초 완료 체크포인트의 실제 상태를 복원해 안쪽20/50mm/s로1회 추가 시도했다.
11번은26.083초 close 중 native 분리, 후퇴·복귀까지 완료. **같은 송이의 고유 열매10/10**,
첫 실행9개+재개1개다. 단일 중단 없는 엔진 실행이나10개 신규 콜백으로 중복 집계하지 않는다.
두 실행의 실측 전체 로봇 자기충돌 검사PASS, 잎 접촉 기록0건. 좌우 회피 보정0mm.
추가2mm 잎 회피 시험은 사용자 지적으로 폐기 방향으로 분류해 결과 폴더에 보존했다.

이미 분리된 열매의 바닥·기체 접촉에는 기존 진단 예외가 적용된다(첫 실행12접촉점,
재개7접촉점). 낙과 충돌 정확도/포집/실물 파단/다른 송이 반복 성공은 미검증이다.
`training_eligible=false`, `harvest_success=null`, `collection_verified=false` 유지.
결과와 실제 저장 상태 영상:
`/root/docker_share/mujoko_debugging_data/20261011_095724_inward_full_truss/index.html`.
명령은 같은 폴더 `leaf_ignored_recipe.json`, `remaining11_20_50_recipe.json`.
요약 `nvidia-sim/harvest_s300/validation/inward_leaf_ignored_full_truss_20261011.json`.
현재 변경은 미커밋. 아래7/10 잎 중단 기록은 이 정책 적용 이전의 결과다.

## 2026-10-11 양쪽 벨트 안쪽 이송 재시험 — 첫 닫힘 7/10 분리, 잎 접촉으로 중단

기존 fresh Truss_02 9/10 모델/경로/120Hz/물성/0.5mm 기준으로 재시험했다.
`--belt-speed-m-s 0 --belt-infeed-speed-m-s .05`가 양쪽 접촉면의 안쪽 이송이다.
기존 common-infeed 부호 오류를 실제 native 구체 이송으로 확인해 수정했다:
local-Y API 명령 -20mm/s는 구체를 입구(+Y)로40.008mm, +20mm/s는 안쪽(-Y)으로39.984mm 이동시켰다(2초).
새 계획에 `native_positive_local_y_inward_v2`를 기록한다. infeed=0인 기존 반대 방향 명령은 그대로다.
저장된 옛 음수 common-infeed 결과를 안쪽 이송 성공으로 해석하지 않는다.

정상 초기 상태, 벨트 시작19.5초까지 기존 baseline의277바디 pose/관절/명령 완전 일치.
수정 방향에서는 첫 close에서01,02,03,04,05,07,08번7개 native 분리(기존 같은 구간3개).
26.025초에 `L/part_209`(CAD `L_C13_outer_finger_cover`)와 `Leaf_CompoundLeaf_04/Appendage_140_0`
접촉으로 중단. post-solve convex 상한0.503177mm, callback 약0.506070mm, 최대 접촉점힘11.402N.
09/10/11은 미분리. 실제 자기충돌 검사PASS, 전체 수확/포집 성공은 아니다.
26.025sim초/102.392실행loop초. 기존418초9개 결과와 전체 성공률을 단순 비교하지 않는다.

첫 부호 오해 실행과 중단한 재실행도 보존했다. 그 과정에서 이미 분리된 열매의 정적
`Foundation_6x10m/Mesh` 낙과 분류 누락을 수정했다. 정확한 정적 바닥·native 파단 이후에만
기존 opt-in 낙과 예외를 적용하며 벨트/팔/붙은 열매/잎의 물리와0.5mm 기준은 유지한다.
관련95자동 검사PASS, 방향 native 대조PASS. 결과/동일 시점 실제 상태 영상:
`/root/docker_share/mujoko_debugging_data/20261011_085604_both_belts_inward_harvest/index.html`.
요약 `nvidia-sim/harvest_s300/validation/both_belts_inward_20261011.json`.
이번 변경은 미커밋. `training_eligible=false`, `harvest_success=null`, `collection_verified=false`.

## 2026-10-11 잎의 수동 탄성 연결과 수확 재시험 — 수확은 미완료

`nvidia-sim/harvest_s300/compliant_foliage.py`로 별도 모델에 활성 식물의 잎27개를
수동3축 연결로 분리했다(원래17+V9추가7+어린잎3). 초기 외형/충돌체/원래 충돌 제외 관계 보존.
식물612→693DOF, 전체248→275강체. 잎 질량·강성은 미보정 추정이며 잎자루 회전만 표현한다.
정지 통과. 초기17개 모델의0.05N/1초 하중은28.506° 회전 후 잔류0.021°로 복원했다.
층별 USD overlay 초기 발산을 동일 합성 형상의 materialized USD로 해결했다. 내부 엔진 원인은 미확정.

수평 송이 방향163.905°/상단01·02번 기존 경로 재사용(명령 동일성 확인).
초기17개 모델에서 native callback1.415mm와 계산 후 cooked형상 겹침 상한0.304mm가 달랐다.
`--verify-leaf-contact-after-step` 선택형 진단은 **판정 시점이 달라 기존 callback 검사와 동등하지 않다**.
0.5mm 수치와 원래 기본 검사 보존. 확인 불가/초과는 중단, 스텝 사이 비관통 보증 아님.
최종27개 모델도12.700초 진입 중 잎16/고정 배경 잎 접촉0.616mm에서 중단, 분리0.
목표01/02 최대 이동82.0/79.3mm. 접촉점 최대힘은 초기17개6.021N→27개2.450N이나 정상 수확은 아니다.
65→45mm 얕은 경로는 link4/link2 여유1.830mm<기존2mm로 계획 차단(실제 관통과 다름).
주변 잎은 여전히 고정 triangle mesh. 잎의 국소 굽힘/축 방향 압축과 밀린 송이에 대한 접근은 미해결.

최종55테스트PASS. 결과/실제 상태 영상:
`/root/docker_share/mujoko_debugging_data/20261011_004347_compliant_leaf_harvest/index.html`.
최종 모델 `assembly_all_leaves_v2`, 작은 요약 `nvidia-sim/harvest_s300/validation/compliant_foliage_20261011.json`.
상세 `nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md` 마지막 절.
별도 모델/결과만 생성, V6 사용자 작업 보존, 미커밋. `training_eligible=false`, `harvest_success=null`.

## 2026-10-11 Isaac 그리퍼 탑뷰 송이 방향 yaw 정렬

`nvidia-sim/harvest_s300/run_motion.py --approach-yaw-mode truss-axis` 추가.
붙어 있는 열매의 XY 주축으로 그리퍼 yaw/전진 경로를 맞추고 pitch/roll은 수평 유지.
기존 robot-radial 기본과 sequence60 명령은 보존. 현재 송이 전진각은 월드163.90°,
기존175.94°와12.04° 차이. 아래10·11번은 자기충돌 계획 검사에서 차단, 위1·2번은
계획 통과 후 실제60Hz에서 옆판/주줄기 부속 형상 겹침으로10.02초 중단(분리0).
방향 수정 완료와 수확 성공을 구분한다. 관련43테스트와 기존 실제 USD 명령 회귀 통과.
결과 `20261011_001548_truss_yaw_alignment/index.html` (docker_share 기존 data 아래),
상세는 `nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md` 마지막 절.
이번 변경은 아직 커밋하지 않았다. 사용자 gripper-design V6 작업은 보존한다.

## 최신: 한 송이 전체 연속 수확 시도 — 미완료 (2026-10-10)

`--whole-truss`로 같은 native scene에서 남은 열매 위치를 읽어 다음 접근을 계획한다.
cycle 사이 식물/열매/로봇 상태 reset이나 파단 조인트 복구가 없다. 현 Truss_02의10개를 정확한
native joint ID로 집계하며, 이미 분리된 열매는 다음 목표에서 제외한다. 기존 runner/IK/FCL/분석/영상 재사용.
7회 물리 시험: 아래부터60/120/240Hz, 위부터60/120Hz, 위부터 높이 선이동60Hz,
높이 선이동+진입 깊이65→35mm60Hz. **전체 수확 완료0회, 가장 많은 분리5/10개**.
아래부터60Hz는7·8·9·10·11번 분리 후9번 낙하/바닥1.329mm 겹침 중단.
120/240Hz는4개 분리 후7번 과실/9번 꼭지 가지0.534/0.588mm 겹침 중단.
위쪽 직선 준비는잎에 차단. 높이 선이동으로 접근은 했지만 송이 밀림/주변잎1.831mm 겹침 중단.
얕은 진입은 최대겹침0.279mm이나 분리0, 실제 재개방91.1mm로96mm 미도달하여 재닫기 전 중단.
나머지01·02·03·04·05를 정상 수확했다고 보고하지 않는다. 다음은 움직인 송이를 고려한 포획 위치/장애물 회피 경로가 필요하다.
물성·3N/0.008Nm·충돌체·actuator·0.5mm 기준 보존. 기존 탄성 연결과 벨트 접촉면 시험안을 재사용한 미보정 진단이다.
자동 관련100테스트PASS, 실제 자기충돌7회PASS, 연속 timestamp/측정 위치 연결 확인.
60Hz 첫39초 명령/관절/식물 pose가 이전2개 분리 실행과 bit-exact. 실패도 모두 보존.
결과 `/root/docker_share/mujoko_debugging_data/20261010_203914_whole_truss_harvest/index.html`.
상세는 [CONTROLLED_BELT_HARVEST_KO.md](nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md) 마지막 절.
`training_eligible=false`, `harvest_success=null`. 현 턴에서 커밋/푸시하지 않음. 사용자 별도V6 변경 보존.


## 최신: 연속 벨트 개폐 패턴 및 닫기 중 분리 허용 (2026-10-10)

사용자 정정: 열린 채 벨트 시작 → 원래 최소28mm까지 닫기 → 벨트 유지하며96mm 완전 열기
→ 다시28mm 닫기. 닫기/열기 중 분리는 실패가 아니다. 이후 벨트를 끄고 열어 후퇴한다.
`belt_open_close_cycle`/live `cycle` opt-in 추가, 기존 명령/물성/파단값 보존.
양쪽 표면−0.05/+0.05m/s, full612DOF, 같은 탄성 연결3N/0.008Nm·5mm 접촉면 시험안.
60/120Hz 모두10·11번 분리 후39초 모션 완주. 모든 활성 접촉0.5mm 기준과 실제 자기충돌 통과.
60Hz 최대겹침0.187mm/47.358초 계산(0.824배속),120Hz0.246mm/90.310초.
60Hz 둘 다 첫 닫기 중 분리;120Hz10번 닫기 중/11번 재개방 시작 때 분리.
재닫기 전 실제95.994mm 개방 확인. on/off 명령/시간 및 벨트 이전 실제상태 일치.
벨트off도10번 분리이므로 모든 분리가 벨트 덕분인 것은 아님. 실물 보정·포집·240Hz 미검증.
기존 이송(좌−0.10/우0)60Hz는2개 분리 후11번/벨트0.632mm 겹침 중단,120Hz 완료.
headless5회와 실패를 모두 보존,91테스트PASS. training_eligible=false/harvest_success=null.
결과 `/root/docker_share/mujoko_debugging_data/20261010_201647_continuous_belt_cycle/index.html`.
실행 `bash /root/docker_share/mujoko_debugging_data/20261010_201647_continuous_belt_cycle/run_cycle_gui.sh`.
상세: [CONTROLLED_BELT_HARVEST_KO.md](nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md) 마지막 절.
기존 아래 기록의 '닫기 중 조기 파단' 표현은 벨트off 닫기였던 당시 비교의 해석이며 현재 성공 조건으로 적용하지 않는다.


## 최신: 잡는 간격·높이 개선 및 실제 화면 재검증 (2026-10-10)

같은 탄성 연결 시험안(3N/0.008Nm)에서 간격40mm/기준점보다10mm 낮은 진입으로
60·120Hz 모두 닫기 중 조기 파단 없이10번 과실이 벨트 단계에 분리되고 후퇴까지 완료됐다.
60Hz GUI도 원래0.5mm 기준으로 완료: 최대 겹침0.192mm,30.5sim초/38.586wall초(0.790배속).
headless와 실제 robot q/식물 pose/명령/시간이 bit-exact. 두 Hz의 벨트off 대조는 분리0.
전체 headless15회+GUI1회, 자동 테스트89개 통과. 실패/분리0도 모두 보존했다.
240Hz/40mm는 닫기 중 조기 파단,41·42mm 비교도 주파수에 따라 분리 여부가 달라
시간 간격에 수렴한 수확 모델을 완성한 것은 아니다. 실물 보정/포집/새 병렬 성능은 미검증.
기존 원본·물성·충돌체·actuator 보존, 간격/높이 opt-in과 live 환경변수만 추가했다.
target 복사 메타데이터의 파단값을 실제 USD와 맞춘 수정도 포함하며 USD 해시는 이전과 같다.
상세/재실행: [CONTROLLED_BELT_HARVEST_KO.md](nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md).
결과 `/root/docker_share/mujoko_debugging_data/20261010_185843_controlled_belt_harvest/index.html`.
커밋 여부는 `git status`로 확인한다. 별도 gripper V6 작업은 수정하지 않았다.

## 최신: 열매–꼭지 탄성 연결 비교 (2026-10-10)

기준 `76479ee`, 별도 assembly에서 현재 송이10개 과실 연결만 고정→수동 탄성 회전으로 변경했다.
3N 유지, 파단 토크0.08/0.008Nm를 고정/탄성으로 나누고 벨트off 대조 및120Hz를 포함해 농장6회 실행.
60Hz 탄성/0.008Nm: 10번이벨트단계19.283초에 native 분리, 전체30.5초를37.469초에완료(0.814배속).
같은명령/벨트이전실제상태가일치하는 off대조는분리0. 원래0.08Nm의탄성은11번비틀림46.35°이나분리0.
고정약화는닫기중3개분리후2.098mm겹침중단. 120Hz탄성약화도닫기중8·10번분리후1.099mm겹침중단.
따라서 한 사례의개선증거이며 시간간격에안정적인모델/실물파단/포집성공은아니다. 기본모델/물성/접촉기준보존.
87자동검사와60/120Hz단독fixture통과. native이벤트만으로힘/비틀림/굽힘파단원인을구분할수없다.
상세·영상·재실행: [FRUIT_ATTACHMENT_TEST_KO.md](nvidia-sim/harvest_s300/FRUIT_ATTACHMENT_TEST_KO.md).
결과 `/root/docker_share/mujoko_debugging_data/20261010_182133_fruit_attachment/index.html`.
현재 변경의커밋여부는 `git status`로확인한다. 사용자V6작업은수정하지않았다.

## 최신: 벨트 수확 속도·접촉 허용치 비교 (2026-10-10)

같은 full612DOF/전체충돌/5mm 접촉면 시험안에서30.5초 동작을 비교했다.
기존480Hz221.770초 → TGS120/32는52.691초(약4.21배 개선),10번 native 분리·후퇴 완료.
TGS60/64는38.126초(0.800배속)에 완료하지만 분리0이므로 빠른 모션 확인용으로 구분한다.
현재 송이 열매–그리퍼만1mm로 완화한 opt-in 진단이며, 다른 활성 접촉/초기상태는0.5mm 유지.
0.5mm 초과는60Hz 오른쪽상판–7번0.948mm,120Hz 왼쪽상판–8번0.588mm 각각한물리표본.
새 headless16회/GUI2회,81테스트PASS. GUI도38.526/52.354초, headless와 실제상태bit-exact.
다른저주파설정은최대4.1mm겹침으로제외했다. full식물/물성/파단값은유지하며 새병렬성능은미측정.
상세·모든실패·GUI실행법: [HARVEST_SPEED_CONTACT_KO.md](nvidia-sim/harvest_s300/HARVEST_SPEED_CONTACT_KO.md).
결과 `/root/docker_share/mujoko_debugging_data/20261010_173338_s300_harvest_speed/index.html`.


## 최신: 여러 열매 접촉·그리퍼 접촉면 시험안 (2026-10-10)

- 기존 그리퍼는10·11번 중간으로 진입/6초 닫기/480Hz에서 벨트2초와 개방·후퇴까지 완료했지만 분리0.
- 별도5mm 접촉면 시험안+감속 벨트+scene CCD에서 Truss_02_Fruit_10 native 분리와 전체30.5초 동작 완료.
  원래 CAD/V6 유지, 표시·충돌2개와 질량74.88g을 명시적으로 추가한 시험안이다. 물성/파단값/0.5mm 기준 유지.
- 전체9회(컨트롤러 timeout1 포함) 기록. 빠른 벨트/낮춘 높이/바닥 CCD off 실패도 보존.54테스트PASS.
- 480Hz 검증용이며 실시간 아님, 제작/실물 안전/동시 다수 수확 검증 전. 상세 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md).
- 결과 `/root/docker_share/mujoko_debugging_data/20261010_161007_s300_cluster_grasp/index.html`. 요약 `validation/cluster_grasp_20261010.json`. 학습제외, 커밋 여부는 `git status`.

## 최신: S300 벨트2초 수확 순서 연결·실제 분리 미달 (2026-10-10)

- 기준 `d54b498`, `--harvest-profile belt_spin_release`: 벌려 진입 → 닫기 → 대향 벨트0.15m/s·2초 → 벌려 후퇴. 기존 legacy/entry-only 명령은 그대로다. 실제 송이/CAD/V6작업/충돌체/물성/파단값 보존.
- 같은11번 열매의120Hz는14.0833초 닫기 중 왼쪽상판–옆10번0.5059mm 관통으로중단. 240Hz는14.5875초 오른쪽벨트–목표11번0.5309mm 관통으로중단, 벨트실행0.0875초. 목표분리0/양쪽포획미확보/수확성공아님.
- 목표폭23.053mm 대 최소벨트간격28mm. 전체30개 활성과실의 최대두점거리도27.235mm 이하. 현재 기구의 잡기 범위 및 옆과실 상판간섭이 우선 문제. 고정 과실–꼭지 조인트의3N/0.08Nm 파단은 유지했고 시간에 따른 강제분리 없음.
- `analyze_belt_trial.py`에 실제 단계시간/좌우접촉/열매–꼭지 상대회전/정확한 파단조인트 측정. 관련42테스트PASS, 기존명령 회귀일치,120Hz 진입까지1,381개 실제상태bit-exact. raw/영상: `/root/docker_share/mujoko_debugging_data/20261010_153918_s300_belt_spin_release/`. 상세 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md). 커밋 여부는 `git status` 확인.

## 최신: S300 실시간 GUI 미리보기 (2026-10-10)

- 기준 `56c5d01`, 브랜치 `codex/s300-harvest-isaac-20261009`. 기존 물리 실행기/실제 상태 뷰어를 별도 프로세스로 연결하는 `run_motion_live.sh` 추가. VNC `DISPLAY=:0` 화면 준비 후 시작, 완료 시 상태/접촉/HTML 저장. 새 변경의 커밋 여부는 `git status` 확인.
- 같은 `assembly_v3`의23.5초 진입·유지·복귀: 기존120Hz 동기GUI95.304초 → 분리GUI120Hz40.388초(0.582배), **선택형60Hz23.500초(1.000배)**. full612DOF/전체충돌31,464개/자기충돌/물성/0.5mm관통기준 유지. 로딩·검사·저장시간별도.
- 60Hz는 다른 수치 동역학이다. 같은0.1N 부하 최대변위15.011→16.095mm(+7.23%), 복원확인. reference120Hz 유지. 120Hz 분리GUI/기존headless 전체2,821표본 bit-exact,60Hz 동설정 재현도일치. 최종GUI 두실행 자기접촉0/실제자세검사PASS, 관련53테스트PASS.
- 실행 `bash /root/docker_share/mujoko_debugging_data/20261010_145123_s300_realtime_review/run_gui.sh`; `reference`를 붙이면120Hz. 같은폴더index.html에비교/로그/실제화면. 상세 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md), 요약 `validation/realtime_preview_20261010.json`.
- 열린 그리퍼 진입·유지·복귀만 검증. 닫기/벨트/분리수확/새GPU병렬은 미검증, 학습제외유지. 초기런처/가상화면시작오류도보존. 원본/별도gripper V6작업은유지.

## 최신 수정: S300 팔–그리퍼 자기충돌 수정·실제 재검증 (2026-10-10)

- `codex/s300-harvest-isaac-20261009`, 기준 `aad8340` 이후 수정. 코드·테스트·문서·작은 검증 JSON을 함께 커밋하며 실제 상태/영상/대형 USD는 아래 결과 폴더에 보존한다. native 로봇 자기충돌 활성화, 그리퍼 슬라이더 부모를 실제 고정 레일로 수정하고 초기 IK 자세/접근을 다시 선택했다. 목표11번 유지, 수평 대기160mm → 준비60mm → 진입100mm → 유지 → 복귀. 비인접 팔/그리퍼 충돌 제외 추가 없음.
- 새 `assembly_v3`의 headless/GUI 모두23.5sim초 완주. 각2,821개 실제 자세 교차 검사PASS, native 자기접촉0, 상태/명령bit-exact. 계획120Hz/2mm여유 검사도PASS. 물성/형상/구동게인/0.5mm관통기준 유지, full식물612DOF/활성충돌31,464개 유지.
- 진입pitch최대0.0331°, 진입TCP높이범위0.150mm. 초기중력정착TCP하강8.66mm는 별도 기록. headless38.62초(0.608배속), GUI95.30초(0.247배속), 로딩/검사/저장별도. 첫2회초기관통실패도보존. 관련37테스트PASS.
- 결과/실제영상/실행스크립트: `/root/docker_share/mujoko_debugging_data/20261010_142120_s300_self_collision_fix/`. 상세 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md) 최신 절. 아래 "수정/재실행 미실시"는 과거 기록이다.
- 검증 범위는 진입·유지·복귀다. 과실닫기/벨트이송/수확분리 및 새GPU병렬은 미검증. `training_eligible=false`, `harvest_success=null` 유지.

## 최신 정정: S300 영상 기준 자세에 그리퍼–팔 자기관통 (2026-10-10)

- 사용자 지적 후 실제 저장 pose를 FCL 원본 삼각형 교차로 검사했다. 대기·준비·복귀에서 그리퍼 프레임/레일과 RB5 `link2`의 교차 확인. 로봇 `enabledSelfCollisions=false`이므로 기존 native 접촉 감시가 검출하지 못했다.
- 아래23.5초 GUI 완주는 명령 완료이며 **유효한 비충돌 경로가 아니다**. 28테스트/수평FK 통과도 자기충돌 검증이 아니다. 기존 원본·영상·접촉은 보존하고 보고서에 후속 정정을 추가했다. 초기 자세·경로·자기충돌 설정의 수정/재실행은 아직 하지 않았다.
- 12개 저장 시점 검사,2초24개/4초28개 메시쌍 교차. 깊이/힘 추정이나 연속 비충돌 보증 아님. 근거 `20261010_141508_s300_self_collision_audit/mesh_intersection_audit.json`; 자세한 정정은 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md).

## 최신: 영상 기준 수평 대기·진입·복귀 완료 (2026-10-10)

- inaho 영상49~69초를 확인하고 S300/RB5에 `horizontal_ready`를 추가했다. 초기부터 수평 대기, 96mm 선개방, Cartesian 수평 접근/진입을 사용한다. 하부 `L_T01_sloped_receiver` 및 전용 체결부4개의 외형·충돌체와 추정질량167.664g을 제거했다. 다른 활성충돌체/식물속성은 보존했다.
- 바깥쪽 `Truss_02_Fruit_11`의 GUI `--entry-only`가23.5sim초 전체완료: 진입→2초유지→후퇴→대기복귀. 실제진입pitch최대0.0171°, 전체TCP높이변화0.256mm, 목표변위최대0.0316mm, 관통중단없음. 수확시도/성공은 아니다.
- 같은11번의 전체동작은 진입후 닫기13.233초에 왼쪽상판과08번과실의1.522mm관통으로중단. 기존02번은열매/잎충돌로진입중단. 모든시도·초기USD구성실패도보존했다.
- GUI23.5sim초/92.714wall초=0.253배속. 공통0~11.5초의GUI/headless실제상태·명령1381표본bit-exact. 관련28테스트PASS. 기존수집/학습제외유지.
- 결과 `/root/docker_share/mujoko_debugging_data/20261010_134811_s300_video_entry/`. `run_gui.sh`로완료된진입·복귀시연재실행. 상세·실패·한계·명령: [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md). 커밋 여부는 `git status` 확인.

## 최신: S300 그리퍼 수평 진입 수정 (2026-10-10)

- `codex/s300-harvest-isaac-20261009`, 수정 기준 `aad8340`. 수확 진입 pitch 기본0° 고정, 기존 자동0/-30/-60/+30° 선택 제거. yaw는 명시적인 월드Z 수평 방향 보정이다. 현재타깃 radial0° 준비 IK 미발견, yaw+15/+30°의100mm 수평 경로 FK 통과.
- 기존 SI 실행의 앞 두 준비 구간을 그대로 재사용하는 opt-in과, RB5 wrist1을 보정하는 `tight_fold_level_approach`를 추가. 실제 composed FK/관절 제한을 확인하며 기존 준비 profile은 유지한다.
- 관련24테스트PASS. 실제5회 모두 approach 전 주변/활성 식물 접촉·관통으로 중단했다. 원본 prefix 재사용3회는 처음12초 명령·time·phase·실제bodypose가 원본과bit-exact였다. 수평 진입 완주나 수확 성공은 미확인이다.
- 기록/영상: `/root/docker_share/mujoko_debugging_data/20261010_125732_s300_horizontal_entry/`. 변경범위·모든실패·GUI명령: [HORIZONTAL_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/HORIZONTAL_ENTRY_STATUS_KO.md). 새 결과는 학습 제외이며 커밋 여부는 `git status` 확인.

## 최신 정정: S300 native stage 단위·축 및 SI 재검증 (2026-10-10)

- 첫 118개 성능/수확 연결 trial의 assembly root에 `metersPerUnit`·`upAxis`가 빠져 실제 stage가 0.01m/unit·Y-up fallback으로 동작했다. 아래 이전 속도·변위·힘 기록은 역사적 수치이며 **검증된 SI 농장 성능으로 해석하지 않는다**. 같은 잘못된 stage의 bit-exact PASS도 SI 타당성 증거가 아니다. 기존 결과와 코드2964d4a는 보존했다.
- builder가 농장 source의 metre/Z/kg metadata를 전파하도록 수정했고 native 실행 전 fail-fast 검사를 추가했다. 올바른 metadata만 씌운 wrapper는 초기 파단에 실패했으므로 원인이 단위 숫자만이라고 단정하지 않는다. 실제 schema/spec을 다시 생성한 `legal_motion_assembly_metre_z_rebuilt`를 새 기준으로 사용한다.
- 새 full SI 모델은 235바디/612 DOF/605 활성 collider와 배경 전체 충돌을 유지한다. PGS120/64/4의 7초0.1N 부하·복원은10.5246loop초에 완료(0.665sim-s/s), 초기화·관통 통과/파단0, 목표 최대7.018mm/종료0.0104mm다. 이전 24worker7.04sim-s/s를 새 SI 처리량으로 승격하지 않는다.
- 새 SI 독립 CPU8/24worker의 합산 처리량은5.915/7.019sim-s/s, 준비·저장 포함2.059/2.835sim-s/s다. peak RSS 합29.77/89.23GiB. full 32worker 전부 단일 SI 기준과 pose/native q/명령/접촉 bit-exact였다. 234DOF/TGS60의 별도24worker13.438sim-s/s는 바뀐 동역학이며 full 모델과 동등하지 않다.
- 새 SI 실제 wide-open 접근은29.917sim초/49.452wall초에 목표 과실–벨트 관통0.536mm로 중단, 최대 밀림43.457mm/파단0이다. 닫기·벨트 구간까지 진행하지 못했고 수확 성공이 아니다. full SI의30mm 합성 구 대조군은40.560mm 이송했지만 D6 guide fixture이며 농장 과실 수확을 입증하지 않는다.
- cold layer를 새로 여는 CPU namespace clone1은 full SI 기준과 pose/q/명령/12900접촉 bit-exact로 정상화됐다. GPU2는 capacity 오류 및 SDK의 readback reset을 구분·수정한 뒤 실제 자기 식물 접촉2.653mm/파단30개로 중단했고, 마지막 GPU 활성화 지연 대조도 실패했다. 기록된 cross-env 접촉0; native 원인 미확정. GPU8은 입력만 준비, **미실행**이다.
- 최종 설치 SDK 검사는151개 중150PASS/1skip(21.031초), 최종 보고서20개 추가 검사PASS. Torch 미설치로 skip된 unsigned actor-ID 저장 검사는 별도 Python3.11/Torch에서1PASS했다. 실제 SI 접근 전체/접촉 확대 영상과 guided ball 벨트 대조 영상3개는 저장 pose 재생이며 물리 재실행 없이 전체 디코드를 통과했다. raw 로그/MP4/입력은 `20261010_s300_performance/`, 작은 검증·해시는 `nvidia-sim/harvest_s300/validation/performance_20261010.json`에 보존한다.
- 새 SI pool/접촉/실제 접근 결과와 제외한 빠른 설정은 [PERFORMANCE_STATUS_KO.md](nvidia-sim/harvest_s300/PERFORMANCE_STATUS_KO.md)의 최신 절과 `20261010_s300_performance/index.html`에서 확인한다. 목표 포획·분리 수확 및 유효한 GPU 병렬 처리량은 아직 미확보다. 원본/실패/학습 제외를 유지한다.


## 최신: S300 Isaac 속도 최적화·24 CPU 환경 검증 (2026-10-10)

- 브랜치 `codex/s300-harvest-isaac-20261009`. 원본 USD/CAD/실험을 보존했다. 상세 [PERFORMANCE_STATUS_KO.md](nvidia-sim/harvest_s300/PERFORMANCE_STATUS_KO.md), 코드/검증의 커밋 여부는 `git status` 확인.
- 새 `legal_layout_v2`에서 기존 합법 Y회전/부착 배치만 조정해 활성 식물의 초기 겹침을 해결했다. target Truss02는 그대로이며 메시·충돌체·질량·관성·탄성·파단 기준을 유지했다. 배경 충돌을 끄지 않았다. 활성 Plant22는 235바디/612 DOF/605 활성 collider, 주변 식물은 full triangle 고정 장애물이다.
- 최소 native app의 `--physics-only`, 물리와 별도인 표시/replay, 접촉 버퍼를 추가했다. 실제 CPU/GPU dynamics·broadphase·Hz를 검사한다. 초기화 중 중력만 중립화하고 고정 preload 계산 뒤 원래 중력을 복구한다. 초기조건 개선과 bit-exact 코드 최적화를 구분했다.
- CPU PGS120/64/4, full 식물의 동일7초 부하·복원 시험: 단일10.647초(0.657 sim-s/s); 병렬2/4/8/16/24개는 합산1.289/2.329/5.873/6.564/7.042 sim-s/s. **24개는168sim초/23.855loop초, 시작·저장 포함59.821초**다. peak RSS 합계 약87.36GiB(공유 페이지 중복 집계 가능). 효율 기본8개, 최고 실측 처리량24개. 모든54 worker의 바디 자세/관절/접촉이 단일 기준과 bit-exact였다.
- 각 환경은 목표 식물 한 개만 full 탄성이며, 모든106송이의 동시 탄성화나 병렬 수확 성공률 시험이 아니다. 단일 PGS120는 아직 실시간 미달이다. GPU dynamics는 느렸고 native namespace 복제는 초기 파단으로 invalid; GPU 대규모 학습 준비 완료로 해석하지 않는다. CPU/SAP는 차이 없음, CPU solver+GPU broadphase는 더 느렸다.
- 60Hz PGS는 부하 벨트 접촉에서 관통0.5mm를 넘었다. TGS64/60은 부하 벨트 대조군은 통과하지만 탄성 응답이 달랐다. 120Hz를 모든 접촉의 보증으로 취급하지 않는다. 충돌/마찰/물성/actuator/기준을 바꿔 성공시키지 않았다.
- 새 좁은 회전 준비 경로는 목표 앞까지 실제 도달했다. 준비 후44→60mm 개방도 시험했다. 실제4회는 삽입 단계의 과실/자기 가지 또는 잎 접촉·관통으로 중단했고 닫기·벨트는 미도달. 240Hz의30.7417초에 실제 목표 꼭지 파단이 발생했으나 다음 스텝에 관통 중단되어 검증된 포획/수확이 아니다. 유한30과실/기존 경로 검사에서 대체 clear 경로 미발견이며 절대적 불가능 판정은 아니다.
- 30mm 합성 구의 native 벨트 이송 약40.56mm는 확인했으나 별도 D6 guide 시험이며 농장 수확 성공이 아니다. 모든 결과 `training_eligible=false`, `harvest_success=null` 유지.
- 테스트120개 중119통과/별도 SDK 경로에서 Torch 검사1skip. 실제 native 실행은 Torch tensor를 사용했다. 결과/실제 저장 상태 MP4: `/root/docker_share/mujoko_debugging_data/20261010_s300_performance/index.html`. 호스트 `file:///home/rdv/docker_share/mujoko_debugging_data/20261010_s300_performance/index.html`.
- 다음 병목은 준비가 아니라 밀집 송이의 insert/grip 경로다. 기본 경로는 −60° 기울기를 선택해 receiver가 다른 Fruit04를 먼저 밀고, 목표 Fruit02는 왼쪽 벨트 앞단에서 밀리며 의도한 그립 깊이에 들어오지 않는다. 자세/경로와 실제 추종·접촉을 함께 개선해야 한다. persistent worker reset/다양한 수확 동작의 병렬 동등성은 아직 미검증이다.

## 최신: S300 수확 구동·탄성 연결 시험 (2026-10-10)

- `codex/s300-harvest-isaac-20261009`. 원본 USD/CAD 보존, 별도 `assembly_v5/scene.usda`에 V5.1 등가 개폐/벨트 표면 속도와 Plant_22 full 탄성 연결.
- 뒷판↔리프트 선반 교차를 발견해 새 동작 모델의 리프트·RB5만 Y−0.20m 이동. 뒷판 유지. 기계 체결 검증은 아님.
- 분리 시험: 개폐 실제28.02~60.01mm,0.1N 과실 외력 최대7.18mm/제거 후0.43mm 복원. 240Hz CPU 실제 상태/접촉 저장.
- 원래 농장은 초기 잎 겹침 최대12.22mm로 중단. 분리 시험은 배경 식물↔활성 물체 및 활성 잎↔식물 내부 접촉을 명시적으로 제외했으며 full 농장 성공이 아님.
- 접근3회: 지지선/유인선 차단 뒤 접고 회전하는 경로로 개선했으나14.73초에 잎 접촉·관통0.507mm로 중단. 잎 밀림으로 과실62.85mm 이동. 실제 수확0, `harvest_success=null`, 학습 제외.
- 과실 약26mm 대 CAD 최소 간격28mm 미해결. 그리퍼/과실 크기는 임의 수정 안 함. RB5 원본 자기충돌false도 유지되어 자기충돌 경로 검증 추가 필요.
- 결과/실제 상태 MP4: `/root/docker_share/mujoko_debugging_data/20261010_s300_harvest_motion/index.html`. 자산10검사/기구학4검사 통과.
- 상세·명령·실패 포함 기록: [MOTION_STATUS_KO.md](nvidia-sim/harvest_s300/MOTION_STATUS_KO.md). 이번 변경의 커밋 여부는 `git status` 확인.

## 최신 정정: 하우스 0.5 크기 복원 (2026-10-10)

- 기존 `farmily_greenhouse_robot.usd`의 하우스 배율0.5 확인. S300 생성기 기본값1.0은 잘못 가져온 크기였으며0.5로 수정.
- 줄기68개/송이106개 뿌리·부착 세계 좌표도 검토된 배치로 복원(행 내25cm). 식물 template 자체 크기는 유지.
- S300/리프트/RB5/그리퍼는 축소하지 않고 x=0.55→0m로 통로 중앙 배치. 원본 메시/조립 관계 보존.
- 기하27검사+크기/보존8검사 통과. 현재 Isaac 창 적용. 결과 `/root/docker_share/mujoko_debugging_data/20261010_s300_restored_house_scale/index.html`, 최종 `assembly_final/scene.usda`.
- 이번 변경은 크기/배치 정정. 새 접촉·탄성·수확 물리 검증은 미실시. 아래1.0 배치 기록은 이전 상태다.

## 최신: Isaac 농장의 줄기·잎·송이 자산 정정 (2026-10-10)

- S300 조립에서 옛 v2 농장 식물을 사용했던 것을 수정. `build_scene.py` 기본 `--farm-layout reviewed`, 옛 장면은 `legacy`로 재현.
- 원본 v8 줄기 36개 + v9 잎 줄기 32개, GLB 5종 송이 106개. 기존 Y축 0~180°/송이0.5배/Fruit06 제외/white 보정 유지. 원본 USD/GLB 면·정점은 줄이지 않음.
- 하우스 1.0 유지, 기존 0.5 하우스 배치의 **뿌리 좌표만 2배**로 맞춤. 식물 자체 크기는0.5 유지. 행 내 명목 간격25→50cm이며 이전 MuJoCo 장면과 세계 좌표/밀도가 동일하다는 뜻은 아님.
- S300 뒷판·휠 커버 수정 및 로봇 조립 보존. 기하27검사/로봇보존5검사 통과, 실제 Isaac6.1 전후 동일 카메라·근접·양측배치 촬영.
- 최종 화면/결과: `/root/docker_share/mujoko_debugging_data/20261010_s300_latest_plants/index.html`, 장면 `assembly_final/scene.usda`. 현재 Isaac 창에도 반영.
- **고정 full triangle 충돌체 상태이며 탄성 연결/수확 물리/새 속도 시험은 미실시.** `training_eligible=false` 유지. 상세 [harvest_s300/README.md](nvidia-sim/harvest_s300/README.md).

## S300 수확로봇 · Isaac Sim 6.1 조립 (2026-10-09)

- 새 브랜치 `codex/s300-harvest-isaac-20261009`, 시작점 V5.1 그리퍼 수정 `b890fc6`. 이전 작업과 원본 자산 보존.
- Isaac Sim 6.1 standalone을 `/root/isaac-sim-6.1.0`에 별도 설치, 공식 MD5 일치. 기존 5.1 환경 유지.
- 원본 54식물 농장 USD의 형상·잎·재질 보존, 원본 공간 배율 1.0. MuJoCo light/ultralight 미사용.
- S300 V10 실제 외형에서 물통·펌프·분무부/후방 상부 지지물 제거, 평판 위에 기존 리프트·RB5, V5.1 그리퍼 356부품 부착.
- 13개 원본/기하/좌표 검사 통과, 실제 6.1 렌더 3시점 저장. GPU PhysX 120Hz 정지 2초/240스텝에 1.662초, 팔 끝 최대 이동 0.120mm. 초기화 별도, 수확/접촉 성능 시험 아님.
- **현재 농장 식물은 고정 triangle 장애물이며 탄성 모델 연결 전. 그리퍼도 44mm 개방 고정 상태.** 기존 `elastic_plant.py` full+keep 연결, 새 도구 하중/구동/체결 검증은 남아 있다. 학습 데이터 사용 금지.
- 사용법/검증/한계: [nvidia-sim/harvest_s300/README.md](nvidia-sim/harvest_s300/README.md).
- 결과: `/root/docker_share/mujoko_debugging_data/20261009_s300_harvest_isaac61/index.html`, 최종 USD `assembly_v3/scene.usda`.
- VNC 조작 지연 조사: 내부 정지 약161 FPS/회전 표본 약117 FPS로 렌더 정지는 재현되지 않음. 정지 GUI 기본30 FPS 제한·FPS 표시 추가, Play 시 원래 갱신 설정 복귀. 짧은 GUI 표본 CPU528→124%, 화질/형상/충돌/물리 smoke 유지. 원격 체감 개선은 사용자 확인 필요. 원자료 `20261009_isaac_viewport_diagnosis/`, 실행 옵션 `--viewer-fps 0`은 원래 무제한.
- 외형 정정: 리프트 뒤 원본 뒷판 1개 복원, 메카넘 휠 커버 및 전용 체결부 96개 제거(바퀴 유지). 새 장면 `20261009_s300_rear_panel_no_wheel_covers/assembly/scene.usda`, 전후 사진 `index.html`. 기하15검사·변경범위4검사 통과, 기존 자산/결과 보존. 현재 Isaac 창에도 반영.

## 힘·변위 중단값 비교 (2026-09-28 추가)

- 사용자 요청으로 제한값을 변경한 추가12회. 기준b75e6bc, 별도 `codex/contact-limit-sweep-20260928`. 모델/경로/물성/운영정책 유지.
- 00번 원래 속도·감속에서2N/5mm,1N/10mm,2N/10mm 비교, 기존5경로+감속00에서5N/20mm 비교.
- 전체 검사12/12 통과. 실제 전체 동작 완료3, 목표 힘 중단1, 변위 중단5, Rachis 비허용 접촉 중단3. 꼭지 접촉 기반 안착·유지 확인0.
- 5N/20mm에서00·04·감속00 완료. 실제 최대 과실 힘1.577/1.585/0.899N, 최대 변위10.951/10.628/10.673mm. 기존1N/5mm는 완주 관찰을 막았으나 실물 안전 기준은 미검증.
- 04번 hold1.429초 기하 안착 범위 유지(간격약1.4mm), 목표 꼭지 접촉력0. verify는 일부 기하 범위 이탈. 다음 검토 출발점이며 hook_success로 승격하지 않음.
- 원본해시/timestamp 연속성/종전 기준과 공통 저장 상태 qpos 차이0 확인. 목표 과실 침투0, 전체 최대0.228mm. training_eligible=false, hook_success=null.
- [결과 보고서](mujoco-benchmark/CONTACT_LIMIT_SWEEP_RESULTS_KO.md), [GPT-6 Pro 전달문](mujoco-benchmark/CONTACT_LIMIT_SWEEP_PRO_HANDOFF_KO.md). 결과 `20260928_contact_limit_sweep/index.html`, 실제 상태 영상4개와240Hz 원자료 보존.

## 목표 과실 접촉 허용 재시험 (2026-09-28)

- 기준 `804e40c`, 별도 브랜치 `codex/server-target-fruit-contact-20260928`. 마지막 under5개 원본 경로를 재계획 없이 검사했고 모두 전체 완료·통과.
- 정확한 목표 과실/고리 와이어만 허용. 실제 기존5회는 과실 힘 합1N 초과, 동일 경로 seat18배 감속1회는 변위5.015mm로 중단. 총6회/예산7회, 정상 접근 후 꼭지 안착·유지 미확보.
- 감속 목표 과실 힘 최대0.604N, 실제 TCP 최대2.397mm/s(명목1.942mm/s). 한계 자동 상향 없음. 손상/수확 불가능 판정 아님.
- 기록된 비허용 로봇·환경 접촉력0, 목표 꼭지 접촉력0, 목표 과실 침투0. 전체 활성 접촉 초기 최대침투0.228mm로 기존0.5mm 기준 이내. 중단 이후는 미검증.
- 과거 baseline3.397N에는 목표 과실 접촉도 포함된다. 모두 다른 토마토/줄기의 힘으로 해석하지 말 것. 과거 단일 법선력 최대와 이번 접촉별 힘 크기 합도 구분.
- 테스트188+5 통과, 실제 상태 영상3개/240Hz 증거/원본 해시 보존. 학습 제외, hook_success=null. [상세 보고서](mujoco-benchmark/TARGET_FRUIT_CONTACT_SERVER_RESULTS_KO.md).
- 결과 `20260928_target_fruit_contact_server/index.html`. 충돌체·물성·원본 경로·기존 운영 정책 유지.

## 꼭지 안착 목표·전체 경로 연결 실험 (2026-09-27 후속)

- `f24709c` 기반 별도 `codex/server-hook-seating-20260927`. 실제 gutter02/Tomato02에서 기하 목표32개 생성, 선택12개 IK·새 기하 통과, 전체 도구/환경의 끝 자세5개 통과.
- opt-in `diagnostic_pose_waypoints_v1`: 경유점별 RING 위치·회전을 실제 IK에 반영. 기존 명령2,512개/phase 동일, action14 사용 안 함, 학습 제외.
- 직접 준비12개/우회10개/하부 삽입5개 모두 경로 검사 차단. 마지막에는 안착 상승 중 뒤쪽 와이어가 목표 과실과 겹침. 새 물리0회, 비교 baseline1회만 실행. 정상 진입·안착·유지 동작 미확보.
- baseline 부분 중심 진입이지만 꼭지 안착0; private 재계산 전체 비허용 힘 최대3.397N. 접촉/기하240Hz와 시각 구분 저장. 물리·충돌·성공 기준 유지.
- 자동116+5개 통과, 독립 기하/시간 대조군. 정적 fixture는 자동 진입 성공이 아님. [서버 결과](mujoco-benchmark/HOOK_SEATING_SERVER_RESULTS_KO.md), 출력 `20260927_hook_seating_server/index.html`.

## 환경 사전검사 재검토·접촉력 진단 (2026-09-27)

- 작업 브랜치 `codex/server-preflight-rereview-20260927`, 기준 `bd59ea3`(필수 `1f5bc1f` 포함). 7경로 전체 스캔 완료, 모두 blocked. 상세 저장 상한64개와 클래스별 전체 집계를 구분.
- gutter02는 양의 간격에서도 실제 법선력 최대0.734N. stem01은 고정 장애물에 막혀 회전 관절 추종 오차0.293rad. 무겹침을 무접촉으로 해석하지 않는다.
- 별도 MjData에서 동기화한 힘/기하와 live 직전 solver 힘을 구분. 진단 유무에 따른 전체 qpos/qvel 해시 동일. 물리·성공 판정·운영 정책 유지.
- Tomato02 5회 국소 변경: 기존 부분 중심 진입5/5, 목표 꼭지 안착/유지 지표0/5. 거터 장면의 별도 오프라인 조사이며 preflight를 통과한 수확 결과가 아니다. 학습 제외.
- 자동65테스트, 실행 gate3조건 및 대표4영상. 상세/Pro 전달: [재검토 서버 결과](mujoco-benchmark/ENVIRONMENT_PREFLIGHT_SERVER_REREVIEW_RESULTS_KO.md). 출력 `20260927_preflight_rereview_server/index.html`.

## 환경 사전검사 서버 검증 (2026-09-27)

- 검토 브랜치 38cdb6a 반영, 자동46테스트 통과. 기존7경로 모두 blocked. 거터/주줄기 사례 검출, 실행 보류/판단 유보에서 물리0회 확인. 첫1초 통과 실행은 전체 경로 안전 검증 아님.
- 기존 성공 경로의 계획 겹침과 실제 간격 차이 확인: gutter02 검출 쌍은 새 물리10,044스텝 최소0.866mm. 접촉 기록은 존재하므로 무접촉 증명 아님. 정책/경로 수정 안 함.
- 상세: [1차 서버 검증 보존본](mujoco-benchmark/ENVIRONMENT_PREFLIGHT_SERVER_FIRST_RESULTS_KO.md). 출력 `20260927_environment_preflight_server_review/index.html`, 공유 폴더 `ENVIRONMENT_PREFLIGHT_PRO_FEEDBACK.zip`.

GPT-6 Pro 논의용 현황: [비목표 무접촉 수확 논의](mujoco-benchmark/NONCONTACT_HARVEST_DISCUSSION.md). 공유 폴더에도 동일 파일 저장.


## 주변 식물 탄성화 비용 진단

- `benchmark_neighbor_elastic.py`: 기존 생성기/RobotEngine 재사용, 주변1/4개 고정·탄성 비교. 운영 장면은 변경하지 않음.
- 식물당 송이1개, 240Hz, 동일 명령10초 각2회: 주변1개 3.227→5.029초, 4개 3.409→16.708초. 접근 실행은 관통 기준 실패이므로 정상 수확 성능 아님.
- 정지2초는 모두 관통 기준 통과. 주변4개 0.929→4.006초로 탄성 비용 확인. 전체68개/실시간 타깃 전환 미측정. 타깃만 탄성·나머지 고정 충돌체 방식이 유리할 근거이나 비접촉 보장은 별도.
- 현재3송이 조밀 배치는 주변4개 탄성화 시 초기7.645mm 겹침 발견. 모델 전환 전에 배치 충돌 검사 필요.
- 결과 `20260927_neighbor_elastic_one_truss/index.html`, 정지대조 `20260927_neighbor_elastic_idle/index.html`. 상세 GREENHOUSE_VISUAL.md 마지막 절.


## 로봇 쪽 타깃·주변 주줄기 장애물

- `preview_neighbor_plants.py --stem-obstacles-layout`로 양측 주변68줄기를 고정 캡슐1088개로 충돌 처리. 기존 수확 줄기 탄성 유지.
- 통로 쪽 뿌리 위치 기준 타깃 적격 구분. 실제 실행 대상은 `eligible_targets`의 기존10열매. 주변 복제 송이는 아직 물리 타깃 아님.
- `20260927_stem_obstacle_scene/index.html`: 초기 검사 통과, 저장01/02/09 경로 모두 주줄기 침투1.18/1.49/1.62mm로 invalid_physics. 자동 회피·비관통 해결 완료가 아님.
- 관련9테스트 통과. 주줄기 접촉/침투 metrics 및0.5mm invalid 판정 연결.

## 거터 충돌 추가

- `--gutter-collisions`(새 장면/직접 테스트), `greenhouse_visual.py --gutter-collision-only`(기존 XML) 추가. 원본 USD의 두 거터 판/리브18개를 별도 고정 충돌체로 구성해 열린 내부 유지.
- 거터–팔/고리/식물 관통0.5mm 초과는 invalid_physics. 초기 검사 및 rollout metrics에 거터 포함. FCL 자동 회피는 미연결.
- `20260927_gutter_collision_scene/index.html`: 제어된 옆판/바닥 접촉0.250mm, 기존 경로3개 재실행 완료. 09에서 실제 거터 접촉 발생. 자유낙하 고속 시험은3.328mm로 실패 기록 보존.
- 주변 식물·철사·거터 다리·배지는 여전히 시각 전용. 상세는 GREENHOUSE_VISUAL.md.

## 복잡한 하우스 배경 직접 각도 테스트 완료

- `test_direct_angle.py --existing-scene DIR --physics-hz 240 --gui` 추가. 현재 배경을 그대로 새 RGB-D 촬영→각도 예측→독립 물리 실행에 사용.
- `20260927_dense_house_harvest_test`: 10열매 중 중심진입6/미진입2/가림 미시도2(03,11), 실행8개 물리오류0. GUI 포함620.16초.
- 예측은 모두 -90° 부근에 몰림. 주변106송이는 충돌 없는 외형이므로 장애물 회피/수확 분리 검증 아님. 보고서 미시도/실제 계획 집계 수정.
- 재실행 명령과 범위는 `mujoco-benchmark/GREENHOUSE_VISUAL.md` 마지막 절.

## 주변 식물 랜덤 송이 미리보기

- 양측 배치 커밋 `0317ce6`. 후속 `--random-trusses`로 주변 줄기당 송이 1개를 기존 GLB 규칙으로 랜덤 부착.
- 5종/0.5배/Y회전0~180°/segment4~13/6번 제외/white보정 유지. 기존 부착 계산 공유. 주변은 정적 외형 전용.
- 결과 `/root/docker_share/mujoko_debugging_data/20260927_mixed_house_random_trusses/index.html`.

## 거터 양측 줄기 혼합 배치

- `preview_neighbor_plants.py --paired-gutters --alternate-model` 추가. 거터 양쪽에서 줄기가 서로 반대 방향(0°/180°)으로 휘어 올라가도록 배치.
- seed27, 간격25cm, 주변68개: 기존/v9 각34개 랜덤 선택. 주변 식물은 시각 전용.
- 결과 `/root/docker_share/mujoko_debugging_data/20260927_paired_mixed_stem_house/index.html`. side 화면만 외벽/다른 거터 식물을 숨겨 배치 확인.

## 2026-09-27 새 줄기 leaves_v9 외형 추가

- `tomato_stem_leaves_v9.glb`를 `--stem-glb`로 랜덤 장면 생성/직접 각도 테스트에서 선택 가능. 기본 줄기 유지.
- 기존 16분절 주줄기 골격과 일치 확인. 신규 하부 잎을 포함한 모든 primitive 보존. 새 잎은 시각 전용이며 충돌/질량/개별 탄성 추가 없음.
- 비교: `/root/docker_share/mujoko_debugging_data/20260927_stem_leaves_v9/index.html`. 기존/신규 240스텝 qpos 동일.
- 상세: [STEM_GLB.md](mujoco-benchmark/STEM_GLB.md).

## 2026-09-27 충돌 없는 하우스 표시

- 기존 GUI/75도 상승 변경 커밋 `62e66be`. 물리 기본 Hz 변경 없음.
- `greenhouse_visual.py`, 생성기/직접예측 `--house`: 기존 Isaac 하우스1,078메시를 visual-only로 추가. 기존 로봇/식물 충돌 유지.
- 동일25초/240Hz 명령에서 배경 전후 qpos 완전 일치. 상세: [GREENHOUSE_VISUAL.md](mujoco-benchmark/GREENHOUSE_VISUAL.md).
- 결과 `/root/docker_share/mujoko_debugging_data/20260927_greenhouse_visual/index.html`. 이 하우스 추가 작업의 커밋 여부는 git status로 확인.

## 최신 수정: 송이 50% 축소와 고리–주줄기 관통 판정

상세 변경·재현 명령·검증 한계: [HALF_SCALE_CONTACT_FIX.md](mujoco-benchmark/HALF_SCALE_CONTACT_FIX.md).

- 새 GLB 생성/수집 CLI는 `--truss-scale 0.5` 기본. 주줄기·잎·로봇은 그대로, 송이 전체 가지·꼭지·열매의 길이/반경 50%, 질량 1/8. 강성·감쇠·armature 유지로 동역학 동등성은 주장하지 않음. 원래 크기는 `--truss-scale 1`.
- 기존 크기 기록의 resume에는 `--truss-scale 1` 명시. 다른 크기를 같은 수집에 섞지 않도록 검사.
- `20260923_random_glb_full_smoke/scene_0001/targets/Tomato_05` 후보00005의 기존 성공은 잘못된 유효성 판정. 고리–기존 주줄기 관통3.860mm를 GLB 전용 검사에서 누락. 새 실행은 invalid_physics로 처리하고 학습에서 제외. 기하 중심 진입 조건은 변경하지 않음.
- 구버전 GLB 자료도 학습 로더에서 저장된 고리 관통 지표로 제외. 이전 학습 smoke 가중치/성능은 이 오류의 영향을 받으므로 유효 결과로 사용하지 말 것.
- 근거/독립 재실행: `/root/docker_share/mujoko_debugging_data/20260923_center_entry_audit/`. 기존 수치/상태는 보존하고 해당 기존 HTML에 정정 안내만 추가.
- 절반 크기3장면·기존 형상 비교·2후보 물리: `/root/docker_share/mujoko_debugging_data/20260923_half_scale_glb/comparison.html`. 정지3장면 통과, 로봇2후보 miss. 수확 성공 검증이 아님.

## 최신: 랜덤 GLB 장면 → 전체 열매 10후보·9 RGB-D → 장면 분할 학습 → 새 장면 물리 검증

2026-09-23, 브랜치 `mjlab-performance`. 기존 장면 생성기·후보 수집기·D435 촬영·camera action v2·학습기를 연결했다. **이번 작업의 커밋 여부는 `git status`로 확인한다.** 랜덤화는 GLB 송이 종류, 원래 주줄기·잎에서의 송이 부착 위치, GLB Y각도이며 GLB 안 개별 열매의 배열은 바꾸지 않는다. 9-view 출력은 원본 RGB·Depth를 저장하고 별도 열매 mask 및 Depth 유효성 PNG는 저장하지 않는다.

작은 전체 연결 시험: seed5의 3장면×송이1개×열매10개×후보10개 = 300회 시도. 계획 실패110, 물리 미진입160, 중심 진입18, 물리 오류12. RGB·Depth 각270장. 장면0/1/2를 train/validation/test로 분리했고 3모델×3seed 학습에서 검증 BCE로 ResNet18 RGB-D seed2를 선택했다. 테스트 장면은 발견된 진입이0개이며 계획 가능한 후보가 없는 열매3개를 포함한다. 실패 열매가 실제로 불가능할 수 있지만, 10후보 미발견을 그 증명으로 해석하지 않는다.

별도 seed24 새 장면에서 열매10개의 후보를 결과 없이 계획해 모델 추천을 확정하고, 추천10개를 새 MuJoCo 물리로 독립 실행했다. 중심 진입1(Tomato_10), 미진입9, 물리 오류0. 저장 상태10개 재생 해시 확인. 꼭지 걸림 수확 및 RGB-D 우위·일반화는 미입증. 수집 `/root/docker_share/mujoko_debugging_data/20260923_random_glb_full_smoke/index.html`, 학습 `/root/docker_share/mujoko_debugging_data/20260923_random_glb_training_smoke/index.html`, 새 장면 물리 `/root/docker_share/mujoko_debugging_data/20260923_random_glb_fresh_physics_smoke/index.html`. 명령·한계: [RANDOM_GLB_SCENES.md](mujoco-benchmark/RANDOM_GLB_SCENES.md), 보존 수치: `mujoco-benchmark/validation/random_glb_collection_pipeline.json`.

## 최신: 주줄기 송이 위치·Y각도 랜덤 물리 장면 생성

`mujoco-benchmark/scripts/generate_random_glb_scenes.py`는 seed로 GLB 종류·주줄기 segment 안 위치·Y축 각도를 재현 가능하게 뽑고, 기존 주줄기·잎을 유지한 여러 송이 물리 장면을 생성한다. 각 장면 초기 2초 겹침·정지 이동 검사를 저장하며 실패 장면도 표시한다. 2장면×2송이 샘플: `/root/docker_share/mujoko_debugging_data/20260923_random_glb_physics_final/index.html`. 실행 명령과 범위: [RANDOM_GLB_SCENES.md](mujoco-benchmark/RANDOM_GLB_SCENES.md).

2장면 모두 초기 검사 통과. 두 번째 송이를 대상으로 로봇 1경로 실제 실행: 최대 겹침0.347mm, 중심 미진입, 열매 이동52.51mm. 다중 송이의 이름 공간도 `RobotEngine`의 접촉 검사에 포함하도록 수정했다. 한 경로의 정상 실행을 전체 랜덤 장면의 수집 승인으로 해석하지 않는다.

## 최신: 전체 GLB 로봇 수집 소량 검사 완료

2026-09-23, 최신5종/각1개 전체 로봇 경로, Tomato_05, Y90°/segment11, 240Hz. white는2번 위치·강성 수정 모델, 첫seed0 준비자세IK실패 후seed1실행. 총 계획6회/실제물리5회. 결과: green/red/white miss(물리 기준통과), rotated90 partial_center_entry(물리 기준통과), cyan invalid_physics(최대겹침0.514mm로0.5mm기준초과, 학습제외). 경고/수치불안정0.

최대목표밀림 cyan219.00/green156.96/red81.02/rotated90 83.86/white90.58mm. 현재center_entry_only_v2는밀림을실패로분류하지않으므로 유효4개 또는 rotated90부분진입을 안전수확 성공으로 해석하지 않는다. 각1경로라 전체각도/부착위치 성공률이나대량수집승인아님. cyan 접촉중겹침은미해결로보존.

5개 초기RGB-D/목표mask640×480 저장, 각가상카메라로action_camera재변환 및복원검사통과. 후보states/trace해시·수치유한성·관측모델해시일치검사통과, 회귀12테스트통과. 관측의실제관절도달성은미검증.

결과 `/root/docker_share/mujoko_debugging_data/20260923_024914_all_glb_robot_smoke/index.html`. 보존요약 `mujoco-benchmark/validation/all_glb_robot_smoke.json`.


## 최신: white 2번 간격 및 탄성 안정성 수정

2번을 중심가지에서 멀어지는 방향으로3mm 이동(원본GLB 좌표 offset은 build.json), 꽃받침/말단 꼭지를 함께 이동하고 proximal 시작점은 유지해 변형했다. 원본GLB 보존, 나머지 body 위치와 모든 질량 유지 검사 통과. 6번 제거 및 연결부 필터 유지. 초기0.5mm 초과 겹침0개.

간격 수정만으로 힘 시험 미복원이 해결되지 않았다. 고정 중력 preload를 쓰는 초기 평형의 (중력 토크 미분+스프링) 대칭 행렬 최소 고유값이 -0.409로 국소 불안정 방향이 확인됐다. white 전용 `--rachis-stiffness-scale 2` 적용: Rachis k2→4, d0.15→0.2121. 최소 고유값+0.203. 물성 조정이며 실측 보정/기존과 동역학 동등성 아님. 다른GLB/기본값은1 유지.

240Hz 0.2N(2.5~3초) 이후12초까지 검사: 5번 X/Y/Z 최대51.89/19.43/26.00mm→잔류2.47/0.287/0.198mm. 2번 X 최대29.89mm→잔류1.47mm. 네 시험 모두 접촉 관통0, 경고0; 별도 구형 물체 접촉 반응26.24mm. 6초 시점 X잔류18.65mm로 빠른복원을 주장하지 않는다. 초기 정지2초 이동 수치오차 수준. 회귀12개 통과. 로봇 수집은 이 수정 모델로 미실행, 모든각도/부착위치 안정성은 미검증.

최신 white: `/root/docker_share/mujoko_debugging_data/20260923_white_fruit02_stable/index.html`. 이전모음의 white 실패기록은 보존한다. 요약 `mujoco-benchmark/validation/white_fruit02_fix.json`.


## 최신: 전체5 GLB 6번 제거 완료

cyan/green/red/white/rotated90 모두 `--remove-fruit 6` 적용, 열매10개 유지. 기존 주줄기·잎 및 Y90° 배치/segment11 유지. 각 모델 240Hz 정지·0.2N 힘·접촉·복원·reset 검사 완료, 회귀12테스트 통과. 초기0.5mm 초과 겹침은 cyan/green/red/rotated90에서0개. white는 Fruit_02와 Rachis_04 충돌체 약1.001mm 겹침이 별도로 남아 사용 보류. 또한 힘 시험 무부하 대비 최대/종료 변위가 모두626.09mm로 복원 검증 실패했으며, 이 큰 이동의 원인을 해당 초기 겹침으로 단정하지 않는다. 다른 알맹이를 임의 삭제하거나 해당 충돌을 숨기지 않았다. 초기2초 최대 바디 이동 cyan2.20/green2.30/red2.35/white3.18/rotated90약0mm. 로봇 후보 수집은 제거 모델들로 재실행하지 않아 전체 수집 물리 유효성은 미검증.

모음 `/root/docker_share/mujoko_debugging_data/20260923_all_glb_without_fruit06/index.html`. 요약 `mujoco-benchmark/validation/all_glb_without_fruit06.json`. 개별 하위폴더의 model.mjb/reference.json 사용. 원본GLB 보존.


## 최신: cyan 6번 알맹이 제거

`build_glb_physics.py --remove-fruit 6`로 Fruit_06 body/충돌/질량(60.45g)/꽃받침 시각/타깃을 제거한다. 꼭지 가지는 남기고 원본 GLB는 보존한다. 나머지 열매 번호 유지, 총10개. 기존9·10번 연결부 필터 적용. cyan Y90°, segment11에서 초기0.5mm 초과 겹침0개, 정지2초 최대 이동2.20mm. 0.2N 힘 시험 최대25.19mm, 잔류4.85mm. 접촉/복원/reset 및4테스트 통과. 로봇 후보 수집은 이 제거 모델로 재실행하지 않았으므로 수집 성공률/전체 물리 유효성 미검증.

결과 `/root/docker_share/mujoko_debugging_data/20260923_glb_without_fruit06/index.html`.


## 최신: 9·10번 정상 연결부 충돌 수정

GLB 첫 꼭지 가지와 바로 옆 중심가지 분절의 접합부 충돌 두 쌍 수정. cyan 정지2초 최대 이동51.69→8.66mm, 힘·접촉·복원 및4테스트 통과. 6번 열매 겹침4.93mm는 남아 학습 사용 미승인. 상세 [GLB_PHYSICS.md](mujoco-benchmark/GLB_PHYSICS.md).

## 최신: GLB 탄성·충돌 이식 및 로봇 수집 시험

기존 주줄기·잎과 GLB Y축 배치를 유지하여 241자유도 물리 모델로 이식했다. 힘/접촉에 의한 휘어짐·복원 및 후보3개 실제 실행 완료. 초기 내부 겹침4.93mm로 모두 invalid_physics이며 학습 사용 금지. 기존과 동일한 동역학이나 수확 성공을 의미하지 않는다. [구현·검증·실행 명령](mujoco-benchmark/GLB_PHYSICS.md).

최종 갱신: 2026-09-23 KST (실험 폴더 이름은 기존 기록의 표기를 그대로 유지함).
이 파일은 대화 기록 없이 다른 계정/새 세션에서 작업을 이어가기 위한 시작점이다.
아래 상태는 작성 시점 기준이므로, 재개할 때 `git status`를 먼저 확인한다.

## 최신: 단일 GLB 관측 수집 파일럿 — 로봇 실행 전 제외

- `collect_glb_pilot.py`: cyan송이1개/Y90°/기존 주줄기/목표Fruit_05 장면 생성, RGB·metric depth·유효깊이 마스크·목표 마스크·카메라 행렬 저장 및 초기 convex 겹침 검사 수행.
- 최종 출력 `/root/docker_share/mujoko_debugging_data/20260923_020731_glb_single_collection_pilot/index.html`. RGB640×480, 목표1765픽셀, 유효깊이42121픽셀. 배열/마스크/카메라 직교성 및 라벨 제외 검사 통과.
- 수집 전 검사에서 새 송이 충돌 꺼짐/탄성 없음/로봇 없음/장면별 계획·목표 참조 미연결을 검출. 상태rejected_before_rollout, 로봇 실행0회/라벨0개/training_eligible=false. 가상 카메라 관측이며 기존 D435/action14 학습 데이터로 바로 사용할 수 없다.
- 실제 경로 수집을 하려면 GLB 물리 이식과 장면별 로봇·계획·평가 참조 연결이 선행되어야 한다. 기존 데이터셋은 수정하지 않았다.

## 최신: GLB 부착 커밋 및 초기 겹침 검사

- 사용자 확인한 GLB5종/Y축0~180°/무작위 위치 부착은 `67483c7`로 커밋했다. GLB 원본5개 포함. push는 하지 않았다.
- `audit_glb_collisions.py`: 접촉 마스크를 우회한 명시적 convex 형상 거리 검사. 다중송이 장면1072형상에서 AABB 선별 후4592쌍 검사, 약1.49초. 임계값0.5mm.
- 서로 다른 송이 겹침0쌍. 털 제외 시 송이–주줄기7쌍/최대3.042mm, 송이–잎·잘린가지1쌍/최대3.201mm. 내부1292쌍은 정상 접합/꽃받침/convex 과대근사 포함이므로 실패 수로 해석하지 않는다.
- 원본 메시의 정확한 삼각형 관통 또는 동적 물리 검증은 아니다. 털 포함/제외 수치를 분리하며 원본/접촉 마스크는 변경하지 않았다. 새 송이 탄성·충돌은 여전히 미적용.
- 결과 `/root/docker_share/mujoko_debugging_data/20260923_multi_glb_collision_audit_classified/index.html`, 보존 요약 `mujoco-benchmark/validation/glb_initial_overlap.json`. 비활성 접촉 마스크 상태의 겹침/접함/분리 거리3조건 검사 통과.

## 최신: GLB 5종을 동일 주줄기에 무작위 부착

- 사용자 확정 규칙: GLB Y축0~180°만 회전. Y-up→Z-up 변환 C=Rx90 후 C·Ry(theta), 추가 X/Z 회전 없음. 기존 주줄기·잎 보존, GLB 원본 크기1 유지.
- `assemble_glb_plant.py --output NEW_DIR --seed 23 --view`: 5파일을 각각1번 사용, 주줄기 호 길이 구간별 무작위 위치에 동시 부착. `--axes` 선택 가능. seed 재현 가능, 출처해시/각도/부착점은 scene.json 저장.
- 결과 `/root/docker_share/mujoko_debugging_data/20260923_multi_glb_y_random/index.html`. red20.5°, cyan153.6°, green39.2°, white84.7°, rotated90 62.8°. 컴파일 후5송이 변환/Y축 위쪽 및 기존 주줄기302형상 위치·몸체 질량/관성 보존 검사 통과.
- 정지 시각 미리보기다. 송이 간·잎 간 겹침 필터와 송이 충돌/탄성은 미적용이며 학습용 물리 장면이 아니다.

## 최신 수정: GLB Y-up → MuJoCo Z-up 좌표계 변환

- `attach_glb_preview.py`는 좌표계 변환 C=Rx(+90°) 뒤 원본 GLB Y회전 적용: R=C·Ry(theta). GLB +X→월드 +X, +Y→월드 +Z, +Z→월드 -Y(추가회전0도). 이 좌표계 변환은 송이의 추가 X회전 조작과 구분한다.
- 기존 축 변환 없는 부착 샘플은 과거 결과다. 새 cyan0도 부착: `/root/docker_share/mujoko_debugging_data/20260923_glb_y_up_attached/axes.png`. GUI 열림, 주줄기·잎 보존. -90/-30/0/30/90도 모두 GLB Y가 월드 위쪽인지 검사.
- 새 송이는 시각 전용이며 탄성/충돌 연결 미완료. `view_glb_truss.py`의 이전 단독 뷰어는 아직 숫자 좌표 그대로 표시하므로 위쪽 축 수정은 부착 뷰어에 적용된 상태다.

## 현재 사용자 확정: 새 GLB, Y축 회전만

- 원본 송이5개: `nvidia-sim/env_usd/tomato_rotate_glb/*.glb`. 사용자 조건은 **GLB의 Y축만 회전하고 X/Z축 회전을 추가하지 않는 것**. 아래 과거 방향 추정/반구 제한을 새 기준으로 적용하지 않는다.
- `mujoco-benchmark/scripts/view_glb_truss.py FILE.glb`: 원본 노드 변환·스케일·좌표/원점 유지한 송이 단독 미리보기. `[`, `]`로 Y축±5도, `0`으로 원본. 마우스는 카메라만 회전. 원점 기준 공통 GLB XYZ 표시.
- 5파일 각154메시 읽기 및 Y회전시 Y좌표/원점거리 보존 검사 통과. cyan0도 화면 실행 확인. 시각 확인 단계이며 새 GLB의 탄성/충돌/주줄기 연결은 아직 구현하지 않았다. 원본 GLB는 수정하지 않았다.

## 현재 기준: 사용자 지정 단계로 롤백

사용자 요청으로 `20260923_tipward_surface_attachment` 단계의 생성 동작으로 복원했다. 짧은 시작 가지 기준 회전과 당시 표면 부착 계산을 사용한다. 이후 캡슐 축 순서 수정, 엄격한 접합 거부/재샘플링, 긴 중심가지/열매 중심 각도 변경은 현재 코드에서 되돌렸다. 아래 후속 변경 기록은 과거 이력이며 현재 동작이 아니다. 기존 주줄기·잎과 모든 결과 폴더는 보존한다. 이 단계의 알려진 초기 겹침도 복원되므로 수집 검증 완료로 해석하지 않는다.

기준 결과: `/root/docker_share/mujoko_debugging_data/20260923_tipward_surface_attachment/index.html`.

## 최신 수정: 부착점→열매 평균 중심 기준(v5)

- 사용자 최종 정정: 주줄기 표면 부착점에서 각 열매 중심 좌표의 평균을 향하는 방향으로 각도를 계산한다. 국소 주줄기 위쪽0°, 0~90도 허용. 아래 v3/v4는 과거 기준이다.
- 새9개 샘플: `/root/docker_share/mujoko_debugging_data/20260923_fruit_center_angle_samples/index.html`. 컴파일된 열매 body 좌표에서 각도 독립 재계산 통과. 부착 불량2개 제외 기록 보존.
- `--fruit-center-angle` 사용. 기존 주줄기·잎 보존, 접합/지지 가지 비관통 검사 유지. 전체 장면 물리/학습 수집 준비 완료는 아니다.

## 최신 수정: 긴 중심가지 기준 샘플

- 각도 기준을 짧은 시작 가지에서 **송이 Rachis 중심선 시작→끝 방향**으로 변경(v4). 국소 주줄기 위쪽0°, 0~90도 허용. 기존32도 등 v3 수치와 다른 정의다.
- `assemble_plant_scene.py --rachis-angle 60`으로 명시 가능. 생략시15~85도 seed 샘플링. 기존 주줄기·잎/송이 내부 형상 유지, 표면 접합·주줄기 관통 검사는 별도 유지.
- 새9개 샘플: `/root/docker_share/mujoko_debugging_data/20260923_rachis_angle_samples_checked/index.html`. 전체 장면 물리/로봇 수집 검증은 미완료.

## 최신 수정: 송이 허용 방향과 표면 접합

- 사용자 확정: 주줄기 위쪽 끝 방향0°, ±90° 허용. 국소 주줄기 끝 방향과 시작 가지의 각도가90° 이내인지 검사하며 뿌리 방향 배치는 거부한다.
- 기존 주줄기·잎 보존. 원본 캡슐 fromto 순서로 축 방향을 복구하고, 송이 시작 캡슐 끝을 주줄기 표면에 접합한다. 접합 간격/지지 가지 전체의 주줄기 관통을 컴파일된 형상으로 검사하고 실패하면 생성 거부.
- 새 미리보기 `/root/docker_share/mujoko_debugging_data/20260923_tipward_attachment_final/index.html`. 전체 장면 충돌/로봇 수집 준비 완료를 의미하지 않는다.

## 최신 수정: 기존 주줄기와 잎 보존

- 조립기가 기존 `plant_mujoco_optimized.xml`의 주줄기 트리를 복사하고 기존 송이만 교체하도록 수정했다. 주줄기16몸체와 잎·잘린 가지 포함302형상을 유지하며 주줄기를 재생성하지 않는다.
- 기존 월드 배치·메시·색상·질량·관성·관절·충돌 정책을 보존한다. 새 송이의 위치·종류·회전만 바꾸며 새 하중에 따른 물리 응답까지 동일하다는 뜻은 아니다.
- 전체 식물/송이 확대 미리보기: `/root/docker_share/mujoko_debugging_data/20260923_preserved_stem_leaves/index.html`. 새 송이 물리와 수집기 연결은 여전히 미완료.

## 최신 추가: USD 송이 부착 장면 생성 미리보기

- `assemble_plant_scene.py` / `view_plant_scene.py`: 원본 송이 선택, 주줄기 호 길이 위치·국소 회전 변경, seed 재현, MuJoCo XML/JSON/미리보기 HTML 저장.
- 온실 USD 54식물/162부착 사례 확인. 162종 형상이라는 뜻은 아니다. L_00의3개 송이 배치로6장면 생성·렌더 완료.
- 2초 물리 발산은 없지만 초기 충돌 겹침1.03~2.82mm로6개 모두 겹침 기준 실패. **형태 검토용, 학습 수집 준비 미완료**. 기존 모델/데이터는 보존.
- 로봇·리프트·30후보 수집기 연결, 새 장면 분할 및 Depth+mask 학습은 아직 구현하지 않았다.
- 상세 범위/명령/검사: [PLANT_ASSEMBLY.md](mujoco-benchmark/PLANT_ASSEMBLY.md).
- 결과 `/root/docker_share/mujoko_debugging_data/20260923_attachment_preview_convex/index.html`.

## 최신 상태: RGB-D 진입각 학습·물리 검증 완료

- 브랜치 `mjlab-performance`. 수집/경로 변경 커밋 `3f636c2`, 다중 토마토 학습·검증 커밋 **`ed13ca9`**.
- 현재 방향은 고정 경로 후보를 RGB-D로 평가하는 지도학습이다. 아래 RL 기록과 구분한다.
- 데이터10대상×30경로/710관측, Tomato_05는 없음. 중심 정렬, 하부2mm, 방위각±90° Sobol, 전진45°상승→후퇴45°상승. 최종 상승32.5mm. 정상 물리에서 중심진입이면 열매 밀림과 무관하게 부분 성공.
- 8대상 학습(01,02,03,04,07,08,10,11),09검증,06테스트. DINOv2 RGB-D seed2 선택.06의64관측 모두 candidate_00001(−81.09°) 추천.
- 고유 추천1경로를 새 물리로1회 실행하여 중심진입 성공 확인, 열매 최대변위30.78mm. 64회 독립 물리 성공이나 꼭지 수확 성공이 아니다.
- 영상 없는 기준 모델도 top1성공이므로 RGB-D 우위 미입증. 확률 calibration도 불량(Brier약0.315); sigmoid1.0을 실제 성공확률100%로 해석하지 않는다.
- 결과: `/root/docker_share/mujoko_debugging_data/20260922_234403_multi_tomato_training/index.html`.
- 물리/재생: 같은 폴더 `physics/index.html`. 실행 명령·학습 분할·해석은 [learning/MULTI_TOMATO.md](mujoco-benchmark/learning/MULTI_TOMATO.md).
- 다음 판단: 재생으로 실제 진입 확인, 다른 토마토 holdout 비교, 영상 없는 모델 대비 이점 평가. 현재 모델은 저장된 관측 특징과 기존 후보를 사용하며 새 실물 이미지 입력/자유 경로 생성 기능은 아니다.

## 이전 단계: Tomato_06 RL (당시 상태 기록)


- 브랜치 `mjlab-performance`, 구현 커밋 `a085f47`. 최신 작업 요약은 [RL 작업 인수인계](mujoco-benchmark/RL_HANDOFF.md), 세부 설계는 [TOMATO06_RL.md](mujoco-benchmark/TOMATO06_RL.md).
- 팔6관절+리프트를 자유 제어하는 mjlab/MJWarp + RSL-RL PPO 구현. 고정 장면 GT 사용, 카메라 정책/RL 수확 성공은 아직 아니다.
- 256환경 10분 추가학습 완료: 170업데이트/139만 전이. 평가 최근접 거리56.16→43.17mm, 성공은 전후0. 더 가까이 접근한 뒤 머무르는 상태.
- 근접 보상이 정지에도 지급되는 문제가 유력한 원인. **보상 progress_v2 수정 완료, 새 학습은 미실행**이며 물리적 진입 가능성도 추가 확인 필요.
- 1,000회 장기 실행은 중단했고, 10분 실행은 완료했다. 아래 과거 기록의 실행 중/PID 표기를 현재 상태로 해석하지 말 것.
- 다음 작업: 저장 평가 재생 확인 → 보상과 탐색 개선 → 동일 기준의 10분 비교. 무조건 장시간 학습부터 늘리지 않는다.

## 이전 카메라 데이터 수집 단계 기록 (이후 완료, 아래 내용은 당시 상태)

1. Tomato_05의 1,000개 경로를 카메라 좌표 action v2로 저장하고 재실험 완료: 510.83초. 기존 결과와 일치.
2. 71개 RGB-D 관측과 71,000개 관측–후보 쌍의 좌표 변환 검증 완료. 목표는 부분 중심 진입과 최대 이동량이며, 꼭지 걸림 성공 판정은 아직 없다.
3. ResNet18 / DINOv2의 고정 영상 특징 + 작은 평가기, 영상 없는 기준 모델을 각각 3회 학습 완료. 검증 손실 기준 선택은 영상 없는 모델이다. AP는 실제 성공 확률이 아니다.
4. 모델 추천 21회(서로 다른 경로 3개)를 새 물리 실행으로 확인하고 2배속 영상 저장. 영상 모델은 각각 7/7 중심 진입 및 최대 이동 20mm 이하. 동일 장면 반복이며 실물 일반화 검증은 아니다.
5. 05를 제외한 10개 토마토의 수집 실행기 준비 완료. 각 대상 1후보 + 1시점 촬영 소량 검증 통과. 이후 본 수집을 진행하다 사용자 중단: 01~04 완료, 06은 정상 저장931개/남음69개. 아래 최신 재개 기능으로 이어갈 수 있다.

중단된 수집을 이어가는 명령:

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/collect_tomatoes.py \
  --resume --output /root/docker_share/mujoko_debugging_data/20260922_134834_multi_tomato_collection \
  --candidates 1000 --workers 48 --planning-workers 16 --postprocess-workers 8
```

- 상세 수집 명령과 카메라 의미: [MULTI_TOMATO_COLLECTION.md](mujoco-benchmark/MULTI_TOMATO_COLLECTION.md)
- 학습 비교와 재실행 방법: [learning/README.md](mujoco-benchmark/learning/README.md)
- 센서 저장 형식: [D435_RGBD.md](mujoco-benchmark/D435_RGBD.md), [OBSERVATION_DATASET.md](mujoco-benchmark/OBSERVATION_DATASET.md)
- 결과 데이터·모델 가중치·영상은 `/root/docker_share/mujoko_debugging_data/`에 있다. Git에는 코드와 문서를 저장한다. 계정/머신 이동 시 이 디렉터리와 기존 MuJoCo 모델·계획 자산·Python 환경도 별도 보존해야 한다.
- 카메라는 대상별 가상 시점이며 실제 로봇 관측 자세의 도달성을 검증하지 않았다. 다음 학습에서는 토마토 ID 단위로도 학습/평가를 분리해야 한다.

## 이전 측정: Isaac 로봇 팔 비용 비교

- `mujoco-benchmark/scripts/robot_cost_comparison.py` 추가. 후보49 첫10초, full/CPU240Hz/PGS64/4,
  파단 비활성. 실제 관절 모터 명령을 재생하고 측정한 고리 경로로 단독 조건과 비교했다.
- 실행 시간: 고리 headless9.87초 / 팔 headless10.78초 / 고리 GUI18.62초 /
  팔 물리+외형 숨김20.41초 / 팔 외형 표시21.24초. 준비/검사/저장 제외, 각1회 측정.
- 팔 물리/제어 약+0.91초, 팔 외형 약+0.83초. 이번 조건에서는 팔보다 전체 화면 갱신 부담이 큼.
- 팔 세 조건의 상태 완전 일치. 고리 경로 차이0.00015mm 미만. kinematic/dynamic 구동 차이로
  식물 반응 차이 최대1.22mm는 있음. 모두 접촉/관통·부착 검사 통과. 고리걸기 성공률 시험 아님.
- 결과 `mujoco-benchmark/outputs/20260922_044331_robot_cost/index.html`, CSV/JSON/스크린샷.
  요약 `mujoco-benchmark/validation/robot_cost.json`, 해석 `RESULTS.md`.
- benchmark_isaac.py에 --robot-trace/--robot-visuals/--camera-scale 추가. MuJoCo 로봇 이식 아님.
  기존 GUI는 --gui --trajectory fixture_entry_lift --hz240 사용; 조명은 stage open 전에 추가.
- timestamp 경로 입력 수정, 초기 고리 USD notice를 모든 모드에서 flush 후 reset하도록 수정.
  실패한 초기 실험은 최종 표에서 제외. 자동 테스트7개 통과. 기존 Isaac 원본 코드는 유지.

## 최신 추가: 원본 식물의 MuJoCo 비교 (2026-09-22)

- 별도 `mujoco-benchmark/`에 이식/benchmark/viewer/RGB-D/영상 생성기 추가. 기존 Isaac 코드와 USD 유지.
- 기준 `8e0d820`의 **full** 모델. 90강체/300 DOF/식물376+고리35 collision 유지.
  팔은 제외하고 world 고리 기록을 직접 재생. 양쪽 파단 비활성화한 Phase 1이다.
- CPU 원본120Hz 100개 순차 실행: Isaac 전체565.27초, MuJoCo539.83초. 병렬100환경이 아니다.
  부착 오차0.5mm 초과는 각각5/100,22/100. 엔진 변경만으로 큰 속도 이득 없음.
- MuJoCo 원본60Hz는6/6 폭주.240/480Hz는6개 fixture 검사 통과, 각각 RTF1.07/0.54.
- 별도 최적화는 잎/잘린 가지 충돌286개 비활성, 열매free/weld 병합(파단 불가).
  120Hz10개 RTF3.46, 검사 통과.60Hz6개 중2개 약1mm 관통으로 정상 채택 불가.
- GPU RGB-D10초5.63초(RTF1.78), 실제 DISPLAY=:0 viewer10초5.73초. scoped NVIDIA EGL 설정 사용.
- 결과 `mujoco-benchmark/outputs/20260922_002000_original_comparison/index.html`: 영상16개·RGB-D·CSV/JSON.
- 결론/사용법/구조 차이: `mujoco-benchmark/RESULTS.md`, `README.md`, `CONVERSION.md`.
  테스트6개, 입력 동일성/초기 상태/반복성 확인. raw 출력/대형 모델/venv는gitignore,
  작은 보존 요약은 `mujoco-benchmark/validation/engine_comparison.json`.
- 아직 전체 로봇/자동 detach/병렬 MuJoCo/RL/탐색 미구현. 원본120Hz 부착 오차를 먼저 해결해야 한다.
- 이 추가 작업의 커밋 여부는 `git status`로 확인한다.

## 최신 확인: 초경량 GPU 1·16·32·64환경 비교

- 같은 후보49 저장 경로를 모든 복제에 재생했다. 서로 다른 후보 탐색이나 계획 성능 시험은 아니다.
- 각 환경의 첫 10초 처리: 1개 22.26초 / 16개 62.96초 / 32개 106.88초 / 64개 200.27초.
  GPU 1환경 대비 합산 처리량 5.66 / 6.66 / 7.11배. 32→64의 추가 이득은 약 7%다.
- 전체 접촉 실행도 완료. 모든 복제는 GPU 1환경과 상태·접촉 결과가 일치하지만,
  동일한 0.521mm 관통으로 invalid_physics 처리됐다. 0.5mm 기준 유지. 정상 데이터로 사용 금지.
- reset/파단 API 통과, 환경 간 접촉 혼선 및 네이티브 오류 없음. 128환경은 미측정.
- 64개 전체 재생 376.16초 중 물리 56.87초, 쓰기·물리·읽기 외 비용 288.08초.
  CPU 약 110%, 전체 GPU 메모리 최대 3534MiB. 다음 병목은 환경별 판정·상태 처리다.
- 추가 도구 `ultralight_scale_benchmark.py`; 관련 자동 테스트 4개 통과.
  로그는 한글. 보고서 `runs/20260921_231100_ultralight_gpu_scale/benchmark.md` 및 CSV/JSON.
  보존 요약 `nvidia-sim/rl/validation/ultralight_gpu_scale.json`; 상세 `ULTRALIGHT_PLANT.md`.
- 모델/물리 설정은 이번 비교에서 변경하지 않았다. 사용자 세션은 유지했다.

## 이전 수정: 초경량 모델과 CPU 단일 환경 실시간 미리보기

- 이전 light 작업은 `a270554`로 커밋했다. 그 뒤 초경량 작업의 커밋 여부는 `git status`로 확인한다.
- `--plant-resolution ultralight` 추가. full/light와 원본 USD 유지. 기본값 full/GPU 유지.
- 식물 79강체/234자유도(full), 79/165(light) → **33/66(ultralight)**.
  줄기 캡슐 68개는 원래 위치·반경대로 유지하고 여러 조각을 한 강체에 묶는다.
  전체 활성 식물 충돌체 90개, 총질량 유지. 목표 꼭지 3분절, 열매 11개와 파단 유지.
  원거리 말단 힌지 병합으로 물리 응답과 충돌 제외 범위는 달라진다.
- `--physics-device cpu --num-envs 1` 추가. 실제 CPU PhysX/CCD, GPU 렌더링이다.
  120Hz/PGS64 유지, 매 스텝 0.5mm 관통 검사 유지. CPU/GPU 데이터·영상 비교를 분리한다.
- CPU 초경량 10초 처리 7.54초(계획·판정·저장 포함), full CPU 12.54초.
  화면을 켠 10초 고정 구간도 9.24초에 처리했다.
  초경량 화면 표시 후보49는 16.48초 분량을 13.71초에 실행. 초기 로딩/촬영 별도.
- CPU 정지·휘어짐/복원·파단·reset·4방향 선삽입 고리 유지 검사 모두 통과.
  실제 후보49는 excessive_displacement이며 수확 성공이 아니다. 관통 검사 통과.
- GPU 초경량 4환경×10초는 46.41초. 이전 light의 66.08초보다 개선됐지만 실시간은 아니다.
  GPU 60Hz/낮은 반복 수의 후보49는 관통 기준 초과. 그 설정을 기본값으로 채택하지 않았다.
- 상세 실행법/범위: `nvidia-sim/rl/ULTRALIGHT_PLANT.md`.
  실험: `nvidia-sim/rl/runs/20260921_ultralight/`, 요약: `validation/ultralight_plant.json`.

## 이전 수정: 경량 모델의 주줄기 잎·잘린 가지 충돌 제외

- `full`은 그대로 기본값이다. `light`는 이제 `--main-appendage-collisions ignore`가 기본이다.
  `light --main-appendage-collisions keep`으로 이전 경량 모델을 비교할 수 있다.
- 주줄기 부속 잎·잘린 가지 286개 convex 충돌체를 생성하지 않는다. 원본 시각 메시와
  주줄기 추종은 유지한다. 두꺼운 주줄기, TRUSS 송이 가지·꼭지, 열매 충돌은 유지한다.
- 송이는 이미 원거리 관절을 63 → 50개로 줄였다. 목표 주변의 충돌 형상은 유지한다.
- 정책을 config/backend/CSV/JSON/summary/대표 영상에 전달하고 정책이 다르면 resume 및
  영상 비교를 거부한다. 정책이 없는 이전 실행은 keep으로 해석한다.
- 실제 활성 충돌체 개수는 plant_model.json의 collision_inventory로 확인한다.
- 실제 집계 376 → 90개(줄기 68, 열매 11, 열매 꼭지 11). 4환경/10초 비교:
  full 초기화/실행 17.76/76.03초, light keep 17.67/68.55초, light ignore 9.85/66.08초.
  이번 잎 충돌 제외의 추가 실행 개선은 약 3.6%로, 주된 효과는 초기화와 충돌체 수 감소다.
- 관절·줄기 형상/질량·남은 충돌 필터는 light keep과 동일하고 초기 depth가 정확히 같다.
  관련 자동 테스트 39개 통과. 실행 폴더 `nvidia-sim/rl/runs/20260921_main_appendages_comparison`.
- ignore로 5·49 접촉 실행 완료: 과도 변위 2개, 관통 제외 0개(최대 0.222/0.317mm),
  잎/잘린 가지 접촉 이벤트 0개. 정지/휘어짐·복원/파손/리셋/네 방향 고리 fixture 유지 통과.
  검증 요약 `validation/main_appendage_light.json`. 수확 성공을 의미하지 않는다.
- 자세한 실행법/범위: [LIGHT_PLANT.md](nvidia-sim/rl/LIGHT_PLANT.md).

## 이전 수정: 원본을 유지하는 선택형 경량 식물 (잎 충돌 keep)

- 원본 USD와 기본 모델은 유지한다. GPU dataset의 `--plant-resolution full`이 기본값,
  `--plant-resolution light`로만 별도 경량 구조를 사용한다. 자동 출력 폴더는 `_light`로 구분.
- 충돌체·질량·열매 11개와 파손 연결은 유지. 목표 70mm 이내/목표 꼭지는 원래 관절 유지,
  먼 구간은 관절 4개당 약 1개만 휘게 하고 나머지는 부모에 고정 연결한다.
- Tomato_05 자유도 234 → 165. 주줄기의 탄성 연결 15 → 5개, 잎 충돌체 286개는 유지.
- 동일 4환경/10초 비교: 실행 75.43 → 66.86초(11.4% 단축), 물리 45.76 → 38.23초.
  64/256환경 처리량은 재검증하지 않았다. 초기화는 각각 18.06/17.77초.
- full의 4후보 처음 600스텝이 기존 원본 trace와 정확히 일치했다.
- light 0·1·5·49 완료: miss 1개/과도 변위 3개/관통 오류 0개. 5번은 원본의 부분 진입과
  달리 조기에 과도 변위로 중단되어 모델 간 판정 차이가 있다. 최종 후보는 원본 재검증 필요.
- 실제 모델·관절 목록을 plant_model.json에 저장하고 JSON/CSV/summary/영상에도 종류를 전달한다.
  서로 다른 모델로 resume/영상 비교를 통과시키지 않는다. 관련 자동 테스트 37개 통과.
- 경량 mechanics: 정지/휘어짐·복원/파손/리셋/고리 유지 통과. 0.2N 최대 변위 24.67mm,
  잔류 1.51mm, 연결부 최대 간격 0.022mm. 네 방향 고리 fixture 유지 통과.
  로봇의 진입 성공을 의미하지 않는다. 검증 요약 `validation/light_plant.json`.
- 자세한 범위/실행법: [LIGHT_PLANT.md](nvidia-sim/rl/LIGHT_PLANT.md).
  검증 실행: `nvidia-sim/rl/runs/20260921_plant_model_comparison`,
  경량 접촉 결과: `nvidia-sim/rl/runs/20260921_light_plant_4env`.

## 이전 수정: 단일 환경 GUI 속도 개선

- 동일 49번 GUI 실행 구간 150.34초 → 103.95초(약 31% 단축).
- 단일 환경은 원본 하우스를 표시하며, 다중 환경은 표시용 격자를 유지한다.
- 화면용 USD 동기화를 렌더 시점으로 제한하고 식물 skin 계산을 캐시/일괄 적용한다.
- 120Hz 물리/60Hz 제어/관통 검사 유지. 계획 명령, 제어 trace 995개,
  물리 접촉 trace 1,990개, 접촉 이벤트 306개 및 audit이 원본과 정확히 일치했다.
- 화면 시간 960Hz 고정값 수정. 화면 비용 기반 갱신율 조정 및 display_timings 추가.
- 실행법/검증 범위: [SINGLE_VIEW_PERFORMANCE.md](nvidia-sim/rl/SINGLE_VIEW_PERFORMANCE.md).
  대규모 환경 처리량 개선으로 일반화하지 않는다.

## 이전 수정: candidate_00049 접촉 관통 대응

- GPU dataset 기본값은 `contact120`: 120Hz 물리 / 60Hz 제어 / PGS 64/4 /
  armature 5e-4. 이전 practical60과 동일한 형상/물성/로봇 명령에서 timestep만 절반.
- 49번 동일 명령의 매 물리 스텝 겹침: 60Hz 최대 1.793mm → 120Hz 0.196mm.
  수정 후 줄기가 밀려 목표 변위 20mm 중단. 수확 성공으로 바꾼 것이 아니다.
- contact offset/CCD만 조정, 드라이브 강성 감소, velocity iteration 증가만으로는
  관통이 남았다. TGS 60Hz는 초기 파손. 120Hz 32/4도 0.5mm 선별 기준 미달.
- 새 dataset은 실제 USD/native pose의 고리 arc/rail 대 식물 capsule/sphere 겹침을
  매 스텝 검사한다. 0.5mm 초과면 `invalid_physics`로 중단/별도 저장/대표 영상 제외.
  mesh와 스텝 사이 연속 swept 검사까지 보장하는 것은 아니다.
- `physical_audit.json`, `tool_contact_trace.json`, CSV 물리 유효성 필드를 저장한다.
  오류를 숨기거나 고리/식물 pose를 인위적으로 보정하지 않는다.
- 문서: [CONTACT_PENETRATION_FIX.md](nvidia-sim/rl/CONTACT_PENETRATION_FIX.md).
  진단 원본: `nvidia-sim/rl/runs/20260921_contact_fix/`.
  기존 원본 데이터/영상은 변경하지 않고 새 결과와 구분한다. 실행 원본과 영상은
  git 제외 대상인 `runs/`에 로컬 보관하고 코드·문서·검증 요약을 커밋한다.
- 4환경에서 0·1·5·49 완료: 관통 선별 기준 초과 0개, 과도 변위 2개/miss 2개.
  5번 중심 진입 부분 성공 0.833초 유지, 완전 걸림 0. 모든 관절 명령은 원본과 동일.
  정지/탄성·복원/파손/리셋 및 미리 끼운 고리 네 방향 접촉 유지 fixture 통과.
- 49번 단독 측정: 60Hz guard 45.17초 / sim16.183초,
  120Hz 84.65초 / sim16.583초. sim 시간당 비용 약 1.83배, 초기화/렌더 제외.
  전체 64환경 처리량은 아직 재측정하지 않았다.
- 수정 전후 2배속 영상:
  `nvidia-sim/rl/runs/20260921_200605_candidate49_contact_fix_2x/index.html`.
  검증 요약 `validation/contact_penetration_fix.json`, 관련 자동 테스트 37개 통과.

## 이전 진단: candidate_00049 물리 관통 확인

- 사용자 영상 지적으로 저장된 native pose/접촉을 재검사했다. 49번 고리 segment_22가
  TRUSS_Rachis_08을 삽입 중 16.6~16.9초에 관통했다. 최대 겹침 1.793mm.
- 재실행 비교 통과와 물리적 유효성은 다르다. 이전의 49번 "정상 비목표 접촉 사례"
  해석을 정정하고 영상 모음에서 물리 오류로 분리했다. 대표 2개/진단 2개로 변경.
- 원본 CSV/판정은 보존하고 candidate_00049/physical_audit.json을 추가했다.
  당시에는 자동 제외 필터가 없었다. 후속 수정으로 새 dataset에 제외 검사를 추가했다.
- 근거: `nvidia-sim/rl/runs/20260921_candidate49_collision_audit/README.md`,
  `validation/candidate49_penetration.json`. 이 변경의 영상 분류 테스트 6개 통과.
- 다른 영상의 rachis 겹침 선별: 05 최대 0.266mm, 01 없음, 00 최대 0.701mm.
  전체 64개/모든 충돌체 검증은 아니다. 이 최초 진단 때는 물리 설정을 변경하지 않았다.

## 최신 후속 작업: 64환경 실측과 대표 영상 자동화

- 64환경/64개 서로 다른 staged6d 후보, CPU 경로 16 workers, PhysX CPU threads 8.
  practical60 물리/로봇 속도/중단 기준 유지. 실제 전체 실행 **814.70초 (13분 35초)**.
- rollout 740.32초: physics 481.42초, 실행/판정 192.93초, 명령 쓰기 38.61초.
  CPU 경로 작업 전체 10.59초. 명령 FK를 미리 계산하고 동일 tick 조회를 재사용했다.
- 이전 4환경/8후보 대비 후보당 관측 처리량 2.64배. 풀 크기/조기 중단 비율이 달라
  순수 코드 최적화 효과로 해석하면 안 된다. 64/64는 마지막 빈 슬롯 손실이 크다
  (active env-step fraction 0.414). 대량 실행은 continuous 재충전을 사용한다.
- CPU 초기화 최대 2801%, 경로 계산 시 최대 약 1892%, rollout 중앙값 117%.
  GPU 메모리 최대 6191MiB, 샘플 GPU 사용률 중앙값 25%. CPU 직렬 판정과
  physics 동기화 병목이 남아 있다. 전체 자원을 최대 활용하는 최적 설정을 찾았다는 뜻은 아니다.
- 54개 과도 변위, 8개 miss, 2개 비목표 접촉, 완전 걸림 0.
  candidate_00005만 안전 기준 내 중심 진입 1.22초 달성, 최종 miss.
- 이전 8개와 새 관절 명령 파일은 전부 수치적으로 동일. 최종 범주/부분 진입 여부도 일치.
  GPU 환경 수 차이로 과실 최대 변위 수치는 최대 0.294mm 차이가 있었다.
- 원본: `nvidia-sim/rl/runs/20260921_175044_staged6d_64env_videos/`.
  `performance_report.json`, `resource_samples.json`, `dataset/summary.json` 참고.
- `--representative-videos`로 완료 후 대표 사례 선정→저장 명령 재실행→원본 trace 비교→
  두 외부 시점 2배속 MP4/HTML 생성. 부분 성공을 완전 걸림으로 바꾸지 않는다.
  자세한 설명: [DATASET_REPRESENTATIVE_VIDEOS.md](nvidia-sim/rl/DATASET_REPRESENTATIVE_VIDEOS.md).
- 관련 자동 테스트 35개 통과. `representative_videos_2x/index.html` 영상 생성 완료:
  00005 부분 진입, 00001 과도 변위, 00049 비목표 접촉은 원본 비교 통과.
  00000은 최종 miss는 같지만 중간 주줄기 변위가 최대 15.29mm 달라 별도 진단 영상으로 분리.
  두 시점 2배속, 1280x612/30fps, 네 MP4 전체 디코드와 프레임 수 검사 통과.
  영상 후처리 추가 시간은 843.39초(약 14분). `validation/trajectory_64env.json`에 검증 요약.
  재실행이 원본 병렬 실행과 항상 같지는 않으므로 모든 원본 영상이 필요하면 본 실행에서
  물리 pose를 저장하는 방식으로 확장해야 한다. 현재 코드는 차이를 숨기지 않고 표시한다.

## 이전 작업: 떨어진 위치부터 접근하는 6D 소규모 시험

- 기준 커밋 `973806b` 이후 작업. [TRAJECTORY_PILOT.md](nvidia-sim/rl/TRAJECTORY_PILOT.md) 참고.
- `--trajectory-mode staged6d` 선택 시 170mm 떨어진 준비 위치→진입→수평 삽입→상승→1초 유지.
  기존 legacy 기본 동작은 그대로다. 새 모드는 접촉만으로 상승을 조기 종료하지 않는다.
- 실제 로봇 4환경·8개 Sobol 후보: 한 번의 완료 실행 269.06초, rollout 234.25초.
  6개 과도 변위, 2개 끝까지 실행 후 miss. candidate_00005는 GT 중심 진입 1.87초를
  달성했지만 꼭지 걸림은 아님. 이 새 후보의 영상 검증은 아직 하지 않았다.
- 동일 4환경/실패 비율 단순 예상: 64개 약 32분, 128개 약 63분, 2,048개 약 16시간 40분.
  영상 제외, 8개 표본의 제한이 있다. 새 경로의 16/32/64환경 처리량은 미측정이다.
- 완료 원본은 `runs/20260921_170634_staged6d_timing_pilot/env4_n8_final/`.
  앞선 두 폴더는 대화 중단으로 종료된 불완전 실행이다. 마지막 그래프 변수명 오류는
  물리 8개가 모두 저장된 뒤 발생했고 오프라인 후처리로 복구했다. 원래 오류 기록 보존.
- 관련 테스트 30개 통과. 새 source fingerprint 및 두 parameter schema의 plot 지원 추가.

## 이전 작업: 실용 60Hz

사용자가 960Hz와 완전히 같은 결과보다 밀림·걸림·휘어짐·복원을 유지하는 실용적
처리량을 우선하도록 기준을 변경했다. 따라서 아래의 이전 "960Hz 유지" 작업 지침은
후속 작업에서 대체됐다. 원래 성능/물리 이력은 삭제하지 않고 보존한다.

- 기준 커밋은 `bc84d41`; 실용 preset/검사/문서와 관찰 영상 도구는 후속 커밋에 포함했다.
  한글 커밋 제목은 `시뮬레이션: 실용 60Hz GPU 물리 설정과 접촉 관찰 영상 도구 추가`다.
- GPU dataset 기본은 `--physics-preset practical60`: 60Hz, armature 5e-4, PGS 64/4 iterations.
- `reference960`으로 원래 960Hz/1e-5 모델 선택 가능. `practical120`은 120Hz/3e-4 중간 비교용.
- 60Hz/5e-4에서 정지·실제 과실 하중/복원·원래 임계값 파손·리셋·네 방향 CAD 고리
  접촉 유지 검사가 통과했다. 고리 fixture는 미리 삽입된 별도 kinematic 도구이며 로봇 진입 성공은 아니다.
- 저장 로봇 경로 4환경 재생: 17.50초에 sim 4.4초 실행. 과거 960Hz는 168.69초/sim 4.333초였다.
  물리 모델이 다르므로 정밀 동등성 speedup이라고 부르지 않는다.
- 16환경 같은 경로 재생도 27.71초/sim 4.4초로 통과했고 native 접촉 라우팅/reset 오류가 없었다.
- 실제 dataset 4환경/서로 다른 4후보/8초 제한: RGB-D·계획·reset 통과. 결과 1개 과도 변위,
  3개 incomplete로 전체 수확 성공 검사는 아니다. 관련 자동 테스트 60개 통과.
- 30Hz/1e-3 잔류 변위가 커 제외. 60Hz/1e-3/32 iterations는 연결 간격 0.555mm로 0.5mm 기준 초과해 제외.
- 60Hz/3e-4 초기 검사는 idle을 과실만 검사했던 문제가 있었다. 주줄기도 포함하도록 수정했고
  최종 채택 모델은 5e-4다. 초기 `behavior60`의 passed 값을 그대로 최종 판정에 사용하지 않는다.
- 코드: `gpu_physics_presets.py`, `gpu_behavior_probe.py`, `gpu_low_hz_benchmark.py`, GPU runner/sim 변경.
- 새 검증 원본: `nvidia-sim/rl/runs/20260921_low_hz_practical/` (ignored).
- 최신 문서/실행법/레퍼런스: [GPU_PRACTICAL_PHYSICS.md](nvidia-sim/rl/GPU_PRACTICAL_PHYSICS.md).
- 60Hz 동작 관찰 영상: [GPU_PHYSICS_VIDEOS.md](nvidia-sim/rl/GPU_PHYSICS_VIDEOS.md).
  `runs/20260921_162920_practical60_contact_videos_2x/`에 실제 로봇 밀림, 하중/복원,
  미리 끼운 고리 유지의 두 시점 영상과 원본 pose/contact를 저장했다. 2배속 기본,
  고리 1배속 추가. 촬영 코드는 후속 커밋에 포함했고, 영상은 ignored runs에만 있다.
- 요약: [gpu_practical_physics.json](nvidia-sim/rl/validation/gpu_practical_physics.json).
- 다음에는 실제 로봇의 성공 진입/상승 경로, 타겟 1~11, 빠른 접촉과 더 많은 환경 수를 검증한다.
  아직 실측 보정이나 모든 상황의 비관통 보증은 없다. RL/DR을 추가하지 않았다.

## 1. 현재 결론과 바로 이어갈 일

- 기존 작업은 **`1305651`**까지 커밋했다. 브랜치는 **`nvidia-sim`**이다.
- 그 이후 **저주파 탄성 안정성 진단·설정 옵션·검증 자료를 이 문서와 함께 후속 커밋에 포함**했다.
  해당 커밋 제목은 `feat(sim): add elastic stability diagnostics and handoff`이며 해시는 `git log`로 확인한다.
- CPU 경로 계산 병렬화는 검증했고 기본 최대 8개 worker로 동작한다.
- GPU 물리 및 여러 환경의 동시 실행은 구현되어 있다. CPU 물리 worker pool과 혼동하지 않는다.
- 현재 핵심 미해결 문제는 **물리 결과를 유지하면서 960Hz를 낮추는 것**이다.
- 보조 관성(armature)을 키워 저주파의 정지 흔들림은 줄였지만, 접촉 변형과 파손 응답이 달라졌다.
  **60/240/480Hz를 기존 물리의 동등한 고속 대체 설정으로 채택하지 않았다.**
- 기본값은 **960Hz, armature 1e-5 kg·m², 로봇 제어 60Hz**를 유지했다.
- 이 파일을 작성할 때 관련 dataset/probe/planning worker는 실행 중이지 않았다.

다음 담당자는 [저주파 실험 상세](nvidia-sim/rl/GPU_ELASTIC_STABILITY.md)와
[수치 기록](nvidia-sim/rl/validation/gpu_elastic_stability.json)을 먼저 읽고,
후속 커밋의 코드를 검토한 뒤 탄성 관절·초기 하중·파손 연결부의 시간 간격 민감도를 조사하면 된다.
이미 실패한 설정을 다시 기본값으로 채택하거나, 같은 실패 라벨만 보고 물리 동등성을 선언하지 않는다.

## 2. 사용자 목표와 제약

사용자는 고리형 그리퍼로 방울토마토 아래에서 접근하고, 안으로 진입한 뒤 상승하여
꼭지 윗부분을 걸고 필요하면 당기는 동작을 연구 중이다. 주줄기를 밀면 송이와 과실도
움직여 고리걸기가 실패하는 상황까지 시뮬레이션되어야 한다.

현재 단계는 **고정 장면의 접근 pose 후보 → 실제 physics 실행 → 결과 데이터셋 생성**이다.
현재 성능/물리 진단 타겟은 `Tomato_05`이며, 장기적으로 Tomato_01~11 및 다양한 실험을 다룬다.

- 이번 데이터 생성 단계에서는 RL/IL, pose inference, segmentation 학습, domain randomization을 하지 않는다.
- 후보 생성과 사후 판정에는 simulator GT를 쓸 수 있다. 실제 perception이 구현되었다고 설명하지 않는다.
- RGB-D는 **진입 전** 촬영한다. 후보 실행 뒤 성공/실패 라벨을 붙인다.
  성공한 순간의 사진만 저장하는 방식이 아니다.
- 목표를 바라보는 관찰 자세, 원본 영상과 타겟 주변 문맥을 포함하는 crop을 사용한다.
  꼭지나 고리가 항상 보인다고 가정하지 않는다. 가림 상태도 실제로 존재한다.
- 장기 sim-to-real에서는 카메라와 joint 정보를 우선 고려하고, 힘 센서는 제외하고자 한다.
- 사용자가 제공한 house/식물/로봇 장면을 기반으로 한다. 무관한 단순 환경으로 바꾸지 않는다.
  멀티환경 표시/실행 최적화의 세부 범위는 기존 문서와 코드를 확인한다.
- 식물 위치 예: `--spawn-stem -0.75 0.55 0.32`.
  초기 자세는 ROS2 GUI의 left pick ready 계열, 리프트는 타겟 송이보다 0.4m 낮게 시작하는 요구가 있었다.
  실제 GPU runner의 cfg 설정을 기준으로 확인한다.
- 영상/실험 출력은 datetime으로 시작하고 간단한 설명이 붙은 폴더에 저장한다. 영상은 주로 2배속 요청이었다.
- 목표 처리량은 약 1,000개 후보/1시간이지만 **달성 검증은 안 됐다**.
- 최근 사용자는 큰 실험의 로딩을 피하려고 **1환경과 4환경으로 먼저 비교**하도록 요청했다.
  성능 검증 때 곧바로 256/1,000환경을 띄우지 않는다.

`--num-envs 32 --candidates 128`은 총 128개 pose를 최대 32개씩 실행한다.
32×128개의 후보를 생성하는 것이 아니다. `continuous`는 빈 슬롯에 다음 후보를 넣는 방식이다.

## 3. 실행 환경과 Git 상태

| 항목 | 값 |
|---|---|
| 저장소 | `/root/farmily_tomato` |
| 브랜치 | `nvidia-sim` |
| 인수인계 포함 후속 커밋 | `feat(sim): add elastic stability diagnostics and handoff` (해시는 `git log` 확인) |
| 성능 개선 기준 커밋 | `1305651 perf(sim): precompute candidate paths on CPU workers and benchmark timesteps` |
| 이전 관련 커밋 | `ec0c9db perf(sim): batch GPU commands and continuously schedule candidates` |
| 이전 관련 커밋 | `8995518 feat(sim): scale GPU candidate experiments and add live grid viewer` |
| 이전 관련 커밋 | `d14589d feat(sim): add parallel dataset workers, GPU probes and result dashboard` |
| 런타임 | Isaac Sim 5.1 + Isaac Lab |
| Python | `/root/isaaclab_env/bin/python` |
| GPU | RTX 3090 Ti, 약 24GB |
| CPU | Threadripper 3960X, 24코어/48스레드 |
| 물리 모니터 | `DISPLAY=:0` 사용 가능 |

`run_gpu_candidate_dataset.sh`는 Python 경로를 `FARMILY_ISAAC_PYTHON`으로 재지정할 수 있다.
기본은 위 Python이며 `PYTHONHOME`, `VIRTUAL_ENV`를 해제하고 `PYTHONPATH`를 설정한다.

후속 커밋에 포함한 파일(아래 상태 문자는 커밋 전 기록이며, 이 문서도 함께 포함):

```text
 M nvidia-sim/rl/GPU_TIMESTEP_AND_PLANNING.md
 M nvidia-sim/rl/elastic_plant.py
 M nvidia-sim/rl/gpu_dataset_runner.py
 M nvidia-sim/rl/gpu_dataset_sim.py
 M nvidia-sim/rl/gpu_probe.py
 M nvidia-sim/rl/gpu_probe_worker.py
 M nvidia-sim/rl/test_gpu_dataset_runner.py
 M nvidia-sim/rl/test_gpu_probe.py
?? nvidia-sim/rl/GPU_ELASTIC_STABILITY.md
?? nvidia-sim/rl/validation/gpu_elastic_stability.json
```

**계정 전환 시 보존:** 같은 워크스페이스를 사용하면 이 문서와 코드를 읽어 이어갈 수 있다.
코드와 요약 자료는 로컬 Git에 커밋했으며 push는 하지 않았다. 새 머신에서 원격 저장소만
clone하면 이 로컬 커밋이 포함되지 않을 수 있다. 또한 `nvidia-sim/rl/runs/`는 Git ignore
대상이므로, 다른 머신으로 옮길 경우 로컬 커밋과 필요한 실험 원본 폴더를 별도로 보존해야 한다.

## 4. 현재 실행 구조

| 파일 | 역할 |
|---|---|
| `nvidia-sim/run_gpu_candidate_dataset.sh` | 현재 GPU 데이터셋 실행 진입점 |
| `nvidia-sim/rl/gpu_dataset_runner.py` | CLI, config, resume 일치 검사, subprocess 관리 |
| `nvidia-sim/rl/gpu_dataset_sim.py` | 실제 장면, GPU 설정, 관측/검증, 후보 실행 스케줄 |
| `nvidia-sim/rl/dataset_scene.py` | 복제 환경/슬롯과 물리 실행 |
| `nvidia-sim/rl/dataset_motion.py` | 기존 후보 경로 계산 및 실행 |
| `nvidia-sim/rl/gpu_planning.py` | 실장면 모델 export 및 CPU 경로 사전 계산 서비스 |
| `nvidia-sim/rl/gpu_planning_worker.py` | Isaac을 띄우지 않는 CPU worker 진입점 |
| `nvidia-sim/rl/elastic_plant.py` | 탄성 식물 articulation, preload, 시각 모델 연결 |
| `nvidia-sim/rl/gpu_probe_worker.py` | 저장 명령을 재생하는 물리/정지/하중 분리 진단 |
| `nvidia-sim/rl/gpu_probe.py` | probe 실행/비교, 유효하지 않은 speedup 제외 |
| `nvidia-sim/rl/gpu_timestep_benchmark.py` | 1/4환경의 물리 Hz 비교 |
| `nvidia-sim/rl/gpu_planning_benchmark.py` | 동일 모델의 순차/병렬 CPU 경로 비교 |

현재 GPU dataset은 GPU dynamics + GPU broadphase + CPU tensor readback/native collider reports를 사용한다.
**완전히 CUDA tensor로만 처리하는 파이프라인이 아니다.** 접촉 object 식별을 위해 readback을 유지한다.
현재 GPU 설정은 PGS, position iteration 64, velocity iteration 4,
`gpu_max_num_partitions=1`, GPU CCD off다. 이전 CPU/TGS와 동일하다고 보증하지 않는다.
probe의 기본 solver/partition은 dataset과 다를 수 있으므로 재현 명령에서는 명시한다.

CPU planning은 실제 USD에서 추출한 FCL 도형 44개/자기충돌 검사 쌍 228개와 FK/IK 정보를 전달해
동일한 `dataset_motion.plan()`을 호출한다. IK·joint limit·자기충돌 검사를 생략하지 않는다.
최대 8개 worker로 다음 후보를 미리 준비하고 물리와 겹쳐 실행한다.
`--planning-workers 0`은 원래 순차 방식이다.

`DATASET READY`는 경로 계산 완료, `DATASET PHYSICS`는 물리 실행이다.
예전 `DATASET START`가 순차로 느리게 증가하던 큰 원인은 경로 계산이었다.
`planning_s`는 메인 프로세스의 대기 시간이며, worker 계산 시간의 합과 다르다.

타겟/성공 판정은 기존 object identity와 접촉 판정을 유지한다. probe의 `identity()`에서는
목표 distal `PedicelCollider` 및 proximal 마지막 collider, ring segment 08~23을 기록한다.
이 문서만 보고 성공 조건을 새로 추정하지 말고 `dataset_motion.py` 및 실제 target prim 정보를 확인한다.

## 5. 커밋된 성능 개선 결과

상세: [GPU_TIMESTEP_AND_PLANNING.md](nvidia-sim/rl/GPU_TIMESTEP_AND_PLANNING.md),
[검증 JSON](nvidia-sim/rl/validation/gpu_timestep_planning.json).

64개의 서로 다른 전체 후보 경로를 계산한 결과:

| 방식 | 경로 준비 시간 | 순차 대비 |
|---|---:|---:|
| 순차 | 86.14초 | 1.00배 |
| 4 workers | 23.86초 | 3.61배 |
| 8 workers | 13.57초 | 6.35배 |

명령 배열, waypoint, pose와 preflight 결과가 정확히 같았다. **전체 물리 실험이 6.35배 빨라진다는 뜻은 아니다.**
4환경/8후보/8제어 스텝의 통합 비교도 명령·trace·reset이 같았으며 경로 대기는 12.61→3.24초였다.
이 짧은 검사에는 접촉이 없었고 결과는 모두 `incomplete`다.

원래 armature 1e-5로 Hz만 낮추면:

- 60/120/240Hz는 접촉 전에도 식물이 밀려 조기 종료했다.
- 480/720Hz는 같은 접촉 구간을 진행했지만 960Hz 대비 1mm 회귀 기준을 초과했다.
- 높은 물리 스텝 수의 비용은 여전히 큰 병목이다. CPU 사용률만으로 GPU 물리 비용을 판단하지 않는다.

## 6. 후속 저주파 안정화 작업과 결과

추가한 코드:

- dataset CLI `--elastic-joint-armature` (기존값 1e-5, 유효 범위 0~0.01, 유한값 검사).
- 실제 PhysX DOF armature 적용 확인, config/backend/후보 metadata 저장, 설정이 다른 resume 거부.
- probe의 `--no-gravity`, `--no-preload`, `--preload-mode`, `--joint-armature`, `--pulse-force`.
- 정지 중 식물/과실 이동 및 joint 속도 기록. probe 예외 시 비정상 종료 코드 반환.
- 비교 시 다른 armature/preload 모델 또는 실패한 native API 검사는 유효한 speedup으로 인정하지 않음.
- production preload/reset은 원래 방식 그대로다. 실패한 drive/velocity 방식은 probe 안의 진단용 monkeypatch만 남겼다.

기존 모델은 articulation에 가는 줄기와 과실 연결 anchor를 구성하고,
초기 Jacobian으로 중력 보상 preload를 계산하여 고정 effort로 넣는다.
이번 진단에서는 geometry, 분할 수, 질량, 강성/감쇠, 마찰, 파손 임계값을 바꾸지 않았다.
단, **추가 armature 자체는 동역학을 바꾸는 추가 관성**이다.

240Hz에서 로봇 정지 2초 시험:

- 원래 모델의 최대 식물 바디 이동 약 58.5mm.
- 중력+preload 제거 시 약 0.072mm. 충돌만 제거 시 약 32.4mm.
- preload를 position/velocity drive로 옮기면 불안정/파손. 채택하지 않았다.
- armature 1e-4: 약 0.43mm. armature 1e-3: 약 0.10mm.
- 60Hz / armature 1e-3: 약 0.50mm. 정지 결과만으로 접촉 정확도를 보증하지 않는다.

저장된 `candidate_00003` 명령의 접촉 재생 시간(초기화 제외):

| Hz / armature | 1환경 wall | 4환경 wall | 실행 sim 시간 |
|---|---:|---:|---:|
| 960 / 1e-5 기존 | 156.25초 | 168.69초 | 4.333초 |
| 480 / 1e-4 | 79.54초 | 88.31초 | 4.350초 |
| 240 / 1e-4 | 40.83초 | 47.90초 | 4.383초 |
| 60 / 1e-3 | 11.74초 | 17.32초 | 4.350초 |

모두 비목표 잎을 접촉한 뒤 `excessive_displacement`로 종료했다.
성공 고리걸기/전체 60초 모션 처리량 검증이 아니다. 4환경은 같은 명령의 복제다.
물리 모델과 종료 시점이 다르므로 위 비율을 동등한 물리의 가속률로 발표하지 않는다.

0.2N에 해당하는 초기 Jacobian 고정 토크를 0.5~1.5초 가한 뒤 제거한 시험:
최대 목표 변위는 기존 약 37.64mm, 240Hz/1e-4 약 21.39mm, 60Hz/1e-3 약 16.56mm였다.
복원은 일어나지만 원래 변형 응답과 다르다. 변형 중인 지점에 일정 world force를 주는 시험은 아니다.

같은 armature끼리 시간 간격을 비교해도:

- 1e-4, 960 vs 240Hz: 고리 간격 지표 최대 차이 3.83mm, 종료 3제어 스텝 차이.
- 1e-4, 960 vs 480Hz: 고리 간격 지표 최대 차이 1.96mm, 종료 1제어 스텝 차이.
- 1e-3, 960Hz: **첫 제어 스텝 0.0167초에 로봇 접촉 없이 비목표 `HarvestJoint_05_4` 파손**.
  native API 종합 검사도 false. 이것은 안정한 비교 기준이 아니다.

회귀 허용치는 위치 지표 1mm / 로봇 joint 0.005rad / 종료 1제어 스텝으로 두었다.
이 기준은 실물 정확도나 tunneling 부재를 보증하는 값이 아니다.
960Hz도 실물 정답으로 검증된 모델은 아니다.

## 7. 실험 자료 위치와 검증 범위

모두 저장소 기준 `nvidia-sim/rl/` 아래다.

| 위치 | 내용 |
|---|---|
| `validation/gpu_timestep_planning.json` | 커밋된 CPU 병렬화/Hz 비교 요약 |
| `validation/gpu_elastic_stability.json` | 후속 안정성·접촉·복원·통합 시험 요약 |
| `runs/20260921_143244_gpu_timestep/` | 원래 모델 1/4환경 × 60~960Hz 시험 |
| `runs/20260921_144300_planning_validation/` | CPU 64경로 benchmark 및 순차/병렬 통합 비교 |
| `runs/20260921_150500_elastic_diagnosis/` | 중력/preload/contact/drive/armature 분리 진단 |
| `runs/20260921_151000_elastic_validation/` | 접촉 재생, pulse, 같은 관성의 Hz 비교, 데이터셋 통합 |
| `runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003/` | 저장된 재생 fixture |

fixture 명령 SHA256:
`cd300bcd280159339580dcba375ed759459f458eb8e84d03da730701aa5d2ad2`.

마지막 validation 폴더의 주요 하위 폴더:

- `motion_240_{1,4}`, `motion_60_{1,4}`, `motion_480_{1,4}`: 접촉 재생.
- `motion_240_stronger_{1,4}`: 240Hz / armature 1e-3 추가 진단.
- `reference_240_model_960`, `reference_60_model_960`: 각각 같은 armature의 960Hz 기준 시도.
- `pulse_960`, `pulse_240`, `pulse_60`, `pulse_240_stronger`: 하중/복원 시험.
- `*_harness_error`: 첫 pulse 구현 오류 때문에 실패한 자료. 유효 결과에서 제외했다.
  실제 joint effort 접근을 수정한 뒤 위 pulse 폴더로 재실행했다.
- `dataset_240`, `dataset_60`: 각 4환경/8개의 서로 다른 후보/16제어 스텝 통합 검사.
  초기 상태·독립 reset·camera·armature·planning 연동 통과. **각 8개 `incomplete`, dataset_complete=false**.
- `run_*.py`, `summarize.py`: 당시 실험 orchestration/요약 스크립트. `runs/` 내부라 Git 미추적이다.
  과거 폴더 완료 조건을 기다리는 스크립트도 있으므로 그대로 재실행하지 말고 명령을 읽어 새 폴더에 실행한다.

기록된 마지막 자동 검증은 관련 테스트 **55개 통과** 뒤 비교 도구에 API 실패 제외 테스트를
추가하고 해당 파일 **6개 통과**를 확인한 상태다. 새 테스트를 포함한 전체 묶음을 한 번에
다시 실행한 것은 아니다. `git diff --check`도 통과했다.

## 8. 재개 및 재현 명령

먼저 상태와 문서를 확인한다. 이 문서 작성 요청에서는 추가 시뮬레이션을 돌리지 않았다.

```bash
cd /root/farmily_tomato
git status --short
git log -4 --oneline
```

현재 기본 물리로 작은 dataset 실행(전체 후보 동작이므로 짧게 끝난다고 보장하지 않음):

```bash
./nvidia-sim/run_gpu_candidate_dataset.sh \
  --num-envs 4 --candidates 8 --schedule continuous \
  --physics-hz 960 --elastic-joint-armature 0.00001 --planning-workers 8
```

짧은 연결 검사만 하려면 `--max-control-steps 16`을 추가한다.
이때 중단된 후보는 `incomplete`이며 성공/실패 학습 데이터로 쓰지 않는다.
GUI 1환경 확인은 위 명령을 `--num-envs 1 --candidates 1 --gui`로 바꾸고 `DISPLAY=:0`을 사용한다.
새 계정에서 재현하기 전에 `--help`로 현재 옵션을 확인한다.

동일 fixture의 240Hz / armature 1e-4 진단 재현(**동등성 검증에 실패한 실험 설정**):

```bash
task_probe_dir="nvidia-sim/rl/runs/$(date +%Y%m%d_%H%M%S)_elastic_240_probe"
/root/isaaclab_env/bin/python nvidia-sim/rl/gpu_probe_worker.py \
  --headless --mode gpu --solver pgs --batched-io --native-replication \
  --gpu-partitions 1 --num-envs 1 --physics-hz 240 --joint-armature 0.0001 \
  --fixture nvidia-sim/rl/runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003 \
  --output "$task_probe_dir"
```

같은 설정으로 4환경을 검사하려면 `--num-envs 4`와 새로운 output 폴더를 사용한다.
동일 GPU에서 성능 시험을 동시에 실행하지 않는다. 초기화 시간과 motion wall time을 분리해 비교한다.

관련 테스트 재실행:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=nvidia-sim/rl \
/root/isaaclab_env/bin/python -m pytest -q \
  nvidia-sim/rl/test_gpu_planning.py \
  nvidia-sim/rl/test_gpu_step_profile.py \
  nvidia-sim/rl/test_gpu_throughput_benchmark.py \
  nvidia-sim/rl/test_gpu_replication.py \
  nvidia-sim/rl/test_gpu_scale_validation.py \
  nvidia-sim/rl/test_gpu_prim_lookup.py \
  nvidia-sim/rl/test_gpu_initialization.py \
  nvidia-sim/rl/test_gpu_batch_views.py \
  nvidia-sim/rl/test_gpu_dataset_runner.py \
  nvidia-sim/rl/test_gpu_validation.py \
  nvidia-sim/rl/test_gpu_probe.py \
  nvidia-sim/rl/test_dataset_pool.py
```

## 9. 다음에 검토할 작업 (미실시)

1. 후속 커밋 diff와 실험 기록을 검토하고 보존한다. 원래 preload/reset을 실험 실패 버전으로 되돌리지 않는다.
2. 보조 관성 1e-3/960Hz에서 접촉 전 파손되는 이유를 파손 연결부의 하중/초기화부터 분리 진단한다.
   원인이 엔진 버그라고 확정되지는 않았다.
3. 주줄기·꼭지·과실의 결합 및 하중 전달을 대상으로 작은 정지/토크/접촉 시험을 만든다.
   주파수를 바꿔도 변형·파손 응답이 수렴하는지 검사한다.
4. 실측 식물 변형 자료가 확보되면 모델의 응답을 보정한다. armature를 키우는 것만으로 sim-to-real을 보증하지 않는다.
5. 유효한 저주파 설정을 확보한 뒤 서로 다른 후보, 성공 trajectory, 타겟 1~11로 검증을 넓힌다.
6. 그 다음 대규모 환경 수에서 초기화/경로/physics/read-write/판정 시간을 다시 측정한다.

사용자가 요구한 충돌/탄성/파손 실패 사례를 없애거나 파손 임계값을 올려서 속도 시험을 통과시키지 않는다.
물리 정확도를 바꾸는 설정과 결과를 유지하는 코드 최적화는 기록에서 구분한다.

## 10. 이전 작업 문서 찾아보기

- [전체 시뮬레이터 README](nvidia-sim/README.md)
- [GPU 데이터셋](nvidia-sim/rl/GPU_CANDIDATE_DATASET.md)
- [GPU throughput](nvidia-sim/rl/GPU_THROUGHPUT.md)
- [GPU 병목 조사](nvidia-sim/rl/GPU_BOTTLENECK.md)
- [GPU 최적화](nvidia-sim/rl/GPU_OPTIMIZATION.md)
- [GPU probe](nvidia-sim/rl/GPU_PROBE.md)
- [후보 데이터셋/관측](nvidia-sim/rl/CANDIDATE_DATASET.md)
- [이전 CPU 다중 프로세스 pool](nvidia-sim/rl/CANDIDATE_DATASET_POOL.md)
- [탄성 식물](nvidia-sim/rl/ELASTIC.md)
- [고리걸기](nvidia-sim/rl/HOOKING.md)
- [pose 탐색](nvidia-sim/rl/POSE_SEARCH.md)
- [RGB 관찰/밀림 감지](nvidia-sim/rl/RGB_GUARD.md)

옛 문서의 CPU/TGS/worker pool 설정을 현재 GPU runner의 설정으로 착각하지 않는다.
충돌하는 수치는 최신 코드, 이번 검증 JSON, 마지막 두 성능/안정성 문서를 우선 확인한다.

## 새 대화에 붙일 문장

> `/root/farmily_tomato/HANDOFF.md`를 읽고 현재 Git diff와 연결된 최신 검증 자료를 확인한 뒤 이어서 작업해줘.
> 성능 개선 기준 커밋은 `1305651`이고, 저주파 안정화 진단과 이 문서는 후속 커밋에 포함했어.
> GPU 멀티환경과 CPU 경로 병렬화는 구현되어 있지만, 물리 결과를 유지하는 저주파 설정은 아직 검증하지 못했어.
> 그 이후 실용적인 물리 동작을 우선하는 기준으로 바뀌었고, 최신 기본값은 practical60이야.
> 이 문서 맨 위의 최신 후속 작업과 GPU_PRACTICAL_PHYSICS.md를 먼저 확인해줘.

## 2026-09-22 로봇 포함 MuJoCo CPU 기준선

- 현재 사용자 작업 브랜치 `mujoco-sim`에서 로봇 USD 관절/질량/관성/충돌 형상을 MuJoCo에 추가.
- `mujoco-benchmark/scripts/build_robot_model.py`, `robot_engine.py`, `robot_viewer.py`, `benchmark_robot_pool.py`.
- 기준 모델은 기존 최적화 식물, 로봇 7축, 전체 241 DOF. 고리는 동적 로봇 팔 끝에 고정. 하우스 없음.
- CPU 1/2/4/8/16/24/32/48/64 프로세스, 동일 49번 16.5초 기록을 각각 3회 실행.
- 48개 평균 6.443초/배치, 처리량 122.97 sim-s/s. 64개는 비슷한 처리량. 최대 검증64, 권장48.
- 모든 반복 발산0. 전체 영상/기하 검사 `outputs/20260922_robot_validation_final`, 순기구학 위치 오차 3.3e-8m, 9테스트 통과.
- 원본 시간/자원/코드 해시는 `outputs/20260922_robot_cpu_scaling{,64}`; 커밋용 요약은 `validation/robot_cpu_scaling.json`.
- 사용자는 CPU 기준선 커밋 후 새로운 브랜치에서 mjlab/MJWarp를 구축해 같은 조건의 GPU 성능 비교를 요청했다.

## mjlab GPU 비교 완료

- CPU 기준선 `38bac4d` 이후 `mjlab-performance` 브랜치 생성. 별도 `.mjlab-venv` 설치, 기존 `.venv` 보존.
- mjlab 1.6.0 / MuJoCo·MJWarp 3.11 / Warp 1.17 / Torch 2.7 cu128. `setup_mjlab.sh`, lock 파일 포함.
- 동일 모델 1/8/32/128/256/512/1024환경, 각 16.5초 × 3회. CUDA 제어·물리·평가 시간과 자원 시계열 저장.
- 가장 효율적인 GPU 설정 256: 평균 32.030초, 131.88 sim-s/s. 1024는 평균 179.960초, 93.89 sim-s/s.
- CPU 3.13/48프로세스 122.97 sim-s/s; 버전 맞춘 CPU3.11/24프로세스 75.62 sim-s/s.
- 모든 반복 비정상/overflow 0. GPU world0와 CPU 전체 스텝의 위치 오차는 validation/mjlab_robot_equivalence.json.
- 상세/한계/명령: mujoco-benchmark/MJLAB_PERFORMANCE.md. 비교 화면은 outputs/20260922_robot_backend_comparison/index.html.
- 파단은 비활성화, 동일49번 반복 성능 시험. 다양한 후보 탐색이나 RL 학습/수확 성공 평가기는 아님. mjlab Simulation 계층을 사용했으며 ManagerBasedRlEnv는 아직 추가하지 않음.

## 2026-09-22 CPU/GPU 운영 권장 정리

- GPU 구성 및 비교 구현 커밋: `115560b` (`mjlab-performance`). CPU 기준선: `38bac4d`.
- 대량 탐색 후속 구현 기본값은 GPU 256환경 권장. 기존 CPU 3.13/48 대비 처리량 약 7% 우세이며, 소수 경로 디버깅은 CPU 권장.
- 아직 동일 경로 재생 벤치마크이므로 다양한 후보 생성/저장까지 포함한 완료 시간은 미검증.
- autoreset 호환 변경은 별도 모델의 disable → enable 변경이며, MJWarp의 미지원 기능을 구현한 것이 아님. 원본과 오류 처리 의미가 같다고 설명하지 않는다.
- 상세 근거와 다음 단계: [MJLAB_PERFORMANCE.md](mujoco-benchmark/MJLAB_PERFORMANCE.md)의 운영 권장 절.

## MuJoCo 다양한 경로 생성 연결

- `mujoco-benchmark/scripts/candidate_experiment.py`: 기존 Isaac Sobol staged6d/IK/FCL 계획 → MuJoCo CPU 작업자별 서로 다른 경로 실행.
- 매 스텝 상태 저장, HTML/CSV, `replay_candidate.py`로 원본 상태 재생.
- 단순 벤치마크와 별도 실행기. 상세 명령/판정 한계: [CANDIDATE_SEARCH.md](mujoco-benchmark/CANDIDATE_SEARCH.md).
- 현재 부분 중심 진입/과도 변위/물리 이상 진단이며 꼭지 걸림 성공 평가기는 아직 아님.

- 경로 탐색 기본 출력 위치 변경: `/root/docker_share/mujoko_debugging_data/YYYYMMDD_HHMMSS_candidate_search/`. `candidate_experiment.py --output`으로 개별 지정 가능.

## D435 RGB-D 촬영

- `mujoco-benchmark/scripts/d435_capture.py`: 기존 ROS/Isaac URDF 광학 프레임 체인과69.4° FOV 사용, 저장된 후보 상태에서RGB/native depth/color-aligned depth와 행렬 저장.
- 사용법과 실물 센서 모사 한계: `mujoco-benchmark/D435_RGBD.md`.
- 후보87 preapproach/entry 샘플은 `/root/docker_share/mujoko_debugging_data/20260922_111838_candidate_search/candidates/candidate_00087/rgbd/`.

## Tomato_05 시점 변경 관측 준비 완료

- `prepare_observations.py`로 초기 상태 고정, 가상 D435 이동·회전, 자동 타깃 중앙정렬 없이71개RGB-D/GT목표표시/3배·6배크롭 촬영.
- 최종 데이터: `/root/docker_share/mujoko_debugging_data/20260922_122302_tomato05_observations/` (`index.html` 갤러리). 약192MiB.
- 기존 `20260922_111838_candidate_search`의1,000개경로/결과와 연결, per-view camera좌표라벨 제공. 당시 관측 생성 단계에는 새 물리시험0회. 이후 재실험·학습은 아래 기록 참조.
- `validate_observations.py`의71,000관측-후보pair좌표검증 및RGB/Depth/mask/crop검증통과. 대상gt마스크ID보간을피하기위해촬영MSAA비활성.
- 가상카메라 변환은 실제관절만으로복구불가. `camera.json`의유효tool_from_optical/world_from_optical 사용.
- 실제로봇도달성/새토마토일반화는검증하지않음. 상세: `mujoco-benchmark/OBSERVATION_DATASET.md`.

## 카메라 기준 action v2 및 1,000개 재실험 (2026-09-22)

- `camera_action.py`: 카메라 광학축 기준 진입 방향/고리 quaternion/거리/상승 방향의14차원 입력. 절대 시작 위치는 제외, 정확한 재연용 토마토 중심 상대 경유점은 별도 보존. 중력 방향도 관측별 저장.
- `candidate_experiment.py` → `action_frame.json`, `plan.json`/`result.json:action_camera`. 기존6개변수/월드경로 보존. 보고서 방위각은 명시적으로 월드 기준이라고 표시.
- 새 실험 `/root/docker_share/mujoko_debugging_data/20260922_124028_camera_action_v2_1000/`: CPU48/계획16,120Hz,seed0,1,000개 완료, 총510.83초. 부분중심진입36/과도변위908/miss56. 중심진입 자체는200개(과도변위 포함). 오류0. 이전111838실험과모든분류/최대변위동일.
- 기존71개관측은 `upgrade_observation_actions.py --run NEW_RUN`으로 새결과에연결. `actions_camera_v2.npz:features`를학습에사용하고 `candidate_ids`로`actions.json`과join. 원래초기카메라의단일action_camera를다른관측에그대로사용하면안됨.
- 사진과물리장면은보존. 모델해시/초기qpos동일확인. 71,000쌍좌표검증최대위치오차3.33e-16m. 카메라병진에대한불변성/회전에대한월드복원검증통과. v1원본메타는schema_v1_backup보존.
- 이 저장 형식 변경 이후 학습 완료(아래 참조). 꼭지걸림성공평가기는미구현. 동일Tomato05의시점변경만있으므로영상없는기준모델과비교필요.

## Tomato_05 카메라 기준 후보 학습 예비 비교 (2026-09-22)

- `mujoco-benchmark/learning/train.py`: 고정 DINOv2/ResNet18의 local/context RGB 특징 + metric Depth/validity/K + 카메라 action14/중력3 → 부분 중심 진입 확률/최대 이동량. 영상 없는 대조군 포함 3종×3seed 실제 GPU 학습, 41.21초.
- 결과 HTML/가중치/CSV/분할/코드 사본: `/root/docker_share/mujoko_debugging_data/20260922_130308_camera_action_training/`.
- 후보 train699/val149/test152와 관측 train50/val7/test7/잘림검사7을 별도 분리. 새 시점+새 후보가 주 평가. 모델/정규화/체크포인트 선택은 train/val만 사용.
- 테스트 AP: 영상 없음.837 / ResNet.849 / DINO.855. 이동 MAE 2.575/2.535/2.618mm. 검증 손실은 영상 없는 모델이 최소라 공식 선택도 영상 없음. 영상 모델의 확실한 우위 또는 sim-to-real 성공을 주장하지 않음.
- `predict.py`로 저장 관측의 후보 순위 재계산 가능. 세 모델 checkpoint 재로드/분할 비중복 확인 완료. 새 실물 이미지 배포는 미구현. 추가 physics 검증은 다음 절 참조.
- 모델 조사 근거와 명령: `mujoco-benchmark/learning/README.md`. DINOv3는 공식 가중치 접근 신청 필요해 미학습. 단일 장면 반복 관측의 한계/영상 없는 기준과의 비교를 계속 유지할 것.

## 학습 추천의 실제 물리 검증

- `learning/test_physics.py` / `render_physics.py` 추가. 평가용7시점×3모델(seed0)의1순위추천21회를독립reset후MuJoCo120Hz새실행. 중복제외3경로.
- `/root/docker_share/mujoko_debugging_data/20260922_model_recommendation_physics/index.html` 결과/상태/대표3개2배속영상. 물리41.52초(렌더제외).
- ResNet/DINO각진입7/7·20mm이하7/7. 영상없는모델진입7/7·20mm이하4/7. 초기상태동일/원본결과일치/불안정0확인.
- 기존카탈로그경로를추천해관절명령재실행. 새trajectory생성·새장면일반화·꼭지걸림성공실험은아님.

## Tomato01~11 다중 대상 수집 (05 제외)

- `candidate_experiment.py --target Tomato_XX`와 `RobotEngine(target=...)`로계획기하/과실진입판정/변위측정대상을선택. 기본05보존.
- `prepare_observations.py`는manifest대상으로GT과실마스크/라벨/action좌표자동연결. 가상카메라기준위치는05대비대상중심차만큼이동하고원래장착카메라pose와이동량을명시. 로봇실제관측자세검증은아님.
- `collect_tomatoes.py --targets 1,2,3,4,6,7,8,9,10,11 --candidates 1000 --workers 48 --planning-workers 16`으로대상순차물리시험→RGBD저장. 전체10,000개본수집은사용자가실행.
- 명령/출력/공간/카메라한계: `mujoco-benchmark/MULTI_TOMATO_COLLECTION.md`.
- 다중대상시상태재연카메라/HTML대상표시도연결. 기존데이터변경없음.

## 후처리 최적화와 재개 기능 (최신 추가)

- `camera_action.py`: 고정 기하 일괄 준비 / 카메라 변환·검증 벡터화. 관측별 압축·검증은 기본8 CPU 스레드. `prepare_observations.py`의 v1 중복 검증 제거, 최종v2 모든검사유지.
- 71,000쌍 후처리 131.81초→1.41초. 71장 촬영 포함65.61초(장면 초기화 제외). 기존RGB/Depth/mask각71파일완전일치, action오차1e-12이내. 물리 약8분은변경없음.
- `collect_tomatoes.py --resume --output 기존폴더` + `resume_candidates.py` 추가. 개별결과/상태/명령해시검사후누락만실행. 기존계획/완성관측재사용, Ctrl+C시단계프로세스그룹정리.
- 실제중단수집 `20260922_134834_multi_tomato_collection`: 01~04완료,06은정상931개/남음69개검사확인. 사용자대신본수집을다시시작하지않음. 재개명령은 MULTI_TOMATO_COLLECTION.md 최신절.
- 성능검증복사본 `20260922_144727_postprocess_benchmark/` 보존. 중단복구/원래물리결과일치/손상데이터검출확인.


## 2026-09-22 진입 탐색 범위 변경

새 staged6d 후보는 진입각 −45°~+45°, 하부 여유 0~10mm를 Sobol 샘플링한다. MuJoCo와 Isaac의 공통 생성기 `nvidia-sim/rl/trajectory_search.py`에 적용했다. 여유는 초기 열매 충돌 구의 아랫면과 고리 와이어 윗면 기준이다. 0mm는 기하학상 여유가 없는 조건이며 실제 접촉은 물리 엔진의 접촉 설정에도 영향을 받는다. Sobol은 끝값 조합을 반드시 포함하지 않는다. 기존 데이터/저장된 후보/재생은 변경하지 않았고, resume은 저장된 후보를 계속 사용한다. 같은 seed와 candidate_id라도 새 실행은 이전 범위와 다른 경로이므로 실행 폴더를 구분한다. 실제 새 범위 물리 실험은 아직 수행하지 않았다.

## 2026-09-22 Tomato_06 자유 동작 RL 파일럿

- `mujoco-benchmark/scripts/train_tomato_rl.py`: mjlab/MJWarp + RSL-RL PPO, 고정 Tomato_06 GT 장면에서 팔6관절+리프트7액션 학습. 회전 고정/직선 trajectory 제약 없음.
- 상세 실행·판정·제약: [TOMATO06_RL.md](mujoco-benchmark/TOMATO06_RL.md). 기존 데이터 수집/모델은 유지하고 RL 전용 모델에 자기충돌을 추가했다.
- 학습 완료 후 결정적 독립 평가 상태를 저장하며 `replay_tomato_rl.py`로 재생 가능. 짧은 검증은 학습 루프/물리 안정성 확인이며 수확 성공을 주장하지 않는다.

- 최종 16환경/20업데이트 검증: 학습28.88초, 준비·평가 포함45.93초(커널 캐시 사용). 10,240 전이, 학습32에피소드 및 독립평가 모두 부분 진입 성공0. 성공 정책이 아니라 실행 루프 확인 완료. 가중치 업데이트/reset/체크포인트 로드/재생 검증 통과. `mujoco-benchmark/validation/tomato06_rl_smoke.json`에 결과 보존.

## 2026-09-22 GPU 환경 확대 및 추가 학습 실행

기존 사용자 실행 `20260922_184128_tomato06_rl`의 200회 체크포인트에서 추가 학습을 시작했다. 짧은 동일 조건 비교(3초 에피소드, 32스텝, 4업데이트): 64환경 1,150전이/초, 256환경 2,352전이/초. 256환경에서 관측한 GPU 사용률 95%, 메모리4,414MiB(평균/최댓값 아님). 기존16환경 357전이/초는 30초 에피소드의 장기 측정이라 엄밀히 동일 조건 비교는 아니다.

- 실행 중: `/root/docker_share/mujoko_debugging_data/20260922_185257_tomato06_rl_gpu256`
- PID 286916 (종료/상태 확인 전 프로세스 명령을 다시 확인할 것)
- 환경256, 추가1,000업데이트, 32스텝, 에피소드30초. 총8,192,000전이. 기존 물리/보상 유지.
- 로그: 같은 경로 뒤 `.log`; 실행 인자/PID는 `.launcher.json`. 25업데이트마다 체크포인트, 완료 후 summary/평가 재생 데이터 저장.
- progress.jsonl에 처리량·누적 학습시간·잔여 예상분 추가. 약1시간 예상이나 접촉 복잡도에 따라 달라짐. 이 기록 작성 시 본 학습은 진행 중이며 성공 여부 미확인.
- 측정 보존: `mujoco-benchmark/validation/tomato06_rl_scaling.json`.

## 10분 학습 비교로 전환

사용자 요청으로 기존 1,000회 추가학습(PID286916)은 중단. 해당 실행의 model_00025.pt를 보존했으며 이후 미저장 업데이트는 보존되지 않았다. 비교의 출발점은 원래 사용자 200회 체크포인트다.

`--train-seconds 600 --compare-before --iterations 100000`으로 학습 구간만 600초 제한, 진행 중인 업데이트 종료까지 몇 초 초과 가능. 준비/기준평가/최종평가는 별도. 실제 완료 업데이트와 전이 수 기록, 마지막 체크포인트 저장. 2초 제한 검증에서 2.76초/2업데이트 후 자동 종료·저장·전후 비교 통과.

현재 실행: /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min

PID 291847; 로그: /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min.log

완료 후 summary.json의 comparison에 전후 성공수, 평균 최근접 거리, 평균 최대 열매 이동량 저장. baseline_world0.npz와 evaluation_world0.npz 저장. 실제 향상 여부는 완료 후 확인해야 한다.

## 10분 추가 학습 완료 및 진단

`20260922_185721_tomato06_rl_10min` 완료: 학습600.43초, 전체735.75초, 170업데이트/1,392,640전이. 학습2,306에피소드 성공0, 결정적 평가 전후 성공0. 평균 최근접 중심 거리56.16→43.17mm, 평균 최대 열매 이동량1.13→1.13mm. 결과 요약은 `mujoco-benchmark/validation/tomato06_rl_10min.json`에 보존.

평가 궤적은 약5초에 접근한 뒤30초까지 비슷한 위치에 머문다. 현재 근접 항은 정지 상태에서도 시간 비용 차감 후 약+0.024/제어스텝(다른 감점 전)을 지급한다. 성공 경험이 없는 정책에서 머무르기 보상과 밀림 실패 페널티가 정체를 유도했을 가능성이 크다. 성공 조건은 고리 평면2mm 이내/0.2초 유지이며 좁다. 다음 작업 후보는 정지 근접 보상 제거와 실제 정렬/삽입 진전 보상 설계, 물리적 진입 가능성 점검이다. **보상 수정은 아직 하지 않았다.**

이전 문단의 '실행 중'은 과거 기록이다. 1,000회 실행은 사용자 요청으로 중단했고 10분 비교 실행은 정상 완료했다. 학습 모델/대형 상태 기록은 docker_share에 유지하며 Git에는 코드·문서·작은 검증 요약만 포함한다.

## 최신 변경: 보상 progress_v2 (학습 실행 안 함)

사용자 요청으로 보상만 변경. 정지 근접 보상과 반복 내부 체류 보상을 제거했다. 기존 목표점 거리 감소20배 + 고리 좌표계의 포획 조건 위반 거리 감소20배로 진전을 보상한다. 포획 오차는 반원 내부/고리 평면/와이어 여유의 위반량이며 특정 세계 방향이나 경로를 고정하지 않는다. 최초 안전 부분 진입은 에피소드당1회 +2, 기존0.2초 유지 성공 +10. 기존 밀림/접촉/급격한 액션/시간 감점과 실패 -5 유지. 후퇴는 signed progress 감점이지만 물리적으로 금지하지 않는다.

물리·관측·행동·종료·성공 기준·학습기는 변경하지 않았다. manifest의 reward_version=progress_v2로 구분한다. 정지/진전/후퇴/왕복/최초 진입/포획 오차 산술 검증 통과. 수정 보상의 실제 학습 개선 여부는 아직 미검증. 학습은 사용자가 직접 실행한다.

2시간 추가 학습(이전10분 모델에서 시작):

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.mjlab-venv/bin/python -u \
  mujoco-benchmark/scripts/train_tomato_rl.py \
  --num-envs 256 --iterations 100000 \
  --steps-per-env 32 --episode-seconds 30 \
  --train-seconds 7200 --compare-before \
  --checkpoint /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min/model_00170.pt
```

학습 구간2시간이며 업데이트 경계까지 수 초 초과 가능, 준비·전후 평가는 별도다. 새 datetime 폴더로 저장. 기존 보상과 점수 크기가 달라졌으므로 보상 합보다는 독립 평가 성공/거리/변위를 비교한다.

## 최신 데이터 생성 조건: 각도만 탐색

새 staged6d/Sobol 후보는 1차원 scrambled Sobol로 진입 방위각 −45°~+45°만 바꾼다. GT 중심에 인식 오차 없음. 좌우 오프셋0mm, 하부 여유2mm 고정. 삽입42.5mm/수직 상승32.5mm는 이전 범위 중간값으로 고정, pre-hook170mm/roll0/elevation0 유지. 이 조건은 중심선을 향하는 계획이며 접촉/변형/제어 오차까지 제거하거나 실제 진입 성공100%를 보장하지 않는다. 삽입 깊이도 모든 후보에서 동일하고 열매 중심 도달을 자동 보장하지 않는다.

공통 trajectory_search 생성기와 새 실행 metadata에 sampling=scrambled_sobol_azimuth_only 기록. 파일/CLI trajectory_mode=staged6d 명칭은 호환성을 위해 유지. 기존 저장 후보/재생/계속 실행(resume)은 이전 경로를 유지하므로 새 조건 수집은 새 실행 폴더로 시작해야 한다. 이전 seed/candidate 번호와 새 경로는 다르다. RL 제어에는 적용하지 않는다.

생성/기하 검사5개 통과(1,000후보에서 각도만 다름, 좌우 중심 정렬, 와이어 여유2mm 확인). 물리 실행/데이터 수집은 실행하지 않았다.

## 두 구간 45° 상승 적용

새 후보에 `lift_profile=diagonal_45_return`을 저장한다. 진입 방향 기준 전진16.25mm+상승16.25mm 후, 후퇴16.25mm+상승16.25mm. 최종 위치는 이전 수직32.5mm 상승과 동일하며 고리 회전은 바꾸지 않는다. 경유점명은 `rise_mid`, 마지막은 `rise`; 실제 제어 단계는 두 구간 모두 rise로 처리하여 동일 상승 속도/접촉 판정을 적용한다. 이동 거리는 기존 상승의 √2배이므로 동일 속도에서는 상승 시간이 길어진다.

기존 저장 후보에 profile이 없으면 기존 상승을 유지한다. 새 방식은 waypoint5개를 카메라 좌표에도 모두 저장한다. 기존14개 compact features의 lift 방향/거리는 최종 순변위이며, 이14개만으로 직선 상승과 대각 상승을 구분할 수 없다. 두 방식을 학습용으로 섞으려면 phase_names/전체 waypoint 또는 별도 profile 입력을 사용해야 한다. 서로 다른 phase 구조를 한 batch로 섞으면 명시적으로 거부한다.

기하 검사6개 통과, 기존4점/새5점의 카메라 좌표 변환·복원 검증 통과.

## 중심 진입 단독 성공 기준

새 MuJoCo 데이터 수집의 classification_rule=center_entry_only_v2: 물리 오류는 invalid_physics, 정상 실행에서 중심 진입 이력이 있으면 열매 최대 밀림과 무관하게 partial_center_entry, 없으면 miss. target_center_max_displacement_m과 target_displacement_exceeded(20mm 초과)는 참고 지표로 별도 저장. 결과 JSON/CSV와 HTML의 판정 설명/밀림 집계를 갱신했다.

기존 데이터는 자동 덮어쓰지 않았다. 과거 manifest에 rule이 없으면 displacement_first_v1로 이어 실행하여 혼합 판정을 방지한다. 기존 ±90°/30개 데이터를 읽기 전용으로 재분류 검증하면 중심 진입 성공3개, 미진입27개다. RL 보상/종료 조건은 이번 변경 대상이 아니다.

## 다중 토마토 RGB-D 진입각 학습 완료

데이터10대상×30경로/710관측(05없음). 8대상 학습,09검증,06테스트로 분리. 진입 BCE만 학습하며 밀림 감점 제거. DINOv2 seed2 선택,06의64시점 모두−81.09° 후보00001 추천. 고유 경로1개 새 물리 실행에서 중심진입 성공(밀림30.78mm). action_only도동일 top1성공으로 영상 우위 미입증. 전체 후보 확률 calibration 불량(Brier약0.315). 상세/명령: [learning/MULTI_TOMATO.md](mujoco-benchmark/learning/MULTI_TOMATO.md). 결과 `/root/docker_share/mujoko_debugging_data/20260922_234403_multi_tomato_training/index.html`. 초기 요청 커밋3f636c2 이후 학습기 확장 작업.
## 2026-11-08 Newton 독립 실행 · 새 로컬 장면 검증

- 새 브랜치 `codex/s300-newton-standalone-20261108`, 출발 `f984e6ecf918fb939e073afa79a6f9f1f6f84dda`. 기존 미커밋 변경 없는 상태에서 시작했다. NAS 결과/영상 복구가 선행조건이 아니라는 사용자 정정에 따라 로컬 원본과 기존 빌더로 새 장면·명령을 만들었다. 잃어버린 물리 말단의 연속 재개가 아니다.
- 기본 실행 `nvidia-sim/run_s300_newton.sh standalone`, 브라우저 `http://192.168.222.27:8091`. Newton GPU 기존 실행기와 별도 Viser 실측 상태 소비자이며 Isaac/Kit 없이 실행한다. 첫 렌더와 시작 버튼 전에는 t=0; 완료 후 동일 GPU 모델/브라우저에서 초기화 후 재시험 가능. 기본 summary는 약0.5MB/실행, 전체 trace/영상은 필요조건이 아니다.
- 자산은 `/root/farmily_runtime/newton/assets/`, 실행은 `runs/`. 기본 `assets/current`는 `harvest_floor_20261108`을 가리킨다. 원본 농장/식물 GLB·USD와 RB5/그리퍼/측면 리프트 빌더를 재사용했다. 원래 S300 snapshot을 대신해 체크인된 조립 GLB의 정적 차체만 추출했으므로 과거 차체 부품별 충돌체와의 동일성은 미검증이다.
- 같은 새 입력으로 headless/Viser 실제 GPU 실행44.0002초를 각각19.3158초(RTF2.278),24.6068초(RTF1.788)에 완주했다. GUI 카메라 이동 중에도6,800스텝 전체 qpos/qvel/pose/명령/접촉·힘/연결해제·솔버 반복이 정확히 같다. GUI 최저 단계는 belt_finish0.813RTF이므로 모든 구간 실시간 해결로 해석하지 않는다.
- 최초 source-box 바닥 입력에서 분리 과실3개가 바닥을 관통해 무한 낙하했다. 그 결과는 보존하고 정상 환경 판정에서 제외했다. 기존 top-plane 옵션으로 같은 바닥 윗면에 무한 충돌평면을 두고 시각적인 바닥은 보존했다. 다른 geom/물성/관절/명령/초기 상태 불변을 확인했다. 수정본의6,801개 저장 상태에서 과실 중심이 바닥 아래로 이탈하지 않았다. 원래 바닥과 완전히 같은 충돌 표현이라고 주장하지 않는다.
- 수정본5알 분리: close3/cycle_reclose1/approach1. 접근 중 우발 분리와 포집 미검증을 포함하므로 수확5알·송이 전체 완료로 세지 않는다. 활성 최대침투0.9204mm, finite/capacity PASS. 기존 coarse4/열매proxy,220/60Hz, 지정 belt stroke2mm/그 외 감시1mm 및 분리 과실 예외 유지. 타깃 송이·주줄기 중심의 충돌범위이고 주변 농장은 시각 표현이다.
- 수정본으로 브라우저 시작/반복 실제2회가 각각RTF0.964/0.966에 완주했다(1배속 대기 포함). 같은 viewer PID/브라우저, 두번째 model load0초·준비3.43초, 최종 상태와 분리 이벤트가 headless와 동일. 종료 버튼 정리도 확인했다.
- 오프라인 Newton/viewer/launcher46시험, 실제 측면장착+GLB복구9시험, 기존 바닥/농장adapter8시험 모두통과. 물리 패키지 Newton1.7/MuJoCo3.15/MJWarp3.15/Warp1.17 유지. Viser1.1.1을 위해 전송패키지websockets만15.0.1로 호환 고정했으며 설치 검사 통과.
- 아직 독립 뷰어의 RGB-D/GT·ROS2·카메라 기반 여러 송이 연속 수확은 연결하지 않았다. 기존 카메라 장착과 calibration은 자산에 보존했다. 단일 송이 새 물리 실행/표시/반복의 이식 검증 범위이며 `training_eligible=false`, `harvest_success=null`.
- 실행 안내: `nvidia-sim/S300_NEWTON.md` 첫 절. 작은 근거: `nvidia-sim/validation/s300_newton_standalone_20261108.json`. MP4를 새로 만들지 않았고 실시간 확인용 스크린샷은 `runs/newton_live_ready.png`, `runs/newton_live_finished.png`다.

## 2026-11-08 — Newton mounted-camera entrance patrol

- New launcher: `./nvidia-sim/run_s300_newton.sh camera-patrol` (Viser 5905, waits for browser Start) or `--headless`; native GPU physics and Newton GPU RGB-D/SIM-GT camera, no Isaac/Kit. Starts at rail entrance 0m, left bed, 15cm increments (max20cm), default validated camera 240x320. Reuses existing CameraPatrol/CameraRegionSelector, plan_adapter, GPU runner and strict measured window continuation.
- Detailed commands, scope and measured outcomes: `nvidia-sim/S300_NEWTON_CAMERA_PATROL_KO.md`. Local sources/results are under `/root/farmily_runtime/newton`; deleted NAS recordings are not dependencies.
- First valid cycle: approach/harvest/withdraw completed, 3 releases, max penetration0.905mm. Release phases include preparation and withdrawal; this is not three verified belt-only collections. Fresh-process versus retained-engine movement and GUI versus headless harvest traces matched every saved field exactly.
- Repeated second attempt reached a return-path collision (Tool_R versus another truss peduncle,1.456mm) and stopped; it is not counted as a completed harvest. Physical failure never triggers a reset-and-continue disguised as continuous harvesting. Failed planning has finite recorded alternatives/probes.
- Prepared plant assets retain every truss of each plant. Active plant flexible, other prepared plants static obstacles, released fruit dynamic; outside prepared route plants visual only. This is more work than the old single-truss133-DOF benchmark. End-to-end patrol is NOT yet realtime: model preparation/verification/planning transitions still dominate.
- Existing uncommitted standalone implementation and old results preserved. Newly generated identical MJB/XML are SHA-verified and shared inside each new run. `training_eligible=false`, `harvest_success=null` remain.

## 2026-11-08 — user-authorized main-stem-only geometric harvest

- Latest user rule overrides the earlier plant-contact policy: fruit and all thin plant/truss geometry have zero collision masks; only thick `Stem_MainStem_*` segments remain plant obstacles. Robot self/structure collisions remain. After actual goal arrival, every fruit center in the existing open-jaw 96×158×48mm region is assumed harvested, regardless of color. Physical attachment breakage is disabled in this explicit mode; no physical harvest claim.
- `newton_geometric_harvest.py` derives immutable prepared assets, retains the existing GPU runner/state continuation/planner, and records measured capture receipts. `prepare-patrol` now defaults to this mode; `prepare-geometric-patrol SOURCE_ASSETS NEW_ASSETS` converts existing local source assets. Old physical assets and results remain usable explicitly.
- Default `assets/patrol_current` now points to `patrol_mainstem_ideal_v3`. Native model geometry/material/joint/inertia arrays remain unchanged except the exact collider masks and their body caches (477 unchanged arrays checked). 657 fruit/thin geoms disabled, 48 main-stem geoms retained. Prepared route0..0.85m, plants17/18/19,9trusses90fruit; original full-farm visual scene retained.
- Fresh camera patrol `runs/patrol_mainstem_ideal_v6`: 8 actual GPU segments completed, movement0→.15→.30→.45→.50m, 3 reached insertions, geometric credit2+3+4=9unique fruit (8red-proxy,1other-color). All9 removed from subsequent camera instances and selection. Actual physical releases0. All active penetration logs0. Plans blocked by main stems/self collision remain recorded; wider observed yaw proposals0/±25/±50 and existing minimum15mm entry depth found valid paths.
- Current goal tracking errors0.0346/0.0382/0.0306mm. Source initial state preserved. No reset between decisions; accumulated logical credit is separate from the physical release ledger. Native70 + camera/group-policy43 regression tests passed.
- Small checked-in evidence: `nvidia-sim/validation/s300_mainstem_geometric_harvest_20261108.json`; detailed Korean scope/results in `nvidia-sim/S300_NEWTON_CAMERA_PATROL_KO.md`. Run index and first-insertion MP4 are under the run directory. Video is actual recorded-state replay at1x, not a fresh physics execution; small captions.
- Whole patrol still has preparation/planning pauses (82.26sim seconds /475.28wall seconds in this run); it is not an end-to-end realtime result. Test ended at the3-insertion budget, not full-farm/truss completion. Prior failure/probe outputs preserved. Existing uncommitted work remains intact.
# 2026-11-08: Newton 카메라 순회의 반복 대기 시간 개선

- 현재 기본 `patrol_current`는 `patrol_mainstem_static_v1`. 사용자 지정 영역 수확 모드에 맞춰 준비된 모든 식물을 고정하고 로봇 10축만 Newton CUDA로 적분한다. 두꺼운 주줄기/자기충돌/구조물은 유지한다. 과거 물리 수확 모드의 기본 동작을 변경하지 않았다.
- 모델 1회 컴파일/상주 GPU 엔진, 동일 MJB measured resume, 기존 USD/IK 계획기 상주 실행, 변경을 거절하는 순회 범위 SHA 재사용, 큰 불변 메타데이터 공유, 동일 순서의 빠른 descendant 배열 생성, 구간당 3개 제안/다음 관측의 방향 변경을 적용했다. `--proposals-per-view 15`로 기존 전체 관측 후보 수 사용 가능.
- 최종 headless: 실제 78.71초 동작/3회 진입/9개 가상 수확을 74.46초에 완료(초기 준비 15.26초 별도). 이전 475.28초 대비 작업 시간 6.38배 개선. 물리 루프 RTF 3.66. 원래와 식물 동역학/계획 순서가 다른 실험이며 동일 모델 솔버 가속이 아니다.
- 확장 순회: 0..0.85m, 두 식물에서 5회 진입/15개 영역 수확. headless 275.49초, GUI 281.12초. 후반 자기충돌/IK 검사 때문에 전체 순회는 아직 실시간이 아니다. 제한된 경로 탐색 실패를 불가능으로 해석하지 않는다.
- GUI 카메라 회전 중에도 13개 물리 구간의 모든 저장 상태/최종 warmstart·damping·제어/명령이 headless와 정확히 일치. 최종 live-publisher OFF headless 첫 7개 구간도 GUI와 정확히 일치. 활성 관통 0mm/물리 중단 0/물리적 과실 분리 0. 실제 수확 분리가 아닌 영역 수확 원장이다.
- 보고서: `nvidia-sim/S300_NEWTON_CAMERA_PATROL_KO.md`, 증거: `nvidia-sim/validation/s300_mainstem_patrol_speed_20261108.json`. 결과/영상: `/root/farmily_runtime/newton/runs/patrol_static_speed_gui/index.html`. `render-patrol RUN --output VIDEO.mp4`는 기존 GPU 센서로 저장된 실제 상태를 영상화하며 물리를 재실행하지 않는다.
