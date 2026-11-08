# 실제 중단 지점에서 원본 수확 사이클의 나머지만 실행

이 기능은 `--mode recovery --resume-original-cycle-suffix`의 명시적 진단 옵션이다. 이전 정상 사이클로 돌아가거나 새 접근을 만들지 않는다. 기존 실행기·네이티브 상태 복원·자체충돌 검사·환경 검사를 그대로 사용하며 기본 실행은 바꾸지 않는다.

## 이번에 승인 가능한 원본

- 자산: `20261023_164703_maximum_camera_harvest/gt_row_maximum_local_heading_v5_campaign/windows/000_plant_25`
- 실제 원본: `plant25_resume_threads1_native/plant25_resume_yaw0_seed5_threads1_harvest`
- 반드시 사용할 상태: 위 실행의 **terminal `route_checkpoint`, 64.7666666667초**. 이전 완료 체크포인트 45.4초는 현재 상태가 아니다.
- 원본 `completed=false`, `penetration_over_contact_policy`와 원자료는 보존한다.
- 원본 마지막 위반은 이미 네이티브 분리된 Fruit04와 정적 `RowSupply_20mm`의 3개 접촉점이다. 기존에 활성화한 분리 과실/정적 농장 지지체 정책의 정확한 역할 목록 누락만 보완했다. 실제 충돌 응답, 1mm 기준, 질량, 형상, 마찰과 조인트 분리 조건은 그대로다.

`validate_terminal_forward_suffix`는 실제 저장 체크포인트의 모든 배열·컨트롤러·분리 이력, 마지막 실제 프레임, 원본 전체 경로 승인 해시, 원본 요약·상태·접촉 해시를 검증한다. 원본의 마지막 raw 1mm 초과 접촉이 전부 네이티브 분리 증명과 source SHA에 묶인 정적 RowSupply 진단으로 재분류되는 경우에만 허용한다. 초기화/실제 자체충돌 검사 실패, 다른 물체 접촉, 이른 체크포인트와 일반 물리 실패는 허용하지 않는다.

## 원래 명령 곡선을 보존하는 방법

실행 중이던 `cycles/002/plan.json`의 모든 phase 위치·시간·jaw·belt 명령을 보존한다. 사이클 시작은 45.4초, 실제 중단은 64.7666666667초이므로 원래 사이클 시계 19.3666666667초부터 남은 24.0333333333초만 실행한다. 이미 지난 준비·접근은 재실행하지 않는다.

닫기/열기 구간을 잘라 새 smoothstep으로 보간하면 기존 속도와 jaw 궤적이 달라진다. 따라서 원본 전체 phase를 유지하고 `(원본 마지막 physics tick + 새 local tick) / 60 - 원본 cycle 시작시각`으로 원래 연산 순서까지 재현한다. 마지막으로 보낸 명령 시각 64.75초와 그 뒤 실제 상태 시각 64.7666666667초를 구분한다. 재개 첫 명령은 원래 다음 명령 시각 64.7666666667초의 값이다. belt ramp 0.5초와 원본 on/off 상태도 고정한다.

초기 실제 q/qvel·body pose/velocity·비대칭 jaw·분리 이력·네이티브 drive target은 기존 상태 복원기로 복원한다. 초기 실제 충돌 검사는 그 실제 body pose를 사용하며 nominal 명령으로 대신하지 않는다. rail의 실제 좌표는 현재 실측값으로 검사하되 네이티브 rail hold target은 원본 그대로 유지한다. 새로운 PhysX 엔진이므로 solver/contact cache까지 이어지는 bitwise 재시작은 아니다.

## 오프라인 검증

결과 폴더: `/root/docker_share/mujoko_debugging_data/20261023_164703_maximum_camera_harvest/plant25_terminal_original_suffix_offline`

- 전용/기존 관련 회귀: 118/118 PASS.
- 실제 초기 비대칭 jaw를 포함한 자체충돌/환경 검사: 각각 PASS.
- 초기 FK와 실제 위치 차이 최대 2.521µm, 기존 0.1mm 수치 기준 유지.
- 남은 원본 경로 자체충돌 검사: 60Hz, 기존 네이티브 40개 부품 캐시, 1443표본 PASS.
- 같은 경로 전체 환경 검사: 1443표본, 거리 질의 28,072개, hard collision 0, complete/eligible PASS.
- 1443은 실행 명령 1442개와 최종 endpoint 검사 1개다. 실제 실행 예정 시간은 24.0333333333초다.

`validation.json`, `proof.json`, `plan.json`, `self_collision_preflight.json`, `full_environment_preflight.json`과 로그에 증거를 보존한다. 이는 **실제 재개 실행 전 오프라인 통과 결과**이며 실제 수확/후퇴 완료를 의미하지 않는다. 원본의 4개 신규 분리는 기존 이력이고 새 수확 시도나 새 분리로 다시 세지 않는다. `training_eligible=false`, `harvest_success=null`을 유지한다.
