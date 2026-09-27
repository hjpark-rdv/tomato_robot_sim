# 다중 경로군 서버 검증 자료

[결과 보고서](../../MULTIFAMILY_SEARCH_SERVER_RESULTS_KO.md) · [Pro 전달문](../../MULTIFAMILY_SEARCH_PRO_HANDOFF_KO.md)

6개 생성 장면 중4개 통과,12타깃의 원래240제안과 측면 수정48제안을 비교했다. 새 경로는 모두 계획 또는 환경 검사에서 막혀 실제 물리 실행0이다. 기존 대조 경로2회는 실제 완주했지만 접촉 유지 증거는 없다.

## 바로 읽을 자료

- `pilot_summary.json`, `side_repair_summary.json`: 모든 후보의 단계별 결과.
- `*_blockers.json`: 실제 native body 소속, 최초 충돌 phase/쌍, 허용32개 와이어 여부.
- `prefix_comparison.json`: 네 경로군의 공유 prefix48그룹, 분기 이전 차단23그룹.
- `side_repair_comparison.json`: 동일48개 파라미터 쌍의 원래/수정 결과와 경유점.
- `*_fk.json`: 저장 관절명령으로 계산한 FK 위치·회전과 오차.
- `scene_inventory.json`, `inputs/`: 초기 탈락2장면과 미선택 열매를 포함한 전체 분모·배치.
- `executor_control_summary.json`, `rachis_contact_control_summary.json`, `executor_regression.json`: 실제 물리 대조군 및 이전 qpos 동일성.
- `azimuth_baseline_summary.json`: 같은 정책의 기존 azimuth 비교(차단).
- `native_tests_repair.log`, `planning_tests_repair.log`: 246+5개 회귀.
- `*.resources.json`, `*_launch.json`: 자원 표본과 실제 실행 명령/예산.
- `delivery_verification.json`: 완료·source 불변·예산·영속 모델 링크·MP4 전체 디코딩 검사.
- `SHA256.json`: 이 디렉터리 파일별 SHA256/크기.

## 압축 원자료

`all_candidate_evidence.tar.gz`에는 모든 후보의 plan/trace/전체 환경 검사와 접촉 범위, 실행 대조군의 실제240Hz qpos/접촉/기하 이력, 소스 코드 스냅샷과 실행 로그를 넣었다. 최초 시간초과 진단도 별도 smoke 폴더에 보존한다. 잘 나온 후보만 선별한 압축 파일이 아니다.

명목 trace는 로봇 명령이며 실제 식물의 움직임이 아니다. `trial_states.npz`만 실제 물리 qpos다. 대형 MJB/메시/계획 pickle은 포함하지 않는다. 서버 `/root/docker_share/mujoko_debugging_data/20260928_multifamily_server`와 NAS `/mnt/nas_rdv_md3/tomato_robot_sim_20260928_multifamily`에 원본이 있다.

## 대표 영상

실제 저장 상태1개와 물리 미실행 명목 계획7개다. 각 MP4 옆 JSON에 원본/상태/종류가 있고 PNG로 프레임도 보존했다. 명목 영상은 충돌 뒤에도 계획을 보여주지만 실제로 진행했다는 뜻이 아니다.

- **recorded_physics** — [actual_executor_control](videos/smoke_videos/actual_executor_control.mp4)
- **nominal_only** — [nominal_side_mouth_blocked](videos/smoke_videos/nominal_side_mouth_blocked.mp4)
- **nominal_only** — [nominal_flank_left_blocked](videos/smoke_videos/nominal_flank_left_blocked.mp4)
- **nominal_only** — [nominal_flank_left_seat_blocked](videos/pilot_supplement_videos/nominal_flank_left_seat_blocked.mp4)
- **nominal_only** — [nominal_flank_right_blocked](videos/pilot_supplement_videos/nominal_flank_right_blocked.mp4)
- **nominal_only** — [nominal_pivot_sweep_blocked](videos/pilot_supplement_videos/nominal_pivot_sweep_blocked.mp4)
- **nominal_only** — [nominal_under_center_blocked](videos/pilot_supplement_videos/nominal_under_center_blocked.mp4)
- **nominal_only** — [nominal_side_robot_facing_blocked](videos/pilot_supplement_videos/nominal_side_robot_facing_blocked.mp4)

## 재현 범위

Python 런처는 새로운 파이프라인이 아니라 기존 실행기 호출·측정과 읽기 전용 집계용이다. 절대 경로를 사용하므로 다른 서버에서는 자산/경로를 먼저 맞춰야 한다. 완료된 결과 폴더를 덮어쓰는 방식으로 재실행하지 않는다. `launch_side_repair.py`는 원래 파일럿 완료와 남은 예산을 검사한다.

`training_eligible=false`, `hook_success=null` 유지. 접촉 기준을 완화하거나 기존 action14 학습 자료에 합치지 않는다.

직접 열람용 results.csv는 Git의 줄끝 검사에 맞춰 LF로 정규화했다. CSV 내용은 동일하며 원본 바이트는 압축 원자료에 보존했다.
