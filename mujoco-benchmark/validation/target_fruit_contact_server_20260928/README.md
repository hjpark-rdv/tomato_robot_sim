# 2026-09-28 목표 과실 접촉 재시험 증거

[서버 결과 보고서](../../TARGET_FRUIT_CONTACT_SERVER_RESULTS_KO.md). 기준804e40c. SIM-GT 진단, training_eligible=false, hook_success=null.

- `results.json`: 5개 원본 + 1개 감속의 최종 표.
- `audit_seating_*`: 원본5개 전체 명목 검사, complete/status와 허용정책·입력 해시.
- `trial_seating_*`, `trial_slow_*`: 실행 직전 전체 재검사, 실제 240Hz 원자료, legacy/authorized 판정, 중단 결과, 실제 qpos 상태.
- `summary_*.json`: live/private 힘 합, 개별 법선력, 접촉시간, geom별 최대, 변위·침투·추종 및 timestamp 누락 검사. 초기/말기 sample 간격과 양의 힘 시간 합을 분리한다.
- `trial_states.npz`: 실제 초기/최대30fps/최종 qpos와 timestamps. 물리 재실행용 완전 상태 체크포인트가 아니라 영상 재생 자료다.
- `inputs/*/trace.json.gz`, `plan.json`: 사용한 명령과 plan 보존. 원본5개 변경 없음. 감속은 같은 선형 joint 경로를 보존.
- `retiming.json`, `verification.json`: 원래 명령 인덱스, source/retimed hashes, 경로 일치, 명목 TCP 속도, 원본 전체 SHA 재확인.
- `mapping_verified.json`: 실제32와이어/2목표 꼭지, RING/Hook/과실 좌표.
- `contact_events_240hz.json.gz`: live previous-solve와 private-forward를 분리한 force6/거리/활성 제약. robot/environment 쌍만 저장하며 전체 활성 접촉 침투 감시는 별도 sample에 있다.
- `videos/`: 동일 카메라 3개 MP4와 포스터/메타. 왼쪽 명목, 오른쪽 실제 저장 상태. 결과 성공 영상이 아니며 모두 중단 사례다.
- `video_validation.json`: FFmpeg 전체 디코드, 프레임 수, SHA256.
- `tests_final.log`, `tests_planner.log`: 188 + 5 passed, skip 없음.
- `verify_evidence.py`, `render_trials.py`, `package_results.py`: 해당 서버 절대경로를 사용하는 후처리 재현 스크립트. 물리는 실행하지 않는다. 실행기 CLI는 상위 보고서 참고.

대형 원본 모델/메시는 Git에 추가하지 않았다. 원래 서버의 `20260927_hook_seating_server/robot_under/replay_assets`가 실제 재실행·재렌더에 필요하다. 보고서와 저장 영상은 이 저장소만으로 검토 가능하다. 이번 물리6회는 이미 완료됐으며 재실행은 추가 실험이다.
