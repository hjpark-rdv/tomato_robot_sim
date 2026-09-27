# 힘·변위 중단값 비교 증거 (2026-09-28)

[결과보고서](../../CONTACT_LIMIT_SWEEP_RESULTS_KO.md). SIM-GT 오프라인 자료이며 training_eligible=false, hook_success=null이다.

- `sweep_manifest.json`: 실행 전에 고정한12개 조건과 기준 커밋. 원본5경로 + 기존 감속00 경로 재사용.
- `progress.json`, `runner.log`: 모든 실행 명령/종료 코드/결과/벽시계 시간. 성능 벤치마크로 해석하지 않는다.
- `results.json`: 최종 전체 비교표, 꼭지 기하·힘·유지, 비목표 쌍, 추종 오차.
- 각 실험 폴더: 전체 사전검사와 실제 physics 판정, 정확한 접촉 정책과 source/code 해시, 240Hz 원자료, 실제 qpos 재생 상태 NPZ, 요약 JSON.
- `verification.json`: 전체 검사 완료, timestamp 누락, 원본 해시 및 종전1N/5mm 실험과 공통 시간의 실제 qpos 동일성.
- `videos/`: 실제 저장 상태 비교 영상. 왼쪽은 명목, 오른쪽은 실제 상태. 종료 또는 중단 원인 표시.
- `force_displacement.png/svg`: 00번 원래 속도/18배 감속의 5N/20mm 조건. private-forward 과실 힘 합과 변위의 관계. x표시는 최종 기록 상태다.
- `video_validation.json`: 전체 디코드·프레임 수·파일 해시.
- `run_sweep.py`: 이번 서버에서 사용한 실행기. 새12회 물리 실행을 호출하므로 이미 완료된 결과에 재실행하지 않는다. 기존 폴더 덮어쓰기 거부.
- `summarize_sweep.py`, `render_results.py`, `plot_results.py`, `package_results.py`: 후처리 재현. 서버 절대 경로 사용. 실제 physics를 추가 실행하지 않는다.

대형 모델과 메시, 기존 trace는 이전 서버 경로에 유지했다. 기존 경로/trace의 Git 사본은 `../target_fruit_contact_server_20260928/inputs/`에 있다. 보고서·영상 검토에는 별도 파일 전달이 필요 없지만 타 머신에서 물리 재실행/재렌더하려면 원래 MJB/메시 자산이 필요하다.
