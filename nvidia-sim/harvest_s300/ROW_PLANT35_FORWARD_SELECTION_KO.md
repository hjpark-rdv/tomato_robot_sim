# Plant35부터 시작하는 후속 레일 제안

2026-10-11. 실제 수확/레일 시험을 새로 실행한 결과가 아니라, 기존 제안의 순서를 검증한 준비 자료다.

첫 제안은 Plant35 / Truss02, 레일 Y `1.7779228352467091 m`, `joint` connector / seed `6`이다. 원본 계획 SHA는 `ae887ab5b854637561223557ea3f673e468df4d9ccf37202099ed8959aa2e468`이다. 그 뒤에는 기준 Y 1.6 m에서 앞쪽에 있는 기존 제안을 가까운 순서로 놓고, 뒤쪽 제안도 모두 남겼다.

기존 47제안 / 27송이 / 18식물을 한 개도 삭제하거나 수정하지 않았다. 기존 `run_row_harvest.select_preferred_stations`로 전체 18식물, 첫 1식물 예산, 이미 지나간 식물의 재생성 차단을 검증했다. 기존 Plant29 우선순위 파일과 입력 5개는 해시가 그대로다. 유한 탐색에서 아직 SELF 통과를 발견하지 못한 19송이와 배치 정책상 제외한 60송이도 기존대로 기록한다.

- 새 순서: `/root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/next_plant35_forward_selection.json`
- 실행 전 준비 명세: 같은 결과 폴더의 `next_plant35_after_truss02_operational_deferral/recipe.json`
- 선택 검증: 같은 폴더의 `selection_validation.json`
- 저장된 생성 코드: 같은 폴더의 `code/prepare_plant35_forward_selection.py`
- 저장소 검증 요약: [validation/row_plant35_forward_selection_20261011.json](validation/row_plant35_forward_selection_20261011.json)

현재 9/10 상태나 실행 중인 결과를 완료한 부모 상태로 사용하지 않았다. 준비 명세의 부모 trial/checkpoint/모델 해시는 미지정이다. 실제 Truss02 10개 분리와 최종 실제 상태 검사 통과를 확인한 후, Truss01의 별도 유한 계획 차단 자료를 부모 상태에 정확히 묶어야 한다. 이 경우도 식물 전체 20/20 완료로 기록하지 않는다. 새 활성 창의 초기 유효성, 실제 부모 상태에서의 전체 SELF·환경 검사, 물리 레일 이동은 아직 수행하지 않았다.

정확한 native tool 조각을 쓴 과거 보고서도 검토했다. 원본 모델의 Y1.392 제안은 6 Hz 차단 진단이며, raw 환경 보고서에 `source_sha256`와 `allowed_truss_key`가 없다. 120 Hz 수동 경로 보고서는 별도의 수동 잎 모델에서 실행했고 그 모델의 배경 안정성이 검증되지 않았다. 따라서 이 보고서들을 기존 엄격한 운영 보류 증명에 자동 편입하지 않았고, 원자료와 한계를 `historical_accurate_tool_blocker_review.json`에 보존했다.

새 native 실행, 새 IK/FCL 질의, 충돌체·물성·판정 기준 변경은 없다. `training_eligible=false`, 수확 성공은 미판정이다.
