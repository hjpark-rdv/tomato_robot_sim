# 서버 재검토 증거 (2026-09-27)

[검토 보고서](../../ENVIRONMENT_PREFLIGHT_SERVER_REREVIEW_RESULTS_KO.md)를 먼저 읽는다. 이 디렉터리를 포함하는 Git 커밋이 결과 버전이다. 대용량 MJB·메시·전체 qpos는 서버 공유 폴더에 보존하고, 리뷰에 필요한 요약·접촉 원자료·MP4를 커밋했다.

- [전체 경로 표](full_scan_tables.md), [전체 JSON](full_results.json), [진단 정책](full_policy.json)
- [실제 모델 매핑](mapping.json), [입력·코드 해시/버전](provenance.json)
- [실행 gate 원자료](gate_results.json), [자동 테스트](tests.txt), [영상 디코드 검사](video_validation.json)
- [gutter02 요약](sync_gutter02/summary.json), [동일 시각 표본](sync_gutter02/representative_samples.json), [전체 동기화 기록 gzip](sync_gutter02/synchronized_samples.json.gz), [전체 선택 쌍 힘 기록 gzip](sync_gutter02/selected_contact_records.json.gz)
- [stem01 요약](sync_stem01/summary.json), [동일 시각 표본](sync_stem01/representative_samples.json), [전체 동기화 기록 gzip](sync_stem01/synchronized_samples.json.gz), [전체 선택 쌍 힘 기록 gzip](sync_stem01/selected_contact_records.json.gz)
- [사전 고정한 실험 예산](pilot/experiment_budget.json), [5회 걸림 지표](pilot/hook_results.json). 개별 폴더에 preflight/계획/기존 물리 분류도 보존했다.

## 대표 영상

왼쪽은 명목 명령+초기 식물, 오른쪽은 새 물리 시험에서 저장한 실제 qpos다. 영상 렌더링은 새 물리 계산을 하지 않는다. 각 열 위는 전경, 아래는 확대이며 좌우 카메라가 같다. 힘 표시는 private forward 재계산이며 live 직전 solve 힘과 구분한다.

|영상|검토 포인트|
|---|---|
|[gutter02 MP4](videos/sync_gutter02.mp4)|양의 간격에서도 Rachis 비목표 접촉력 발생|
|[stem01 MP4](videos/sync_stem01.mp4)|고정 주줄기와 충돌해 실제 로봇이 명령을 크게 벗어남|
|[baseline MP4](videos/pilot_baseline.mp4)|부분 중심 진입과 목표 꼭지 안착/유지가 다름|
|[방위각+4° MP4](videos/pilot_azimuth_plus4.mp4)|작은 변경에도 이번 지표로는 걸림 개선 없음|

로컬 `index.html`로 4개 영상을 한 화면에서 볼 수 있다. GitHub는 HTML을 앱으로 실행하지 않으므로 MP4 링크를 사용하거나 폴더를 내려받는다.

## 실행 명령

서버 전용 원본 자산은 `provenance.json`의 source_inputs를 따른다. 새 폴더에서만 실행한다.

```bash
cd /root/farmily_tomato
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/audit_environment_preflight.py \
 /root/docker_share/mujoko_debugging_data/20260927_gutter_collision_scene/robot_checks/Tomato_02 \
 --candidate predicted_00000 \
 --policy mujoco-benchmark/validation/preflight_rereview_20260927/full_policy.json \
 --output /root/docker_share/mujoko_debugging_data/NEW_REVIEW/full_gutter02 --all-violations

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 mujoco-benchmark/.venv/bin/python \
 mujoco-benchmark/scripts/diagnose_contact_timing.py \
 /root/docker_share/mujoko_debugging_data/20260927_gutter_collision_scene/robot_checks/Tomato_02 \
 --pair g390 glb_col_TRUSS_Rachis_05 \
 --output /root/docker_share/mujoko_debugging_data/NEW_REVIEW/sync_gutter02
```

stem01의 source는 `20260927_stem_obstacle_scene/robot_checks/Tomato_01`, 쌍은 `g410 neighbor_stem_collision_18_05`다. 출력 디렉터리는 새 이름을 사용한다. 전체7경로와 gate의 실제 실행 스크립트는 `executed_orchestration/*.py.txt`에 보존했으며 기존 폴더에서 재실행하면 안 된다.

파일럿 재현은 새 폴더에 원본 `replay_assets`를 읽기 전용으로 연결하고 `pilot/{manifest,planning_inputs,action_frame,candidates}.json`을 복사한 뒤 다음 기존 계획기를 호출한다.

```bash
/root/isaaclab_env/bin/python mujoco-benchmark/scripts/plan_candidates.py NEW_PILOT_DIR --workers 1
```

그 다음 `executed_orchestration/run_pilot.py.txt`의 실행 당시 래퍼를 새 폴더 기준으로 사용한다. 래퍼의 `O.parent/full_policy.json` 경로도 맞춰야 한다. 이 래퍼는 별도 진단용이며 blocked 경로를 오프라인 실행하므로 운영 데이터 수집에 사용하지 않는다. 기존 결과 폴더에서는 hold가 중복 추가될 수 있다. 정확한 실행 trace는 각 후보의 `trace*.json.gz`로 보존했다.

목표 꼭지 걸림/잘못된 걸림/스침의 양성·음성 물리 fixture 검증은 아직 완성되지 않았다. `legacy_retention_proxy`는 진단값이며 `hook_success`는 null이다.
