# Hook seating server evidence

[한국어 결과 보고서](../../HOOK_SEATING_SERVER_RESULTS_KO.md).

**정상 시작에서의 안착·유지 미확보.** baseline1회만 물리 실행했다. 새 명령 경로27개는 모두 환경 검사에서 차단되어 물리0회다. 조건을 바꿔 강제 실행하지 않았다.

- `seating_goals.json`: 실제 자산에서 생성한32개 기하 목표. trace가 아니다.
- `stage_results.json`: 28건 전체(기존 baseline + 새 계획27). 미실행의 실제 안착/유지/힘은 미측정이며0성공률로 계산하지 않는다.
- `actual_ik_goal_geometry.json`, `endpoint_results.json`: 선택12개 실제 FK와 전체 도구의 끝 자세 검사.
- `plans/{robot,robot_routed,robot_under}/`: 계획, 모든 실패의 최초 쌍/phase, result, 정확한 명령trace gzip.
- `inputs/`: 해당 계획군의 manifest/candidate/계획 입력. 서버 원본의 replay_assets가 별도로 필요하다.
- `diagnostic_policy.json`: 정확한 목표 capsule/rear-wire 쌍과 seat/hold/verify 단계만 허용하는 별도 정책. 운영 기본값을 바꾸지 않음.
- `baseline/`: 실제240Hz 기하와 접촉 원자료. `live_previous_solve`와 `private_forward` 시각을 구분한다.
- `controls/`: 독립 native 기하/접촉 fixture7개와 합성 시간 대조군. 정적 fixture와 합성 timestamp는 자동 진입/실제 hold 물리 증거가 아니다.
- `legacy_regression.json`: 실제 원본 명령2,512개/phase 동일.
- `provenance.json`, `tests.txt`, `video_validation.json`: 코드·모델·trace 해시, 시험 결과, 영상 전체 디코드 검사.

## 영상

|파일|의미|
|---|---|
|[baseline.mp4](videos/baseline.mp4)|정상 초기 상태에서 실제 물리 재실행. 부분 중심 진입, 꼭지 안착 없음|
|[final_under_plan.mp4](videos/final_under_plan.mp4)|최종 하부 삽입→상승 명목 경로. blocked이며 물리 미실행|
|[positive_actual_fixture.mp4](videos/positive_actual_fixture.mp4)|실제 자산을 IK 안착 pose에 처음부터 놓은 정적 기하 fixture|
|[negative_actual_fixture.mp4](videos/negative_actual_fixture.mp4)|과실 중심 진입만 있는 baseline 상태의 정적 snapshot|
|[actual_mapping.mp4](videos/actual_mapping.mp4)|실제 MJB 충돌 형상만 따로 표시. 목표 노랑 / 뒤쪽 와이어 청록 / Rachis 분홍|
|[positive_fixture.mp4](videos/positive_fixture.mp4)|독립 좌표의 native capsule 기하 양성|
|[negative_fixtures.mp4](videos/negative_fixtures.mp4)|앞쪽/바깥/관통/다른 꼭지/Rachis/과실 중심만 내부 대조|

앞4개 영상은 같은 전경/확대 카메라다. 마지막3개는 형상 식별/독립 fixture 확대용 별도 카메라다. 렌더러는 물리 계산을 하지 않는다. 정적 영상 길이는 실제 유지 시간의 증거가 아니다.

## 재현

새 폴더에서만 실행한다. 필요한 Python 환경은 `.venv`(MuJoCo3.13)와 `/root/isaaclab_env/bin/python`(Torch/FCL 계획)이다.

```bash
cd /root/farmily_tomato
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/propose_hook_seating_goals.py \
 /root/docker_share/mujoko_debugging_data/20260927_gutter_collision_scene/robot_checks/Tomato_02 \
 --output /root/docker_share/mujoko_debugging_data/NEW_HOOK_REVIEW/goals \
 --surface-gap-m .0004 --tilt-deg -15 0 15 --fraction .5 --include-axis-aligned --max-goals 32

mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/seating_controls.py \
 /root/docker_share/mujoko_debugging_data/NEW_HOOK_REVIEW/goals/seating_goals.json \
 /root/docker_share/mujoko_debugging_data/NEW_HOOK_REVIEW/controls
```

경로 재현은 새 run에 `inputs/robot_under` 등의4개 JSON을 복사하고, provenance의 source_run/replay_assets를 연결한 뒤 기존 `plan_candidates.py NEW_RUN --workers 4`를 계획 Python으로 호출한다. 기존폴더에 실행하지 않는다. 정확한 생성 당시 trace도 후보별gzip으로 보존했다. action14를 만들거나 원본 학습 데이터와 합치지 않는다.

환경 검사에는 `diagnostic_policy.json`을 사용한다. 원래 운영 정책을 수정하지 말고, blocked를 임의로 건너뛰어 실행하지 않는다. 실행 당시 서버 orchestration은 `executed_orchestration/*.txt`에 기록했으며 서버 고정 경로를 포함한다. 진단 원자료용으로 보존한 파일로서 운영 CLI가 아니다. 초기 endpoint 검사에는 phase 설정 오류가 있었고, 별도 `check_endpoints.py.txt` 재검사 결과를 사용했다.

시험 명령:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 mujoco-benchmark/.venv/bin/python -m pytest -q \
 mujoco-benchmark/tests/test_environment_preflight.py \
 mujoco-benchmark/tests/test_environment_preflight_native_extended.py \
 mujoco-benchmark/tests/test_environment_preflight_review.py \
 mujoco-benchmark/tests/test_contact_diagnostics.py \
 mujoco-benchmark/tests/test_hook_seating_geometry.py \
 mujoco-benchmark/tests/test_hook_seating_native.py \
 mujoco-benchmark/tests/test_hook_seating_diversity.py \
 mujoco-benchmark/tests/test_seating_timeline.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /root/isaaclab_env/bin/python -m pytest -q \
 mujoco-benchmark/tests/test_diagnostic_pose_planner.py
```
