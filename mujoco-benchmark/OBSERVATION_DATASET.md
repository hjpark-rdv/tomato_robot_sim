# Tomato_05 관측 데이터 준비

## 목적과 범위

기존 Tomato_05 초기 장면과1,000개 물리 시험 결과를 재사용한다. 로봇과 식물은 초기 상태로 고정하고 가상 D435의 위치·방향만 변경한다. 새로운 후보 경로 생성, 물리 실행, 모델 학습은 하지 않는다.

```bash
cd /root/farmily_tomato
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/prepare_observations.py \
 /root/docker_share/mujoko_debugging_data/20260922_111838_candidate_search
```

결과는 `/root/docker_share/mujoko_debugging_data/YYYYMMDD_HHMMSS_tomato05_observations/`에 저장된다. `--seed`로 관측 조합을 재현하고 `--output`으로 결과 폴더를 지정한다.

## 중앙 고정 방지

- 기본 D435 카메라 좌표 기준 이동: X/Y±5cm, Z±2.5cm.
- 카메라 로컬 X/Y 회전±20°, Z회전±5°. 프레임 가장자리까지 분포시키기 위한 가상 관측 범위이며 실제 레일/로봇 도달성을 검증한 값이 아니다.
- 위치/각도를 독립적으로 Sobol sampling한다. 목표를 향한 look-at/자동 재정렬은 하지 않는다.
- 목표 중심이 영상의3×3구역 각각에 들어오는 후보7개씩 선택한다. 기준 관측1개를 더해64장, 제한 범위에서 나온 화면 밖 중심 후보최대8장을 추가한다. seed42에서는7장이 추가되어 총71장이다.
- 이는 관측 다양성 시험이며, 서로 다른71개 식물 장면이 아니다. 기하/탄성/마찰 랜덤화 없음.

## 저장 구조

- `index.html`: GT 표시 미리보기 갤러리와 가림/분할 필터. 원본 RGB는 표시 없는 별도 파일.
- `coverage_preview.jpg`: 화면9개구역 대표 관측 비교.
- `dataset.json`: 원본 결과/모델 해시, 촬영 범위, 라벨 의미, 평가분할과 검증 요약.
- `initial_state.npz`: 공유 초기 상태와 실제 로봇 관절값.
- `actions.json`: 원본 후보ID/6개변수/월드 경유점/부분진입 여부/최대이동량/결과. 한 번만 저장한다.
- `observations.jsonl`: 모든 관측의 인덱스와 목표·크롭 정보.
- `observations/view_XXXX/`
  - `rgb.png`:640×480 원본 RGB.
  - `depth_m.npy`, `depth_aligned_to_color_m.npy`, 유효mask 및preview: 실제 optical Z 거리(m), 무효NaN.
  - `target_visible_mask.png`: 과실 g362/v43 geom-ID 기반 가시 마스크. 꼭지/꽃받침은 과실마스크에 포함하지 않는다.
  - `target_isolated_mask_gt.png`: 가림 측정용 격리 렌더. 학습 관측 입력이 아니다.
  - `annotated_preview.jpg`: 사람이 목표를 확인하는 그림. 모델 RGB 입력으로 사용하지 않는다.
  - `crop_local_rgb.png`, `crop_context_rgb.png`: 투영된 과실 충돌지름의3배/6배 폭. 동일영역Depth는압축NPZ. resize 없이 유효영상 범위로 잘라 crop 좌표와 K를 함께 기록한다.
  - `camera.json`: RGB/Depth 광학 pose, K, clipping, 가상 이동 여부. `nominal_tool_from_optical`은 기존 장착값, `tool_from_optical`은 가상 이동을 반영한 실제 로봇tool기준 유효변환이다.
  - `observation.json`: 목표GT중심, 가시bbox/centroid, 잘림, 가림 비율, input_usable, 공유데이터 링크.
  - `actions_camera.npz`: 각 후보4개 단계의 고리 중심 position_xyz와orientation_xyzw를 해당 카메라 좌표계로 변환한 라벨. candidate_ids/phase_names로 원본과 연결한다.

카메라 위치만 바꿨으므로 로봇 관절값은 모든 관측에서 같다. 가상 관측의 카메라 자세를 관절값만으로 복구하면 안 되며 저장된world_from_optical 또는 유효장착변환을 사용해야 한다. 실행 때는 원본 초기 자세로 복귀한다는 조건이다.

## 가림 및 학습 분할

가림은 `1 - 가시과실pixel / 격리과실의 영상안pixel`이다. 화면 밖으로 잘린 비율까지 나타내는 값은 아니며 `truncated`를 별도 기록한다. 가시pixel0은not_visible,20pixel미만은input_usable=false. 가림80%초과heavily_occluded,10%초과partially_occluded로 표시한다. 카메라near0.1m/noise미모사 등의 한계는 D435_RGBD.md를 따른다.

같은 이미지의 후보pair를 무작위로 train/test에 나누면 누수가 생긴다. 기본 예비 분할은 화면우상단7관측test, 좌하단7관측validation, 나머지50관측train, 화면밖중심7관측visibility_test다. 실제사용시 input_usable을 먼저 확인한다.

모든 분할이 같은 토마토의 같은 물리결과를 공유하므로 이 분할은 시점변화에 대한 예비 평가다. 새로운 토마토 일반화나 영상 정보의 필요성을 증명하지 않는다. 후속 학습 때 카메라 자세·후보만 사용하는 기준모델과도 비교해야 한다. GT중심/마스크는 목표 지정용 시뮬레이터 라벨이며 실제 검출기 성능을 검증한 것이 아니다. `hook_success`는 여전히 미판정이며 중심진입 여부와최대이동량을 학습 대상으로 사용한다.

## 검증

`validate_observations.py RUN`은 RGB/Depth/mask 크기·유효성·과실마스크 포함관계, crop내부행렬,15mm RGB/Depth baseline, 카메라 경유점의월드복원, 장착변환 일관성을 확인한다. 생성기 종료 시 자동 수행한다.

격리 마스크 경계에서MSAA가 object ID를 섞는 문제를 피하기 위해GT마스크 촬영 시 multisampling을 끈다. 목표 외형이나 실제 RGB 안의 장애물을 지우지 않는다. 격리렌더의geometry투명도는곧바로복구한다.

## 생성 완료 자료

최종 폴더: `/root/docker_share/mujoko_debugging_data/20260922_122302_tomato05_observations/`.
71개관측/1,000개공유경로, 약192MiB. 촬영·라벨생성구간57.7초(이후검증시간제외).
가시성은부분가림62/심한가림2/화면잘림7, 완전히안보이는관측0.
71,000pair 월드복원 위치오차최대6.7e-16m, 회전행렬성분오차1.5e-15.
기존 원본 후보0/87/999의시작qpos와이번초기상태의정확일치도별도확인.
Chromium에서71개갤러리 및test7개/심한가림2개필터확인.

## 카메라 기준 action v2 (2026-09-22)

`actions_camera_v2.npz`가 새 학습 입력이다. `candidate_ids`로 공통 `actions.json`의 진입 여부/최대 이동량 라벨과 결합한다. 계획 실패로 경유점이 없는 후보는 이 배열에서 제외한다.

14개 값: 카메라 기준 진입 방향(3), 고리 자세 quaternion XYZW(4), 하부 여유/측면 오프셋/삽입 거리(3), 카메라 기준 상승 방향(3), 상승 거리(1). 거리 단위는 m. 광학 축은 오른쪽 X/아래 Y/앞 Z다. 중력 방향 `gravity_direction_camera`는 관측별 별도 입력으로 제공한다. 여유는 기존 계획의 수직 여유, 측면 오프셋은 접근 방향에 대한 측면 거리로, 카메라 XYZ 이동량이 아니다.

절대 로봇 시작 좌표는 14개 입력에 포함하지 않는다. 정확한 경로 복원용으로 토마토 초기 중심 기준 경유점 `target_relative_waypoint_xyz_camera`를 함께 보존한다. 이것은 실제 계획의 고리 기준점 위치다. 카메라→월드 회전으로 방향/자세를 변환하고, 위치에는 타깃의 월드 중심을 더하면 된다. 기존 6개 Sobol 변수와 월드 경유점은 재현용이며 카메라 기준 예측값과 혼용하지 않는다.

실험 `plan.json`/`result.json`의 `action_camera`는 **원래 초기 D435** 기준이다. 관측 이미지별 학습에는 각 `view_XXXX/actions_camera_v2.npz`를 사용해야 한다. 관측마다 카메라 방향이 다르므로 동일 경로의 수치도 달라진다.

기존 이미지의 변환/결과 연결 갱신:

```bash
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/upgrade_observation_actions.py \
  /root/docker_share/mujoko_debugging_data/20260922_122302_tomato05_observations \
  --run /root/docker_share/mujoko_debugging_data/20260922_124028_camera_action_v2_1000
```

동일 모델 해시·초기 qpos를 확인한 뒤 결과를 연결한다. 이미지 재촬영이나 물리 형상 변경은 없다. v1 메타데이터는 `schema_v1_backup/`, 절대 경유점은 `actions_camera.npz`에 남긴다. 신규 생성기는 v2 변환도 자동 수행한다. v2 변환기는 모든 관측-후보 쌍의 위치/방향/자세 월드 복원을 검사한다.
