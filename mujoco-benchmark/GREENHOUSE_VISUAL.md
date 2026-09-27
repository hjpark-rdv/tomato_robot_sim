# 충돌 없는 원본 하우스 표시

`greenhouse_visual.py`는 Isaac 실행에 쓰는 `nvidia-sim/scenes/farmily_greenhouse_robot.usd`의 `/World/Greenhouse`를 같은 월드 좌표로 MuJoCo에 추가한다. 하우스/거터/배관/기둥/지붕 1,078개 메시, 170,701개 삼각형을 색상별9개 mesh geom으로 합치고 표시용 바닥1개를 추가한다. `contype=conaffinity=0`, 월드에 직접 부착, 질량/관절 추가 없음. 로봇·식물의 기존 충돌은 유지한다.

원본 diffuse 색상/opacity를 가져오며, USD 조명/재질을 완전히 동일하게 재현하는 것은 아니다. 전체 구조를 보기 위한 보조 directional light를 추가한다. 원본 하우스 자산은 창문 일부가 제거된 visual USD이다. 없는 창문을 새로 만들지 않는다.

## 직접 예측 테스트

기존 명령에 `--house`를 추가한다. `--gui`를 함께 쓰면 실제 물리 실행 창에도 배경이 보인다. RGB-D에도 배경이 반영되므로 과거 배경 없는 학습 이미지와 입력 분포가 달라질 수 있다. 기존 데이터는 수정하지 않는다.

```bash
cd /root/farmily_tomato
DISPLAY=:0 /root/isaaclab_env/bin/python -u mujoco-benchmark/learning/test_direct_angle.py \
 /root/docker_share/mujoko_debugging_data/20260923_direct_angle_training_full \
 --seed 2026 --glb red --angle 20 --house --gui
```

`generate_random_glb_scenes.py --house`도 지원. 기본은 배경 없음으로 유지한다.

## 생성된 하우스만 열어보기

```bash
DISPLAY=:0 ./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/greenhouse_visual.py \
 /root/docker_share/mujoko_debugging_data/20260927_greenhouse_visual/model.mjb --view-only
```

## 검증

`/root/docker_share/mujoko_debugging_data/20260927_greenhouse_visual/`에 전체/통로/로봇 PNG, HTML, validation.json 및 재현 스크립트 보존. 같은 Tomato01 명령25초/240Hz에서 배경 전후 qpos가 완전히 같음. 동적 body 수/질량/관성/강성/감쇠 동일. 추가10개 geom의 충돌 비활성 및 world 부착 확인. 첫 측정 headless 실행6.84/6.80초(각1회); GUI/RGB-D 처리량 비교는 아님. 소스별 메시 목록과 USD 의존 layer 해시는 `model.house.json`에 기록.

기존 생성/수집 관련 테스트6개 통과. 전체75도 직접예측 테스트나 배경 추가 후 모델 성공률은 이번 작업에서 검증하지 않음.

## 10열매 75도 경로 시간 비교

`20260927_house_timing_75/index.html`: 기존 예측각으로 새75도 경로10개 계획 후,240Hz에서 하우스 유무 각10개 순차 실행(순서 교대). 모든 qpos 및 판정 일치, 양쪽 중심진입10/10·invalid0. 각 조건1회 묶음.

10개 합계: 물리루프159.622→159.278초, 실행+상태저장164.187→163.834초, RGB-D촬영/저장8.664→9.812초, 초기화4.644→4.931초. 초기화+실행/저장+촬영 합177.495→178.578초(+0.61%). 물리루프는 실행/저장 시간에 포함되어 중복 합산하지 않는다. 장면 생성·계획·추론·GUI·mask/crop 후처리는 측정 제외. 배경으로 예측각이 달라지는 영향은 분리하기 위해 재추론하지 않았다. 재현 plan.py/bench.py/report.py와 상태를 같은 결과 폴더에 저장.

## 주변 식물 미리보기

`preview_neighbor_plants.py MODEL_XML --output NEW_DIR --count 16 --seed 27`은 기존 식물 외형을 색상별 공유 mesh로 묶어 거터 두 줄에 배치한다. `--spacing`으로 행 내 간격(m)을 지정하며 해당 간격으로 거터에 들어가는 개수까지 지원. 원본 줄기/잎/송이 외형 반복이며 식물별 위치 및 yaw만 조금 다르다. 주변 식물 충돌·탄성 없음, 수확용 수집 장면으로 사용하지 않는다. 기존 수확 대상은 그대로 유지한다.

16개 샘플: `/root/docker_share/mujoko_debugging_data/20260927_neighbor_plants_preview/index.html`. 추가480 geoms, 공유 mesh14개, 관절/바디/질량 변화 없음 확인. 전체/로봇/통로 시점 촬영 및 GUI 표시. 성능 벤치마크는 아직 미실시.

더 촘촘한 미리보기: `--count 34 --spacing .25`, 주변34개/명목간격25cm. 실제 배치는±2.5cm 위치 지터 포함. `20260927_dense_neighbor_plants_preview/index.html`. 추가1020개 visual geom, 기존14개 template mesh 공유. 성능 및 주변 충돌 미검증.

## 거터 양측 반대 방향 배치 + 줄기 2종

`preview_neighbor_plants.py --paired-gutters --alternate-model V9_MODEL.xml`로 거터 양쪽(중심에서 ±0.10m)에 줄기를 배치한다. 한쪽 yaw 0°, 다른 쪽 180°로 기존 줄기의 굽은 방향을 반대로 배열한다. 위치 기준은 첫 줄기 분절 중심이 아닌 뿌리 끝점이다. 줄기 종류는 seed로 랜덤 선택한다. 기본 단일 줄기 배치도 유지한다.

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/preview_neighbor_plants.py \
  /root/docker_share/mujoko_debugging_data/20260927_stem_color_matched/model.xml \
  --alternate-model /root/docker_share/mujoko_debugging_data/20260927_stem_leaves_v9/model.xml \
  --paired-gutters --count 68 --spacing .25 --seed 27 \
  --output /root/docker_share/mujoko_debugging_data/NEW_paired_mixed_house
```

검토 장면: `20260927_paired_mixed_stem_house/index.html`. 기존 수확 대상/로봇은 유지하고 주변 68개만 시각 복제한다. 거터는 기존 좌표 x=±0.775m, y=0.4~4.65m를 사용한다. 주줄기·잎·송이를 함께 복제하며, 종류별 공유 mesh를 재사용한다. 주변 식물은 충돌/탄성이 없는 표시용이며 이번 배치의 렌더 성능은 아직 벤치마크하지 않았다.

## 주변 줄기마다 랜덤 송이 부착

위 명령에 `--random-trusses`를 추가하면 복제된 기존 고정 송이를 제거하고 주변 줄기마다 하나씩 랜덤 송이를 표시한다. 기존 `profiles`/`sample_placements`/`build_glb_physics`를 재사용한다. GLB 5종, scale 0.5, segment 4~13, 분절 내 비율 0.15~0.85, GLB Y각도 0~180° 규칙이다. 기존 6번 열매 제외 및 white 보정 유지. 주줄기 표면 부착 계산은 물리 생성기와 같은 `attachment_frame` 함수를 쓴다. 거터 반대편에서는 식물 전체 방향을 180° 돌리며, 송이 자체의 추가 회전은 GLB Y축뿐이다.

송이 종류별로 기존 물리 모델을 한 번 생성한 뒤 초기 외형만 공유 mesh로 추출하여 배치한다. 주변 송이는 **정적 외형**이며 물리 관절이나 충돌을 68개 추가한 것이 아니다. 수확 대상 원본은 유지한다. 겹침 없는 물리 학습 장면이라고 해석하지 않는다. 각 배치의 GLB 해시/위치/각도/보정/부착 좌표는 scene.json에 저장한다.

결과: `/root/docker_share/mujoko_debugging_data/20260927_mixed_house_random_trusses/index.html`.

## 로봇 쪽 송이 밀도 추가

기존 장면 폴더에 `preview_neighbor_plants.py SCENE_DIR --densify-robot-side --extra-per-plant 2 --seed 127 --output NEW_DIR`를 적용한다. 기존 줄기/송이 위치는 보존하고 공유 송이 mesh를 추가 사용한다. 현재 로봇 근처의 배치를 위한 휴리스틱: 뿌리 y≤2.25m 줄기에서 열매 중심 높이0.55~1.10m, y0.1~1.8m, |x|≤0.85m, 부착점보다 통로 안쪽으로4cm 이상인 후보를 선택한다. 추가 송이끼리 같은 줄기 부착점 간격12cm 이상을 둔다. 최대600회 탐색 후 부족한 줄기는 그대로 기록한다.

기존 GLB 회전/표면부착 규칙을 유지한다. 로봇 IK/주변 관통/수확 성공 검증은 아니며 추가 송이는 정적 표시용이다. 결과 `20260927_robot_side_dense_trusses/index.html`.

## 현재 장면 직접 예측 테스트

`learning/test_direct_angle.py --existing-scene DIR --physics-hz 240 --gui`로 장면을 새로 생성하지 않고 촬영/예측/물리 실행을 연결한다. DIR에는 `model.mjb`, 대응 `reference.json`, 초기 검사 통과 기록을 담은 `scene.json`이 필요하다. 예전 사진/각도를 재사용하지 않고 각 타깃을 다시 촬영한다. 기존 장면 모드는 생성 옵션과 동시에 사용할 수 없다.

이번 대상: `20260927_dense_harvest_scene` (배경 `20260927_robot_side_dense_trusses`, 원래 물리 송이 red Y20°). 기존 타깃 10개만 물리 대상이며 주변 106송이는 충돌 없는 시각 배경이다. 원본 물리 body/활성 충돌 형상 동일 확인, 240Hz 2초 초기 검사 통과(최대 겹침0.228mm). 최종 수확/분리 성공 평가가 아니라 기존 중심진입 기준이다.

```bash
cd /root/farmily_tomato
DISPLAY=:0 /root/isaaclab_env/bin/python -u mujoco-benchmark/learning/test_direct_angle.py \
  /root/docker_share/mujoko_debugging_data/20260923_direct_angle_training_full \
  --seed 2026 --existing-scene /root/docker_share/mujoko_debugging_data/20260927_dense_harvest_scene \
  --physics-hz 240 --gui
```

완료 결과 `20260927_dense_house_harvest_test/index.html`: 전체10개 중 중심진입6(02,04,05,07,08,10), 미진입2(01,09), 가림으로 미시도2(03,11), 실행8개 물리오류0. 촬영/모델 추론/GUI 포함620.16초. ResNet18 seed2가 새 RGB-D에서 예측한 각도는 모두 -90° 부근(-89.99999~-89.94080°)으로 몰렸다. 배경에 대한 일반화나 장애물 회피 성능을 입증하지 않는다. 각 타깃은 독립 reset; 수확한 열매를 제거하는 연속 수확이 아니다. 저장 재생 상태8개 유한값 확인. 보고서의 고정 '계획100%/4알 성공' 문자열을 실제 집계로 바꾸고 미시도 및 시각 배경 한계를 표시했다.

## 거터 충돌 추가

`greenhouse_visual.py MODEL.xml --gutter-collision-only --output NEW/model.xml`로 기존 외형에 거터 충돌을 추가한다. 새 장면 생성/직접 예측 CLI에서는 `--gutter-collisions`를 사용한다(하우스 외형도 함께 추가). 기존 저장 데이터에는 소급 적용하지 않는다.

USD의 두 거터를 구성하는 바닥2/옆판4/접힌 테두리4/끝판4/받침리브4, 총18개를 각각 고정 충돌체로 변환한다. 직육면체는 정확한 box, 모서리가 다듬어진 부품은 원본 mesh의 폐곡면 부피와 convex hull 부피 일치를 검사한 뒤 개별 convex mesh로 사용한다. 거터 전체를 하나의 convex hull로 막지 않는다. 지지 다리/철사/배지/다른 하우스 부품과 주변 식물은 아직 시각 전용이다. 마찰값은 실측 보정되지 않았다.

`RobotEngine`은 거터–고리뿐 아니라 거터–로봇 팔/식물 접촉도 기록한다. 최대 관통0.5mm 초과면 기존 `invalid_physics` 판정으로 전달한다. `max_gutter_penetration_m`, `gutter_contact_steps`, `gutter_contact_pairs`를 결과 metrics에 저장한다. 초기 장면 검사에도 거터 접촉을 포함한다. 정상 접촉 자체를 수확 실패로 강제하지 않고, 진입 여부와 물리 유효성을 기존 기준으로 판정한다. FCL 사전 계획에는 아직 거터가 추가되지 않았으므로 새 자동 회피 계획 기능은 아니다.

검증/장면: `/root/docker_share/mujoko_debugging_data/20260927_gutter_collision_scene/`. 240Hz, 속도0.15m/s 구형 접촉체의 옆판/열린 내부 바닥 시험에서 각각 최대0.250mm, 통과/뚫림 없음. 별도의 초기 자유낙하 시험은 최대3.328mm 침투로 실패했으며 `freefall_probe_failed.json`에 보존했다. 제어된 시험 통과를 고속 충격/모든 로봇 경로의 비관통 보장으로 일반화하지 않는다.

기존 RGB-D 예측 경로 3개를 새 물리로 재실행: Tomato_01 miss/거터접촉0, Tomato_02 중심진입/거터접촉0, Tomato_09 miss/거터접촉8577스텝(테두리 및 받침리브 대 g410), 기록된 거터 최대침투0.0mm. 새 사진으로 추론한 결과는 아니며 동일 명령의 충돌 추가 검증이다. 성공 기준 자체는 유지했다.

## 로봇 쪽 타깃 구분 및 주변 주줄기 충돌

```bash
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/preview_neighbor_plants.py \
  /root/docker_share/mujoko_debugging_data/20260927_gutter_collision_scene \
  --stem-obstacles-layout /root/docker_share/mujoko_debugging_data/20260927_robot_side_dense_trusses \
  --output /root/docker_share/mujoko_debugging_data/NEW_stem_obstacles
```

거터 중심에서 통로(x=0) 쪽에 뿌리가 있는 줄기를 로봇 쪽으로 구분한다. 색상은 기준이 아니다. `stem_obstacles.json`에 양측 줄기와 타깃 적격 여부를 기록하고, 물리 송이의 실제 실행 가능 이름은 scene.json의 `eligible_targets`에 저장한다. 직접 각도 테스트는 이 목록만 실행한다. 현재 원래 물리 송이의 10개 열매가 로봇 쪽이며, 주변 복제 송이들은 적격 구분만 있을 뿐 물리 타깃으로 전환되지 않았다.

주변 양측 68개 줄기에 원래 16분절 캡슐과 동일한 곡선/굵기를 배치: 총1088개. 두 줄기 외형은 같은 골격이므로 동일 캡슐 구조를 공유한다. 주변 주줄기는 고정 장애물이며 별도 탄성/질량을 추가하지 않는다. 기존 수확 줄기의 탄성은 유지한다. 주변 잎/송이 자체는 아직 시각 전용이다. `neighbor_stem_*` 접촉·최대침투 metrics 및 초기 검사에 포함하며 0.5mm 초과는 invalid_physics. FCL 회피 계획은 미연결.

검토 장면: `20260927_stem_obstacle_scene`. 초기 2초 검사 통과. 결과 HTML은 저장 명령3개를 새 장애물 물리로 재실행한 검사이며 신규 이미지 추론 시험은 아니다.

동일 명령의 01/02/09 재실행은 모두 invalid_physics: 주줄기 최대 침투1.183/1.492/1.621mm. 기존02의 중심진입을 정상 성공으로 유지하지 않는다. 충돌 반응과 오류 검출은 확인했지만, 접촉을 고려한 회피 계획이나 밀어붙이기 방지 제어는 아직 구현하지 않았다. 관련9테스트 통과.
