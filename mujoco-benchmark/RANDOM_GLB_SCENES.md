# 랜덤 GLB 송이 물리 장면

`scripts/generate_random_glb_scenes.py`는 기존 주줄기와 잎에 GLB 송이를 부착한다. 한 장면에 송이를 여러 개 넣을 수 있으며, 각 송이는 탄성 관절·질량·충돌체를 가진다. 원본 GLB와 기존 고정 모델은 수정하지 않는다.

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/generate_random_glb_scenes.py \
  --seed 23 --scenes 2 --trusses 2 \
  --segment-min 4 --segment-max 13 \
  --angle-min 0 --angle-max 180 \
  --output /root/docker_share/mujoko_debugging_data/NEW_random_glb_scenes
```

`--scenes`는 장면 개수, `--trusses`는 장면마다 붙일 송이 개수다. `--seed`와 장면 번호가 GLB 종류·위치·각도를 결정한다. 위치는 주줄기 segment와 그 segment 안의 연속적인 비율(`stem_fraction`)로 저장한다. 여러 송이는 지정 구간을 나누어 배치한다. 각도는 사용자가 확인한 **GLB 자체의 Y축 회전**만 0~180°에서 뽑고, GLB Y-up을 MuJoCo Z-up으로 바꾸는 좌표계 변환은 고정이다. `--source-dir`로 GLB 폴더를 바꿀 수 있다. 송이 수가 GLB 파일 수보다 많으면 종류를 다시 뽑을 수 있다.

현재 다섯 GLB의 물리 설정을 적용한다. 모두 6번 열매를 제거하고, white는 2번 열매 위치 보정과 중심가지 강성·감쇠 수정을 적용한다. 원본 GLB에는 손대지 않는다. 로봇·주줄기·잎은 기존 모델을 그대로 사용한다.

각 `scene_XXXX/`에는 `model.xml`, `model.mjb`, `reference.json`, `scene.json`, `preview.png`가 있다. `scene.json`에 송이별 원본 해시·segment·segment 안의 비율·Y각도·실제 부착 좌표를 저장하고, 최상위 `manifest.json`에 전체 목록을 저장한다. 중간 조립 파일은 최종 모델을 만든 뒤 삭제한다. 여러 송이를 넣으면 모델 파일이 크다. 2송이 시험의 `model.mjb`는 약 485MiB였다.

첫 송이의 목표 이름은 `Tomato_05`, 두 번째는 `truss_01__Tomato_05`처럼 접두사가 붙는다. 로봇 후보를 시험할 때는 해당 장면의 모델과 reference를 함께 넘긴다.

```bash
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/candidate_experiment.py \
  --model /root/docker_share/mujoko_debugging_data/NEW_random_glb_scenes/scene_0000/model.mjb \
  --reference /root/docker_share/mujoko_debugging_data/NEW_random_glb_scenes/scene_0000/reference.json \
  --target truss_01__Tomato_05 --candidates 1 --workers 1 --planning-workers 1 --hz 240
```

생성기는 각 장면의 초기 상태를 240Hz로 2초 실행해 최대 겹침 0.5mm 이하·최대 식물 바디 이동 5mm 이하·수치 경고 없음인지 검사한다. 이 판정은 `scene.json:validation`과 HTML에 남긴다. 실패한 무작위 장면도 숨기거나 자동 보정하지 않고 남긴다. 이 짧은 정지 검사는 로봇 접촉 중 물리 유효성, 실제 관절 도달성, 수확 성공을 보증하지 않는다. `physics_ready`는 별도 로봇 실행과 검증 전까지 false다.

검증 예: `/root/docker_share/mujoko_debugging_data/20260923_random_glb_physics_final/index.html`. seed23의 2장면×2송이가 각각 초기 검사에 통과했고, 원래 주줄기·잎의 geom 302개 초기 위치·회전 보존, 송이당 10열매/2송이 모델 430DOF, 로봇 FK 확인 및 seed 재현성 확인을 완료했다.

`scene_0000`의 두 번째 송이(`truss_01__Tomato_05`)로 로봇 경로 1개를 추가 실행했다. 계획과 물리 실행이 끝났고 최대 겹침 0.347mm로 물리 기준을 통과했다. 결과는 중심 미진입(`miss`), 열매 최대 이동 52.51mm다. 실행 결과: `/root/docker_share/mujoko_debugging_data/20260923_random_glb_physics_final/robot_smoke_second_truss_checked/index.html`. 작은 보존 요약은 `validation/random_glb_scenes.json`이다. 모든 랜덤 장면의 경로 도달성·접촉 유효성을 보증하는 결과는 아니다.

## 모든 열매의 후보·RGB-D 수집과 장면 단위 학습

기존 `candidate_experiment.py`의 Sobol 진입 경로/IK/FCL/물리 판정/결과 형식과 `prepare_observations.py`의 D435 촬영·camera action v2를 사용한다. `collect_random_glb_scenes.py`는 장면과 열매 이름을 열거하며 기존 실행기를 순차 호출한다. 랜덤화 대상은 **GLB 송이의 종류·주줄기 부착 위치·송이 Y각도**이고 송이 안의 열매 배열 자체는 바꾸지 않는다.

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/collect_random_glb_scenes.py \
  --seed 23 --scenes 3 --trusses 1 --candidates 10 \
  --workers 8 --planning-workers 4 --hz 240 \
  --output /root/docker_share/mujoko_debugging_data/NEW_random_glb_collection
```

각 유효 장면의 `reference.json:fruit_specs`에 있는 모든 열매를 target으로 삼는다. 6번 열매는 GLB 물리 모델에서 제거했으므로 송이당 10개다. 각 열매에 후보 10개를 시도하고, **실행 전 동일 초기 상태**에서 가상 RGB-D 9장을 3×3 영상 구역별 한 장씩 찍는다. 관측 하나에 실행 가능한 후보들의 action v2를 연결한다. 계획 실패나 물리 오류는 그대로 저장하고 학습용 성공/실패로 바꾸지 않는다. 모두 미진입이면 `not_found_in_tested_candidates`이며 수확 불가능 판정이 아니다. 초기 장면 겹침·정지 검사에 실패한 장면은 원인을 남기고 로봇 수집 대상에서 제외한다.

열매가 주줄기 뒤에 가려지거나 팔이 닿지 않거나 주변 송이와 충돌해 실제로 접근 불가능한 경우도 있을 수 있다. 어떤 열매에도 성공 경로가 반드시 있다고 가정하지 않는다. 10개 중 계획 가능한 후보가 0개면 사진 9장은 그대로 저장하고 해당 열매에는 학습용 물리 라벨을 만들지 않는다. 계획 가능한 후보가 있더라도 10개 모두 실패할 수 있으며, 이때 기록은 **이번 제한된 탐색에서 진입을 발견하지 못함**이다.

출력은 `scene_XXXX/targets/<target>/physics/`(기존 결과·상태·재생 HTML)와 `observations/`(원본 RGB·Depth·크롭·camera action v2·HTML)이다. **9-view 수집은 별도 mask 파일과 Depth 유효성 PNG를 저장하지 않는다.** 촬영 도중 GT 분할은 목표의 가림 정도와 크롭 정보를 계산하는 데만 사용하고 삭제한다. Depth 유효성은 저장된 실수 Depth의 유한값에서 복구된다. `collection.json`은 장면/열매/판정·train/validation/test 장면 분할을 기록한다. `--resume`은 완료된 열매를 건너뛰고 중단된 물리는 기존 `resume_candidates.py`로 이어간다. 장면과 각 열매의 seed, 원본 GLB 해시, 물리 모델 해시가 저장된다. 모델 바이너리는 장면과 후보 실행 사이에 같은 파일시스템 하드링크를 사용할 수 있으며 각 실행의 해시는 계속 검증한다.

먼저 `--scenes 1`로 만든 smoke 수집을 같은 seed·설정으로 3장면까지 늘리려면 `--resume --extend --scenes 3`을 쓴다. 기존 장면의 물리·사진은 재실행하지 않고 scene_0001~0002를 생성한다. 장면 수가 3개가 되면 기존 smoke 장면도 train 분할로 기록한다. 중간에 장면 개수만 바꾸고 `--extend`를 생략하면 설정 불일치로 중단된다.

3장면 이상이면 **장면 전체**를 train/validation/test로 나눈다. 1~2장면 실행은 smoke 전용이다. 기존 `train_multi.py`가 이 수집 포맷을 감지하면 같은 ResNet18/DINOv2 특징 추출·Scorer·검증 BCE 선택을 쓰되, 열매마다 다른 실행 가능 후보만 학습하고 장면 경계를 넘는 정규화나 모델 선택을 하지 않는다.

```bash
/root/isaaclab_env/bin/python mujoco-benchmark/learning/train_multi.py \
  /root/docker_share/mujoko_debugging_data/NEW_random_glb_collection \
  --output /root/docker_share/mujoko_debugging_data/NEW_random_glb_training
```

처음 보는 장면은 수집기에 `--plan-only`를 넣어 같은 10개 경로를 **계획만** 하고 9장씩 촬영한다. 학습 결과로 실행 가능한 후보 점수를 9시점에 걸쳐 평균한 뒤, 열매당 선택 경로 1개만 기존 `candidate_experiment.execute`로 새로운 MuJoCo 물리에서 실행한다. 학습 컬렉션과 다른 seed·출력 폴더를 사용한다.

```bash
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/collect_random_glb_scenes.py \
  --seed 24 --scenes 1 --trusses 1 --candidates 10 --plan-only \
  --workers 8 --planning-workers 4 --hz 240 \
  --output /root/docker_share/mujoko_debugging_data/NEW_random_glb_unseen

/root/isaaclab_env/bin/python mujoco-benchmark/learning/test_multi_physics.py \
  /root/docker_share/mujoko_debugging_data/NEW_random_glb_training \
  --collection /root/docker_share/mujoko_debugging_data/NEW_random_glb_unseen \
  --workers 8
```

점수는 아직 교정된 성공 확률이 아니다. GT 열매 위치로 가상 관측 위치와 크롭을 지정하며 실제 로봇 카메라 자세 도달성을 보장하지 않는다. 현재 라벨은 **중심 진입**이고 꼭지 걸림/수확 성공은 평가하지 않는다. 후보 계획에 쓰는 FCL 모델은 원래 계획 자산이므로 새로 붙인 GLB 송이들에 대한 완전한 사전 충돌 회피를 보증하지 않는다. 실제 물리 접촉과 GLB 관통은 후보 실행에서 다시 검사한다.

### 2026-09-23 작은 전체 연결 시험

- 수집: `/root/docker_share/mujoko_debugging_data/20260923_random_glb_full_smoke/index.html`. Seed 5의 독립 장면 3개(각 송이 1개)를 train/validation/test로 분리했다. 열매 30개에 후보 300개를 시도하여 계획 실패 110, 물리 미진입 160, 중심 진입 18, 물리 오류 12개를 기록했다. 물리 실행은 190개이며, 25개 열매에서 진입 후보를 발견하지 못했다. 이 중 4개 열매는 계획 가능한 후보가 0개였다.
- 관측: 열매마다 원본 RGB·Depth 9장, 각각 총 270장. 열매 mask 및 Depth 유효성 PNG는 0개. 실행 가능한 경로와 관측의 카메라 좌표 변환 1,710쌍 검사 통과했다.
- 학습: `/root/docker_share/mujoko_debugging_data/20260923_random_glb_training_smoke/index.html`. 세 모델×세 seed를 기존 평가기와 고정 영상 특징으로 학습하고, 검증 BCE로 ResNet18 RGB-D seed 2를 선택했다. Pair 수는 train 459 / validation 792 / test 351. 테스트 장면 자체에 발견된 중심 진입이 0개이고, 열매 10개 중 3개는 계획 가능한 경로가 없으므로 이 작은 실행의 테스트 1순위 진입률 0%로 모델 간 우열을 판단할 수 없다.
- 새 장면: `/root/docker_share/mujoko_debugging_data/20260923_random_glb_unseen_seed24/index.html` (seed 24, 기존 장면과 다른 GLB 종류·부착 위치·각도). 열매 10개에 100개 후보를 계획하여 88개가 통과했고, 물리 결과를 보지 않고 RGB·Depth 각 90장과 모델 점수로 열매당 1개씩 선택했다.
- 새 물리: `/root/docker_share/mujoko_debugging_data/20260923_random_glb_fresh_physics_smoke/index.html`. 추천 확정 후 10개를 독립 초기화로 새 실행하여 Tomato_10에서만 중심 진입(열매 최대 변위 13.25mm), 나머지 9개는 미진입, 물리 오류 0개였다. 저장 상태 10개의 모델·상태 해시 및 재생 검사 통과. **RGB-D의 영상 없는 기준 모델 대비 이점, 새로운 장면에 대한 일반화, 꼭지 걸림 수확 성공은 입증되지 않았다.**

작은 보존 요약: `validation/random_glb_collection_pipeline.json`. 저장된 성공 사례를 화면에서 보려면 다음 명령을 사용한다.

```bash
cd /root/farmily_tomato
DISPLAY=:0 ./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/replay_candidate.py \
  /root/docker_share/mujoko_debugging_data/20260923_random_glb_fresh_physics_smoke/scene_0000/Tomato_10/physics \
  --candidate candidate_00004
```
