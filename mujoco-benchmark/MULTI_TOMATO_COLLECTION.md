# Tomato_05를 제외한 데이터 수집

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/collect_tomatoes.py \
  --targets 1,2,3,4,6,7,8,9,10,11 \
  --candidates 1000 --workers 48 --planning-workers 16
```

총10개대상×1,000후보=10,000회. 토마토는순차처리하며각대상의물리후보를48개CPU프로세스로분산,계획은16개CPU작업자로처리한다.물리120Hz/기존최적화모델/식물장면및로봇초기자세유지.도달불가능한후보는IK실패로기록하며억지로성공시키지않는다.소량테스트에서성공해도1,000개모두가실행가능하다는뜻은아니다.

각대상경로시험후RGB-D를자동저장한다. 출력:

```
/root/docker_share/mujoko_debugging_data/YYYYMMDD_HHMMSS_multi_tomato_collection/
  index.html
  collection.json
  Tomato_01/
    physics.log
    observations.log
    physics/        # 1000개 결과, JSON/CSV, 30fps 상태, 재연 자산
    observations/   # RGB, metric depth, crop, camera action v2, HTML
  Tomato_02/
  ...
  Tomato_11/
```

실행터미널은토마토/단계진행을표시한다.후보별진행은각physics.log에기록한다.예상공간은Tomato05의1,000개약3.7GB를기준으로전체약40GB지만경로길이에따라달라진다.대표영상은자동렌더하지않으며상태재생용파일은저장한다.중단된출력폴더를덮어쓰지않는다.일부대상만다시실행하려면새폴더로`--targets 7,8,9,10,11`처럼지정한다.

## 카메라의 의미

기존71개관측과마찬가지로**가상카메라데이터**다.실제D435장착체인/내부파라미터를사용하되,기준카메라를`대상중심−Tomato05중심`만큼평행이동한뒤기존±5cm/±2.5cm,±20°/±5°의관측변화를준다.각시점이타깃을중앙에고정하도록재조준하지않으며3×3영상구역을분산수집한다.실제로봇과식물은움직이지않는다.

`dataset.json:actual_robot_world_from_color`에는실제초기관절자세의D435pose,`virtual_reference_translation_world`에는대상별가상평행이동을저장한다.각camera.json은실제촬영에사용한world_from_optical/effective tool_from_optical을기록한다.실제로봇이그관측자세에갈수있는지검증하지않았다.타깃GT라벨/마스크/카메라좌표action은각토마토를기준으로다시계산한다.관측수는기본64개화면내시점+최대8개화면가장자리시점으로가림/잘림과함께보존한다.

## 개별 명령

```bash
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/candidate_experiment.py \
  --target Tomato_01 --candidates 1000 --workers 48 --planning-workers 16 \
  --output /root/docker_share/mujoko_debugging_data/my_tomato01_physics
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/prepare_observations.py \
  /root/docker_share/mujoko_debugging_data/my_tomato01_physics
```

대상은manifest에서자동연결되므로카메라저장명령에다시번호를지정할필요없다.기존Tomato05기본실행과기존데이터는보존한다.향후여러토마토학습시관측/후보분할외에**토마토ID단위holdout**도필요하다.

검증 완료: `20260922_134307_multi_tomato_collection`에서05제외10개대상각1후보실행+1시점RGBD+camera-action v2변환완료. 대상ID/물리중심/영상GT중심일치,좌표복원검증통과.11개대상의공유로봇·식물초기qpos와preload일치도확인. 전체1만후보수집은아직실행하지않음.
