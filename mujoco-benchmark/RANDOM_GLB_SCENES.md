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
