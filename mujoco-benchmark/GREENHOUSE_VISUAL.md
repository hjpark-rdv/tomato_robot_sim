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
