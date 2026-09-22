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

실행터미널은토마토/단계진행을표시한다.후보별진행은각physics.log에기록한다.예상공간은Tomato05의1,000개약3.7GB를기준으로전체약40GB지만경로길이에따라달라진다.대표영상은자동렌더하지않으며상태재생용파일은저장한다.중단된수집은아래`--resume`명령으로이어갈수있다.별도새실험으로일부대상만다시실행하려면새폴더로`--targets 7,8,9,10,11`처럼지정한다.

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

## 후처리 최적화 및 중단 재개 (2026-09-22)

- 경로별 고정 기하를 한 번만 준비하고, 각 관측의1,000개 action을 NumPy 배열 연산으로 변환한다. 좌표/회전 복원 검사도 전체 배열에 적용한다.
- `--postprocess-workers 8`(기본값): 시점별 압축 저장·파일 읽기·검증을 CPU 스레드 작업자로 분산한다. NumPy와 압축 코드가 GIL 밖에서 실행되므로 이 구간은 스레드 병렬을 사용한다. 다중 프로세스의 배열 복사/새 인터프리터 시작 비용을 피한다.
- 생성 중 v1을 먼저 검증한 뒤 v2로 재변환하던 중복 전체 검증을 제거했다. 최종 v2의 모든71,000쌍과 RGB/Depth/mask/crop/장착변환을 검증한다. v1 파일은 호환성을 위해 보존한다.
- 71시점×1,000후보 실측: 구버전 v1검증+v2변환/검증 **131.81초**, 개선 버전 v2변환+최종검증 **1작업자1.95초 / 8작업자1.41초**. 주된 개선은 배열 연산이며 작업자 수만 늘린 효과가 아니다.
- 촬영 포함 개선 버전 **65.61초**(모델/장면 초기화 전 시간 제외). 기존 완성 로그 구간은 약3분16초~3분23초였다. 물리 계획·실행 약8분은 이 변경으로 빨라지지 않는다.
- 원본71,000쌍과 모든 배열 값 오차1e-12이내 일치. RGB/정렬Depth/GTmask각71파일은 SHA256완전일치. 손상 방향값을 주입한 데이터는 검증에서 거부됨.
- 측정 자료: `/root/docker_share/mujoko_debugging_data/20260922_144727_postprocess_benchmark/`의 `benchmark.json`, `legacy_benchmark.json`, `capture_comparison.json`.

중단된 실제 수집을 이어가기:

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/collect_tomatoes.py \
  --resume \
  --output /root/docker_share/mujoko_debugging_data/20260922_134834_multi_tomato_collection \
  --candidates 1000 --workers 48 --planning-workers 16 --postprocess-workers 8
```

1~4번 완성 데이터는 건너뛴다. 6번은 집계 JSON 대신 개별 결과/상태/trace 해시를 검사하여931개 정상 결과를 발견했으며69개만 실행하면 된다(검사 당시). 기존 계획도 재사용한다. 이후7~11번은 순서대로 신규 수집한다. 여기서는 본 수집을 자동 재시작하지 않았다.

- `resume_candidates.py RUN --inspect-only`: 재실행 없이 파일 무결성과 남은 개수 확인.
- 같은 run의 부모를 잃은 작업자 중 stdout이 정확히 해당physics.log를 가리키는 작업자만 재개 전에 정리한다. 다른 시뮬레이터는 건드리지 않는다.
- Ctrl+C 시 해당 단계의 프로세스 그룹을 종료하여 작업자만 남는 문제를 방지한다. 복사본에서 중단 정리/누락후보 재실행/원래 물리 결과 일치/완료대상 건너뛰기 검증 완료.
- 부분 생성된 관측 폴더는 `observations_interrupted_...`로 보존하고 촬영을 다시 수행한다. 완성 관측 데이터는 재촬영하지 않는다. 대상목록과후보수는원래수집설정과같아야한다.
- 집계결과와수집진행JSON은임시파일에서원자적으로교체하며동일수집/재개중복실행은파일잠금으로막는다.


## 2026-09-22 진입 탐색 범위 변경

새 staged6d 후보는 진입각 −45°~+45°, 하부 여유 0~10mm를 Sobol 샘플링한다. MuJoCo와 Isaac의 공통 생성기 `nvidia-sim/rl/trajectory_search.py`에 적용했다. 여유는 초기 열매 충돌 구의 아랫면과 고리 와이어 윗면 기준이다. 0mm는 기하학상 여유가 없는 조건이며 실제 접촉은 물리 엔진의 접촉 설정에도 영향을 받는다. Sobol은 끝값 조합을 반드시 포함하지 않는다. 기존 데이터/저장된 후보/재생은 변경하지 않았고, resume은 저장된 후보를 계속 사용한다. 같은 seed와 candidate_id라도 새 실행은 이전 범위와 다른 경로이므로 실행 폴더를 구분한다. 실제 새 범위 물리 실험은 아직 수행하지 않았다.

## 최신 데이터 생성 조건: 각도만 탐색

새 staged6d/Sobol 후보는 1차원 scrambled Sobol로 진입 방위각 −45°~+45°만 바꾼다. GT 중심에 인식 오차 없음. 좌우 오프셋0mm, 하부 여유2mm 고정. 삽입42.5mm/수직 상승32.5mm는 이전 범위 중간값으로 고정, pre-hook170mm/roll0/elevation0 유지. 이 조건은 중심선을 향하는 계획이며 접촉/변형/제어 오차까지 제거하거나 실제 진입 성공100%를 보장하지 않는다. 삽입 깊이도 모든 후보에서 동일하고 열매 중심 도달을 자동 보장하지 않는다.

공통 trajectory_search 생성기와 새 실행 metadata에 sampling=scrambled_sobol_azimuth_only 기록. 파일/CLI trajectory_mode=staged6d 명칭은 호환성을 위해 유지. 기존 저장 후보/재생/계속 실행(resume)은 이전 경로를 유지하므로 새 조건 수집은 새 실행 폴더로 시작해야 한다. 이전 seed/candidate 번호와 새 경로는 다르다. RL 제어에는 적용하지 않는다.

생성/기하 검사5개 통과(1,000후보에서 각도만 다름, 좌우 중심 정렬, 와이어 여유2mm 확인). 물리 실행/데이터 수집은 실행하지 않았다.


### 추가 변경: 진입각 ±90°

새 angle-only 후보의 각도 범위를 −90°~+90°로 확대. 나머지 고정값은 동일. Tomato_06 재생성: `./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/collect_tomatoes.py --targets 6 --candidates 30 --workers 30 --planning-workers 16`. 기존 데이터를 resume하지 않고 새 폴더에 생성한다. Sobol은 정확한 끝값을 포함하지 않을 수 있다. 이 변경 후 수집은 실행하지 않았다.

## 두 구간 45° 상승 적용

새 후보에 `lift_profile=diagonal_45_return`을 저장한다. 진입 방향 기준 전진16.25mm+상승16.25mm 후, 후퇴16.25mm+상승16.25mm. 최종 위치는 이전 수직32.5mm 상승과 동일하며 고리 회전은 바꾸지 않는다. 경유점명은 `rise_mid`, 마지막은 `rise`; 실제 제어 단계는 두 구간 모두 rise로 처리하여 동일 상승 속도/접촉 판정을 적용한다. 이동 거리는 기존 상승의 √2배이므로 동일 속도에서는 상승 시간이 길어진다.

기존 저장 후보에 profile이 없으면 기존 상승을 유지한다. 새 방식은 waypoint5개를 카메라 좌표에도 모두 저장한다. 기존14개 compact features의 lift 방향/거리는 최종 순변위이며, 이14개만으로 직선 상승과 대각 상승을 구분할 수 없다. 두 방식을 학습용으로 섞으려면 phase_names/전체 waypoint 또는 별도 profile 입력을 사용해야 한다. 서로 다른 phase 구조를 한 batch로 섞으면 명시적으로 거부한다.

기하 검사6개 통과, 기존4점/새5점의 카메라 좌표 변환·복원 검증 통과.

## 중심 진입 단독 성공 기준

새 MuJoCo 데이터 수집의 classification_rule=center_entry_only_v2: 물리 오류는 invalid_physics, 정상 실행에서 중심 진입 이력이 있으면 열매 최대 밀림과 무관하게 partial_center_entry, 없으면 miss. target_center_max_displacement_m과 target_displacement_exceeded(20mm 초과)는 참고 지표로 별도 저장. 결과 JSON/CSV와 HTML의 판정 설명/밀림 집계를 갱신했다.

기존 데이터는 자동 덮어쓰지 않았다. 과거 manifest에 rule이 없으면 displacement_first_v1로 이어 실행하여 혼합 판정을 방지한다. 기존 ±90°/30개 데이터를 읽기 전용으로 재분류 검증하면 중심 진입 성공3개, 미진입27개다. RL 보상/종료 조건은 이번 변경 대상이 아니다.
