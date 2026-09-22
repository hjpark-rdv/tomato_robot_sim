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
