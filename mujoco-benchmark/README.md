# 원본 식물의 Isaac / MuJoCo 비교

기존 Isaac 코드를 수정하지 않는 별도 실험이다. 로봇 암은 제외하고 원본 `full` 식물과
동일 고리의 world 궤적을 두 엔진에서 재생한다. RL·경로 탐색·domain randomization은 없다.
파단은 Phase 1에서 양쪽 모두 비활성화한다. 상세 차이는 [CONVERSION.md](CONVERSION.md).

## 환경과 원본 추출

저장소 루트 `/root/farmily_tomato`에서 실행한다. MuJoCo 가상환경은 Isaac 환경과 분리한다.

```bash
/root/isaaclab_env/bin/python -m venv mujoco-benchmark/.venv
mujoco-benchmark/.venv/bin/python -m pip install -r mujoco-benchmark/requirements.txt
/root/isaaclab_env/bin/python -u mujoco-benchmark/scripts/export_isaac.py
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/convert_model.py
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/suite.py \
  --recordings nvidia-sim/rl/runs/20260920_190518_candidate_dataset_pool/worker_05
```

`export_isaac.py`는 네이티브 PhysX 질량/관성, 실제 USD collision/joint/필터를 추출한다.
`reference.json`, 로봇 없는 `plant_reference.usda`, 고리 궤적/FK 정보를 저장한다.
`convert_model.py`가 `models/plant_original_equivalent.xml`을 생성한다.
원본 식물 376개 충돌체와 고리 35개 충돌체를 유지한다. hole은 열린 상태다.
생성물은 크기가 커 Git에서 제외하지만 디스크에 보존한다. 소스와 작은 검증 요약은 별도다.

기본 suite는 기계적 확인용 6개 동작, 기존 후보49의 전체 기록, 기존 로그의 짧은 기록
93개를 합친 100개다. 기존 시험에서 종료된 지점까지 그대로 사용한다. 100개의 새 고리걸기
계획을 만든 것이 아니다. 로그가 부족하면 반복 경로를 몰래 채우지 않고 오류를 낸다.

## 비교 실행

```bash
mujoco-benchmark/.venv/bin/python -u mujoco-benchmark/scripts/run_comparison.py
```

기본 순서: 각 엔진 1 → 10 → 100개, 이후 MuJoCo의 60/240/480Hz와 Euler 비교.
성능 측정 중 서로 영향을 주지 않도록 순차 실행한다. 120Hz는 두 엔진의 동일 입력 비교다.
날짜·시간으로 시작하는 `outputs/*_engine_comparison/` 안에 실행별 로그와 결과가 생성된다.
실행이 끝나면 비교표와 양쪽 엔진의 대표 영상도 생성한다. 영상이 필요 없으면
`--skip-videos`를 붙인다. 영상 생성 시간은 물리 처리량에 포함하지 않는다.

작은 단독 실행:

```bash
mujoco-benchmark/.venv/bin/python -u mujoco-benchmark/scripts/benchmark_mujoco.py \
  --hz 120 --count 10 --output mujoco-benchmark/outputs/my_mujoco_10
/root/isaaclab_env/bin/python -u mujoco-benchmark/scripts/benchmark_isaac.py \
  --hz 120 --count 10 --output mujoco-benchmark/outputs/my_isaac_10
```

출력 폴더는 새 이름을 사용한다. 후보당 고리 입력, 모든 물리 스텝의 body pose,
접촉 객체, 변위, 관통/부착 오차, RTF, CPU/RAM 사용량을 저장한다.
native step 시간, 상태 기록 포함 시간, 후처리/저장 포함 전체 시간을 구분한다.
reset 100회의 시간도 별도 측정한다. 렌더링·로딩은 physics 성능에 포함하지 않는다.

```bash
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/summarize.py \
  mujoco-benchmark/outputs/실행폴더
```

`comparison.md`, `comparison.json`, `results.csv`가 생성된다.
초기 좌표와 고리 명령이 실제로 일치하는지도 검사한다. 서로 다른 조건의 예전
로봇 포함 데이터셋 실행 시간과 이번 물리 시간은 직접 비교하지 않는다.

## 직접 보기와 영상

Isaac에서 원본 식물·고리의 진입/상승 동작을 화면으로 확인하기:

```bash
DISPLAY=:0 /root/isaaclab_env/bin/python mujoco-benchmark/scripts/benchmark_isaac.py \
  --gui --trajectory fixture_entry_lift --hz 240 \
  --output "mujoco-benchmark/outputs/$(date +%Y%m%d_%H%M%S)_isaac_view"
```

토마토 관찰 카메라로 시작한다. 동작을 한 번 실행하고 결과와 `gui_final.png`를 저장한 뒤
마지막 상태에서 창을 유지한다. 창을 닫으면 종료하며 `--exit-on-finish`로 자동 종료할 수도 있다.
주줄기 밀기는 `--trajectory fixture_stem_push`, 기존 후보49는 `--trajectory candidate_00049`다.
전체 6개를 보려면 `--trajectory` 대신 `--count 6`을 사용한다.
이 시험은 로봇 팔 없이 원본 식물과 고리를 표시한다. GUI는 기본 30Hz로 갱신하고
물리는 `--hz`를 유지한다. `--view-fps 15`로 화면 갱신만 줄일 수 있다.
GUI 실행은 별도 `physics_plus_viewer` 결과이며 headless 처리량과 구분한다.
RGB-D도 필요할 때만 `--rgbd`를 추가한다.
GUI/RGB-D용 실행 폴더에 `display_stage.usda`를 만들고, 원본 식물 레이어 위에
조명을 먼저 배치한 뒤 장면을 연다. 초기 `No lights found in stage` 알림을 방지하며
원본 식물 USD와 물리 설정은 변경하지 않는다.

MuJoCo에서 확인하기:

```bash
DISPLAY=:0 mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/viewer.py \
  --trajectory fixture_stem_push --hz 120
```

창을 닫을 때까지 동일 동작을 초기화하여 반복한다. `--speed 2`는 화면 재생 속도이며
물리 timestep은 바꾸지 않는다. 처리 성능이 부족하면 지정 배속보다 느릴 수 있다.

```bash
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/render_results.py \
  --run mujoco-benchmark/outputs/실행폴더/mujoco_120hz_100_implicitfast --speed 1
```

상태 기록을 재생해 `videos/index.html`과 MP4를 만든다. Isaac 실행 폴더에도 사용할 수 있다.
원래 실행된 물리 상태를 그리므로 영상을 위해 다른 물리를 다시 돌리지 않는다.
고리와 식물이 보이는 전체/접촉 확대 두 화면이다. `--speed .5`로 슬로모션을 저장할 수 있다.

## RGB-D 별도 시험

```bash
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/rgbd.py \
  --output mujoco-benchmark/outputs/my_rgbd
```

640×480, 15Hz RGB와 metric optical-Z depth를 실제 렌더링한다. `rgb.png`, `depth_m.npy`,
카메라 K와 raycast/depth 비교, 렌더링 포함 RTF를 저장한다. 정책/비전 추정은 구현하지 않는다.

## 검증과 별도 최적화

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 mujoco-benchmark/.venv/bin/python -m pytest -q \
  mujoco-benchmark/scripts/test_reference.py
```

원본 비교 뒤에만 `scripts/optimize_model.py`로 별도 `plant_mujoco_optimized.xml`을 만든다.
이 버전은 잎/잘린 가지 286개 collision을 끄고 열매 11개의 free joint/weld를
강체 연결로 합친다. 질량·탄성 관절·시각 형상은 유지하며 파단은 지원하지 않는다.
원본과 다른 조건이므로 그 속도를 엔진만 바꾼 이득으로 해석하지 않는다.

아직 다루지 않는 항목: 자동 detach, 원본 줄기의 visual skinning, MuJoCo 전체 로봇 이식,
실물 마찰/탄성 보정, RGB-D 학습, 병렬 MuJoCo 환경. 이 단계는 수확 성공률 검증이 아니다.

## Isaac 로봇 팔의 비용 비교

```bash
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/robot_cost_comparison.py
```

기존 후보49의 첫 10초 관절 명령을 실제 Isaac 로봇 articulation으로 재생한다.
팔은 기존 800/40, 리프트는 50000/2500의 stiffness/damping을 사용하며 기존 effort 제한을
유지한다. 새 IK나 경로 계획은 하지 않는다. CPU 240Hz·PGS 64/4·full 식물·파단 비활성이다.

팔의 실제 고리 위치를 먼저 기록한 후, 그 경로를 고리만 있는 환경에서 재생한다.
고리만/팔 포함의 headless 2개와 고리만/팔 외형 숨김/팔 외형 표시의 GUI 3개를 순차 비교한다.
GUI 카메라·960×720 해상도·30Hz 화면 갱신은 동일하다. 고리 외관도 공통 표시 모델을 사용한다.
각 단계가 끝나면 해당 시험 창을 닫고 다음 단계로 넘어간다. 기존 사용자 창은 중단하지 않는다.

`outputs/날짜_robot_cost/`에 CSV/JSON/비교표/최종 화면을 저장한다.
이것은 Isaac 내부의 로봇 비용 비교이며 MuJoCo로 로봇을 이식한 것은 아니다.
고리만 조건은 kinematic, 팔 조건은 관절 모터로 구동되는 dynamic 고리이므로
같은 경로라도 접촉 반응이 완전히 같다는 전제는 두지 않는다. 실제 경로 오차를 별도로 검사한다.
초기화/물리/렌더링/나머지 실행/후처리 시간을 구분하며, 각 조건 1회 측정임을 명시한다.

## 이번 측정 결과 보기

- [측정 결론과 한계](RESULTS.md)
- [영상 16개 · RGB-D · 비교표 통합 화면](outputs/20260922_002000_original_comparison/index.html)
- [전체 경로 CSV](outputs/20260922_002000_original_comparison/results.csv)
- [작은 보존용 검증 요약](validation/engine_comparison.json)

100개 경로는 **한 환경에서 순서대로 재생한 것**이다. 100개 병렬 환경 측정이 아니다.
원본 120Hz 100개는 검사/저장까지 Isaac 565.3초, MuJoCo 539.8초였다.
240Hz/480Hz sweep은 6개 fixture로 제한했으므로 100개 안정성을 보장하지 않는다.
영상의 오른쪽은 접촉 확인을 위해 잎/잘린 가지 시각 mesh를 숨긴 검사 화면이며
물리 상태·접촉 로그는 그대로다. 왼쪽에는 전체 형상을 표시한다.

이미 저장한 실행으로 통합 화면을 다시 만들기:

```bash
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/behavior_metrics.py \
  mujoco-benchmark/outputs/실행폴더
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/summarize.py \
  mujoco-benchmark/outputs/실행폴더
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/render_suite.py \
  mujoco-benchmark/outputs/실행폴더
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/make_index.py \
  mujoco-benchmark/outputs/실행폴더
```

### GPU 렌더러와 화면 표시 측정

물리는 CPU에서 계산하며 RGB-D/영상은 NVIDIA GPU에서 렌더링한다.
이 장비는 기본 EGL이 Mesa llvmpipe로 연결되었으므로 `render_backend.py`가
프로젝트의 `config/nvidia_egl.json`만 선택한다. 시스템 드라이버 설정은 바꾸지 않는다.
`render_backend.json` 또는 `rgbd_report.json`의 vendor/renderer를 확인한다.
사용자가 이미 지정한 `MUJOCO_GL`, `__EGL_VENDOR_LIBRARY_FILENAMES`는 덮어쓰지 않는다.

```bash
DISPLAY=:0 mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/viewer.py \
  --trajectory fixture_fruit_collision --hz 120 \
  --benchmark-seconds 10 --output mujoco-benchmark/outputs/my_viewer_benchmark
/root/isaaclab_env/bin/python mujoco-benchmark/scripts/benchmark_isaac.py \
  --rgbd --start 1 --count 1 --recovery 0 --hz 120 \
  --output mujoco-benchmark/outputs/my_isaac_rgbd
```

뷰어 측정은 속도 제한 없이 실행하고 자동 종료한다. 일반 viewer는 기존처럼 창을 닫을
때까지 반복한다. GUI는 비동기이므로 `sync_calls`를 실제 화면 프레임 수로 해석하지 않는다.
Isaac RGB-D는 native hook pose를 물리 속성이 없는 시각 복사본에 반영한다.
렌더링 준비 중 물리 상태가 변하지 않았는지도 검사한다.
MuJoCo depth는 MSAA를 꺼서 픽셀 깊이 resolve 오차를 줄이고 5mm~5m clip을 사용한다.

### 별도 최적화 모델 실행

화면에서 진입·상승 동작을 반복 확인하려면 저장소 루트에서 실행한다.
로봇 암 없이 식물과 고리만 표시하며, 한 동작이 끝나면 초기화 후 같은 동작을 반복한다.

```bash
DISPLAY=:0 mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/viewer.py \
  --model mujoco-benchmark/models/plant_mujoco_optimized.xml \
  --trajectory fixture_entry_lift --hz 120
```

주줄기 밀기는 `--trajectory fixture_stem_push`, 기존 후보49 기록은
`--trajectory candidate_00049`로 선택한다. 창을 닫으면 종료한다.

```bash
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/optimize_model.py
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/benchmark_mujoco.py \
  --model mujoco-benchmark/models/plant_mujoco_optimized.xml \
  --hz 120 --count 10 --output mujoco-benchmark/outputs/my_optimized_10
```

60Hz 최적화 모델은 빠르지만 6개 중 2개에서 0.5mm 관통 기준을 넘었다.
속도만 보고 정상 데이터 생성 모델로 채택하면 안 된다.

## 로봇 팔 및 CPU 병렬 실행

실제 7축 로봇을 포함하는 모델과 1~64프로세스 측정 결과는
[ROBOT_PARALLEL.md](ROBOT_PARALLEL.md)를 참고한다. 기존 고리 단독 모델도 유지한다.
