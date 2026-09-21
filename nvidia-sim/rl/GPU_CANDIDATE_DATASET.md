# GPU 물리 데이터 생성기 (실험용)

`run_gpu_candidate_dataset.sh`는 **한 Isaac Sim 세션에 여러 환경**을 올린다.
PhysX dynamics와 broadphase는 GPU에서 계산하고, 고리 segment / 꼭지 / 과실을
구분하는 native collider contact report는 CPU readback으로 받는다.
GPU 렌더링만 켜는 실행기가 아니다. 상태·effort I/O는 articulation batch view로 묶는다.
IK/FCL 계획, 결과 판정과 파일 저장은 CPU에서 수행한다.

기존 CPU 실행기와 데이터 폴더는 변경하지 않는다. GPU 데이터는 새 폴더에서 생성한다.
현재 GPU 기본은 **`practical60`: PGS 64 position / 4 velocity iterations, 60Hz,
식물 보조 관성 5e-4 kg·m²**다. 원래 960Hz/1e-5는 `--physics-preset reference960`으로 선택한다.
최신 기능 검사와 범위는 [GPU_PRACTICAL_PHYSICS.md](GPU_PRACTICAL_PHYSICS.md)를 읽는다.
아래의 960Hz 대규모 시험 수치는 이전 모델의 기록이며 새 모델의 처리량 보증이 아니다.
CPU의 TGS와 수치적으로 같다고 보장하지 않는다. 각 데이터의 config / backend /
candidate physical_inputs에 GPU·PGS·experimental 정보를 명시한다.
원래 식물, 11개 과실, 로봇, 탄성/감쇠/마찰, 파단 임계값을 유지한다.
CCD는 GPU 경로에서 끈다. 고리 관통 방지와 다양한 후보의 CPU 대비 검증은 별도로 필요하다.

## 실행

```bash
# RGB-D, clone 격리와 reset 검증만 수행
./nvidia-sim/run_gpu_candidate_dataset.sh --num-envs 4 --candidates 8 --validate-only

# 생성할 후보 중 특정 번호만 전체 물리 실행 (후보를 다시 샘플링하지 않음)
./nvidia-sim/run_gpu_candidate_dataset.sh --num-envs 1 --candidates 4 --candidate-indices 3

# 여러 후보를 한 GPU 세션에서 실행. 기존 CPU pool의 workers 옵션은 사용하지 않는다.
./nvidia-sim/run_gpu_candidate_dataset.sh --num-envs 16 --candidates 32
```

## 처리량 개선 실행

```bash
./nvidia-sim/run_gpu_candidate_dataset.sh \
  --num-envs 256 --candidates 1000 --schedule continuous
```

`--schedule continuous`는 완료된 슬롯만 원래 상태로 리셋하고 다음 후보를 배정한다.
느린 후보가 끝날 때까지 모든 슬롯이 기다리는 기존 방식은 `--schedule batch`이며,
호환성을 위해 기본값으로 유지한다. 후보 시작마다 reset 상태를 검사한다.
계획도 방금 리셋된 슬롯의 GT를 사용하므로, 실행 중인 env_0의 변형을 새 후보에
가져오지 않는다. RGB-D는 동일한 초기 상태의 공통 관측을 계속 참조한다.

관절 명령은 기본 `--command-uploads batched`로 전달한다. 모든 implicit actuator
계산을 먼저 마치고 position/velocity/effort를 articulation 종류별로 묶어 보낸다.
기존 비교용 경로는 `--command-uploads legacy`이다. 물리 주기, 솔버 반복 횟수,
관절 목표값과 접촉·파단 기준을 변경하는 옵션이 아니다.

짧은 동일 명령 구간에서 기존 32환경 대비 256환경의 처리량은 약 3.4배였다.
이는 1,000개 전체 후보의 완료 시간이나 1시간 내 완료를 보장하는 수치가 아니다.
상세 측정 범위와 검증 결과는 [GPU 처리량 개선 기록](GPU_OPTIMIZATION.md)에 정리한다.
변경 전 실험 폴더와 섞지 않도록 새 datetime 폴더로 실행한다.

`--candidate-indices 0,3,27`은 `--candidates`로 생성한 원래 목록의 인덱스만 고른다.
초기 Sobol 표본 수와 seed를 같게 유지해야 기존 후보와 비교할 수 있다.
대규모 데이터셋에 쓰기 전 해당 후보들의 결과/접촉/변위를 확인한다.
시간 제한은 자동으로 설정하지 않는다. 재개는 동일 설정에 `--run-dir <기존 GPU 폴더> --resume`을 추가한다.

출력:

- `backend.json`: 실제 활성화된 GPU dynamics / GPU broadphase / readback 설정
- `scene_0001/observation_0001/`: 접근 전 full/local RGB-D, 카메라와 초기 관절 정보
- `results/candidate_*/`: 후보, 계획 명령, 전체 실행 trace, native contacts
- `candidates.csv`, `candidates.jsonl`, `summary.json`, `candidate_results.png`
- `reset_validation.json`: 반복 reset, 독립 reset, 초기 clone/무동작 일치 검사
- `source_sha256.json`: 기존 물리/자산과 GPU 어댑터의 fingerprint

## 실시간 4×4 화면

물리 환경은 같은 좌표에서 독립적으로 계산하고, 원본 로봇/과실의 현재 자세와
각 환경의 탄성 줄기 상태를 충돌 없는 화면용 모델에 표시한다. 최대 16개씩 보여준다.
온실 배경은 격자 화면에서 숨기지만 원래 물리 장면과 충돌은 유지한다.
데이터셋 RGB-D는 화면용 모델을 만들기 전에 원래 장면에서 저장한다.

```bash
# 물리 환경 16개, 서로 다른 후보 16개를 실행하고 결과 화면을 열어 둠
DISPLAY=:0 ./nvidia-sim/view_gpu_candidates.sh

# 화면 기능을 켜고 총 64개 후보를 16개씩 실행
DISPLAY=:0 ./nvidia-sim/view_gpu_candidates.sh --candidates 64

# 화면 갱신 부담을 줄이려면 (물리 시간 간격/모션 속도는 바뀌지 않음)
DISPLAY=:0 ./nvidia-sim/view_gpu_candidates.sh --view-fps 2
```

초기화와 RGB-D/reset 검증 뒤 4×4 화면이 나타난다. 후보들을 계획한 뒤 물리 시간을
함께 전진시킨다. 계획에 실패한 후보는 해당 칸에 실패로 표시하고 움직이지 않는다.
완료된 환경은 다음 배치의 reset까지 그 장면에 남으며 결과를 표시한다.

- `Parallel tomato tests` 창: env 번호, 후보 번호, 진행/종료 상태.
- 행 클릭: 해당 환경의 타겟/고리 부위 확대. `Overview`: 전체 보기.
- `Previous 16` / `Next 16`: 물리 환경이 16개보다 많을 때 표시 페이지 변경.
- `Pause` / `Resume`: 모든 환경의 물리 진행을 일시 정지/재개.
- `Save screenshot`: 실행 폴더에 viewport PNG 저장. 시작/종료 시에도 자동 저장.
- 완료 후 Isaac Sim 전체 창을 닫으면 종료한다. 계속 계산 중 닫거나 Ctrl+C를 누르면 완료된 후보만 보존한다.

직접 실행기에서는 `--view-grid --keep-open`을 추가하면 된다. `--view-grid`가 GUI를
자동으로 켠다. 변형 형상은 최소 16 physics step(한 제어 주기) 간격으로 갱신하며,
화면 갱신은 기본 최대 5회/벽시계 초이다. 실제 속도는 장면/장비에 따라
더 낮을 수 있다. 선택한 물리 주기와 모션 명령은 화면 갱신 때문에 변경되지 않는다.

검증: `runs/20260921_grid_view_smoke/`에서 16개 화면 생성/렌더 전후 모든 물리 상태가
일치했고, 화면 트리에 Physics/PhysX schema가 없는 것을 검사했다.
기본 물리 격자 배치(2.5 m 간격)도 먼저 시험했으나 16 step 후 일부 환경의 preload와
각속도가 기존 clone 일치 기준을 벗어나 화면용 모델 방식을 선택했다.
`--diagnostic-grid-spacing`은 probe 전용으로 남기며 데이터 생성기에 적용하지 않는다.
`runs/20260921_grid_view_motion_v2/`에서는 서로 다른 후보 16개를 각각 64개 제어 스텝
동시 실행했고, 최종 관절 자세도 16개 모두 달랐다. 확인용 제한에 따라 모두 `incomplete`로
저장했다. 이 시험의 초기화는 약 112초, 계획/reset/동작 구간은 약 421초였다.
실시간 재생 속도를 보장하지 않으며 고리걸기 성공을 검증한 실행도 아니다.
실험 기록은 [gpu_grid_view.json](validation/gpu_grid_view.json)을 참고한다.
화면은 16환경에서 검증했으며 1,000환경 GUI의 메모리/처리량 검증과는 구분한다.

## 구현 선택과 제한

CUDA tensor contact view 시험에서 collider를 sensor로 지정하면 view 생성이 실패했고,
filter로 지정해도 `GPU contact filter for collider ... is not supported` 경고가 발생했다.
몸체만 구분하면 동일한 몸체에 붙은 과실과 꼭지를 혼동하거나 고리 뒷부분 접촉을
확정할 수 있으므로, 추정으로 성공 라벨을 만들지 않고 native readback을 유지한다.
참고: [Omni Physics tensor contact API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html).

`gpu_batch_views.py`는 환경별 asset adapter를 유지하면서 동일 종류의 articulation을
한 번에 조회한다. 물리 step 및 모든 local write 때 cache를 무효화한다.
`gpu_dataset_scene.py`는 매 물리 스텝의 effort를 두 batch 호출로 전달하며, drive target은
변경될 때 전송한다. 외력이나 explicit actuator가 추가되면 이 경로는 오류로 중단한다.

GPU worker는 기존 worker의 실험용 분기다. 계획/접촉 판정/식물/카메라 모듈은 공유한다.
기존 CPU dataset의 resume fingerprint를 바꾸지 않기 위해 CPU worker를 수정하지 않았다.
추후 backend가 검증되면 별도의 버전 마이그레이션과 함께 공통 worker로 통합할 수 있다.

`gpu_probe.py --batched-io --gpu-solver pgs`로 저장된 CPU 명령의 비교 시험을 실행할 수 있다.
현재 병렬 물리 실행이 된다는 사실과 CPU pool보다 빠르다는 것은 별개다.
성공률, 고리 접촉 정확도, end-to-end 처리량을 검증하기 전에는 CPU 데이터를 대체하지 않는다.

## 대규모 환경 수정 (2026-09-21)

기존 64환경의 실패는 VRAM 부족이나 상태 배열의 인덱스 혼선이 아니었다.
원래 asset view와 batch view의 조회 값, 복제 질량·관성·구동 설정이 같아도
GPU constraint partition 8에서는 첫 step부터 환경별 식물 상태가 달라졌다.
같은 장면에서 `gpu_max_num_partitions=1`로 바꾸면 재현 시험의 차이가 사라졌다.
이 설정을 GPU 기본값으로 사용하며 검증 기준은 완화하지 않는다.
PhysX 내부 결함의 일반적인 원인까지 규명했다는 뜻은 아니다.

환경 수 상한은 1,024이다. 수를 늘릴 때 설정을 임의로 다시 조절하는 대신 다음을 적용한다.

- PhysX native replication과 환경 ID로 겹쳐 놓은 환경을 broadphase부터 분리한다.
  USD collision group만 사용한 1,000환경에서는 수천만 개 overlap pair의 버퍼 부족이
  발생했다. ID 비트 수는 환경 수에 따라 계산한다 (64→7, 128→8, 1,000→10).
- 정확한 prim 경로는 전체 stage 검색 없이 조회한다. 같은 초기 장면의 FK/중력 preload
  계산을 재사용하고, reset의 전역 forward/update를 묶는다. 각 환경의 물리 상태는 독립이다.
- 전체 reset의 harvest joint 제거 중에만 Isaac asset 삭제 알림을 잠시 멈춘다.
  1,000환경에서 11,000개 조인트 삭제가 13,000개 asset의 regex callback을 각각 호출하는
  병목을 피한다. 삭제 대상은 별도 harvest joint인지 검사하며 PhysX USD listener는 유지한다.
  실제 조인트는 제거·재생성한다. asset 자체를 삭제할 때 이 최적화를 사용하지 않는다.
- headless 129환경 이상에서는 동일 장면의 RGB-D를 단일 환경 프로세스에서 먼저 저장하고
  그 프로세스가 종료된 뒤 N환경 물리 프로세스를 띄운다. 렌더러가 대규모 장면을 함께
  복제하지 않게 한다. 사진/물리 소스 SHA와 초기 상태를 비교한 뒤에만 공유한다.
  두 프로세스는 순차 실행이며 물리 worker pool이 아니다.
- PhysX buffer overflow, interaction 누락, CUDA 오류가 출력되면 실행을 중단하고
  `execution_error.json`에 `dataset_valid: false`를 남긴다.

현재 확인한 범위:

- 64·128환경 각각 무동작 960 physics step: 복제 위치/속도 차이 0, 파단 없음.
- GPU 한 환경과 64환경에서 저장된 후보 3번의 동일 명령 260 control step 재생:
  각 환경의 관절, 목표/주줄기 변위, 고리 간격, 최종 식물/과실 위치 비교 값이 모두 일치.
  native 환경 ID를 적용한 64환경에서도 동일하다. **서로 다른 64종 후보 시험은 아니다.**
  결과는 모두 `excessive_displacement`이므로 수확 성공을 의미하지 않는다.
- 128환경 RGB-D/reset 검증 및 129환경의 순차 RGB-D/물리 프로세스 검증 통과.
- 최종 reset 경로의 16환경 native 파단 후 복원 검증 통과 (32 articulations, 176 constraints).
- **1,000환경 최종 검증 통과**: 초기화, 반복/독립 reset, 순차 RGB-D 공유,
  16 physics step의 clone 일치, 실행 후 reset까지 완료했다. 위치·관절 상태 차이 0,
  의도하지 않은 파단 0, contact routing 오류 0이다. 전체 시작 검증은 약 575.7초였다.
  후보 trajectory를 실행한 검증은 아니다. 초기화 중 관측 GPU 메모리는 전체 약 4,930 MiB
  (최댓값 측정 아님), 종료 후 1,699 MiB였다. 소유한 시험 프로세스는 모두 종료했다.
- 자동 테스트 33개 통과. 자세한 검증 결과와 근거는
  [gpu_scaling.json](validation/gpu_scaling.json)에 기록한다.

CPU TGS 대비 동등성은 여전히 미달이다. 새 partition 1의 후보 3번은 주줄기 변위 차이가
최대 1.302 mm여서 1 mm 기준을 넘는다. GPU 내부의 환경 수 재현성과 CPU 동등성은 별개다.
1,000개의 서로 다른 후보 전체 완주, 다양한 접촉 경계의 성공 라벨, 시간당 처리량은
아직 검증하지 않았다. 초기화 비용과 CPU IK/FCL/접촉 판정도 남아 있어 환경 수에 비례한
속도 향상을 약속하지 않는다.

```bash
# 1,000환경 초기화·reset·RGB-D 공유·짧은 물리 검사 (후보 실행은 안 함)
./nvidia-sim/run_gpu_candidate_dataset.sh --num-envs 1000 --candidates 1000 --validate-only

# 128환경에서 총 1,000후보 처리
./nvidia-sim/run_gpu_candidate_dataset.sh --num-envs 128 --candidates 1000

# 1,000환경에서 총 1,000후보 처리: 대규모 전체 동작은 추가 검증 대상
./nvidia-sim/run_gpu_candidate_dataset.sh --num-envs 1000 --candidates 1000
```

## 과거 검증: 수정 전 GPU partition 8 (2026-09-20)

- 저장된 후보 `candidate_00003` 재생: GPU 1환경 209.95초, GPU 16환경 288.34초.
  16개 모두 260 제어 스텝에서 `excessive_displacement`. 접촉 대상, 파단 이벤트 수집과
  접촉/파단 후 reset 검사를 완료했다. 이는 **16종류 후보가 아니라 동일 후보의 16개 복제 시험**이다.
- Native 통계: 32 articulations, 176 constraints, GPU articulation heap 사용 확인.
- CPU TGS와 GPU PGS는 해당 후보의 최초 접촉 대상/종료 분류가 같지만 주줄기 변위 차이가
  최대 1.092 mm로 기존 1 mm 비교 기준을 넘었다. 엄격한 동등성 검사는 탈락이다.
- GPU clone끼리도 최대 고리 간격 차이 1.434 mm가 관측되었다. 결과 분류는 같지만
  접촉 경계에서의 성공 안정성이 같다고 보장하지 않는다.
- GPU의 단일 세션 batching 자체는 동작하나, 검증된 기존 CPU 4-process replay의
  4후보/27.43초 처리량보다 이 GPU 16환경 시험의 처리량이 낮다. GPU 가속 목표는 아직 미달이다.
- TGS 연결 조인트의 body0/body1을 뒤집거나 모든 축을 잠근 D6로 바꾼 진단도
  첫 물리 스텝에 11개 연결이 끊어졌다. 진단 변형은 GPU 데이터 실행기에 적용하지 않는다.
- Fabric을 켠 채 기존 FCL 모델을 초기화하면 오래된 USD 자세를 읽어 검사가 실패했다.
  데이터 실행기는 USD readback을 유지한다 (`use_fabric=False`). GPU dynamics는 그대로 켠다.

실측 기록: [gpu_batched_pipeline.json](validation/gpu_batched_pipeline.json).

### GPU 전용 초기 동작 검사

초기 reset은 모든 상태에 대해 기존 `1e-6` 기준을 유지한다. 짧은 무동작 실행 후의
비교에는 `gpu_validation.py`의 단위별 기준을 사용한다. 기존 CPU 검사처럼 위치·속도·
각속도 전체에 숫자 `0.002` 하나를 적용하지 않는다.

- 위치 차이: 0.1 mm 이하
- 회전 차이: 0.001 rad 이하
- 선속도 차이: 0.001 m/s 이하, 각속도 차이: 0.02 rad/s 이하
- 식물 관절 위치/속도 차이: 0.001 rad / 0.01 rad/s 이하
- 로봇 상태와 강성/감쇠/preload는 `1e-6` 비교, 비유한 값·파단은 실패

16환경의 저장된 초기 동작 상태는 위치 차이 0.0077 mm, 최대 각속도 차이
0.0126 rad/s로 이 검사를 통과했다. 원래 혼합 단위 검사 결과도 metadata에 남긴다.
이것은 **초기 동작 screening**이며 접촉 궤적의 CPU 동등성 검사와 구분한다.
CPU 재생 비교의 1 mm 기준이나 고리 성공 조건은 완화하지 않는다.

수정 전 partition 8의 64환경은 초기화·RGB-D·독립 reset까지 실행했으나 초기 동작에서 식물 관절 차이가
최대 0.0048 rad / 0.684 rad/s로 나타나 새 기준으로도 실패 범위다.
64환경 데이터 생성이 검증됐다는 뜻이 아니다. 해당 실행은 후보를 저장하기 전에 중단됐다.
64환경 초기화에는 약 5분이 소요됐다. 세션 복제·초기화 비용도 별도의 최적화 대상이다.

모든 clone은 작은 좌표를 유지하기 위해 같은 위치에 둔다. 현재는 native 환경 ID로
broadphase 단계부터 격리하며 USD collision group도 유지한다.
화면에는 env_0만 표시한다. GUI에 로봇 한 대만 보여도 물리 환경 수는 backend/검증 기록에서 확인한다.

### End-to-end 데이터 저장 확인

`runs/20260920_225424_gpu_16env_dataset_smoke/`에서 16환경 GPU 장면을 생성하고
후보 3번 한 개를 RGB-D 촬영부터 결과 저장까지 실행했다. 나머지 15개는 이 시험에서
새 후보를 실행하지 않았다. 초기화 41.52초, 후보 실행/계획/reset 구간 261.13초,
결과는 `excessive_displacement`이며 단위별 초기 동작 검사와 독립 reset을 통과했다.
앞의 16개 동시 재생 시험과 구분한다. 이 숫자를 16개 후보 처리 시간으로 해석하면 안 된다.

GPU 결과도 다음과 같이 대시보드로 볼 수 있다.

```bash
python nvidia-sim/rl/dataset_dashboard.py <GPU 실행 폴더>
```

HTML에는 GPU/PGS 실험용이라는 표시가 포함된다. 원본 CSV/JSON/사진은 수정하지 않는다.

## CPU 경로 병렬화와 물리 주파수 실험

이제 경로 계산은 기본 최대 8개 CPU worker에서 미리 수행한다.
`--planning-workers 0`으로 기존 순차 방식을 선택할 수 있다.
`DATASET READY`는 후보 경로 준비 완료이며 spawn 로그가 아니다.
물리 기본값은 현재 기능 검사를 거친 `practical60`이다. `--physics-hz`만 바꾸면
preset의 나머지 물성은 유지된다. 낮은 주파수에서는 탄성/접촉 결과가 달라질 수 있다.
자세한 명령과 검증 범위는 [물리 주기·경로 병렬화](GPU_TIMESTEP_AND_PLANNING.md)를 참고한다.
