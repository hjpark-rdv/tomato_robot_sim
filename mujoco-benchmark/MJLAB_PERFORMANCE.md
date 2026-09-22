# mjlab / CPU 동일 로봇 성능 비교

CPU 기준선 커밋: `38bac4d` (`mujoco-sim`). 후속 작업 브랜치: `mjlab-performance`.

## 구성

- mjlab 1.6.0의 `Simulation` 계층을 실제로 사용한다. 단순히 MJWarp만 직접 호출하고 mjlab 환경이라고 부르는 구조가 아니다.
- RL 학습/매니저 기반 RL 태스크는 만들지 않았다. 같은 관절 명령 재생, reset, 접촉·변위·비정상 상태 진단을 제공한다.
- MuJoCo/MJWarp 3.11.0, Warp 1.17.0, PyTorch 2.7.0+cu128. 전체 의존성은 `mjlab-requirements.lock.txt`.
- 로봇/작물은 CPU 기준 모델과 동일. 7축 로봇과 234개 식물 회전 자유도, 총 241 DOF. 하우스 없음.
- 120Hz, implicitfast, sparse Newton, 64회 반복 상한, elliptic friction. 질량/관성/스프링/마찰을 성능 목적으로 변경하지 않았다.
- MJWarp에서 지원하지 않는 `mjDSBL_AUTORESET`만 공통 호환 모델에서 제거했다. 동일한 모델을 MuJoCo 3.11 CPU에도 사용한다. 기존 3.13 CPU 모델은 유지한다.
- 초기 상태, preload, 관절 명령은 공통 `RobotEngine`에서 만든다. GPU 제어·변위·접촉 통계는 전체 world에 일괄 처리하고 매 스텝 CPU로 각 환경을 읽지 않는다.
- 모든 반복은 같은 49번 입력을 16.5초 재생한다. 무작위 환경이나 다른 경로를 생성하는 시험이 아니며, 처리량을 비교하는 weak-scaling 기준선이다.

## 실행

기존 로봇 기준 모델이 만들어진 상태에서:

```bash
./mujoco-benchmark/setup_mjlab.sh
mujoco-benchmark/.mjlab-venv/bin/python mujoco-benchmark/scripts/run_mjlab_scaling.py \
  --envs 1,8,32,128,256,512,1024 --repeats 3
```

GPU 단일 설정:

```bash
mujoco-benchmark/.mjlab-venv/bin/python mujoco-benchmark/scripts/benchmark_mjlab_robot.py \
  --num-envs 128 --output "mujoco-benchmark/outputs/$(date +%Y%m%d_%H%M%S)_mjlab128"
```

버전까지 맞춘 CPU 비교:

```bash
mujoco-benchmark/.mjlab-venv/bin/python mujoco-benchmark/scripts/benchmark_robot_pool.py \
  --model mujoco-benchmark/models/robot_plant_mjlab.mjb --workers 1,24,48 --repeats 3
```

GPU는 화면 없이 측정한다. CPU 원본 모델/GUI/벤치마크는 `.venv`, mjlab 및 호환 모델은 `.mjlab-venv`에서 실행한다. 다른 버전으로 저장된 MJB를 섞지 않는다.

## 저장 범위와 비교 한계

- 모델·명령 파일 해시, 버전, 실행 인자, 코드 커밋, UTC 및 monotonic 시각.
- CPU: 작업자별 모델 로딩, 워밍업, 명령 보간, 대기, 초기화, 제어, 물리+기구학, 접촉·변위 진단, 각 반복/프로세스 전체 시간.
- GPU: 모델 로딩, 장치 준비/커널 컴파일/그래프 생성, 접촉 진단 커널 준비, 워밍업, 명령 업로드 준비, 초기화, CUDA event별 제어/물리/판정, 상태 저장, 반복 시간. 실행기 전체 시간은 별도 `*_process.json`.
- CUDA event 시간은 장치 실행 구간이며 Python 벽시계와 더해서 사용하지 않는다. CPU/GPU 진단 구현이 다르므로 통계 수집 비용 자체가 완전히 동일하지는 않다.
- CPU RAM은 작업자 RSS 합계, GPU 실행 RAM은 해당 프로세스 RSS. GPU 메모리는 디스플레이 등 다른 사용량을 포함한 시스템 전체 값이며 1초 샘플 최대치다.
- CPU startup에는 프로세스 시작/모듈 import가 포함된다. GPU 준비 항목의 합계는 최초 import를 제외하므로 전체 소요 비교에는 process_wall_s를 사용한다.
- 각 설정 3회 개별 기록과 평균/표준편차를 저장한다. GPU world0는 모든 물리 스텝 qpos, 모든 world는 최종 qpos와 변위/접촉 통계를 저장한다. 전체 world의 모든 시점 원본 접촉점/힘을 저장하지는 않는다.
- MuJoCo 3.13과 3.11의 성능을 혼합하지 않는다. 같은 3.11로 별도 CPU 재측정했다.
- MJWarp의 일부 capsule–mesh 다중 접촉 지원 차이와 float32 정밀도 때문에 CPU와 수치가 완전히 같지는 않다. 접촉/변위 오차를 별도 분석한다.
- 최적화 식물의 열매 파단은 비활성화되어 있다. 발산/오버플로 없음은 고리걸기 성공이나 sim-to-real 정확도를 뜻하지 않는다.
- 가장 큰 시험 환경 수는 측정 상한이다. OOM까지 늘려 찾은 절대 최대치라고 표현하지 않는다.

공식 자료: https://github.com/mujocolab/mjlab , https://mujoco.readthedocs.io/en/latest/mjwarp/

## 측정 결과 (2026-09-22)

모든 설정은 16.5초 동작을 3회 반복했다. 아래 실행 시간은 로딩/컴파일/워밍업/저장을 제외한다.

| 구성 | 환경 수 | 한 회 평균 | 전체 처리량 |
|---|---:|---:|---:|
| CPU 3.13.0 | 1 | 4.194초 | 3.93 sim-s/s |
| CPU 3.13.0 | 24 | 4.752초 | 83.35 sim-s/s |
| CPU 3.13.0 | 48 | 6.443초 | 122.97 sim-s/s |
| CPU 3.13.0 | 64 | 8.630초 | 122.41 sim-s/s |
| CPU 3.11.0 | 1 | 4.331초 | 3.81 sim-s/s |
| CPU 3.11.0 | 24 | 5.236초 | 75.62 sim-s/s |
| CPU 3.11.0 | 48 | 12.171초 | 65.08 sim-s/s |
| mjlab/MJWarp 3.11.0 | 1 | 9.395초 | 1.76 sim-s/s |
| mjlab/MJWarp 3.11.0 | 8 | 10.757초 | 12.27 sim-s/s |
| mjlab/MJWarp 3.11.0 | 32 | 12.297초 | 42.94 sim-s/s |
| mjlab/MJWarp 3.11.0 | 128 | 20.011초 | 105.54 sim-s/s |
| mjlab/MJWarp 3.11.0 | 256 | 32.030초 | 131.88 sim-s/s |
| mjlab/MJWarp 3.11.0 | 512 | 74.653초 | 113.16 sim-s/s |
| mjlab/MJWarp 3.11.0 | 1024 | 179.960초 | 93.89 sim-s/s |

- 최대 검증 수: CPU 64프로세스, GPU 1,024환경. 절대 메모리 한계까지 실행한 것은 아니다.
- 측정 범위 최적: MuJoCo 3.13 CPU 48프로세스, MuJoCo 3.11 CPU 24프로세스(1/24/48 비교), mjlab GPU 256환경.
- GPU 256 처리량은 같은 버전 CPU 24 대비 약 1.74배, 기존 3.13 CPU 48 대비 약 1.07배다. 버전 차이 때문에 두 비교를 섞지 않는다.
- GPU 512/1,024는 메모리 부족 없이도 처리량이 내려갔다. 1,024의 시스템 GPU 메모리 샘플 최대 약 6.5GiB, GPU 256은 약 3.25GiB.
- 모든 측정 반복에서 상태 비정상/버퍼 오버플로 0. 물리 정확도/수확 성공을 인증하는 의미는 아니다.
- 같은 MuJoCo 3.11 CPU와 GPU의 world0 매 물리 스텝을 비교한 최대 위치 차이: 식물 body 원점 0.1906mm, 고리 원점 0.0153mm. 모든 GPU world의 전체 시간 궤적을 비교한 것은 아니다.
- 최초 비캐시 GPU 준비/컴파일 84.61초(단일 smoke). 이후 캐시 사용 측정의 준비/컴파일 시간은 별도 summary에 기록.

원본 데이터:

- `outputs/20260922_robot_cpu_scaling`, `outputs/20260922_robot_cpu_scaling64`
- `outputs/20260922_robot_cpu_mjlab_matched`
- `outputs/20260922_mjlab_robot_scaling`
- `outputs/20260922_robot_backend_equivalence`
- 비교 화면/CSV/JSON: `outputs/20260922_robot_backend_comparison/index.html`

커밋에 포함되는 요약: `validation/mjlab_robot_scaling.json`, `validation/mjlab_robot_equivalence.json`. 대용량 상태·영상·자원 시계열은 위 outputs에 보존하고 git에는 넣지 않는다.

## 운영 권장 및 후속 작업 (2026-09-22)

대량 경로 탐색을 이어서 구현할 기본 구성은 **GPU 256환경**을 권장한다.
현재 측정 처리량은 CPU 3.13의 48프로세스보다 약 7% 높으며, 압도적인 속도 차이는 아니다.
소수 경로 확인과 디버깅은 CPU가 적절하고, 기존 CPU 대량 실행은 48프로세스를 기준으로 유지한다.
GPU 환경 수를 512 또는 1024로 올리는 것은 현재 모델에서는 처리량 개선이 아니다.

이 권장은 동일한 candidate_00049 관절 명령을 반복 재생한 결과에 기반한다.
서로 다른 후보의 경로 계산, 실행 길이 차이, 초기화, 판정, 파일 저장을 포함한
실제 탐색 시스템 전체 처리량은 아직 측정하지 않았다. 1000개 탐색 완료 시간을
위 물리 재생 처리량만으로 확정하지 않는다.

### 호환 모델 변경의 의미

- 원본 모델의 `autoreset="disable"`은 MJWarp에서 지원하지 않아,
  별도 호환 모델에서만 `autoreset="enable"`로 변경했다.
- 같은 MuJoCo 3.11 비교에서는 CPU와 GPU 모두 이 호환 모델을 사용했다.
  형상·질량·제어값은 유지했으며 원본 MuJoCo 3.13 모델은 보존했다.
- 미지원 기능 자체를 구현한 것은 아니다. 비정상 상태 발생 시 오류 처리 의미까지
  원본과 동일해진 것은 아니며, 이번 정상 실행 검증에서는 비정상 상태/overflow가 없었다.

### 다음 단계

1. GPU 256환경에서 서로 다른 후보를 배정하고 기존 경로 생성·평가와 연결한다.
2. 로딩을 포함한 전체 시간과 순수 실행 시간을 분리하여 소규모 후보 묶음으로 재측정한다.
3. 대표 성공/실패 영상을 확인한 후 대량 탐색 수와 영상 저장 수를 결정한다.

작업 이력: CPU 기준선 `38bac4d`, mjlab GPU 구성·성능 비교 `115560b`.
이번 문서 보완에서는 코드를 변경하거나 벤치마크를 다시 실행하지 않았다.

## 2026-09-22 原STL 그리퍼 외형 복원

- 실제 ROS/Isaac URDF가 참조하는 `assy_gripper_ver_6.stl`을 로봇 모델 생성기에 반영했다.
- 원본 STL 전체 삼각형을 유지하고 mm→m 변환 후 Hook 좌표계에 표시한다. 직접 STL 디코더의 면 수 제한을 피하기 위해 동일 메시를 XML vertex/face로 저장한다.
- 기존 고리/레일/근위부 충돌 프록시는 투명하게 표시하며 물리 설정은 유지한다. 시각 메시에는 충돌과 질량을 부여하지 않는다. 원본 STL과 충돌 프록시가 정확히 일치한다는 검증은 아니다.
- CPU 및 mjlab 호환 XML/MJB를 재생성했다. CPU 0.5초 재생에서 비정상 상태 없음, EGL 렌더링 확인. GPU 재실행 및 성능 재측정은 하지 않았다.
- 확인 이미지: `outputs/20260922_original_gripper_stl/gripper_closeup.png`.
