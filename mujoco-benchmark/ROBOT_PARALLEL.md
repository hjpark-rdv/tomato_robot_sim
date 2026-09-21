# 로봇 포함 MuJoCo CPU 병렬 기준선

2026-09-22. Threadripper 3960X, 24코어/48스레드, RAM 128GB급. MuJoCo 3.13.0.

기존 최적화 식물 + 실제 Isaac USD 로봇 질량/관성/관절/충돌 형상. 하우스 없음. 로봇 7축, 총 241 DOF. 고리는 mocap이 아니라 link6에 고정. CPU 120Hz, implicitfast/Newton 64회, 기존 friction 및 스프링 설정 유지. 로봇 자체 충돌은 원본 USD처럼 비활성화, 로봇-식물 충돌은 활성화. 최적화 식물은 원본과 구조가 다르며 파단은 비활성화.

각 프로세스가 동일 candidate_00049 관절 명령을 16.5초 재생. 프로세스당 2초 별도 워밍업 후 3회 반복. 환경 수만큼 같은 작업을 처리하는 weak-scaling 시험이며 서로 다른 후보 탐색이나 RL 학습이 아니다. 경로/IK는 기존 기록을 사용해 측정 범위에서 제외. 렌더링 없음.

| CPU 프로세스 | 한 회 평균 ± 표준편차 (초) | 전체 처리량 (sim-s/s) | 작업자 RSS 합계 최대 (GiB) |
|---:|---:|---:|---:|
| 1 | 4.194 ± 0.026 | 3.93 | 0.62 |
| 2 | 4.195 ± 0.005 | 7.87 | 1.25 |
| 4 | 4.247 ± 0.002 | 15.54 | 2.49 |
| 8 | 4.324 ± 0.011 | 30.53 | 5.45 |
| 16 | 4.459 ± 0.017 | 59.21 | 9.96 |
| 24 | 4.752 ± 0.063 | 83.35 | 16.84 |
| 32 | 5.453 ± 0.032 | 96.83 | 21.59 |
| 48 | 6.443 ± 0.152 | 122.97 | 33.57 |
| 64 | 8.630 ± 0.195 | 122.41 | 40.22 |

**검증한 최대 동시 실행 수: 64. 권장 시작점: 48.** 64개에서 처리량은 48개와 비슷하고 메모리와 개별 지연은 늘었다. 64개는 하드웨어 한계가 아니라 이번 시험 상한이다.

원본 데이터: `outputs/20260922_robot_cpu_scaling`, `outputs/20260922_robot_cpu_scaling64`. 각 폴더에 manifest, 모든 반복/작업자 시간, CPU/RAM/GPU 시계열, CSV가 있다. 구간은 모델 로딩, 워밍업, 배리어 대기, 명령 보간, reset, 제어 업로드, 물리+기구학, 접촉·변위 진단, 기록, 저장으로 분리했다. GPU 컴파일·렌더링·IK는 해당 없는 구간이다.

상태 발산 경고는 모든 반복에서 0. 성공 판정은 수확 성공이 아니라 접촉/변위 진단만 제공한다. 현재 hook_contact_steps는 비활성 접촉 후보를 포함하는 native contact 목록 수이며 실제 힘이 발생한 횟수로 해석하지 않는다. 별도 전체 궤적 기하 검사를 수행하며 이는 엔진 간 동등성 인증이 아니다.

## 재현

```bash
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/build_robot_model.py
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/benchmark_robot_pool.py \
  --workers 1,2,4,8,16,24,32,48,64 --repeats 3
```

화면에서 로봇과 식물 보기:

```bash
DISPLAY=:0 mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/robot_viewer.py
```

영상/상태/기하 검사 저장:

```bash
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/robot_viewer.py \
  --output "mujoco-benchmark/outputs/$(date +%Y%m%d_%H%M%S)_robot_video"
```

대형 XML/MJB 및 상태/영상은 git 제외. 기존 reference.json/robot_export_only.usda 생성은 README의 원본 추출 절차를 사용한다. 동작 입력은 기존 candidate_00049 trace.json이며 실행 manifest에 경로와 해시를 보존한다.
