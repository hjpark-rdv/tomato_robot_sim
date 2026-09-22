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

## CPU 결과 HTML 보고서

`benchmark_robot_pool.py` 완료 시 결과 폴더에 `index.html`을 자동 생성한다.
평균/표준편차, 반복별 시간, 합산 처리량, 비정상 실행 수, 준비/전체 시간,
샘플 기준 작업자 RSS 합계 최대치와 원본 CSV/JSON 링크를 표시한다.
기존 결과는 시뮬레이션을 다시 실행하지 않고 다음 명령으로 변환한다.

```bash
python3 mujoco-benchmark/scripts/cpu_report.py mujoco-benchmark/outputs/20260922_090946_robot_cpu_scaling
```

이번 결과는 48환경 × 3회 동일 경로 재생이며, 다양한 후보의 성공률이 아니다.

## 회차·환경 재연

HTML에서 반복 회차(1부터)와 환경 번호(0부터)를 선택하면 전체 원본 지표와 실행 명령을 볼 수 있다.
`replay_cpu_case.py RUN --workers 48 --repeat 2 --env 17`로 선택 실행을 검증한 뒤 실시간 화면으로 반복 재연한다. `--check-only`는 화면 없이 결과 비교만 수행한다.
모델/명령 해시와 MuJoCo 버전을 확인하고, 최종 고리 위치·최대 목표 변위·접촉 침투·접촉 스텝·비정상/경고 지표를 비교한다. 불일치는 파일로 기록하고 화면 실행을 중단한다.
매 스텝 원본 상태를 재생하는 기능은 아니며, 요약 지표 일치가 전체 궤적의 동일함을 보장하지 않는다. 현재 시험은 동일49번 경로 반복이다.
앞으로 CPU 실행은 `replay_assets`에 모델 MJB·trace·reference 및 Python 소스 사본을 한 번씩 보관한다. 모델 아카이브는 수백 MB 추가된다. 기존 실행은 원본 파일이 남아 있고 해시가 맞을 때만 재연 가능하다. Python 의존 환경은 별도 보존이 필요하며 소스 변경 시 경고한다.
