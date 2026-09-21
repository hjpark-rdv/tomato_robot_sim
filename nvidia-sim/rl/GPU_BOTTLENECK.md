# GPU 실행 병목 실측 — 2026-09-21

32환경이 한 개씩 32회 실행하는 것과 같은 속도는 아니었다. 동일 명령 재생에서
처리량은 14.69배 증가했다. 다만 GPU 관절 제약 계산 자체가 느리고, 주요 커널이
아주 적은 작업 블록으로 실행돼 GPU 전체 자원을 충분히 쓰지 못했다.

## 동일 궤적 완주 비교

기존 `candidate_00003`의 저장된 명령을 사용했다. 화면·카메라 렌더링 없이
GPU PGS, 960 Hz, position 64 / velocity 4 iterations, partition 1,
native environment IDs, CPU collider readback, batched I/O를 유지했다.
로봇 속도·식물 모델·접촉/파단 조건은 바꾸지 않았다.

| 측정 | 1환경 | 32환경 |
|---|---:|---:|
| 실행한 동작 수 | 1 | 32 |
| 각 동작의 시뮬레이션 시간 | 4.333초 | 4.333초 |
| 실행 시간(시작·계획 제외) | 156.64초 | 341.16초 |
| 물리 스텝 수 | 4,160 | 4,160 |
| `fetch_results` 시간 | 152.93초 | 288.91초 |
| 위 호출의 메인 스레드 CPU 시간 | 1.96초 | 30.15초 |
| 실행 전체의 프로세스 CPU 시간 | 181.43초 | 381.97초 |

시간은 2.18배, 작업량은 32배: **처리량 14.69배**.
1환경 동작을 32회 순차 반복하면 계산상 약 83.54분이며, 32환경 실측은 5.69분이다.
이는 **같은 조기 종료 사례를 복제한 비교**다. 다양한 pose의 완료 시간이나
1,000개 pose의 예상 시간으로 그대로 적용하면 안 된다.

모든 32환경의 명령·접촉 대상·종료 분류·관절/변위 trace·최종 상태 비교가 통과했다.
초기 상태 차이는 0이었다. 모두 260 control steps 후 `excessive_displacement`로 종료했다.
기준 CPU TGS와 GPU PGS의 동등성을 검증한 실험은 아니다.

## CUDA 추적에서 확인한 비용

처리량 측정과 별도의 실행에서 **처음 32 physics steps만** CUPTI/Kineto로 기록했다.
이 추적은 오버헤드가 있으므로 처리량 표에 사용하지 않았다.

| GPU 커널 | 1환경 GPU 커널 시간 비중 | 32환경 GPU 커널 시간 비중 |
|---|---:|---:|
| `artiSolveInternalConstraints1T` | 71.52% | 73.77% |
| `artiPropagateRigidImpulsesAndSolveSelfConstraints1T` | 9.59% | 8.06% |
| `artiPropagateImpulses2PGS` | 6.38% | 6.29% |
| `doSelfCollision` | 3.81% | 3.40% |

비중의 분모는 기록된 **GPU 커널 duration 합계**이며 전체 프로그램 시간이 아니다.
첫 번째 커널은 32 physics steps 동안 2,176번, 스텝당 68번 호출됐다.
grid는 1환경에서 `[1,1,1]`, 32환경에서 `[2,1,1]`, block은 둘 다 `[32,1,1]`였다.
장치에는 SM 84개가 있지만, 이 커널 호출은 작업 블록이 1~2개뿐이다.
이는 GPU가 작업 중이어도 전체 연산 자원을 충분히 쓰지 못할 수 있는 구체적인 증거다.
하드웨어 occupancy 카운터를 직접 측정한 것은 아니다.

환경 하나에 로봇 7 DOF와 식물 234 DOF(몸체 79개), 열매 11개가 있다.
복잡한 탄성 식물이 큰 비용을 만들 것으로 추정되지만, 위 커널은 로봇과 식물을
함께 처리하므로 식물만의 시간 비중은 이 실험에서 분리하지 않았다.

`fetch_results`를 모두 CPU readback 시간으로 해석하면 안 된다. 그 안에서
GPU 작업 완료·내부 작업 스레드·접촉 콜백을 기다린다. 호출 스레드의 CPU 시간과
프로세스 전체 CPU 시간도 다르다. CPU 120%라는 수치만으로 원인을 정할 수 없다.
이번 CUDA 기록은 GPU 관절 솔버에 실제로 큰 비용이 있다는 점을 추가로 확인했다.

## 다음 최적화 우선순위

1. 128/256/512환경에서 같은 짧은 명령 구간을 재생해 블록 수·스텝 비용·상태 일치를
   비교한다. 비용이 큰 전체 후보 탐색은 효율이 확인된 뒤 재개한다.
2. 전체 배치가 끝날 때까지 기다리는 비용을 줄이는 방식을 검토한다.
   개별 환경 reset이 다른 환경의 제약조건/접촉에 영향을 주지 않는지 검증해야 한다.
3. 960 Hz·솔버 반복 횟수·식물 관절 모델 변경은 물리 결과를 바꿀 수 있으므로,
   별도의 정확도/접촉/파단 검증 없이 속도 설정으로 적용하지 않는다.

CPU 코어 수 확대나 Python 루프 개선만으로 주요 GPU 커널 비용이 사라지지는 않는다.
이 작업에서는 성능을 위해 물리 설정을 변경하지 않았다.

## 재현과 기록

- 원본 결과: `runs/20260921_132000_gpu_bottleneck/`
- 요약: [validation/gpu_bottleneck.json](validation/gpu_bottleneck.json)
- `gpu_01/`, `gpu_32/`: 전체 명령 재생, `command.json`, native timing, trace, 접촉 로그.
- `replay_comparison.json`: 모든 32환경의 기준 궤적 비교.
- `cuda_trace_01/`, `cuda_trace_32/`: 별도 짧은 진단 실행.
  `cuda_trace.json`은 Perfetto 등 Chrome trace 호환 뷰어로 확인할 수 있다.
- `cuda_summary_01.json`, `cuda_summary_32.json`: 커널별 집계와 grid/block.

`gpu_probe_worker.py`에 `--profile-motion`을 주면 기존 native simulate/fetch 호출의
wall/process CPU/caller CPU 시간을 기록한다. 추가 GPU 동기화는 하지 않는다.
`--python-profile-motion`은 cProfile 파일을 남기며, 프로파일링 오버헤드가 있다.
`gpu_cuda_trace.py`는 같은 인자와 `--profile-motion`을 받아 처음 32 physics steps의
CUDA trace를 남긴다. 짧은 진단에는 `--max-control-steps 4`를 사용했다.
각 실행 폴더의 `command.json`에 실제 전체 인자가 저장되어 있다.

기존 `20260921_gpu_throughput_32_64_128` 비교는 31/128개 완료 상태에서 진단을 위해
중단했다. 완료 후보와 로그는 보존했으며 64/128환경 실행을 자동 재개하지 않았다.
이전 실험 폴더의 `diagnostic_interruption.json`에 중단 시점과 이유를 기록했다.

참고: [NVIDIA Isaac Sim 5.1 성능 안내](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/reference_material/sim_performance_optimization_handbook.html).
위 수치는 문서의 일반 권장값이 아니라 현재 장면에서 수집한 실측값이다.
