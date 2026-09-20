# 최소 GPU 물리 비교 시험

원본 greenhouse/로봇/11개 과실/탄성 식물을 그대로 사용한다. 기존 후보의 초기 관절 자세와
저장된 관절 명령을 재생한다. 새 IK/후보 탐색/RGB-D 촬영 시간은 비교 구간에 포함하지 않는다.
현재 본 데이터 생성기는 CPU 그대로이며 GPU를 기본값으로 변경하지 않았다.

기본 GPU 시험은 **GPU dynamics + GPU broadphase, CPU tensor readback**이다.
`--cuda-tensors`로 CUDA 상태 API도 시험할 수 있지만 현재 native collider contact report가 누락되어
이 모드의 라벨/속도 비교는 자동 탈락한다. 모든 환경을 하나의 CUDA view로 묶는 벡터화는 아직 아니며, 이 시험의 속도를
64개 GPU 환경이나 최적화된 Isaac Lab 학습 환경의 처리량으로 일반화할 수 없다.

## 실행

저장소 루트에서 기존 candidate 폴더를 지정한다. 최소 시험은 `Tomato_05`만 지원한다.

```bash
./nvidia-sim/run_gpu_probe.sh \
  --fixture nvidia-sim/rl/runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003 \
  --diagnose-external-forces
```

기본 환경 수는 1개이다. `--num-envs 4`도 지정할 수 있으나 현재 GPU 모델은 아래 검증에서 실패했다.
CPU(CCD 켬), CPU(CCD 끔), GPU(CCD 끔)를 별도 프로세스에서 순서대로 실행한다.
`--diagnose-external-forces`는 GPU TGS 외력 적분 설정만 변경한 추가 진단을 실행한다.
각 worker의 기본 제한 시간은 600초이고 `--timeout-seconds`로 지정할 수 있다.
출력은 `rl/runs/YYYYmmdd_HHMMSS_gpu_physics_probe/`에 저장된다.

- `fixture/`: 복사한 초기 자세·관절 명령·원 후보 metadata
- 각 mode의 `report.json`, `trace_0.json`, `contacts_0.json`, 초기/종료 상태 NPZ, `run.log`
- `summary.json`: 실행 상태, CPU 대비 비교, 유효한 동작일 때만 속도 비율
- `experiment.json`, `source_sha256.json`: 시험 조건과 코드 출처

CCD 차이를 분리하기 위해 CPU에서도 CCD를 끈 대조군을 실행한다.
물리 주기 960 Hz, 제어 60 Hz, 식물 position solver 64회, 원래 강성/감쇠/마찰/분리 임계값을 유지한다.
후보 종료 후 수행하는 `native_joint_break_probe`만 별도 진단으로 target joint 임계값을 낮춰
native break event와 다시 reset하는 API를 검사한다. 이 진단은 후보의 결과나 속도에 포함되지 않는다.

## 2026-09-20 결과

실행 폴더: `runs/20260920_165852_gpu_physics_probe/`
기록: [gpu_physics_probe.json](validation/gpu_physics_probe.json)

| 조건 | 후보 실행 시간 | 제어 스텝 | 관찰 |
|---|---:|---:|---|
| CPU, CCD 켬 | 26.23초 | 259 | 접촉 후 excessive_displacement, 꼭지 분리 없음 |
| CPU, CCD 끔 | 25.59초 | 259 | CCD 켠 CPU와 관절값/변위/고리 간격 정확히 일치 |
| GPU, 기본 외력 적분 | 0.86초 | 1 | 로봇 접촉 전에 11개 꼭지 연결 모두 분리 |
| GPU, 외력 적분 설정 변경 | 0.74초 | 1 | 같은 조기 분리 현상 |

**GPU의 0.86초는 속도 향상이 아니라 즉시 실패한 시간이다. 정상 동작의 GPU 가속은 입증되지 않았다.**
CPU/GPU 초기 상태(관절, 속도, body pose, preload, 강성, 감쇠)는 정확히 일치했다.
외력 진단 GPU 시험에서 native GPU articulation heap 사용량이 13,768 bytes로 관측되었고
GPU 전체 heap은 약 1.44 GiB였다. 단순히 renderer만 GPU를 사용한 시험은 아니다.

현재 비교는 GPU 전환의 물리 결과 검증 실패로 판정한다. 원인이 GPU 솔버, 연결 생성/reset 또는
다른 내부 호환성 중 무엇인지는 아직 확정하지 않았다. 파단 임계값을 올려 실패를 숨기지 않았다.
전체 trajectory의 접촉 정확도, 고리 관통 방지, 4/64개 GPU 병렬 성능은 검증하지 않았다.

`gpu_probe.compare()`는 초기 상태, 동일 명령 hash, 종료 사유, 분리 여부, 첫 접촉 대상,
관절/변위/고리 간격과 실행 스텝 수를 검사한다. 결과가 다른 조기 실패에는 `speedup=null`을 남긴다.
허용 오차는 작은 회귀 시험의 screening 기준이며 실제 식물의 물리 보정이나 관통 방지 보증이 아니다.

## 추가 원인 분리 및 실행 옵션

```bash
# 정지 로봇: 기존 reset, 조인트 선생성, velocity iteration 0,
# 충돌/중력/프리로드 제외 진단, PGS를 각각 별도 프로세스에서 비교
./nvidia-sim/run_gpu_probe.sh --fixture <candidate_directory> --suite isolation

# 후보 동작의 PGS / CPU readback 검증
./nvidia-sim/run_gpu_probe.sh --fixture <candidate_directory> --gpu-solver pgs --fabric

# CUDA API의 접촉 로그 누락을 재현하는 진단 (데이터 생성용 아님)
./nvidia-sim/run_gpu_probe.sh --fixture <candidate_directory> --gpu-solver pgs --cuda-tensors --fabric
```

`--gpu-position-iterations N`은 정확도/안정성 진단 옵션이다. 일반 데이터 생성기에 적용되지 않는다.
`--num-envs 1|4|8|16|32|64`는 실험 범위이며 해당 환경 수의 성능/정확도 보장이 아니다.
충돌/외력 제외와 조인트 재사용은 **정지 진단에서만 허용**한다. 파단 이후 같은 조인트를
재사용하면 온전한 reset이 아니므로 실제 후보 실행에서 사용하지 않는다.

추가 실측:

- GPU TGS: 로봇 정지, 조인트 선생성, velocity iteration 0, 충돌 제외에서도 초기 파단.
  조인트 생성 시점이나 로봇 동작만으로 설명되지 않는다. 내부 엔진 원인은 확정하지 않았다.
- GPU PGS 64회: 원래 중력/충돌/파단 임계값으로 정지 960스텝 무파단.
  CUDA+Fabric 후보 재생은 209.24초 / 260 제어 스텝. 기본 CPU 26.23초 / 259스텝보다 느리고
  CUDA 모드는 첫 접촉 로그까지 누락되므로 채택하지 않았다.
- 8회 반복: CPU TGS도 초기 비목표 꼭지 파단, CPU/GPU PGS도 접촉 전 주줄기 30 mm 드리프트.
  빠르더라도 물리 검증 실패다. CPU TGS 32회도 기준 대비 주줄기 최대 오차 2.23 mm로 탈락.
- CPU PGS 64회는 이 후보의 작은 회귀 검사를 통과했지만, 다른 후보/목표/고리 유지 검증이
  없으므로 기본 설정은 바꾸지 않았다.

현재 검증된 처리량 개선 경로는 [독립 CPU 데이터 생성기](CANDIDATE_DATASET_POOL.md)다.
GPU 고속화를 계속하려면 안정된 솔버/관절 모델과 collider 단위 접촉 정보의 GPU 수집을 함께
검증해야 한다. 이번 결과는 물리 모형을 단순화하거나 파단 임계값을 높여 통과시킨 결과가 아니다.

관련 엔진 자료: [Omni Physics limitations](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/current_limitations.html),
[NVIDIA contact readback discussion](https://forums.developer.nvidia.com/t/contact-report-does-not-make-sense-when-enabling-gpu-pipeline-contact-sensor-ant-example/266782).
문서는 진단의 근거이며 이 프로젝트의 TGS 파단 버그 원인을 확정하는 증거는 아니다.
