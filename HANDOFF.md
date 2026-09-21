# Farmily Tomato 작업 인수인계

작성일: 2026-09-21 KST (실험 폴더 이름은 기존 기록의 표기를 그대로 유지함).
이 파일은 대화 기록 없이 다른 계정/새 세션에서 작업을 이어가기 위한 시작점이다.
아래 상태는 작성 시점 기준이므로, 재개할 때 `git status`를 먼저 확인한다.

## 최신 후속 작업: 실용 60Hz (이 절을 먼저 읽기)

사용자가 960Hz와 완전히 같은 결과보다 밀림·걸림·휘어짐·복원을 유지하는 실용적
처리량을 우선하도록 기준을 변경했다. 따라서 아래의 이전 "960Hz 유지" 작업 지침은
후속 작업에서 대체됐다. 원래 성능/물리 이력은 삭제하지 않고 보존한다.

- 기준 커밋은 `bc84d41`; 실용 preset/검사/문서와 관찰 영상 도구는 후속 커밋에 포함했다.
  한글 커밋 제목은 `시뮬레이션: 실용 60Hz GPU 물리 설정과 접촉 관찰 영상 도구 추가`다.
- GPU dataset 기본은 `--physics-preset practical60`: 60Hz, armature 5e-4, PGS 64/4 iterations.
- `reference960`으로 원래 960Hz/1e-5 모델 선택 가능. `practical120`은 120Hz/3e-4 중간 비교용.
- 60Hz/5e-4에서 정지·실제 과실 하중/복원·원래 임계값 파손·리셋·네 방향 CAD 고리
  접촉 유지 검사가 통과했다. 고리 fixture는 미리 삽입된 별도 kinematic 도구이며 로봇 진입 성공은 아니다.
- 저장 로봇 경로 4환경 재생: 17.50초에 sim 4.4초 실행. 과거 960Hz는 168.69초/sim 4.333초였다.
  물리 모델이 다르므로 정밀 동등성 speedup이라고 부르지 않는다.
- 16환경 같은 경로 재생도 27.71초/sim 4.4초로 통과했고 native 접촉 라우팅/reset 오류가 없었다.
- 실제 dataset 4환경/서로 다른 4후보/8초 제한: RGB-D·계획·reset 통과. 결과 1개 과도 변위,
  3개 incomplete로 전체 수확 성공 검사는 아니다. 관련 자동 테스트 60개 통과.
- 30Hz/1e-3 잔류 변위가 커 제외. 60Hz/1e-3/32 iterations는 연결 간격 0.555mm로 0.5mm 기준 초과해 제외.
- 60Hz/3e-4 초기 검사는 idle을 과실만 검사했던 문제가 있었다. 주줄기도 포함하도록 수정했고
  최종 채택 모델은 5e-4다. 초기 `behavior60`의 passed 값을 그대로 최종 판정에 사용하지 않는다.
- 코드: `gpu_physics_presets.py`, `gpu_behavior_probe.py`, `gpu_low_hz_benchmark.py`, GPU runner/sim 변경.
- 새 검증 원본: `nvidia-sim/rl/runs/20260921_low_hz_practical/` (ignored).
- 최신 문서/실행법/레퍼런스: [GPU_PRACTICAL_PHYSICS.md](nvidia-sim/rl/GPU_PRACTICAL_PHYSICS.md).
- 60Hz 동작 관찰 영상: [GPU_PHYSICS_VIDEOS.md](nvidia-sim/rl/GPU_PHYSICS_VIDEOS.md).
  `runs/20260921_162920_practical60_contact_videos_2x/`에 실제 로봇 밀림, 하중/복원,
  미리 끼운 고리 유지의 두 시점 영상과 원본 pose/contact를 저장했다. 2배속 기본,
  고리 1배속 추가. 촬영 코드는 후속 커밋에 포함했고, 영상은 ignored runs에만 있다.
- 요약: [gpu_practical_physics.json](nvidia-sim/rl/validation/gpu_practical_physics.json).
- 다음에는 실제 로봇의 성공 진입/상승 경로, 타겟 1~11, 빠른 접촉과 더 많은 환경 수를 검증한다.
  아직 실측 보정이나 모든 상황의 비관통 보증은 없다. RL/DR을 추가하지 않았다.

## 1. 현재 결론과 바로 이어갈 일

- 기존 작업은 **`1305651`**까지 커밋했다. 브랜치는 **`nvidia-sim`**이다.
- 그 이후 **저주파 탄성 안정성 진단·설정 옵션·검증 자료를 이 문서와 함께 후속 커밋에 포함**했다.
  해당 커밋 제목은 `feat(sim): add elastic stability diagnostics and handoff`이며 해시는 `git log`로 확인한다.
- CPU 경로 계산 병렬화는 검증했고 기본 최대 8개 worker로 동작한다.
- GPU 물리 및 여러 환경의 동시 실행은 구현되어 있다. CPU 물리 worker pool과 혼동하지 않는다.
- 현재 핵심 미해결 문제는 **물리 결과를 유지하면서 960Hz를 낮추는 것**이다.
- 보조 관성(armature)을 키워 저주파의 정지 흔들림은 줄였지만, 접촉 변형과 파손 응답이 달라졌다.
  **60/240/480Hz를 기존 물리의 동등한 고속 대체 설정으로 채택하지 않았다.**
- 기본값은 **960Hz, armature 1e-5 kg·m², 로봇 제어 60Hz**를 유지했다.
- 이 파일을 작성할 때 관련 dataset/probe/planning worker는 실행 중이지 않았다.

다음 담당자는 [저주파 실험 상세](nvidia-sim/rl/GPU_ELASTIC_STABILITY.md)와
[수치 기록](nvidia-sim/rl/validation/gpu_elastic_stability.json)을 먼저 읽고,
후속 커밋의 코드를 검토한 뒤 탄성 관절·초기 하중·파손 연결부의 시간 간격 민감도를 조사하면 된다.
이미 실패한 설정을 다시 기본값으로 채택하거나, 같은 실패 라벨만 보고 물리 동등성을 선언하지 않는다.

## 2. 사용자 목표와 제약

사용자는 고리형 그리퍼로 방울토마토 아래에서 접근하고, 안으로 진입한 뒤 상승하여
꼭지 윗부분을 걸고 필요하면 당기는 동작을 연구 중이다. 주줄기를 밀면 송이와 과실도
움직여 고리걸기가 실패하는 상황까지 시뮬레이션되어야 한다.

현재 단계는 **고정 장면의 접근 pose 후보 → 실제 physics 실행 → 결과 데이터셋 생성**이다.
현재 성능/물리 진단 타겟은 `Tomato_05`이며, 장기적으로 Tomato_01~11 및 다양한 실험을 다룬다.

- 이번 데이터 생성 단계에서는 RL/IL, pose inference, segmentation 학습, domain randomization을 하지 않는다.
- 후보 생성과 사후 판정에는 simulator GT를 쓸 수 있다. 실제 perception이 구현되었다고 설명하지 않는다.
- RGB-D는 **진입 전** 촬영한다. 후보 실행 뒤 성공/실패 라벨을 붙인다.
  성공한 순간의 사진만 저장하는 방식이 아니다.
- 목표를 바라보는 관찰 자세, 원본 영상과 타겟 주변 문맥을 포함하는 crop을 사용한다.
  꼭지나 고리가 항상 보인다고 가정하지 않는다. 가림 상태도 실제로 존재한다.
- 장기 sim-to-real에서는 카메라와 joint 정보를 우선 고려하고, 힘 센서는 제외하고자 한다.
- 사용자가 제공한 house/식물/로봇 장면을 기반으로 한다. 무관한 단순 환경으로 바꾸지 않는다.
  멀티환경 표시/실행 최적화의 세부 범위는 기존 문서와 코드를 확인한다.
- 식물 위치 예: `--spawn-stem -0.75 0.55 0.32`.
  초기 자세는 ROS2 GUI의 left pick ready 계열, 리프트는 타겟 송이보다 0.4m 낮게 시작하는 요구가 있었다.
  실제 GPU runner의 cfg 설정을 기준으로 확인한다.
- 영상/실험 출력은 datetime으로 시작하고 간단한 설명이 붙은 폴더에 저장한다. 영상은 주로 2배속 요청이었다.
- 목표 처리량은 약 1,000개 후보/1시간이지만 **달성 검증은 안 됐다**.
- 최근 사용자는 큰 실험의 로딩을 피하려고 **1환경과 4환경으로 먼저 비교**하도록 요청했다.
  성능 검증 때 곧바로 256/1,000환경을 띄우지 않는다.

`--num-envs 32 --candidates 128`은 총 128개 pose를 최대 32개씩 실행한다.
32×128개의 후보를 생성하는 것이 아니다. `continuous`는 빈 슬롯에 다음 후보를 넣는 방식이다.

## 3. 실행 환경과 Git 상태

| 항목 | 값 |
|---|---|
| 저장소 | `/root/farmily_tomato` |
| 브랜치 | `nvidia-sim` |
| 인수인계 포함 후속 커밋 | `feat(sim): add elastic stability diagnostics and handoff` (해시는 `git log` 확인) |
| 성능 개선 기준 커밋 | `1305651 perf(sim): precompute candidate paths on CPU workers and benchmark timesteps` |
| 이전 관련 커밋 | `ec0c9db perf(sim): batch GPU commands and continuously schedule candidates` |
| 이전 관련 커밋 | `8995518 feat(sim): scale GPU candidate experiments and add live grid viewer` |
| 이전 관련 커밋 | `d14589d feat(sim): add parallel dataset workers, GPU probes and result dashboard` |
| 런타임 | Isaac Sim 5.1 + Isaac Lab |
| Python | `/root/isaaclab_env/bin/python` |
| GPU | RTX 3090 Ti, 약 24GB |
| CPU | Threadripper 3960X, 24코어/48스레드 |
| 물리 모니터 | `DISPLAY=:0` 사용 가능 |

`run_gpu_candidate_dataset.sh`는 Python 경로를 `FARMILY_ISAAC_PYTHON`으로 재지정할 수 있다.
기본은 위 Python이며 `PYTHONHOME`, `VIRTUAL_ENV`를 해제하고 `PYTHONPATH`를 설정한다.

후속 커밋에 포함한 파일(아래 상태 문자는 커밋 전 기록이며, 이 문서도 함께 포함):

```text
 M nvidia-sim/rl/GPU_TIMESTEP_AND_PLANNING.md
 M nvidia-sim/rl/elastic_plant.py
 M nvidia-sim/rl/gpu_dataset_runner.py
 M nvidia-sim/rl/gpu_dataset_sim.py
 M nvidia-sim/rl/gpu_probe.py
 M nvidia-sim/rl/gpu_probe_worker.py
 M nvidia-sim/rl/test_gpu_dataset_runner.py
 M nvidia-sim/rl/test_gpu_probe.py
?? nvidia-sim/rl/GPU_ELASTIC_STABILITY.md
?? nvidia-sim/rl/validation/gpu_elastic_stability.json
```

**계정 전환 시 보존:** 같은 워크스페이스를 사용하면 이 문서와 코드를 읽어 이어갈 수 있다.
코드와 요약 자료는 로컬 Git에 커밋했으며 push는 하지 않았다. 새 머신에서 원격 저장소만
clone하면 이 로컬 커밋이 포함되지 않을 수 있다. 또한 `nvidia-sim/rl/runs/`는 Git ignore
대상이므로, 다른 머신으로 옮길 경우 로컬 커밋과 필요한 실험 원본 폴더를 별도로 보존해야 한다.

## 4. 현재 실행 구조

| 파일 | 역할 |
|---|---|
| `nvidia-sim/run_gpu_candidate_dataset.sh` | 현재 GPU 데이터셋 실행 진입점 |
| `nvidia-sim/rl/gpu_dataset_runner.py` | CLI, config, resume 일치 검사, subprocess 관리 |
| `nvidia-sim/rl/gpu_dataset_sim.py` | 실제 장면, GPU 설정, 관측/검증, 후보 실행 스케줄 |
| `nvidia-sim/rl/dataset_scene.py` | 복제 환경/슬롯과 물리 실행 |
| `nvidia-sim/rl/dataset_motion.py` | 기존 후보 경로 계산 및 실행 |
| `nvidia-sim/rl/gpu_planning.py` | 실장면 모델 export 및 CPU 경로 사전 계산 서비스 |
| `nvidia-sim/rl/gpu_planning_worker.py` | Isaac을 띄우지 않는 CPU worker 진입점 |
| `nvidia-sim/rl/elastic_plant.py` | 탄성 식물 articulation, preload, 시각 모델 연결 |
| `nvidia-sim/rl/gpu_probe_worker.py` | 저장 명령을 재생하는 물리/정지/하중 분리 진단 |
| `nvidia-sim/rl/gpu_probe.py` | probe 실행/비교, 유효하지 않은 speedup 제외 |
| `nvidia-sim/rl/gpu_timestep_benchmark.py` | 1/4환경의 물리 Hz 비교 |
| `nvidia-sim/rl/gpu_planning_benchmark.py` | 동일 모델의 순차/병렬 CPU 경로 비교 |

현재 GPU dataset은 GPU dynamics + GPU broadphase + CPU tensor readback/native collider reports를 사용한다.
**완전히 CUDA tensor로만 처리하는 파이프라인이 아니다.** 접촉 object 식별을 위해 readback을 유지한다.
현재 GPU 설정은 PGS, position iteration 64, velocity iteration 4,
`gpu_max_num_partitions=1`, GPU CCD off다. 이전 CPU/TGS와 동일하다고 보증하지 않는다.
probe의 기본 solver/partition은 dataset과 다를 수 있으므로 재현 명령에서는 명시한다.

CPU planning은 실제 USD에서 추출한 FCL 도형 44개/자기충돌 검사 쌍 228개와 FK/IK 정보를 전달해
동일한 `dataset_motion.plan()`을 호출한다. IK·joint limit·자기충돌 검사를 생략하지 않는다.
최대 8개 worker로 다음 후보를 미리 준비하고 물리와 겹쳐 실행한다.
`--planning-workers 0`은 원래 순차 방식이다.

`DATASET READY`는 경로 계산 완료, `DATASET PHYSICS`는 물리 실행이다.
예전 `DATASET START`가 순차로 느리게 증가하던 큰 원인은 경로 계산이었다.
`planning_s`는 메인 프로세스의 대기 시간이며, worker 계산 시간의 합과 다르다.

타겟/성공 판정은 기존 object identity와 접촉 판정을 유지한다. probe의 `identity()`에서는
목표 distal `PedicelCollider` 및 proximal 마지막 collider, ring segment 08~23을 기록한다.
이 문서만 보고 성공 조건을 새로 추정하지 말고 `dataset_motion.py` 및 실제 target prim 정보를 확인한다.

## 5. 커밋된 성능 개선 결과

상세: [GPU_TIMESTEP_AND_PLANNING.md](nvidia-sim/rl/GPU_TIMESTEP_AND_PLANNING.md),
[검증 JSON](nvidia-sim/rl/validation/gpu_timestep_planning.json).

64개의 서로 다른 전체 후보 경로를 계산한 결과:

| 방식 | 경로 준비 시간 | 순차 대비 |
|---|---:|---:|
| 순차 | 86.14초 | 1.00배 |
| 4 workers | 23.86초 | 3.61배 |
| 8 workers | 13.57초 | 6.35배 |

명령 배열, waypoint, pose와 preflight 결과가 정확히 같았다. **전체 물리 실험이 6.35배 빨라진다는 뜻은 아니다.**
4환경/8후보/8제어 스텝의 통합 비교도 명령·trace·reset이 같았으며 경로 대기는 12.61→3.24초였다.
이 짧은 검사에는 접촉이 없었고 결과는 모두 `incomplete`다.

원래 armature 1e-5로 Hz만 낮추면:

- 60/120/240Hz는 접촉 전에도 식물이 밀려 조기 종료했다.
- 480/720Hz는 같은 접촉 구간을 진행했지만 960Hz 대비 1mm 회귀 기준을 초과했다.
- 높은 물리 스텝 수의 비용은 여전히 큰 병목이다. CPU 사용률만으로 GPU 물리 비용을 판단하지 않는다.

## 6. 후속 저주파 안정화 작업과 결과

추가한 코드:

- dataset CLI `--elastic-joint-armature` (기존값 1e-5, 유효 범위 0~0.01, 유한값 검사).
- 실제 PhysX DOF armature 적용 확인, config/backend/후보 metadata 저장, 설정이 다른 resume 거부.
- probe의 `--no-gravity`, `--no-preload`, `--preload-mode`, `--joint-armature`, `--pulse-force`.
- 정지 중 식물/과실 이동 및 joint 속도 기록. probe 예외 시 비정상 종료 코드 반환.
- 비교 시 다른 armature/preload 모델 또는 실패한 native API 검사는 유효한 speedup으로 인정하지 않음.
- production preload/reset은 원래 방식 그대로다. 실패한 drive/velocity 방식은 probe 안의 진단용 monkeypatch만 남겼다.

기존 모델은 articulation에 가는 줄기와 과실 연결 anchor를 구성하고,
초기 Jacobian으로 중력 보상 preload를 계산하여 고정 effort로 넣는다.
이번 진단에서는 geometry, 분할 수, 질량, 강성/감쇠, 마찰, 파손 임계값을 바꾸지 않았다.
단, **추가 armature 자체는 동역학을 바꾸는 추가 관성**이다.

240Hz에서 로봇 정지 2초 시험:

- 원래 모델의 최대 식물 바디 이동 약 58.5mm.
- 중력+preload 제거 시 약 0.072mm. 충돌만 제거 시 약 32.4mm.
- preload를 position/velocity drive로 옮기면 불안정/파손. 채택하지 않았다.
- armature 1e-4: 약 0.43mm. armature 1e-3: 약 0.10mm.
- 60Hz / armature 1e-3: 약 0.50mm. 정지 결과만으로 접촉 정확도를 보증하지 않는다.

저장된 `candidate_00003` 명령의 접촉 재생 시간(초기화 제외):

| Hz / armature | 1환경 wall | 4환경 wall | 실행 sim 시간 |
|---|---:|---:|---:|
| 960 / 1e-5 기존 | 156.25초 | 168.69초 | 4.333초 |
| 480 / 1e-4 | 79.54초 | 88.31초 | 4.350초 |
| 240 / 1e-4 | 40.83초 | 47.90초 | 4.383초 |
| 60 / 1e-3 | 11.74초 | 17.32초 | 4.350초 |

모두 비목표 잎을 접촉한 뒤 `excessive_displacement`로 종료했다.
성공 고리걸기/전체 60초 모션 처리량 검증이 아니다. 4환경은 같은 명령의 복제다.
물리 모델과 종료 시점이 다르므로 위 비율을 동등한 물리의 가속률로 발표하지 않는다.

0.2N에 해당하는 초기 Jacobian 고정 토크를 0.5~1.5초 가한 뒤 제거한 시험:
최대 목표 변위는 기존 약 37.64mm, 240Hz/1e-4 약 21.39mm, 60Hz/1e-3 약 16.56mm였다.
복원은 일어나지만 원래 변형 응답과 다르다. 변형 중인 지점에 일정 world force를 주는 시험은 아니다.

같은 armature끼리 시간 간격을 비교해도:

- 1e-4, 960 vs 240Hz: 고리 간격 지표 최대 차이 3.83mm, 종료 3제어 스텝 차이.
- 1e-4, 960 vs 480Hz: 고리 간격 지표 최대 차이 1.96mm, 종료 1제어 스텝 차이.
- 1e-3, 960Hz: **첫 제어 스텝 0.0167초에 로봇 접촉 없이 비목표 `HarvestJoint_05_4` 파손**.
  native API 종합 검사도 false. 이것은 안정한 비교 기준이 아니다.

회귀 허용치는 위치 지표 1mm / 로봇 joint 0.005rad / 종료 1제어 스텝으로 두었다.
이 기준은 실물 정확도나 tunneling 부재를 보증하는 값이 아니다.
960Hz도 실물 정답으로 검증된 모델은 아니다.

## 7. 실험 자료 위치와 검증 범위

모두 저장소 기준 `nvidia-sim/rl/` 아래다.

| 위치 | 내용 |
|---|---|
| `validation/gpu_timestep_planning.json` | 커밋된 CPU 병렬화/Hz 비교 요약 |
| `validation/gpu_elastic_stability.json` | 후속 안정성·접촉·복원·통합 시험 요약 |
| `runs/20260921_143244_gpu_timestep/` | 원래 모델 1/4환경 × 60~960Hz 시험 |
| `runs/20260921_144300_planning_validation/` | CPU 64경로 benchmark 및 순차/병렬 통합 비교 |
| `runs/20260921_150500_elastic_diagnosis/` | 중력/preload/contact/drive/armature 분리 진단 |
| `runs/20260921_151000_elastic_validation/` | 접촉 재생, pulse, 같은 관성의 Hz 비교, 데이터셋 통합 |
| `runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003/` | 저장된 재생 fixture |

fixture 명령 SHA256:
`cd300bcd280159339580dcba375ed759459f458eb8e84d03da730701aa5d2ad2`.

마지막 validation 폴더의 주요 하위 폴더:

- `motion_240_{1,4}`, `motion_60_{1,4}`, `motion_480_{1,4}`: 접촉 재생.
- `motion_240_stronger_{1,4}`: 240Hz / armature 1e-3 추가 진단.
- `reference_240_model_960`, `reference_60_model_960`: 각각 같은 armature의 960Hz 기준 시도.
- `pulse_960`, `pulse_240`, `pulse_60`, `pulse_240_stronger`: 하중/복원 시험.
- `*_harness_error`: 첫 pulse 구현 오류 때문에 실패한 자료. 유효 결과에서 제외했다.
  실제 joint effort 접근을 수정한 뒤 위 pulse 폴더로 재실행했다.
- `dataset_240`, `dataset_60`: 각 4환경/8개의 서로 다른 후보/16제어 스텝 통합 검사.
  초기 상태·독립 reset·camera·armature·planning 연동 통과. **각 8개 `incomplete`, dataset_complete=false**.
- `run_*.py`, `summarize.py`: 당시 실험 orchestration/요약 스크립트. `runs/` 내부라 Git 미추적이다.
  과거 폴더 완료 조건을 기다리는 스크립트도 있으므로 그대로 재실행하지 말고 명령을 읽어 새 폴더에 실행한다.

기록된 마지막 자동 검증은 관련 테스트 **55개 통과** 뒤 비교 도구에 API 실패 제외 테스트를
추가하고 해당 파일 **6개 통과**를 확인한 상태다. 새 테스트를 포함한 전체 묶음을 한 번에
다시 실행한 것은 아니다. `git diff --check`도 통과했다.

## 8. 재개 및 재현 명령

먼저 상태와 문서를 확인한다. 이 문서 작성 요청에서는 추가 시뮬레이션을 돌리지 않았다.

```bash
cd /root/farmily_tomato
git status --short
git log -4 --oneline
```

현재 기본 물리로 작은 dataset 실행(전체 후보 동작이므로 짧게 끝난다고 보장하지 않음):

```bash
./nvidia-sim/run_gpu_candidate_dataset.sh \
  --num-envs 4 --candidates 8 --schedule continuous \
  --physics-hz 960 --elastic-joint-armature 0.00001 --planning-workers 8
```

짧은 연결 검사만 하려면 `--max-control-steps 16`을 추가한다.
이때 중단된 후보는 `incomplete`이며 성공/실패 학습 데이터로 쓰지 않는다.
GUI 1환경 확인은 위 명령을 `--num-envs 1 --candidates 1 --gui`로 바꾸고 `DISPLAY=:0`을 사용한다.
새 계정에서 재현하기 전에 `--help`로 현재 옵션을 확인한다.

동일 fixture의 240Hz / armature 1e-4 진단 재현(**동등성 검증에 실패한 실험 설정**):

```bash
task_probe_dir="nvidia-sim/rl/runs/$(date +%Y%m%d_%H%M%S)_elastic_240_probe"
/root/isaaclab_env/bin/python nvidia-sim/rl/gpu_probe_worker.py \
  --headless --mode gpu --solver pgs --batched-io --native-replication \
  --gpu-partitions 1 --num-envs 1 --physics-hz 240 --joint-armature 0.0001 \
  --fixture nvidia-sim/rl/runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003 \
  --output "$task_probe_dir"
```

같은 설정으로 4환경을 검사하려면 `--num-envs 4`와 새로운 output 폴더를 사용한다.
동일 GPU에서 성능 시험을 동시에 실행하지 않는다. 초기화 시간과 motion wall time을 분리해 비교한다.

관련 테스트 재실행:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=nvidia-sim/rl \
/root/isaaclab_env/bin/python -m pytest -q \
  nvidia-sim/rl/test_gpu_planning.py \
  nvidia-sim/rl/test_gpu_step_profile.py \
  nvidia-sim/rl/test_gpu_throughput_benchmark.py \
  nvidia-sim/rl/test_gpu_replication.py \
  nvidia-sim/rl/test_gpu_scale_validation.py \
  nvidia-sim/rl/test_gpu_prim_lookup.py \
  nvidia-sim/rl/test_gpu_initialization.py \
  nvidia-sim/rl/test_gpu_batch_views.py \
  nvidia-sim/rl/test_gpu_dataset_runner.py \
  nvidia-sim/rl/test_gpu_validation.py \
  nvidia-sim/rl/test_gpu_probe.py \
  nvidia-sim/rl/test_dataset_pool.py
```

## 9. 다음에 검토할 작업 (미실시)

1. 후속 커밋 diff와 실험 기록을 검토하고 보존한다. 원래 preload/reset을 실험 실패 버전으로 되돌리지 않는다.
2. 보조 관성 1e-3/960Hz에서 접촉 전 파손되는 이유를 파손 연결부의 하중/초기화부터 분리 진단한다.
   원인이 엔진 버그라고 확정되지는 않았다.
3. 주줄기·꼭지·과실의 결합 및 하중 전달을 대상으로 작은 정지/토크/접촉 시험을 만든다.
   주파수를 바꿔도 변형·파손 응답이 수렴하는지 검사한다.
4. 실측 식물 변형 자료가 확보되면 모델의 응답을 보정한다. armature를 키우는 것만으로 sim-to-real을 보증하지 않는다.
5. 유효한 저주파 설정을 확보한 뒤 서로 다른 후보, 성공 trajectory, 타겟 1~11로 검증을 넓힌다.
6. 그 다음 대규모 환경 수에서 초기화/경로/physics/read-write/판정 시간을 다시 측정한다.

사용자가 요구한 충돌/탄성/파손 실패 사례를 없애거나 파손 임계값을 올려서 속도 시험을 통과시키지 않는다.
물리 정확도를 바꾸는 설정과 결과를 유지하는 코드 최적화는 기록에서 구분한다.

## 10. 이전 작업 문서 찾아보기

- [전체 시뮬레이터 README](nvidia-sim/README.md)
- [GPU 데이터셋](nvidia-sim/rl/GPU_CANDIDATE_DATASET.md)
- [GPU throughput](nvidia-sim/rl/GPU_THROUGHPUT.md)
- [GPU 병목 조사](nvidia-sim/rl/GPU_BOTTLENECK.md)
- [GPU 최적화](nvidia-sim/rl/GPU_OPTIMIZATION.md)
- [GPU probe](nvidia-sim/rl/GPU_PROBE.md)
- [후보 데이터셋/관측](nvidia-sim/rl/CANDIDATE_DATASET.md)
- [이전 CPU 다중 프로세스 pool](nvidia-sim/rl/CANDIDATE_DATASET_POOL.md)
- [탄성 식물](nvidia-sim/rl/ELASTIC.md)
- [고리걸기](nvidia-sim/rl/HOOKING.md)
- [pose 탐색](nvidia-sim/rl/POSE_SEARCH.md)
- [RGB 관찰/밀림 감지](nvidia-sim/rl/RGB_GUARD.md)

옛 문서의 CPU/TGS/worker pool 설정을 현재 GPU runner의 설정으로 착각하지 않는다.
충돌하는 수치는 최신 코드, 이번 검증 JSON, 마지막 두 성능/안정성 문서를 우선 확인한다.

## 새 대화에 붙일 문장

> `/root/farmily_tomato/HANDOFF.md`를 읽고 현재 Git diff와 연결된 최신 검증 자료를 확인한 뒤 이어서 작업해줘.
> 성능 개선 기준 커밋은 `1305651`이고, 저주파 안정화 진단과 이 문서는 후속 커밋에 포함했어.
> GPU 멀티환경과 CPU 경로 병렬화는 구현되어 있지만, 물리 결과를 유지하는 저주파 설정은 아직 검증하지 못했어.
> 그 이후 실용적인 물리 동작을 우선하는 기준으로 바뀌었고, 최신 기본값은 practical60이야.
> 이 문서 맨 위의 최신 후속 작업과 GPU_PRACTICAL_PHYSICS.md를 먼저 확인해줘.
