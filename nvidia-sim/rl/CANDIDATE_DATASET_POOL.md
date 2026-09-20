# 독립 프로세스 병렬 데이터 생성

```bash
./nvidia-sim/run_candidate_dataset_pool.sh --workers 4 --num-envs 1 --candidates 1000
```

CPU 물리/960 Hz/TGS 64회/원래 강성·감쇠·마찰·파단 임계값을 유지한다. GPU는 RGB-D 렌더링에
사용한다. 기존 greenhouse, 전체 11개 과실, target 중심의 관찰 자세와 성공 판정 코드를 재사용한다.
RL/인지 모델/환경 랜덤화는 없다. 이는 GPU 물리 전환을 완료한 프로그램이 아니다.

`--workers`는 독립 Isaac 프로세스 수(1~8), `--num-envs`는 각 프로세스 내부 clone 수(1 또는 4)다.
우선 **4 workers × 1 env**를 사용한다. 매 후보마다 새 프로세스를 띄우지 않고, 각 worker가
할당된 후보를 연속 실행하며 기존 전체 reset 검사를 수행한다. 원본 후보 목록을 한 번 생성하고
고유 candidate ID를 유지해 분배하므로 worker마다 nominal/Sobol 후보를 중복 생성하지 않는다.

Kit 시작을 순차적으로 수행하고 프로세스당 startup thread를 8개로 제한해 CPU 과점유와 초기화
캐시 경합을 줄인다. 시작 이후 물리 실행은 동시에 진행된다. 각 worker는 독립 generated robot
USD를 쓰므로 다른 프로세스의 파일을 덮어쓰지 않는다. PhysX thread는 기존 기본 4개다.

기존 카메라/목표/동작 옵션은 그대로 전달할 수 있다:

```bash
./nvidia-sim/run_candidate_dataset_pool.sh --workers 4 --num-envs 1 \
  --target Tomato_05 --candidates 8 --goal rise --physics-threads 4

# 후보와 분배만 생성
./nvidia-sim/run_candidate_dataset_pool.sh --workers 4 --candidates 1000 \
  --run-dir /absolute/path/new_run --prepare-only

# 동일 후보/worker/물리 설정 및 동일 소스로 재개
./nvidia-sim/run_candidate_dataset_pool.sh --workers 4 --candidates 1000 \
  --run-dir /absolute/path/new_run --resume
```

GUI, validate-only, benchmark-candidates, profile은 pool 옵션이 아니다. 소수의 후보를 실제 끝까지
실행해 평가한다. `--max-control-steps`를 전달한 debug run은 기존과 같이 incomplete로 기록되고
완성된 학습 데이터로 표시되지 않는다. 실패/중단된 worker는 완료로 간주하지 않으며,
Ctrl+C는 해당 pool이 띄운 프로세스만 정리한다. 소스 변경 후에는 새 폴더를 사용한다.

## 결과

기본 폴더: `runs/YYYYmmdd_HHMMSS_candidate_dataset_pool/`

```
pool.json                  # 전체 후보 ID의 고정 worker 할당
summary.json               # 완료/미완료 수, 분류별 개수, worker 상태
candidates.jsonl           # 통합 metadata, 경로는 pool 기준
candidates.csv
_template/                 # 전체 설정 및 한 번 생성한 전체 후보
worker_00/
  config.json
  generated/rb5_ring.usda
  scene_0001/observation_0001/  # full/local RGB-D, intrinsics, camera pose
  results/candidate_00000/     # metadata, commands, contact, trace
  run.log
worker_01/ ...
```

RGB-D는 각 worker의 실제 초기 상태에서 저장한다. `candidate_record_path`는 원본 metadata,
`worker_dataset_root`는 원본 파일의 경로 기준이다. 통합 JSONL의 observation/command 경로는
pool root에서 바로 찾을 수 있다. observation JSON 내부의 이미지 경로는 원래 observation
폴더를 기준으로 해석한다. 서로 다른 worker의 `observation_0001`을 같은 파일로 합치지 않는다.

## 검증 범위

동일 saved trajectory를 CPU 1환경에서 4개 독립 프로세스로 재생한 실측:

- 단독 기준 26.23초 × 4 = 104.90초 분량을 동시 실행 구간 27.43초에 처리: **3.82배 처리량**.
- 네 프로세스 모두 기준과 259 제어 스텝, 종료 결과, 첫 접촉 대상 일치.
- 관절, 목표/주줄기 변위, 고리 간격 trace의 최대 차이 모두 0.
- 이 수치는 카메라/초기화/IK를 제외한 동일 동작 비교이며, 기존 4-clone 프로그램 전체 대비
  3.82배라는 뜻은 아니다. 1,000개를 1시간에 완료한다는 검증도 아니다.

RGB-D를 포함한 실제 후보 4개 실행은
`runs/20260920_173734_candidate_dataset_pool/`에서 **169.56초**에 완료했다.
과도한 밀림 2개, miss 1개, 비목표 접촉 1개이며, 기존 4-clone 실행의 분류와 같다.
worker 간 촬영 관절값의 차이는 0이고 depth 배열도 동일하다. 4개 worker 실행 중 GPU 메모리는
12,295 MiB로 관측했다. 초기화는 순차이고 동작은 병렬이다.
기록: [전체 파이프라인](validation/candidate_dataset_pool.json),
[동일 명령/접촉 비교 및 GPU 진단](validation/gpu_isolation_and_parallel.json).

독립 worker는 RAM/VRAM을 각각 사용한다. 24 GB GPU에서 모든 작업에 4개가 항상 최적이라는
보장은 없으며, 다른 GPU 작업이 동시에 있다면 worker 수를 줄여 실행한다.

### 6~8개 worker로 확장

4개는 하드웨어 최대치가 아니라 완료된 전체 후보 실행 검증 범위다. 5~8개는 실험적으로 허용하며,
기본값은 4개를 유지한다. 2026-09-20의 실제 4-worker 장시간 실행에서 VRAM 12,199 MiB,
worker당 CPU 약 280%, 가용 RAM 96 GiB를 확인했다. 같은 메모리 증가량을 가정하면 6개는
VRAM 약 17 GiB로 추정되지만, 6개 동시 실행의 실제 메모리/처리량 검증은 아직 하지 않았다.

새 실행에서 `--workers 6 --num-envs 1`을 지정한다. PhysX/Torch/app thread 수는 우선 유지한다.
기존 폴더의 worker 수를 바꿔 `--resume`할 수는 없다. 이미 배정된 후보를 중복 실행하거나
빠뜨리지 않도록 할당을 고정하기 때문이다. 실행 중인 4-worker 작업은 자동 확장되지 않는다.

부모 실행기에 SIGTERM/SIGHUP이 전달돼도 worker 정리를 수행한다. 모든 worker에 먼저 종료를
요청하고, 공통 20초 유예 이후 남은 worker를 강제 종료한다. 부모에 SIGKILL을 직접 보내면
이 정리 코드는 실행될 수 없으므로 정상 중단에는 Ctrl+C 또는 SIGTERM을 사용한다.

2026-09-20의 **8-worker 짧은 용량 검사**에서는 8개 모두 RGB-D/reset 검사를 통과하고,
8개 프로세스가 함께 물리 실행 중인 상태를 15초 이상 관측했다. 최대 관측 VRAM은
22,984 / 24,564 MiB로 여유 약 1.5 GiB였다. `--workers 8 --num-envs 1` 실행은 허용하지만,
이 검사는 후보를 240 제어 스텝에서 잘라 실행한 것이므로 장시간 안정성, 완전한 후보 결과,
4개 대비 처리량 개선을 검증한 것은 아니다. 기본값은 여유가 있는 4개를 유지한다.
첫 용량 검사는 마지막 초기화 전에 앞쪽 worker가 끝나 결론을 내리지 않았고,
후보 대기열을 늘린 재검사에서 실제 8개 동시 실행을 확인했다.

시험 후 부모에 SIGTERM을 보내 8개 worker가 모두 정리됐고 VRAM은 1,699 MiB로 돌아왔다.
기록: [eight_worker_capacity.json](validation/eight_worker_capacity.json).
