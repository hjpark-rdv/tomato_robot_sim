# 진입 전 RGB-D + 후보 동작 + 물리 결과 데이터 생성기

RL/지도학습 없이 현재 온실과 탄성 송이를 사용한다. 고정 31개 후보 대신 nominal 1개와
scrambled Sobol 후보를 생성한다. 목표는 `Tomato_01`부터 `Tomato_11`까지 선택 가능하다.

## 실행

저장소 루트에서 실행한다. 기본 Isaac Python은 `/root/isaaclab_env/bin/python`이며
다른 환경이면 `FARMILY_ISAAC_PYTHON`으로 지정한다. 실제 GPU RGB-D 렌더링이 필요하다.

```bash
# 초기화·환경 분리·카메라만 검사; 수확 후보 실행 없음
./nvidia-sim/run_candidate_dataset.sh --num-envs 4 --candidates 8 --validate-only

# 첫 소규모 실험: 4개 환경에서 8개 후보
./nvidia-sim/run_candidate_dataset.sh --num-envs 4 --candidates 8

# 검증 후 확장
./nvidia-sim/run_candidate_dataset.sh --num-envs 4 --candidates 100
./nvidia-sim/run_candidate_dataset.sh --num-envs 4 --candidates 1000 --benchmark-candidates 8

# 목표 변경, 상승 후 당김까지 수행
./nvidia-sim/run_candidate_dataset.sh --target Tomato_07 --num-envs 4 --candidates 100 --goal pull

# 후보/config만 준비: Isaac 실행 없음
./nvidia-sim/run_candidate_dataset.sh --num-envs 4 --candidates 1000 --prepare-only
```

기본 목표는 `rise`: 진입·상승 후 1초 유지까지. `pull`은 기존 당김 단계를 추가한다.
상승 기본 속도는 2 mm/s, 당김 기본 속도는 **4 mm/s**이다. `--rise-speed .002`,
`--pull-speed .004`로 각각 지정한다(단위 m/s). 속도 변경은 실제 접촉 동역학을 바꾸며
영상 배속과 다르다. 기본 `rise` 실행에는 당김 단계가 없으므로 당김 속도만 바꿔도 빨라지지 않는다.
`--max-control-steps 20`은 짧은 실행 점검 전용이다. 중간 절단 결과는 `incomplete`이며
실패 학습 데이터로 사용하지 않는다. `--gui`로 화면을 켤 수 있다.

출력은 `rl/runs/YYYYmmdd_HHMMSS_tomato05_candidate_dataset/`에 저장한다.
터미널과 `run.log`에 진행 상황이 남는다. 동일 설정으로 중단 후 재개하려면:

```bash
./nvidia-sim/run_candidate_dataset.sh --num-envs 4 --candidates 1000 \
  --run-dir /absolute/path/to/previous/run --resume
```

완료한 `results/*/candidate.json`은 재실행하지 않는다. 코드, 주요 USD/URDF/그리퍼 CAD hash나 설정이 바뀌면 재개를
거절한다. `--prepare-only`로 준비한 폴더도 같은 설정에 `--resume`을 붙여 실행한다.
검증 전용 폴더를 본 실험으로 재사용하지 말고 새 폴더를 사용한다.

## 처리 속도와 1,000개 실행 전 측정

기본값은 `--physics-sync optimized --physics-threads 4 --torch-threads 1`이다.
초기 USD 구성·카메라 촬영·reset 검사는 유지한다. 위치 drive target은 새 명령이 있을
때 전송하고 식물 preload effort는 매 물리 스텝 전송한다. 0인 과실 외력을 불필요하게 재전송하지 않는다.
960 Hz 물리 계산, 60 Hz 제어, 64회 position solver iteration, CCD, 강성·감쇠·마찰은 유지한다.
기존 방식 비교는 `--physics-sync legacy --torch-threads 4`를 사용한다.

전체를 시작하기 전에 일부 **완전한 후보**를 실행해 시간을 측정할 수 있다:

```bash
./nvidia-sim/run_candidate_dataset.sh --num-envs 4 --candidates 1000 --benchmark-candidates 8
# 위에서 출력된 같은 폴더로 나머지 이어서 실행
./nvidia-sim/run_candidate_dataset.sh --num-envs 4 --candidates 1000 \
  --run-dir /absolute/path/from/benchmark --resume
```

benchmark는 나머지를 실패 처리하지 않는다. 요청 1,000개와 완료 8개, 남은 992개를 구분하며
`dataset_complete=false`, `benchmark_only=true`를 기록한다. `--profile`은 CPU profile 파일을 남긴다.
`--benchmark-candidates`와 `--profile`은 재개 시 생략 가능하다. 나머지 물리/실행 설정은 동일해야 한다.

약 15초 간격으로 `progress.json`과 터미널에 완료/실행/대기 수와 예상 남은 시간을 기록한다.
`summary.json`의 `estimated_total_minutes`는 현재 실행의 계획·reset·물리 계산을 포함한 처리량으로
산출한 추정이며 1시간 완료 보장이 아니다. 초기 조기 실패 몇 개만 끝난 시점의 추정은 낙관적일 수 있다.
짧게 자른 debug 실행에는 전체 시간 추정을 내지 않는다. 진행 파일은 물리 루프에서 갱신되므로
초기 장면 구성과 IK 계획 중에는 15초보다 길게 갱신이 없을 수 있다.

2026-09-20 측정: 4개 환경 × 120개 제어 스텝의 동일 CPU profiling 시험에서 반복 물리 스텝 경로는
56.60 → 43.68초(22.8% 감소), worker 전체는 85.59 → 72.30초(15.5% 감소)였다.
관절값·타겟/주줄기 변위·고리 간격은 정확히 일치했다. 별도로 이전 `candidate_00003`을 종료까지
재실행해 260개 제어 스텝, `excessive_displacement` 결과와 native contact 기록까지 일치함을 확인했다.
이는 일부 구간/접촉 사례 검증이며 모든 후보의 동등성이나 **1,000개/1시간 달성을 의미하지 않는다**.
측정 상세는 [candidate_dataset_performance.json](validation/candidate_dataset_performance.json)에 있다.

16개 환경/16개 physics thread 시험은 한 clone의 동적 상태 일치 검사를 통과하지 못했다.
검사를 완화하지 않고 실행을 거절했으며, 16/32/64개 환경을 현재 검증된 설정으로 권장하지 않는다.
물리 주기를 낮춘 시험은 줄기 변위를 바꿔 채택하지 않았다. Fabric 시험도 실질적인 물리 시간 개선이
없어 기존 USD 경로를 유지한다. 현재 주된 남은 병목은 native PhysX 단계이며 대규모 처리량은 추가 검증이 필요하다.

## 병렬 방식과 초기화

Isaac 프로세스 하나, SimulationContext 하나, Isaac Lab InteractiveScene 하나를 사용한다.
원본 온실·로봇·11개 과실·탄성 줄기를 clone하고 환경 간 collision filtering을 적용한다.
가는 줄기의 float32 좌표 오차를 줄이기 위해 clone들은 동일 좌표에 배치하고 명시적인
collision group으로 격리한다. 촬영에는 env_0만 표시한다. 다른 환경의 렌더 가시성을
끄더라도 해당 환경의 물리 collider는 활성화되어 있다.
각 슬롯은 별도의 로봇/식물 asset view와 접촉·분리 이벤트 기록을 가진다.
cooperative executor가 모든 슬롯의 명령을 적용하고 **전체 시뮬레이션을 한 번씩만** 진행한다.

현재 break joint 재생성은 CPU PhysX 경로이므로, **물리는 CPU / RGB-D는 GPU**이다.
GPU tensor 방식으로 포팅한 고속 RL 환경은 아니다. 환경 수 증가에 비례한 속도 향상을
보장하지 않는다. 64개를 기본 완료 조건으로 삼지 않으며, 메모리와 `candidates_per_second`를
보고 4 → 8 → 16 → 32/64 순서로 사용자가 늘린다. 전체 온실 형상을 보존하므로 clone 수에
따라 준비 시간과 메모리 사용량도 증가한다.

초기 구현은 batch barrier를 사용한다. 먼저 끝난 슬롯은 기다리고, 배치 전체가 끝난 뒤
모든 슬롯을 reset한다. 따라서 진행 중인 다른 후보 옆에서 USD break joint를 수정하지 않는다.
독립 슬롯 reset 기능 자체는 시작 검사에서 로봇·탄성 관절을 고의로 바꾼 뒤 복구하고,
다른 슬롯 상태가 유지되는지 확인한다. 초기 식물 배치/강성/마찰 randomization은 없다.

## 카메라와 데이터

기본 `--camera-profile current`는 URDF의 D435 optical frame 장착 관계를 사용한다.
카메라만 임의로 이동하지 않고, target을 향하는 카메라 자세에서 필요한 로봇 관절값을
IK로 구한다. PICK_READY에서 관찰 자세까지의 관절 보간 경로를 self-collision과 식물
proxy에 대해 검사한다. 관찰 자세를 찾지 못하면 명시적으로 종료한다.

촬영을 위한 관찰 자세는 시뮬레이션 초기 관절 상태로 적용한다. 실제 로봇의 관찰 자세
이동을 실행하거나, 온실 전체/실제 작업장의 이동 안전성을 보증하는 코드는 아니다.
기존의 `.4 m` 낮은 리프트 PICK_READY는 이 자세 탐색의 출발 관절 상태로 사용한다.

`--camera-profile left/right`는 이전 실험의 제안된 장착 위치로, 실제 장착이 검증된 위치가
아니다. `--camera-distance .35`, `--crop-extent .20`, `--depth-min .10`, `--depth-max 3`
옵션이 있다. 깊이 범위 기본값은 실제 장치의 측정 보증이 아니며 실물에서 보정해야 한다.
렌더 Depth는 이상적인 깊이와 범위 마스크이며 실제 센서 노이즈/결손을 재현하지 않는다.
RGB clipping range도 metadata에 남긴다. 기존 current profile의 near clip 0.10 m를
사용하므로 이보다 가까운 그리퍼 외형은 RGB에 표시되지 않는 렌더링상의 제한이 있다.

꼭지나 고리가 가려졌다는 이유로 사진을 제거하지 않는다. 타겟 중심의 화면 투영 위치와
측정 범위는 검사하지만, 가려진 표면을 복원하거나 GT 깊이로 채우지 않는다.

```text
run/
  config.json, candidates.json, source_sha256.json
  reset_validation.json, self_collision_model.json
  scene_0001/observation_0001/
    rgb.png                 # 원본 960x720, annotation 없음
    depth.npy               # optical Z, metres, float32, invalid=NaN
    depth_valid.png         # 0/255
    local_rgb.png           # target 주변 넓은 crop, resize 없음
    local_depth.npy
    local_depth_valid.png
    observation.json        # K / crop K / extrinsics / joints / crop rectangle
  results/candidate_00000/
    planned_commands.npy
    candidate.json
    trace.json
    contacts.json
  candidates.jsonl, candidates.csv, summary.json, candidate_results.png
```

관찰 이미지 한 세트를 모든 동일 초기 상태 후보가 참조한다. 크롭은 target 깊이에서
약 20 cm 문맥 범위를 투영하여 줄기/이웃 과실/진입 공간을 포함한다. 화면 경계에서
잘린 crop과 깊이 결손률은 metadata로 기록한다. 원본을 남기므로 나중에 다시 자를 수 있다.
GT 타겟 중심은 후보 생성/크롭에만 사용하며, 별도의 segmentation 입력은 만들지 않는다.

## 후보와 판정

초기 Sobol 범위:

| 변수 | 범위 |
|---|---|
| 접근 방위각 | ±45° |
| 접근 고도각 | ±15° |
| 고리 roll / pitch | ±20° / ±15° |
| XYZ offset | 각각 ±15 mm |
| pre-hook 거리 | 140–200 mm |
| 기존 경로의 insertion offset | 2–14 mm |
| 상승 끝점 높이 보정 | ±10 mm |

`insertion_distance_m`은 기존 motion skeleton의 과실 중심 대비 바깥쪽 offset이다.
전체 진입 구간 이동 길이라는 뜻은 아니다. 실제 각 waypoint와 전체 관절 명령도 저장한다.
`candidate_pose_target_frame`은 **과실 초기 중심을 원점, world 방향을 축**으로 하는 상대
pre-hook pose다. 식물의 가려진 회전축을 알고 있다고 가정하지 않는다. quaternion은 xyzw이다.

관절 한계/경로 IK/로봇 self-collision을 먼저 검사한다. 식물 접촉의 결과는 physics로
평가한다. 기존 원시 접촉 형상, 수정된 직선 와이어 collider, 동일 960 Hz physics/60 Hz
control, 기존 속도와 drive gain을 사용한다. 의도한 뒤쪽 반원과 목표 distal/terminal
pedicel의 실제 접촉, 내부 seating, 삽입 이력, 1초 유지 및 안정성으로 성공을 판정한다.
단순히 상승 좌표에 도착한 것은 성공이 아니다. GT 힘은 판정/분석용이며 vision 입력이 아니다.

**기존 31개 실험과 label policy가 다르다.** 가벼운 목표 과실 접촉만으로 실패시키지는 않는다.
비목표 접촉은 event로 모두 저장하고, 기존 3 N 위험 접촉/20 mm 과실 변위/30 mm 주줄기
변위/분리/비목표 꼭지 걸림은 실패시킨다. 데이터셋 간 성공률은 label policy를 확인해서 비교한다.
수확 성공이 아닌 접촉·걸림 실험이며, 줄기 모델은 실물로 보정되지 않았다.

## 구현 시 수행한 검사

단위 검사 45개 통과. 실제 Isaac에서 4개 clone의 반복 reset, 단일 슬롯의 로봇/식물
관절 변형 후 복구, 다른 슬롯 상태 유지, 동일 명령의 초기 물리 진행 및 RGB-D 저장을
검사했다. 관찰 이미지는 타겟 중심 약 (480,360), 문맥 crop 388×388, crop 유효 깊이
약 92.6%였다. 이는 이 고정 장면의 관찰 결과이며 다른 타겟의 보증은 아니다.

8개 후보를 4개 환경/2개 배치에서 각각 20 control step만 실행하여 데이터 저장을
검사했다. 동일 nominal 후보의 1개 환경/4개 환경 짧은 실행에서 관절/명령/목표 및
주줄기 변위/거리 기록의 최대 차이는 0이었다. 이 기록은 모두 `incomplete`이고 성공률
평가용 데이터가 아니다. 전체 진입·상승 결과, 다른 타겟, 32/64개 환경의 처리량은 아직
실행 검증하지 않았다. 검증 요약은 `validation/candidate_dataset_smoke.json`에 있다.

같은 사진에서 1,000개 동작을 실행해도 새로운 1,000개 시각 장면이 되는 것은 아니다.
향후 시각 모델 평가 시 같은 scene/observation을 train/test에 섞지 않는다.
