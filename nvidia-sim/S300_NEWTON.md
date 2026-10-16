# S300 농장 수확 · Newton 실행

2026-10-17 솔버·식물 표현 비교는 [S300_NEWTON_SOLVER_COMPARISON_KO.md](S300_NEWTON_SOLVER_COMPARISON_KO.md)에 있다.
원본을 보존한 opt-in `./nvidia-sim/run_s300_newton.sh gpu-fast --gui`는
133자유도·보수적 열매 proxy 모델을 사용하며 전체 약2.50배속이다.
원본과 물리 표현이 다르고 접촉 구간은 약0.62배속으로 남아 있다.
실제 GPU8환경까지 상태·접촉·힘·분리 이벤트 일치를 확인했지만,
병렬 시험은 proxy 없는 별도133자유도 입력/수치 경로다. 아래 과거 수치를 현재 최선으로 읽지 않는다.

2026-10-17 GPU 후속 검증은 `codex/s300-newton-gpu-realtime-20261016`에서 진행했다.
실제 구성은 **Isaac 장면 표시 → Newton → MuJoCo Warp CUDA 물리**이며,
전체 GUI/headless 상태 일치는 통과했다. 접촉 구간의 실시간 성능은 아직 미달성이다.
전체 평균은 headless1.561배속/GUI1.351배속이다. 벨트 구간은.382/.328배속으로 남아 있다.
최신 수치·실패 대조·실제 GPU 영상은 [GPU 결과 보고서](S300_NEWTON_GPU_REALTIME_KO.md)를 참조한다.

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh gpu --gui
```

`--gui`를 생략하면 같은 GPU 물리를 화면 없이 실행한다. CPU fallback은 없다.
원본의 농장 외형·활성 충돌 범위·1mm 관통 기준·분리 규칙을 유지한다.
GPU 모드는 검증한 단일 환경 구성이다. 여러 환경 수확의 성능 검증은 아직 하지 않았다.

아래는 출발 브랜치 `codex/s300-newton-20261016`의 **CPU 비교 실행** 설명이다.
Newton 1.7의 CPU MuJoCo 백엔드 + 별도 Isaac 실시간 화면을 사용한다.
동등성을 확인한 최적화 구성에서 기존 수확 동작82.795초를 화면 없이61.994초,
화면을 켜서63.476초에 실행했다. 각각 실시간의 **1.336배 /1.304배**다.
초기 로딩·GUI 준비·결과 압축 시간은 별도다.
이 수치는 이동을 포함한 전체 평균이다. 벨트와 열매가 밀접하게 접촉하는 일부 단계는
약 **0.30~0.65배속**으로 느려진다. 모든 순간이 실시간을 넘는다는 뜻은 아니며,
접촉 구간의 처리량을 별도로 개선·비교한다.

동일 입력을 사용한 GUI/헤드리스 두 실행의 **13,806개 물리 스텝 전체와 초기 상태**를
비교했다. 시간·명령·phase·qpos/qvel·열매 연결 활성 상태·모든 body 자세·접촉 지표·분리
이벤트가 정확히 같았다. 실제 GUI 프로세스가 측정 상태를 표시했고, 화면의 physics step과
timeline time은 모두 0이었다. 저장된 결과를 나중에 재생한 것만으로 판정한 비교가 아니다.

## 실행 명령

이 서버에는 입력과 별도 환경이 준비되어 있다. 기본 명령은 다음과 같다.

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh run --gui
```

`DISPLAY=:0`, 전체 농장 표시, CPU 물리, 접촉 단계 400Hz, 비접촉 이동 단계 60Hz,
화면 전달 상한 15FPS가 기본이다. 화면 FPS와 물리 Hz는 독립적이다.
기본 입력은 검증된 `inputs_render_separated`이며 `--native-observables-only`를 적용한다.
원래 충돌 형상·물성은 유지하고, 물리에서 비충돌 표시geom과 metadata 중복 갱신만 분리했다.
물리 초기화와 GUI 최초 로딩은 먼저 수행하고, 화면 준비 확인 뒤 물리 측정을 시작한다.

화면 없이 같은 수확 동작을 실행하려면:

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh run
```

결과 폴더는 실행마다 새로 만든다.
기본 위치는 `/root/docker_share/mujoko_debugging_data/YYYYMMDD_HHMMSS_s300_newton_harvest/`다.
다음처럼 출력과 입력을 명시할 수도 있다.

```bash
./nvidia-sim/run_s300_newton.sh run \
  /root/docker_share/mujoko_debugging_data/20261016_s300_newton/inputs_300hz \
  --gui --output /root/docker_share/mujoko_debugging_data/my_newton_trial
```

이미 있는 출력 폴더는 덮어쓰지 않는다. `--seconds 5`는 초기 5초 연결 확인용이며,
접촉·수확 완료 검증으로 해석하지 않는다. 전체 옵션은 `./nvidia-sim/run_s300_newton.sh --help`와
`./nvidia-sim/run_s300_newton.sh run --help`에서 확인한다.

## 설치와 입력 준비

물리 Python은 `nvidia-sim/.newton-venv`에 따로 설치했다.
기존 Isaac Python과 MuJoCo 환경은 각각 유지한다.

```bash
cd /root/farmily_tomato
./nvidia-sim/setup_s300_newton.sh
./nvidia-sim/run_s300_newton.sh check
```

고정 버전은 [newton-requirements.lock.txt](newton-requirements.lock.txt)에 있다.
Python 3.12 / Newton 1.7.0 / MuJoCo 3.15.0 / MuJoCo Warp 3.15.0 /
Warp 1.17.0을 사용한다. Warp 1.17은 이 서버의 NVIDIA 드라이버 570.211.01과
실제 CUDA 실행을 확인한 버전이다. 최신 패키지라는 이유로 임의 교체하지 않고 비교를 재검증한다.
`check`는 패키지·의존성을 확인하며 CUDA나 시뮬레이터를 시작하지 않는다.

입력이 없는 머신에서 기존 portable 모델로 입력을 준비하려면:

```bash
./nvidia-sim/run_s300_newton.sh prepare \
  /root/docker_share/mujoko_debugging_data/20261012_s300_mujoco_rebuilt/window_00 \
  /root/docker_share/mujoko_debugging_data/my_newton_inputs
```

준비는 기존 `export.json`, MJCF, 원본 Isaac 계획과 상태를 읽고,
로봇 초기 q/qvel·열매 연결·고정 식물 중력 preload를 계산한다.
**실제 물리 적분은 0스텝**이며 원본 모델·compiled cache·결과를 수정하지 않는다.
`input_scene.xml`, `input_model.mjb`, `initial_state.npz`, `commands.npz`,
`source_export.json`, `sampled_plan.json`, `inputs.json`을 새 폴더에 저장한다.
XML의 메시 경로는 원본 자산을 참조하므로 Git만 복사해서는 실행 자산이 모두 생기지 않는다.
MJB는 생성한 MuJoCo 버전에 종속된다. 실행기는 같은 버전의 서명된 MJB를 물리 기준으로
사용한다. XML만 새로 컴파일한 모델을 동등하다고 가정하지 않는다.

기본 입력은 300Hz로 준비되어 있지만, 권장 실행기는 기존 계획 sampler를 사용해
400/60Hz 명령 시간표를 명시적으로 다시 만든다. 결과의 `executed_commands.npz`가
그 실행에 실제 사용한 명령·시간 간격·phase의 기준이다.

### 물리에서 표시 전용 메시만 분리하는 입력 준비 옵션

`prepare --discard-visual-geoms`는 MuJoCo의 `compiler.discardvisual=true`를 사용한다.
화면은 원래 농장 USD를 그대로 표시하고, 물리 모델에서 `contype=conaffinity=0`인
표시 전용 geom만 제거한다. 충돌 메시를 단순화하는 옵션이 아니다.

```bash
./nvidia-sim/run_s300_newton.sh prepare \
  /root/docker_share/mujoko_debugging_data/20261012_s300_mujoco_rebuilt/window_00 \
  /root/docker_share/mujoko_debugging_data/my_render_separated_inputs \
  --hz 300 --discard-visual-geoms
```

서버의 새 `inputs_render_separated`는 **4,720→901geom**으로 줄었지만,
**활성 충돌체901개·body132개·DOF385개·actuator10개·열매 연결10개**는 그대로다.
810종의 실제 충돌 메시와 질량·관성·관절·actuator·equality·충돌 제외·접촉 설정 등을
이름으로 연결해 **12,370항목을 수치 오차 허용 없이 비교**했다. 두 물리 semantic SHA가 같다.
MJB 크기는 1,150,100,042→595,926,694바이트다. 준비 중 물리 적분은0스텝이다.

컴파일러가 시각 형상에서도 계산하는 각속도 길이 척도 `dof_length`는 원본 값을 명시적으로
복원했다. 이 값은 sleep 판단에 쓰일 수 있으므로 단순히 비교에서 제외하지 않았다.
복원 내역과 모든 비교는 `physics_semantic_proof.json`에 저장했다.
물리의 기준은 서명된 `input_model.mjb`이며, XML만 다시 컴파일해 동일하다고 가정하지 않는다.

정적 비교에 더해 전체 실행 상태·접촉·분리 배열과 최적화 구성의 GUI/헤드리스 동등성까지
확인했다. 이 서버는 해당 입력을 기본으로 사용한다. 기존 `inputs_300hz`도 그대로 보존한다.

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/inputs_render_separated/physics_semantic_proof.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/inputs_render_separated/physics_semantic_proof.json)

## 실제 물리를 실행하는 구성

| 실행 선택 | 실제 물리 계산 | 화면 |
|---|---|---|
| 기본 `--backend cpu` | Newton `SolverMuJoCo.step`을 통한 native MuJoCo CPU | 별도 Isaac 프로세스가 실제 측정 자세 표시 |
| `--backend gpu` | Newton `SolverMuJoCo.step`을 통한 MuJoCo Warp CUDA | 같은 실제 측정 자세 표시 방식 |

**`run`의 기본 CPU 실행을 GPU Newton 성능이라고 설명하면 안 된다.**
최신 GPU 구성은 별도 `gpu` 명령이며, 예전300Hz 진단과 구분한다.
GPU 병렬 환경 수별 성능·유효성은 별도 결과로 판단한다.
1개 환경에서 느린 계산을 환경 수만 곱해 빠르다고 표시하지 않는다.

물리 실행 프로세스는 Kit를 import하지 않는다. 단일 소유자가 정해진 순서로
명령 적용 → 접촉 계산 → 실제 적분 → 분리·물리 검사 → 상태 기록을 수행한다.
GUI는 `newton_live.py`와 기존 `motion_live.py`의 최근 측정 프레임을 받아 표시한다.
카메라 조작·화면 프레임 생략·표시 지연이 물리 step 수나 dt를 변경하지 않는다.
GUI는 기존 [view_motion_live.py](harvest_s300/view_motion_live.py)를 사용하고
timeline 자동 진행과 Play를 차단한다. 준비 신호·실제 Python PID·최종 프레임 표시 확인을 남긴다.

Isaac Sim 6.1에 동봉된 Newton은 1.5.0이다. 그 USD importer를 통해 기존 PhysX
`surfaceVelocity`, native `breakForce/breakTorque`, soft angular limit 의미가 그대로
이전된다고 가정할 수 없었다. 원본 USD 직접 가져오기 시험도 convex decomposition 준비가
8분 예산을 넘어 중단됐다. 이 시험을 완료된 수확·속도·GPU 안정성 검증으로 집계하지 않는다.
따라서 기존에 검증한 **MuJoCo portable 변환 자산과 제어·분리 adapter**를 재사용했다.
PhysX와 접촉 결과까지 동일하다는 주장은 하지 않는다.

일반 Newton MJCF 변환이 일부 native 필드를 바꾸는 문제는
[newton_mjcf.py](harvest_s300/newton_mjcf.py)의 제한된 `SourceModelSolverMuJoCo` adapter로 처리한다.
실제 물리 모델에는 원래 MjModel의 메시·질량·관절·actuator·equality·충돌 제외를 유지하고
모델 배열 digest를 확인한다. Newton metadata의 `shape_count=106`은 실제 충돌체 수가 아니다.
물리 접촉 검사와 결과의 기준은 native MuJoCo/MJWarp 배열이다.
이 adapter는 버전에 의존하므로 설치 버전 변경 시 관련 회귀시험이 필요하다.

## 농장과 접촉·분리 의미

기준 장면은 기존 `D_target_only` / Plant29다.
원본 Isaac 조립 자산은
`/root/docker_share/mujoko_debugging_data/20261011_194707_plant_interaction_comparison/D_target_only/`이며,
그 계획·명령은 같은 실험의 `trials/D_target_only_connected/`에서 가져왔다.
MuJoCo 변환기는 `mujoco-benchmark/scripts/s300_smoke/`의 기존 구현을 재사용한다.

- 전체 농장 표시용 기존 집계: source mesh **29,357개**, 정점 **27,049,149개**,
  삼각형 **48,671,177개**. 표시를 위해 줄기·잎·하우스 메시를 새로 경량화하지 않았다.
- 원본 단일 창 native 입력: **132body /385DOF /4,720geom /1,549mesh /
  10actuator /10열매 연결**. 현재 기본 입력은 표시 전용 geom만 분리해901geom이며,
  활성 충돌체901개·body·DOF·물성은 같다. 로봇·리프트·jaw는 유한 힘 PD로 움직인다.
- 타깃 주줄기 16분절, 송이 가지와 꼭지, 잎 27개 등 기존 full 타깃 구성을 유지한다.
  새로운 DOF 병합·충돌체 축소·질량/마찰/강성/actuator 완화는 하지 않았다.
- 충돌 범위는 사용자가 이전에 선택한 **타깃 송이와 그 주줄기**, 로봇·거터·바닥이다.
  주변 식물은 표시하지만 충돌하지 않는다. 잎 충돌 무시 정책도 그대로다.
  전체 농장의 모든 식물이 충돌·탄성 처리된 시험으로 설명하지 않는다.
- 과거 Plant22에서 이미 떨어진 5열매는 기존 portable export에서 제외되어 있었다.
  이번에 새로 숨겨 초기 상태를 통과시킨 것이 아니다. 기존 checkpoint semantic SHA,
  NPZ SHA, 분리 이벤트를 확인하고 누락 범위와 근거를 `inputs.json`에 기록했다.
- 열매 연결은 기존 MuJoCo adapter의 connect + passive angular spring/deadband다.
  **3N / 0.008Nm** 분리 조건, 벨트의 검증된 안쪽 이송 방향,
  **활성 접촉 겹침 1mm** 기준을 유지한다. 이들은 실물에서 보정된 손상 한계가 아니다.
- 분리된 열매의 바닥·기체·같은 송이 접촉은 기존 승인된 lifecycle 진단으로 별도 집계한다.
  활성 로봇·부착 중인 열매·주줄기 접촉까지 임의 허용하지 않는다.

기존 `phase_frequency`를 그대로 재사용해 준비·레일 이동·복귀를 60Hz로 실행하고,
접근·벨트·닫기·두 차례 개방/재닫힘·후퇴 등 접촉 단계는 400Hz로 실행한다.
정수 master tick으로 만든 명령 시간표와 각 step의 dt를 저장한다.
이 조건은 300Hz와 수치적으로 다른 설정이며 같은 물리 결과라고 해석하지 않는다.

## 검증 결과와 해석

| 단일 환경 조건 | 실제 처리 결과 | 실시간 대비 | 분리 | 활성 접촉 최대 겹침 |
|---|---|---:|---:|---:|
| CPU 300Hz / free60 | 55.507초에서 꼭지04–벨트 접촉으로 중단 | 2.154배 | 전체 완주 아님 | 1.019mm |
| CPU 400Hz / free60, 원본 입력 | 82.795초 완주 / wall66.733초 | 1.241배 | 6개 | 0.681mm |
| 같은 원본 입력, GUI 표시 | 82.795초 완주 / wall68.037초 | 1.217배 | 같은6개 | 같은0.681mm |
| 현재 기본 최적화, 화면 없음 |82.795초 완주 /wall61.994초 |1.336배 |같은6개 |같은0.681mm |
| 현재 기본 최적화, GUI 표시 |82.795초 완주 /wall63.476초 |1.304배 |같은6개 |같은0.681mm |

300Hz 중단을 숨기거나 겹침 기준을 높여 통과시키지 않았다.
400Hz 결과는 **한 송이의 기존 한 번 진입/두 차례 개폐 동작**의 분리6개다.
송이 열매10개 전체 수확, 실제 포집, 여러 장면 일반화, 자율 RGB-D 판단까지 완료된 결과가 아니다.
`training_eligible=false`, `harvest_success=null`을 유지한다.

GUI 비교는 초기 포함 13,807개 상태 표본, 13,806개 실제 물리 스텝 전체를 검사했다.
중단 이전의 공통 prefix만 같거나 요약 숫자만 같다고 PASS하지 않는다.
원본 입력·실제 명령 시간표 SHA, 전체 qpos/qvel·pose·equality·접촉력/침투·분리 이벤트와
실제 GUI 표시 증거를 각각 확인한다.

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/gui_equivalence.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/gui_equivalence.json)

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/gui_optimized_equivalence.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/gui_optimized_equivalence.json)

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_free60_full/summary.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_free60_full/summary.json)

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_free60_gui_verified/summary.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_free60_gui_verified/summary.json)

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_free60_gui_verified/live_final.png](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_free60_gui_verified/live_final.png)

## 결과 저장·재생·GPU 비교

각 run의 `states.npz`는 실측 상태, `executed_commands.npz`는 실제 제어 시간표,
`physics_metrics.npz`는 물리 주기의 접촉·분리 검사, `summary.json`은 범위와 속도다.
`newton_break_events_all_worlds.json`에 원래 전체 환경의 분리 이벤트를 보존한다.
기존 renderer용 `break_events.json`에는 world0의 정확한 source joint와 실제 분리 유효 시각을 붙인다.

기존 [render_motion.py](harvest_s300/render_motion.py)는 저장된 실제 body pose를 그린다.
명목 계획 영상이나 두 번째 물리 실행이 아니다. 동영상의1배속과 실제 물리 계산의 실시간 배율은 별개다.
동영상 생성 시간도 물리 처리 시간에 포함하지 않는다.

GPU 후속 결과는 [GPU 결과 보고서](S300_NEWTON_GPU_RESULTS_KO.md)에 따로 보존한다.
전체 물리·GUI 일치를 확보했으나 접촉 구간의 실시간 조건을 충족하지 못해
8/32환경 수확 처리량으로 확장하지 않았다.

```bash
./nvidia-sim/run_s300_newton.sh gpu --gui
```

GPU는 최초 kernel compile과 메모리 준비가 필요하다.
1/8/32환경 등의 성능은 같은 접촉 정책·시간표·분리/유효성 검사를 완료한 측정으로 판단한다.
기동·짧은 연결 확인·메모리 확보만으로 대규모 수확을 완료했다고 설명하지 않는다.
CPU1/8/24/48환경의81회 완주와 GPU 실패를 포함한 결과,
남은 접촉 구간 병목과 최종 동영상 목록은 [S300_NEWTON_RESULTS_KO.md](S300_NEWTON_RESULTS_KO.md)에 모았다.

전체 동작1배속 영상(전경 잎 가림 포함):

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_render_separated_gui/newton_actual_entry.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_render_separated_gui/newton_actual_entry.mp4)

접촉 탑뷰1배속 영상(45~71.995초,재생 화면에서만 배경/잎133경로 숨김):

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_render_separated_gui/newton_actual_truss_top.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_render_separated_gui/newton_actual_truss_top.mp4)

원래 입력으로 측정한 병렬 비교를 다시 실행하려면 다음을 사용한다. 각 그룹을 순서대로
실행하며, 그룹 안에서는 모든 환경이 초기화된 뒤 동시에 물리를 시작한다.

```bash
./nvidia-sim/.newton-venv/bin/python -u nvidia-sim/harvest_s300/benchmark_newton.py \
  /root/docker_share/mujoko_debugging_data/20261016_s300_newton/inputs_300hz \
  --backend cpu --jobs 1 8 24 48 --hz 400 --free-hz 60 \
  --output /root/docker_share/mujoko_debugging_data/my_newton_cpu_scaling
```

선택한 표시 분리 입력의 병렬 성능을 새로 비교할 때는 입력을 `inputs_render_separated`로
바꾸고 `--native-observables-only`를 추가한다. 해당 새 비교를 기존 표의 측정값으로 대신하지 않는다.
