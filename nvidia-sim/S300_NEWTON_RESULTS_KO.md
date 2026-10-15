# S300 Newton 서버 결과 · 2026-10-16

현재 확보한 것은 **전체 농장 화면을 유지하면서, 한 송이의 기존 진입·벨트 수확 동작을
Newton의 CPU 백엔드로 평균 실시간보다 빠르게 실행하고, GUI 유무에 따른 물리 결과 차이를
없앤 구성**이다. GPU 병렬 수확이 성공한 결과는 아니다. 접촉이 많은 일부 구간은 아직
실시간보다 느리다.

브랜치 `codex/s300-newton-20261016`, Isaac 보존 기준 `ca3a716`, 기존 MuJoCo 이식 자산을
가져온 서버 작업 전 기준 `6b32dd0d0c6c21aa0405c5b49f54fe189bbb58d3`이다.
최종 변경 SHA는 이 문서를 포함한 Git 커밋에서 확인한다. 기존 결과를 덮어쓰지 않았고,
모든 실험은 `/root/docker_share/mujoko_debugging_data/20261016_s300_newton/` 아래에 분리했다.

## 바로 실행

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh run --gui
```

`run`은 새 출력 폴더를 만들며, 화면 없이 실행하려면 `--gui`를 생략한다.
현재 기본 입력은 검증된 `inputs_render_separated`와 `--native-observables-only`이며,
실행 시간표는 **접촉400Hz /비접촉 이동60Hz**다. 원래 `inputs_300hz` 기준 결과도 보존했다.
입력 준비 주파수와 실행 주파수를 구분한다. 실제 제어 명령은 각 실행의
`executed_commands.npz`에 다시 저장한다. 기본 사용법과 설치는 [S300_NEWTON.md](S300_NEWTON.md).

## 무엇을 유지했고 무엇을 바꿨나

- 기존 전체 농장 원본 외형을 표시한다. 기존 집계29,357메시 / 정점27,049,149개 /
  삼각형48,671,177개이며, 이번에 하우스·잎·줄기 외형을 단순화하지 않았다.
- 원래 승인된 `D_target_only` 접촉 범위다. **타깃 송이와 자기 주줄기**, 로봇·거터·바닥이
  충돌하며 다른 식물은 표시만 한다. 잎 충돌 무시 정책을 유지한다. 전체 농장의 모든
  송이를 탄성·충돌 처리한 모델이 아니다.
- 기준 물리는132body /335joint /385DOF /4,720geom /1,549mesh /10actuator /
  10열매 연결이며, 실제 활성 충돌 geom은901개다. 동적 RB5·리프트·턱, 안쪽 벨트,
  원래 식물의 탄성·밀림·열매 분리를 재사용했다. pose 강제 재생으로 수확하지 않는다.
- 기존 `s300_smoke`의 변환기·계획 sampler·초기상태·중력 preload·각도 spring/deadband·
  분리 adapter와 실제 상태 영상 renderer를 재사용했다. 새 trajectory나 학습기는 만들지 않았다.
- 분리 조건 **3N /0.008Nm**, 활성 접촉 겹침 **1mm**, 질량·마찰·강성·actuator·충돌
  형상을 완화해 성공을 만들지 않았다. 수치 적분 주파수를 바꾼 실험은 각각 표시했다.
- 과거 Plant22에서 이미 떨어져 원래 portable export가 제외했던5열매의 checkpoint·NPZ·
  break event 근거를 `inputs.json`에 남겼다. 이번에 새로 생긴 불리한 물체를 숨긴 것이 아니다.

## CPU와 GPU는 다른 결과다

별도 Python3.12 환경의 Newton1.7.0 /MuJoCo3.15.0 /MuJoCo Warp3.15.0 /Warp1.17.0을 쓴다.
Isaac6.1에 동봉된 Newton1.5.0과 분리했다. 장비는 Threadripper3960X
24코어48스레드, RTX3090Ti24GB, 드라이버570.211.01이다.

기본 CPU는 **Newton `SolverMuJoCo.step` → native MuJoCo CPU**다. 실제 GPU 모드는
**Newton `SolverMuJoCo.step` → MuJoCo Warp CUDA**다. CPU 결과를 GPU 가속 성능으로
설명하지 않는다. Newton metadata의106shape는 실제901개 충돌체 수와 다르며, 물리는
원래 native MjModel 배열을 검증해 업로드하는 제한된 adapter가 계산한다.

원본 USD 직접 import도 시도했지만 convex decomposition이8분 예산을 넘어 중단했다.
완료한 물리 step은0개다. PhysX의 belt surface velocity와 joint break 속성이 새 엔진에서
자동으로 같은 의미가 된다고 가정하지 않고 기존 검증된 MuJoCo 변환 자산을 사용했다.
PhysX와 접촉 결과 자체가 같다는 의미는 아니다.

## GUI/헤드리스 일치와 단일 환경 속도

| 조건 | sim초 /실제 물리 loop초 | 평균 실시간 배율 | 분리 | 활성 최대 겹침 |
|---|---:|---:|---:|---:|
| CPU400/free60 기준, 화면 없음 |82.795 /66.733 |1.241× |6개 |0.681mm |
| 같은 기준, Isaac GUI 표시 |82.795 /68.037 |1.217× |같은6개 |같은0.681mm |
| native 상태 읽기 중복 제거 |82.795 /63.392 |1.306× |같은6개 |같은0.681mm |
| 위 최적화 +표시 전용 geom 분리 |82.795 /61.994 |1.336× |같은6개 |같은0.681mm |
| 같은 현재 기본 최적화 +GUI 표시 |82.795 /63.476 |1.304× |같은6개 |같은0.681mm |

초기 로딩·GUI 준비·결과 압축·MP4 렌더는 이 loop 시간에 포함하지 않는다.
기준 headless 준비는6.919초, GUI 준비는33.582초였고, 최적화 GUI 준비는31.570초였다.
각각1회 측정이며 성능 오차 범위를
정량화한 반복 benchmark는 아니다.

GUI는 별도 Isaac 프로세스에서 **측정한 body pose**를 실시간 표시한다. 물리 프로세스는
Kit를 import하지 않고 명령·dt·적분 순서를 단독 소유한다. GUI의 physics step과 timeline은0이며,
카메라를 돌려도 물리 명령이나 step을 진행시키지 않는다. 최종 측정 프레임이 실제 표시됐는지
ready/ACK와 실제 viewer Python PID를 확인한다.

기준 두 실행은 **13,806개 physics step 전체와 초기 상태를 포함한13,807개 표본**의
시간·명령·phase·qpos/qvel·전체 body pose·열매 연결·접촉 지표·break event가 정확히 같다.
중간까지 같은 prefix만 비교한 PASS가 아니다. 원래 Isaac GUI 급변의 세부 원인은 여전히
미확정이지만, 이 새 실행 구성에서는 해당 재현 동작의 GUI 의존 차이를 제거했다.

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/gui_equivalence.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/gui_equivalence.json)

### 평균 속도와 접촉 구간 속도를 구분해야 한다

물리 결과가 기준과 정확히 같은 native-observables 실행에서 측정했다.

| phase |sim초 |실제 loop초 |실시간 배율 |
|---|---:|---:|---:|
|레일 이동 |30.800 |7.083 |4.349× |
|접근 |3.995 |6.157 |0.649× |
|첫 닫기 |6.000 |12.802 |0.469× |
|두 번째 닫기 |6.000 |11.317 |0.530× |
|마지막 벨트 유지 |2.000 |6.654 |0.301× |
|후퇴 |4.000 |6.209 |0.644× |

따라서 **사용자가 모든 접촉 동작을 실시간으로 편하게 확인한다는 목표는 아직 미달**이다.
solver 반복은 최대20회, 마지막 벨트 단계 평균8.54회라 설정 상한64회를 항상 채워서
느린 상황도 아니다. 주파수만 내리거나 반복 상한만 줄이면 해결된다고 단정하지 않았다.

별도 CPU240Hz/tolerance1e-6 **실패 설정**의 닫기 구간 profiling에서는 native STEP 누적
4,077ms 중 CONSTRAINT3,056ms, COLLISION300ms였다. 그 설정에서는 제약 풀이 쪽 비중이
컸다는 진단이다. 중단된 다른 설정의 계측이므로 현재 기본 구성의 깨끗한 성능 비교로
사용하거나, 충돌체를 줄이면 즉시 빨라진다는 근거로 사용하지 않는다.

## CPU 병렬 시험

같은 장면·같은82.795초 명령을 독립 CPU 프로세스1/8/24/48개로 실행했다.
모든 입력 준비가 끝난 후 gate를 열어 실제 물리 loop가 겹치는 구간을 측정했다.
합산 속도는 `전체 sim초 합 /실제 동시 loop 시간 범위`다. 개별 배율을 단순히 더하지 않았다.

|환경 수 |동시 loop초 |환경당 배율 |합산 sim초/실제초 |최대 RSS GiB |평균 CPU% |
|---:|---:|---:|---:|---:|---:|
|1 |66.268 |1.249× |1.249 |2.43 |117.6 |
|8 |68.476 |1.209× |9.673 |14.28 |729.9 |
|24 |75.634 |1.095× |26.272 |38.82 |1132.9 |
|48 |110.680 |0.748× |35.907 |75.71 |1437.3 |

총**81회 모두 완주·활성1mm 기준 통과·6개 분리**, 저장 상태와 physics 지표 NPZ 각각의
SHA가 모두 한 종류로 같다. 요약 수확 개수만 같은 결과가 아니다.
병렬 시험은 상태 저장5step 간격, 접촉/유효성 지표는 매physics step이다.
위 단일 GUI 비교는 상태도 매step 저장했다. 서로 다른 장면·정책을81개 시험한 결과가 아니다.

자원 통계는 준비·gate 대기·저장까지 포함한 소유 process tree이며 CPU100%는 논리CPU1개다.
48환경 최고 CPU는4771.3%였다. GPU 통계는 다른 프로세스도 포함한 전체 GPU 값이다.
동시 물리 시간과 별도로 저장한 프로세스 전체 실행 시간은1/8/24/48환경 각각
79.271 /92.958 /140.964 /239.266초다. 초기 준비를 공짜로 보이지 않게 구분한다.

**24환경은 환경당 평균 실시간을 유지했고,48환경은 합산 처리량이 가장 컸다.**
48은 이번에 실제 검증한 최대 수이며,64이상의 절대 자원 한계를 입증한 수는 아니다.
이 표는 원래 `inputs_300hz`와 metadata 동기화 설정의 결과로, 선택 최적화를 적용한
다중 환경 성능이라고 섞어 표기하지 않는다.

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu_scaling/summary.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu_scaling/summary.json)

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu_scaling/evidence_identity.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu_scaling/evidence_identity.json)

## 통과하지 못한 대안도 보존했다

|시험 |결과 |채택 여부 |
|---|---|---|
|CPU120/free60 |52.942초, 벨트–열매 겹침1.079mm로 중단 |미채택 |
|CPU300/free60 |55.507초, 꼭지04–벨트1.019mm로 중단 |미채택 |
|CPU240/free60, tolerance1e-6 |54.442초, 열매07–벨트1.354mm로 중단 |미채택 |
|CPU400/free60, CG solver |53.345초, 구조물–열매4.188mm로 중단; 접근 전 분리도 발생 |미채택 |
|CPU400, dense Jacobian |완주1.312× /5개 분리; 벨트 유지0.195× |기준과 동작 달라 미채택 |
|CPU400, native worker4 |완주1.309×, 기준과 같은 결과 |worker0 대비 이득 근거 부족, 기본0 |
|GPU300 고정 주기 초기 실행 |완주0.356× /5개 분리 /활성0.851mm |단일 환경 실시간 미달 |
|GPU400/free60 재검증 |56.525계획초, 활성 겹침4.246mm로 중단 |N8/32 실행 중지 |
|GPU300/free300 재검증 |54.900계획초, 활성 겹침1.028mm로 중단 |N8/32 실행 중지 |

GPU 재검증은 finite q/qvel/qacc·접촉/constraint capacity·실제 접촉 검사를 추가한 실행이다.
capacity는 넘지 않았지만 활성1mm 기준을 통과하지 못했다. 따라서 GPU1/8/32 병렬 처리량을
확보했다거나 GPU 수확 문제를 해결했다고 보고하지 않는다. 초기300Hz 완주만 골라
나중의 중단을 지우지 않았다. GPU float32 실제 clock과 명령 schedule 간 drift도 별도 기록한다.

연결 오류도 수확 실패와 구분했다. 초기 GPU 결과 저장의 `np.r_` 문자열 오류,
benchmark의 잘못된 Python 경로, GUI shell/Python PID ACK 오판을 고치고 회귀시험했다.
해당 초기 폴더와 로그를 보존했으며 성공 건수에 포함하지 않는다.
원본 USD import budget 중단도 물리 실패나 수확 불가능 판정으로 세지 않는다.

## 결과를 바꾸지 않은 최적화

native-observables는 Newton metadata를 표시용으로 매번 갱신하는 중복 계산을 줄이고
원래 native 물리 상태를 직접 읽는다. 전체 상태·명령·접촉·분리 기록이 기준과 정확히 같다.

표시 전용 geom 분리는 별도 opt-in 입력이다. 활성901개는 유지하면서 비충돌3,819개를
물리 MjModel에서 제거했다. 원래 전체 농장 USD 화면은 유지한다.
810종 실제 충돌 메시 및 body/joint/dof/actuator/equality 등을12,370항목 정확 비교했고,
컴파일러가 재계산한 `dof_length`121개도 원래 값으로 복원했다.
MJB는1,150,100,042→595,926,694바이트가 됐다.
전체 실행의 qpos/qvel/pose/equality/접촉/분리 배열이 원래 모델과 정확히 같았다.
최적화 구성도 GUI/헤드리스13,806step 전체가 정확히 같고 실제 최종 프레임 표시를 확인했다.
따라서 이 구성만 기본으로 채택했다. 원본 모델·명령과 실패한 대안들은 그대로 보존했다.

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/native_observables_equivalence.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/native_observables_equivalence.json)

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/render_separation_runtime_equivalence.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/render_separation_runtime_equivalence.json)

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/gui_optimized_equivalence.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/gui_optimized_equivalence.json)

## 저장한 증거와 남은 목표

Git의 작은 요약은 [validation/newton_server_20261016.json](harvest_s300/validation/newton_server_20261016.json)이다.
전체 결과 대시보드:

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/index.html](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/index.html)

최종 자동검사83개와 USD/표시 검사11개, 합계94개가 통과했다.
별도 환경의 `pip check`와 shell 문법 검사도 통과했다. 자동검사는 위 실제 온실 수확 실행을
대신하는 증거가 아니며, 서버 실험과 구분한다.
NAS에는 inputs의 원본/자산/SHA, 실행별 명령·실제 전체 상태·매step 접촉 지표·분리 이벤트,
setup/loop/resource 로그, GUI ready/ACK/최종 화면, 실패 실행을 그대로 보존한다.

현재 확인한 수확은 **한 번 진입한 뒤 두 차례 닫기/열기에서 열매6개가 모델상 분리된 것**이다.
한 송이10개 완전 수확, 포집, 레일 전체 연속 수확, RGB-D 선택·학습·실물 손상 검증은 아니다.
분리 과실의 바닥·기체·같은 송이 접촉은 기존 lifecycle 진단 예외이며,
최대9.681mm인 그 값을 활성 물리 통과 수치0.681mm에 섞거나 숨기지 않는다.
`training_eligible=false`, `harvest_success=null`을 유지한다.

다음 판단은 GUI와 분리된 현재 CPU 실행기를 기준으로 접촉 구간 자체를 가속하는 것이다.
원래 충돌 형상을 유지한 최적화의 이득은 수% 수준이었고, CG·dense·낮은Hz는 결과가 달라
기본 설정으로 채택하지 않았다. GPU는 먼저 동일 입력의 유효한 단일 실행을 확보한 뒤
동일한 검사로 N을 늘려야 한다. 주변 식물 전체 탄성화·대규모 학습부터 늘리지 않는다.

영상은 저장된 **실제 qpos/body pose**를 기존 renderer로 그렸다. 명목 계획 영상이나
두 번째 물리 실행이 아니다. 둘 다960×720 /15FPS /1배속이며 작은 자막을 사용했다.
전체 프레임 ffmpeg 디코드·프레임 수 검사를 통과했다. 렌더 중 physics step과 timeline은0,
저장된 physics 상태SHA는 그대로다.

1. **전체 동작**: 실제0~82.795초,1,243프레임 /영상82.867초.
   숨긴 형상 없이 전체 농장을 표시하며, 전경 잎 때문에 접촉부가 일부 가려진다.

   [/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_render_separated_gui/newton_actual_entry.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_render_separated_gui/newton_actual_entry.mp4)

2. **송이 접촉 탑뷰**: 실제45~71.995초,406프레임 /영상27.067초.
   접촉부를 보기 위해 **재생 화면에서만** 배경 식물과 타깃 잎133경로를 숨겼다.
   계산된 물리와 원본 상태를 바꾸지 않은 cutaway이며, 전체 농장 표시 영상과 구분한다.

   [/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_render_separated_gui/newton_actual_truss_top.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/cpu400_render_separated_gui/newton_actual_truss_top.mp4)

[/root/docker_share/mujoko_debugging_data/20261016_s300_newton/video_validation.json](/root/docker_share/mujoko_debugging_data/20261016_s300_newton/video_validation.json)
