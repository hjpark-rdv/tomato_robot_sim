# Newton GPU 솔버·식물 표현 비교 (2026-10-17)

전체 수확 시퀀스의 GPU 실행은 원본 약 **1.56배속**에서 변경 식물 표현 약 **2.50배속**으로 빨라졌다. 다만 가장 느린 벨트 접촉 단계는 아직 약 **0.62배속**이다. 따라서 **접촉 구간까지 전부 실시간이라는 목표는 미달성**이다. CPU 성능이나 이동 구간 평균으로 이 한계를 대체하지 않는다.

권장 시험 후보는 Newton `SolverMuJoCo → MuJoCo Warp CUDA`에 **133자유도 식물 + 외형 확장 0.1mm 이내 열매 proxy**를 사용한다. 원본을 그대로 빠르게 만든 결과가 아니라, 물리 표현을 변경하고 별도로 검증한 실험 모드다. 기존 `gpu` 기본값은 원본을 유지한다.

## 코드·원자료

- 브랜치: `codex/s300-newton-solver-compare-20261017`
- 시작 코드: `7930cb0eddf015b22962edce7f8df8849684f969`
- 구현 커밋: `507653e` (결과 커밋은 이 커밋을 포함한다).
- 전체 원자료: `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare`
- 결과 대시보드: 같은 폴더 `index.html`; 전체 실행/단계 표: `comparison/runs.csv`, `comparison/phases.csv`.
- 작은 Git 보존 요약: [validation/s300_newton_solver_comparison.json](validation/s300_newton_solver_comparison.json).
- 원본 입력: `/root/docker_share/mujoko_debugging_data/20261016_s300_newton/inputs_render_separated`

RTX 3090 Ti 24GB, Threadripper 3960X에서 측정했다. 독립 Newton 1.7.0 / Warp 1.17.0 / MuJoCo 3.15.0, Isaac Sim 6.1.0 화면을 사용했다. 기존 패키지·원본 장면·미커밋 gripper 작업은 덮어쓰지 않았다.

## 고정한 작업

같은 82.795초 명령표, 13,806 physics step, 접촉 400Hz / 자유 이동 60Hz를 재사용했다. 준비 상태에서 시작해 레일·리프트 이동 → 미리 열기 → 수평 접근 → 양쪽 안쪽 방향 벨트 → 닫기·열기 2회 → 후퇴까지 실행한다. 각 실행은 새 초기 상태에서 시작하며 파단된 상태를 다음 반복으로 넘기지 않는다.

로봇·리프트·그리퍼·벨트 형상, 명령 시간, actuator, 마찰, margin/gap, 타깃 송이·주줄기 충돌 범위, 1mm 활성 관통 기준, 3N/0.008Nm 과실 연결 해제 조건은 유지했다. 잎은 사용자 승인대로 충돌하지 않고, 분리 열매의 기존 허용 접촉은 별도 기록한다. 허용된 분리 열매의 낙하 접촉까지 포함한 최대값을 활성 관통 실패와 혼동하지 않는다.

농장 전체 시각 배경은 원본 USD를 사용한다. 화면은 별도 Isaac 프로세스가 **현재 계산 중인 실제 상태**를 받아 표시하고 physics를 진행시키지 않는다. 최종 MP4는 저장 상태를 재생한 것이며, GUI/headless 일치 검증은 별도의 새 physics 실행으로 수행했다.

## 실행한 비교

| 후보 | 실제 시험 | 판단 |
|---|---|---|
| A. 현재 Newton → MuJoCo Warp | 원본 전체 재측정·프로파일·기존 결과 회귀 비교 | 원본 기준 확보, 전체 상태/접촉/분리 이벤트가 기존 검증 결과와 정확히 일치 |
| B. 공식 Isaac Newton importer | 실제 USD → 공식 importer → GPU 120step | 동적 과실 누락 수정. 벨트·preload·분리 adapter 미연결로 수확 성능 비교에서 제외 |
| C. XPBD / VBD | 원본 실제 식물 collider·질량의 작은 push/복원 fixture | XPBD nonfinite, VBD 큰 반응 차이. legacy와 modern compliant-ALM 모두 실행 후 제외 |
| D. 공식 MuJoCo↔VBD ADMM | 실제 식물과 동적 belt-box의 접촉/비접촉 대조 | 양방향 반작용 확인. 침투 1.0226mm 및 반응 차이로 현재 기준 미통과 |
| E. 식물 좌표 표현 변경 | 잎 관절 고정, 2/4분절 compliance 보정, 전체 수확 | 유효한 전체 실행과 속도 개선 확보. 동일 모델 최적화라고 주장하지 않음 |
| 열매 보수적 convex proxy | 원래 과실 외형을 모두 포함하는 두 오차 단계 | 0.1mm 후보 권장. 0.25mm 후보는 더 빠르지만 접촉·분리 시점 변화가 큼 |
| 해석적 타원체 과실 | 실제 원본 hull을 포함하도록 fitting, 전체 수확 1회 | 2.500배속·활성0.705mm·6개 분리. 전체 이득이 거의 없고 형상 차이가 더 커서 교체하지 않음 |
| 기존 GPU solver 옵션 | broadphase AABB/SAP, CG, 수렴 기준, 배치 선형대수 | 실패/연결 오류/짧은 대조를 전체 수확 속도로 집계하지 않음 |

설치 버전의 공식 API·소스를 확인하고 실제 GPU에서 시험했다. 공식 Isaac 경로 역시 내부 `SolverMuJoCo`를 쓰므로 별도 물리 솔버로 세지 않는다. 자세한 제외 근거는 [공식 연결](harvest_s300/native_newton_comparison_ko.md), [XPBD/VBD·양방향 결합](harvest_s300/alternative_solver_results.md), [proxy의 외형 보증](harvest_s300/newton_fruit_proxy_ko.md)에 있다.

## 전체 실행과 접촉 단계

아래는 프로파일 계측을 끈 선별 실행이다. RTF = 시뮬레이션 시간 / 완료까지 실제 시간. 6개 분리는 연결 해제 수이며 송이 전체 10개 수확·수거 성공이 아니다.

| 모델 | nv | 전체 RTF | 접근 | 첫 닫기 | 재닫기 | 마지막 벨트 | 활성 최대 침투 | 분리 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 원본 | 385 | 1.556 | 0.873 | 0.499 | 0.652 | 0.378 | 0.794mm | 6 |
| 충돌 없는 잎 관절만 고정 | 304 | 1.678 | 0.948 | 0.523 | 0.683 | 0.430 | 0.766mm | 6 |
| 2분절 보정 | 190 | 2.069 | 1.203 | 0.660 | 0.799 | 0.485 | 0.680mm | 6 |
| 4분절 보정 | 133 | 2.436 | 1.358 | 0.769 | 1.057 | 0.596 | 0.658mm | 6 |
| 4분절 + 0.1mm proxy | 133 | 2.503 | 1.396 | 0.808 | 1.064 | 0.621 | 0.657mm | 6 |
| 4분절 + 0.25mm proxy | 133 | 2.587 | 1.460 | 0.871 | 1.061 | 0.644 | 0.655mm | 6 |
| 4분절 + 해석적 타원체 | 133 | 2.500 | 1.471 | 0.936 | 0.989 | 0.650 | 0.705mm | 6 |

0.1mm proxy는 원본 26,240개 과실 정점을 9,994개로 줄였고, 실제 컴파일한 외형의 확장 상한은 **0.08687mm**다. 0.25mm proxy는 3,058개 / 0.24805mm다. 외형을 안쪽으로 깎아 접촉을 없애지 않았다.

정점 탐색을 피하는 해석적 타원체도 실제로 시험했다. 원본 과실 hull 전체 포함을 검증했고 질량·관성은 그대로다. 다만 외형 차이의 표본 support gap 최대는 약0.846mm이며, 이는 convex proxy처럼 증명한 전 방향 오차 상한이 아니다. 첫 닫기는 빨라져도 전체 속도 이득이 거의 없고 모든 접촉 구간 실시간에도 도달하지 못해, 더 작은 형상 오차의0.1mm convex proxy를 권장한다. 타원체는1회 선별 결과이며 반복·GUI 검증 후보로 승격하지 않았다. [타원체 상세](harvest_s300/newton_fruit_ellipsoid_ko.md).

타원체의 최대 전체 접촉력은49.87N, Fruit_07 분리는 재닫기 중63.15초로 바뀌었다. 힘의 정확한 pair 귀속은 기존 전체 최대 기록만으로 알 수 없으며, 타깃 부착 하중이나 손상이라고 단정하지 않는다. 같은6개 분리라는 숫자만으로 proxy 후보와 동등하다고 보지 않는다.

0.25mm 후보는 최대 전체 접촉력이 43.85N으로 커졌고 Fruit_07의 분리가 64.01초에 발생했다. 0.1mm 후보는 15.16N, Fruit_07 53.92초였다. 원본은 18.44N / 54.96초다. 최대 전체 접촉력에는 분리 열매 접촉도 포함되므로 이를 타깃 부착 힘이나 손상 힘으로 해석하지 않는다. 단순 분리 개수와 침투 통과만으로 물리 반응이 같다고 판정하지 않고 0.1mm 후보를 권장한다.

CPU 참조의 같은 133자유도 모델은 약 4.03배속·5개 분리였다. GPU는 float32와 별도 solver reduction을 사용해 6개가 분리된다. CPU/GPU 간 물리 궤적까지 동일하다고 주장하지 않으며 CPU 속도를 GPU 달성 실적으로 세지 않는다.

## 무엇을 바꿨고 어떻게 검증했나

모든 변경 입력은 별도 폴더에 저장했다. body 132개, collider 901개, 원래 질량·관성·초기 transform·과실 10개·연결 10개·로봇 actuator·명령 파일을 보존했다. 충돌하지 않던 잎 27개의 독립 관절 81자유도는 고정하고 질량과 줄기 추종은 남겼다.

줄기·꼭지의 굽힘 좌표는 두/네 분절마다 모으고 **수확 실행 전에** 양수 spring compliance를 맞췄다. 4분절의 정적 선형 compliance 상대 오차는 약 1.32%다. 그러나 고정된 내부 분절은 더 이상 서로 움직이지 않는다. 독립 collider 감사 결과 4분절 모델에서 원래 충돌 가능했던 **55개 내부 pair가 새 weld group 내부로 들어가 억제**된다. 외부 장애물 충돌체를 제거한 것은 아니지만 충돌 자격도 원본과 완전히 같지는 않다. 2분절 모델은 새 내부 pair 38개가 모두 원래 exclude 대상이어서 추가로 잃은 충돌 가능 pair는 0개다.

원본/변경 모델에 같은 CPU 적분기로 무입력, 줄기 0.2N 밀기·복원, 열매 0.05N 당기기, 0.002Nm 비틀기 3초 시험을 했다. 이는 표현 차이의 offline 반응 검사이며 GPU 성능 시험이 아니다. 진단에서는 연결을 유지하고, 실제 수확 실행에서만 원래 조건으로 파단한다. `response_v2_*`는 적분 후 kinematics를 갱신해 timestamp와 pose를 맞춘 수정 결과다. 이전 `response_*`는 한 step 전 pose를 기록한 과거 진단이라 최종 반응 표에 사용하지 않는다.

반응 비교의 작은 오차는 원래 모델과의 일관성 근거다. **실물 stiffness·마찰·손상·과실 분리 정확도는 미보정**이며, 한 장면에서 확보한 결과를 모든 배치에 일반화하지 않는다. 실제 수확 중 분리 시점도 모델 사이에서 달라진다.

| 133DOF와 원본의 위치 반응 차이 | 관측 body 전체 최대 | 전체 시간·body RMS |
|---|---:|---:|
| 줄기 0.2N 밀기·복원 | 0.579mm | 0.0792mm |
| 열매 0.05N 당기기 | 0.173mm | 0.0286mm |
| 열매 0.002Nm 비틀기 | 0.0578mm | 0.0130mm |

비틀기에서 Fruit_04의 원본 최대 회전은 첫 관측 대비 약14.08°이며, 133DOF 후보와의 자세 차이 최대는 약1.112°였다. 위치 차이가 작다는 이유만으로 회전 반응까지 동일하다고 주장하지 않는다.

## 병목과 연결 수정

원본 접촉 단계 CUDA event 프로파일에서 collision 약 1.4ms, Newton constraint solve 약 4.4~5.0ms가 관찰됐다. 190자유도 모델은 collision이 약 1.4ms로 남고 solve가 약 2.15~3.37ms로 줄었다. 단순 Python 로그 제거만으로 해결되는 병목이 아니다. 계측 실행의 속도와 clean 실행 속도를 섞지 않았다.

마지막으로 권장0.1mm 후보도 별도 전체 계측 실행했고 clean 결과와 모든 상태/접촉/분리가 정확히 일치했다. 실제 첫 닫기 표본(step5885, Newton10회)은 collision **1.228ms**, solve **2.608ms**였다. 벨트 마지막 표본(step10485,6회)은1.050/1.742ms였다. nested event 구간은 겹치므로 모두 합산하지 않는다. 이 결과는 모든 스텝의 평균이 아니라 실제 접촉 상태에서 채취한 진단 표본이다.

GPU graph, GPU 제어·분리·관통 판정, 제한된 CPU 동기화, 고정 reduction을 기존 구현에서 재사용했다. AABB/SAP 교체는 190자유도 전체 약 2.06배속으로 유의한 추가 개선이 없었다. CG는 작은 정상 대조와 원본 CPU에서는 작동했지만 실제 GPU 식물에서 조기 파단·1mm 초과가 발생했다. 두 번의 전체 시도는 중단 결과로 남기고 빠른 완료 실적으로 세지 않았다.

수렴 허용오차도 실제 GPU 유효값 약1e-6→1e-5로 한 번 비교했다. 첫 닫기 개선은 약1.9%에 그쳤고, 분리 Fruit08과 거터의 겹침으로56.57/82.795초에 중단했다(최대5.149mm). 이 짧은 prefix의 RTF3.197을 전체 속도로 세지 않았다. 더 느슨한1e-4 시험은 실행하지 않고 원래 설정을 유지했다. [힘·분리·추종·단계별 상세](harvest_s300/alternative_solver_convergence_results.md).

4분절은 plant tree가 63DOF가 되어 native의 작은 dense inertia 경로를 쓴다. 이때 기존 adapter가 빈 sparse tail slice를 만들던 연결 오류를 고쳤다. 빈 tail이면 native 계산을 그대로 쓰는 no-op이며 물성이나 방정식을 바꾸지 않는다. 이 수정 전 시작 오류를 수확 물리 실패로 세지 않는다.

## 반복·화면 일치

4분절 후보는 한 번 로딩한 session에서 headless 2회와 실제 카메라 이동 GUI 1회를 완주했다. 전체 RTF는 2.428 / 2.429 / 2.125였다. 13,806 physics step의 qpos/qvel/물체 pose/접촉/침투/연결 힘/분리 이벤트가 **정확히 일치**했다. 0.25mm 후보도 2.553 / 2.555 / 2.243으로 완주했다. 권장 0.1mm 후보는 **2.487 / 2.489 / GUI 2.170배속**, 각6개 분리·0.6565mm였고, headless 반복과 카메라 이동 GUI 모두 전체 비교가 정확히 일치했다.

전체 명령표 끝까지 실행했는지, 13,807개 상태가 있는지, 명령/상태 SHA가 맞는지 확인한다. float32 native clock의 누적 오차는 별도로 남기고 실제 solver step과 signed schedule로 제어한다. 저장 영상 재생을 fresh GUI physics 비교의 대신으로 사용하지 않았다. GUI는 15FPS 상태 소비자이며 느린 화면 표시가 물리 dt를 바꾸지 않는다.

초기화·JIT·graph capture·반복 loop·저장/렌더 시간을 별도로 남겼다. RTF는 동기화 후 실제 loop 완료 시간이며 비동기 CUDA 제출 시간만 측정한 값이 아니다. 첫 실행의 컴파일·농장 화면 로딩은 clean 반복 RTF에 들어가지 않으므로 첫 로딩까지 실시간이라는 뜻도 아니다.

권장 후보의 첫/반복 headless setup은 약9.49/3.96초, GUI setup은 약30.28초다. 실제 loop는 각각33.29/33.26/38.15초였다. 캐시가 채워진 이 실행의 graph capture는 약1.76/0.61/0.59초이며, 캐시가 없는 설치 직후의 최악 JIT 시간을 나타내지는 않는다.

## GPU 병렬

다중 환경은 별도 수치 경로인 native Warp 선형대수로 먼저 검증했다. 입력은 **proxy를 적용하지 않은 133자유도 모델**이다. 단일 환경 최저 지연에 쓰는 고정 reduction/cuSOLVER 경로와 구분한다.

| 환경 수 | 환경당 전체 RTF | 합산 처리량 (sim-s / wall-s) |
|---:|---:|---:|
| 1 | 1.229 | 1.229 |
| 2 | 1.093 | 2.187 |
| 4 | 0.907 | 3.629 |
| 8 | 0.669 | 5.356 |

모든 환경의 전체 qpos/qvel/eq/time, 8종 접촉·연결 지표, 원본 분리 이벤트를 단일 환경과 비교해 정확히 일치했다. 각각 6개 분리·0.683mm 활성 침투다. 동일 장면·명령 복제의 격리/처리량 시험이며 서로 다른 작업의 일반화 시험은 아니다.

8환경은 단일 환경 대비 처리량 약 4.36배지만 환경 하나는 더 느리다. sampled GPU 사용률 평균 약 93.6%, 장치 전체 메모리 최대 약 4,879MiB였다. 이는 메모리 한계나 최대 가능한 N을 찾았다는 뜻이 아니다. CPU 약 99%는 논리 코어 하나에 가까운 사용량이며 16코어 병렬 물리 실행이라는 뜻도 아니다. batched cuSOLVER의 추가 비교 결과는 보존 JSON에 별도 기록한다.

batched cuSOLVER도 같은 입력 N=1에서 실제 완주했다. 1.150배속·0.7066mm로 native Warp의1.229배속보다 느렸으므로 조건부 N=4 추가 실행은 하지 않았다. 전체 GUI 일치를 확인한 단일 환경 권장 경로와 병렬 처리량 경로를 따로 둔다.

## 실행

현재 서버에서 권장 변경 모델을 실제 physics + Isaac 화면으로 실행:

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh gpu-fast --gui
```

화면 없이 실행하려면 `--gui`를 빼고, 한 번 로딩 후 반복하려면 `--repeats 3`을 추가한다. 기존 원본은 `./nvidia-sim/run_s300_newton.sh gpu --gui`로 실행한다. `gpu-fast`는 검증 입력 폴더를 참조하며 파일이 없으면 자동으로 다른 모델로 대체하지 않고 중단한다. 다른 서버에서는 원본 자산·환경과 함께 입력 폴더를 옮기거나 다음 준비 기능을 재사용한다.

```bash
PY=/root/farmily_tomato/nvidia-sim/.newton-venv/bin/python
$PY nvidia-sim/harvest_s300/newton_plant_variant.py ORIGINAL_INPUTS NEW_COARSE4 --mode coarse4
$PY nvidia-sim/harvest_s300/newton_fruit_proxy.py NEW_COARSE4 NEW_FAST_INPUTS --max-error-m 0.0001
S300_NEWTON_FAST_INPUTS=NEW_FAST_INPUTS ./nvidia-sim/run_s300_newton.sh gpu-fast --gui
```

새 출력 경로를 사용한다. 입력 hash·증거·MP4를 덮어쓰지 않는다. 실험 manifest 실행과 집계는 기존 실행기를 호출하는 `benchmark_solver_compare.py`를 사용한다.

## 남은 일과 다음 판단

현재 유효하게 완주하는 최선은 기존 MuJoCo Warp 계열이다. 새로운 솔버라는 이유만으로 공식 importer·XPBD·VBD를 우승자로 바꾸지 않는다. 다음 성능 작업은 실제 접촉 solve/collision의 단일 환경 지연을 줄이는 데 집중해야 한다. 큰 식물의 반응·분리 힘 adapter까지 미완료인 VBD 전체 이식이나 RL을 먼저 시작할 근거는 없다.

가장 느린 벨트 단계는 물리1step당 평균 약4.03ms로,400Hz의2.5ms 예산보다 약1.53ms 길다. 따라서 목표를 달성하려면 그 단계의 총 비용을 추가로 약38% 줄여야 한다. GPU 제약 solve의 반복당 계산/launch 비용과 충돌 검출을 함께 줄이는 개선이 필요하다. 후보 변경 없이 단순히 N을 늘리거나 화면 FPS를 낮추는 것으로 단일 접촉 지연 목표가 해결되지는 않았다. 이번에 시험한 더 느슨한 수렴 설정과 더 큰 충돌 근사는 물리 차이를 늘려 추가 선택 근거가 되지 못했다.

이 결과는 `training_eligible=false`, `harvest_success=null`인 성능/물리 비교다. 새 수확 알고리즘, 대규모 수집, 주변 식물 전체 탄성화는 하지 않았다. 접촉 구간 실시간 목표·실물 보정·서로 다른 배치의 병렬 검증은 미완료로 유지한다.

실제 MP4 목록과 SHA·완료 검사·반복 수치는 대시보드와 작은 보존 JSON에 모두 포함한다. 원본 영상은 현 baseline이 과거 검증 실행과 정확히 일치함을 확인한 뒤 재사용하고, 변경 모델 영상은 이번에 저장한 실제 physics 상태로 생성했다. 접촉 확대 cutaway는 재생 화면에서만 주변 잎을 숨기며 physics 충돌 정책을 바꾸지 않는다.

## 검사와 영상

최종 Newton 회귀177개 중158개 통과·19개 명시적 skip, controller/대체 솔버14개 통과, 공식 importer5개·USD 준비3개 통과다. CUDA 전용 제약 순서·CG 대조·dense-only inertia 검사는 별도 실제 GPU 로그에 남겼다. skip된 host suite를 실제 CUDA 실행 통과로 세지 않는다. 모든 새 MP4 4개를 끝까지 decode했고, 렌더 후 source 상태 파일 SHA가 바뀌지 않았음을 확인했다. 자막은 `FontSize=6`이다.

권장 후보의 전체 환경/접촉 확대:

- [/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/proxy01_screen_campaign/coarse4_proxy01_screen/actual_recommended_gpu_full_aisle_1x.mp4](/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/proxy01_screen_campaign/coarse4_proxy01_screen/actual_recommended_gpu_full_aisle_1x.mp4)
- [/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/proxy01_screen_campaign/coarse4_proxy01_screen/actual_recommended_gpu_contact_1x.mp4](/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/proxy01_screen_campaign/coarse4_proxy01_screen/actual_recommended_gpu_contact_1x.mp4)

0.25mm 비교 후보의 전체/접촉 확대 (전체 시점은 전경 잎에 가려져 접촉 확인에는 확대 영상을 사용):

- [/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/proxy025_screen_campaign/coarse4_proxy025_screen/actual_best_gpu_full_1x.mp4](/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/proxy025_screen_campaign/coarse4_proxy025_screen/actual_best_gpu_full_1x.mp4)
- [/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/proxy025_screen_campaign/coarse4_proxy025_screen/actual_best_gpu_contact_1x.mp4](/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/proxy025_screen_campaign/coarse4_proxy025_screen/actual_best_gpu_contact_1x.mp4)

원본 기준 영상 (이번 baseline과 전체 상태 일치를 확인한 기존 저장 상태):

- [/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/final_session/repeat_001/actual_gpu_full_1x.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/final_session/repeat_001/actual_gpu_full_1x.mp4)
- [/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/final_session/repeat_001/actual_gpu_contact_1x.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/final_session/repeat_001/actual_gpu_contact_1x.mp4)
