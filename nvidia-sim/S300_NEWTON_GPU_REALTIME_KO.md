# S300 Newton GPU 후속 최적화 결과

2026-10-17 KST. 브랜치 `codex/s300-newton-gpu-realtime-20261016`, 출발 커밋
`18686103228103de2b64ff1ebbbb88ddf9f594a9`.

**전체 평균은 headless 1.561배속, 카메라 이동 GUI 1.351배속으로 개선됐다.
접촉 구간까지 실시간이라는 최종 목표는 아직 달성하지 못했다.**
같은 입력의 이전 GPU 실행보다 경과 시간을 각각 36.8%, 36.0% 줄였다.
전체 13,806스텝의 실제 상태·접촉 지표·힘·분리 이벤트는 이전 결과와 정확히 같다.
새 headless/GUI 쌍도 전체 기록이 정확히 같다. CPU 최적화로 대신한 결과가 아니다.

## 비교 조건과 실행 구조

Isaac Sim 6.1.0의 전체 농장·로봇 화면과 별도 Newton 프로세스를 연결한다.
Newton SolverMuJoCo → MuJoCo Warp CUDA가 실제 적분·접촉·제약을 GPU에서 계산한다.
Isaac은 공유된 측정 상태를 표시하며 자체 physics step/timeline은 0이다.
GUI 검증은 새 초기 상태에서 실제 물리를 다시 수행했다. 저장 영상 재생과 구분한다.

- RTX 3090 Ti 24GiB, driver 570.211.01, Threadripper 3960X, RAM 약 125.7GiB.
- Newton 1.7.0, MuJoCo/MuJoCo Warp 3.15.0, Warp 1.17.0.
- 같은 132 body, 335 joint, nq395/nv385, 활성 geom901/mesh810, actuator10, CONNECT10.
- 타깃 송이와 자기 주줄기의 충돌, 거터·로봇·바닥, 기존 잎 무시 및 다른 식물 비충돌 정책 유지.
  전체 농장의 외형은 실제 GUI에 유지한다. 주변 식물을 새로 탄성화하지 않았다.
- 같은 초기 상태, 기체·리프트·팔·그리퍼·벨트 명령. 진입 전 열기, 두 차례 개폐, 분리, 후퇴 포함.
- 접촉 400Hz / 자유 이동 60Hz, 예정 82.795초 / 13,806스텝. 활성 관통 1mm 유지.
- 분리 조건 force > 3N 또는 passive torque > 0.008Nm와 기존 분리 열매 접촉 예외 유지.
  충돌체, 물성, margin/gap, 마찰, actuator 및 수렴 기준을 바꾸지 않았다.
- `training_eligible=false`, `harvest_success=null`. 6개 분리는 연결 해제 수이며
  10개 전체 수확, 포집 또는 실물 손상 한계 검증을 뜻하지 않는다.

원본 입력:
`/root/docker_share/mujoko_debugging_data/20261016_s300_newton/inputs_render_separated`

| 해시 대상 | SHA256 |
|---|---|
| source MJB | `2029e0e4cb5c9775b2138abadc6cd290045d89fefcd79b5065e5cedf2a5377c0` |
| runtime source model arrays | `2ac335c86cb40b642ae287c7eb380a920c0debdd7e6763874bf3648bb9f636a9` |
| 실행 명령 NPZ | `d187adac43e87b05c8d1e6588bd094f9260451eaab2456f80d7984176350da0a` |
| reset 초기 buffer | `99d0f9ec8ecc2d192059eda7c8228987de9949dd6d610c11beeb94562eb920b8` |

실행별 코드 해시·입력·명령·구간별 수치·영상 해시는
[작은 검증 JSON](validation/s300_newton_gpu_realtime.json)에 보존한다.
대형 원자료와 기존 미커밋 gripper 자산은 변경하지 않았다.

## 전체 성능과 물리 결과

| 실행 | loop 경과초 | 전체 RTF | 분리 | 활성 최대 관통 mm |
|---|---:|---:|---:|---:|
| 이전 GPU headless | 83.983 | 0.986 | 6 | 0.794207 |
| 새 GPU headless | 53.052 | 1.561 | 6 | 0.794207 |
| 이전 GPU + Isaac GUI | 95.780 | 0.864 | 6 | 0.794207 |
| 새 GPU + Isaac GUI·카메라 이동 | 61.267 | 1.351 | 6 | 0.794207 |
| CPU 참고값, 같은 source/400·60Hz | 61.994 | 1.336 | 6 | 0.680636 |

RTF는 실제 기록된 시뮬레이션 시간 / GPU 완료와 상태 읽기를 포함한 loop 경과 시간이다.
각 비교는 개별 실행 측정이며 통계적 신뢰구간은 아니다. CPU 행은 기존 비교 실행값이다.
CPU float64/tolerance1e-8, GPU float32/업로드 tolerance 최소1e-6 차이가 있어
CPU와 GPU의 궤적 동일성이나 PhysX 대비 엔진 속도 배수로 해석하지 않는다.

최종 두 실행은 각각 전 구간을 완료했다. 분리 대상은 02/03/04/05/07/08이다.
qpos/qvel, body pose, 시간·명령·phase, equality, 매 스텝 접촉·부착 힘/토크,
분리 대상·시점의 비교 오차가 모두 0이다. 원본 설정을 느슨하게 바꾸지 않았다.
분리 열매 최대 겹침 10.234831mm와 최대 접촉력 18.444820N도 원자료로 남긴다.
분리 열매의 기존 예외가 적용되므로 전체 geometry pair가 1mm 이내라는 뜻은 아니다.

GUI 비교는 같은 retained model/process에서 완전 reset한 두 번째 실행이다.
649회의 실제 카메라 위치 측정·이동과 최종 물리 상태 표시 ACK를 기록했다.
다른 프로세스의 이전 최적화 실행과도 물리 배열이 정확히 같았지만,
다른 GPU/드라이버/장면 전체의 결정성을 입증한 것은 아니다.

새 GUI 쌍의 `final_gui_comparison.json`은 설정까지 같아 PASS다.
이전 구현과의 `prior_to_final_comparison.json`은 **설정 동일성은 FAIL**이다.
고정 reduction 색인·제약 정렬·결정성 커널 설정이 실제로 바뀌었기 때문이다.
그 파일의 `all_compared_arrays_exact=true`, `break_events_exact=true`, 전체 coverage 통과가
최적화 전후 물리 기록 동일성의 증거다. 비교기를 느슨하게 바꿔 PASS시키지 않았다.

GPU float32 시계는 최종 82.793121초로 예정 시간보다 1.878662ms 작고,
전체 구간 최대 누적 오차는 11.0657ms다. 이전 실행과 같은 오차이며
명시적 solver step과 저장된 비균일 timestep schedule로 누락 없는 실행을 확인했다.

## 단계별 RTF

| 단계 | headless | GUI |
|---|---:|---:|
| settle | 4.767 | 4.092 |
| rail_move | 5.075 | 4.387 |
| ready_hold | 5.031 | 4.024 |
| prepare_open | 4.915 | 4.326 |
| prepare_open_hold | 4.838 | 4.282 |
| prepare_lift_branch | 4.772 | 4.112 |
| prepare_joint_branch | 5.227 | 4.578 |
| approach | 0.875 | 0.759 |
| belt_start_open | 0.893 | 0.767 |
| close | 0.499 | 0.432 |
| cycle_open | 0.480 | 0.414 |
| cycle_open_hold | 0.595 | 0.516 |
| cycle_reclose | 0.655 | 0.569 |
| belt_finish | 0.382 | 0.328 |
| cycle_open_final | 0.667 | 0.580 |
| cycle_open_final_hold | 0.905 | 0.792 |
| withdraw | 0.913 | 0.795 |
| return_joint_branch | 5.786 | 5.019 |
| return_lift_branch | 5.236 | 4.486 |
| ready_close | 5.395 | 4.719 |
| hold | 5.329 | 4.448 |

닫기는 이전 headless 0.333→0.499배속, 벨트 끝은 0.244→0.382배속이다.
개선됐지만 전체 평균으로 접촉 구간의 실시간 미달을 가리지 않는다.

## 적용한 최적화

1. **phase 경계의 나머지 스텝도 CUDA graph 사용.** 기존 10스텝 batch 밖의 16스텝에서
   결정성 커널의 호스트 준비가 반복됐다. 동일 1스텝 graph로 실행하고 13,806스텝 전체를
   graph에 포함했다. `full_graph_steps=13790`, `tail_graph_steps=16`, `uncaptured_steps=0`.
   native 상태만 사용하는 모드에 한정하고, capture 중 물리 진행이 0인지 확인한다.
2. **고정 순서 힘 합산의 sparse 색인 재사용.** 매 Newton 반복마다 하던 이진 탐색을
   매 물리 스텝 constraint 생성 후 한 번 구축한 색인으로 대체한다.
   행 수 변경·분리·reset마다 갱신하며 합산 순서는 유지한다.
3. **불필요한 결정성 처리 제거.** 검토한 MW3.15 커널의 정수 OR/감소 및 활성 atomic scatter가
   없는 sparse/elliptic 특수화에만 일반 CUDA 실행을 사용한다. 부동소수 합산의 순서 고정은 유지한다.
   atomic 반환값으로 순서를 결정하는 row reservation과 혼동하지 않는다.
4. **제약 행을 생성 후 정렬.** native 병렬 allocation 뒤 `(type, id, constraint 내부 성분)` 순서로
   정렬한다. 14개 행 필드와 contact/group 포인터를 함께 옮긴다. sparse Jacobian 값 자체는
   변경하지 않는다. 직렬화된 allocation 비용을 줄이면서 같은 계산 순서를 복구한다.
   현재 모델 기능·schema와 단일 환경에만 허용하고 불일치 시 거부한다.

`run_s300_newton.sh gpu` 기본값에 네 변경을 적용했다. 설치 패키지 파일을 직접 수정하지 않았다.
기존 source model과 수확 알고리즘을 재사용한다.

## 실제 GPU 계측으로 확인한 남은 병목

최종 `final_graph_profile`은 실제 captured graph의 GPU 구간을 계측했다.
바깥 구간은 CUDA event, 조건부 solver graph 내부는 GPU `%globaltimer`를 사용했다.
cuSOLVER/cuBLAS 실행을 포함한다. 계측 오버헤드가 있으므로 성능표는 계측하지 않은 실행값이다.
중첩 구간은 서로 겹치므로 단순 합산하지 않는다.

대표 `belt_finish`, step10485, 제약85개, Newton 반복10회:

| 측정 구간 | GPU ms | 범위 |
|---|---:|---|
| collision | 1.375 | 대표 물리 스텝 |
| make_constraint | 0.203 | 대표 물리 스텝 |
| solve | 4.921 | 대표 물리 스텝의 전체 반복 |
| update_constraint | 0.049 | 마지막 내부 반복 |
| update_gradient | 0.303 | 마지막 내부 반복, factor 포함 |
| Cholesky factor/solve | 0.214 | 마지막 내부 반복 |
| line search | 0.080 | 마지막 내부 반복 |

400Hz 실시간은 전체 물리 스텝이 2.5ms 이내여야 한다. 이 대표 접촉 스텝은
solver만 약4.9ms이므로 화면 FPS나 CPU 스레드 수만 조정해 해결할 상태가 아니다.
이 값은 구간 전체 평균이 아니라 해당 스텝의 GPU 측정값이다.
headless 자원 관측10개에서 GPU54~100%, 메모리 최대4765MiB,
CPU 약99%(논리 코어 1개=100%), 프로세스 RSS 최대1.663GB였다.
GUI 포함 관측12개에서 GPU49~100%, 메모리 최대7511MiB, 프로세스 트리 RSS 최대10.579GB.
GPU 사용량/메모리는 장치 전체 표본이며 프로세스별 전용량이나 연속 측정 최댓값은 아니다.

## 시도했지만 채택하지 않은 것과 도구 오류

| 실행/시험 | 결과 | 채택 여부 |
|---|---|---|
| final_config_profile | 구형 미캡처 스텝 계측, 전체159.194초 | 진단만, 처리량 증거 아님 |
| captured_tails | 81.134초 /1.020배, 이전 기록 정확 일치 | 채택 |
| graph_events | 초기 profiler의 사용하지 않은 event 읽기 오류400 | 잘못된 profiler 증거로 보존 |
| indexed_graph_events | CUDA conditional body가 event record node를 거부, 물리 실행 전 실패 | 도구 수정, 물리 실패로 집계 안 함 |
| indexed_clean | 80.142초 /1.033배, 이전 기록 정확 일치 | 채택 |
| cached_aabb_profile | 82.014초, factor 재사용8312/51176회(16.24%) | 기본값 미채택 |
| integer_native_clean | 68.874초 /1.202배, 이전 기록 정확 일치 | 채택 |
| canonical_constraints | 53.294초 /1.554배, 이전 기록 정확 일치 | 채택 |
| N1 batched cuSOLVER microbench | 일반 .223ms / batched .814ms | 느려 기본값 미채택 |
| final_session 두 실행 | 53.052/61.267초, 전체 GUI 비교 PASS | 최종 성능 |
| final_graph_profile | 54.302초, profiler 오류 없음, 물리 완료 | 최종 병목 계측 |

Hessian cache는 uint32 전체 bit 비교가 같은 경우만 factor를 재사용하고 RHS는 매번 새로 푼다.
근사 비교로 물리 결과를 바꾸지 않았다. 실제 접촉에서는 행렬이 자주 바뀌었다.
cache/AABB/profile을 함께 사용한 시험이므로 위 시간에서 AABB 단독 효과를 추론하지 않는다.
N1 batched 시험은 385/400 SPD 행렬의 합성 시험이며 N개 수확 성능이 아니다.
과거 낮은 Hz/CG/일부 sparse factor의 유효성 실패는
[이전 보고서](S300_NEWTON_GPU_RESULTS_KO.md)에 보존되어 있다. 이를 기본값으로 되살리지 않았다.

## 준비 시간·검사

| 실행 | setup 전체초 | 모델 load초 | reset초 | graph capture초 |
|---|---:|---:|---:|---:|
| headless | 13.785 | 5.261 | .010 | 5.802 |
| GUI 반복 | 33.867 | 0 | .978 | 4.612 |

setup은 loop와 별도다. graph capture는 setup의 부분이며 JIT/module 준비를 포함할 수 있다.
GUI setup에는 화면 준비도 포함된다. JIT만의 독립 시간은 미측정, 미기록 physics warmup은0스텝이다.
반복은 retained model + complete reset을 사용한다. 결과 압축/영상 생성 시간도 loop와 별도다.

- host suite: 184개 중17skip, 실제167개 통과.
- sparse 색인 대조: 9개 중1skip, 실제8개 통과(CUDA 포함). 행 수80→3→0→37→80 변경 검증.
- constraint 행/포인터 permutation: CPU/CUDA2개 통과, graph 재현·멱등성 검증.
- bitwise Hessian cache CUDA1개 통과: RHS 변경/1ULP 변경/비양정 실패/graph.
- N1 batched CUDA1개 통과: 정확성·graph·반복 확인, 속도 이득은 없음.
- 최종 comparator/CLI guard26개 통과. `git diff --check` 통과.
- 실제 전체 실행 두 개와 이전 전체 실행의 상태·지표 대조 완료.

위 테스트 일부는 host suite와 중복되므로 합계로 독립 시험 개수를 부풀리지 않는다.

## 실행과 원자료

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh gpu --gui
```

화면 없이 같은 GPU 물리: `./nvidia-sim/run_s300_newton.sh gpu`

한 프로세스에서 완전 reset 후 GUI 대조:

```bash
./nvidia-sim/run_s300_newton.sh gpu \
  --repeats 2 --gui-repeat-index 1 --camera-test-motion
```

원자료/대시보드:
`/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/index.html`

최종 실행 `final_session/repeat_000`, `repeat_001`, 전체 대조 `final_gui_comparison.json`,
이전 결과 대조 `prior_to_final_comparison.json`, GPU 계측 `final_graph_profile/gpu_graph_event_profile.json`.
각 실행의 invocation.json에 실제 argv 및 그 프로세스 시작 시의 코드 SHA를 보존했다.

영상은 실제 저장된 상태를 1배속으로 재생한 검토 자료다. GUI 성능 측정 영상이 아니며
1배속 영상이 실시간 계산 능력을 증명하지 않는다. 작은 자막을 사용한다.

- 전체 농장 외형·전체 동작(가까운 잎이 동작 일부를 가림):
  [actual_gpu_full_1x.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/final_session/repeat_001/actual_gpu_full_1x.mp4)
- 접촉 탑뷰40~72초(검토용으로 배경 식물·잎만 숨김, 원본 물리/상태 불변):
  [actual_gpu_contact_1x.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/final_session/repeat_001/actual_gpu_contact_1x.mp4)

## 남은 목표와 다음 기술적 범위

접촉 중 0.33~0.50배속인 구간이 남아 **최종 실시간 목표와 N환경 수확 검증은 미완료**다.
단일 환경 조건을 통과하지 않아 이번에 N개 실제 수확 확장을 실행하지 않았다.
현재 canonical constraint 구현도 단일 환경에 한정되어 N>1은 명시적으로 거부한다.

다음 병목은 385DOF factor/solve를 반복하는 접촉 solver와 collision이다.
동일 수렴 조건을 유지한 GPU 커널 통합, 작은 행렬 전용 factor/solve,
또는 정확성 대조가 가능한 다른 선형대수 분해가 구체적인 후속 대상이다.
현재 측정으로 이 작업들의 성공을 보장하거나 불가능하다고 단정하지 않는다.
시뮬레이션 Hz·모델·수렴 허용치를 낮추는 방법은 이번 동일 조건 최적화와 별개이며,
그렇게 얻은 속도를 위 결과와 동등한 물리로 보고하면 안 된다.
