# S300 Newton GPU 실행 최종 검증 결과

**실제 GPU 물리와 GUI 유무의 전체 결과 일치는 확보했다. 실시간 성능 목표는 아직 미달성이다.**

최신 구성은 headless **0.986×**, 카메라를 움직인 GUI **0.864×**다. 전체13806스텝과13807개 상태·접촉·힘·분리 이벤트가 bit-exact이며, 두 실행 모두6개 열매 분리·활성 최대 관통0.794207mm로 완료했다. 접촉 구간은 아래 표와 같이1배속보다 느리다. CPU 실행으로 GPU 성과를 대신하지 않았고, 기존 동작을 바꿔 성공시키지 않았다.

브랜치: `codex/s300-newton-gpu-20261016` · 출발 커밋: `f1941f57bc23c74925274003f4ab4b9ef5fc3ad3` · 집계: 2026-10-16T01:52:50.659317+00:00

## 구성과 유지한 조건

`기존 Isaac 농장·로봇 화면 → Newton SolverMuJoCo → MuJoCo Warp CUDA → GPU 적분·접촉·제약 계산`이다. 별도 Isaac 프로세스는 측정 자세를 표시하며 physics_steps=0/timeline=0을 유지한다. GUI 두 번째 실행도 새 초기화 상태에서 실제 물리를 다시 수행했다. 영상 재생으로 비교를 대신하지 않았다.

- RTX3090Ti24GB/driver570.211.01, Threadripper3960X24코어48스레드/RAM약125.7GiB. IsaacSim6.1.0, Newton1.7.0, MuJoCo/MuJoCoWarp3.15.0, Warp1.17.0.
- 같은 prepared source와 초기 상태·로봇·리프트·기체·그리퍼·벨트 명령. 타깃 송이1개+자기 주줄기,132body/335joint/nv385/geom901/physicalmesh810/actuator10/CONNECT10. 원래 전체 농장 시각 배경과 기존 비타깃 식물 비충돌 정책 유지.
- 접촉400Hz/자유 이동60Hz, 예정82.795초/13806스텝. 활성 관통1mm, 열매 분리 force>3N 또는 torque>0.008Nm, 분리 열매 접촉 예외 유지. 물성·margin/gap·마찰·actuator·충돌체를 바꾸지 않았다.
- `training_eligible=false`, `harvest_success=null`. 분리 개수는 열매 attachment 해제 수이며 실제 수거/실물 수확 성공률이 아니다.

모델 MJB SHA: `2029e0e4cb5c9775b2138abadc6cd290045d89fefcd79b5065e5cedf2a5377c0`
runtime source model arrays SHA: `2ac335c86cb40b642ae287c7eb380a920c0debdd7e6763874bf3648bb9f636a9`
실행 명령 NPZ SHA: `d187adac43e87b05c8d1e6588bd094f9260451eaab2456f80d7984176350da0a`

입력·초기 상태·원본 파일·실행별 코드 SHA와 전체 비교 증거는 [validation JSON](validation/s300_newton_gpu_recovery.json)에 보존한다. 원본과 이전 결과는 덮어쓰지 않았다.

## 전체 실행과 GUI 검증

| 구성 | 실행 | loop초 | RTF | 분리 | 활성 최대 mm | 전체 GUI 비교 |
|---|---|---:|---:|---:|---:|---|
| dense_hessian | headless | 89.797 | 0.922 | 5 | 0.569432 | bit-exact PASS |
| dense_hessian | 카메라 이동 GUI | 103.000 | 0.804 | 5 | 0.569432 | bit-exact PASS |
| fixed_reductions | headless | 83.983 | 0.986 | 6 | 0.794207 | bit-exact PASS |
| fixed_reductions | 카메라 이동 GUI | 95.780 | 0.864 | 6 | 0.794207 | bit-exact PASS |

CPU 참고 비교(같은 prepared 입력·400/60Hz):

| backend | loop초 | 평균 RTF | 분리 | 활성 최대 mm | 해석 |
|---|---:|---:|---:|---:|---|
| Newton→native MuJoCo CPU | 61.994 | 1.336 | 6 | 0.680636 | 변경하지 않은 기존CPU 실행기 참고값·GPU 성과 아님 |
| Newton→MuJoCoWarp GPU 최신headless | 83.983 | 0.986 | 6 | 0.794207 | 실제GPU 물리 |
| Newton→MuJoCoWarp GPU 최신GUI | 95.780 | 0.864 | 6 | 0.794207 | 실제GPU 물리+독립화면 |

CPU는float64/tolerance1e-8, GPU는float32/업로드tolerance최소1e-6이다. 입력은 같지만 동일 수치 구현의 속도비나CPU/GPU 궤적 일치로 해석하지 않는다. 과거CPU GUI 검증과 이번GPU GUI 검증도 각각의 별도 증거다.

두 실제 전체 실행은 같은 retained model/process에서 complete reset 후 수행했다. 별도 프로세스·다른 GPU까지의 재현성은 미검증이다. 최신 실제GPU clock82.793121초와예정82.795초 차이는−1.878662ms다. 활성 식물/로봇 관통 검사를 통과했지만 분리 열매 최대 겹침10.234831mm와 최대접촉력18.44482N도 원자료로 보존한다. 전체geometry pair가1mm 이내이거나 수거함 접촉이 검증됐다는 뜻은 아니다.

두 구성 모두 각각의 headless/GUI 쌍 안에서 전체 결과가 정확히 같다. fixed-reduction은 float32 합산 순서를 바꾸므로 이전 dense-Hessian 구성의5개 분리와 최신6개 분리를 서로 동일 궤적으로 주장하지 않는다. qpos/qvel/pose/equality/접촉/힘 비교 허용오차는0이며 실제 측정 오차도0이다. timestamp는 explicit solver step과 저장된 비균일 timestep schedule로 전체 coverage를 확인했다. GPU float32 시계 누적 오차는 별도 진단으로 보존한다. 전체 trace에서 schedule 대비 최대 clock 오차는약11.0657ms이고 최종 차이는−1.878662ms다.

GUI 카메라 실제 이동 기록: `/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_recovery/repro_fixed_reductions_session400/repeat_001/live_camera_motion.json`. 실제 USD camera transform을 render 완료 후 측정했으며, 카메라 조작이 물리 스텝에 개입하지 않음을 전체 trace로 검사했다.

## 최신 구성의 구간 속도

| 구간 | headless RTF | GUI RTF |
|---|---:|---:|
| rail_move | 2.937 | 2.610 |
| prepare_lift_branch | 3.242 | 2.848 |
| prepare_joint_branch | 3.461 | 3.050 |
| approach | 0.464 | 0.415 |
| belt_start_open | 0.590 | 0.516 |
| close | 0.333 | 0.290 |
| cycle_open | 0.301 | 0.265 |
| cycle_reclose | 0.420 | 0.366 |
| belt_finish | 0.244 | 0.212 |
| cycle_open_final | 0.434 | 0.381 |
| withdraw | 0.599 | 0.521 |
| return_lift_branch | 3.617 | 3.151 |

빠른 이동으로 평균만 높아지는 경우를 성공으로 세지 않는다. 접촉·닫기·벨트 구간이 여전히 병목이므로 사용자 목표는 미달성이다. 성능 기준을 통과하지 못해 실제N환경 수확 확장은 시작하지 않았다. N4 GPU 선형대수·graph·실패 격리 검사는 실제N환경 수확 처리량의 증거가 아니다.

## 로딩·반복 실행·비교의 한계

| 실행 | setup총초 | 모델 load초 | reset초 | unrecorded physics warmup |
|---|---:|---:|---:|---:|
| repeat_000 | 15.234 | 5.259 | 0.0104 | 0 |
| repeat_001 | 34.258 | 0.000 | 0.9781 | 0 |

두 번째 실행은 모델을 재로딩하지 않았다. 실제 reset은 약0.98초였으며 항상0.01초라고 보장하지 않는다. setup은 초기화·JIT·graph capture 등을 포함하고 GUI setup에는 viewer 준비도 포함한다. **JIT만의 시간은 별도로 정밀 계측하지 못했다.** loop 시간은 GPU 완료/readback을 포함하며 비동기 제출 시간만 측정한 값이 아니다.

같은 prepared 입력의 CPU400/60 참고 결과는61.994loop초/RTF1.336이다. CPU는float64/tolerance1e-8이고 MuJoCoWarp3.15 GPU 업로드는float32에서 tolerance를최소1e-6으로 올린다. 이 차이를 명시하며 CPU/GPU bit-identical 물리나 GPU가 CPU보다 빠르다고 주장하지 않는다. 과거2송이739DOF 값과 이번1송이385DOF 값도 직접 엔진 속도 비교에 사용하지 않는다.

## 원인과 변경

- 현재 속도 cache와 제어 시점을 native constraint stage에 맞췄다. CONNECT에서 이전 cvel/cdof_dot를 사용하던 문제, 벨트 명령 적용 순서, 중복 Newton 메타데이터 복사/관측을 실제 재현과 stage 대조로 확인했다.
- 단일 CTA Newton Cholesky 병목을 GPU cuSOLVER32 POTRF/POTRS로 바꿨다. Hessian·gradient·line search·수렴 조건은 유지하며 특정 모델에만 opt-in 적용한다. 설치 라이브러리는 수정하지 않는다.
- tree 합산, contact 정렬, inertia factor/solve, constraint Hessian/reduction의 부동소수 합산 순서를 고정했다. packed dense-prefix와 solve_LD(...,x,x)의 RHS 보존도 검증했다. 최신 fixed reduction은 같은 수학식을 GPU에서 계산하며 물리 모델은 바꾸지 않는다.
- synthetic385/400 SPD factor microbenchmark에서는 native2.698ms/cu32 0.212ms/cu64 1.836ms였다. 이는 수확 RTF가 아니다. 과거 baseline_gpu400은 제어/cache 보정 전 코드이므로 최신 RTF와의 비율을 순수 최적화 배수로 주장하지 않는다.

실제385DOF qH의 random RHS residual은 native1.49e-6/cu32 5.67e-7, 실제 force RHS는 native1.82e-7/cu32 5.37e-8로 고정5e-5 기준을 통과했다. 반면 undamped M random-RHS stress는 condition약9.91e12에서 native0.742/cu32 0.176으로 미통과다. 기준을 완화하지 않았고 현재 DISCRETE qH 실행 경로의 검증과 구분한다. CSR mapping은 CPU mj_fullM과 정확히 일치했다.

Newton metadata builder의12개 inertia 보정 및 float32 cast 경고, MULTICCD capsule/cylinder·capsule/mesh의1contact 경고를 원자료로 보존한다. authoritative source MjModel 배열/hash와 실제 native 상태·qacc 유한성/용량 검사를 확인했다. PhysX와 동일 contact manifold 또는 실물 물성 보정을 검증한 것은 아니다.

## 테스트와 보존한 실패

최종 host suite147개 중13개skip으로134개를 실제 실행해 통과했다(7.322초). 별도 parser guard9개 통과, inertia6개 대조(2host+4실제CUDA)(385DOF qH/75DOF/혼합81DOF/alias/graph/비양정), cuSOLVER base/batch·cache stage·camera ACK·결정성 대조 로그도 보존한다. 최종 테스트 파일별 실행/skip/결과와 SHA는 JSON의 tests를 참조한다.

| 실행 | 상태/범위 | RTF | 분리 | 활성 mm | 중단·오류 |
|---|---|---:|---:|---:|---|
| baseline_gpu400 | completed·전체 | 0.519 | [6] | 0.703 |  |
| bounded_all_gui_prefix/repeat_000 | error·prefix/연결/미완료 | — | — | — | KeyboardInterrupt() |
| bounded_solver_prefix | completed·prefix/연결/미완료 | 0.003 | [0] | 0.222 |  |
| cg_gpu400 | stopped·prefix/연결/미완료 | 1.636 | [10] | 10.586 | active_contact_penetration_1mm |
| cusolver32_session400/repeat_000 | completed·전체 | 1.174 | [5] | 0.673 |  |
| cusolver32_session400/repeat_001 | completed·전체 | 1.064 | [6] | 0.696 |  |
| cusolver64_gpu400 | completed·전체 | 0.515 | [5] | 0.685 |  |
| deterministic_connection | error·prefix/연결/미완료 | — | — | — | ValueError('Array shapes must not exceed the maximum representable value of a signed 32-bit integer, got 10790 |
| deterministic_nxn_connection | error·prefix/연결/미완료 | — | — | — | RuntimeError('Deterministic counter buffer capacity exceeds the supported int32 limit (19637548315 > 214748364 |
| equilibrated_session400/repeat_000 | completed·전체 | 1.111 | [6] | 0.602 |  |
| equilibrated_session400/repeat_001 | completed·전체 | 0.986 | [6] | 0.695 |  |
| fused_dynamics_batch2_session400/repeat_000 | error·prefix/연결/미완료 | — | — | — | KeyboardInterrupt() |
| fused_dynamics_session400/repeat_000 | error·prefix/연결/미완료 | — | — | — | RuntimeError("Failed to allocate 77669900 bytes on device 'cuda:0'") |
| fused_nodet_session400/repeat_000 | stopped·prefix/연결/미완료 | 0.985 | [3] | 3.436 | active_contact_penetration_1mm |
| mathdx_connection | completed·prefix/연결/미완료 | 1.868 | [0] | 0.286 |  |
| newton128_fixed_gpu400 | completed·전체 | 0.400 | [6] | 0.880 |  |
| optimized_profile_gpu400 | completed·전체 | 0.517 | [7] | 0.701 |  |
| repro_algebra_prefix/repeat_000 | completed·prefix/연결/미완료 | 0.001 | [0] | 0.222 |  |
| repro_algebra_prefix/repeat_001 | completed·prefix/연결/미완료 | 0.130 | [0] | 0.222 |  |
| repro_algebra_session400/repeat_000 | error·prefix/연결/미완료 | — | — | — | KeyboardInterrupt() |
| repro_compact_session400/repeat_000 | error·prefix/연결/미완료 | — | — | — | KeyboardInterrupt() |
| repro_dense_hessian_session400/repeat_000 | completed·전체 | 0.922 | [5] | 0.569 |  |
| repro_dense_hessian_session400/repeat_001 | completed·전체 | 0.804 | [5] | 0.569 |  |
| repro_fixed_reductions_session400/repeat_000 | completed·전체 | 0.986 | [6] | 0.794 |  |
| repro_fixed_reductions_session400/repeat_001 | completed·전체 | 0.864 | [6] | 0.794 |  |
| s32_buffers_gpu150 | stopped·prefix/연결/미완료 | 3.049 | [0] | 1.214 | active_contact_penetration_1mm |
| s32_buffers_gpu200 | stopped·prefix/연결/미완료 | 2.672 | [1] | 1.255 | active_contact_penetration_1mm |
| s32_buffers_gpu400_profile | completed·전체 | 1.124 | [5] | 0.615 |  |
| s32_explicit_det_gui_prefix/repeat_000 | completed·prefix/연결/미완료 | 0.014 | [0] | 0.222 |  |
| s32_explicit_det_gui_prefix/repeat_001 | completed·prefix/연결/미완료 | 0.210 | [0] | 0.222 |  |
| s32_reset_repeat_gui_prefix/repeat_000 | completed·prefix/연결/미완료 | 0.014 | [0] | 0.222 |  |
| s32_reset_repeat_gui_prefix/repeat_001 | completed·prefix/연결/미완료 | 0.289 | [0] | 0.222 |  |
| s32_reset_repeat_prefix/repeat_000 | completed·prefix/연결/미완료 | 0.014 | [0] | 0.222 |  |
| s32_reset_repeat_prefix/repeat_001 | completed·prefix/연결/미완료 | 0.933 | [0] | 0.222 |  |
| s32_reset_repeat_prefix/repeat_002 | completed·prefix/연결/미완료 | 0.937 | [0] | 0.222 |  |

CG·낮은Hz·일부 factor 변경은 관통/유효성 실패로 채택하지 않았다. deterministic counter 초기화 오류는 자산/코드 연결 오류이며 수확 불가능으로 분류하지 않는다. prefix의 높은 RTF나 한 스텝 일치를 전체 완료 성능/재현성으로 승격하지 않았다.

## 실행

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh gpu --gui --camera-test-motion
```

`--gui`를 생략하면 화면 없이 실행한다. 같은 source·명령을 사용하며 CPU fallback은 없다. 신규 출력 폴더로 저장한다. wrapper는bash-n 및/bin/echo argument probe로 최종 검증 옵션·사용자GUI/출력 경로 전달을 확인했다. 실제 실행 옵션·코드 SHA는 각 invocation.json에 있다.

## 실제 저장 상태 영상

영상의1배속은 저장 상태의 재생 속도이며 측정된 physics RTF가 아니다. 최종 구성의 contact top 영상은 먼저 표시한다. 이전 dense-Hessian 두 영상은 해당 구성의 증거로 보존한다. contact top은133개 배경/잎 visual 경로만 숨겼으며 원본 geometry·충돌·저장 물리는 그대로다. full entry는 원래 잎에 의한 강한 가림을 유지한다. 모두 작은FontSize10 자막,960×720 H264/yuv420p이며 ffprobe frame count·blackdetect·중간 PNG 육안 검사를 통과했다.

- [/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_recovery/repro_fixed_reductions_session400/repeat_001/actual_gpu_fixed_contact_top_1x.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_recovery/repro_fixed_reductions_session400/repeat_001/actual_gpu_fixed_contact_top_1x.mp4)
- [/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_recovery/repro_dense_hessian_session400/repeat_001/actual_gpu_full_entry_1x.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_recovery/repro_dense_hessian_session400/repeat_001/actual_gpu_full_entry_1x.mp4)
- [/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_recovery/repro_dense_hessian_session400/repeat_001/actual_gpu_contact_top_1x.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_recovery/repro_dense_hessian_session400/repeat_001/actual_gpu_contact_top_1x.mp4)

MP4별 video/source-state SHA와 render/validation JSON을 함께 보존했다. 임시 PNG는 새/dev/shm 폴더에 생성하고 검증 후 그 새 폴더만 지웠으며, 영상별 대표 PNG3개와 원본 결과는 보존했다.

## 남은 과제

GUI와 물리 결과 차이는 이번 고정 조건에서 해결했다. 남은 핵심은 접촉 단계GPU 처리시간이다. 반복별 floating grouping을 다시 바꾸지 않으면서 constraint/Hessian·factor 호출 및 작은 GPU 작업의 dispatch 비용을 프로파일하고, 같은 전체 시퀀스로 속도와1mm 유효성을 함께 검증해야 한다. 현재 구현과RTX3090Ti에서 접촉 실시간은 미달성이며 모든 하드웨어에서 불가능하다는 결론은 아니다. 검증 전에는N환경 수확 처리량이나 목표 전체 달성을 선언하지 않는다. 새로운 수확 알고리즘/RL/물성 약화는 이번에 시작하지 않았다.
