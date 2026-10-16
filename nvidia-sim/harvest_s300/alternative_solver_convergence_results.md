# Newton GPU 수렴 허용오차 추가 시험 (2026-10-17)

결론: 기존 GPU 수렴 허용오차를 완화하는 후보는 채택하지 않았다. 실제 GPU 기본값 약 `1e-6` → `1e-5` 변경으로 실행했으나, 열기 단계에서 분리된 08번 열매가 거터와 1 mm를 초과해 겹쳐 기존 판정에 따라 중단됐다. `1e-4` 후속 실행은 하지 않았다. 기존 proxy01 모델과 실행 기본값은 수정하지 않았다.

## 조건과 비교 범위

- 모델: `inputs_coarse4_fruit_proxy_01`; 동일 입력 파일·명령 파일·전체 명령 스케줄 SHA 확인.
- 기존 native XML tolerance는 `1e-8`이지만 설치된 MuJoCo Warp의 실제 업로드 값은 `9.999999974752427e-7`이다. 시험의 실제 값은 `9.999999747378752e-6`이다. GPU에서 약 10배 완화한 비교다.
- 64 solver iterations, 400 Hz 접촉/60 Hz 비접촉, 물성·충돌체·벨트·분리 규칙·1 mm 기준 유지. solver tolerance 외 실행 옵션을 변경하지 않았다.
- 같은 코드의 새 검증 `final_proxy01_validation/repeat_000`을 주 비교군으로 사용했다. 원래 proxy01 screen도 별도로 비교했다. invocation 해시 목록에서 `newton_fruit_ellipsoid.py` 파일 항목만 추가돼 있으며 해당 모델/실행에는 ellipsoid 옵션을 사용하지 않았다.
- 실제 실행 1회. 이전 proxy025 요청은 GPU lock 취득 전에 취소돼 물리 실행 0회였고 로그를 보존했다. 별도 수확 파이프라인을 만들지 않고 기존 `benchmark_solver_compare.py` 및 `run_s300_newton.sh gpu`를 사용했다.

## 결과

| 항목 | 기존 GPU 기본값 | tolerance 1e-5 |
|---|---:|---:|
| 실행 완료 | 13806/13806 steps | 7396/13806 steps, 중단 |
| 실행된 시뮬레이션 시간 | 82.795 s | 56.570 s |
| 실행 벽시계 시간 | 33.286 s | 17.692 s |
| 전체 완료 RTF | 2.487 | 해당 없음 |
| 관찰된 중단 전 구간 RTF | — | 3.197, 전체 성능으로 사용 금지 |
| active 최대 겹침 | 0.657 mm | 5.149 mm, FAIL |
| 분리 열매 수 | 6 | 7, 동일 결과 아님 |
| 실제 attachment 힘 최대 | 4.206 N | 3.516 N, 중단까지 |
| 전체 접촉력 최대 | 15.157 N | 14.506 N, 중단까지 |
| 접근 전 열매 최대 이동 | 0.551 mm | 0.517 mm |
| 접근 전 조기 분리 | 없음 | 없음 |

첫 위반: scheduled `56.555 s`, solver step `7390`, `cycle_open`. `Truss_00_Fruit_08` ↔ `farm_structural_17` (`/World/Greenhouse/House/Environment/Gutter_1_Base_2/Mesh`), 해당 스텝 겹침 `4.175 mm`. Fruit08은 `56.150 s`에 이미 분리돼 있었다. 기존 분리 열매 예외 정책에서 거터 접촉은 제외되므로 active 판정으로 중단된 것이다. 이를 피하려고 예외 정책이나 기준을 변경하지 않았다. 비유한 상태·제약 버퍼 오류는 없었다.

## 단계별 RTF

전체 단계를 명시한다. 중도 중단된 단계는 전체 단계 대비 속도 향상을 계산하지 않는다.

| 단계 | GPU 기본값 RTF | tolerance 1e-5 RTF | 동일 길이 단계 비교 |
|---|---:|---:|---|
| settle | 6.907 | 6.916 | 1.001× |
| rail_move | 7.758 | 7.822 | 1.008× |
| ready_hold | 7.781 | 7.846 | 1.008× |
| prepare_open | 7.902 | 7.963 | 1.008× |
| prepare_open_hold | 8.146 | 8.257 | 1.014× |
| prepare_lift_branch | 7.622 | 7.693 | 1.009× |
| prepare_joint_branch | 8.741 | 8.781 | 1.005× |
| approach | 1.386 | 1.391 | 1.004× |
| belt_start_open | 1.405 | 1.405 | 1.001× |
| close | 0.802 | 0.818 | 1.019× |
| cycle_open | 0.901 | 0.868 | 부분 실행 310/400 steps |
| cycle_open_hold | 1.229 | 미실행 | 미실행 |
| cycle_reclose | 1.057 | 미실행 | 미실행 |
| belt_finish | 0.617 | 미실행 | 미실행 |
| cycle_open_final | 1.029 | 미실행 | 미실행 |
| cycle_open_final_hold | 1.354 | 미실행 | 미실행 |
| withdraw | 1.365 | 미실행 | 미실행 |
| return_joint_branch | 8.676 | 미실행 | 미실행 |
| return_lift_branch | 7.535 | 미실행 | 미실행 |
| ready_close | 7.835 | 미실행 | 미실행 |
| hold | 7.694 | 미실행 | 미실행 |

닫기 단계는 `0.802 → 0.818 RTF`로 약 1.9% 개선됐을 뿐 여전히 실시간보다 느리다. 열기 단계는 310/400 steps에서 중단됐으므로 완료 단계와 직접 비교할 수 없다. 따라서 이번 변경은 접촉 구간 실시간 목표의 해법으로 인정하지 않는다.

## 분리와 힘

분리 시점은 scheduled release time이다. 힘은 실제 CONNECT 제약에서 읽은 attachment 힘의 norm이며, 접촉력 최대와 다른 측정량이다.

| 열매 | 기본 분리 시점 s | 시험 분리 시점 s | 기본 attachment 최대 N | 시험 최대 N (중단까지) |
|---|---:|---:|---:|---:|
| Fruit_01 | 미분리 | 미분리 | 1.272 | 2.162 |
| Fruit_02 | 54.4450 | 54.3875 | 1.512 | 1.482 |
| Fruit_03 | 53.5175 | 54.0025 | 2.219 | 3.516 |
| Fruit_04 | 54.0900 | 53.9875 | 4.206 | 1.763 |
| Fruit_05 | 53.3550 | 53.3525 | 2.133 | 2.088 |
| Fruit_07 | 53.9200 | 53.9825 | 1.537 | 1.384 |
| Fruit_08 | 54.1375 | 56.1500 | 1.631 | 2.316 |
| Fruit_09 | 미분리 | 56.3100 | 1.375 | 3.417 |
| Fruit_10 | 미분리 | 미분리 | 0.421 | 2.593 |
| Fruit_11 | 미분리 | 미분리 | 0.754 | 2.936 |

Fruit08의 분리가 약 2.013 s 늦어졌고, 기본값에서 분리되지 않던 Fruit09가 56.310 s에 분리됐다. 분리 수만으로 동등성을 주장할 수 없다. 접근 전 이동은 유사했으나 접촉 이후 궤적은 달라졌다.

## 로봇 추종 오차

관측 관절 위치 − 해당 시점 명령의 최대 절댓값이다. rail/lift/jaws는 m, arm은 rad. 시험은 중단까지의 기록으로 후반부 추종을 검증하지 못했다.

| DOF | 기본 최대 | 시험 최대 |
|---|---:|---:|
| root_joint | 0.00400008 | 0.00400008 |
| farmily_lift_height_joint | 0.00281216 | 0.00281216 |
| base | 0.01249488 | 0.01249484 |
| shoulder | 0.00800752 | 0.00796551 |
| elbow | 0.00412757 | 0.00411372 |
| wrist1 | 0.01100350 | 0.01098541 |
| wrist2 | 0.01726726 | 0.01726726 |
| wrist3 | 0.00016936 | 0.00015220 |
| jaw_L | 0.00442693 | 0.00446778 |
| jaw_R | 0.00526119 | 0.00519106 |

## 증거와 검증

- Git 요약: `nvidia-sim/harvest_s300/alternative_solver_convergence_evidence.json`.
- 실제 시험: `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/convergence_tolerance/proxy01_tol1e5_campaign/proxy01_tol1e5`.
- 새 기본값 비교군: `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/final_proxy01_validation/repeat_000`.
- 전체 원자료 비교: `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/convergence_tolerance/proxy01_tol1e5_vs_final_default_v2.json` 및 `proxy01_tol1e5_vs_screen_v2.json`.
- 분석기 `alternative_solver_convergence.py`는 저장된 기록만 읽는다. 중단 시험에 전체 RTF 향상을 부여하거나, 부분 단계의 RTF를 완료 단계와 동일 비교하는 오류를 막는 회귀시험 2개를 추가했다.
- `python -m unittest discover -s nvidia-sim/harvest_s300 -p "test_alternative_solver*.py" -v`: 8개 통과 (기존 fixture 시험 6개 + 새 보고 회귀시험 2개).
- `training_eligible=false`, `harvest_success=null`. 추가 물리 실행 및 새 영상 렌더링은 하지 않았다. 기본값 채택/비채택 판단은 이번 한 후보에 한정한다. 더 작은 tolerance 조정이나 다른 모델 전체가 불가능하다는 결론은 아니다.
