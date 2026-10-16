# Newton 대체 솔버 소규모 검증

2026-10-17. `codex/s300-newton-solver-compare-20261017`의 비교 작업 중 C/D 후보 검증이다.
전체 수확 실행의 결과는 상위 비교 보고서를 따른다. 아래 RTF는 **작은 식물 fixture**의 값이며 전체 농장/로봇 성능이 아니다.

## 범위와 입력

- 원본: `/root/docker_share/mujoko_debugging_data/20261016_s300_newton/inputs_render_separated`.
- 변경 구조: 같은 비교 실행의 `inputs_coarse4`.
- 결과: `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/alternatives`.
- 실행 환경: Newton 1.7.0 / Warp 1.17.0 / MuJoCo 3.15.0 / RTX 3090 Ti.
- 원본 식물 116 body, 활성 collider 89개, mesh 10개를 사용했다. 그림용 잎 mesh와 로봇/배경은 **단위 시험 범위에서만** 제외했다. 잎의 질량·관절은 입력 모델에 있는 대로 유지했다.
- `CONNECT` 10개는 공식 MJCF importer의 `convert_mjc_equality_constraints=True`로 loop BALL joint 10개에 대응시켰다. Native의 정규화된 CONNECT compliance와 generic BALL constraint가 수치적으로 같다고 주장하지 않는다.
- 전체 수확에 필요한 벨트·과실 각도 spring/deadband·3N/0.008Nm 분리 실행기는 이 fixture에 연결하지 않았다. `training_eligible=false`, `harvest_success=null`이다.

기본 시험은 400Hz, 1초이며 첫 0.25초 무입력, 다음 0.25초 Fruit_05에 0.05N 힘, 마지막 0.5초 회복이다. Native CPU에도 동일한 fixture/질량/초기상태/preload/시간표를 적용했다. Native CPU RTF는 물리 반응 비교용이며 GPU 가속 결과로 세지 않는다.

## 발견하고 수정한 연결 문제

1. 설치된 standalone Newton 환경에 `trimesh`가 없어 실제 mesh import가 처음 실패했다. 기존 asset 환경의 순수 Python `trimesh`만 임시 dependency path로 연결했다. 기존 Python 환경을 수정하지 않았다.
2. generic MJCF importer는 source hinge의 passive `stiffness`를 generic `joint_target_ke`로 옮기지 않았다. source 식물의 315개 spring/damping coefficient를 XPBD/VBD가 지원하는 drive로 정확히 대응시켰다. preload는 동일한 `-k*(q-q0)+preload`를 implicit target과 필요한 잔여 force로 분해했다. target은 기존 joint limit 안에 두고, 그 밖의 preload를 삭제하지 않았다.
3. generic finalizer는 `1e-10 kg m²` absolute floor 아래의 inertia에 `1e-6 kg m²`를 더했다. source 최소 principal inertia는 양수인 `8.053e-11 kg m²`로, 12개 body가 큰 영향을 받았다. `balance_inertia=False`만으로는 이 처리를 끌 수 없었다. source principal tensor의 양수·삼각 부등식을 확인한 뒤 **원래 tensor와 inverse를 복원**했다. source mass/inertia를 작게 조정한 것이 아니다.
4. 이미 고정된 coarse4 group에 공식 `collapse_fixed_joints=True`를 적용하여 116→31 body로 병합했다. collider 89개와 compound mass/inertia를 유지했다. source 파일은 변경하지 않았다.
5. VBD의 rest-angle cache는 model의 초기 `joint_q`와 `body_q`가 일관되어야 한다. 단일 관절 검증에서 constructor 전에 model에도 `eval_fk`를 적용해야 함을 확인했다. 불일치했던 초기 단위 결과는 제외했다.

원본/초기 generic import 실패, 제거된 API명(`Control.joint_target_pos`), inertia 변경이 남아 있던 실행, target 한 개만 고려한 specimen 위치의 초기 실행을 solver 성능/물리 한계의 증거로 사용하지 않는다. 원자료는 덮어쓰지 않고 결과 폴더에 남겼다.

## 물리 반응 비교

아래 VBD는 설치 버전의 **legacy AVBD 경로**다. modern compliant-ALM은 별도 항목으로 구분한다.

| 방법 | 식물 body | idle 최대 변위 | Fruit_05 최대/마지막 변위 | attachment 최대 gap | fixture RTF | 해석 |
|---|---:|---:|---:|---:|---:|---|
| Native CPU 원본 | 116 | 4.716mm | 11.005 / 4.461mm | 0.00110mm | 0.412 | 동일 fixture 물리 반응 기준 |
| Native CPU coarse4 | 116 | 4.757mm | 10.975 / 4.500mm | 0.00112mm | 3.338 | 변경 구조의 반응 기준 |
| XPBD16 원본, source inertia 복원 | 116 | NaN | 무효 | 무효 | 비교 제외 | 5ms에 nonfinite 발생 |
| VBD16 coarse4, source inertia 복원 | 116 | 126.357mm | 14541 / 4194mm | 21729mm | 비교 제외 | 접촉/연결 안정성 실패 |
| VBD16 coarse4, fixed group 병합 | 31 | 260.756mm | 249.120 / 112.702mm | 0.06373mm | 0.878 | finite/contact 유효성만으로 반응 일치를 주장할 수 없음 |
| VBD64 동일 병합 모델 | 31 | 65.785mm | 35.571 / 23.653mm | 0.20074mm | 0.241 | 반복 수 증가로 개선되지만 source 반응과 다르고 느림 |
| VBD16 modern compliant-ALM, 동일 병합 모델 | 31 | 236.749mm | 224.343 / 80.694mm | 0.01110mm | 0.769 | 권장 ALM 경로도 실제 실행했으나 source 반응/속도 우위 확보 안 됨 |

idle에 임의의 universal 5mm gate를 적용하지 않았다. Native 자체가 약 4.7mm 움직인다. 같은 초기조건의 A/B 반응을 비교했다. attachment gap 수십 m 및 nonfinite는 정상 물리로 해석하지 않는다. 접촉의 1mm 기준은 유지했다.

`coarse4_vbd16_collapsed_exact`와 `coarse4_vbd64_collapsed_exact`의 자동 summary는 `fixture_valid_response_comparison_pending_runtime_features`다. 이는 **source 반응 검증까지 통과했다는 의미가 아니다**. 상기 A/B 비교에서 수확 후보로 승격하지 않았다.

설치된 VBD의 기본값은 deprecated legacy 경로였다. 공식 예제에서 사용하는 `rigid_compliant_alm=True`도 별도 입력 `coarse4_vbd_alm16_collapsed`에서 실행했다. ALM에서 attachment gap은 더 작아졌지만 source idle/push 반응 차이는 남았고 fixture 처리도 느려서 전체 수확으로 승격하지 않았다. 따라서 위 결론은 legacy 기본값만 시험하고 모든 VBD 기능을 배제한 결과가 아니다.

## 실제 관절 재료의 작은 대조 시험

실제 `Truss_00_Pedicel_proximal_05_0`의 질량·inertia·3축 spring 0.25Nm/rad·damping 0.02를 이용했다. 중력을 없앤 단일 관절 시험에서 0.1rad를 놓고 400Hz, 0.4초 회복을 비교했다.

| 방법 | 0.4초 x 회전 | 전체 x 회전 오차 최대 |
|---|---:|---:|
| Native | 0.0006221rad | 기준 |
| XPBD16 | 0.0012547rad | 0.0052933rad |
| VBD16 legacy | 0.0007274rad | 0.0011499rad |

CPU/GPU 양쪽에서 같은 경향을 확인했다. 따라서 generic drive가 전혀 작동하지 않는 단순 연결 오류와, 큰 plant tree의 수렴/반응 문제를 구분할 수 있다. 이 작은 대조 시험을 전체 식물 정확도나 수확 성공으로 확대 해석하지 않는다.

## 공식 양방향 결합 솔버의 실제 접촉 시험

공식 experimental `SolverCoupledADMM`에 MuJoCo Warp가 담당하는 실제 inner-belt box specimen과 VBD가 담당하는 coarse4 식물을 연결했다. specimen은 source belt-body 질량/inertia를 유지한 동적 slide body다. 전체 그리퍼/로봇을 대신하는 성능 결과는 아니다.

초기 위치는 target radius만 고려하지 않고, specimen의 YZ footprint와 겹치는 **모든 활성 식물 collider의 AABB 앞에서 5mm 양의 여유**를 두었다. 0.25초 후에 밀던 최초 valid-start 시험에서는 식물이 먼저 휘어 probe 접촉이 없었으므로 결합 접촉 증거로 세지 않았다.

0.2초 시험에서 1N을 0~0.1초 동안 specimen에 적용했다. source 물성·질량을 바꾸지 않았고, 동일 힘/시간의 negative control은 specimen만 Y방향 0.4m 이동하여 비접촉 상태로 두었다.

| 항목 | 실제 접촉 | 비접촉 대조 |
|---|---:|---:|
| probe 접촉 physics step | 56 / 80 | 0 / 80 |
| probe 마지막 x 이동 | 49.249mm | 97.113mm |
| probe 마지막 x 속도 | 0.12196m/s | 0.64742m/s |
| Fruit_05 마지막 이동 | 64.494mm | 84.610mm |
| 최대 pre-step penetration | 1.02258mm | 0.32281mm |
| fixture RTF | 0.335 | 0.345 |

식물 움직임과 동적 specimen의 이동/속도가 모두 달라져, 일방향 kinematic 영향만 있는 구성이 아님을 확인했다. 그러나 실제 접촉의 1.02258mm는 현재 1mm 기준을 넘는다. 기준을 늘려 통과시키지 않았다. 원래 식물의 반응과도 다르므로 전체 수확 실행으로 승격하지 않는다.

거리/침투는 `state_in`의 collision generation 시점, 저장한 body pose는 integration 이후다. 이 두 계산 시점을 혼동하지 않는다. 이 fixture는 force observables를 수확 성공 판정으로 사용하지 않는다.

## 전체 수확 연결에서 남는 기능

- XPBD/VBD 모두 `rigid_contact_surface_velocity`를 받아 실제 벨트의 접선 속도를 적용할 수 있다.
- XPBD의 `BODY_PARENT_F`는 공식 설명상 유한 반복/relaxation에 따른 근사 reaction이다. 기존 3N 분리와 대응하려면 별도 load 검증이 필요하다.
- VBD의 공개 observable은 `CONTACT_F`이며 `body_parent_f`는 구현되어 있지 않다. 내부 ALM joint multiplier를 통해 reaction adapter를 만들 수 있는지는 추가 검증 사항이다. **접촉력으로 CONNECT 힘을 대체해 분리시키지 않았다.**
- source fruit spring은 local_rot0/1, anisotropic `[0.01,0.03,0.03] Nm/rad`, 1° deadband, damping 0.00015다. 이를 작은 inertia의 부모 body에 explicit torque로만 넣으면 안정성이 다시 달라질 수 있다. 원래 joint-frame을 갖는 implicit attachment 표현이 필요하다.
- isolated plant의 armature와 frictionloss는 실제로 모두 0이다. unsupported armature가 이번 plant fixture에 추가 차이를 만들지는 않았다. 전체 로봇의 actuator/armature/effort-limit 대응은 이 시험으로 검증하지 않았다.

현재 결과로 이 대체 솔버들을 전체 수확 GPU 실행의 우승자로 선택할 근거는 없다. 이는 Newton 전체가 불가능하다는 결론이 아니다. 현 식물의 재료/표현·반응·분리 semantics를 맞추는 작업이 추가로 필요하고, 현재 검증된 동일 계열 전체 실행과 비교해 유리함을 확보하지 못했다.

## 재현과 테스트

```bash
cd /root/farmily_tomato
PYTHONPATH=/tmp/s300_alt_solver_deps flock -x -w 900 /tmp/s300_solver_compare_gpu.lock \
  nvidia-sim/.newton-venv/bin/python nvidia-sim/harvest_s300/alternative_solver_probe.py \
  --inputs /root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/inputs_coarse4 \
  --output /root/docker_share/mujoko_debugging_data/NEW_ALTERNATIVE_FIXTURE \
  --solver vbd --iterations 16 --collapse-fixed --vbd-mode legacy
```

modern 경로를 재현할 때는 `--vbd-mode compliant-alm`을 사용한다. 두 경로를 같은 결과로 섞지 않는다.

`/tmp/s300_alt_solver_deps/trimesh`는 기존 asset 환경의 순수 Python package를 가리키는 dependency 전용 symlink다. 다른 환경의 전체 site-packages를 섞지 않았다. source 환경을 설치/업그레이드하지 않았다.

자동 테스트 6개: 실제 collider/attachment 보존과 원본 불변, 동적 specimen 범위, preload algebra 및 limit 밖 잔여 force 보존, 작은 유효 inertia 보존, 잘못된 inertia 복원 거부.

```bash
nvidia-sim/.newton-venv/bin/python -m unittest discover \
  -s nvidia-sim/harvest_s300 -p test_alternative_solver_probe.py -v
```

각 실행 폴더에 `plant_fixture.xml`, `summary.json`, 실제 모든 physics step의 `actual_states.npz`를 남겼다. source/input/code SHA는 summary에 있다. 큰 상태 파일은 Git에 추가하지 않는다. 이 문서와 작은 evidence JSON은 상위 작업의 결과 커밋에 포함한다.
