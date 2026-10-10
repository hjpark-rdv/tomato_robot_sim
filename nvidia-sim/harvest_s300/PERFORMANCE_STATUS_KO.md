# S300 · RB5 · V5.1 Isaac 실행 성능과 다음 병목

2026-10-10 KST. 브랜치 `codex/s300-harvest-isaac-20261009`, 작업 시작 HEAD `b890fc6c39b752aa48236c7c1b6bc8ea15774c30`.
이 문서는 서버에서 보존한 실제 측정 결과를 정리한다. 코드·보고서의 정확한 최종 커밋 SHA는 이 브랜치의 Git 기록으로 확인한다.

**화면 갱신 때문에 물리가 멈추던 문제와 접촉 초기 상태 문제를 분리해서 해결했다. 원래 메시와 충돌체를 유지한 새 배치에서 정지·탄성 시험을 완료했고, 독립 CPU worker 24개는 합산 7.04sim-s/s를 기록했다. 각 환경은 목표 식물 한 개의 full 탄성과 주변 전체 충돌을 유지한다. GPU 병렬 성능 목표와 검증된 과실 포획·분리 수확은 아직 확보하지 못했다.**

최신 원자료와 모든 실패를 포함한 표:

- 서버: [index.html](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/index.html)
- 호스트에서 복사해 여는 주소: `file:///home/rdv/docker_share/mujoko_debugging_data/20261010_s300_performance/index.html`
- 수치·코드 해시: [performance_summary.json](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/performance_summary.json), [results.csv](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/results.csv).
- 이전 수확 연결 결과: [MOTION_STATUS_KO.md](/root/farmily_tomato/nvidia-sim/harvest_s300/MOTION_STATUS_KO.md).

## 1. 실제 농장 초기 상태를 확보했다

원래 `assembly_v5`는 활성 잎과 주변 식물이 이미 겹쳐 첫 스텝에서 중단됐다. 이 중단 시간으로 농장 처리 속도를 비교하지 않았다.
잎/과실/가지 충돌체를 지우거나 관통 기준을 높이는 대신 별도 source overlay를 만들었다.

변경은 일곱 placement 속성뿐이다. Plant_22의 Truss_01을 기존 GLB 회전 규칙 안에서 120°로 돌리고 부착점에 맞춰 이동했으며,
주변 Plant_01/02/03/21/23을 행 방향으로 각각 −0.2/−0.3/+0.4/−0.6/+0.6m 이동했다.
목표 `Truss_02_Fruit_02`와 목표 송이의 원래 세계 좌표는 그대로다. 이 배치는 원래 실패 배치와 별도 모델이다.

보존 검사 결과:

- 전체 source의 94,179 prim, 31,544 mesh, 31,540 CollisionAPI prim, 질량/관절/물리 관계와 로컬 정점·면 데이터 유지.
- 새 활성 식물은 기존 full 235바디/612자유도/30파단 연결을 유지한다. **활성 충돌체는 605개**다.
  1,266개는 CollisionAPI 전체 개수이며, 이 중 661개는 원래 비활성 시각 메시다.
- 235바디 질량 합 0.3945432849577628kg, 205개 명시 관성, 204개 spring의 드라이브/제한/파단/필터 유지.
  회전한 줄기 관성 텐서는 좌표계 변환으로 비교했다. 과실 30개 COM/관성은 기존대로 PhysX가 산출하므로 실측 보정 완료를 뜻하지 않는다.
- 회전한 송이의 메시 세계 위치 오차 최대 0.118μm, 변경하지 않은 메시 위치 오차 0.
  기존 빌더가 회전 과실의 바디 원점을 월드 AABB 중심으로 다시 잡는 차이는 별도 기록했다.
- 기존 USD/CAD/농장 모델은 덮어쓰지 않았다. 다른 고정 배경 식물끼리의 잎 겹침까지 모두 해소한 배치는 아니다.

증거: [배치 manifest](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_layout_v2/manifest.json),
[source 보존 검사](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_layout_v2/preservation.json),
[full 동역학 모델 비교](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_motion_assembly/comparison_to_v5.json).

새 `legal_motion_assembly/scene.usda` SHA는 `9220e5a2dddf8da4caf524fde7d0a4647769270061da3a2e9411e16a5308fe81`이다.
배경 충돌 비활성화 0개, 초기화 두 native 스텝 검사 통과, CPU 240Hz 정지 3초를 물리 루프 10.381초에 완료했다.
최대 과실 변위 3.694mm, 종료 변위 0.424mm, 최소 접촉 separation +0.1635mm, 파단 0이다.
초기 과도 응답을 포함한 값이며, 로봇이 수확 자세에 도달한 결과는 아니다.

## 2. 원형을 유지한 탄성 시험과 Hz 비교

같은 새 배치, full 식물, 모든 배경 충돌을 켜고 목표 과실에 world +X 0.1N을 1~2초 동안 가한 뒤 제거했다.
외력은 native rigid body force다. 로봇 접촉으로 이 힘을 만들어낸 수확 시험은 아니다.

| 설정 | sim 시간 | 물리 루프 wall | 결과 저장 | 앱 시작~종료 | 자체 첫 저장 상태 대비 최대 / 종료 변위 | 최소 separation |
|---|---:|---:|---:|---:|---:|---:|
| CPU 240Hz | 7초 | 24.211초 | 0.796초 | 50.373초 | 6.789 / 0.423mm | +0.1635mm |
| CPU 120Hz | 7초 | 11.658초 | 0.395초 | 36.766초 | 7.301 / 1.653mm | +0.1376mm |
| CPU 60Hz | 7초 | 6.527초 | 0.257초 | 32.550초 | 13.261 / 6.347mm | +0.0895mm |

세 실행 모두 초기화·완주·관통 기준 통과, 파단 0이다. 60Hz는 이 짧은 시험에서 실시간을 넘었지만,
**이 첫 비교는 시작 상태가 동일하지 않았다.** 두 native 초기화 스텝이 각각 8.33/16.67/33.33ms여서,
첫 저장 목표 위치는 원래 authored 중심에서 각각 0.420/1.616/6.004mm 이동해 있었다.
위 표의 종료 값은 각 실행의 첫 저장 위치 대비 거리다. 60Hz의 6.347mm를 원래 위치에서 남은 잔류 변형으로 해석하면 안 된다.
authored 세계 기준으로 보면 세 실행 모두 원래 목표 중심 가까이 돌아온다. 최대값에도 외력 적용 전 초기 과도 응답이 포함된다.

공통 실제 timestamp에서 248바디를 비교한 240→120/60Hz의 최대 세계 위치 차이는 5.421/18.179mm이며,
각자 첫 상태를 뺀 이동 차이는 3.814/16.161mm다. 세계 기준과 자체 초기 기준을 모두 보존했고,
서로 다른 Hz를 단일 tolerance로 PASS/FAIL시키지 않았다. 초기 중력을 중립화한 새 비교에서 하중 응답을 따로 확인한다.
이를 동일 물리의 속도 개선이나 모든 수확 접촉에 쓸 수 있는 기본 설정으로 채택하지 않았다.
현재 동등성 회귀 기준은 240Hz/PGS64/4/armature0.0005다.

비교 그림: [두 초기 기준을 분리한 목표 변위](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_hz_target_displacement.png).

원자료: [240Hz](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_cpu_pulse240/summary.json),
[120Hz](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_cpu_pulse120/summary.json),
[60Hz](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_cpu_pulse60/summary.json).
각 폴더에 실제 모든 바디 pose, 관절 q/명령, native 접촉, 초기화 검사, 코드 SHA를 저장했다.
앱/장면 준비와 종료 비용은 표의 전체 시간과 루프·저장을 구분해서 확인한다.

초기화 중에만 중력을 중립화하고 원래 중력/drive를 복구하는 진단을 추가했다.
과실을 이미 휜 상태에서 시작하지 않도록 한 초기 조건 대조이며, 기존 source 자산과 원래 실행 결과는 보존한다.
첫 목표 중심 편차는 PGS240/120/60과 TGS240에서 0.167/0.326/0.541/0.710μm다.

| 같은 초기 조건의 진단 | 7sim초 물리 루프 | 목표 최대 / 종료 변위 | PGS240 대비 전체 248바디 최대 세계 위치 차이 |
|---|---:|---:|---:|
| PGS240 · 64/4 | 22.742초 | 6.698 / 0.010mm | 기준 |
| PGS120 · 64/4 | 11.189초 | 7.018 / 0.010mm | 0.720mm |
| PGS60 · 64/4 | 5.637초 | 9.579 / 0.017mm | 5.347mm |
| TGS240 · 64/4 | 27.675초 | 7.667 / 0.305mm | 1.943mm |
| TGS60 · 64/4 | 6.726초 | 11.565 / 0.076mm | 8.034mm |

동일한 저장 timestamp만 맞추고 보간하지 않았다. 9개 native 로봇 q와 명령, 전체 바디의 회전도 비교했다.
native 식물 q 이력은 기존 NPZ에 없어 바디 pose 대조와 구분해 미보존으로 기록한다.
접촉 impulse는 원래 Ns 단위 그대로 시간 구간별 합을 보존했다. solver/Hz가 다르면 접촉점 수와 순간 힘이 달라지므로
접촉 bit-exact 대조로 취급하지 않는다. 서로 다른 Hz/solver의 결과에 universal PASS를 부여하지 않았다.
낮은 Hz는 이 외력 시험에서 빨라졌으나 실제 벨트/로봇 접촉까지 사용할 공통 preset으로 검증되지 않았다.

solver 반복을 16으로 낮춘 TGS60은 7sim초를 3.707초에 처리했지만 목표 최대 이동이 32.533mm였다.
PGS16/120Hz도 약 13mm까지 이동했다. 이 값은 속도만 개선된 대체 설정으로 채택하지 않는다.
동일 부하·시작 상태에서 변형 응답이 달라졌다는 근거와 원자료를 함께 보존한다.

그림: [정상 시작을 맞춘 Hz·solver 비교](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_neutral_hz_target_displacement.png).

## 3. 화면 문제는 표시 경로에서 고쳤다

원본 시각 메시에는 collisionEnabled=false인데도 PhysX collision/cooking API가 남아 있었다.
변형 mesh를 USD에 쓸 때 불필요한 PhysX 처리가 발생할 가능성을 줄이도록, 명시적으로 비활성인 표시 mesh의 dormant API만
**session layer에서** 제거한다. 바디/MassAPI가 있는 mesh는 거부한다. 실제 활성 충돌체·질량·관절·마찰·소스 파일은 수정하지 않는다.
676개 표시 mesh의 정확한 경로·제거 schema를 각 실행 `display_schema_cleanup.json`에 저장했다.

145개 skin mesh와 그리퍼 링크 표시를 한 USD ChangeBlock으로 일괄 반영했다.
skin의 원래 float32 계산과 full 메시 89,136정점은 유지하고, USD notice는 프레임당 148회에서 1회로 줄였다.
offline 피부 계산 자체는 약 23ms로 큰 차이가 없었다. 계산을 빨라졌다고 과장하지 않는다.

수정 후 GUI 시험은 3초 동작을 끝까지 완료했다. 같은 headless 실행과 비교한 248바디 × 721시점의 pose/q/명령과
9,928 native 접촉점의 timestamp/충돌체/분리거리/impulse가 **bit-exact**였다.
다만 GUI를 같은 프로세스에서 갱신하면 루프 8.939→13.865초로 표시 비용은 남았다.

표시를 별도 프로세스로 분리하는 `--publish-live`를 추가했다.
물리 프로세스는 완료된 실제 native pose의 최신 한 프레임만 고정 13,952byte mmap에 발행한다.
전체 물리 이력은 기존 states.npz로 계속 저장한다. viewer는 같은 full source 메시를 열고 측정 pose를 표시하며 **물리를 다시 실행하지 않는다.**
odd/even sequence로 쓰기 중인 프레임을 거부하고, source SHA와 env0 바디 매핑을 검사한다.

동일 코드 대조군 `cpu_pairs_control`과 publisher 실행의 248바디 × 721시점 및 9,926 native 접촉점은 **bit-exact**였다.
발행 함수 전체 97회 비용은 0.0165초였다. 실제 native viewer의 timeline 시간 0, physics step 0,
마지막 measured timestamp 3.0초를 PNG와 검증 JSON에 보존했다.
이 viewer 검증은 물리 완료 후 실행했다. **viewer와 GPU 물리를 동시에 실행하는 자원 경쟁은 아직 측정하지 않았다.**

증거: [GUI 대조](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/gui_headless_comparison.json),
[발행 대조](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/controlled_live_physics_comparison.json),
[실제 viewer PNG](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/live_display.png).
이전 코드/초기화가 달랐던 대조의 실패도 `cpu_trace_comparison.json`, `live_physics_comparison.json`에 그대로 보존했다.

물리 worker의 렌더러 시작도 분리했다. 설치된 Isaac 6.1의 공식 `test_headless_no_rendering.py` 방식을 재사용해
`SimulationApp(experience=None)`과 필요한 PhysX/USD/SimulationManager 확장만 켠다.
원래 full 장면과 native physics/tensor 실행기는 계속 사용한다. 기본 app 경로는 유지하고 `--physics-only`로만 선택한다.
실제 켜진 확장 73개에 RTX/viewport가 없음을 기록했다. GPU Foundation은 PhysX 의존성이므로 GPU 메모리 0이라고 보장하지 않는다.

첫 minimal app 실행은 SimulationManager의 scene cache가 비어 CLI CPU 선택만으로 실제 authored GPU scene이 바뀌지 않는 오류가 있었다.
그 결과 3sim초에 87.084초가 걸렸다. 새 코드는 실제 composed scene의 GPU dynamics/broadphase를 명시적으로 설정·검사하고,
public API로 실제 source PhysicsScene을 등록한다. 실제 scene의 timestep도 확인한다.
오류 실행은 원자료를 보존하고 CPU 속도 비교에서 제외한다. 표시된 device 이름만 보고 CPU 물리로 간주하지 않는다.

수정한 A/B에서 기존 app은 3sim초/10.622loop초, physics-only app은 9.832loop초다.
248바디와 native q/명령 및 11,370접촉점이 bit-exact였다. 최고 RSS는 약 6.80→3.76GiB(십진 7.30→4.04GB),
장치 메모리 표본은 약 2,039→570MiB로 줄었다. 같은 worker를 여러 독립 프로세스로 실행한 측정은 다음 절에 구분한다.

native 접촉 콜백도 실제 point 데이터 복사를 줄이고 후처리를 묶어 저장한다.
full 농장 PGS60 7sim초의 동일 조건 대조는 5.776→5.127loop초였고, 248바디×421시점과 6,468접촉점이 bit-exact였다.
이 비교는 콜백 최적화의 회귀 증거다. 60Hz를 다른 수확 동작에도 그대로 채택한다는 의미는 아니다.

증거: [worker 대조](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/physics_worker_comparison.json),
[native 접촉 버퍼 대조](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/contact_buffer_equivalence.json).

## 4. GPU와 병렬화를 실제로 시험한 결과

아래 CPU/GPU 대조는 원래 assembly_v5의 **분리 component fixture**다.
배경 충돌 28,971개와 초기 자체 겹침용 leaf filter를 제외한 진단이며, full 농장 수확 처리량으로 해석하지 않는다.
식물은 full 612자유도를 유지한다.

| 실행 | 3sim초 루프 | 결과·범위 |
|---|---:|---|
| CPU / exact pair filter | 10.235초 | 초기화·완주 통과 |
| CPU / 같은 exclusion을 collision group으로 표현 | 10.415초 | 대조군과 상태/접촉 bit-exact. 단일 측정에서 루프 개선 없음 |
| GPU / group filter / tensor contact | 87.949초 | 초기화·완주 통과. 이 구성에서 CPU보다 약 8.44배 느림 |
| GPU / zero-speed conveyor API 제외 진단 | 87.316초 | idle 대조 87.396초와 거의 같음. 단독 원인으로 보기 어려움 |
| GPU / target truss만 움직이는 진단 | 50.503초 | 612→234자유도. 동역학을 바꾼 결과이며 동등 최적화 아님 |

GPU device 사용률 최대 100%도 관측했지만 처리량은 낮았다. device-wide 사용률이 높다는 사실과 유용한 물리 처리량을 구분한다.
native Python profile에서는 simulate 구간이 대부분을 차지했다. 접촉 callback 시간이 simulate 안에 포함될 수 있으므로 둘을 합산하지 않는다.
tensor 측정은 actor-level normal-only force이며 friction과 개별 collider impulse를 포함한 native callback 측정과 같은 수치가 아니다.

정상 full 배치에서 CPU dynamics를 유지하고 broadphase만 바꾸는 대조도 했다.
같은 buffered PGS120/64/4·2threads의 7sim초는 CPU/MBP 10.647loop초,
CPU/SAP 10.650초, CPU dynamics+GPU broadphase 13.079초였다.
세 설정의 저장 바디 pose/q/명령/native 접촉은 bit-exact였으며, 새 단일 1/2thread 대조를 포함한 여섯 비교가 strict 기준을 통과했다.
SAP는 이 한 번의 시험에서 개선이 없고 GPU broadphase는 느려서 CPU/MBP 자동 선택을 유지한다.
GPU broadphase 사용을 GPU dynamics 사용으로 표시하지 않는다.
runtime 기록은 composed scene 속성으로 두 backend를 구분한다. compiled native selector를 별도 API로 조회한 증거는 아니다.

증거: [동역학과 broadphase를 구분한 대조](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/single_broadphase120_equivalence.json).

CPU thread 1/8/automatic, PGS position16, callback/tensor, GPU buffer/API, full/target_truss, cloning, collision group을 시험했다.
더 많은 thread를 지정하거나 solver 반복을 낮춘 값만으로 검증된 최적안을 선언하지 않았다.
PGS16 및 target_truss는 실제 변형이 달라져 물리 변화 진단으로 별도 표시한다.

collision-group 압축은 기존 155,461 exclusion pair 집합의 정확한 hash를 검사하고 CPU bit-exact 대조를 통과했다.
이전 복제 실험의 환경 간 접촉과 group 중복 경고, 자기 관통/초기 파단으로 0.0083초에 멈춘 시험을 모두 표에 남겼다.
이 과거 실행에는 실제 source physics scene 등록 및 composed native device 확인이 없었다.
따라서 과거 실패만으로 현재 CPU 복제의 정상 작동 여부를 단정하지 않고, 수정된 동일 full 배치에서 별도 대조를 했다.
최신 `legal_registered_replica1_cpu`는 실제 source scene의 CPU dynamics/MBP/120Hz 등록과 초기화 검사를 통과했다.
그러나 namespace를 복제한 1환경에서도 settle 0.016667초에 native 파단 18개,
목표 변위 48.913mm 및 최대 관통 2.65037mm로 중단됐다. 장치/cache 수정만으로 현재 복제의 제약 문제가 해결되지 않았다.
전체 원인과 GPU 복제의 정상 처리량은 아직 확인하지 못했으며, 이 0.0452wall초 중단을 속도 성과로 세지 않는다.
증거: [실제 CPU 등록 후 복제 대조](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_registered_replica1_cpu/runtime_settings.json),
[실패 상태·파단 기록](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_registered_replica1_cpu/summary.json).
별도 재검토에서 612개 preload·관절 순서, 초기 상태와 첫 스텝의 248바디 pose는 단일 환경과 bit-exact였다.
두 번째 스텝부터 벌어지며, authored joint frame·relationship remap 보존 검사에서는 차이를 찾지 못했다.
native standalone fixed-constraint frame/solver 상태는 조회하지 못했으므로 엔진 오류라고 확정하지 않는다.
증거: [초기 두 스텝 대조](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/registered_replica_initial_step_review.json).
**짧은 중단을 병렬 처리량으로 세지 않으며, 대규모 GPU 병렬 성능 목표는 미달이다.**

증거: [pair/group 대조](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/cpu_pair_group_comparison.json).
독립 CPU 프로세스는 각자 full USD와 native scene을 갖고 같은 정상 초기 상태에서 동시에 시작했다.
모든 배경 충돌을 켠 PGS120/64/4, authored armature, 초기 중력 중립화, native 접촉 버퍼 설정으로
각 worker가 7sim초의 0.1N 부하·복원 시험을 수행했다. 활성 Plant_22는 235바디/612자유도를 유지하며,
주변 식물은 원래 고정 장애물이다. 106송이 전체를 동시에 탄성화한 시험은 아니다.
120Hz와 240Hz 물리 응답의 차이는 앞 절의 진단 그대로이며,
병렬 처리량을 얻으려고 leaf/fruit collider나 DOF를 줄이지 않았다.

| 독립 workers | worker당 CPU threads | 합계 sim초 | 실제 동시 physics wall초 | 합산 sim-s/s | 동시 RSS 최대 GiB | 장치 GPU 메모리 MiB | physics CPU 중앙 / 최대 % |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | 2 | 7 | 10.647 | 0.657 | 3.655 | 538 | 공통 monotonic origin 미보존 |
| 2 | 8 | 14 | 10.857 | 1.289 | 7.287 | 873 | 825.7 / 840.2 |
| 4 | 4 | 28 | 12.022 | 2.329 | 14.557 | 1,544 | 1,576.3 / 1,611.4 |
| 8 | 2 | 56 | 9.536 | 5.873 | 29.066 | 2,885 | 2,910.7 / 2,940.7 |
| 16 | 1 | 112 | 17.062 | 6.564 | 58.284 | 5,569 | 3,885.3 / 3,916.0 |
| 24 | 1 | 168 | 23.855 | 7.042 | 87.365 | 8,254 | 4,343.3 / 4,397.9 |

여기서는 최초 worker의 loop 시작부터 마지막 worker의 종료까지 실제 monotonic timestamp로 나눴다.
개별 worker 처리량을 단순 더하거나 각자 가장 빠른 구간을 합치지 않았다.
동시 RSS는 같은 시각의 모든 child RSS 합이며 공유 라이브러리 페이지가 중복 집계될 수 있다.
CPU 합계 100%는 코어 하나 기준이다. GPU 값은 장치 전체이며 CPU PhysX worker의 foundation/context도 메모리를 사용한다.
모든 child의 초기화·완주·관통 지표와 실제 native CPU scene을 확인한 집단만 처리량을 인정한다.
별도로 54개 worker 모두를 단일 `legal_neutral_pulse120` 대조군과 비교했다.
248바디×841시점, 9개 native 로봇 q/명령, 12,902접촉점의 timestamp·충돌체·separation·impulse가 모두 bit-exact였다.
worker thread 수와 접촉 버퍼가 달라도 이 저장 부하 시험의 물리 결과는 달라지지 않았다.
공통 tolerance를 완화하지 않았다. 실제 조작·수확 경로의 병렬 동등성은 이 비교 범위에 포함되지 않는다.
앱/장면 준비와 NPZ·접촉 저장은 별도로 HTML 집단 표에 기록했다.
준비·저장을 포함한 전체 처리량은 2/4/8/16/24worker에서 각각 0.557/1.054/2.060/2.651/2.808sim-s/s다.
같은 buffer·PGS120 설정의 새 단일 대조군은 2threads에서 0.657sim-s/s,
준비·저장 포함 0.278sim-s/s였다. 24worker는 이 대조군 대비 loop 처리량 약 10.71배다.
단일 프로세스 resource 표본에는 elapsed 시간만 있어 loop CPU 중앙값은 추정하지 않았다.
24worker의 첫 부하·저장 완료까지는 59.821초이며, 이 중 앱/장면 준비 29.143초,
동시 물리 구간 23.855초, 종료·저장 6.821초다. 짧은 7초 시험에서는 시작 비용이 크다.
상주 worker를 재사용하는 후속 대량 실행의 처리량을 이 단일 배치 수치에서 보장하지 않는다.

8→16개의 추가 이득은 약 11.8%이며 메모리는 약 두 배다. 이 서버의 24코어/48스레드 자원을 더 많이 쓴다는 이유만으로
가장 큰 worker 수를 최적값으로 고정하지 않는다. 16→24개의 추가 이득은 약 7.3%이며 메모리는 약 50% 증가한다.
이는 전체 농장의 **동일 탄성 부하 시험 병렬 처리량**이다. 여러 로봇 수확 정책이나 모든 식물의 동시 탄성화·성공률을 검증한 결과는 아니다.

증거: 각 `poolN_neutral120_results/*_parallel_groups.json`과 실제 child `states.npz`/`contacts.json`/`summary.json`.
물리 대조: [54개 worker 상태·접촉 동등성](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/pool_neutral120_equivalence.json).
비교 그림: [worker 수에 따른 처리량·메모리](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/cpu_process_scaling.png).
보고서 생성기는 모든 중첩 worker와 실패도 읽고, 그룹 summary가 PASS여도 child가 invalid 또는 누락이면 집단 처리량을 표시하지 않는다.

## 5. 작은 실제 벨트 이송은 확인했지만 수확 성공은 아니다

CAD 최소/최대 개구 28~96mm와 원래 벨트 충돌·마찰을 유지했다.
30mm 합성 구를 벨트 사이에 둔 **별도 이송 fixture**는 초기화 통과, 5sim초/17.692wall초 완료,
벨트 방향 실제 이동 40.493mm, 최소 separation −0.1058mm였다.
관측한 구 접촉점 최대 힘은 6.779N이다. 한 점 impulse/dt 크기이며 합력·손상 한계가 아니다.

이후 정상 full 농장 충돌과 physics-only worker에 같은 합성 구 대조군을 연결했다.
PGS60은 약 0.667sim초에서 −0.517mm 관통으로 중단되어 기존 0.5mm 물리 기준을 넘었다.
PGS120은 5sim초를 7.534loop초에 완료했고 최소 separation −0.423mm,
TGS64/60은 5sim초/4.405loop초, −0.123mm로 완료했다.
TGS64/60의 부하 응답은 앞 절에서 PGS240과 달랐으므로 빠른 실용 후보이지 동등한 물리 최적화가 아니다.
단순 탄성 pulse가 통과했다고 동일 Hz의 벨트 접촉도 통과한다고 가정하지 않는다.

이 구는 농장 과실이 아니다. fixture의 별도 D6 guide와 gravity-disabled 구를 사용했다.
가짜 과실 attachment/grasp를 만들었다는 뜻은 없지만 자유 과실의 포획·꼭지 분리 실험도 아니다.
원래 40mm 구 시험은 초기화 t=0에 −1.773mm/65.2N 접촉이 있어 사후 무효로 분리했다.
원래 completed=true summary를 덮어쓰지 않고 성능 표에서 제외했다.

현재 농장 1,060개 과실의 raw mesh 최대 유클리드 직경은 27.235mm이며 원래 CAD의 명목 최소 개구는 28mm다.
이는 상대 크기를 검토할 참고치다. raw CAD의 위치별 벨트 모양과 native cooked convex collider/접촉 normal을
검증하지 않고 이 두 숫자만으로 양쪽 접촉이 불가능하다고 단정하지 않는다.
실제 목표 과실–벨트 접촉은 이미 기록됐다. 다만 앞쪽 nose/lip 접촉이며 과실이 중앙의 이송 위치까지 들어간 증거는 없다.
뒤의 실제 조작 시험과 접촉 좌표 분석을 통해 어디에서 밀리는지 구분했다.
송이 크기·CAD 개구·충돌체를 임의 수정하지 않았다.

증거: [30mm fixture](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/cpu_belt_loaded30/summary.json),
[초기 접촉·과실 치수 검사](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/farm_initial_contacts_and_fruit_size.json).
모든 자료는 `training_eligible=false`, `harvest_success=null`이며 fixture의 수확 결과는 false다.

## 6. 실제 로봇 접근도 실행했고 실패 원인을 보존했다

정상 시작의 같은 full 장면에서 기존 로봇 계획과 opt-in wide-open 접근을 실제로 실행했다.
wide-open은 준비와 삽입 중 jaw 명령을 44→60mm로 바꾸는 별도 모션이며 물성/충돌체를 바꾸지 않는다.
원래 경로와 명령은 보존했다. 아래 시간은 중단까지의 시간이며 완주 처리량 표에서 제외한다.

| 실제 경로 / 설정 | 마지막 sim초 | 물리 wall초 | 중단 접촉 | penetration | 목표 최대 변위 | native 파단 |
|---|---:|---:|---|---:|---:|---:|
| tight / PGS120 | 27.883 | 45.484 | 목표 Fruit_02 ↔ 자기 Rachis_4 | 0.50065mm | 40.559mm | 0 |
| tight / TGS60 · 64/4 | 28.767 | 28.800 | gripper outer cover part209 ↔ 활성 주줄기 leaf Appendage140_2 | 2.19883mm | 57.862mm | 0 |
| wide-open / PGS120 | 29.942 | 48.796 | L_belt ↔ 목표 Fruit_02 | 0.50218mm | 43.930mm | 0 |
| wide-open / PGS240 | 30.746 | 100.554 | L_belt ↔ 목표 Fruit_02 | 0.55796mm | 54.456mm | 목표 joint 1 |

네 경로 모두 접근 중 기존 0.5mm 물리 기준에서 중단됐다. close/belt 구간은 실행되지 않았다.
TGS60은 이 접근에서 거의 실시간으로 진행됐지만 관통 실패가 있어 공통 수확 preset으로 채택하지 않는다.
240Hz로 올려도 같은 경로의 모든 문제가 해결되지 않았다. 힘·밀림·충돌 위치와 접근 형상을 함께 확인해야 한다.

wide-open/240Hz의 native 목표 joint 파단은 30.741667초에 기록됐으며,
약 4.17ms 뒤 관통으로 중단됐다. 이는 실제 목표 분리 이벤트지만 **물리 유효성에 실패한 접근의 이벤트**다.
검증된 포획·유지·벨트 이송·수확 성공으로 집계하지 않는다.
중단 순간의 한 접촉점 force=0인 기록도 이전 순간에 힘이 없었다는 뜻으로 해석하지 않는다.

기존 계획을 30개 과실에 대입한 정적 sample 검사는 고정 장애물과 비목표 식물이 모두 없는 경로를 찾지 못했다.
단계당 9개 joint-space sample의 한정된 검사이며, native convex cooking·동역학과 일부 primitive/self-collision은
이 screen 범위 밖이다. 30개 과실 모두 수확 불가능하다는 결론이나 연속 경로 인증으로 사용하지 않는다.

접촉 좌표 분석에서는 원래 tight120 경로의 목표–벨트 접촉 519개를 확인했다.
raw CAD local bbox만으로 중앙 지지 폭을 단정하지 않았다. 좌우 벨트 nose의 Y 범위가 비대칭이며,
기록된 과실은 중앙의 목표 Y≈93mm 위치에 도달하지 못하고 앞쪽 nose 부근에서 밀렸다.
즉 다음 조작 과제는 단순히 Hz를 더 낮추는 것이 아니라, 실제 접촉 형상·좌표를 기준으로
안전한 준비와 중앙 진입을 연결하는 것이다. 이번 테스트의 성공한 부하·표시·병렬 결과와 이 접근 실패를 구분한다.

증거: [실제 접촉 좌표 분석](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_harvest_tight120_contact_geometry_analysis.json),
[30과실 정적 검사](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/offline_all_fruits_screen.json),
[wide-open 경유점 검사](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/offline_tight_open_screen.json).
각 `legal_harvest_*` 폴더에 실제 qpos/바디 pose/native 접촉/중단 시점을 저장했고, MP4는 저장 상태를 표시한다.
명목 계획을 재실행한 물리 성공 영상이 아니다.

실행 상태 영상:

- [벨트 대조 · 입구 정면 39프레임](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_belt30_neutral120/belt_mouth_actual_v1.mp4)
- [벨트 대조 · 기존 시점 39프레임](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_belt30_neutral120/belt_loaded_actual_v2.mp4)
- [원래 tight120 실제 접근 · 141프레임](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_harvest_tight120/actual_approach_v1.mp4)
- [wide-open240 실제 접근·중단 · 155프레임](/root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_harvest_tight_open240/actual_approach_open240_v1.mp4)

네 영상 모두 full ffmpeg decode와 저장 프레임 수를 확인했고 render metadata의 `physics_rerun=false`다.
실패한 첫 렌더 폴더/로그는 보존한다. 별도 session fixture의 xformOp가 root edit와 충돌하던 표시 오류를
session layer 작성으로 수정했고, 실제 렌더에서 재확인했다.

## 재현 및 표시

농장 전체 충돌을 유지한 짧은 탄성 검사를 새 output에 실행한다. 아래 명령은 수확 시험이 아니다.

```bash
cd /root/farmily_tomato
motion_out="/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_s300_full_pulse"
DISPLAY=:0 OMNI_KIT_ALLOW_ROOT=1 /root/isaac-sim-6.1.0/python.sh --no-ros-env -u \
  nvidia-sim/harvest_s300/run_motion.py \
  /root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_motion_assembly \
  --output "$motion_out" --mode pulse --seconds 7 --pulse-force-n 0.1 \
  --headless --physics-only --device cpu --hz 120 --threads 2 --performance optimized \
  --gravity-neutral-warmup --armature-setup authored --buffer-contact-events \
  --contact-backend callback --collision-filter pairs --plant-dynamics full \
  --solver PGS --position-iterations 64 --velocity-iterations 4 \
  --publish-live --live-fps 10
```

별도 터미널에서 같은 output을 연다. display FPS는 물리 Hz와 별개다.

```bash
cd /root/farmily_tomato
DISPLAY=:0 OMNI_KIT_ALLOW_ROOT=1 /root/isaac-sim-6.1.0/python.sh --no-ros-env -u \
  nvidia-sim/harvest_s300/view_motion_live.py "$motion_out" --fps 10
```

다른 터미널에서는 첫 터미널의 `motion_out` 값이 공유되지 않으므로 실제 출력 경로를 대입한다.
기존 완료 결과를 열어도 저장된 최신 실제 상태가 나타난다. 화면을 회전해도 물리를 다시 계산하지 않는다.
별도 viewer의 concurrent GPU 비용은 아직 비교하지 않았으므로 성능 측정은 viewer를 끄고 순차 실행한다.

위 120Hz 명령은 같은 부하 비교에서 실측한 CPU worker 설정이다. 240Hz가 기존 물리 회귀 기준이며,
120Hz를 모든 실제 조작에 충분한 정확도의 universal preset으로 선언하지 않는다.
같은 비교를 240Hz로 하려면 새 output에서 `--hz 240`으로 바꾼다.

검증한 8worker 부하 시험을 새 폴더에 반복하려면 저장한 cases를 기존 controller로 재사용한다.
각 worker가 다른 열매를 수확하는 명령이 아니라 독립 장면 8개의 동일 부하 대조다.

```bash
cd /root/farmily_tomato
pool_out="/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_s300_cpu_pool8"
/root/isaaclab_env/bin/python nvidia-sim/harvest_s300/benchmark_motion.py \
  /root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_motion_assembly \
  /root/docker_share/mujoko_debugging_data/20261010_s300_performance/pool8_neutral120_cases.json \
  --output "$pool_out" --jobs 8 --timeout 240
```

결과 표만 갱신하려면 물리를 실행하지 않는 기존 보고서 생성기를 쓴다.

```bash
./nvidia-sim/.asset-venv/bin/python nvidia-sim/harvest_s300/report_performance.py \
  /root/docker_share/mujoko_debugging_data/20261010_s300_performance
```

## 검증과 남은 일

최종 설치 SDK 묶음은 120개 테스트를 실행해 119개 통과, Torch 없는 interpreter의 GPU tensor test 1개 skip했다.
로그는 원자료 루트의 `offline_tests_final_complete_sdk.log`다.
표시 schema/정점/notice, 저장 상태·접촉 및 공통 시점, mmap/parent 축 변환/session fixture,
보고서의 초기화/device/중첩 병렬 집단, app bootstrap/device와 controller 검사를 포함한다.
별도 Torch GPU 실행은 이 offline skip으로 통과했다고 해석하지 않는다.
usd-core 환경의 PhysxSchema import 실패를 물리 실행 실패로 집계하지 않는다.

우선 full 장면의 정상 시작과 실제 cooked contact를 기준으로 준비 경로·중앙 진입을 개선한다.
8worker CPU 병렬 운영은 현재 측정에서 메모리 대비 효율이 좋으며, 16/24worker는 남는 메모리를 보고 선택한다.
상주 worker 재사용과 실제 조작의 동등성은 다음 검증이다. GPU cloning은 초기화/제약을 먼저 해결해야 한다.
모든 collider가 켜진 상태로 도달·포획을 검증하기 전에는 새로운 RL/대규모 수집을 시작하지 않는다.
이번 수치만으로 싱글 환경 실시간 수확과 대규모 GPU 자동수확 목표가 달성됐다고 선언하지 않는다.
