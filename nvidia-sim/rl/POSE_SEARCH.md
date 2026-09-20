# Tomato_05 고리 접근 자세 물리 탐색

## 구현 전 구조 조사

- 장면: `scenes/farmily_greenhouse_robot.usd`; 식물 원본:
  `env_usd/tomato_stem_v8_with_rotated90_cluster_harvestable_FIXED.usd`.
  현재 spawn `(-0.75, 0.55, 0.32)` m, scale 0.5, yaw 0을 유지한다.
- 목표: `/World/envs/env_0/HarvestableStem/Harvestables/Tomato_05`.
  `greenhouse_env.py`는 해당 rigid body에 `FruitCollider`와 `PedicelCollider`를 둔다.
  후자는 과육 표면에서 authored attachment까지 이어지는 반경 1.5 mm 캡슐이다.
  시각 메시 자체가 접촉 형상인 것은 아니다.
- 탄성 꼭지: `ElasticPlant/TRUSS_Pedicel_proximal_05_00..02/StemCollider`.
  마지막 세그먼트는 `Attachment_04`와 연결되고, 그 attachment와 Tomato_05 사이의
  분리 가능한 fixed joint는 reset마다 `HarvestJoint_04_<generation>`으로 다시 생성된다.
  원본 파손 임계값과 탄성 계수는 변경하지 않는다.
- 고리: `/World/envs/env_0/Robot/link6/tcp/tomato_gripper/RingCollision/segment_00..31`.
  CAD 기준 열린 반원이며 반경 27.5 mm, wire 반경 1 mm다. 도구 좌표 X 음수 쪽의
  반원(90~270°)이고, 막힌 뒤쪽은 180°다. 중심은 `geometry.RING_CENTER`다.
  이번 검증에서 의도한 접촉 영역은 뒤쪽 135~225°(segment 08..23)로 명시한다.
- 기존 경로: `configs/tomato01_engage.json`을 Tomato_05에 적용했던
  `hook_motion.py` / `contact_planner.motion_waypoints`의
  preapproach → below → insert → rise → pull을 nominal로 사용한다.
  yaw 0°, below 20 mm, lateral 13.5 mm, insertion 4 mm, neck height +3 mm.
  이는 Tomato_05 성공 경로라는 뜻이 아니다.
- Reset: `GreenhouseHarvestEnv._reset_idx`에서 robot PICK_READY, lift 초기 높이,
  과실 pose/velocity, 탄성 articulation의 joint position/velocity와 preload,
  파손 joint와 event flags를 복원한다. 새 runner는 후보마다 **새 프로세스**로 장면을
  만들고 reset하여 PhysX warm-start/contact cache까지 후보 간 격리한다.
- 접촉 기록: `_on_contacts`의 native collider 경로·접촉점·impulse/dt를 사용한다.
  기존 callback은 robot self-contact를 기록하지 않으므로 자기 충돌은 별도 사전 검사한다.
- 기존 성공 판정은 `inserted/hooked/contact_stopped`와 최종 pedicel gap을 조합한다.
  이번 runner는 접촉 대상/고리 영역을 명시하고 유지 중 기하/접촉을 기록하는 별도 판정을 사용한다.

RGB-D는 동일한 고정 관찰 카메라에서 실행 직전에 저장한다. 후보 생성·판정은 GT를
사용하며 영상 inference, RL, 식물/물성 randomization은 사용하지 않는다.

## 실행

```bash
# 기존 Isaac 환경에 추가되는 자기 충돌 검사 의존성
/root/isaaclab_env/bin/python -m pip install -r nvidia-sim/rl/pose_requirements.txt

# 전체 실험: coarse 31개 + 성공 상위 2개의 tolerance 각 36개
./nvidia-sim/run_pose_search.sh
# 먼저 2개만 확인
./nvidia-sim/run_pose_search.sh --limit 2
# 동일 코드/설정에서 중단된 실행 재개
./nvidia-sim/run_pose_search.sh --run-dir ABSOLUTE_RUN_DIR --resume

# GPU 접근 불가 시 CPU 물리 검증만. RGB-D 데이터셋 완성으로 취급하지 않는다.
./nvidia-sim/run_pose_search.sh --physics-only --workers 3
```

기본 출력은 `runs/YYYYmmdd_HHMMSS_tomato05_pose_search/`이다. 정상 RGB-D 실험은
기존과 같이 NVIDIA 렌더링 장치가 필요하다. 기본 worker 수는 1이며, `--workers`로
최대 4개의 독립 프로세스(CPU-only 검증은 최대 8개)를 사용할 수 있다. 식물·로봇을 다른 위치에 복제하지 않는다.
각 프로세스가 원본 장면을 독립적으로 열고 완전히 같은 배치를 사용한다.

`--physics-only`에서는 이미지 경로가 `null`, `dataset_complete=false`이며
누락 요구사항을 summary에 남긴다. `candidate_execution_complete`와 `experiment_complete`도 구분한다.
GPU/프로세스 오류는 IK 실패나 miss로 변환하지 않는다.
그런 오류에서는 배치를 실패로 중단하고 worker 로그를 보존한다.

## 후보와 실행 프로토콜

고정된 nominal 주변 one-factor coarse 31개:

- 방위각: 0 및 ±15°, ±30°, ±45°, ±60°. 넓은 방위각도 IK/자기 충돌 검사에 통과해야 실행한다.
- elevation, hook roll, hook pitch: 각각 ±15°.
- 월드 XYZ offset: 각 축 ±10 mm, ±20 mm.
- pre-hook distance: nominal 166 mm, 변형 140/200 mm. 이 값은 preapproach에서 below까지의 거리다.
  기존 nominal의 과실 중심 기준 170 mm 위치와 insertion 4 mm 차이를 반영했다.
- 방위각 ±15° + elevation 10° + roll ±10°의 결합 후보 2개.

방위각/elevation은 목표 중심 주위에서 동작 골격과 접근 방향을 회전한다. Roll/pitch는
고리 자세를 바꾼다. XYZ는 전체 동작 골격을 평행 이동한다. Quaternion 순서는 **xyzw**,
위치 단위는 **m**, pose 원점은 **CAD 고리 중심**이다. 모든 waypoint도 metadata에 저장한다.

IK는 준비 자세에서 연속적인 해를 따라 풀고, 전 경로의 joint limit/명령 속도를 검사한다.
USD의 convex-hull mesh/cube/capsule을 FCL로 검사하며 샘플 간 최대 간격은
팔 0.015 rad / lift 1 mm다. 연속 충돌의 수학적 보증은 아니다.
기존 SRDF의 정확한 link pair, 같은 fixed body, USD joint의 `collisionEnabled=false`인
연결 body pair를 제외한다. 초기 PICK_READY의 근위 그리퍼 hull과 link5가 기하학적으로
겹치지만 이 pair는 기존 USD의 인접 joint collision filter에 포함된다.
SRDF의 `Never` 제외를 부착 도구 전체로 확대 적용하지 않는다. 형상/FK와 초기 USD
transform 일치 오차, 적용한 native joint filter 목록도 audit에 기록한다.

사전 검사 통과 후 preapproach → below → insert → rise → pull을 물리 drive로 실행한다.
접근 속도 상한은 35 mm/s, rise/pull은 2 mm/s이며 smoothing을 적용한다.
기존 방식처럼 pull 구간과 걸림 후에는 낮은 robot drive gain을 사용한다.
이 gain 일정은 모든 후보에 동일하며 **식물의 탄성/감쇠/마찰은 바꾸지 않는다.**

의도한 목표 접촉이 고리 안쪽에서 확인되면 physics substep에서 현재 관절 위치를
목표로 유지하고 1초 관찰한다. 즉, 이 실험은 기존 방식의 **GT 접촉 기반 정지 기능을
포함한 접근 프로토콜**의 가능성을 조사한다. RGB-D 기반 정책이나 실제 로봇에 그대로
배포할 수 있는 감지 제어기를 학습/검증하는 것이 아니다. 위험 접촉, 큰 변위 또는
파손 시에는 조기 중단하며 중단 이유와 실행된 단계 수를 남긴다.

## 성공 판정과 event

`success_target_hook`에는 모두 필요하다.

1. 과실이 고리 aperture에 삽입된 이력이 있다(기존 `_geometry` 조건).
2. 기존 lip 위치 `[-R + wire + 1.5 mm, 0, 0]` 주변인 뒤쪽 segment 08..23이
   Tomato_05의 distal proxy 또는 마지막 proximal capsule에 실제로 접촉한다.
3. **그 접촉 대상 capsule**이 고리 평면 근처에서 고리 안쪽에 위치한다.
   바깥쪽에서 고리를 누르는 접촉은 제외한다. rear arc surface gap은 1.5 mm 이하다.
4. 1초 유지 전체에서 안쪽 geometry를 유지하고, 유지 중 최소 3개 control tick에
   의도한 native contact가 기록된다. 마지막 0.5초의 고리 기준 attachment 움직임은 2 mm 이하다.
5. 목표/비목표 열매 파손, 비목표 최초 접촉, 위험 접촉, 과도한 변위가 없다.

이는 현재 충돌 proxy에서의 보수적인 **안정 걸림 operational definition**이다.
한 번 스치기, 기하학적 근접만 있는 상태, 분리된 열매는 성공이 아니다. 실제 꼭지의
재료 강도나 수확 안정성을 보증하지 않는다.

기본 허용 변위는 target 20 mm / 주줄기 세그먼트 최대 30 mm다. 비목표 접촉 위험
기준은 3 N이며, 원래 contact logger의 0.01 N 초과 접촉이 최초 접촉 판정에 사용된다.
여기서 비목표 접촉은 **의도한 target pedicel family 이외의 collider 접촉**을 뜻하므로
목표 과육이나 온실에 닿는 것도 포함한다. 한 후보에 여러 event가 기록된다.

Primary label 우선순위는 planning failure → excessive displacement → non-target
contact → success → miss다. 따라서 primary count는 합계가 total과 같고, event count는
서로 겹칠 수 있다. `miss` event는 모든 물리 실행 실패에 함께 남긴다.
원시 collider 경로, contact position, force, break event는 `contacts.json`에 저장한다.

## Reset 검증

각 새 프로세스에서 두 번 reset하고 robot q/dq/root, 모든 과실 root state,
탄성 joint q/dq와 전체 body state, 고정 preload, 식물 stiffness/damping을 비교한다.
그 다음 audit 프로세스의 `initial_state.npz`와 최대 절대 오차 1e-6 이내인지 확인한다.
RGB-D 촬영 후에도 물리 상태가 바뀌지 않았는지 검사한다. 하나라도 다르면 실험 오류로 중단한다.
후보 실행 간 stage/solver/contact warm-start cache를 공유하지 않는다.

## 데이터 구조

```text
RUN/
  experiment.json           # sampling, thresholds, source hashes, no randomization
  audit/
    structure.json          # live target/꼭지/고리/joint 경로와 물리 설정
    self_collision_model.json
    initial_state.npz
    reset_check.json
    observation.json
    rgb.png / depth.npy / depth_valid.png
  dataset/scene_0001/coarse_0000/
    candidate.json          # pose, direction, q_start, label, contacts, displacements
    rgb.png / depth.npy / depth_valid.png
    observation.json        # intrinsics, optical extrinsics, units, file hashes
    initial_state.npz / reset_check.json / structure.json
    planned_commands.npy / planned_phases.json
    trace.json / contacts.json
  metadata.jsonl
  results.csv
  summary.json
  inputs/ / logs/
```

카메라는 초기 Tomato_05 중심에서 월드 `(0.28,-0.28,0.16)` m 떨어진 고정 위치에 둔다.
모든 후보에 동일한 초기 중심/관찰 위치를 사용한다. Depth는 optical Z 방향 거리(m),
`distance_to_image_plane`이며 range가 아니다. 유효 depth mask와 카메라 보정값을 저장한다.
렌더링 노이즈 때문에 픽셀 단위 동일성을 가정하지 않고 후보별 RGB-D를 저장한다.

## Tolerance와 통계

Coarse가 끝난 뒤 실제 성공 후보 중 목표 변위가 작은 상위 2개에 대해 자동 실행한다.
위치 ±2/5/10 mm × XYZ, 자세 ±2/5/10° × 도구 local XYZ의 systematic perturbation으로
후보당 36개다. 자세 perturbation은 위치 waypoint를 바꾸지 않는다. 각 magnitude는
6개 표본의 성공률이며 95% 등 임의 통계로 보간하지 않는다. Planning reject도 분모에 포함한다.

성공이 없으면 성공 범위는 `null`, tolerance는 `not_run_no_successful_coarse_candidate`다.
성공 범위는 관찰된 파라미터 최솟값/최댓값일 뿐, 그 사이 모든 pose가 성공한다는 뜻이 아니다.
summary는 전체/valid 성공률, primary/event counts, 실제 성공 후보, tolerance 표본 수와
성공률을 구분해서 출력한다.

완료된 실행은 아래 명령으로 원시 기록과 reset/source 일치를 검증하고 표·그래프를 만든다.
걸림 자체를 유지한 후보(`retained_hook`)와 모든 조건을 통과한 성공(`hook_success`)은 별도로 집계한다.
변위 최대값은 60 Hz control tick 표본 기준이다. 접촉 기록은 physics substep에서 수집한다.

```bash
/root/isaaclab_env/bin/python nvidia-sim/rl/pose_report.py RUN
```

## 검증

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /root/isaaclab_env/bin/python -m pytest -q nvidia-sim/rl
```

Nominal 골격 동일성, 후보 중복/유효 회전, tolerance 위치/자세 분리, 고리 바깥쪽 접촉 제외,
실패 event가 걸림 성공을 무효화하는 조건, 성공이 없을 때 범위/통계 미생성 등을 검사한다.

## 2026-09-20 실제 실행 결과

실행 폴더: `runs/20260920_113624_tomato05_pose_physics_validation/`.
기존 Isaac 환경에서 CPU PhysX로 coarse 31개를 실행했다. RL/scene randomization은 없다.
전체 31개가 IK/경로/자기 충돌 사전 검사를 통과했고, 모든 후보의 검사한 초기 상태는
기준 상태와 최대 절대 차이 **0**이었다. 도달 불가능한 별도 negative control은
`endpoint_ik`에서 실제 접근 전에 거절됐다(31개 통계에서 제외).

| Primary category | 개수 |
|---|---:|
| success_target_hook | 0 |
| non_target_contact | 10 |
| excessive_displacement | 21 |
| miss | 0 |
| ik_or_planning_failure | 0 |

전체/valid 성공률은 모두 0%다. 중복 event 집계에서는 비목표 접촉 31개, miss 31개,
과도한 밀림 21개다. 따라서 위의 primary miss 0개가 모두 목표를 걸었다는 뜻은 아니다.

목표 꼭지의 기계적 걸림을 1초 유지한 후보는 아래 3개였다. 그러나 모두 **Tomato_07의
꼭지를 먼저 접촉**했으므로 최종 성공으로 집계하지 않았다. Roll/방위각은 nominal 대비 값이다.

| 후보 | 변화 | 목표 최대 변위 | 주줄기 최대 변위 | 유지 중 접촉 tick / 60 |
|---|---|---:|---:|---:|
| coarse_0006 | 방위각 +30° | 18.669 mm | 29.393 mm | 44 |
| coarse_0008 | 방위각 +60° | 13.444 mm | 18.203 mm | 32 |
| coarse_0011 | roll −15° | 11.329 mm | 18.062 mm | 53 |

이는 현재 범위에서 걸림 가능성과 주변 꼭지 간섭을 구분해서 관찰한 결과다.
성공 후보가 없으므로 tolerance 물리 실행은 하지 않았고, 성공 가능한 pose 범위도
추정하지 않았다. Tolerance 생성/집계 로직은 테스트로 검증했다. 관련 전체 테스트는 39개 통과했다.

**RGB-D 데이터셋은 아직 미완성이다.** 현재 컨테이너에서 `nvidia-smi`가
`Failed to initialize NVML: Unknown Error`를 반환하고 CUDA 장치를 사용할 수 없어
명시적 `--physics-only`로 검증했다. RGB-D 파일은 없으며 null 경로와 미완성 상태를 저장했다.
GPU 접근 복구 후 기본 명령으로 새 실행을 해야 RGB-D 촬영까지 검증/완료할 수 있다.

실행 폴더의 `REPORT.md`, `validation_report.json`, `pose_results.png`, `results.csv`,
`metadata.jsonl`에서 결과를 확인할 수 있다. 추적 가능한 소형 요약은
`validation/pose_search_tomato05.json`에 보존한다.

## 31개 후보 동영상

완료된 coarse 실험을 아래 명령으로 다시 실행해 각 후보를 2배속 MP4로 저장한다.

```bash
./nvidia-sim/run_pose_videos.sh --source ABSOLUTE_POSE_RUN --workers 4

# GPU 복구 후: 같은 폴더의 완성 영상은 보존하고 누락/미완성 영상만 GPU로 렌더링
./nvidia-sim/run_pose_videos.sh --source ABSOLUTE_POSE_RUN \
  --run-dir ABSOLUTE_VIDEO_RUN --render-only --gpu --display :0 --workers 4
```

출력 폴더는 `runs/YYYYmmdd_HHMMSS_tomato05_31cases_video_2x/`다.
`index.html`에서 전체 영상을 볼 수 있고, `videos/coarse_XXXX_2x.mp4`는 개별 영상이다.
`videos.csv`와 `video_manifest.json`에 후보별 결과/영상 경로가 기록된다.
같은 `--run-dir`를 지정하면 완료된 기록과 영상은 재사용한다.

GPU 장치 접근이 막힌 동안에는 **실제 PhysX 상태를 CPU로 다시 그린 영상**을 만든다.
Isaac의 RTX 카메라 녹화나 학습용 RGB-D로 취급하지 않는다. 기존 Isaac Python 환경과
Blender를 사용하고, 독립적인 CPU 소프트웨어 화면을 위해 `xvfb`를 추가 설치했다.
CPU 모드는 사용자의 물리 모니터 `DISPLAY=:0`이나 GPU 드라이버 설정을 변경하지 않는다.
`--gpu`는 지정한 display의 NVIDIA OpenGL context에서 같은 원본 형상/기록을 렌더링한다.
GPU 모드는 실제 renderer가 NVIDIA인지 검사하고 장치명을 영상 metadata와 로그에 저장한다.
완료된 CPU 영상은 유지하므로 재개한 폴더에는 CPU/GPU 영상이 함께 있을 수 있다.
`videos.csv`/`video_manifest.json`의 `render_backend`와 영상 상단에서 구분할 수 있다.

- 원래 USD의 보이는 형상 1,439개(정점 3,563,415개)를 내보내고, 원래 상수 재질 색을 사용한다.
  RTX/MDL 조명·텍스처 대신 Blender Workbench의 studio shading으로 표시한다.
  카메라에 가려진 물체를 투명하게 만들거나 식물을 제거하지 않는다.
- 시뮬레이션은 기존 planner/executor를 그대로 호출한다. 별도 recorder가 scene update 후
  로봇·과실·탄성 articulation의 실제 body pose를 읽기만 한다. 이후 렌더링에서 원래
  `ElasticPlant`의 skinning weight와 attachment frame으로 시각 메시를 변형한다.
- 초기 상태, 계획 명령 전체, 실행 tick 수, 모든 tick의 관절 위치, 최초 접촉 대상,
  최대 변위, 걸림 유지 여부와 최종 판정을 기존 실험과 비교한다. 차이가 허용 오차
  1e-6을 넘거나 판정이 달라지면 오류로 처리하고 영상을 만들지 않는다.
- 물리 960 Hz / control 60 Hz를 유지하며, 4 control tick마다 상태를 기록한다.
  simulation 15 fps를 영상 30 fps로 인코딩하므로 물리 동작은 정확히 2배속이다.
  마지막에는 정확한 terminal state의 정지 화면을 1초 붙이며 이 구간은 명시적으로 표시한다.
- 긴 영상은 서로 겹치지 않는 프레임 구간으로 나누어 CPU 렌더링을 병렬 실행하고
  MP4를 시간 순서대로 연결한다. 중간 프레임을 생략하지 않으며, 마지막 구간에만
  1초 정지 화면을 넣는다. 연결 후 전체 프레임 수를 `ffprobe`로 검증한다.
- 전체 화면과 확대 화면을 나란히 배치한다. 노란색은 목표 Tomato_05, 하늘색은
  이웃 Tomato_07의 GT 중심 투영이다. 이는 물체 ID 확인용 표시이며 vision 검출이 아니다.
  화면의 변위는 현재 tick 값이고, 최종 판정/retained hook 표시는 실행 종료 후의 결과다.

`captures/coarse_XXXX/motion.npz`는 실제 body pose와 simulation timestamp,
`recording.json`은 원본 실행과의 일치 검증, `trace.json`/`contacts.json`은 물리 기록이다.
오프라인 렌더링은 이 기록에만 의존하므로 재생 배속과 화면 구성이 physics 결과에 영향을 주지 않는다.

## 직선 와이어 충돌 보완 및 27번 재검증 (2026-09-20)

원본 CAD에서 반원 끝과 몸체 사이의 직선 와이어 두 구간(각 28.40287 mm)에
collider가 누락된 것을 확인했다. `assets.py`는 이제 반경 1 mm의
`RailCollision/rail_00`, `rail_01` capsule을 추가한다. 기존 생성 파일을 사용한다면
`--rebuild`로 로봇 override를 갱신해야 한다. 기존 뒤쪽 반원의 걸림 판정은 유지한다.

31개 기록 중 11번과 27번에서 이 누락 구간을 가지가 관통했다.
이전 모델의 성공/실패 통계는 보완된 모델의 결과로 취급하지 않는다.
11개 과실의 가지 연결부는 기존 31개 실행의 15 Hz 기록에서 틈이 발견되지 않았다.
점검 요약은 `validation/connection_audit_tomato05.json`에 있다.

보완 후 27번만 동일 초기 상태/계획 명령으로 재실행했다. 직선 와이어가 목표 가지에
접촉하고, 상승 중 송이 중심줄기와 3.826 N으로 접촉해 기존 3 N 제한으로 중단됐다.
고리걸기는 실패했고 당김 단계는 실행하지 않았다. 2배속 GPU 영상은 마지막 정지 화면을
포함해 11.133초다. 경로와 결과는 `validation/pose_case27_repaired_rails.json`에 저장했다.

이처럼 충돌 모델을 수정한 비교 실행에서는 단일 후보 recorder에
`--pose-video-model-update`를 명시한다. 기존 planner/식물 코드 hash, 초기 상태,
계획 명령 일치 및 보완 collider를 검증하지만, 충돌 이후의 결과 일치는 요구하지 않는다.
`validation.reexecution`은 이전 결과와의 차이를 그대로 남기고 `validation.model_update`에
비교 실행 검증을 따로 저장한다. 과거 실험에서 assets.py hash가 없었던 사실도 기록한다.

```bash
./nvidia-sim/run_ring_rl.sh --mode pose-video --headless --rebuild \
  --pose-video-model-update --pose-video-source ABSOLUTE_POSE_RUN \
  --pose-candidate ABSOLUTE_POSE_RUN/inputs/coarse_0027.json \
  --run-dir NEW_RUN/captures/coarse_0027 --pose-video-export NEW_RUN/scene.npz
```

`runs/`의 대용량 영상/물리 기록은 기존 Git 제외 정책을 유지하며 로컬에 보관한다.
