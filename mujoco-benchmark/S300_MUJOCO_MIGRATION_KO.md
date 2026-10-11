# S300 농장·연속 수확의 MuJoCo 3.15 이식

2026-10-12 KST. 브랜치 `codex/s300-mujoco-belt-smoke-20261012`.
Isaac 보존 기준 `ca3a716`, 소규모 MuJoCo 시험 기준 `51e22eb` 이후 작업이다.

## 확보한 동작

기존 Isaac의 S300 기체, 리프트, RB5, 실제 턱 구동과 양쪽 안쪽 벨트를
MuJoCo **3.15.0**의 유한 힘 actuator로 연결했다. 전체 농장 외형을 표시하면서
한 환경에서 레일 이동 → 리프트/팔 준비 → 수평 진입 → 두 번 닫기/열기
→ 열매 분리 → 후퇴 → 다음 위치 이동을 정상 시작부터 실행했다.
기록 그리퍼 pose를 강제로 옮기던 이전 mocap 시험과 다르다.

두 송이의 선정된 익은 집단은 각각 03/04/05다. 480Hz 접촉/60Hz 이동 시험은
선정한 6개 모두 포함해 8개를 분리하고 두 동작을 완주했다. 활동 접촉 최대
0.483mm, 경고/발산/1mm 중단은 없었다. 분리 후 한 번에 잡을 수 있는 잔여 집단은 최대 2개라
기존 **최소 집단 3개** 규칙에 따라 종료했다. 이것은 모든 열매의 수확이 아니다.

300Hz 접촉/60Hz 이동의 새 자산 재생성 시험과 최종 실행기 재시험도 완주했고 9개가 분리됐다.
최종 9개는 **모두 기존 빨간색 수확 대상(color proxy)**이다. 선정한 03/04/05의 총 6개와
첫 송이의 02/07/08도 함께 분리됐다. source policy에서 07/08도 빨간색 판정 대상이며,
미숙으로 집계하지 않는다. 잔여 대상은 첫 송이 1개, 두 번째 4개지만 한 번에 잡을
집단은 최대 1/2개라 최소 3개 조건 미달로 추가 진입하지 않았다. 실제 레일 이동은 총 약 1.714m다.
활동 겹침 0.983mm로 1mm 기준은 통과하지만 여유가 작고, 480Hz와 분리 결과도
다르다. 더 낮은 주기를 모든 장면의 안전한 설정으로 일반화하지 않는다.
수거, 숙도 선택성, 실물 손상 한계는 검증하지 않았으며 `harvest_success=null`,
`training_eligible=false`, `full_physics_validated=false`를 유지한다.

## 사용 방법

```bash
cd /root/farmily_tomato
# 저장된 실제 결과를 1배속 GUI로 보기. 카메라 이동은 물리에 영향을 주지 않는다.
./mujoco-benchmark/run_s300_mujoco.sh replay

# 화면 없이 새 초기 상태부터 실제 물리 시험. 새 날짜 폴더, 5sim초마다 진행 로그.
./mujoco-benchmark/run_s300_mujoco.sh test

# 원본 자산에서 다시 변환: 기존 결과를 덮어쓰지 않는다.
./mujoco-benchmark/run_s300_mujoco.sh prepare
```

`replay`는 저장 qpos/qvel/attachment 상태의 표시이며 새로운 물리 계산이 아니다.
새 결과의 MJB·NPZ 해시를 재생 전에 검사한다. 해시가 없던 옛 smoke 결과는
호환 목적으로 열 수 있지만 해시 검사 완료로 기록하지 않는다.
`test`는 실제 물리 계산이다. 각 결과 폴더에 `index.html`, `summary.json`,
`states.npz`, `model.mjb`, 접촉/분리/주기/단계 로그, 실행한 plan, 입력·코드 해시를 저장한다.
영상 렌더링은 실행 시간과 분리하며 필요할 때 다음처럼 만든다.

```bash
./mujoco-benchmark/.mujoco315-venv/bin/python \
  mujoco-benchmark/scripts/s300_smoke/view_smoke.py RESULT_FOLDER \
  --camera aisle --output RESULT_FOLDER/actual_aisle.mp4 --fps 20
```

새 `prepare` 결과를 사용할 때는 출력 마지막 줄의 모델 경로를
`S300_MUJOCO_MODEL=/path/to/prepared/model`로 지정해 `test`를 실행한다.
최초 source 변환은 약 30초, 이번 새 모델 최초 컴파일·로딩은 약 4.7초였다.
캐시 재사용 로딩은 약 2초다. 캐시 검사·로딩, 계획, 저장, 렌더 시간은 루프 RTF와 구분한다.
새 모델의 컴파일 캐시는 병렬 시작 전에 한 번 생성한다.
현재 `test` 기본값은 **접촉 300Hz / 준비·이동 60Hz / native worker 0**다.
원하는 경우 `./mujoco-benchmark/run_s300_mujoco.sh test --hz 480`으로
활동 접촉의 주기만 높여 별도 새 시험을 만든다. 준비·이동은 60Hz를 유지한다.

## 실제 재사용한 코드와 자산

- `motion_kinematics.ToolKinematics/plan_motion`, `rail_motion`의 속도·도착 판정,
  `harvest_group_policy`의 색상 proxy/최소 수량/측정 상태 재선정 기능을 재사용했다.
- `plan_adapter.py`가 USD Python에서 기존 IK를 호출한다. native 실행 Python에는 USD가 필요 없다.
  계획은 IK 경로이며 별도의 FCL/전체 환경 무충돌 인증을 생성하지 않는다.
  계획과 actual command/drive/qpos는 별도 저장한다.
- 첫 기존 plan은 집단·belt aim·깊이가 동일할 때만 재사용한다. 다음 위치에서는
  실제 관절/턱/기체 위치를 시작점으로 다시 계획한다. 레일 원점은 한 MJB 안에서 공통이다.
- 원본 Plant29와 Plant41 assembly/trial은 읽기 전용이다.
  새 결합 모델의 window별 plan은 독립 복사와 SHA로 묶는다.
- 주줄기 16분절, 잎 27바디, 송이·열매 10개를 각 대상에서 유지한다.
  원래 줄기 분절이나 3축 탄성 자유도를 병합하지 않았다.
- 총 241바디/739자유도/5,101geom. rail/lift/arm/jaws 10구동, mocap 0개.
  로봇 실제 FK 16자세 최대 위치 오차 약 `5.02e-8m`.
- 주변 농장 메시를 동일 prototype/material 단위로 캐시·인스턴싱한다.
  새 snapshot의 약 4,867만 instanced triangle을 줄이지 않았고,
  source vertex 변환 오차는 약 `2.85e-8m`였다.
  이는 mesh/capsule의 생물학적 정확도를 의미하는 수치가 아니다.

대상 송이·자기 주줄기만 식물 충돌을 켜며 잎은 충돌하지 않는다.
다른 식물은 원본 외형만 표시한다. 다음 송이로 전환할 때 새 대상의 충돌을 켜고
이전 대상의 식물 충돌을 끈다. 이미 분리된 열매의 물리 충돌은 유지한다.
구조물은 거터 18개와 바닥, 로봇/그리퍼를 포함한다. 하우스의 모든 기둥·벽·레일이
동일한 충돌체로 이식됐다고 주장하지 않는다. 현재 미선택 두 번째 식물의 탄성 DOF도
모델 안에 남지만 충돌은 꺼져 있다.

## 실제 발견·수정한 문제

1. **검은 영상:** `mj_kinematics`와 `mj_comPos`만 호출하면 카메라/조명 위치는 갱신되지 않는다.
   `mj_camlight`를 추가하고 같은 기록에서 다시 렌더했다. 원본 물리 파일은 보존했다.
   캡션 이전 raw pixels와 전체 MP4 디코딩을 검사한다.
2. **관절 원점:** USD 비영점 joint state를 이미 구부린 body zero pose로 해석하지 않는다.
   실제 joint local frame으로 로봇을 조립한 뒤 초기 q를 적용한다.
3. **과도한 내부 힘:** PhysX `restOffset=0/contactOffset=.5mm`를 native
   `margin=.5mm/gap=.5mm`로 옮긴 것은 오류였다. MuJoCo 3.9 이후 gap은
   감지 범위를 추가하고 margin은 힘 발생 면을 팽창시킨다.
   `margin=0/gap=.5mm`로 수정했다. 동일 저장 상태의 짧은 비교에서 약 5,163N→12.05N.
   [MuJoCo 설명](https://mujoco.readthedocs.io/en/stable/computation/index.html#margin-and-gap),
   [3.9 변경 기록](https://mujoco.readthedocs.io/en/stable/changelog.html).
   정상 시작부터 새 결과로 재시험했다. 실패 원본도 남아 있다.
4. **마지막 열기만 60Hz:** 이름 목록에서 `cycle_open_final` 등을 빠뜨렸다.
   한 coarse step 뒤 꼭지/벨트 겹침이 .271→2.427mm로 늘었다.
   준비 구간만 명시적으로 저주기로 분류하며 모든 belt/open/hold/withdraw와
   알 수 없는 단계는 높은 주기를 유지한다.
5. **낙하 바닥:** 4cm 두께 바닥 cuboid를 빠른 낙하 과실이 통과했다.
   opt-in `top-plane`은 원본 cuboid 외형과 상면 -0.10m를 유지하면서 collision만 plane으로 바꾼다.
   같은 낙하 fixture에서 box는 1초 후 z=-5.606m, plane은 z=-.088m에 정착했다.
   plane의 무한 범위와 낙하 impact 겹침은 명시한다. 이를 1mm 낙하 정확도로 포장하지 않는다.
6. **낙하 과실/거터 중단:** 이미 떨어진 과실의 거터 impact도 이번 opt-in
   `--released-gutter-diagnostic`에서는 별도 진단한다. 충돌과 실제 힘은 유지한다.
   부착 과실/로봇의 거터 충돌, 다른 송이·주줄기·팔 접촉은 예외로 확대하지 않는다.
7. **재생 느림:** 압축 NPZ 전체를 매 프레임 다시 풀던 호출을 없앴다.
   한 번 메모리에 읽고 GUI/영상이 사용한다. 마지막 중단 표본도 영상에 포함한다.
8. **캐시 원본 보호:** merge의 하드링크된 캐시를 재컴파일해 원본을 변경하지 않도록 분리했다.
9. **GUI 종료:** passive viewer의 daemon render thread가 native model 정리보다 늦게
   종료되는 경합을 피하도록 창 닫기/CTRL-C 후 새 viewer thread의 종료를 기다린다.
   실제 DISPLAY=:0에서 제한 시간 재생·정상 종료를 확인했다.

## 속도와 CPU 실측

| 실제 전체 두 지점 시험 | sim초 | loop wall초 | RTF | 분리 | 최대 활동 겹침 |
|---|---:|---:|---:|---:|---:|
| 480Hz 접촉 / 60Hz 이동, worker 0 | 136.135 | 252.419 | .539 | 8 | .483mm |
| 300Hz 접촉 / 60Hz 이동, worker 0 | 136.137 | 177.790 | .766 | 9 | .983mm |
| 300Hz / 60Hz, worker 4 전 구간 | 136.137 | 194.703 | .699 | 9 | .983mm |
| 300Hz / 60Hz, worker 4 접촉 구간만 | 136.137 | 192.415 | .708 | 9 | .983mm |
| **최종 명령 실행: 300Hz / 60Hz, worker 0** | **136.137** | **174.589** | **.780** | **9** | **.983mm** |

짧은 53~56초 접촉 구간에서 worker 0/2/4/8 각 2회 평균은
16.80/14.51/14.41/14.58초였다. 모든 스텝 위치·속도·명령·분리 이벤트가 정확히 같았다.
하지만 전체 구간 worker 4는 오히려 느렸다. 큰 접촉 구간과 가벼운 준비 구간의
thread overhead를 구분해야 한다. 최종 실행과 worker 0/4 전 구간/4 접촉 구간의
네 실행 모두 저장된 13종 배열(time/qpos/qvel/attachment/명령/drive/과실 위치 등)이
완전히 같다. `campaign_final/repeat_reproduction.json`에 비교 결과를 보존했다.
GPU 실행, 16-thread 사용, 48코어 최대 활용을 주장하지 않는다.

최종 실행의 native physics는 144.85초, 검사·제어·상태 기록을 포함한 루프는
174.59초다. 모델 로딩 2.03초, 계획 2.71초, 저장 등을 포함한 run 내부 전체는
182.29초다. 루프 RTF .780은 아직 실시간을 넘지 못한다. 영상은 1배속 표시라
계산 속도와 구분한다. CPU process 시간 178.90초, RSS 약 1.40GiB다.
낙하 후 별도 진단 접촉은 최대 15.37mm였으므로 전체 접촉이 1mm 이하였다고
해석하지 않는다. 최악의 활동 접촉은 첫 송이 꼭지 distal04_0 ↔ `Tool_L_belt`다.

## 검사와 재현

native 기존 40개, USD 자산 12개, 보고서 회귀 4개, 재생 해시 5개,
진단 조명 1개로 **총 62개 PASS**를 확인했다.
최종 재실행 결과와 전체 테스트 로그는 공유 폴더의 `logs/`에 보존한다.

```bash
MUJOCO_GL=egl PYTHONPATH=mujoco-benchmark/scripts/s300_smoke \
  ./mujoco-benchmark/.mujoco315-venv/bin/python -m unittest \
  test_smoke test_view_smoke test_dynamic_session test_run_harvest \
  test_plan_adapter test_contact_semantics test_merge_windows test_make_report -v

PYTHONPATH=mujoco-benchmark/scripts/s300_smoke \
  ./nvidia-sim/.asset-venv/bin/python -m unittest test_farm_adapter test_robot_adapter -v
```

실제 source 첫 plan의 4,969개 표본 명령은 adapter와 최대 약 `2.38e-7` 이내였다.
300Hz 전체 반복은 모든 저장 배열이 정확히 같았다. GUI는 물리 엔진을 재실행하지
않고 기록을 보여준다. replay SHA 보호와 보고서의 역사적 schema 호환 수정은
최종 physics 실행 뒤에 추가했으며, 실행 당시 코드 사본과 현재 표시 코드를 구분한다.

## 실제 상태 영상

대표 결과 `campaign_final`에서 새로 렌더한 영상이다. 작은 15px 자막이며
전체 농장 외형을 숨기지 않았다. 캡션 이전 프레임 밝기와 전체 MP4 디코딩을 검사했다.

- `campaign_final/actual_aisle_full.mp4`: 0~136.137초, 두 지점 이동·수확 전체, 20fps.
- `campaign_final/actual_belt_plant29.mp4`: 45~72초, 첫 송이의 두 번 닫기/열기, 30fps.
- `campaign_final/actual_fruit_plant41.mp4`: 106~121초, 두 번째 송이의 최종 04 분리까지, 30fps.
- `campaign_final/actual_harvest_gap_plant41.mp4`: 같은 106~121초를 턱 입구의
  진단용 가상 카메라·렌더 전용 조명으로 표시. 형상/물리/저장 모델은 유지한다.
  기본 근접 뷰의 실제 잎 가림도 숨기지 않고 원본 영상에 남긴다.

각 영상의 JSON은 원본 MJB/상태 SHA, 시간 범위, 화면 검사, 렌더 시간을 포함한다.
이전 480Hz 영상과 실패 중단 영상도 root `index.html`에 별도 trial명으로 보존했다.
두 번째 송이의 106~114초 초기 분리만 표시한 원본 클립도
`actual_fruit_plant41_first_releases.mp4`로 보존했다. 최종 04 분리를 판단할 때는
121초까지 표시하는 위 영상들을 사용한다.

## 엔진 차이와 미완료

- source PD 강성·감쇠·force cap 유지: 레일/리프트 10,000N, 팔 250Nm, 턱 각각 10N.
  native robot bias feedforward를 cap 안에서 적용하고 별도 drive 배열로 저장한다.
  plant는 source처럼 초기 자세에서 계산한 고정 gravity preload를 사용한다.
- Fruit freejoint + connect + 상대 회전 spring/감쇠, 측정 힘 3N 또는 torque .008Nm 해제다.
  타이머나 pose 보정으로 열매를 분리하지 않는다. PhysX native D6와 생체 조직의 동등성은 미검증이다.
- CAD mesh별 convex hull은 PhysX convex decomposition과 다르다. 추정 관성과
  MuJoCo solver 접촉 파라미터를 그대로 실물 정확도나 엔진 완전 동등성으로 해석하지 않는다.
- active 가지 표시는 원래 collision capsule과 강체 메시를 사용한다.
  source GUI의 blended skin 변형과 crank/link 표시 애니메이션은 아직 동일하게 옮기지 않았다.
  crank/link는 source에서도 collision 없는 visual-only이며 실제 턱 구동과 질량은 옮겼다.
- 최종 실행에서는 색상상 비대상 09/10/11의 분리가 없었다. 이 한 번의 결과로
  selective harvest, 생물학적 숙도나 손상 한계가 검증되는 것은 아니다. 받이/수거도 미완료다.
- 현재는 SIM-GT 집단 선택이다. RGB-D/실물 카메라 판단과 전체 행로 최적화,
  전체 온실 모든 송이의 자동 수확, GPU 대량 환경은 아직 구현·검증하지 않았다.
- Isaac GUI의 과거 한 스텝 동시 파단 원인이 해결됐다고 주장하지 않는다.

결과와 모든 실패 시험: `/root/docker_share/mujoko_debugging_data/20261012_s300_mujoco_full/index.html`.
새 재생성 자산: `/root/docker_share/mujoko_debugging_data/20261012_s300_mujoco_rebuilt/model`.
코드/작은 요약은 Git, MJB/상태/MP4/원본 로그는 공유 폴더에 보존한다.
