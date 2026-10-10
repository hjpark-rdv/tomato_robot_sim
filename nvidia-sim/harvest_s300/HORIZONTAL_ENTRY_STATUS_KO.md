# S300 · RB5 그리퍼 수평 진입 수정

2026-10-10. 작업 브랜치 `codex/s300-harvest-isaac-20261009`, 수정 기준 `aad8340`.

## 수정한 동작

사용자가 요구한 것은 그리퍼 입구를 수평으로 유지한 채 옆으로 전진하는 진입이다.
이전 실행의 `plan.json`은 `pitch_deg=-60`이었다. 기존 계획기가 0/-30/-60/+30°를
자동 비교해 -60°를 선택했고, CAD +Y 방향으로 100mm 접근하면서 86.60mm 내려가게 했다.

`plan_motion`은 이제 요청한 pitch/yaw만 사용한다. 기본 pitch는 0°이며 IK가 어렵다고
그리퍼를 숙이지 않는다. 같은 목표 자세에서 다른 관절 branch와 전체 접근을 검사한다.
`--approach-yaw-offset-deg`는 로봇→타깃 방향에 대해 **월드 +Z축 주위로** 회전하는
수평 방향 보정이다. 상하 기울기는 `--approach-pitch-deg`다.

현재 타깃은 그대로 `Truss_02_Fruit_02`다. radial 0° 방향의 수평 준비 자세는 기존
9개 seed에서 IK를 찾지 못했고, 수평 yaw offset +15/+30°에서는 100mm 준비 거리와
61개 접근 waypoint를 연결했다. +30°의 실제 세계 방향은 약 `[-0.999902,-0.013990,0]`이다.
고리/벨트 기준점, 목표 중심, 과실 크기, 충돌체, 물성, actuator와 0.5mm 관통 기준은 바꾸지 않았다.

별도 `tight_fold_level_approach` 준비 profile은 RB5의 어깨·팔꿈치 변화에 따라 wrist1을
보정한다. 이 로봇의 실제 composed USD FK로 extension 중 81개 위치를 검사하고,
관절 제한과 요청 회전에 대한 오차를 확인한다. 일반적인 모든 로봇에 통하는 공식으로
해석하지 않는다. 준비 단계에서는 그리퍼 높이가 변할 수 있고, 최종 `approach`는 높이가 일정하다.

## 검사 결과와 한계

결과 폴더:
`/root/docker_share/mujoko_debugging_data/20261010_125732_s300_horizontal_entry/`

- 기존 기구/준비 회귀 9개 + 새 수평/prefix/수평 준비 검사 15개, 총 **24개 통과**.
- +30° 기본 준비의 61개 접근 FK: 높이 변동 `2.38e-7mm`, 최대 입구 상하각 `4.46e-7°`.
- 수평 준비 profile의 81개 extension FK: 최대 입구 상하각 약 `0.000159°`.
- 기존 SI 실행의 처음 두 준비 구간을 재사용한 3회는 처음 12초의 **명령·timestamp·phase·실제 body pose가 원본과 bit-exact**였다.
- 실제 물리 5회는 **모두 approach 전에 중단**했다. 수평 진입 완주나 수확 성공은 확인하지 못했다.
- 마지막 수평 준비 실행은 실제 extension의 157개 표본에서 입구 상하각 최대 약 `5.94°`였다.
  이는 명령 FK와 다른 실제 추종/접촉 결과다. 목표 진입 구간의 수평 유지 증거로 집계하지 않는다.

| 실제 실행 | 준비 방식 | 중단 시점 | 중단 단계 / 물체 |
|---|---|---:|---|
| yaw +30° | 기존 profile, 새 IK branch | 6.883s | prepare_fold / 주변 Plant_19 |
| yaw +15° | 기존 profile, 새 IK branch | 5.417s | prepare_fold / 주변 Plant_19 |
| yaw +30° | 앞 두 구간 원본 재사용 | 23.450s | prepare_extend / 목표 줄기 잎 및 다른 과실 꼭지 |
| yaw +15° | 앞 두 구간 원본 재사용 | 21.542s | prepare_extend / 목표 줄기 잎 |
| yaw +30° | 원본 앞 두 구간 + 수평 준비 | 21.308s | prepare_extend / 주변 Plant_20 |

이 결과는 정상 시작에서의 실제 접촉 진단이다. `training_eligible=false`,
`harvest_success=null`을 유지했다. 실패를 절대 수확 불가능으로 분류하지 않았다.
기존 source/실험 결과는 유지하며 모든 새 상태·접촉·로그·입력 hash는 새 폴더에 있다.
영상은 실제 저장 상태를 표시하며 physics를 다시 실행하지 않는다.

## 직접 화면에서 실행

아래 명령은 마지막 수평 준비 진단을 새 결과 폴더에서 실제 physics로 다시 실행한다.
준비 중에는 자세를 바꾸고, 펼치기와 최종 진입에서는 요청한 수평 자세를 유지하도록 명령한다.
현재 장애물 충돌로 중단됐던 경로이므로 정상 수확 완료 명령으로 해석하지 않는다.

```bash
cd /root/farmily_tomato
motion_out="/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_s300_horizontal_gui"
DISPLAY=:0 OMNI_KIT_ALLOW_ROOT=1 /root/isaac-sim-6.1.0/python.sh --no-ros-env -u \
  nvidia-sim/harvest_s300/run_motion.py \
  /root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_motion_assembly_metre_z_rebuilt \
  --output "$motion_out" --mode harvest \
  --preparation tight_fold_level_approach \
  --preparation-prefix-plan /root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_metre_z_rebuilt_harvest_open120/plan.json \
  --approach-pitch-deg 0 --approach-yaw-offset-deg 30 \
  --device cpu --hz 120 --threads 2 --performance optimized --display-fps 30 \
  --gravity-neutral-warmup --armature-setup authored \
  --buffer-contact-events --record-contact-frames \
  --contact-backend callback --collision-filter pairs --plant-dynamics full \
  --solver PGS --position-iterations 64 --velocity-iterations 4
```

화면 갱신은 30FPS, 물리는 120Hz다. 결과 및 상태는 새 `motion_out`에 저장된다.
source scene SHA는 `b25148c8210ef0ff540f7186c4a03840c0e355eaa71a360c340cb1a4dbd22f44`다.

원자료: [horizontal_entry_validation.json](/root/docker_share/mujoko_debugging_data/20261010_125732_s300_horizontal_entry/horizontal_entry_validation.json).
영상/전체 시도: [index.html](/root/docker_share/mujoko_debugging_data/20261010_125732_s300_horizontal_entry/index.html).
호스트 주소: `file:///home/rdv/docker_share/mujoko_debugging_data/20261010_125732_s300_horizontal_entry/index.html`
