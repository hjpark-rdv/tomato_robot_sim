# S300 · RB5 영상 기준 대기 자세와 수평 진입

2026-10-10 KST. 브랜치 `codex/s300-harvest-isaac-20261009`, 작업 기준 `aad8340`.
이 문서의 수정은 이전 수평 진입 작업 위에 추가했다. 코드·테스트·문서·검증 요약을 함께 커밋한다.
커밋은 `git log -- nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md`로 확인한다.
검증 JSON의 `uncommitted=true`는 실험 당시 코드 상태이며, 이 기록을 사후에 커밋한 것과 구분한다.

## 최신: 여러 열매 접촉 허용 · 접촉면 시험안에서 분리·후퇴 완료 (2026-10-10)

사용자가 여러 열매의 접촉·밀림을 허용했다. 기존 실행기는 접촉 대상만으로 중단하지 않았고,
앞선 중단은0.5mm 관통 기준 때문이었다. 관통 기준은 유지했다. 타깃11번 이외에도
활성 식물30열매의 정확한 접촉/파단 조인트와 실제 이동을 개별 기록한다.

**최종 시험안에서 Truss_02_Fruit_10의 native 연결부가 끊어져 낙하했고,
30.5초의 선개방 진입 → 닫기 → 벨트2초 → 개방 → 후퇴·복귀가 완료됐다.**
전체 관통 최댓값은0.2463mm이며 실제 로봇 자세 자기충돌 검사도 통과했다.
이 결과는 원래 V5.1의 성능이 아니라 **별도 접촉면 형상 시험안**이다.
동시에 여러 열매를 수확하거나 포집한 결과/실물 안전 검증으로 확대하지 않는다.

### 변경한 동작과 별도 형상 시험안

- 먼저 기존 V5.1 그대로10·11번 과실 중심의 중간 위치에 수평 진입했다. 기존 리프트 정착 오차를
  반영해 계획상의 높이만9mm 올렸고, 닫기는3→6초로 변경했다. 실제 상태를 보정/순간이동하지 않았다.
- 원래 V5.1의480Hz 두 실행은 주변5열매를 접촉하며 완주했으나 파단0이었다.
  한 열매보다28mm 최소간격이 넓다는 사실만으로 여러 열매 묶음의 포획을 불가능으로 단정하지 않는다.
- 별도 `assembly_tread5`: 좌우 벨트 안쪽에5mm 두께·120×52mm 접촉면을 추가했다.
  **표시 메시와 충돌체를 함께 추가**했으며 원래 충돌체는 제거하지 않았다(31,464→31,466).
  TPU 밀도1200kg/m³로 좌우 각각37.44g을 추가했다. 기계적 jaw stroke28~96mm는 그대로,
  추가 접촉면 기준 최소간격은18mm다. 원본 CAD 및 별도 V6 작업은 건드리지 않았다.
  이 접촉면은 기존 벨트와 같은 강체/표면속도 모델이며 제작·벨트 순환/변형 검증 전 시험안이다.
- 최종 벨트 명령: 왼쪽−0.10m/s, 오른쪽0m/s, 총2초 안에서 시작/끝0.5초씩 부드럽게 가감속.
  차동 성분0.05m/s와 안쪽 이송 성분0.05m/s를 합친 구동이다. 기존±0.15m/s 시험과 구분한다.
  팔은 벨트 단계에서 고정한다. 마찰·구동게인/최대힘·식물 강성·3N/0.08Nm 파단값을 바꾸지 않았다.
- 과실 바디 CCD는 켜져 있었으나 scene CCD는 기본false였다. `--scene-ccd`를 opt-in으로 추가했고
  최종 CPU 시험의 runtime/native 설정에서true를 확인했다. 기본 실행의 CCD 설정은 보존한다.
  기존480Hz 감속 시험은10번 분리 후 바닥에서0.811mm 관통으로 중단됐고,
  동일 모델/명령에 scene CCD를 켜자 낙하·후퇴까지 완료됐다.

### 전체 시도 기록

| 실행 | 전체 동작 | 벨트 실행 s | native 열매 파단 수 | 중단/비고 |
|---|---|---:|---:|---|
| pair_close6_240 | 미완료 | 0.0292 | 0 | penetration_over_0_5mm |
| pair_close6_480 | 미완료 | 미확인 | 미확인 | controller timeout; no stored state |
| pair_ramp_240 | 미완료 | 0.1542 | 0 | penetration_over_0_5mm |
| pair_480_full | 완료 | 2.0000 | 0 | 기존 관통·실제 자기충돌 검사 통과 |
| pair_infeed_480 | 완료 | 2.0000 | 0 | 기존 관통·실제 자기충돌 검사 통과 |
| tread5_infeed_480 | 미완료 | 0.3021 | 2 | penetration_over_0_5mm |
| tread5_slow_480 | 미완료 | 1.4563 | 1 | penetration_over_0_5mm |
| tread5_lower_480 | 미완료 | 0.0000 | 2 | penetration_over_0_5mm |
| tread5_slow_ccd480 | 완료 | 2.0000 | 1 | 기존 관통·실제 자기충돌 검사 통과 |

`pair_close6_480`은 컨트롤러의240초 wall timeout으로 저장 전에 종료됐다. 로그에 벨트·후퇴가
있어도 완료로 세지 않았다. 이후 timeout600초로 재시험했다. 잘못된 `--parallel-jobs` CLI 시도는
실제 물리를 시작하지 않았으며 로그를 보존했다. 두 최종 원래 V5.1 시험은 CPU 프로세스2개로
동시 실행했으므로 각 wall time을 단독 속도 벤치마크로 비교하지 않는다.

빠른 접촉면 시험의10·8번 파단 이벤트2건은 상판–8번3.091mm 관통을 포함하므로 정상 수확으로
세지 않는다. 12mm 낮춘 별도 경로도9번–그리퍼 관통으로 닫기 중 중단돼 채택하지 않았다.
열매의 세계 회전, 꼭지 대비 상대회전, 실제 파단은 따로 기록한다. 고정 조인트가 흔들린 누적
각도 합을 실제 여러 바퀴 회전으로 해석하지 않는다. 모든 결과는 기존 학습 데이터에서 제외한다.
최종10번은 파단 전 세계 자세 변화 최대44.87°, 꼭지 바디 대비 상대변화 최대3.31°였고,
파단 후12.00초간 상태가 저장됐다. 마지막 과실 중심 높이는12.42mm, 초기 꼭지 좌표계 대비
이동은약1,003mm로 실제 낙하를 확인했다. native 이벤트만으로 힘/토크 중 어느 한계가
파단을 유발했는지는 구분할 수 없으며, 순수 비틀림만으로 분리됐다고 주장하지 않는다.

최종 처리 시간은 sim30.50초 / wall221.77초이며,480Hz 접촉 검증용으로
**실시간 동작은 아니다**. 로딩/계획·실제 자세 검사/저장/영상 렌더는 별도다.
렌더는 저장된 실제 body pose를1배속으로 재생한다. 접촉을 보기 위해 재생 화면에서만 잎을 숨겼다.

검증: 관련54테스트PASS, 기존 legacy/entry-only 명령 배열 동일. 원래 모델의 식물·그리퍼
USD subtree가 정확히 같은 것을 확인했고, 접촉면 시험안도 식물/로봇 원본은 동일하다.
최종 입력·코드·상태·접촉 해시와 전체 결과는 `validation/cluster_grasp_20261010.json`.
새 변경의 커밋 여부는 `git status`로 확인한다.

결과/영상:

```text
file:///home/rdv/docker_share/mujoko_debugging_data/20261010_161007_s300_cluster_grasp/index.html
```

최종 시험안 새 물리 실행+GUI(새 폴더 저장):

```bash
bash /root/docker_share/mujoko_debugging_data/20261010_161007_s300_cluster_grasp/run_prototype_gui.sh
```

원래 V5.1의 완주하지만 분리0인 대조군은 같은 폴더의 `run_gui.sh`다.
GUI 런처의 새 인자는 문법 확인했고, 이번 검증의 실제 실행은 headless다.

## 최신: 닫기 → 벨트 2초 → 개방·후퇴 연결 (2026-10-10)

사용자가 확인한 순서는 **벌려 진입 → 닫아 잡기 → 벨트 2초 회전 → 벌려 후퇴**다.
`--harvest-profile belt_spin_release`를 추가했다. 기존 `legacy`와 `--entry-only` 명령,
그리퍼 V6 작업, 원래 CAD/식물은 보존했다. 새 순서는96mm 선개방 → 수평 진입 →
3초간28mm 닫기 → 팔 자세 고정·좌우 벨트−0.15/+0.15m/s로2초 → 1초간96mm 개방
→ 후퇴 → 대기 복귀다. 0.15m/s는 이전 비틀기의10배인 시험 표면속도이며 모터RPM이나
검증된 모터 성능이 아니다. `--belt-speed-m-s`, `--belt-seconds`로 지정할 수 있다.
닫기 명령 완료가 실제 포획을 확인했다는 뜻은 아니다.

같은 `assembly_v3` / `Truss_02_Fruit_11`의 정상 시작2회에서 full 식물612DOF,
자기충돌/주변충돌, 물성/구동, 3N/0.08Nm 파단값과0.5mm 관통 중단을 유지했다.
물리 설정은 대조군의Hz120→240만 달리했다.

| 물리 | 종료 시각 | 중단 원인 | 벨트 실제 실행 | 목표 분리 |
|---|---:|---|---:|---|
| PGS120 | 14.0833초 | 닫기 중 왼쪽 상판part_127 ↔ 옆10번 과실, 관통0.5059mm | 0초 | 없음 |
| PGS240 | 14.5875초 | 오른쪽 벨트 ↔ 목표11번 과실, 관통0.5309mm | 0.0875초/21스텝 | 없음 |

두 경우 모두 **왼쪽 벨트–목표 접촉0점**이다. 오른쪽만120Hz152점(최대 단일점0.985N),
240Hz814점(1.555N)을 기록했다. 접촉점 개수는 독립 포획 횟수가 아니다.
240Hz 벨트 단계의 실제 개방 간격도30.36~30.96mm로 명령28mm에 도달하지 못했다.
후퇴까지의 실제 완주/수확 성공은 아니다. 옆 과실 접촉도 실제 물리로 처리했으며, 관통 기준 초과로 중단했다.

원본 목표 충돌 메시의 잡는 축 폭23.053mm, 두 점 사이 최대거리23.119mm는 최소간격28mm보다 작다.
폭 차이는4.95mm이며, 전체30개 활성 과실도 최대 두 점 거리27.235mm 이하로 확인했다.
이는 원본 mesh/convex hull 기하 분석이며 native cooking/contact offset과 구분한다.
실제 타깃의 양쪽 벨트 접촉도 확보하지 못했다. 열매를 키우거나 CAD와 다른 닫힘 한계를 적용하지 않았다.

파단 연결은 실제 `Truss_02_Pedicel_distal_11_0` ↔ `Truss_02_Fruit_11`의 native FixedJoint다.
두 시험 모두 목표 조인트 파단 이벤트0. 꼭지 전체를 함께 뜯어야만 분리되는 구조는 아니며,
꽃받침(calyx) 표시 메시가 과실 바디를 따르는 기존 연결은 유지했다.
꼭지 앞쪽 spring은 휘고 회전하지만 과실–distal 연결 자체는 파단 전 고정 제약이다.
생체 비틀림 피로/이탈 토크는 미보정이다. 상대회전 최대4.97/7.15°, anchor간격 최대1.07/1.49mm는
제약 오차도 포함한 관측값이며 정상 비틀기/분리 증거로 해석하지 않는다.

`analyze_belt_trial.py`는 저장 상태에서 실제 timestamp별 단계 시간, 정확한 좌우 벨트 접촉,
세계 회전과 꼭지 대비 상대회전, 목표 파단조인트, anchor 간격을 분리해 기록한다.
전체 송이가 돌아간 것을 열매–꼭지 상대회전으로 세지 않는다. `harvest_success=null`, 학습 제외 유지.
다음 우선순위는23mm 과실에 맞는 기계적 닫힘 범위/접촉면과 옆 과실에 닿는 상판 간섭이다.
양쪽 포획을 확보한 뒤 회전·파단 응답을 검증해야 하며, 2초가 지나면 강제로 분리하는 코드는 없다.

검증: 관련42테스트PASS. 직전커밋의 legacy/entry-only 명령과 새 코드가 정확히 같다.
새120Hz 닫기 전1,381개 실제 time/phase/명령/관절/body pose도 기존 기준과 bit-exact다.
두 실행의 계획/실제 자기충돌 검사PASS. 최초 실행기의 잘못된 CLI인자와 초기 테스트의 SciPy
batch Euler 입력 형태 오류는 수정 후 통과했으며 로그는 보존했다. 물리 실패와 구분한다.

실제 상태·접촉·확대 MP4: `/root/docker_share/mujoko_debugging_data/20261010_153918_s300_belt_spin_release/`.
작은 요약은 `validation/belt_spin_release_20261010.json`. 새 변경의 커밋 여부는 `git status` 확인.

GUI 재실행(현재 장애물 중단을 재현하며 수확 성공 시연이 아님):

```bash
cd /root/farmily_tomato
bash nvidia-sim/harvest_s300/run_motion_live.sh \
  /root/docker_share/mujoko_debugging_data/20261010_142120_s300_self_collision_fix/assembly_v3 \
  reference belt
```

```text
file:///home/rdv/docker_share/mujoko_debugging_data/20261010_153918_s300_belt_spin_release/index.html
```

## 이전: 화면을 보면서 실시간 진입·복귀 (2026-10-10)

기준 커밋 `56c5d01`, 동일한 `assembly_v3`에서 실제 새 물리 실행을 비교했다.
별도 프로세스의 기존 measured-pose viewer로 표시를 분리하고, **선택형 PGS60 미리보기에서
23.5초 동작을 실제 23.500초에 완료**했다. 저장 영상의 배속 재생이 아니다.

| 실행 | 물리 설정 | 동작 sim초 / wall초 | 실시간 배율 |
|---|---|---:|---:|
| 기존 동기 GUI | PGS64/4 · 120Hz | 23.5 / 95.304 | 0.247배 |
| 기존 headless | PGS64/4 · 120Hz | 23.5 / 38.621 | 0.608배 |
| 새 분리 GUI · reference | PGS64/4 · 120Hz | 23.5 / 40.388 | 0.582배 |
| 새 분리 GUI · interactive | PGS64/4 · 60Hz | 23.5 / 23.500 | **1.000배** |

로딩·사전/사후 검사·저장은 위 시간에 포함하지 않는다. 각 설정 1회 측정이며 서버 부하에 따라 달라진다.
60Hz headless의 첫 11.5초 구간은 9.350초(1.230배)였다. GUI 실행은 `--realtime`으로
남는 시간에만 sleep을 넣어 1배속으로 맞췄다(총 sleep 3.334초). 물리 스텝을 생략하지 않는다.
뷰어는 VNC의 `DISPLAY=:0`에서 실행하고, 첫 실제 상태가 화면에 표시된 다음 물리를 시작한다.
표시 갱신 상한은 15fps이며 실제 표시율은 렌더링 비용에 따라 더 낮다. 물리 주기와 별개다.

**60Hz는 시간 간격을 바꾼 미리보기이며, 120Hz와 물리적으로 동일하다고 주장하지 않는다.**
같은 0.1N 외력(+X, 1~2초)·7초 복원 시험의 최대 과실 이동은
PGS120 15.011mm → PGS60 16.095mm(약 +7.23%), 마지막 이동은 0.0275/0.0266mm였다.
TGS60도 비교했지만 짧은 진입 0.974배·부하 최대17.610mm로 이번 미리보기에는 채택하지 않았다.
원래 full 식물612 DOF, 전체충돌31,464개, 마찰·강성·질량·구동게인·자기충돌은 유지했다.
관통 중단 기준0.5mm도 유지하지만 60Hz는 검사 시간 간격이 더 크다. 빠른 접촉/닫기/벨트 이송의
유효성까지 보증하지 않으며, 기존 120Hz reference 기본값을 바꾸지 않는다.

새 120Hz 분리 GUI와 이전 headless의 전체2,821개 time/phase/command/관절/body pose는 bit-exact다.
60Hz 분리 GUI와 같은 설정 headless의 공통691표본, 60Hz 재실행 전체1,411표본도 bit-exact다.
최종 GUI 두 설정 모두 native 자기접촉0, 실제 자세 교차검사PASS, 파단0, 관통 중단 없음.
60Hz 실제 진입pitch 최대0.0331°, 진입TCP높이범위0.150mm다.
관련 기존 자동 테스트53개PASS, 문법검사 및 실제 launcher 종료코드0을 확인했다.

빠른 실행(매번 새 결과 폴더, 완료 후 `index.html`과 실제 상태·접촉 저장):

```bash
bash /root/docker_share/mujoko_debugging_data/20261010_145123_s300_realtime_review/run_gui.sh
```

같은 실행기에 `reference`를 붙이면 120Hz 물리로 확인한다. 저장소 코드 직접 실행:

```bash
cd /root/farmily_tomato
bash nvidia-sim/harvest_s300/run_motion_live.sh \
  /root/docker_share/mujoko_debugging_data/20261010_142120_s300_self_collision_fix/assembly_v3 \
  interactive
```

일반 실행기의 프로필 생략 기본값은 `reference`다. `S300_MOTION_OUTPUT`으로 새 출력 폴더,
`S300_DISPLAY`로 화면(기본`:0`), `S300_VIEW_SECONDS`로 표시 시간을 지정할 수 있다.
기본 표시시간0은 완료 후에도 창을 유지한다. `physics.log`/`viewer.log`와 화면에 진행 상태가 보인다.
런처는 자기 자식 프로세스만 정리한다. 기존 실행기·뷰어·결과 생성기를 재사용한다.

결과: `/root/docker_share/mujoko_debugging_data/20261010_145123_s300_realtime_review/`.
`live_pgs60_vnc`/`live_pgs120_vnc`가 최종 실제 VNC 비교다. 처음 `live_pgs60`은 물리는
완료했지만 실행 중 shell 파일 수정으로 launcher 오류가 났고, `live_pgs60_final`은 상속된
가상화면`:171`의 viewer 시작에서 멈춰 물리를 시작하지 않았다. 실패 로그를 보존하고
실행기 오류와 물리 실패를 구분했다. 화면 번호를 명시한 후 재실행했다.
작은 요약·코드 해시는 `validation/realtime_preview_20261010.json`에 저장한다.

```text
file:///home/rdv/docker_share/mujoko_debugging_data/20261010_145123_s300_realtime_review/index.html
```

범위는 **열린 그리퍼의 진입·유지·복귀**다. 과실 포획·분리 수확이나 GPU 병렬 확장은 이번 검증에
포함하지 않았다. `training_eligible=false`, `harvest_success=null` 유지.

## 이전 수정: 자기충돌을 켠 진입·유지·복귀 재검증

사용자 지적의 그리퍼–RB5 관통을 수정했다. 새 결과는
`/root/docker_share/mujoko_debugging_data/20261010_142120_s300_self_collision_fix/`이며
**`assembly_v3`만 최종 검증 모델**이다. 아래의 이전 300mm 대기 자세/자기충돌 미검증 기록은 과거 결과다.

- 로봇 articulation의 자기충돌을 활성화하고, 비활성인 기존 모델의 harvest 실행을 차단한다.
- 초기 IK branch를 다시 선택했다. 목표 11번 열매의 위치는 유지하고, 대기는 목표에서 160mm 뒤,
  수평 준비 60mm → 진입 100mm → 2초 유지 → 후퇴 → 대기 복귀 후 닫기다. pitch 0°, yaw +30° 유지.
- 그리퍼 좌우 슬라이더의 부모를 로봇 플랜지에서 **실제 고정 레일 프레임**으로 바로잡았다.
  조인트의 세계 anchor/축/구동값은 유지했다. 기존 직접 연결 조인트의 충돌 제외를 적용하며,
  그리퍼와 비인접 팔 링크 사이를 제외하는 필터는 추가하지 않았다.
- 실제 USD 충돌체와 움직이는 jaw를 FCL로 검사한다. 계획 120Hz/2,589표본에는 2mm 여유를 요구하고,
  실행 뒤에는 저장한 실제 2,821개 자세를 모두 교차 검사한다. native 접촉·0.5mm 관통 중단도 유지한다.
  도구는 부품별 convex envelope로 사전 선별하며 PhysX의 convex cooking과 동일하다고 가정하지 않는다.

| 실행 | 결과 | sim 시간 | 처리 시간 | 자기접촉 | 실제 자세 교차 검사 |
|---|---|---:|---:|---:|---|
| 1차 | 초기 슬라이더–고정 레일 관통 1.727mm로 중단 | 0초 | 초기화 별도 | 있음 | FCL만으로 native 오류를 대체하지 않음 |
| 2차 | 초기 그리퍼 후면 커버–link3 관통 0.786mm로 중단 | 0초 | 초기화 별도 | 있음 | 약 0.54mm FCL 여유에도 native 충돌 발생 |
| 최종 v3 headless | 진입·유지·복귀 완료 | 23.5초 | 38.62초 | 0건 | 2,821표본 PASS |
| 최종 v3 GUI | 진입·유지·복귀 완료 | 23.5초 | 95.30초 | 0건 | 2,821표본 PASS |

두 최종 실행의 time/phase/command/관절/248개 body pose가 전부 bit-exact다.
진입 기울기는 최대 0.0331°, 진입 중 TCP 높이 변화 0.150mm, 목표 열매 변위 최대 0.0316mm.
초기 중력 정착에서는 기존 리프트 구동이 내려앉으며 TCP가 약 8.66mm 내려간다. 전체 높이 범위 8.86mm를
진입 오차와 혼동하지 않는다. 정착 후 대기와 최종 복귀의 관절 차이는 최대 7.15e-7 rad/m다.
충돌 형상·마찰·물성·구동 게인·물리 Hz는 이번 수정에서 바꾸지 않았다.
활성 식물 1,737개 물리 관련 prim 비교가 일치하고, 인스턴스 포함 활성 충돌체는 31,464개로 같다.

관련 자동 테스트 37개 PASS. 원자료는 각 trial의 `plan.json`, `states.npz`, `contacts.json`,
`self_collision_preflight.json`, `self_collision_actual.json`, `summary.json`이다.
요약·입력/코드 해시는 `validation/self_collision_fix_20261010.json`에 보존한다.
잘못된 첫 두 실행도 삭제하지 않았다. 사후 실제 자세 검사에서 NPZ를 반복 해제하던 비용을 제거했고,
같은 원자료에서 최적화 전후 검사 JSON이 정확히 일치했다.

재실행:

```bash
bash /root/docker_share/mujoko_debugging_data/20261010_142120_s300_self_collision_fix/run_gui.sh
```

대형 결과 폴더는 Git에 포함하지 않는다. 위 로컬 실행 스크립트 없이 실행하려면 다음과 같다.

```bash
cd /root/farmily_tomato
motion_out="/root/docker_share/mujoko_debugging_data/$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_s300_self_clear_gui"
DISPLAY=:0 OMNI_KIT_ALLOW_ROOT=1 /root/isaac-sim-6.1.0/python.sh --no-ros-env -u \
  nvidia-sim/harvest_s300/run_motion.py \
  /root/docker_share/mujoko_debugging_data/20261010_142120_s300_self_collision_fix/assembly_v3 \
  --output "$motion_out" --mode harvest --num-envs 1 \
  --preparation horizontal_ready --entry-only \
  --approach-pitch-deg 0 --approach-yaw-offset-deg 30 \
  --device cpu --hz 120 --threads 2 --performance optimized --display-fps 30 \
  --gravity-neutral-warmup --armature-setup authored \
  --buffer-contact-events --record-contact-frames \
  --contact-backend callback --collision-filter pairs --plant-dynamics full \
  --solver PGS --position-iterations 64 --velocity-iterations 4
```

최종 assembly를 다시 만들 때에는 원래 metre/Z/kg 기준 assembly와 CAD 자산을 보존해야 한다.
기존 폴더를 덮어쓰지 않고 새 경로를 지정한다.

```bash
./nvidia-sim/.asset-venv/bin/python nvidia-sim/harvest_s300/motion_assets.py \
  /root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_motion_assembly_metre_z_rebuilt \
  --output /새로운/출력/assembly \
  --horizontal-ready-revision --ready-distance .16 --ready-yaw 30 --ready-seed 21 \
  --target Truss_02_Fruit_11
```

실행 전 검사 → 새 물리 실행 → 실제 자세 검사 순서이며, 각 실행은 새 날짜 폴더에 저장한다.
화면은 팔꿈치와 그리퍼를 함께 보이도록 넓혔다. GUI 약 0.247배속, headless 약 0.608배속은
이번 1환경 측정이며 장면 로딩·사전/사후 검사·저장·영상 렌더는 별도다.
이번 MP4는 저장된 실제 GUI pose의 1배속 재생이다. 물리 재실행이나 명령 애니메이션이 아니다.

호스트에서 열기:

```text
file:///home/rdv/docker_share/mujoko_debugging_data/20261010_142120_s300_self_collision_fix/index.html
```

한계: 이번 완료는 열린 그리퍼의 진입·유지·복귀다. 닫기/벨트 이송/분리 수확은 검증하지 않았다.
연속 swept 충돌 검증이나 GPU/다중 환경에 대한 검증으로 확대하지 않는다.
`harvest_success=null`, `training_eligible=false` 유지. 아래 과거 전체 동작의 닫기 실패도 여전히 미해결이다.

## 후속 정정: 그리퍼–팔 자기관통 확인

사용자의 영상 지적을 실제 저장 body pose로 재검사했다. 로봇 articulation의
`physxArticulation:enabledSelfCollisions=false`였으며, 대기·준비·복귀 자세에서
그리퍼와 RB5 `link2`의 원본 충돌 메시 삼각형이 실제로 교차했다.
대표 부품은 `A01_rear_fixture_adapter`, `F01_rear_rail_backbone`, `F02_MGN12R_195`다.
실제 2초 대기 자세에서 24개 부품 쌍, 4초 준비 자세에서 28개 쌍의 교차를 확인했다.
이는 접촉력/침투 깊이 측정값이 아니라 원본 삼각형 교차 개수다.

따라서 아래 **23.5초 완주는 명령 실행 완료일 뿐, 충돌 없는 유효한 진입·복귀 경로가 아니다.**
기존 native 접촉 감시는 비활성화된 자기충돌을 포착하지 못했다.
수평 FK와 28개 테스트도 자기충돌 검증을 포함하지 않았다. 앞선 물리 유효성 해석을 정정한다.
현재 동작의 재실행 명령은 문제 재현용이며 정상 경로로 권장하지 않는다.

12개 저장 시점의 실제 body pose + FCL 원본 메시 삼각형 교차 검사다. 장착부 `link6`는 제외했다.
9.5/11.5/13.5초의 검사 표본에서는 해당 교차가 없었지만 연속 전체 경로의 비충돌 보증은 아니다.
원본 상태·접촉 로그·USD는 그대로 보존했다. 자기충돌 설정과 초기 자세/IK 경로의 수정·재검증은 아직 하지 않았다.
근거: `/root/docker_share/mujoko_debugging_data/20261010_141508_s300_self_collision_audit/mesh_intersection_audit.json`.

## 실제 확인된 결과

**GUI에서 대기 → 개방 → 수평 접근/진입 → 2초 유지 → 후퇴 → 대기 자세 복귀를 완료했다.**
타깃은 기존 활성 송이의 바깥쪽 `Truss_02_Fruit_11`이며, 식물 위치를 옮기지 않았다.
진입 4초/480개 실제 물리 표본에서 그리퍼 입구의 상하 기울기는 최대 **0.0171°**였다.
전체 동작의 상하 기울기 최대 0.0402°, TCP 높이 변화 0.256mm, 목표 과실 이동 최대 0.0316mm.
당시 활성 접촉 검사에서는 중단되지 않았고, 실제 timestamp 간격은 최대 1/120초였다.
위 후속 검사에서 자기관통이 확인됐으므로 초기/전체 물리 유효성 통과로 해석하지 않는다.

이 완료 실행은 `--entry-only` 진입·복귀 시연이다. 과실을 닫아 잡거나 벨트로 이송하는 수확 시도는 하지 않는다.
같은 11번 열매의 별도 전체 동작 시험에서는 진입을 완료했지만 **13.233초, 닫기 단계**에서
왼쪽 상판(`L_C02_upper_deck`, `part_127`)이 옆 8번 과실에 닿아
1.522mm 관통으로 중단됐다. 그 접촉점의 impulse/dt 크기는 0.871N이었다.
이 값은 손상 한계가 아니며, 수확·분리는 아직 확인하지 못했다.
모든 결과는 `training_eligible=false`, `harvest_success=null`이다.

## 영상에서 적용한 동작

참고: [inaho 영상 49~69초](https://www.youtube.com/watch?v=-V8sVICfdxg&t=49).
해당 20초 구간을 로컬로 확보하고 시간별 프레임을 확인했다.
그리퍼를 세우고 입구를 옆으로 향하게 한 대기 자세, 몸체 가까운 접힌 팔,
개방 후 수평으로 뻗는 접근을 RB5에 적용했다. SCARA 관절 배치를 그대로 복제한 것은 아니다.

- `horizontal_ready`: 타깃 접촉 목표보다 300mm 뒤에서, 입구를 수평으로 유지한 채 시작한다.
- 초기 관절값을 새 USD의 `JointStateAPI`와 구동 목표로 저장한다. 실제 시작 관절과 FK를 확인하며 실행 중 순간이동하지 않는다.
- 멀리 떨어진 대기 위치에서 기존 CAD/슬라이더 최대 폭 **96mm**까지 먼저 연다.
- 수평 준비 이동 200mm → 마지막 진입 100mm. 위치·회전을 가진 Cartesian waypoint를 IK로 연결한다.
- 기본 pitch 0°. yaw +30°는 로봇→타깃 방향 기준 월드 Z축 주위의 수평 방향 보정이다.
- `--entry-only`에서는 진입 뒤 2초 유지하고 같은 경로로 돌아온 다음 대기 위치에서 닫는다.
- `--entry-only`를 빼면 기존 close/infeed/twist/withdraw 실행을 사용한다.

## 제거한 파츠와 보존한 환경

사용자가 지적한 받침은 `L_T01_sloped_receiver`다. 전용 볼트/너트 4개와 함께
`/World/ActuatedTool/L/part_234`~`part_238`의 외형·충돌체를 제거했다.
CAD 부피×재질 밀도 추정 질량 **167.664g**도 왼쪽 바디에서 차감했다.
왼쪽 바디 질량은 0.854585→0.686921kg이며, 실측 질량이라고 주장하지 않는다.

전체 활성 충돌체는 31,469→31,464개로 정확히 이 5개만 줄었다.
활성 식물 1,740 prim의 속성과 관계를 원본과 대조해 차이가 없음을 확인했다.
원본 CAD/기존 실험은 유지하고 새 assembly에 수정했다.
나머지 농장 충돌체, full 식물 612 DOF, CPU PGS120/64/4, 마찰·강성·actuator·관통 기준은 유지한다.

## 실패를 포함한 시험

| 실행 | 결과 |
|---|---|
| 추가 sublayer 방식의 초기 revision 및 분리 대조 4회 | 0.0167초에 식물 내부 접촉/큰 변형으로 중단. 정상 물리 결과로 채택하지 않음 |
| 원본 assembly idle 대조 | 0.2초 정상. 위 문제와 구분 |
| 원본 root layer를 복사한 revision, 02번, 늦은 개방 | 수평 준비 후 6.542초에 앞쪽 04번 과실 접촉·관통 |
| 02번, 대기 위치에서 60mm 먼저 개방 | 진입 중 8.342초에 03번 과실 접촉·관통 |
| 02번, 대기 위치에서 96mm 개방 | 진입 중 8.742초에 잎 접촉·관통 |
| 바깥쪽 11번, 96mm 개방, 전체 동작 | 진입 완료. 닫기 13.233초에 08번 과실 접촉·관통 |
| 같은 11번, GUI 진입·복귀 시연 | **23.5초 전체 완료**, 관통 중단 없음 |

추가 sublayer로 감싼 revision은 선언상 식물 속성이 같아도 native 초기 실행이 달랐다.
기존 adapter root layer를 새 파일로 복사해 수정하는 방식으로 해결했다.
엔진 내부 원인을 확정한 것은 아니며 실패한 입력/로그도 보존했다.
02번에서 11번으로 타깃을 바꾼 것을 같은 타깃의 성능 개선으로 집계하지 않는다.

GUI와 headless 전체 동작의 공통 구간 0~11.5초/1,381표본에서 실제 body pose, 관절값,
명령, phase, timestamp가 bit-exact였다. 후반 동작은 유지·복귀와 닫기로 다르다.

## 실행 속도와 영상

- GUI 진입·복귀: 23.5 sim초 / 92.714 실제초 = **0.253배속**. 로딩·저장 제외.
- 별도 headless 전체 동작: 13.233 sim초 / 21.234 실제초 = 0.623배속. 후반 경로가 달라 순수 GUI 비용 비율로 계산하지 않는다.
- GUI의 30FPS는 갱신 상한이다. 시뮬레이션이 실시간으로 진행한다는 뜻이 아니다.
- 결과 MP4는 **실제 저장 상태의 1배속 재생**이다. 물리를 다시 실행하지 않는다. 재생용 보조 조명은 물리에 영향을 주지 않는다.

## 이전 실행 명령 (자기관통 수정 전 기록)

아래 300mm/seed3와 이전 `run_gui.sh`는 당시 재현용 기록이다. 현재 검증본은 맨 위의
160mm/seed21, `assembly_v3` 명령을 사용한다. 이전 자기충돌 비활성 모델은 현재 실행기가 거부한다.

단일 GUI에서 진입·유지·복귀 시연:

```bash
bash /root/docker_share/mujoko_debugging_data/20261010_134811_s300_video_entry/run_gui.sh
```

같은 타깃의 닫기·벨트 동작도 시도하려면 마지막에 `harvest`를 붙인다.
이는 위에서 옆 과실 접촉으로 중단됐던 시험이다.
실행마다 KST timestamp를 가진 새 output에 상태/접촉/summary를 저장한다.
터미널에는 sim초, 실제 경과초, 실시간 대비 배속을 출력한다.

수정 assembly를 재생성하는 기존 도구 명령:

```bash
cd /root/farmily_tomato
./nvidia-sim/.asset-venv/bin/python nvidia-sim/harvest_s300/motion_assets.py \
  /root/docker_share/mujoko_debugging_data/20261010_s300_performance/legal_motion_assembly_metre_z_rebuilt \
  --output /새로운/출력/assembly \
  --horizontal-ready-revision --ready-distance .30 --ready-yaw 30 --ready-seed 3 \
  --target Truss_02_Fruit_11
```

코드는 기존 assembly 생성기, ToolKinematics/IK, `run_motion.py`, renderer/report를 확장했다.
새 수집/학습 파이프라인은 추가하지 않았다. 기존/새 준비 경로 검사 **28개 통과**.

결과 위치: `/root/docker_share/mujoko_debugging_data/20261010_134811_s300_video_entry/`

- [전체 결과·실제 영상](/root/docker_share/mujoko_debugging_data/20261010_134811_s300_video_entry/index.html)
- [수치·전체 시도·입력 검증](/root/docker_share/mujoko_debugging_data/20261010_134811_s300_video_entry/video_entry_validation.json)
- [실제 GUI 화면](/root/docker_share/mujoko_debugging_data/20261010_134811_s300_video_entry/gui_live_check.png)

호스트 브라우저 주소:
`file:///home/rdv/docker_share/mujoko_debugging_data/20261010_134811_s300_video_entry/index.html`
