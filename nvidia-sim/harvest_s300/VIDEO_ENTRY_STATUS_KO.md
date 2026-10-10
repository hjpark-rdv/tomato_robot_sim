# S300 · RB5 영상 기준 대기 자세와 수평 진입

2026-10-10 KST. 브랜치 `codex/s300-harvest-isaac-20261009`, 작업 기준 `aad8340`.
이 문서의 수정은 이전 수평 진입 작업 위에 추가했다. 코드·테스트·문서·검증 요약을 함께 커밋한다.
커밋은 `git log -- nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md`로 확인한다.
검증 JSON의 `uncommitted=true`는 실험 당시 코드 상태이며, 이 기록을 사후에 커밋한 것과 구분한다.

## 최신 수정: 자기충돌을 켠 진입·유지·복귀 재검증

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
