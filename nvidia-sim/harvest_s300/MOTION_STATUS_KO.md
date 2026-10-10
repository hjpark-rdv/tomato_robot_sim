# S300 · RB5 · V5.1 수확 구동 연결 및 실제 물리 시험

**후속 정정: 아래 최초 실행의 assembly root는 단위·up-axis metadata 누락으로 .01m/Y fallback을 사용했다. 시간·상태·접촉 기록은 보존하지만 SI 거리·힘·올바른 농장 물리의 최종 검증으로 해석하지 않는다.**
source의 metre/Z/kg 전파와 native 사전 검사를 수정했고, 새로 재생성한 `legal_motion_assembly_metre_z_rebuilt`의 full SI 부하·접촉·병렬 결과는 [PERFORMANCE_STATUS_KO.md](PERFORMANCE_STATUS_KO.md)에 따로 기록한다.
metadata-only wrapper의 실패와 재생성 모델의 통과가 달라 composition/native parsing 원인은 아직 확인 중이다.


후속 정상 배치·최소 app·CPU24환경·실제 접근 비교는 [PERFORMANCE_STATUS_KO.md](PERFORMANCE_STATUS_KO.md)에 있다. 아래는 첫 동작 연결의 당시 기록이다.

2026-10-10 KST. 브랜치 `codex/s300-harvest-isaac-20261009`, 기준 HEAD `b890fc6`.
이 문서는 최초 구동 연결 시점의 구현과 서버 실험을 보존한 기록이다. 기존 MuJoCo 실험과 기존 Isaac 조립 자산은 보존했다.

**팔 접근·그리퍼 개폐·벨트 표면 구동·식물 탄성 연결은 구현했지만, 정상 농장에서의 과실 포획·분리 수확은 아직 달성하지 못했다.**
현재 문제는 실행 코드만의 문제가 아니다. 원래 촘촘한 장면의 초기 잎 겹침, 접근 중 잎/유인선 충돌,
과실 크기와 그리퍼 최소 개구폭의 차이가 실제 시험에서 드러났다.

## 바로 확인할 결과

- 서버 결과: `/root/docker_share/mujoko_debugging_data/20261010_s300_harvest_motion/index.html`
- 호스트 결과: `/home/rdv/docker_share/mujoko_debugging_data/20261010_s300_harvest_motion/index.html`
- 최신 동작 모델: 같은 폴더 `assembly_v5/scene.usda`.
- 실제 물리 상태: 각 시험의 `states.npz`, `contacts.json`, `break_events.json`.
- 영상: `component_motion_v3`, `component_jaw_v1`, `component_pulse_v1`의 `actual_motion_verified.mp4`.
  실제 측정한 모든 바디 pose와 관절 상태로 재생하며 물리를 다시 계산하지 않는다. 재생용 보조 조명을 썼다.
- 수치: `results.json`, `motion_metrics.png`. 실패/중단/실행 오류 기록도 삭제하지 않았다.

## 무엇을 연결했나

- 원본 V5.1 그리퍼 356부품 외형 유지. 좌우 슬라이더는 원래 크랭크/링크에 해당하는 등가 구동이며,
  실제 측정된 개구폭으로 크랭크와 링크의 표시 자세를 계산한다. 전체 기계 전달계의 동적 검증은 아니다.
- CAD 최소/최대 개구폭 **28~96mm 유지**. 이번 실제 개폐 시험 범위는 **28.02~60.01mm**다.
- 두 벨트에 PhysX surface velocity 적용. 같은 방향 이송과 반대 방향 비틀기 명령을 분리해서 저장한다.
  벨트 자체가 순환하는 유연체 모델은 아니며, 이번 무부하 명령 시험은 과실을 실제로 운반했다는 증거가 아니다.
- 슬라이더1500N/m·감쇠30·최대10N은 이번 등가 구동 설정이며 실측 보정값이 아니다.
  벨트 모터 토크 한계/전달 손실은 아직 모델링하지 않았다.
- CAD 부품 부피/재료 밀도로 그리퍼 질량을 추정(합계 약2.85kg). 벨트 질량 이중 합산을 제거했고,
  링크류는 고정부 등가 질량에 포함했다. motor/switch 밀도는 임시값이며 실측 하중/관성 보정 전이다.
- Plant_22의 주줄기와 송이3개를 기존 full `ElasticPlant`의 분절/강성/감쇠/중력 preload 방식으로 연결했다.
  식물235바디/612자유도, 로봇/도구 포함248바디. 주줄기100, 송이 줄기25, 중심가지2, 꼭지0.25Nm/rad.
- 기존 native breakable joint의 3N/0.08Nm 설정 유지. 과실 분리는 물리 joint break로만 기록하며
  시간에 맞춰 강제로 분리하거나 그리퍼에 과실을 붙이는 가짜 grasp joint를 만들지 않았다.
- 하우스0.5, 줄기68개, 송이106개의 원래 시각 메시 유지. 다른 식물은 원래 고정 장애물이다.
  전체 농장을 경량 mesh로 바꾸지 않았다. 활성 식물의 충돌은 기존 full 모델처럼 rod/과실/잎 proxy를 사용한다.

## 실행에서 수정한 문제

1. Isaac6.1 native tensor의 CPU armature 인자 요구와 GPU 접촉 readback 연결을 수정했다.
   초기의 접촉0 기록(`idle_v3/v4`)은 접촉이 없다는 증거가 아니므로 무효 진단으로 표시했다.
2. rod의 COM·관성을 명시하고 PGS/240Hz/armature0.0005를 초기화 전에 적용했다.
   초기 TGS/자동 관성 시도의 NaN 결과는 실패 기록으로 남겼다. 안정한 두 엔진의 성능 비교가 아니다.
3. GPU broad-phase aggregate pair 용량 부족을16384로 늘렸다. 충돌을 생략하는 설정이 아니다.
4. 복원된 S300 뒷판과0.95m 리프트 선반이 원래 장착 위치에서 교차했다. 원래 뒷판을 유지한 채
   **새 동작 모델에서만 리프트·RB5 전체를 월드Y−0.20m 이동**했다. 기존 USD/원본 CAD는 수정하지 않았다.
   기계 체결/강성/무게중심 설계 검증을 끝냈다는 뜻은 아니다.
5. 곧바로 송이 쪽으로 회전하는 준비 경로가 거터 지지선/유인선에 부딪혔다.
   통로에서 팔을 먼저 접고 방향을 맞춘 다음 회전하도록 바꿨다. 실제 관절 명령과 FK가 달라진다.

## 원래 농장과 분리 시험의 차이

최종 `farm_idle_final`은 assembly_v5에서 배경 충돌을 그대로 켠 CPU 시험이다.
로봇이 움직이기 전부터 활성 잎과 Plant_03의 잎 등이 겹쳤다. 초기화 접촉을 포함한 최대 관통은
**12.22mm**였고 첫 기록 스텝에서0.5mm 기준으로 중단했다. 이전 GPU 진단 `idle_v5`는14.07mm였다.
서로 다른 모델/엔진 조건이 섞이므로 이 두 숫자를 정밀 CPU/GPU 오차로 해석하지 않는다.

`component_*`는 명시적인 **구동 분리 시험**이다. 다음을 제외했다:

- 배경 식물28,971개 충돌체와 활성 식물/로봇/도구 사이 접촉.
- 활성 주줄기의 잎 proxy와 활성 식물의 다른 충돌체 사이 접촉(초기 자체 겹침).

제외한 정확한 경로는 `fixture_disabled_background_colliders.json`, `fixture_internal_leaf_filters.json`에 저장했다.
로봇/도구↔활성 잎·과실·가지, 하우스·거터·지지선·유인선 접촉은 유지했다.
**이 시험의 완료를 원래 농장 수확 성공이나 초기 충돌 해결로 바꾸지 않는다.**

## 실제 결과

| 시험 | 실제 sim 시간 | 결과 | 핵심 측정 |
|---|---:|---|---|
| 최종 원래 농장 정지 | 0.0042초 | 초기 겹침으로 중단 | 초기화 포함 최대 관통12.22mm |
| 분리 개폐/벨트 명령 | 7초 | 완료 | 간격28.02~60.01mm, 관통 없음, 파단0 |
| 분리 탄성 load/release | 7초 | 완료 | +X0.1N을1~2초에 가함. 과실 최대7.18mm, 종료0.43mm |
| 접근 v1 | 4.85초 | 준비 중단 | 왼쪽 그리퍼↔거터 지지선, 관통0.581mm |
| 접근 v2 | 5.02초 | 준비 중단 | 왼쪽 그리퍼↔세로 유인선, 관통0.529mm |
| 접고 회전하는 접근 v3 | 14.73초 | 준비 회전 중단 | 오른쪽 그리퍼↔주줄기 잎, 관통0.507mm |

개폐 시험의 과실 변위는 초기 과도 응답까지 포함하면 최대3.62mm, 안정화 후 종료0.43mm다.
탄성 시험은 힘 제거 후 정지 대조군 수준으로 돌아왔다. 외력은 과실 바디에 직접 가한 native world force이며
로봇으로 눌러 같은 힘을 만들었다는 뜻은 아니다.

v3에서 유인선과의 충돌은 피했지만,14.375초부터 잎 접촉이 시작되어 식물이 밀렸다.
목표 과실의 직접 접촉력은0이지만 과실은62.85mm 이동했다. 잎에서 줄기/송이로 힘이 전달된 결과다.
해당 잎 접촉점에서 관측한 최대 힘 크기는10.76N이다. 마지막 중단 스텝의0.795N과 최대값은 다르다.
이 수치는 **한 접촉점의 impulse 벡터 크기 / dt**이며, 과실의 힘 합이나 손상 한계가 아니다.

v3의 최대 관절 추종 오차는 리프트13.04mm, 회전 관절 최대0.02525rad였다.
잎에 밀린 결과를 명목 IK 도달로 설명하지 않는다. 최종 접근/닫기/과실 이송/비틀기/후퇴 단계는
경로에는 들어 있지만 정상 시작 물리 시험에서는 아직 그 단계까지 도달하지 못했다.

물리/읽기/결과 저장 시간(앱·장면 준비와 렌더 제외): 개폐22.42초/7sim초,
탄성22.94초/7sim초, v3접근47.86초/14.73sim초. 각1회 CPU 측정이며 약0.31배 실시간이다.
접촉 callback과 매 스텝 상태 저장이 포함된 진단 실행이다. GPU 병렬 속도나 기존 MuJoCo 대비 개선은 미검증이다.

## 남은 중요한 한계

- 선택 과실 `Truss_02_Fruit_02` 외곽 크기는26.09×26.87×26.22mm다.
  그리퍼 최소 간격28mm와 맞지 않아 양쪽 벨트로 과실을 집는 조건을 아직 확보하지 못했다.
  원래 사용자 승인 송이0.5배와 CAD 간격을 임의 변경하지 않았다. 실제 과실 치수/개구 설계를 맞춰야 한다.
- 원본 RB5 USD의 articulation 자기충돌 설정은false인 채 유지된다. 따라서 이번 동작은
  로봇 내부 자기충돌까지 검증된 경로가 아니다. 새 도구 외형을 포함한 자기충돌 검사 연결이 추가로 필요하다.
- 이 FK/IK 연결은 새 CAD endpoint를 읽는 진단용이다. 전체 경로의 FCL/OMPL 안전 인증을 구현한 것이 아니며,
  실제 physics 접촉에서 중단한다. 무한히 접촉 trial을 늘리지 않고 실제 차단 물체를 기록했다.
- 원래 농장에서는 초기 잎 겹침을 먼저 해소해야 한다. 보이지 않게 충돌을 끄거나 접촉 허용치를 올려서
  full 농장 성공으로 만드는 방식을 채택하지 않았다.
- 동작 중 live 표시와 저장 상태 렌더는 같은 `motion_visuals.py` skin/크랭크 계산을 사용한다.
  최종 headless 실험과 영상은 확인했으나 GUI 유무의 완전한 수치 동등성은 별도 미검증이다.
- 최종 시험 로그에는 실행을 막는 native 오류는 없었지만, Fabric의 filteredPairs 속성 경고와
  드라이버R580 미만에 따른 렌더 denoiser fallback 등이 남아 있다. 로그를 보존했다.
- `training_eligible=false`, `harvest_success=null`. 새 학습 데이터에 섞지 않았다.

## 재현

기존 결과를 덮어쓰지 않도록 항상 새 output을 쓴다. 표시하려면 `--headless`만 뺀다.
표시 갱신은30Hz이고 물리는240Hz다. 아래는 **분리 구동 시험** 실행 예다.

```bash
cd /root/farmily_tomato
motion_out="/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_s300_motion_check"
DISPLAY=:0 OMNI_KIT_ALLOW_ROOT=1 /root/isaac-sim-6.1.0/python.sh --no-ros-env -u \
  nvidia-sim/harvest_s300/run_motion.py \
  /root/docker_share/mujoko_debugging_data/20261010_s300_harvest_motion/assembly_v5 \
  --output "$motion_out" --mode harvest --device cpu --isolate-background --headless
```

`--mode jaw`는 개폐/무부하 벨트, `--mode pulse`는0.1N load/release다.
`--isolate-background`를 빼면 원래 농장 충돌을 포함한다. 현재는 초기 겹침으로 중단되는 것이 확인됐다.
`--preparation direct`는 이전 직접 준비 경로이며 기본은`fold_then_turn`이다.

```bash
DISPLAY=:0 OMNI_KIT_ALLOW_ROOT=1 /root/isaac-sim-6.1.0/python.sh --no-ros-env -u \
  nvidia-sim/harvest_s300/render_motion.py \
  /root/docker_share/mujoko_debugging_data/20261010_s300_harvest_motion/assembly_v5 \
  "$motion_out" --camera motion --review-light
```

저장물에는 immutable assembly SHA, 코드 사본/해시, 명목 phase별 q 경유점, 실제 q와 body pose,
매 스텝 timestamp/phase/contact가 있다. 기록 시각0은 native 초기화 후 기준이며, 초기화 과정의
contact callback은 시각0으로 묶여 있다. 초기화 두 내부 스텝의 개별 pose는 별도 저장하지 않았다.

## 검사

- `validate_motion_assets.py`: 원본 SHA, 모든 도구/활성 식물 정점·면 수, 이전 고정 모델 비활성화,
  CAD 개구 제한, 가짜 grasp joint 없음, native 파단값, zero-q FK 일치 **10/10 통과**.
- `test_motion_kinematics.py`: 명목 자세,28~96mm 전체 범위의25mm 크랭크/45mm 링크 핀 일치,
  범위 밖 거부, 그리퍼 기울기별 과실 국소 위치 보존 **4/4 통과**.
- 원래 농장 실패와 분리 시험의 실제 물리 결과는 위 표대로 분리한다. 기하 검사 통과가 수확 성공은 아니다.
- MP4 전체 디코드/프레임 수/최종 physics 상태 포함을 검사했다. Kit의 PNG 비동기 쓰기가 끝나기 전에
  인코딩해 마지막1프레임을 놓친 초기 영상을 보존하고, 완료 PNG로 재인코딩했다. 렌더러에 PNG 종료 chunk 대기와 프레임 수 검사를 추가했다.

다음 작업은 같은 마지막 자세를 반복 조정하기보다 **실물 과실–그리퍼 간격 확정 → 초기 식물 겹침 정리 →
새 그리퍼 전체 외형을 포함한 자기/환경 충돌 계획 → 실제 포획·이송·분리 시험** 순서가 필요하다.
