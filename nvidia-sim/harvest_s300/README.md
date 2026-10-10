# S300 수확로봇 · Isaac Sim 6.1 조립 장면

기준 브랜치: `codex/s300-harvest-isaac-20261009` (시작점 `b890fc6`).
기존 S300 CAD, 농장 USD, RB5 USD, 그리퍼 CAD는 수정하지 않는다.

**최신 여러 열매 접촉 시험:** 기존 V5.1은 수확 순서 완주/분리0.
별도5mm 벨트 접촉면 시험안+감속+scene CCD에서 Truss_02_Fruit_10 분리와 개방·후퇴를 확인했다.
원본 CAD의 성공이나 실물 검증이 아니다. [최신 결과·한계·실행법](VIDEO_ENTRY_STATUS_KO.md)을 참고한다.

**새 수확 순서 진단:** `run_motion_live.sh ASSEMBLY reference belt`는 선개방 진입 → 닫기 →
반대 방향 벨트2초 → 개방·후퇴를 계획한다. 실제120/240Hz에서는 닫기/벨트 시작에서
접촉 관통으로 중단됐고 분리0이다. 타깃 폭23.05mm와 최소간격28mm 불일치 및 옆 과실 상판
간섭을 확인했다. 근거/영상/명령은 [벨트 수확 진단](VIDEO_ENTRY_STATUS_KO.md)의 최신 절에 있다.

**1환경 빠른 확인:** `run_motion_live.sh ASSEMBLY interactive`는 기존 native 실행기와
실제 상태 뷰어를 별도 프로세스로 연결한다. full 식물/전체충돌을 유지한 PGS60에서
23.5초 진입·복귀를23.500초에 완료했다. 표시15fps 상한과 물리60Hz는 별개다.
120Hz와 수치 응답이 달라 부하 최대 변위가 약7.23% 증가했으며 수확 검증용 기본값은 바꾸지 않았다.
`reference`(생략 기본값)는120Hz, 이번 GUI0.582배속. 명령과 결과는
[실시간 미리보기 기록](VIDEO_ENTRY_STATUS_KO.md)의 첫 절을 참고한다.

최신 팔–그리퍼 자기관통 수정은 [VIDEO_ENTRY_STATUS_KO.md](VIDEO_ENTRY_STATUS_KO.md)의 맨 위 절을 따른다.
자기충돌 활성화·슬라이더 부모 수정·새 대기 IK를 적용한 `20261010_142120_s300_self_collision_fix/assembly_v3`에서
진입·유지·복귀 headless/GUI 모두 완료, 각 2,821개 실제 자세 교차 검사와 native 자기접촉 0건을 확인했다.
관련 37테스트 PASS. 열린 그리퍼의 진입 시연이며 과실 포획·분리 성공은 아니다. 이전 비활성 자기충돌 모델로 harvest를 실행하면 명시적으로 거부한다.

최신 속도·병렬·실제 동작 결과는 [PERFORMANCE_STATUS_KO.md](PERFORMANCE_STATUS_KO.md)에 있다.
첫 118개 trial의 assembly root에 단위·축이 빠져 0.01m/unit·Y-up이 적용됐다. 당시 시간과 재현성 기록은 보존하되 올바른 SI 물리 검증에서 제외한다.
새 metre/Z/kg 모델은 원본 형상과 목표 식물 full 탄성 612자유도·주변 전체 충돌을 유지한다. PGS64/120Hz의 7초 부하 시험은 단일 CPU에서 10.525초(0.665sim-s/s), 독립 CPU 24worker에서 합산 7.019sim-s/s다.
234자유도/60Hz/TGS 설정은 더 빠른 별도 진단이며 full 모델과 동등하지 않다. 실제 수확은 접근 중 0.5mm 관통 기준을 초과해 중단됐고, 과실 포획·분리 수확은 아직 검증되지 않았다.
이전 동작 기록은 [MOTION_STATUS_KO.md](MOTION_STATUS_KO.md)에 보존한다. 아래는 기반 조립 장면의 당시 기록이다.

## 구성

| 구성 | 사용한 자산 / 처리 |
|---|---|
| 하우스 | 기존 v2 USD의 구조물 형상·재질 유지, 옛 54식물은 비활성화 |
| 식물 | 검토된 v8 기존형 36개 + `tomato_stem_leaves_v9.glb` 32개. 기존형 색상을 v9에 맞춤 |
| 송이 | 기존 5종 GLB, 106송이, 각 10열매. Fruit06 제외 및 white 보정, GLB Y만 0~180° 회전 |
| 해상도 | 줄기 v8 387,329 / v9 524,809 삼각형, 송이별 190,095 삼각형. 포함된 원본 면·정점 수 유지 |
| 농장 크기 | 기존 사용자 장면과 같은 하우스 0.5. 식물 뿌리/송이 부착 세계 좌표도 검토된 배치 그대로 복원. 줄기·송이 자체 크기 유지 |
| 농장 충돌 | 표시 형상의 full triangle 고정 충돌. 최종 활성 인스턴스 30,728개. 식물 탄성 연결 전 |
| S300 | 기존 웹 뷰어의 **V10 현재 조립 외형**을 읽기 전용 스냅샷으로 취득 |
| 제거 | 물통·펌프·분무 노즐/마스트 및 후방 상부 지지물. 정확한 부품/이유는 manifest에 저장 |
| 뒷판·휠 커버 | 리프트 뒤 원본 `T_06. Rear_plate_V2NR.SLDPRT` 복원. 메카넘 휠 커버와 전용 지지대/체결부 제거, 바퀴 본체 유지 |
| 장착판 | 평평한 570 × 1,170 × 12mm 배치용 판. 설계 하중/강도/체결 검증 전 |
| 리프트·팔 | 기존 `rb5_farmily.usd`의 리프트 1축 + RB5 6축과 질량/드라이브 재사용 |
| 그리퍼 | V5.1 GLB의 356부품, 44mm 개방 조립 상태. 기존 고리·카메라 외형 제거 |
| 그리퍼 부착 | CAD 후면 어댑터 중심을 TCP 밖 12mm 지점에 정렬. 임시 스페이서 사용 |

S300은 이 단계에서 고정 베이스다. 주행 동역학을 추가하지 않았다.
V5.1의 후면 어댑터는 범용 커스텀 치수이며 RB5 볼트 규격과의 기계적 호환성이 검증된 부품이 아니다.
새 그리퍼의 질량/관성도 아직 보정하지 않았으므로 기존 팔의 동적 성능과 동등하다고 해석하지 않는다.

원본 농장 USD에는 물리 스키마가 없었다. 이번 고정 충돌 추가는 탄성 식물을 경량화한 것이 아니며,
기존 `rl/elastic_plant.py`의 **full + main appendage keep** 탄성 모델은 다음 연결 대상이다.
그 연결·접촉·복원 검증 전에는 `training_eligible=false`, 수확/휘어짐 검증 미완료 상태를 유지한다.

## 설치

이 서버에는 Isaac Sim 6.1을 `/root/isaac-sim-6.1.0`에 별도로 설치했다.
기존 `/root/isaaclab_env`의 5.1은 보존했다. 공식 Linux standalone:

- 다운로드: https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.1.0-linux-x86_64.zip
- MD5: `b471383a51f259e0af22541f05c34b0e` (실제 다운로드 일치)
- 배포본 `VERSION`: `6.1.0-rc.26+release.49347.2d230af4.gl`
- Python 3.12.13. 기존 Python 3.11/Isaac Lab 환경의 패키지를 섞지 않는다.

현재 RTX 3090 Ti / 드라이버 570.211.01에서 실제 6.1 기동·원본 장면 렌더를 확인했다.
DLSS-RR Responsive Denoising은 R580 이상 필요 경고와 preset D fallback이 있었다.
공식 권장 드라이버/모든 기능 호환성을 충족했다고 해석하지 않는다.

## 이번 서버 확인 결과

### 하우스 크기 복원 (2026-10-10 후속, 현재 상태)

기존 `nvidia-sim/scenes/farmily_greenhouse_robot.usd`의 `/World/Greenhouse`가
실제로 `(0.5, 0.5, 0.5)` 배율임을 확인했다. 원본 메시 유지 요구를 공간 크기 1.0 유지로
잘못 적용했던 것을 수정했다. `--farm-scale` 기본값은 이제 0.5다.

- 하우스 외곽 bounds 크기(x/y/z): 6.541/10.271/5.277m → 3.270/5.135/2.639m.
  높이는 최저점 -0.045m~최고점 2.594m를 포함한 bounding box 값이다.
- 줄기 68개/송이106개 뿌리와 부착점 세계 좌표를 기존 저장 배치 그대로 복원했다.
  행 내 명목 뿌리 간격은 다시25cm, 거터 중심 x=±0.775m 및 양쪽 뿌리±0.10m다.
- 줄기·송이 template 형상은 직전 장면과 바이너리 해시까지 같으며 더 축소하지 않았다.
- S300·리프트·RB5·그리퍼는 크기를 유지한다. 통로 중앙으로 배치 위치 x=0.55→0m만 변경.
  816개 로봇 메시·법선·면 및 동일 상대 조립 좌표를 확인했다.
- 생성/기하 검사27개와 크기/보존 비교8개 통과. 원본 결과 및 카메라 수정 전 화면도 보존.
  지붕 프레임 아래 통로 시점으로 기본 카메라를 조정했다. 새 물리/충돌 없는 시작 검증은 아니다.
- 현재 장면: `/root/docker_share/mujoko_debugging_data/20261010_s300_restored_house_scale/assembly_final/scene.usda`.
  같은 폴더 상위 `index.html`, `scale_validation.json`, `runtime_final/`에 실제 Isaac 화면/검사 보존.

### 최신 식물 반영 (2026-10-10)

아래 1.0 하우스 배치 설명은 최초 식물 이식 기록이다. 현재 크기는 위 후속 수정의 0.5다.

`build_scene.py --farm-layout reviewed`가 새 기본값이다. 이전 구현은 사용자에게 검토받은
v9 잎/줄기 혼합 및 GLB 송이 배치를 옮기지 않고 옛 v2 농장을 불러온 문제가 있었다.
`--farm-layout legacy`는 이전 54식물 장면 재현용으로 남겼다.

결과: `/root/docker_share/mujoko_debugging_data/20261010_s300_latest_plants/index.html`.
최종 USD: 같은 폴더 `assembly_final/scene.usda`. 현재 Isaac 창에도 적용하고 변경 전후를
같은 카메라에서 촬영했다. `new_plants_close.png`, `paired_rows.png`도 저장했다.

- `latest_farm.py`: 기존 `view_glb_truss.read_glb`, `elastic_geometry`와 Git에 보존된
  `covered_scene_metadata.json`의 68개 배치를 재사용한다. 새 physics 모델 생성기는 아니다.
- v8 USD/신규 v9 GLB의 모든 줄기/잎 primitive를 유지한다. 원본에 편집을 가하지 않는다.
  v8의 USD 중복 이름/재질 해시 접미사는 v9 원래 객체명에 매핑해 색을 맞춘다.
- 송이는 원본 GLB에서 6번 열매·꽃받침만 기존 규칙대로 제외하며 꼭지 가지는 유지한다.
  white 열매 위치와 꼭지 보정도 기존 `build_glb_physics.py`와 같은 방식이다.
- GLB Y-up → 월드 Z-up 변환 후의 월드 Z 회전은 GLB Y 회전에 해당한다.
  식물 전체의 반대편 yaw와 송이 자체의 회전을 분리한다. 기존 부착점은 주줄기 표면과
  대조하며 USD 회전은 double precision으로 저장한다.
- 하우스 1.0에서 양측 거터 중심은 x=±1.55m, 뿌리 위치는 중심에서 ±0.20m이다.
  기존 0.5 하우스의 배치 뿌리 좌표만 2배로 변환했다. 따라서 같은 행의 명목 간격은
  25→50cm이고, 식물 크기/송이 크기를 2배로 늘리지는 않았다. 최신 MuJoCo 장면과
  세계 좌표/공간 밀도가 완전히 같다고 해석하지 않는다.
- 기하/자산/부착 검사 27개 통과. 원본 면 인덱스/정점 수 보존, v9 524,809개 면 확인.
  최대 float32 정점 저장 오차 5.96e-8m, 합성 회전/부착 좌표 오차 7.78e-16 수준.
- S300/리프트/RB5/그리퍼 보존 검사 5개 통과. 원본 뒷판 복원·휠 커버 제거도 유지한다.
  `latest_plants_validation.json`과 결과 폴더의 상세 JSON/코드/로그에 검증을 보존한다.
- 실패한 초기 변환/검사 시도는 `assembly`, `assembly_v2`, `assembly_v3`에 남겼다.
  최종 장면은 `assembly_final`이다. 이번에 물리 실행/속도 벤치마크는 수행하지 않았다.

현재 주변 식물은 고정된 전체 삼각형 충돌체다. 탄성·접촉·수확 검증은 후속 작업이며
이 외형 변경을 그 검증의 완료로 해석하지 않는다.

### 초기 조립 기준선 (2026-10-09, 옛 농장)

결과: `/root/docker_share/mujoko_debugging_data/20261009_s300_harvest_isaac61/index.html`

- 최종 조립: `assembly_v3/scene.usda`. 원본 보존/기하/부착 검사 13항목 통과.
- 실제 6.1에서 농장·로봇·그리퍼 3시점 PNG 저장: `runtime61/`.
- GPU PhysX 120Hz, 정지 2초/240스텝: `physics61_retry/`.
  실행·GPU 상태 읽기 포함 1.662초, link6 최대 이동 0.120mm, 보고된 접촉점 0.
- 캐시가 있는 재실행: 장면/앱 준비 21.03초, 물리 초기화 7.43초. 둘 다 실행 시간과 별도다.
- 첫 렌더는 120.05초 준비. 최초 물리 로딩은 약 118초가 들었고,
  이후 GPU Tensor를 CPU로 옮기지 않은 상태 조회 예외가 발생한 시도는 미완료로 보존했다.
  상태 조회를 수정하고 새 폴더에서 위 240스텝을 완료했다.
- Isaac 6.1의 `SimulationApp.close()`가 프로세스를 종료하므로,
  예외를 먼저 JSON/로그에 저장하고 비정상 exit code를 넘기도록 실행기도 수정했다.

이는 접촉 없는 정지 시험이다. **충돌·휘어짐·수확 상황의 성능이나 병렬 처리량을 측정한 결과가 아니다.**
새 그리퍼의 질량/관성 및 식물 탄성을 연결한 뒤 별도 기준선을 측정해야 한다.

## 장면 생성

오프라인 USD 도구 환경은 `nvidia-sim/.asset-venv`이다 (`usd-core`, `numpy`, `trimesh`, `scipy`, `pillow`).
자기충돌 검사에는 같은 환경의 `python-fcl==0.7.0.8`을 사용한다. 미설치 환경에서는
`./nvidia-sim/.asset-venv/bin/python -m pip install python-fcl==0.7.0.8`로 설치한다.
Isaac의 Python 3.12와 분리된 이 Python 3.11에서 계획/저장 상태를 검사한다.
S300 스냅샷 취득에는 기존 뷰어와 Playwright/Chromium이 필요하다.

```bash
cd /root/farmily_tomato
task_out="/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_s300_isaac61"
PYTHONPATH=/tmp/farmily-s300-tools/python python3 nvidia-sim/harvest_s300/capture_s300.py \
  --chromium /root/.cache/ms-playwright/chromium-1208/chrome-linux64/chrome \
  --output "$task_out/s300_v10_snapshot.json.gz"
./nvidia-sim/.asset-venv/bin/python nvidia-sim/harvest_s300/build_scene.py \
  --s300 "$task_out/s300_v10_snapshot.json.gz" --output "$task_out/assembly"
./nvidia-sim/.asset-venv/bin/python nvidia-sim/harvest_s300/validate_scene.py "$task_out/assembly"
```

`build_scene.py`는 완성된 scene을 덮어쓰지 않는다. 실패한 시도와 최종 결과도 별도 폴더로 보존한다.
`assembly_manifest.json`에 입력 SHA, 포함/제외 부품, 변환 및 검증 범위를 저장한다.
자산은 외부 원본을 참조하므로 다른 머신으로 옮길 때 원본 USD/GLB와 생성 폴더를 함께 보존해야 한다.

## 보기 / 짧은 물리 확인

2026-10-09 후속 로봇 외형 수정본은
`/root/docker_share/mujoko_debugging_data/20261009_s300_rear_panel_no_wheel_covers/assembly/scene.usda`다.
물통 주변 제거 범위에 잘못 포함됐던 원본 뒷판 1개를 복원하고, `feature=wheel-cover`인
좌우 덮개·전용 마운트/체결 부품 96개를 제외했다. S300 원본 메시 수 540 → 445.
남은 S300 메시와 농장 자산 보존 및 기하 검사 15항목 통과. 새 접촉/주행 물리 시험은 아니다.
같은 시점 전후 사진과 바퀴 사진: 위 결과 폴더 `index.html`.

```bash
cd /root/farmily_tomato
DISPLAY=:0 ./nvidia-sim/run_s300_harvest.sh \
  /root/docker_share/mujoko_debugging_data/20261010_s300_restored_house_scale/assembly_final/scene.usda \
  --output "/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_s300_view"
```

기본은 정지 장면을 띄운다. `--headless --seconds 2`를 붙이면 120Hz GPU PhysX의 짧은 실행을 시도한다.
`--camera /World/FarmCamera`는 농장 전체, 기본 `RobotCamera`는 로봇, `ToolCamera`는 새 그리퍼다.
그림 3장, runtime JSON, 물리 실행 시 실제 link6 위치와 접촉 로그를 출력한다.
GUI를 켰을 때는 확인 후 창을 닫으면 종료한다.
최신 식물 장면에는 `--camera /World/PlantCamera`(근접) 및
`--camera /World/PairedRowsCamera`(거터 양측)도 있다.

### VNC 화면 조작 (2026-10-09)

정지/일시정지 GUI는 기본 **30 FPS**로 갱신하며 우측 상단에 내부 FPS를 표시한다.
`--viewer-fps 60`으로 60 FPS, `--viewer-fps 0`으로 원래 무제한 갱신을 선택한다.
Play 중에는 원래 run-loop 설정으로 복귀한다. `--headless` 및 `--seconds` 물리 smoke에는
이 제한을 적용하지 않는다. 렌더링 화질·해상도·농장 형상·충돌체는 바꾸지 않았다.

실행 중 창에서 측정한 정지 화면은 약 161 FPS, 마우스 회전 표본 구간은 약 117 FPS였다.
사용자가 겪은 VNC 조작 지연이 해당 구간의 Isaac 렌더 FPS 저하로 재현되지는 않았다.
별도 5초씩 측정한 GUI 갱신 상한 비교:

| 갱신 설정 | 실제 app 갱신/초 | Isaac 프로세스 CPU 사용량 |
|---|---:|---:|
| 기존 무제한 | 157.45 | 528.1% |
| 60 FPS | 59.66 | 231.9% |
| 30 FPS | 29.96 | 123.6% |

CPU 100%는 논리 코어 하나다. 이는 같은 실행 중 장면의 짧은 GUI 표본이며,
원격 입력부터 사용자 화면 표시까지의 지연 측정이나 물리 처리량 비교가 아니다.
30 FPS 설정은 불필요한 갱신 부하를 줄이지만 VNC 지연 해결 여부는 사용자 쪽 조작 확인이 필요하다.
추가 확인 시 내부 FPS가 유지되는데 원격 화면만 끊기는지, 특정 마우스 버튼 조작만 문제인지 구분한다.

원자료: `/root/docker_share/mujoko_debugging_data/20261009_isaac_viewport_diagnosis/`.
`live_baseline.json`, `drag_baseline.json`, `fps_cap_comparison.json`에 실제 측정값을 보존한다.
관련 공식 자료: [NVIDIA 성능 안내](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/sim_performance_optimization_handbook.html).

`validate_scene.py`는 입력 보존·메시·부착 좌표·관절·재질을 검사한다.
검사 통과가 충돌 없는 시작, 변형/파단의 정확성, 수확 성공 또는 GPU 속도 향상을 의미하지 않는다.
## 수평 진입 변경

수확 진입의 기본 상하 기울기는 이제 0°로 고정한다. IK를 이유로 -60° 등 다른 기울기를
자동 선택하지 않는다. 수평 방향 옵션, 실제 물리 중단 결과와 GUI 명령은
[HORIZONTAL_ENTRY_STATUS_KO.md](HORIZONTAL_ENTRY_STATUS_KO.md)에 있다. 수평 FK 통과와 실제 진입 완료는 별개다.

후속 [영상 기준 대기 자세·수평 진입 결과](VIDEO_ENTRY_STATUS_KO.md)에서는 하부 받침을 제거하고
팔을 접은 수평 대기 자세에서 먼저 개방하도록 수정했다. 기존 11번 과실에서 GUI 진입·2초 유지·복귀를
완료했다. 별도 닫기 시험은 옆 과실 접촉으로 중단됐으므로 수확 완료로 집계하지 않는다.
실행 명령과 실제 저장 상태의 1배속 영상은 해당 문서에 있다.
**후속 정정:** 저장 자세에서 그리퍼–팔 link2의 자기관통이 확인됐다. 위 완주는 명령 실행 완료이며
유효한 비충돌 경로가 아니다. 로봇 자기충돌이 꺼져 있어 기존 native 감시가 놓쳤다. 해당 문서의 정정을 먼저 확인한다.
