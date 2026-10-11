# S300 벨트 그리퍼 MuJoCo 3.15 소규모 시험

작성: 2026-10-12 KST. 브랜치 `codex/s300-mujoco-belt-smoke-20261012`.
Isaac 작업 보존 커밋: `ca3a716`.

## 결론

**송이 하나 + 주줄기 + 기록된 그리퍼 자세** 범위에서 120Hz 실행은 실시간의 약 1.50배,
XML/메시 로딩·컴파일은 약 0.27초였다. 벨트가 열매를 안쪽으로 이동시키고 연결부가
분리되는 동작을 재현했다. 전체 로봇/리프트/레일/농장 환경의 성능이나 이식 완료를 뜻하지 않는다.

중요한 미완료: prescribed 그리퍼는 새 접촉력에 따라 물러나지 않는다. 분리된 열매가
벨트에 끼었을 때 120Hz 최대 약 315N, 240Hz 약 1170N의 접촉력이 기록됐다.
낙하 열매/바닥 겹침도 120Hz 30.42mm다. **빠른 수확 성공 환경으로 채택하지 않는다.**
다음 구현은 전체 하우스 이전보다 동적 턱/유한 구동력과 정확한 충돌 분해 연결을 우선한다.

## 버전과 보존

- 공식 최신 안정판 확인: [MuJoCo 3.15.0, 2026-10-05](https://github.com/google-deepmind/mujoco/releases/tag/3.15.0).
- 시험 Python: `/root/farmily_tomato/mujoco-benchmark/.mujoco315-venv/bin/python`.
- 기존 `.venv`의 MuJoCo 3.13.0, `.mjlab-venv`, Isaac 환경은 변경하지 않았다.
- `scripts/s300_smoke/requirements-lock.txt`에 설치 패키지를 고정했다.
- 사용자 `gripper-design` 미커밋 변경/V6, Git bundle은 보존하고 이번 커밋에 포함하지 않는다.
- 이전 GUI의 t=18.4167s 동시 파단 원인은 여전히 미확정이다. 카메라만 조작했다는 사용자 확인을 유지한다.
  이번 MuJoCo 시험으로 그 Isaac 원인이 해결됐다고 주장하지 않는다.

## 재사용한 것과 실제 범위

원본:

```
/root/docker_share/mujoko_debugging_data/20261011_205000_target_only_row_harvest/assembly_step01_plant41
/root/docker_share/mujoko_debugging_data/20261011_205000_target_only_row_harvest/trials/step01_plant41_shallow_yaw_minus45_runtimefix
```

기존 USD의 형상, 강체/관절 매핑, 질량, 줄기 stiffness/damping, 인접 연결 제외,
저장된 **실제 그리퍼 5바디 pose와 벨트 명령**을 사용했다. `convert_model.py`의
좌표 출력 함수와 기존 3축 힌지 변환 관례, `render_backend.py`의 EGL 설정을 재사용했다.

- 원래 분절의 주줄기 16개, 대상 송이 peduncle/rachis/pedicel 포함 줄기 총 79바디.
- 열매 10개(01~05, 07~11). 다른 송이/질량은 이번 범위에서 제외.
- MuJoCo 총 95바디(월드·그리퍼 포함), 294자유도, 631geom, 충돌 식물/툴 427geom + 바닥 1개.
- 잎은 표시만 한다. 줄기 캡슐은 원래 위치/반경/분절을 유지한다. 가지 표시에는 캡슐을 사용한다.
- 로봇 팔/리프트/기체/하우스는 없음. 그리퍼는 mocap으로 저장된 실제 pose를 재생한다.
- source가 36.8초 마지막 열기 중에 종료되므로 그 시점까지만 재생한다. 완료하지 않은 후퇴를 생성하지 않았다.
- 화면 없는 물리 실행과 저장 qpos 영상/뷰어는 별도다. 영상 생성은 물리를 다시 실행하지 않는다.

## 엔진 차이를 처리한 방법

1. **벨트:** MuJoCo native `geom_surfacevel` 사용. 접촉 없는 열매에 인력/흡착력을 주지 않는다.
   기존 PhysX 양성 fixture는 `+0.02m/s` 명령에서 CAD Y가 2초간 `-39.984mm` 이동했다.
   MuJoCo는 실제 표면 방향을 지정하므로 **기존 명령의 부호를 반전**했다.
   body-local 벨트 방향을 각 geom-local 프레임으로 변환한다. 기존 명령의 수치 부호를 그대로
   복사했던 최초 시험은 outward 오연결로 분리 보존했다.
2. **열매 연결:** free fruit + neck connect constraint + 상대 회전 passive spring으로 구현했다.
   힘 3N/토크 .008Nm 초과가 측정되면 connect를 해제한다. 타이머/강제 pose 이동으로 분리하지 않는다.
   source raw angular coefficients를 Nm/rad로 해석한 **미보정 근사 모델**이다. PhysX native D6 soft limit/break와
   동등하다는 검증은 아니다. 과실 관성도 질량/치수에서 구한 타원체 근사다.
3. **적분:** 초기 implicitfast 시험은 정지 상태에서 연결부 진동/전체 분리가 발생했다.
   자기 각속도 감쇠를 엔진의 implicit damping으로 처리하고 `discrete` 적분기를 사용했다.
   parent reaction은 스텝 시작 속도로 계산한다. discrete는 감쇠가 반영된 유효 관성을 제약 계산에 사용한다.
   이것을 버전 업데이트만의 개선으로 해석하지 않는다. 초기 실패는 삭제하지 않았다.
4. **충돌:** source mesh별 convex hull을 사용한다. PhysX의 convex decomposition과는 다르며,
   구멍/오목한 부품을 포함한 gripper 충돌체 동등성은 미검증이다. 마찰 .5와 MuJoCo 고유 solref/solimp를 사용한다.
5. **경계 조건:** native surfacevel은 벨트 및 그리퍼의 접선 속도를 전달한다.
   mocap 법선 속도, 로봇 관성, 턱의 힘에 따른 정지는 구현되지 않았다. 이것이 현재 힘 결과의 중요한 한계다.
6. 별도의 중력 보상/초기 preload는 넣지 않았다. 실제 3송이 Isaac continuation과 달리 대상 송이만
   중력 아래에서 안정화되므로, 초기 장면의 동일 pose가 이후 동일 위치/변형을 보장하지 않는다.

## 결과

데이터 루프 시간은 물리·접촉 검사·상태 메모리 기록 포함. 모델 변환/로딩, 파일 압축 저장,
렌더링 제외. 36.8초 분량이며 idle만 5초다. CPU 사용률 약 100%: 단일 프로세스/대략 CPU 한 코어.

| 조건 | 루프 시간 | 실시간 배율 | 분리 | 부착 상태 최대 겹침 | 분리 후 최대 겹침 |
|---|---:|---:|---|---:|---:|
| 정지 60Hz, 5초 | 1.56초 | 3.21배 | 0 | 0.228mm | 0 |
| 안쪽 60Hz, 2반복 | 12.17 / 12.14초 | 3.02 / 3.03배 | 9개 | 2.759mm | 65.47mm |
| 안쪽 120Hz, 최종 2반복 | 24.52 / 24.50초 | 1.50 / 1.50배 | 03·04·05·07 | 0.954mm | 30.42mm |
| 안쪽 240Hz | 48.67초 | 0.756배 | 03·04·05·07 | 0.228mm | 17.51mm |
| 벨트 정지 120Hz | 24.18초 | 1.52배 | 02·05 | 0.728mm | 37.80mm |

120Hz는 모델 로딩 약 .272초, 새 MjData/reset 약 .0073초.
변환기 실행 자체는 약 2.53초이며 한 번 변환해 재사용한다. OS 캐시가 있는 조건의 측정으로
전체 농장 Isaac의 초기화 시간과 동등한 모델 조건 비교는 아니다.

60Hz는 1mm 기준을 초과하고 120/240Hz와 결과도 달라 채택하지 않는다.
120/240Hz의 같은 분리 번호만으로 접촉력/궤적 수렴이나 물리 동등성을 선언하지 않는다.
120Hz 두 반복은 `time/qpos/eq_active/fruit_positions/mocap_pos/mocap_quat` 전부 bit-exact다.

120Hz 23.0→24.7초, 분리 전 gripper-local Y 이동:

- 안쪽 벨트: 03번 -26.19mm / 05번 -29.65mm.
- 벨트 정지: 03번 -1.30mm / 05번 -1.70mm.

안쪽 이송 효과는 확인됐지만 belt-off에서도 닫는 하중으로 분리가 일어났다.
03/05/07은 토크 조건, 04는 힘 조건으로 분리됐다. 순수 비틀림 수확만의 결과가 아니다.
07은 미숙 열매이므로 익은 열매만 선별한 성공도 아니다.

120Hz 부착 상태 tool 최대 힘은 10.65N, 분리 후 tool 최대 힘은 315.37N.
240Hz는 각각 7.20N / 1169.71N. 후자의 최대는 이미 분리된 04번과 오른쪽 벨트 접촉이다.
기록된 닫힘 pose가 새 열매 배치의 접촉력에 반응하지 않는 것이 주요 구조적 한계이며,
해당 힘을 실제 그리퍼 하중으로 해석하지 않는다. 상세 시점과 pair는 validation JSON에 있다.

## 판정/저장/회귀시험

- 1mm active-contact 기준을 유지한다. 초과 첫 시점/대상을 저장하고 진단용으로만 계속한다.
  active pair 10mm 또는 solver warning/비정상 상태는 중단한다.
- 이미 분리된 열매 접촉은 별도 집계한다. 큰 겹침을 숨기지 않는다. `full_physics_validated=false`.
- 접촉 거리와 힘은 solver 시점 `t[k]`, 새 qpos와 분리 효력은 `t[k+1]`로 구분한다.
- 접촉 분류는 해당 solver 시작 시점의 부착 상태를 사용한다. 첫 시험의 종료 상태 기준 분류는
  저장된 접촉/분리 timestamp로 재분류했고, 최종 120Hz 반복에는 수정 코드가 적용됐다.
- 최종 120Hz 두 실행에 Python 경로/코드 사본·SHA/입력 XML·trace SHA 보존.
- native 벨트 ±방향/정지, 접촉 없는 상태, 회전 프레임, PhysX 실측 방향 매핑,
  실제 constraint-force break와 임계 이하 유지, torque/idle 대조: **자동시험 6개 통과**.
- 모든 결과 `training_eligible=false`, `harvest_success=null`. 기존 학습 데이터에 포함하지 않는다.

## 바로 재생

```bash
cd /root/farmily_tomato
DISPLAY=:0 ./mujoco-benchmark/.mujoco315-venv/bin/python \
  mujoco-benchmark/scripts/s300_smoke/view_smoke.py \
  /root/docker_share/mujoko_debugging_data/20261012_mujoco_belt_smoke/verified_inward120/repeat_00 \
  --gui
```

이는 저장된 실제 qpos의 1배속 반복 재생이다. 카메라를 움직여도 물리를 다시 계산하지 않는다.

새 headless 실행(새 폴더로 저장):

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.mujoco315-venv/bin/python -u \
  mujoco-benchmark/scripts/s300_smoke/run_smoke.py \
  /root/docker_share/mujoko_debugging_data/20261012_mujoco_belt_smoke/model315 \
  "/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_mujoco315_smoke" \
  --hz 120 --repeats 2
```

환경 재생성은 Python 3.11 `venv`에 `scripts/s300_smoke/requirements-lock.txt` 설치.
모델 변환은 기존 `.asset-venv`로 `export_scene.py ASSEMBLY TRIAL NEW_MODEL_FOLDER` 실행한다.

## 결과/영상

결과 루트: `/root/docker_share/mujoko_debugging_data/20261012_mujoco_belt_smoke/`.
`index.html`, `validation.json`, model315/XML·OBJ·trace, 각 실행 states/contact/metrics/MJB 포함.
모든 영상은 960×720, 30fps, 1배속, 작은 자막, 실제 저장 상태 재생이다.

- 안쪽 120Hz: `verified_inward120/repeat_00/actual_top_visible.mp4`
- 벨트 정지 120Hz: `off120/repeat_00/actual_top_visible.mp4`
- 기준 초과 60Hz: `inward60/repeat_00/actual_top_visible.mp4`

### 검은 영상 수정 (2026-10-12)

최초 `actual_top.mp4` 3개는 자막만 보이고 장면은 검은 화면이었다. 파일 디코드 검사만으로
장면이 보인다고 확인한 것은 잘못이었다. 원인은 재생기가 `mj_kinematics`/`mj_comPos`만
호출하고 `mj_camlight`를 누락해 카메라·조명 world transform이 0으로 남은 것이다.
물리 재실행 없이 저장된 동일 qpos/mocap 기록을 사용하여 `_visible.mp4`로 다시 렌더링한다.
기존 검은 영상은 증거로 보존하고 HTML에는 수정 영상만 표시한다.

재생기에 camera/light 갱신과 자막을 그리기 전 모든 프레임의 균일/검은 화면 검사를 추가했다.
별도 native 렌더 회귀시험은 저장 pose 복원 후 카메라·조명 위치, 실제 물체 픽셀,
시간/qpos 보존을 검사한다. 기존 물리 시험 6개와 렌더 회귀 2개, 총 8개 통과.
수정 영상 검사는 결과 루트의 `render_fix_validation.json`에 저장한다.

## 다음 판단

MuJoCo의 빠른 로딩과 단일 환경 실시간 초과 처리 가능성은 확인했다.
곧바로 전체 농장을 옮기거나 60Hz를 채택하기보다, 같은 소규모 장면에서:

1. 턱을 동적 슬라이더/유한 구동력으로 연결해 끼임 시 강제 pose가 발생하지 않게 한다.
2. 원래 PhysX cooked convex 조각을 내보내거나 동일한 검증된 분해 형상을 두 엔진에서 사용한다.
3. 열매 연결부 응답과 분리 후 접촉을 대조 시험으로 보정한다.
4. 그 뒤 동일 모델의 로봇 포함 실행/반복 처리량을 비교한다.

전체 농장/이동 수확, CPU 다중 프로세스, GPU 병렬 성능은 이번 결과로 확인되지 않았다.
