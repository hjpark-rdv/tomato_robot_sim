# Farmily Tomato 작업 인수인계

## 2026-10-12: RB5 부위 재질·S300 정점 색상 복원

- v6의 메시당 첫 색상만 읽던 누락 수정: RB5 7메시/38재질 구역, S300 20다색 메시.
- balanced/ultra v7 캐시에서 원본 면별 색을 팔레트 UV로 복원. fast는 기존 간이 표시.
- 986,317면의 원본/변환 좌표 대조, 물리 배열·정점·면·법선·원본 MJB/states SHA 동일.
- 비교/영상: `/root/docker_share/mujoko_debugging_data/20261012_rb5_source_colors/index.html`.
- 세부: [S300_MUJOCO_RENDER_QUALITY_KO.md](mujoco-benchmark/S300_MUJOCO_RENDER_QUALITY_KO.md).

## 2026-10-12: MuJoCo 표시 품질 fast / balanced / ultra

- 물리 기준 `3e37040` 이후 표시 기능 확장. 기존 physics 모델/원본 결과는 보존.
- `./mujoco-benchmark/run_s300_mujoco.sh replay --quality ultra`(기본값), `balanced`, `fast` 선택.
- PBR 표시용 Python3.12 `.mujoco315-render-venv` 분리. 물리 Python3.11/MuJoCo3.15 유지.
- balanced/ultra: 원본 5,082geom 재질, 98줄기 skin, 15CAD 링크 시각 동작 복원.
- source-hidden 186충돌 프록시의 표시 material을 해제해 Filament의 흰 hull 표시 오류 수정.
  충돌 masks/물성/actuator/관절/기존 1mm 물리 기준은 변경하지 않았다.
- USD linear 색상 → Filament sRGB 경계 변환, 중립 표시 조명, +1EV 실제 적용.
  3.15 indexed-skin/최초 색보정 setter 오류를 표시 어댑터에서 회피.
- ultra GUI1920×1080 요청/2×SSAA, 영상4K/내부8K. 높은 표시 비용이 있으므로 실제 FPS는 문서 참조.
- 공유 결과 `/root/docker_share/mujoko_debugging_data/20261012_s300_mujoco_quality/index.html`.
- 상세 [S300_MUJOCO_RENDER_QUALITY_KO.md](mujoco-benchmark/S300_MUJOCO_RENDER_QUALITY_KO.md).
- `replay/render`는 실제 저장 상태 표시이며 `test`의 새 물리 실행과 구분한다.


## 2026-10-12: S300 전체 농장·연속 수확 MuJoCo 3.15 연결 완료

- `codex/s300-mujoco-belt-smoke-20261012`, Isaac 보존 `ca3a716`, 이전 소규모 시험 `51e22eb` 후속.
- 전체 농장 원본 외형 + S300/리프트/RB5/동적 턱/안쪽 벨트, 241바디/739DOF. pose 강제 이동 없음.
- 레일 총 약1.714m 이동, 두 송이의 선정 집단03/04/05 모두 포함해 **빨간색 판정 대상9개 분리**.
  원본 policy의07/08도 빨간색 대상. 진입 전96mm 개방, 수평 진입, 닫기/열기2회, 후퇴·다음 위치 완료.
  잔여 대상1/4개 중 한 번에 잡을 집단 최대1/2개라 최소집단3개 미달로 남김.
- 최종 **300Hz 접촉/60Hz 이동/worker0**: 136.137sim/174.589loop초=**.780×**.
  활동 겹침.983mm, 1mm 중단0. 낙하 후 최대15.37mm는 별도 진단이며 전체 물리 정확도 통과 아님.
- 전체 반복/worker4 전 구간/4 접촉 구간의 저장 배열 완전 일치. worker4는 전체 실행에서 더 느려 기본0 유지.
- 타깃 송이·자기 주줄기만 식물 충돌, 잎 무시, 나머지 식물 외형 유지. 구조물은 거터18개+바닥.
  opt-in 낙하 과실↔거터 진단 예외, 바닥 top-plane 사용. 부착 과실/로봇 거터 기준 유지.
- 접촉 margin 이식 오류와 마지막 열기 60Hz 누락, 검은 영상 camera/light 갱신, 캐시 원본 보호 수정.
- 명령: `./mujoco-benchmark/run_s300_mujoco.sh replay`(실제 기록1× GUI), `test`(새 물리), `prepare`(새 변환).
- 결과 `/root/docker_share/mujoko_debugging_data/20261012_s300_mujoco_full/index.html`, 대표 `campaign_final`.
- 자세한 이식/속도/영상/검사/한계: [S300_MUJOCO_MIGRATION_KO.md](mujoco-benchmark/S300_MUJOCO_MIGRATION_KO.md).
- RGB-D 기반 선택·전체 행로·GPU 병렬·수거·실물 손상 기준은 미완료. `harvest_success=null`, `training_eligible=false`.

## 2026-10-12: MuJoCo 3.15 단일 송이 시험

- Isaac 기존 작업 `ca3a716` 보존 후 `codex/s300-mujoco-belt-smoke-20261012`에서 시험.
- 최신 안정판 3.15.0은 별도 `mujoco-benchmark/.mujoco315-venv`; 기존 환경 보존.
- 주줄기+1송이+기록된 그리퍼 pose, 로봇 팔/농장 제외. 120Hz 36.8초를 약24.5초(1.50배) 처리.
- 안쪽 벨트 03/04/05/07 분리, belt-off 02/05 분리. 120Hz 두 반복 상태 완전 일치.
- 60Hz는 약3배지만 1mm 기준 초과·9개 분리로 채택 불가. **전체 이식/검증된 수확 성공 아님**.
- prescribed 턱이 접촉력에 양보하지 않아 분리 후 최대315N/바닥30.4mm 겹침. 동적 턱 및 정확한 충돌 분해가 다음 단계.
- 보고서: [S300_MUJOCO_SMOKE_KO.md](mujoco-benchmark/S300_MUJOCO_SMOKE_KO.md).
- 데이터/영상: `/root/docker_share/mujoko_debugging_data/20261012_mujoco_belt_smoke/index.html`.
- 이전 Isaac GUI 갑작스러운 동시 파단 원인은 미확정 상태 유지.

## 2026-10-12 MuJoCo 소량 비교 전 Isaac 상태 보존

사용자 요청으로 아래 Isaac 연속 수확/식물 충돌 범위/얕은 진입 작업을 먼저 커밋하고,
새 브랜치에서 기존 MuJoCo 코드와 현재 벨트 그리퍼·송이 하나의 소량 비교를 진행한다.
사용자 작성 중인 `gripper-design` 변경과 로컬 Git bundle은 별도 보존한다.

직접 GUI 재시험 `20261011_234926_536009_rail_harvest_gui`는 시작 준비 약165초,
18.4167sim/38.727loop초(0.476배속), 접근 중 조인트10개 동시 파단/겹침 중단이다.
원래 headless 시험과18.400초까지1,105개 표본의 pose/q/명령이 bit-exact이며,
다음 한 스텝에서만 급변한다. 사용자는 카메라만 조작했다고 확인했다.
GUI가 원인이라고 확정하지 않는다. 화면 갱신/앱 구동 차이의 정확한 원인은 미확정.
MuJoCo 시험은 별도 모델·출력이며 기존 Isaac 결과나 학습 데이터에 섞지 않는다.

## 2026-10-11 타깃만 충돌: 레일 이동·얕은 진입 실제 시험 (미완주)

사용자가 D(타깃 송이·자기 주줄기만 식물 충돌)를 선택했다. 이전 Plant29 D의 정상
완료 실제 상태에서 새 창을 구성. 비목표 식물 충돌 제외 정책이 창 전환 후 새 타깃에
정확히 재연결되도록 수정했다. 전체 농장 외형·로봇·집·거터 충돌은 유지한다.
사용자 후속 지시대로 주줄기를 피하는 수평 yaw와 얕은 진입/벨트 인입을 시험했다.

- Plant41 송이축 대비−45°, 기준점 깊이15mm, 동일 열매 집단03/04/05. 실제 레일71.38mm
  이동 후 닫기/열기2회 중03/05의 익은2개 분리, 미숙0. 36.8초의 마지막 열기에서
  하판`R/part_244`↔자기 주줄기10 callback 겹침1.385mm로 중단. 기준1mm 유지.
  후퇴·다음 송이 이동 미완료이며 정상 완료 checkpoint로 승격하지 않았다.
- 36.8sim/36.098loop초=1.019×. 준비·검사·native 초기화·렌더는 별도다.
  분리 전 안쪽 이동03=45.5mm/05=68.2mm. 벨트 단독 원인 비교는 아니다.
  주줄기 중심 약46.6mm 이동/복원과 명목 대비TCP 높이−9.23mm를 함께 확인했다.
- Plant41/43/31의 차단된 계획도 보존. 도착 전 로봇 위치로 송이 PCA 부호를
  고르던 오류를 도착 위치 기준으로 수정. Plant31은 거터 지지 와이어가 집단 중앙을
  지나 얕은 yaw 대안도 보류. Plant41의−75° 최종 자세는 가능했지만 준비 경로와
  회전수 정리/Cartesian/IK branch/리프트 대안은 자기충돌 또는2mm 계획 여유 미달.
  양수 여유 미달을 실제 관통으로 부르지 않는다. 추가 물리 실행0회.
- `--truss-entry-yaw-offset-deg` 선택형±90° 추가. 자연 송이축과 실제 진입 방향을
  분리해 집단 footprint/경로에 동일하게 적용하고 반복 시 각도를 누적하지 않는다.
  기본0° 명령은 유지. 표시용 disabled Skin의 CollisionAPI 정리 호환 오류도 수정.
  첫 native 초기화 오류는 수확 동작 전이었고 성공으로 세지 않는다. 관련177시험PASS.

결과 `/root/docker_share/mujoko_debugging_data/20261011_205000_target_only_row_harvest/index.html`.
작은 요약 `nvidia-sim/harvest_s300/validation/target_only_row_harvest_20261011.json`.
실제 상태1배속/작은 자막/전체 배경의 권장 영상2개(전체/정면 밝기보정).
가림/어두운 카메라 점검 원본2개도 index에 별도 보존. 포집·완주·전체 레일 수확
미검증, `training_eligible=false`, `harvest_success=null`. 미커밋 작업이며 기존 변경 보존.
다음에는 정적 진입각뿐 아니라 약9mm의 추종 오차와 당김 후 주줄기 복원을 고려해야
한다. 중단 상태를 숨기거나 이전 초기 상태로 되돌려 연속 성공처럼 집계하지 말 것.

## 2026-10-11 전체 농장 식물 충돌·탄성 속도 비교

Plant29 동일 명목 이동·수확/60Hz/CPU16을 네 조건에서 새 실행. 현재 탄성1그루는
82.8sim/93.01loop초=0.890×, 주변1그루 추가0.670×(55.93초 중단), 주변2그루 추가
82.8/123.60=0.670×, 타깃 송이·자기 주줄기 외 식물 충돌 제외82.8/76.52=1.082×.
각 익은 새 분리5/4/4/6, 미숙0/0/0/1(D Fruit09). 독립 시행이므로 합산 금지.
최초 미숙0 표기는 원본 summary에 맞게 정정했다. 주변1그루 조건은 분리된
Fruit03↔그리퍼 뒤 프레임2.157mm 겹침으로 중단. 기준1mm/물성은 유지했다.
추가 주변 식물↔로봇 접촉0회이므로 강하게 밀 때의 부하는 미측정. 충돌 제외는
21,187 collider를 끄며 전체 시각 배경·거터·로봇·타깃은 유지한다. D의 실제 초기
자세는 collision 변경 후 최대5.282mm 차이가 있어 동일 결과 최적화가 아니다.
전체 native 프로세스287.71~428.49초, 환경 검사75.70~155.97초로 준비/검사 비용이 남았다.
기존 기본 충돌 정책은 유지하고 선택형 파생 자산 생성기만 추가했다. 추가 탄성 식물의
checkpoint 다음 창 인계는 명시적으로 미지원. 관련89개 테스트PASS.
결과 `/root/docker_share/mujoko_debugging_data/20261011_194707_plant_interaction_comparison/index.html`.
동일 시점·전체 배경·실제 상태·작은 자막1배속 영상4개. 상세/명령/한계는
[ROW_CONTINUATION_KO.md](nvidia-sim/harvest_s300/ROW_CONTINUATION_KO.md).
`training_eligible=false`, `harvest_success=null`, 포집 미검증. 이번 비교는 커밋 전 작업이다.

## 2026-10-11 60Hz 두 송이 연속 이동·수확 완료

기존 `20261011_174730_released_same_truss_60hz`의 Plant22 실제 완료 상태(5개 분리)를
인계해 Plant29/Truss00을 새60Hz/CPU16/TGS64/4 실행으로 이어갔다. 실제 이동1.642386m,
도착오차0.0113mm. 리프트168.921→최저10.596→복귀160.606mm. 접근 전96mm 개방,
수평 진입, 양쪽 안쪽50mm/s, 닫기/열기2회·후퇴·복귀 완료. 새 익은02/03/04/08의
4개 분리, 앞선5개와 연결 합계9개이며 첫 송이는 재실행하거나 초기화하지 않았다.
두 번째 잔여 익은01/05/07의3개를 실제 마지막 자세에서 기존 집단 선택기로 재평가:
한 번에 들어오는 집단 최대2개라 최소3개 기준 미달, 추가 진입 없이 남김. 미숙 새 분리0.
두 번째82.8sim/92.358loop초=0.897×. 첫 송이 포함139.8sim/173.370loop초=0.806×.
장면 준비 약244.6초, 두 번째 native 프로세스 전체402.6초(환경검사154.4초 포함)는
별도다. 전체 작업이 실시간이라는 뜻은 아니다. 초기/60Hz 전체명목/실제 자기충돌PASS,
인계 관련90개 테스트PASS, 다음 인계 가능한 정상 완료 checkpoint까지 검증.
이전 붙은 식물은 측정 변형대로 고정, 분리 열매는 동적으로 유지하는 기존 window 방식.
카메라 판단·전체 레일·포집은 미검증. `training_eligible=false`, `harvest_success=null`.
결과 `/root/docker_share/mujoko_debugging_data/20261011_180823_continuous_harvest60/index.html`.
작은 자막의 실제 상태1배속 영상: 첫 송이/두 번째 이동 전체/두 번째 수확 탑뷰3개.

## 영상 표시 선호 (2026-10-11)

사용자가 자막이 화면을 너무 많이 가린다고 지적했다. 이후 S300 영상은
`render_motion.py`의 작은 기본 자막(`FontSize=10`, 얇은 외곽선, 하단 여백6)을 사용한다.
진단 문구는 유지하며 실제 스타일을 render JSON에 기록한다. 기존 MP4는 덮어쓰지 않는다.

## 2026-10-11 분리 후 같은 송이 접촉 허용, 60Hz 완주

사용자 승인에 따라 native 분리된 열매↔자기 송이의 과실·꼭지·중심가지·peduncle
접촉은 기록 후 계속한다. 실제 namespace/manifest/collider/native break 시각으로
확인하며 다른 송이·주줄기·팔/벨트 등은 기존 검사를 유지한다. 새 row 실행의 기본
인자에 반영했다. 직접/기존 case 실행은 `--released-fruit-same-truss-diagnostic` 명시.
같은 Plant22 경로1회: 이전28.467초 중단을 통과해57초 전체 동작·후퇴·복귀 완료.
익은 열매5개 분리, 익은2개/미숙3개 남음. 1진입 예산이며 송이 전체 수확은 아니다.
60Hz/CPU16/TGS64/4,57sim초/81.012loop초=0.704×. 같은 경로120Hz는151.138초,
이번 loop는46.4% 짧지만 분리 개수도6→5로 달라져 물리 동등 최적화는 아니다.
이전60Hz 중단28.467초까지1,709시점 pose/q/명령/시각/phase 정확히 일치.
초기·전체 명목 환경·실제 자기충돌PASS, 관련149테스트PASS. 물성·형상·1mm 유지.
낙과 접촉 예외로 전체 geometry의1mm 이하를 보증하지 않는다. `harvest_success=null`,
`training_eligible=false`, 포집 미검증. 리프트 복귀 편차는 사용자 요청으로 소수 시험의
blocker가 아니며 실제 시작q/목표 높이에 맞춰 계획한다. 제어기 수정은 하지 않았다.
결과 `/root/docker_share/mujoko_debugging_data/20261011_174730_released_same_truss_60hz/index.html`.
상세 [공통 접촉 기준](nvidia-sim/harvest_s300/COMMON_CONTACT_LIMIT_KO.md).
아래 같은 송이 접촉 중단 기록은 변경 전 역사다. 새 정책60Hz의 두 번째 식물 연속 실행은 미검증.

## 2026-10-11 분리 후 그리퍼 하판·지지대 접촉 중단 제외

사용자 요구에 따라 정확한 native 분리 후 좌우 하판/슬라이더 지지대4개와의 접촉은
별도 옵션으로 기록 후 계속한다. collision/물성/1mm 기준은 그대로이며 붙어 있는
과실 및 다른 접촉은 기존 검사를 유지. `run_row_harvest` 새 기본 인자에도 반영했고,
직접 실행/기존 cases에는 `--released-fruit-gripper-support-diagnostic`를 명시한다.
같은 Plant22 경로60Hz 재시험: 기존 중단27.5초를 통과, 추가1개 포함5개 분리 후
28.467초에 분리 Fruit03↔같은 송이 Pedicel_proximal_09_2 접촉으로 중단.
마지막 raw 겹침1.216mm/계산 후1.092mm, 아직 완주 아님. 하판·지지대 이외의
낙하 후 접촉 허용은 이번 검증 범위에 포함하지 않았다.
27.5초까지1,651시점의 pose/q/명령/시각/phase가 원본60Hz와 정확히 같음.
28.467sim/41.420loop초=0.687×(중도 종료). 초기/실제 자기충돌PASS, 관련144테스트PASS.
결과 `/root/docker_share/mujoko_debugging_data/20261011_170345_released_fruit_support_60hz/index.html`.
정책 상세는 [공통 접촉 기준](nvidia-sim/harvest_s300/COMMON_CONTACT_LIMIT_KO.md).

## 2026-10-11 이동 → 집단 수확 → 다음 송이 실제 연결 완료

`e47a04f` 이후 미커밋 작업. 별도 fresh 설치 장면에서 Plant22/Truss02 수확 후,
정상 최종 native checkpoint를 인계해 Plant29/Truss00까지 실제 이동·수확·후퇴 완료.
첫 실제 이동−20.000mm/새 분리6개, 다음+1,642.386mm/새 분리6개. 두 실행 모두
초기/전체 명목 환경/실제 자기충돌 검사 통과, 공통1mm 중단 없음. 익은 열매는 각1개
남고, 대상 밖 미숙 열매의 새 분리는0. 실패 재시험의 열매 수는 합산하지 않는다.
120Hz CPU16 TGS64/4:57sim/151.138loop초=0.377×,
82.792sim/165.526loop초=0.500×. 준비·계획·검사·렌더 제외이며 실시간 아님.
원본 바퀴가 바닥 이음부에 걸려 명령1.622m 중 실제0.798m만 움직인 문제를 발견했다.
별도 fresh 설치 높이2mm 보정, 실제 rail 도착 확인, 정지 개방 유지0.5초,
전체 진입/후퇴 환경검사 연결로 수정. 충돌 형상/물성/공통1mm는 유지했다.
수확 대기 자세 복귀 후 리프트 높이 편차가 회당 약8.47mm 남아 긴 반복 전 수정 필요.
전체808개 및 후속 바닥 검사5개(추가1개 포함) PASS. 영상3개 전체 디코드/프레임검사 PASS.
카메라 대상 선택/포집 성공/바퀴 구름·슬립은 미검증. `training_eligible=false`,
`harvest_success=null`. 상세는 [연속 실행 기록](nvidia-sim/harvest_s300/ROW_CONTINUATION_KO.md).
결과 `/root/docker_share/mujoko_debugging_data/20261011_140011_move_harvest_repeat/index.html`.
요약 `nvidia-sim/harvest_s300/validation/move_harvest_repeat_20261011.json`.

## 2026-10-11 소량 레일 이동·집단 수확 연결 시험

현재 `e47a04f` 이후 작업. 새 송이2개/준비 경로3개를 검사했다. Plant35/Truss02는
이웃 송이·지지 와이어, Plant37/Truss00는 주줄기 교차로 모두 실행 전 차단.
Plant37 대기 거리100→50mm도 차단되어 새 송이 수확 physics는0회다.
별도 레일 단독 실제 이동451mm를 확인했다. 첫 실행에서 리프트 하중 보정 목표가
측정q로 덮어써져8.370mm 처짐을 발견·수정했다. 같은 시작 상태의 독립 A/B 재실행:
최대 리프트 변화0.000105mm 미만, 실제 자기충돌PASS/중단 없음/새 분리0.
수정 후11.45sim초/11.931loop초=0.960×. 준비·장면 로딩·검사·영상은 별도.
새 active 식물의 잎 무시 정책 상속과60Hz 환경검사 연결도 수정. offline793개 PASS.
소스 물성/충돌체/공통1mm 유지. 기존 gripper-design 사용자 변경은 별도 보존.
이동+리프트 높이 변경+수확의 연속 완주는 아직 아니다. 두 레일 시험을90cm 연속 이동으로
합산하지 않는다. 새 카메라 판단은 미구현; 다음 병목은 그리퍼 전체와 주줄기를 고려한
준비·진입 위치 선정이다. `training_eligible=false`, `harvest_success=null` 유지.
결과/실제 이동 영상: `/root/docker_share/mujoko_debugging_data/20261011_131243_small_row_group_harvest/index.html`.
검증 요약: `nvidia-sim/harvest_s300/validation/small_row_group_harvest_20261011.json`.
기존 [레일 연결 문서](nvidia-sim/harvest_s300/ROW_CONTINUATION_KO.md)에 범위·한계 기록.

## 2026-10-11 수확 집단 중앙으로 접근 위치 보정

`HarvestGroupSelector`가 선택된 열매들의 측정 중심 평균을 두 벨트 사이에 맞추도록
그리퍼 국소X 방향만 보정한다. 기존 집단 선택 우선순위/송이 yaw/진입 깊이·높이를
유지하며, 보정 후 모든 선택 과실의 반경 포함 범위 및 대상 밖 과실 유입을 재검사한다.
식물 위치/물성을 바꾸지 않는다. 원래 기준점·이동량·보정 전후 오차는 선택 이력에 저장.
동일Truss_02 한 번 진입 native시험: 계획15.920mm 옆으로 이동, 캡처16.333초의 실제
집단 중심 좌우15.797→−0.122mm, 진입 끝−0.103mm. 두 닫기/완전 열기와 후퇴 완료.
대상01~05의5개 분리, 대상07/08 및 대상 밖09/10/11 남음. 초기/실측 자기충돌PASS.
53.5sim초/79.660loop초. 이전4개 분리 대비 이번5개이나 한 장면으로 수율 향상 일반화 금지.
offline789개 PASS. `training_eligible=false`, `harvest_success=null`, 포집 미검증.
결과: `/root/docker_share/mujoko_debugging_data/20261011_124843_centered_group_approach/`.
요약: `nvidia-sim/harvest_s300/validation/centered_group_approach_20261011.json`.
앞선 개방·2회 반복 변경과 함께 코드·문서·검증 요약을 커밋에 포함한다.
사용자 gripper-design 변경은 별도 보존하며 영상·대형 상태 기록은 공유 폴더에 유지한다.

## 2026-10-11 한 번 진입에서 닫기→완전 열기2회

`belt_open_close_cycle`은 진입 후 벨트 연속 회전 상태로 닫기/열기2회를 완료한다.
마지막 열림도 벨트 구동에 포함하고 개방 유지0.5초 동안 감속한다. 실제96mm 개방
확인 후 벨트 정지·열린 상태 후퇴. 이전처럼 마지막 열림 전에 벨트를 끄지 않는다.
접근 전 정지 개방, 동일 수평 접근/벨트 속도/공통1mm/물성은 유지한다.
native1진입 시험: 두 완전 개방 피드백95.999/95.999mm, 초기/실측 자기충돌PASS,
53.5sim초/77.179loop초, 대상4개 분리·3개 남음·대상 밖 분리0.
이번 실행은1진입 예산으로 종료했으며 전체 송이 완료가 아니다. 턱 반복2회와
새 집단으로 재진입하는 횟수를 혼동하지 말 것. 닫힘 중 실제 폭약33~34mm는 과실
접촉을 받는 상태이며28mm 명령에 완전 도달했다는 뜻은 아니다.
offline785개 PASS. 기존 명령 해시4개는 의도한 패턴 변경에 맞춰 갱신, 초기 로그 보존.
결과/영상: `/root/docker_share/mujoko_debugging_data/20261011_123435_two_close_open_cycles/`.
요약: `nvidia-sim/harvest_s300/validation/two_close_open_cycles_20261011.json`.
이번 및 바로 앞 접근 전 개방 수정은 위 중앙 정렬 수정과 함께 커밋에 포함한다.

## 2026-10-11 접근 전 그리퍼 완전 개방

수평 진입은 출발 자세에서 먼저96mm 개방한 뒤 리프트/팔 준비 이동·진입을 한다.
`--preparation-opening ready`가 기본이며, 기존44mm transfer 옵션은 이 모드에서
적용하지 않고 요청값/실제 적용값을 기록한다. 옛 동작 재현은 `legacy-staging` 명시.
각 이동 직전 실제 개방 피드백을 확인한다. 자세/방향/벨트/물성과 공통1mm 기준 유지.
같은 Truss_02/TGS60/64 실제1회: 개방 중 팔 명령 변화0, 이동 중 실측95.984~96.000mm.
53sim초/78.720loop초, 대상5개 분리 후 남은2개로 최소3개 규칙 종료, 대상 밖 분리0.
초기/전체 실측 자기충돌 검사PASS, offline784개 PASS. 다음 사이클도 코드상 같은
개방/검사 순서를 사용하나 이번 native 실행에서는 두 번째 사이클이 발생하지 않았다.
결과/영상: `/root/docker_share/mujoko_debugging_data/20261011_122521_open_before_approach/`.
문서: [HARVEST_GROUP_POLICY_KO.md](nvidia-sim/harvest_s300/HARVEST_GROUP_POLICY_KO.md).
요약: `nvidia-sim/harvest_s300/validation/open_before_approach_20261011.json`.
코드·문서·검증 요약은 위 중앙 정렬 수정과 함께 커밋에 포함한다.

## 2026-10-11 공통 겹침1mm 기본 적용 · 집단 수확 속도 비교

사용자 요청으로 현재 Isaac S300 수확 실행기의 공통 active-contact 중단값을1mm로
변경했다. 초기/실행/계산 후 형상 확인에 같은 값을 적용하고, 과실별 override를
생략하면 상속한다. 물성·충돌체·계획 충돌 여유는 유지한다. 과거0.5mm 결과 보존.
`--contact-limit-mm .5`로 이전 공통 값 선택 가능; 명시적 과실 override도 확인할 것.

알고리즘 커밋4d45485 이후: TGS60/32 + native 접촉 벡터 복사에서54sim초/59.955loop초
(0.901배속), 대상5개 분리/2개 남아 최소3개 기준 종료. TGS60/64는 두 사이클
106sim초/180.854loop초로 대상7개 모두 분리. 대상 밖3개 분리는 둘 다0.
빠른 설정의 미수확 과실 최대 이동약92mm(기존76mm)와 계산 전6.635mm 순간 겹침은
별도 기록한다. 계산 후 검사는 통과했지만 연속 비관통/물리 응답 동등성을 보증하지 않는다.
PGS60/120은 계산 후1.020~1.161mm 겹침으로 중단돼 채택하지 않았다.

782 offline 검사PASS. 복사 전후3,241시점·222,044접촉점 완전 일치.
빠른 solver/Hz/복사는 선택형이며 GUI/병렬 성능은 이번 미측정이다.
문서: [COMMON_CONTACT_LIMIT_KO.md](nvidia-sim/harvest_s300/COMMON_CONTACT_LIMIT_KO.md).
결과/실제1배속 영상2개: `/root/docker_share/mujoko_debugging_data/20261011_114941_common_1mm_harvest_speed/index.html`.
기존 알고리즘과 물리 실행/저장/재생기를 재사용했다. 아래0.5mm 표기는 당시 기록이다.

## 2026-10-11 최소 수량 기반 집단 수확 정책

`--harvest-group-policy`로 동일 송이의 대상 열매가 그리퍼 범위 안에3개 이상인
집단을 높은 쪽부터 선택하고, 실제 사이클 후 남은 수량/위치로 재평가한다.
고정 위/아래2회나 남은1개까지 개별 수확하는 정책이 아니다. 원래 모드는 보존했다.
현재 대상 표시는 자산의 빨간 RGB를 이용한 SIM-GT 대용 기준이며 카메라 숙도 인식은 아니다.
Truss_02 한 사이클: 대상7개 중5개 분리, 남은2개로 종료, 대상 밖3개 분리0.
초기/실측 자기충돌 검사PASS, 54sim초/152.656loop초. offline773테스트 PASS.
상세: [HARVEST_GROUP_POLICY_KO.md](nvidia-sim/harvest_s300/HARVEST_GROUP_POLICY_KO.md).
결과: `/root/docker_share/mujoko_debugging_data/20261011_112538_ripe_group_harvest/`.
학습 데이터 아님, 분리와 포집 성공은 구별한다. 앞선 미커밋 수확 기반 작업을 함께 보존한다.

## 2026-10-11 잎 접촉 제외 후 Truss_02 전체 10개 분리 확인

사용자 확정: 잎 때문에 회피하거나 수확을 중단하지 않는다. `leaf_contact_policy.py`로
별도 `assembly_leaves_visual_only`를 만들었다. 원본 잎 외형은 유지하고 정확한 잎 충돌체
8,152개만 비활성화했다. 잎의 저항을 생략하는 근사이며 실제 잎 접힘 모델은 아니다.
원래 주줄기·잘린 가지·꼭지·과실·로봇·거터 속성과 활성 접촉 0.5mm 기준은 유지한다.
모든 비잎 속성 일치 검사, 잎 제외/거터 유지 등 자동 테스트3개 통과. 원본 자산 보존.

정상 초기 상태에서 양쪽 안쪽50/50mm/s로9개 분리. 11번만2회 미분리였으며,
262초 완료 체크포인트의 실제 상태를 복원해 안쪽20/50mm/s로1회 추가 시도했다.
11번은26.083초 close 중 native 분리, 후퇴·복귀까지 완료. **같은 송이의 고유 열매10/10**,
첫 실행9개+재개1개다. 단일 중단 없는 엔진 실행이나10개 신규 콜백으로 중복 집계하지 않는다.
두 실행의 실측 전체 로봇 자기충돌 검사PASS, 잎 접촉 기록0건. 좌우 회피 보정0mm.
추가2mm 잎 회피 시험은 사용자 지적으로 폐기 방향으로 분류해 결과 폴더에 보존했다.

이미 분리된 열매의 바닥·기체 접촉에는 기존 진단 예외가 적용된다(첫 실행12접촉점,
재개7접촉점). 낙과 충돌 정확도/포집/실물 파단/다른 송이 반복 성공은 미검증이다.
`training_eligible=false`, `harvest_success=null`, `collection_verified=false` 유지.
결과와 실제 저장 상태 영상:
`/root/docker_share/mujoko_debugging_data/20261011_095724_inward_full_truss/index.html`.
명령은 같은 폴더 `leaf_ignored_recipe.json`, `remaining11_20_50_recipe.json`.
요약 `nvidia-sim/harvest_s300/validation/inward_leaf_ignored_full_truss_20261011.json`.
현재 변경은 미커밋. 아래7/10 잎 중단 기록은 이 정책 적용 이전의 결과다.

## 2026-10-11 양쪽 벨트 안쪽 이송 재시험 — 첫 닫힘 7/10 분리, 잎 접촉으로 중단

기존 fresh Truss_02 9/10 모델/경로/120Hz/물성/0.5mm 기준으로 재시험했다.
`--belt-speed-m-s 0 --belt-infeed-speed-m-s .05`가 양쪽 접촉면의 안쪽 이송이다.
기존 common-infeed 부호 오류를 실제 native 구체 이송으로 확인해 수정했다:
local-Y API 명령 -20mm/s는 구체를 입구(+Y)로40.008mm, +20mm/s는 안쪽(-Y)으로39.984mm 이동시켰다(2초).
새 계획에 `native_positive_local_y_inward_v2`를 기록한다. infeed=0인 기존 반대 방향 명령은 그대로다.
저장된 옛 음수 common-infeed 결과를 안쪽 이송 성공으로 해석하지 않는다.

정상 초기 상태, 벨트 시작19.5초까지 기존 baseline의277바디 pose/관절/명령 완전 일치.
수정 방향에서는 첫 close에서01,02,03,04,05,07,08번7개 native 분리(기존 같은 구간3개).
26.025초에 `L/part_209`(CAD `L_C13_outer_finger_cover`)와 `Leaf_CompoundLeaf_04/Appendage_140_0`
접촉으로 중단. post-solve convex 상한0.503177mm, callback 약0.506070mm, 최대 접촉점힘11.402N.
09/10/11은 미분리. 실제 자기충돌 검사PASS, 전체 수확/포집 성공은 아니다.
26.025sim초/102.392실행loop초. 기존418초9개 결과와 전체 성공률을 단순 비교하지 않는다.

첫 부호 오해 실행과 중단한 재실행도 보존했다. 그 과정에서 이미 분리된 열매의 정적
`Foundation_6x10m/Mesh` 낙과 분류 누락을 수정했다. 정확한 정적 바닥·native 파단 이후에만
기존 opt-in 낙과 예외를 적용하며 벨트/팔/붙은 열매/잎의 물리와0.5mm 기준은 유지한다.
관련95자동 검사PASS, 방향 native 대조PASS. 결과/동일 시점 실제 상태 영상:
`/root/docker_share/mujoko_debugging_data/20261011_085604_both_belts_inward_harvest/index.html`.
요약 `nvidia-sim/harvest_s300/validation/both_belts_inward_20261011.json`.
이번 변경은 미커밋. `training_eligible=false`, `harvest_success=null`, `collection_verified=false`.

## 2026-10-11 잎의 수동 탄성 연결과 수확 재시험 — 수확은 미완료

`nvidia-sim/harvest_s300/compliant_foliage.py`로 별도 모델에 활성 식물의 잎27개를
수동3축 연결로 분리했다(원래17+V9추가7+어린잎3). 초기 외형/충돌체/원래 충돌 제외 관계 보존.
식물612→693DOF, 전체248→275강체. 잎 질량·강성은 미보정 추정이며 잎자루 회전만 표현한다.
정지 통과. 초기17개 모델의0.05N/1초 하중은28.506° 회전 후 잔류0.021°로 복원했다.
층별 USD overlay 초기 발산을 동일 합성 형상의 materialized USD로 해결했다. 내부 엔진 원인은 미확정.

수평 송이 방향163.905°/상단01·02번 기존 경로 재사용(명령 동일성 확인).
초기17개 모델에서 native callback1.415mm와 계산 후 cooked형상 겹침 상한0.304mm가 달랐다.
`--verify-leaf-contact-after-step` 선택형 진단은 **판정 시점이 달라 기존 callback 검사와 동등하지 않다**.
0.5mm 수치와 원래 기본 검사 보존. 확인 불가/초과는 중단, 스텝 사이 비관통 보증 아님.
최종27개 모델도12.700초 진입 중 잎16/고정 배경 잎 접촉0.616mm에서 중단, 분리0.
목표01/02 최대 이동82.0/79.3mm. 접촉점 최대힘은 초기17개6.021N→27개2.450N이나 정상 수확은 아니다.
65→45mm 얕은 경로는 link4/link2 여유1.830mm<기존2mm로 계획 차단(실제 관통과 다름).
주변 잎은 여전히 고정 triangle mesh. 잎의 국소 굽힘/축 방향 압축과 밀린 송이에 대한 접근은 미해결.

최종55테스트PASS. 결과/실제 상태 영상:
`/root/docker_share/mujoko_debugging_data/20261011_004347_compliant_leaf_harvest/index.html`.
최종 모델 `assembly_all_leaves_v2`, 작은 요약 `nvidia-sim/harvest_s300/validation/compliant_foliage_20261011.json`.
상세 `nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md` 마지막 절.
별도 모델/결과만 생성, V6 사용자 작업 보존, 미커밋. `training_eligible=false`, `harvest_success=null`.

## 2026-10-11 Isaac 그리퍼 탑뷰 송이 방향 yaw 정렬

`nvidia-sim/harvest_s300/run_motion.py --approach-yaw-mode truss-axis` 추가.
붙어 있는 열매의 XY 주축으로 그리퍼 yaw/전진 경로를 맞추고 pitch/roll은 수평 유지.
기존 robot-radial 기본과 sequence60 명령은 보존. 현재 송이 전진각은 월드163.90°,
기존175.94°와12.04° 차이. 아래10·11번은 자기충돌 계획 검사에서 차단, 위1·2번은
계획 통과 후 실제60Hz에서 옆판/주줄기 부속 형상 겹침으로10.02초 중단(분리0).
방향 수정 완료와 수확 성공을 구분한다. 관련43테스트와 기존 실제 USD 명령 회귀 통과.
결과 `20261011_001548_truss_yaw_alignment/index.html` (docker_share 기존 data 아래),
상세는 `nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md` 마지막 절.
이번 변경은 아직 커밋하지 않았다. 사용자 gripper-design V6 작업은 보존한다.

## 최신: 한 송이 전체 연속 수확 시도 — 미완료 (2026-10-10)

`--whole-truss`로 같은 native scene에서 남은 열매 위치를 읽어 다음 접근을 계획한다.
cycle 사이 식물/열매/로봇 상태 reset이나 파단 조인트 복구가 없다. 현 Truss_02의10개를 정확한
native joint ID로 집계하며, 이미 분리된 열매는 다음 목표에서 제외한다. 기존 runner/IK/FCL/분석/영상 재사용.
7회 물리 시험: 아래부터60/120/240Hz, 위부터60/120Hz, 위부터 높이 선이동60Hz,
높이 선이동+진입 깊이65→35mm60Hz. **전체 수확 완료0회, 가장 많은 분리5/10개**.
아래부터60Hz는7·8·9·10·11번 분리 후9번 낙하/바닥1.329mm 겹침 중단.
120/240Hz는4개 분리 후7번 과실/9번 꼭지 가지0.534/0.588mm 겹침 중단.
위쪽 직선 준비는잎에 차단. 높이 선이동으로 접근은 했지만 송이 밀림/주변잎1.831mm 겹침 중단.
얕은 진입은 최대겹침0.279mm이나 분리0, 실제 재개방91.1mm로96mm 미도달하여 재닫기 전 중단.
나머지01·02·03·04·05를 정상 수확했다고 보고하지 않는다. 다음은 움직인 송이를 고려한 포획 위치/장애물 회피 경로가 필요하다.
물성·3N/0.008Nm·충돌체·actuator·0.5mm 기준 보존. 기존 탄성 연결과 벨트 접촉면 시험안을 재사용한 미보정 진단이다.
자동 관련100테스트PASS, 실제 자기충돌7회PASS, 연속 timestamp/측정 위치 연결 확인.
60Hz 첫39초 명령/관절/식물 pose가 이전2개 분리 실행과 bit-exact. 실패도 모두 보존.
결과 `/root/docker_share/mujoko_debugging_data/20261010_203914_whole_truss_harvest/index.html`.
상세는 [CONTROLLED_BELT_HARVEST_KO.md](nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md) 마지막 절.
`training_eligible=false`, `harvest_success=null`. 현 턴에서 커밋/푸시하지 않음. 사용자 별도V6 변경 보존.


## 최신: 연속 벨트 개폐 패턴 및 닫기 중 분리 허용 (2026-10-10)

사용자 정정: 열린 채 벨트 시작 → 원래 최소28mm까지 닫기 → 벨트 유지하며96mm 완전 열기
→ 다시28mm 닫기. 닫기/열기 중 분리는 실패가 아니다. 이후 벨트를 끄고 열어 후퇴한다.
`belt_open_close_cycle`/live `cycle` opt-in 추가, 기존 명령/물성/파단값 보존.
양쪽 표면−0.05/+0.05m/s, full612DOF, 같은 탄성 연결3N/0.008Nm·5mm 접촉면 시험안.
60/120Hz 모두10·11번 분리 후39초 모션 완주. 모든 활성 접촉0.5mm 기준과 실제 자기충돌 통과.
60Hz 최대겹침0.187mm/47.358초 계산(0.824배속),120Hz0.246mm/90.310초.
60Hz 둘 다 첫 닫기 중 분리;120Hz10번 닫기 중/11번 재개방 시작 때 분리.
재닫기 전 실제95.994mm 개방 확인. on/off 명령/시간 및 벨트 이전 실제상태 일치.
벨트off도10번 분리이므로 모든 분리가 벨트 덕분인 것은 아님. 실물 보정·포집·240Hz 미검증.
기존 이송(좌−0.10/우0)60Hz는2개 분리 후11번/벨트0.632mm 겹침 중단,120Hz 완료.
headless5회와 실패를 모두 보존,91테스트PASS. training_eligible=false/harvest_success=null.
결과 `/root/docker_share/mujoko_debugging_data/20261010_201647_continuous_belt_cycle/index.html`.
실행 `bash /root/docker_share/mujoko_debugging_data/20261010_201647_continuous_belt_cycle/run_cycle_gui.sh`.
상세: [CONTROLLED_BELT_HARVEST_KO.md](nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md) 마지막 절.
기존 아래 기록의 '닫기 중 조기 파단' 표현은 벨트off 닫기였던 당시 비교의 해석이며 현재 성공 조건으로 적용하지 않는다.


## 최신: 잡는 간격·높이 개선 및 실제 화면 재검증 (2026-10-10)

같은 탄성 연결 시험안(3N/0.008Nm)에서 간격40mm/기준점보다10mm 낮은 진입으로
60·120Hz 모두 닫기 중 조기 파단 없이10번 과실이 벨트 단계에 분리되고 후퇴까지 완료됐다.
60Hz GUI도 원래0.5mm 기준으로 완료: 최대 겹침0.192mm,30.5sim초/38.586wall초(0.790배속).
headless와 실제 robot q/식물 pose/명령/시간이 bit-exact. 두 Hz의 벨트off 대조는 분리0.
전체 headless15회+GUI1회, 자동 테스트89개 통과. 실패/분리0도 모두 보존했다.
240Hz/40mm는 닫기 중 조기 파단,41·42mm 비교도 주파수에 따라 분리 여부가 달라
시간 간격에 수렴한 수확 모델을 완성한 것은 아니다. 실물 보정/포집/새 병렬 성능은 미검증.
기존 원본·물성·충돌체·actuator 보존, 간격/높이 opt-in과 live 환경변수만 추가했다.
target 복사 메타데이터의 파단값을 실제 USD와 맞춘 수정도 포함하며 USD 해시는 이전과 같다.
상세/재실행: [CONTROLLED_BELT_HARVEST_KO.md](nvidia-sim/harvest_s300/CONTROLLED_BELT_HARVEST_KO.md).
결과 `/root/docker_share/mujoko_debugging_data/20261010_185843_controlled_belt_harvest/index.html`.
커밋 여부는 `git status`로 확인한다. 별도 gripper V6 작업은 수정하지 않았다.

## 최신: 열매–꼭지 탄성 연결 비교 (2026-10-10)

기준 `76479ee`, 별도 assembly에서 현재 송이10개 과실 연결만 고정→수동 탄성 회전으로 변경했다.
3N 유지, 파단 토크0.08/0.008Nm를 고정/탄성으로 나누고 벨트off 대조 및120Hz를 포함해 농장6회 실행.
60Hz 탄성/0.008Nm: 10번이벨트단계19.283초에 native 분리, 전체30.5초를37.469초에완료(0.814배속).
같은명령/벨트이전실제상태가일치하는 off대조는분리0. 원래0.08Nm의탄성은11번비틀림46.35°이나분리0.
고정약화는닫기중3개분리후2.098mm겹침중단. 120Hz탄성약화도닫기중8·10번분리후1.099mm겹침중단.
따라서 한 사례의개선증거이며 시간간격에안정적인모델/실물파단/포집성공은아니다. 기본모델/물성/접촉기준보존.
87자동검사와60/120Hz단독fixture통과. native이벤트만으로힘/비틀림/굽힘파단원인을구분할수없다.
상세·영상·재실행: [FRUIT_ATTACHMENT_TEST_KO.md](nvidia-sim/harvest_s300/FRUIT_ATTACHMENT_TEST_KO.md).
결과 `/root/docker_share/mujoko_debugging_data/20261010_182133_fruit_attachment/index.html`.
현재 변경의커밋여부는 `git status`로확인한다. 사용자V6작업은수정하지않았다.

## 최신: 벨트 수확 속도·접촉 허용치 비교 (2026-10-10)

같은 full612DOF/전체충돌/5mm 접촉면 시험안에서30.5초 동작을 비교했다.
기존480Hz221.770초 → TGS120/32는52.691초(약4.21배 개선),10번 native 분리·후퇴 완료.
TGS60/64는38.126초(0.800배속)에 완료하지만 분리0이므로 빠른 모션 확인용으로 구분한다.
현재 송이 열매–그리퍼만1mm로 완화한 opt-in 진단이며, 다른 활성 접촉/초기상태는0.5mm 유지.
0.5mm 초과는60Hz 오른쪽상판–7번0.948mm,120Hz 왼쪽상판–8번0.588mm 각각한물리표본.
새 headless16회/GUI2회,81테스트PASS. GUI도38.526/52.354초, headless와 실제상태bit-exact.
다른저주파설정은최대4.1mm겹침으로제외했다. full식물/물성/파단값은유지하며 새병렬성능은미측정.
상세·모든실패·GUI실행법: [HARVEST_SPEED_CONTACT_KO.md](nvidia-sim/harvest_s300/HARVEST_SPEED_CONTACT_KO.md).
결과 `/root/docker_share/mujoko_debugging_data/20261010_173338_s300_harvest_speed/index.html`.


## 최신: 여러 열매 접촉·그리퍼 접촉면 시험안 (2026-10-10)

- 기존 그리퍼는10·11번 중간으로 진입/6초 닫기/480Hz에서 벨트2초와 개방·후퇴까지 완료했지만 분리0.
- 별도5mm 접촉면 시험안+감속 벨트+scene CCD에서 Truss_02_Fruit_10 native 분리와 전체30.5초 동작 완료.
  원래 CAD/V6 유지, 표시·충돌2개와 질량74.88g을 명시적으로 추가한 시험안이다. 물성/파단값/0.5mm 기준 유지.
- 전체9회(컨트롤러 timeout1 포함) 기록. 빠른 벨트/낮춘 높이/바닥 CCD off 실패도 보존.54테스트PASS.
- 480Hz 검증용이며 실시간 아님, 제작/실물 안전/동시 다수 수확 검증 전. 상세 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md).
- 결과 `/root/docker_share/mujoko_debugging_data/20261010_161007_s300_cluster_grasp/index.html`. 요약 `validation/cluster_grasp_20261010.json`. 학습제외, 커밋 여부는 `git status`.

## 최신: S300 벨트2초 수확 순서 연결·실제 분리 미달 (2026-10-10)

- 기준 `d54b498`, `--harvest-profile belt_spin_release`: 벌려 진입 → 닫기 → 대향 벨트0.15m/s·2초 → 벌려 후퇴. 기존 legacy/entry-only 명령은 그대로다. 실제 송이/CAD/V6작업/충돌체/물성/파단값 보존.
- 같은11번 열매의120Hz는14.0833초 닫기 중 왼쪽상판–옆10번0.5059mm 관통으로중단. 240Hz는14.5875초 오른쪽벨트–목표11번0.5309mm 관통으로중단, 벨트실행0.0875초. 목표분리0/양쪽포획미확보/수확성공아님.
- 목표폭23.053mm 대 최소벨트간격28mm. 전체30개 활성과실의 최대두점거리도27.235mm 이하. 현재 기구의 잡기 범위 및 옆과실 상판간섭이 우선 문제. 고정 과실–꼭지 조인트의3N/0.08Nm 파단은 유지했고 시간에 따른 강제분리 없음.
- `analyze_belt_trial.py`에 실제 단계시간/좌우접촉/열매–꼭지 상대회전/정확한 파단조인트 측정. 관련42테스트PASS, 기존명령 회귀일치,120Hz 진입까지1,381개 실제상태bit-exact. raw/영상: `/root/docker_share/mujoko_debugging_data/20261010_153918_s300_belt_spin_release/`. 상세 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md). 커밋 여부는 `git status` 확인.

## 최신: S300 실시간 GUI 미리보기 (2026-10-10)

- 기준 `56c5d01`, 브랜치 `codex/s300-harvest-isaac-20261009`. 기존 물리 실행기/실제 상태 뷰어를 별도 프로세스로 연결하는 `run_motion_live.sh` 추가. VNC `DISPLAY=:0` 화면 준비 후 시작, 완료 시 상태/접촉/HTML 저장. 새 변경의 커밋 여부는 `git status` 확인.
- 같은 `assembly_v3`의23.5초 진입·유지·복귀: 기존120Hz 동기GUI95.304초 → 분리GUI120Hz40.388초(0.582배), **선택형60Hz23.500초(1.000배)**. full612DOF/전체충돌31,464개/자기충돌/물성/0.5mm관통기준 유지. 로딩·검사·저장시간별도.
- 60Hz는 다른 수치 동역학이다. 같은0.1N 부하 최대변위15.011→16.095mm(+7.23%), 복원확인. reference120Hz 유지. 120Hz 분리GUI/기존headless 전체2,821표본 bit-exact,60Hz 동설정 재현도일치. 최종GUI 두실행 자기접촉0/실제자세검사PASS, 관련53테스트PASS.
- 실행 `bash /root/docker_share/mujoko_debugging_data/20261010_145123_s300_realtime_review/run_gui.sh`; `reference`를 붙이면120Hz. 같은폴더index.html에비교/로그/실제화면. 상세 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md), 요약 `validation/realtime_preview_20261010.json`.
- 열린 그리퍼 진입·유지·복귀만 검증. 닫기/벨트/분리수확/새GPU병렬은 미검증, 학습제외유지. 초기런처/가상화면시작오류도보존. 원본/별도gripper V6작업은유지.

## 최신 수정: S300 팔–그리퍼 자기충돌 수정·실제 재검증 (2026-10-10)

- `codex/s300-harvest-isaac-20261009`, 기준 `aad8340` 이후 수정. 코드·테스트·문서·작은 검증 JSON을 함께 커밋하며 실제 상태/영상/대형 USD는 아래 결과 폴더에 보존한다. native 로봇 자기충돌 활성화, 그리퍼 슬라이더 부모를 실제 고정 레일로 수정하고 초기 IK 자세/접근을 다시 선택했다. 목표11번 유지, 수평 대기160mm → 준비60mm → 진입100mm → 유지 → 복귀. 비인접 팔/그리퍼 충돌 제외 추가 없음.
- 새 `assembly_v3`의 headless/GUI 모두23.5sim초 완주. 각2,821개 실제 자세 교차 검사PASS, native 자기접촉0, 상태/명령bit-exact. 계획120Hz/2mm여유 검사도PASS. 물성/형상/구동게인/0.5mm관통기준 유지, full식물612DOF/활성충돌31,464개 유지.
- 진입pitch최대0.0331°, 진입TCP높이범위0.150mm. 초기중력정착TCP하강8.66mm는 별도 기록. headless38.62초(0.608배속), GUI95.30초(0.247배속), 로딩/검사/저장별도. 첫2회초기관통실패도보존. 관련37테스트PASS.
- 결과/실제영상/실행스크립트: `/root/docker_share/mujoko_debugging_data/20261010_142120_s300_self_collision_fix/`. 상세 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md) 최신 절. 아래 "수정/재실행 미실시"는 과거 기록이다.
- 검증 범위는 진입·유지·복귀다. 과실닫기/벨트이송/수확분리 및 새GPU병렬은 미검증. `training_eligible=false`, `harvest_success=null` 유지.

## 최신 정정: S300 영상 기준 자세에 그리퍼–팔 자기관통 (2026-10-10)

- 사용자 지적 후 실제 저장 pose를 FCL 원본 삼각형 교차로 검사했다. 대기·준비·복귀에서 그리퍼 프레임/레일과 RB5 `link2`의 교차 확인. 로봇 `enabledSelfCollisions=false`이므로 기존 native 접촉 감시가 검출하지 못했다.
- 아래23.5초 GUI 완주는 명령 완료이며 **유효한 비충돌 경로가 아니다**. 28테스트/수평FK 통과도 자기충돌 검증이 아니다. 기존 원본·영상·접촉은 보존하고 보고서에 후속 정정을 추가했다. 초기 자세·경로·자기충돌 설정의 수정/재실행은 아직 하지 않았다.
- 12개 저장 시점 검사,2초24개/4초28개 메시쌍 교차. 깊이/힘 추정이나 연속 비충돌 보증 아님. 근거 `20261010_141508_s300_self_collision_audit/mesh_intersection_audit.json`; 자세한 정정은 [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md).

## 최신: 영상 기준 수평 대기·진입·복귀 완료 (2026-10-10)

- inaho 영상49~69초를 확인하고 S300/RB5에 `horizontal_ready`를 추가했다. 초기부터 수평 대기, 96mm 선개방, Cartesian 수평 접근/진입을 사용한다. 하부 `L_T01_sloped_receiver` 및 전용 체결부4개의 외형·충돌체와 추정질량167.664g을 제거했다. 다른 활성충돌체/식물속성은 보존했다.
- 바깥쪽 `Truss_02_Fruit_11`의 GUI `--entry-only`가23.5sim초 전체완료: 진입→2초유지→후퇴→대기복귀. 실제진입pitch최대0.0171°, 전체TCP높이변화0.256mm, 목표변위최대0.0316mm, 관통중단없음. 수확시도/성공은 아니다.
- 같은11번의 전체동작은 진입후 닫기13.233초에 왼쪽상판과08번과실의1.522mm관통으로중단. 기존02번은열매/잎충돌로진입중단. 모든시도·초기USD구성실패도보존했다.
- GUI23.5sim초/92.714wall초=0.253배속. 공통0~11.5초의GUI/headless실제상태·명령1381표본bit-exact. 관련28테스트PASS. 기존수집/학습제외유지.
- 결과 `/root/docker_share/mujoko_debugging_data/20261010_134811_s300_video_entry/`. `run_gui.sh`로완료된진입·복귀시연재실행. 상세·실패·한계·명령: [VIDEO_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/VIDEO_ENTRY_STATUS_KO.md). 커밋 여부는 `git status` 확인.

## 최신: S300 그리퍼 수평 진입 수정 (2026-10-10)

- `codex/s300-harvest-isaac-20261009`, 수정 기준 `aad8340`. 수확 진입 pitch 기본0° 고정, 기존 자동0/-30/-60/+30° 선택 제거. yaw는 명시적인 월드Z 수평 방향 보정이다. 현재타깃 radial0° 준비 IK 미발견, yaw+15/+30°의100mm 수평 경로 FK 통과.
- 기존 SI 실행의 앞 두 준비 구간을 그대로 재사용하는 opt-in과, RB5 wrist1을 보정하는 `tight_fold_level_approach`를 추가. 실제 composed FK/관절 제한을 확인하며 기존 준비 profile은 유지한다.
- 관련24테스트PASS. 실제5회 모두 approach 전 주변/활성 식물 접촉·관통으로 중단했다. 원본 prefix 재사용3회는 처음12초 명령·time·phase·실제bodypose가 원본과bit-exact였다. 수평 진입 완주나 수확 성공은 미확인이다.
- 기록/영상: `/root/docker_share/mujoko_debugging_data/20261010_125732_s300_horizontal_entry/`. 변경범위·모든실패·GUI명령: [HORIZONTAL_ENTRY_STATUS_KO.md](nvidia-sim/harvest_s300/HORIZONTAL_ENTRY_STATUS_KO.md). 새 결과는 학습 제외이며 커밋 여부는 `git status` 확인.

## 최신 정정: S300 native stage 단위·축 및 SI 재검증 (2026-10-10)

- 첫 118개 성능/수확 연결 trial의 assembly root에 `metersPerUnit`·`upAxis`가 빠져 실제 stage가 0.01m/unit·Y-up fallback으로 동작했다. 아래 이전 속도·변위·힘 기록은 역사적 수치이며 **검증된 SI 농장 성능으로 해석하지 않는다**. 같은 잘못된 stage의 bit-exact PASS도 SI 타당성 증거가 아니다. 기존 결과와 코드2964d4a는 보존했다.
- builder가 농장 source의 metre/Z/kg metadata를 전파하도록 수정했고 native 실행 전 fail-fast 검사를 추가했다. 올바른 metadata만 씌운 wrapper는 초기 파단에 실패했으므로 원인이 단위 숫자만이라고 단정하지 않는다. 실제 schema/spec을 다시 생성한 `legal_motion_assembly_metre_z_rebuilt`를 새 기준으로 사용한다.
- 새 full SI 모델은 235바디/612 DOF/605 활성 collider와 배경 전체 충돌을 유지한다. PGS120/64/4의 7초0.1N 부하·복원은10.5246loop초에 완료(0.665sim-s/s), 초기화·관통 통과/파단0, 목표 최대7.018mm/종료0.0104mm다. 이전 24worker7.04sim-s/s를 새 SI 처리량으로 승격하지 않는다.
- 새 SI 독립 CPU8/24worker의 합산 처리량은5.915/7.019sim-s/s, 준비·저장 포함2.059/2.835sim-s/s다. peak RSS 합29.77/89.23GiB. full 32worker 전부 단일 SI 기준과 pose/native q/명령/접촉 bit-exact였다. 234DOF/TGS60의 별도24worker13.438sim-s/s는 바뀐 동역학이며 full 모델과 동등하지 않다.
- 새 SI 실제 wide-open 접근은29.917sim초/49.452wall초에 목표 과실–벨트 관통0.536mm로 중단, 최대 밀림43.457mm/파단0이다. 닫기·벨트 구간까지 진행하지 못했고 수확 성공이 아니다. full SI의30mm 합성 구 대조군은40.560mm 이송했지만 D6 guide fixture이며 농장 과실 수확을 입증하지 않는다.
- cold layer를 새로 여는 CPU namespace clone1은 full SI 기준과 pose/q/명령/12900접촉 bit-exact로 정상화됐다. GPU2는 capacity 오류 및 SDK의 readback reset을 구분·수정한 뒤 실제 자기 식물 접촉2.653mm/파단30개로 중단했고, 마지막 GPU 활성화 지연 대조도 실패했다. 기록된 cross-env 접촉0; native 원인 미확정. GPU8은 입력만 준비, **미실행**이다.
- 최종 설치 SDK 검사는151개 중150PASS/1skip(21.031초), 최종 보고서20개 추가 검사PASS. Torch 미설치로 skip된 unsigned actor-ID 저장 검사는 별도 Python3.11/Torch에서1PASS했다. 실제 SI 접근 전체/접촉 확대 영상과 guided ball 벨트 대조 영상3개는 저장 pose 재생이며 물리 재실행 없이 전체 디코드를 통과했다. raw 로그/MP4/입력은 `20261010_s300_performance/`, 작은 검증·해시는 `nvidia-sim/harvest_s300/validation/performance_20261010.json`에 보존한다.
- 새 SI pool/접촉/실제 접근 결과와 제외한 빠른 설정은 [PERFORMANCE_STATUS_KO.md](nvidia-sim/harvest_s300/PERFORMANCE_STATUS_KO.md)의 최신 절과 `20261010_s300_performance/index.html`에서 확인한다. 목표 포획·분리 수확 및 유효한 GPU 병렬 처리량은 아직 미확보다. 원본/실패/학습 제외를 유지한다.


## 최신: S300 Isaac 속도 최적화·24 CPU 환경 검증 (2026-10-10)

- 브랜치 `codex/s300-harvest-isaac-20261009`. 원본 USD/CAD/실험을 보존했다. 상세 [PERFORMANCE_STATUS_KO.md](nvidia-sim/harvest_s300/PERFORMANCE_STATUS_KO.md), 코드/검증의 커밋 여부는 `git status` 확인.
- 새 `legal_layout_v2`에서 기존 합법 Y회전/부착 배치만 조정해 활성 식물의 초기 겹침을 해결했다. target Truss02는 그대로이며 메시·충돌체·질량·관성·탄성·파단 기준을 유지했다. 배경 충돌을 끄지 않았다. 활성 Plant22는 235바디/612 DOF/605 활성 collider, 주변 식물은 full triangle 고정 장애물이다.
- 최소 native app의 `--physics-only`, 물리와 별도인 표시/replay, 접촉 버퍼를 추가했다. 실제 CPU/GPU dynamics·broadphase·Hz를 검사한다. 초기화 중 중력만 중립화하고 고정 preload 계산 뒤 원래 중력을 복구한다. 초기조건 개선과 bit-exact 코드 최적화를 구분했다.
- CPU PGS120/64/4, full 식물의 동일7초 부하·복원 시험: 단일10.647초(0.657 sim-s/s); 병렬2/4/8/16/24개는 합산1.289/2.329/5.873/6.564/7.042 sim-s/s. **24개는168sim초/23.855loop초, 시작·저장 포함59.821초**다. peak RSS 합계 약87.36GiB(공유 페이지 중복 집계 가능). 효율 기본8개, 최고 실측 처리량24개. 모든54 worker의 바디 자세/관절/접촉이 단일 기준과 bit-exact였다.
- 각 환경은 목표 식물 한 개만 full 탄성이며, 모든106송이의 동시 탄성화나 병렬 수확 성공률 시험이 아니다. 단일 PGS120는 아직 실시간 미달이다. GPU dynamics는 느렸고 native namespace 복제는 초기 파단으로 invalid; GPU 대규모 학습 준비 완료로 해석하지 않는다. CPU/SAP는 차이 없음, CPU solver+GPU broadphase는 더 느렸다.
- 60Hz PGS는 부하 벨트 접촉에서 관통0.5mm를 넘었다. TGS64/60은 부하 벨트 대조군은 통과하지만 탄성 응답이 달랐다. 120Hz를 모든 접촉의 보증으로 취급하지 않는다. 충돌/마찰/물성/actuator/기준을 바꿔 성공시키지 않았다.
- 새 좁은 회전 준비 경로는 목표 앞까지 실제 도달했다. 준비 후44→60mm 개방도 시험했다. 실제4회는 삽입 단계의 과실/자기 가지 또는 잎 접촉·관통으로 중단했고 닫기·벨트는 미도달. 240Hz의30.7417초에 실제 목표 꼭지 파단이 발생했으나 다음 스텝에 관통 중단되어 검증된 포획/수확이 아니다. 유한30과실/기존 경로 검사에서 대체 clear 경로 미발견이며 절대적 불가능 판정은 아니다.
- 30mm 합성 구의 native 벨트 이송 약40.56mm는 확인했으나 별도 D6 guide 시험이며 농장 수확 성공이 아니다. 모든 결과 `training_eligible=false`, `harvest_success=null` 유지.
- 테스트120개 중119통과/별도 SDK 경로에서 Torch 검사1skip. 실제 native 실행은 Torch tensor를 사용했다. 결과/실제 저장 상태 MP4: `/root/docker_share/mujoko_debugging_data/20261010_s300_performance/index.html`. 호스트 `file:///home/rdv/docker_share/mujoko_debugging_data/20261010_s300_performance/index.html`.
- 다음 병목은 준비가 아니라 밀집 송이의 insert/grip 경로다. 기본 경로는 −60° 기울기를 선택해 receiver가 다른 Fruit04를 먼저 밀고, 목표 Fruit02는 왼쪽 벨트 앞단에서 밀리며 의도한 그립 깊이에 들어오지 않는다. 자세/경로와 실제 추종·접촉을 함께 개선해야 한다. persistent worker reset/다양한 수확 동작의 병렬 동등성은 아직 미검증이다.

## 최신: S300 수확 구동·탄성 연결 시험 (2026-10-10)

- `codex/s300-harvest-isaac-20261009`. 원본 USD/CAD 보존, 별도 `assembly_v5/scene.usda`에 V5.1 등가 개폐/벨트 표면 속도와 Plant_22 full 탄성 연결.
- 뒷판↔리프트 선반 교차를 발견해 새 동작 모델의 리프트·RB5만 Y−0.20m 이동. 뒷판 유지. 기계 체결 검증은 아님.
- 분리 시험: 개폐 실제28.02~60.01mm,0.1N 과실 외력 최대7.18mm/제거 후0.43mm 복원. 240Hz CPU 실제 상태/접촉 저장.
- 원래 농장은 초기 잎 겹침 최대12.22mm로 중단. 분리 시험은 배경 식물↔활성 물체 및 활성 잎↔식물 내부 접촉을 명시적으로 제외했으며 full 농장 성공이 아님.
- 접근3회: 지지선/유인선 차단 뒤 접고 회전하는 경로로 개선했으나14.73초에 잎 접촉·관통0.507mm로 중단. 잎 밀림으로 과실62.85mm 이동. 실제 수확0, `harvest_success=null`, 학습 제외.
- 과실 약26mm 대 CAD 최소 간격28mm 미해결. 그리퍼/과실 크기는 임의 수정 안 함. RB5 원본 자기충돌false도 유지되어 자기충돌 경로 검증 추가 필요.
- 결과/실제 상태 MP4: `/root/docker_share/mujoko_debugging_data/20261010_s300_harvest_motion/index.html`. 자산10검사/기구학4검사 통과.
- 상세·명령·실패 포함 기록: [MOTION_STATUS_KO.md](nvidia-sim/harvest_s300/MOTION_STATUS_KO.md). 이번 변경의 커밋 여부는 `git status` 확인.

## 최신 정정: 하우스 0.5 크기 복원 (2026-10-10)

- 기존 `farmily_greenhouse_robot.usd`의 하우스 배율0.5 확인. S300 생성기 기본값1.0은 잘못 가져온 크기였으며0.5로 수정.
- 줄기68개/송이106개 뿌리·부착 세계 좌표도 검토된 배치로 복원(행 내25cm). 식물 template 자체 크기는 유지.
- S300/리프트/RB5/그리퍼는 축소하지 않고 x=0.55→0m로 통로 중앙 배치. 원본 메시/조립 관계 보존.
- 기하27검사+크기/보존8검사 통과. 현재 Isaac 창 적용. 결과 `/root/docker_share/mujoko_debugging_data/20261010_s300_restored_house_scale/index.html`, 최종 `assembly_final/scene.usda`.
- 이번 변경은 크기/배치 정정. 새 접촉·탄성·수확 물리 검증은 미실시. 아래1.0 배치 기록은 이전 상태다.

## 최신: Isaac 농장의 줄기·잎·송이 자산 정정 (2026-10-10)

- S300 조립에서 옛 v2 농장 식물을 사용했던 것을 수정. `build_scene.py` 기본 `--farm-layout reviewed`, 옛 장면은 `legacy`로 재현.
- 원본 v8 줄기 36개 + v9 잎 줄기 32개, GLB 5종 송이 106개. 기존 Y축 0~180°/송이0.5배/Fruit06 제외/white 보정 유지. 원본 USD/GLB 면·정점은 줄이지 않음.
- 하우스 1.0 유지, 기존 0.5 하우스 배치의 **뿌리 좌표만 2배**로 맞춤. 식물 자체 크기는0.5 유지. 행 내 명목 간격25→50cm이며 이전 MuJoCo 장면과 세계 좌표/밀도가 동일하다는 뜻은 아님.
- S300 뒷판·휠 커버 수정 및 로봇 조립 보존. 기하27검사/로봇보존5검사 통과, 실제 Isaac6.1 전후 동일 카메라·근접·양측배치 촬영.
- 최종 화면/결과: `/root/docker_share/mujoko_debugging_data/20261010_s300_latest_plants/index.html`, 장면 `assembly_final/scene.usda`. 현재 Isaac 창에도 반영.
- **고정 full triangle 충돌체 상태이며 탄성 연결/수확 물리/새 속도 시험은 미실시.** `training_eligible=false` 유지. 상세 [harvest_s300/README.md](nvidia-sim/harvest_s300/README.md).

## S300 수확로봇 · Isaac Sim 6.1 조립 (2026-10-09)

- 새 브랜치 `codex/s300-harvest-isaac-20261009`, 시작점 V5.1 그리퍼 수정 `b890fc6`. 이전 작업과 원본 자산 보존.
- Isaac Sim 6.1 standalone을 `/root/isaac-sim-6.1.0`에 별도 설치, 공식 MD5 일치. 기존 5.1 환경 유지.
- 원본 54식물 농장 USD의 형상·잎·재질 보존, 원본 공간 배율 1.0. MuJoCo light/ultralight 미사용.
- S300 V10 실제 외형에서 물통·펌프·분무부/후방 상부 지지물 제거, 평판 위에 기존 리프트·RB5, V5.1 그리퍼 356부품 부착.
- 13개 원본/기하/좌표 검사 통과, 실제 6.1 렌더 3시점 저장. GPU PhysX 120Hz 정지 2초/240스텝에 1.662초, 팔 끝 최대 이동 0.120mm. 초기화 별도, 수확/접촉 성능 시험 아님.
- **현재 농장 식물은 고정 triangle 장애물이며 탄성 모델 연결 전. 그리퍼도 44mm 개방 고정 상태.** 기존 `elastic_plant.py` full+keep 연결, 새 도구 하중/구동/체결 검증은 남아 있다. 학습 데이터 사용 금지.
- 사용법/검증/한계: [nvidia-sim/harvest_s300/README.md](nvidia-sim/harvest_s300/README.md).
- 결과: `/root/docker_share/mujoko_debugging_data/20261009_s300_harvest_isaac61/index.html`, 최종 USD `assembly_v3/scene.usda`.
- VNC 조작 지연 조사: 내부 정지 약161 FPS/회전 표본 약117 FPS로 렌더 정지는 재현되지 않음. 정지 GUI 기본30 FPS 제한·FPS 표시 추가, Play 시 원래 갱신 설정 복귀. 짧은 GUI 표본 CPU528→124%, 화질/형상/충돌/물리 smoke 유지. 원격 체감 개선은 사용자 확인 필요. 원자료 `20261009_isaac_viewport_diagnosis/`, 실행 옵션 `--viewer-fps 0`은 원래 무제한.
- 외형 정정: 리프트 뒤 원본 뒷판 1개 복원, 메카넘 휠 커버 및 전용 체결부 96개 제거(바퀴 유지). 새 장면 `20261009_s300_rear_panel_no_wheel_covers/assembly/scene.usda`, 전후 사진 `index.html`. 기하15검사·변경범위4검사 통과, 기존 자산/결과 보존. 현재 Isaac 창에도 반영.

## 힘·변위 중단값 비교 (2026-09-28 추가)

- 사용자 요청으로 제한값을 변경한 추가12회. 기준b75e6bc, 별도 `codex/contact-limit-sweep-20260928`. 모델/경로/물성/운영정책 유지.
- 00번 원래 속도·감속에서2N/5mm,1N/10mm,2N/10mm 비교, 기존5경로+감속00에서5N/20mm 비교.
- 전체 검사12/12 통과. 실제 전체 동작 완료3, 목표 힘 중단1, 변위 중단5, Rachis 비허용 접촉 중단3. 꼭지 접촉 기반 안착·유지 확인0.
- 5N/20mm에서00·04·감속00 완료. 실제 최대 과실 힘1.577/1.585/0.899N, 최대 변위10.951/10.628/10.673mm. 기존1N/5mm는 완주 관찰을 막았으나 실물 안전 기준은 미검증.
- 04번 hold1.429초 기하 안착 범위 유지(간격약1.4mm), 목표 꼭지 접촉력0. verify는 일부 기하 범위 이탈. 다음 검토 출발점이며 hook_success로 승격하지 않음.
- 원본해시/timestamp 연속성/종전 기준과 공통 저장 상태 qpos 차이0 확인. 목표 과실 침투0, 전체 최대0.228mm. training_eligible=false, hook_success=null.
- [결과 보고서](mujoco-benchmark/CONTACT_LIMIT_SWEEP_RESULTS_KO.md), [GPT-6 Pro 전달문](mujoco-benchmark/CONTACT_LIMIT_SWEEP_PRO_HANDOFF_KO.md). 결과 `20260928_contact_limit_sweep/index.html`, 실제 상태 영상4개와240Hz 원자료 보존.

## 목표 과실 접촉 허용 재시험 (2026-09-28)

- 기준 `804e40c`, 별도 브랜치 `codex/server-target-fruit-contact-20260928`. 마지막 under5개 원본 경로를 재계획 없이 검사했고 모두 전체 완료·통과.
- 정확한 목표 과실/고리 와이어만 허용. 실제 기존5회는 과실 힘 합1N 초과, 동일 경로 seat18배 감속1회는 변위5.015mm로 중단. 총6회/예산7회, 정상 접근 후 꼭지 안착·유지 미확보.
- 감속 목표 과실 힘 최대0.604N, 실제 TCP 최대2.397mm/s(명목1.942mm/s). 한계 자동 상향 없음. 손상/수확 불가능 판정 아님.
- 기록된 비허용 로봇·환경 접촉력0, 목표 꼭지 접촉력0, 목표 과실 침투0. 전체 활성 접촉 초기 최대침투0.228mm로 기존0.5mm 기준 이내. 중단 이후는 미검증.
- 과거 baseline3.397N에는 목표 과실 접촉도 포함된다. 모두 다른 토마토/줄기의 힘으로 해석하지 말 것. 과거 단일 법선력 최대와 이번 접촉별 힘 크기 합도 구분.
- 테스트188+5 통과, 실제 상태 영상3개/240Hz 증거/원본 해시 보존. 학습 제외, hook_success=null. [상세 보고서](mujoco-benchmark/TARGET_FRUIT_CONTACT_SERVER_RESULTS_KO.md).
- 결과 `20260928_target_fruit_contact_server/index.html`. 충돌체·물성·원본 경로·기존 운영 정책 유지.

## 꼭지 안착 목표·전체 경로 연결 실험 (2026-09-27 후속)

- `f24709c` 기반 별도 `codex/server-hook-seating-20260927`. 실제 gutter02/Tomato02에서 기하 목표32개 생성, 선택12개 IK·새 기하 통과, 전체 도구/환경의 끝 자세5개 통과.
- opt-in `diagnostic_pose_waypoints_v1`: 경유점별 RING 위치·회전을 실제 IK에 반영. 기존 명령2,512개/phase 동일, action14 사용 안 함, 학습 제외.
- 직접 준비12개/우회10개/하부 삽입5개 모두 경로 검사 차단. 마지막에는 안착 상승 중 뒤쪽 와이어가 목표 과실과 겹침. 새 물리0회, 비교 baseline1회만 실행. 정상 진입·안착·유지 동작 미확보.
- baseline 부분 중심 진입이지만 꼭지 안착0; private 재계산 전체 비허용 힘 최대3.397N. 접촉/기하240Hz와 시각 구분 저장. 물리·충돌·성공 기준 유지.
- 자동116+5개 통과, 독립 기하/시간 대조군. 정적 fixture는 자동 진입 성공이 아님. [서버 결과](mujoco-benchmark/HOOK_SEATING_SERVER_RESULTS_KO.md), 출력 `20260927_hook_seating_server/index.html`.

## 환경 사전검사 재검토·접촉력 진단 (2026-09-27)

- 작업 브랜치 `codex/server-preflight-rereview-20260927`, 기준 `bd59ea3`(필수 `1f5bc1f` 포함). 7경로 전체 스캔 완료, 모두 blocked. 상세 저장 상한64개와 클래스별 전체 집계를 구분.
- gutter02는 양의 간격에서도 실제 법선력 최대0.734N. stem01은 고정 장애물에 막혀 회전 관절 추종 오차0.293rad. 무겹침을 무접촉으로 해석하지 않는다.
- 별도 MjData에서 동기화한 힘/기하와 live 직전 solver 힘을 구분. 진단 유무에 따른 전체 qpos/qvel 해시 동일. 물리·성공 판정·운영 정책 유지.
- Tomato02 5회 국소 변경: 기존 부분 중심 진입5/5, 목표 꼭지 안착/유지 지표0/5. 거터 장면의 별도 오프라인 조사이며 preflight를 통과한 수확 결과가 아니다. 학습 제외.
- 자동65테스트, 실행 gate3조건 및 대표4영상. 상세/Pro 전달: [재검토 서버 결과](mujoco-benchmark/ENVIRONMENT_PREFLIGHT_SERVER_REREVIEW_RESULTS_KO.md). 출력 `20260927_preflight_rereview_server/index.html`.

## 환경 사전검사 서버 검증 (2026-09-27)

- 검토 브랜치 38cdb6a 반영, 자동46테스트 통과. 기존7경로 모두 blocked. 거터/주줄기 사례 검출, 실행 보류/판단 유보에서 물리0회 확인. 첫1초 통과 실행은 전체 경로 안전 검증 아님.
- 기존 성공 경로의 계획 겹침과 실제 간격 차이 확인: gutter02 검출 쌍은 새 물리10,044스텝 최소0.866mm. 접촉 기록은 존재하므로 무접촉 증명 아님. 정책/경로 수정 안 함.
- 상세: [1차 서버 검증 보존본](mujoco-benchmark/ENVIRONMENT_PREFLIGHT_SERVER_FIRST_RESULTS_KO.md). 출력 `20260927_environment_preflight_server_review/index.html`, 공유 폴더 `ENVIRONMENT_PREFLIGHT_PRO_FEEDBACK.zip`.

GPT-6 Pro 논의용 현황: [비목표 무접촉 수확 논의](mujoco-benchmark/NONCONTACT_HARVEST_DISCUSSION.md). 공유 폴더에도 동일 파일 저장.


## 주변 식물 탄성화 비용 진단

- `benchmark_neighbor_elastic.py`: 기존 생성기/RobotEngine 재사용, 주변1/4개 고정·탄성 비교. 운영 장면은 변경하지 않음.
- 식물당 송이1개, 240Hz, 동일 명령10초 각2회: 주변1개 3.227→5.029초, 4개 3.409→16.708초. 접근 실행은 관통 기준 실패이므로 정상 수확 성능 아님.
- 정지2초는 모두 관통 기준 통과. 주변4개 0.929→4.006초로 탄성 비용 확인. 전체68개/실시간 타깃 전환 미측정. 타깃만 탄성·나머지 고정 충돌체 방식이 유리할 근거이나 비접촉 보장은 별도.
- 현재3송이 조밀 배치는 주변4개 탄성화 시 초기7.645mm 겹침 발견. 모델 전환 전에 배치 충돌 검사 필요.
- 결과 `20260927_neighbor_elastic_one_truss/index.html`, 정지대조 `20260927_neighbor_elastic_idle/index.html`. 상세 GREENHOUSE_VISUAL.md 마지막 절.


## 로봇 쪽 타깃·주변 주줄기 장애물

- `preview_neighbor_plants.py --stem-obstacles-layout`로 양측 주변68줄기를 고정 캡슐1088개로 충돌 처리. 기존 수확 줄기 탄성 유지.
- 통로 쪽 뿌리 위치 기준 타깃 적격 구분. 실제 실행 대상은 `eligible_targets`의 기존10열매. 주변 복제 송이는 아직 물리 타깃 아님.
- `20260927_stem_obstacle_scene/index.html`: 초기 검사 통과, 저장01/02/09 경로 모두 주줄기 침투1.18/1.49/1.62mm로 invalid_physics. 자동 회피·비관통 해결 완료가 아님.
- 관련9테스트 통과. 주줄기 접촉/침투 metrics 및0.5mm invalid 판정 연결.

## 거터 충돌 추가

- `--gutter-collisions`(새 장면/직접 테스트), `greenhouse_visual.py --gutter-collision-only`(기존 XML) 추가. 원본 USD의 두 거터 판/리브18개를 별도 고정 충돌체로 구성해 열린 내부 유지.
- 거터–팔/고리/식물 관통0.5mm 초과는 invalid_physics. 초기 검사 및 rollout metrics에 거터 포함. FCL 자동 회피는 미연결.
- `20260927_gutter_collision_scene/index.html`: 제어된 옆판/바닥 접촉0.250mm, 기존 경로3개 재실행 완료. 09에서 실제 거터 접촉 발생. 자유낙하 고속 시험은3.328mm로 실패 기록 보존.
- 주변 식물·철사·거터 다리·배지는 여전히 시각 전용. 상세는 GREENHOUSE_VISUAL.md.

## 복잡한 하우스 배경 직접 각도 테스트 완료

- `test_direct_angle.py --existing-scene DIR --physics-hz 240 --gui` 추가. 현재 배경을 그대로 새 RGB-D 촬영→각도 예측→독립 물리 실행에 사용.
- `20260927_dense_house_harvest_test`: 10열매 중 중심진입6/미진입2/가림 미시도2(03,11), 실행8개 물리오류0. GUI 포함620.16초.
- 예측은 모두 -90° 부근에 몰림. 주변106송이는 충돌 없는 외형이므로 장애물 회피/수확 분리 검증 아님. 보고서 미시도/실제 계획 집계 수정.
- 재실행 명령과 범위는 `mujoco-benchmark/GREENHOUSE_VISUAL.md` 마지막 절.

## 주변 식물 랜덤 송이 미리보기

- 양측 배치 커밋 `0317ce6`. 후속 `--random-trusses`로 주변 줄기당 송이 1개를 기존 GLB 규칙으로 랜덤 부착.
- 5종/0.5배/Y회전0~180°/segment4~13/6번 제외/white보정 유지. 기존 부착 계산 공유. 주변은 정적 외형 전용.
- 결과 `/root/docker_share/mujoko_debugging_data/20260927_mixed_house_random_trusses/index.html`.

## 거터 양측 줄기 혼합 배치

- `preview_neighbor_plants.py --paired-gutters --alternate-model` 추가. 거터 양쪽에서 줄기가 서로 반대 방향(0°/180°)으로 휘어 올라가도록 배치.
- seed27, 간격25cm, 주변68개: 기존/v9 각34개 랜덤 선택. 주변 식물은 시각 전용.
- 결과 `/root/docker_share/mujoko_debugging_data/20260927_paired_mixed_stem_house/index.html`. side 화면만 외벽/다른 거터 식물을 숨겨 배치 확인.

## 2026-09-27 새 줄기 leaves_v9 외형 추가

- `tomato_stem_leaves_v9.glb`를 `--stem-glb`로 랜덤 장면 생성/직접 각도 테스트에서 선택 가능. 기본 줄기 유지.
- 기존 16분절 주줄기 골격과 일치 확인. 신규 하부 잎을 포함한 모든 primitive 보존. 새 잎은 시각 전용이며 충돌/질량/개별 탄성 추가 없음.
- 비교: `/root/docker_share/mujoko_debugging_data/20260927_stem_leaves_v9/index.html`. 기존/신규 240스텝 qpos 동일.
- 상세: [STEM_GLB.md](mujoco-benchmark/STEM_GLB.md).

## 2026-09-27 충돌 없는 하우스 표시

- 기존 GUI/75도 상승 변경 커밋 `62e66be`. 물리 기본 Hz 변경 없음.
- `greenhouse_visual.py`, 생성기/직접예측 `--house`: 기존 Isaac 하우스1,078메시를 visual-only로 추가. 기존 로봇/식물 충돌 유지.
- 동일25초/240Hz 명령에서 배경 전후 qpos 완전 일치. 상세: [GREENHOUSE_VISUAL.md](mujoco-benchmark/GREENHOUSE_VISUAL.md).
- 결과 `/root/docker_share/mujoko_debugging_data/20260927_greenhouse_visual/index.html`. 이 하우스 추가 작업의 커밋 여부는 git status로 확인.

## 최신 수정: 송이 50% 축소와 고리–주줄기 관통 판정

상세 변경·재현 명령·검증 한계: [HALF_SCALE_CONTACT_FIX.md](mujoco-benchmark/HALF_SCALE_CONTACT_FIX.md).

- 새 GLB 생성/수집 CLI는 `--truss-scale 0.5` 기본. 주줄기·잎·로봇은 그대로, 송이 전체 가지·꼭지·열매의 길이/반경 50%, 질량 1/8. 강성·감쇠·armature 유지로 동역학 동등성은 주장하지 않음. 원래 크기는 `--truss-scale 1`.
- 기존 크기 기록의 resume에는 `--truss-scale 1` 명시. 다른 크기를 같은 수집에 섞지 않도록 검사.
- `20260923_random_glb_full_smoke/scene_0001/targets/Tomato_05` 후보00005의 기존 성공은 잘못된 유효성 판정. 고리–기존 주줄기 관통3.860mm를 GLB 전용 검사에서 누락. 새 실행은 invalid_physics로 처리하고 학습에서 제외. 기하 중심 진입 조건은 변경하지 않음.
- 구버전 GLB 자료도 학습 로더에서 저장된 고리 관통 지표로 제외. 이전 학습 smoke 가중치/성능은 이 오류의 영향을 받으므로 유효 결과로 사용하지 말 것.
- 근거/독립 재실행: `/root/docker_share/mujoko_debugging_data/20260923_center_entry_audit/`. 기존 수치/상태는 보존하고 해당 기존 HTML에 정정 안내만 추가.
- 절반 크기3장면·기존 형상 비교·2후보 물리: `/root/docker_share/mujoko_debugging_data/20260923_half_scale_glb/comparison.html`. 정지3장면 통과, 로봇2후보 miss. 수확 성공 검증이 아님.

## 최신: 랜덤 GLB 장면 → 전체 열매 10후보·9 RGB-D → 장면 분할 학습 → 새 장면 물리 검증

2026-09-23, 브랜치 `mjlab-performance`. 기존 장면 생성기·후보 수집기·D435 촬영·camera action v2·학습기를 연결했다. **이번 작업의 커밋 여부는 `git status`로 확인한다.** 랜덤화는 GLB 송이 종류, 원래 주줄기·잎에서의 송이 부착 위치, GLB Y각도이며 GLB 안 개별 열매의 배열은 바꾸지 않는다. 9-view 출력은 원본 RGB·Depth를 저장하고 별도 열매 mask 및 Depth 유효성 PNG는 저장하지 않는다.

작은 전체 연결 시험: seed5의 3장면×송이1개×열매10개×후보10개 = 300회 시도. 계획 실패110, 물리 미진입160, 중심 진입18, 물리 오류12. RGB·Depth 각270장. 장면0/1/2를 train/validation/test로 분리했고 3모델×3seed 학습에서 검증 BCE로 ResNet18 RGB-D seed2를 선택했다. 테스트 장면은 발견된 진입이0개이며 계획 가능한 후보가 없는 열매3개를 포함한다. 실패 열매가 실제로 불가능할 수 있지만, 10후보 미발견을 그 증명으로 해석하지 않는다.

별도 seed24 새 장면에서 열매10개의 후보를 결과 없이 계획해 모델 추천을 확정하고, 추천10개를 새 MuJoCo 물리로 독립 실행했다. 중심 진입1(Tomato_10), 미진입9, 물리 오류0. 저장 상태10개 재생 해시 확인. 꼭지 걸림 수확 및 RGB-D 우위·일반화는 미입증. 수집 `/root/docker_share/mujoko_debugging_data/20260923_random_glb_full_smoke/index.html`, 학습 `/root/docker_share/mujoko_debugging_data/20260923_random_glb_training_smoke/index.html`, 새 장면 물리 `/root/docker_share/mujoko_debugging_data/20260923_random_glb_fresh_physics_smoke/index.html`. 명령·한계: [RANDOM_GLB_SCENES.md](mujoco-benchmark/RANDOM_GLB_SCENES.md), 보존 수치: `mujoco-benchmark/validation/random_glb_collection_pipeline.json`.

## 최신: 주줄기 송이 위치·Y각도 랜덤 물리 장면 생성

`mujoco-benchmark/scripts/generate_random_glb_scenes.py`는 seed로 GLB 종류·주줄기 segment 안 위치·Y축 각도를 재현 가능하게 뽑고, 기존 주줄기·잎을 유지한 여러 송이 물리 장면을 생성한다. 각 장면 초기 2초 겹침·정지 이동 검사를 저장하며 실패 장면도 표시한다. 2장면×2송이 샘플: `/root/docker_share/mujoko_debugging_data/20260923_random_glb_physics_final/index.html`. 실행 명령과 범위: [RANDOM_GLB_SCENES.md](mujoco-benchmark/RANDOM_GLB_SCENES.md).

2장면 모두 초기 검사 통과. 두 번째 송이를 대상으로 로봇 1경로 실제 실행: 최대 겹침0.347mm, 중심 미진입, 열매 이동52.51mm. 다중 송이의 이름 공간도 `RobotEngine`의 접촉 검사에 포함하도록 수정했다. 한 경로의 정상 실행을 전체 랜덤 장면의 수집 승인으로 해석하지 않는다.

## 최신: 전체 GLB 로봇 수집 소량 검사 완료

2026-09-23, 최신5종/각1개 전체 로봇 경로, Tomato_05, Y90°/segment11, 240Hz. white는2번 위치·강성 수정 모델, 첫seed0 준비자세IK실패 후seed1실행. 총 계획6회/실제물리5회. 결과: green/red/white miss(물리 기준통과), rotated90 partial_center_entry(물리 기준통과), cyan invalid_physics(최대겹침0.514mm로0.5mm기준초과, 학습제외). 경고/수치불안정0.

최대목표밀림 cyan219.00/green156.96/red81.02/rotated90 83.86/white90.58mm. 현재center_entry_only_v2는밀림을실패로분류하지않으므로 유효4개 또는 rotated90부분진입을 안전수확 성공으로 해석하지 않는다. 각1경로라 전체각도/부착위치 성공률이나대량수집승인아님. cyan 접촉중겹침은미해결로보존.

5개 초기RGB-D/목표mask640×480 저장, 각가상카메라로action_camera재변환 및복원검사통과. 후보states/trace해시·수치유한성·관측모델해시일치검사통과, 회귀12테스트통과. 관측의실제관절도달성은미검증.

결과 `/root/docker_share/mujoko_debugging_data/20260923_024914_all_glb_robot_smoke/index.html`. 보존요약 `mujoco-benchmark/validation/all_glb_robot_smoke.json`.


## 최신: white 2번 간격 및 탄성 안정성 수정

2번을 중심가지에서 멀어지는 방향으로3mm 이동(원본GLB 좌표 offset은 build.json), 꽃받침/말단 꼭지를 함께 이동하고 proximal 시작점은 유지해 변형했다. 원본GLB 보존, 나머지 body 위치와 모든 질량 유지 검사 통과. 6번 제거 및 연결부 필터 유지. 초기0.5mm 초과 겹침0개.

간격 수정만으로 힘 시험 미복원이 해결되지 않았다. 고정 중력 preload를 쓰는 초기 평형의 (중력 토크 미분+스프링) 대칭 행렬 최소 고유값이 -0.409로 국소 불안정 방향이 확인됐다. white 전용 `--rachis-stiffness-scale 2` 적용: Rachis k2→4, d0.15→0.2121. 최소 고유값+0.203. 물성 조정이며 실측 보정/기존과 동역학 동등성 아님. 다른GLB/기본값은1 유지.

240Hz 0.2N(2.5~3초) 이후12초까지 검사: 5번 X/Y/Z 최대51.89/19.43/26.00mm→잔류2.47/0.287/0.198mm. 2번 X 최대29.89mm→잔류1.47mm. 네 시험 모두 접촉 관통0, 경고0; 별도 구형 물체 접촉 반응26.24mm. 6초 시점 X잔류18.65mm로 빠른복원을 주장하지 않는다. 초기 정지2초 이동 수치오차 수준. 회귀12개 통과. 로봇 수집은 이 수정 모델로 미실행, 모든각도/부착위치 안정성은 미검증.

최신 white: `/root/docker_share/mujoko_debugging_data/20260923_white_fruit02_stable/index.html`. 이전모음의 white 실패기록은 보존한다. 요약 `mujoco-benchmark/validation/white_fruit02_fix.json`.


## 최신: 전체5 GLB 6번 제거 완료

cyan/green/red/white/rotated90 모두 `--remove-fruit 6` 적용, 열매10개 유지. 기존 주줄기·잎 및 Y90° 배치/segment11 유지. 각 모델 240Hz 정지·0.2N 힘·접촉·복원·reset 검사 완료, 회귀12테스트 통과. 초기0.5mm 초과 겹침은 cyan/green/red/rotated90에서0개. white는 Fruit_02와 Rachis_04 충돌체 약1.001mm 겹침이 별도로 남아 사용 보류. 또한 힘 시험 무부하 대비 최대/종료 변위가 모두626.09mm로 복원 검증 실패했으며, 이 큰 이동의 원인을 해당 초기 겹침으로 단정하지 않는다. 다른 알맹이를 임의 삭제하거나 해당 충돌을 숨기지 않았다. 초기2초 최대 바디 이동 cyan2.20/green2.30/red2.35/white3.18/rotated90약0mm. 로봇 후보 수집은 제거 모델들로 재실행하지 않아 전체 수집 물리 유효성은 미검증.

모음 `/root/docker_share/mujoko_debugging_data/20260923_all_glb_without_fruit06/index.html`. 요약 `mujoco-benchmark/validation/all_glb_without_fruit06.json`. 개별 하위폴더의 model.mjb/reference.json 사용. 원본GLB 보존.


## 최신: cyan 6번 알맹이 제거

`build_glb_physics.py --remove-fruit 6`로 Fruit_06 body/충돌/질량(60.45g)/꽃받침 시각/타깃을 제거한다. 꼭지 가지는 남기고 원본 GLB는 보존한다. 나머지 열매 번호 유지, 총10개. 기존9·10번 연결부 필터 적용. cyan Y90°, segment11에서 초기0.5mm 초과 겹침0개, 정지2초 최대 이동2.20mm. 0.2N 힘 시험 최대25.19mm, 잔류4.85mm. 접촉/복원/reset 및4테스트 통과. 로봇 후보 수집은 이 제거 모델로 재실행하지 않았으므로 수집 성공률/전체 물리 유효성 미검증.

결과 `/root/docker_share/mujoko_debugging_data/20260923_glb_without_fruit06/index.html`.


## 최신: 9·10번 정상 연결부 충돌 수정

GLB 첫 꼭지 가지와 바로 옆 중심가지 분절의 접합부 충돌 두 쌍 수정. cyan 정지2초 최대 이동51.69→8.66mm, 힘·접촉·복원 및4테스트 통과. 6번 열매 겹침4.93mm는 남아 학습 사용 미승인. 상세 [GLB_PHYSICS.md](mujoco-benchmark/GLB_PHYSICS.md).

## 최신: GLB 탄성·충돌 이식 및 로봇 수집 시험

기존 주줄기·잎과 GLB Y축 배치를 유지하여 241자유도 물리 모델로 이식했다. 힘/접촉에 의한 휘어짐·복원 및 후보3개 실제 실행 완료. 초기 내부 겹침4.93mm로 모두 invalid_physics이며 학습 사용 금지. 기존과 동일한 동역학이나 수확 성공을 의미하지 않는다. [구현·검증·실행 명령](mujoco-benchmark/GLB_PHYSICS.md).

최종 갱신: 2026-09-23 KST (실험 폴더 이름은 기존 기록의 표기를 그대로 유지함).
이 파일은 대화 기록 없이 다른 계정/새 세션에서 작업을 이어가기 위한 시작점이다.
아래 상태는 작성 시점 기준이므로, 재개할 때 `git status`를 먼저 확인한다.

## 최신: 단일 GLB 관측 수집 파일럿 — 로봇 실행 전 제외

- `collect_glb_pilot.py`: cyan송이1개/Y90°/기존 주줄기/목표Fruit_05 장면 생성, RGB·metric depth·유효깊이 마스크·목표 마스크·카메라 행렬 저장 및 초기 convex 겹침 검사 수행.
- 최종 출력 `/root/docker_share/mujoko_debugging_data/20260923_020731_glb_single_collection_pilot/index.html`. RGB640×480, 목표1765픽셀, 유효깊이42121픽셀. 배열/마스크/카메라 직교성 및 라벨 제외 검사 통과.
- 수집 전 검사에서 새 송이 충돌 꺼짐/탄성 없음/로봇 없음/장면별 계획·목표 참조 미연결을 검출. 상태rejected_before_rollout, 로봇 실행0회/라벨0개/training_eligible=false. 가상 카메라 관측이며 기존 D435/action14 학습 데이터로 바로 사용할 수 없다.
- 실제 경로 수집을 하려면 GLB 물리 이식과 장면별 로봇·계획·평가 참조 연결이 선행되어야 한다. 기존 데이터셋은 수정하지 않았다.

## 최신: GLB 부착 커밋 및 초기 겹침 검사

- 사용자 확인한 GLB5종/Y축0~180°/무작위 위치 부착은 `67483c7`로 커밋했다. GLB 원본5개 포함. push는 하지 않았다.
- `audit_glb_collisions.py`: 접촉 마스크를 우회한 명시적 convex 형상 거리 검사. 다중송이 장면1072형상에서 AABB 선별 후4592쌍 검사, 약1.49초. 임계값0.5mm.
- 서로 다른 송이 겹침0쌍. 털 제외 시 송이–주줄기7쌍/최대3.042mm, 송이–잎·잘린가지1쌍/최대3.201mm. 내부1292쌍은 정상 접합/꽃받침/convex 과대근사 포함이므로 실패 수로 해석하지 않는다.
- 원본 메시의 정확한 삼각형 관통 또는 동적 물리 검증은 아니다. 털 포함/제외 수치를 분리하며 원본/접촉 마스크는 변경하지 않았다. 새 송이 탄성·충돌은 여전히 미적용.
- 결과 `/root/docker_share/mujoko_debugging_data/20260923_multi_glb_collision_audit_classified/index.html`, 보존 요약 `mujoco-benchmark/validation/glb_initial_overlap.json`. 비활성 접촉 마스크 상태의 겹침/접함/분리 거리3조건 검사 통과.

## 최신: GLB 5종을 동일 주줄기에 무작위 부착

- 사용자 확정 규칙: GLB Y축0~180°만 회전. Y-up→Z-up 변환 C=Rx90 후 C·Ry(theta), 추가 X/Z 회전 없음. 기존 주줄기·잎 보존, GLB 원본 크기1 유지.
- `assemble_glb_plant.py --output NEW_DIR --seed 23 --view`: 5파일을 각각1번 사용, 주줄기 호 길이 구간별 무작위 위치에 동시 부착. `--axes` 선택 가능. seed 재현 가능, 출처해시/각도/부착점은 scene.json 저장.
- 결과 `/root/docker_share/mujoko_debugging_data/20260923_multi_glb_y_random/index.html`. red20.5°, cyan153.6°, green39.2°, white84.7°, rotated90 62.8°. 컴파일 후5송이 변환/Y축 위쪽 및 기존 주줄기302형상 위치·몸체 질량/관성 보존 검사 통과.
- 정지 시각 미리보기다. 송이 간·잎 간 겹침 필터와 송이 충돌/탄성은 미적용이며 학습용 물리 장면이 아니다.

## 최신 수정: GLB Y-up → MuJoCo Z-up 좌표계 변환

- `attach_glb_preview.py`는 좌표계 변환 C=Rx(+90°) 뒤 원본 GLB Y회전 적용: R=C·Ry(theta). GLB +X→월드 +X, +Y→월드 +Z, +Z→월드 -Y(추가회전0도). 이 좌표계 변환은 송이의 추가 X회전 조작과 구분한다.
- 기존 축 변환 없는 부착 샘플은 과거 결과다. 새 cyan0도 부착: `/root/docker_share/mujoko_debugging_data/20260923_glb_y_up_attached/axes.png`. GUI 열림, 주줄기·잎 보존. -90/-30/0/30/90도 모두 GLB Y가 월드 위쪽인지 검사.
- 새 송이는 시각 전용이며 탄성/충돌 연결 미완료. `view_glb_truss.py`의 이전 단독 뷰어는 아직 숫자 좌표 그대로 표시하므로 위쪽 축 수정은 부착 뷰어에 적용된 상태다.

## 현재 사용자 확정: 새 GLB, Y축 회전만

- 원본 송이5개: `nvidia-sim/env_usd/tomato_rotate_glb/*.glb`. 사용자 조건은 **GLB의 Y축만 회전하고 X/Z축 회전을 추가하지 않는 것**. 아래 과거 방향 추정/반구 제한을 새 기준으로 적용하지 않는다.
- `mujoco-benchmark/scripts/view_glb_truss.py FILE.glb`: 원본 노드 변환·스케일·좌표/원점 유지한 송이 단독 미리보기. `[`, `]`로 Y축±5도, `0`으로 원본. 마우스는 카메라만 회전. 원점 기준 공통 GLB XYZ 표시.
- 5파일 각154메시 읽기 및 Y회전시 Y좌표/원점거리 보존 검사 통과. cyan0도 화면 실행 확인. 시각 확인 단계이며 새 GLB의 탄성/충돌/주줄기 연결은 아직 구현하지 않았다. 원본 GLB는 수정하지 않았다.

## 현재 기준: 사용자 지정 단계로 롤백

사용자 요청으로 `20260923_tipward_surface_attachment` 단계의 생성 동작으로 복원했다. 짧은 시작 가지 기준 회전과 당시 표면 부착 계산을 사용한다. 이후 캡슐 축 순서 수정, 엄격한 접합 거부/재샘플링, 긴 중심가지/열매 중심 각도 변경은 현재 코드에서 되돌렸다. 아래 후속 변경 기록은 과거 이력이며 현재 동작이 아니다. 기존 주줄기·잎과 모든 결과 폴더는 보존한다. 이 단계의 알려진 초기 겹침도 복원되므로 수집 검증 완료로 해석하지 않는다.

기준 결과: `/root/docker_share/mujoko_debugging_data/20260923_tipward_surface_attachment/index.html`.

## 최신 수정: 부착점→열매 평균 중심 기준(v5)

- 사용자 최종 정정: 주줄기 표면 부착점에서 각 열매 중심 좌표의 평균을 향하는 방향으로 각도를 계산한다. 국소 주줄기 위쪽0°, 0~90도 허용. 아래 v3/v4는 과거 기준이다.
- 새9개 샘플: `/root/docker_share/mujoko_debugging_data/20260923_fruit_center_angle_samples/index.html`. 컴파일된 열매 body 좌표에서 각도 독립 재계산 통과. 부착 불량2개 제외 기록 보존.
- `--fruit-center-angle` 사용. 기존 주줄기·잎 보존, 접합/지지 가지 비관통 검사 유지. 전체 장면 물리/학습 수집 준비 완료는 아니다.

## 최신 수정: 긴 중심가지 기준 샘플

- 각도 기준을 짧은 시작 가지에서 **송이 Rachis 중심선 시작→끝 방향**으로 변경(v4). 국소 주줄기 위쪽0°, 0~90도 허용. 기존32도 등 v3 수치와 다른 정의다.
- `assemble_plant_scene.py --rachis-angle 60`으로 명시 가능. 생략시15~85도 seed 샘플링. 기존 주줄기·잎/송이 내부 형상 유지, 표면 접합·주줄기 관통 검사는 별도 유지.
- 새9개 샘플: `/root/docker_share/mujoko_debugging_data/20260923_rachis_angle_samples_checked/index.html`. 전체 장면 물리/로봇 수집 검증은 미완료.

## 최신 수정: 송이 허용 방향과 표면 접합

- 사용자 확정: 주줄기 위쪽 끝 방향0°, ±90° 허용. 국소 주줄기 끝 방향과 시작 가지의 각도가90° 이내인지 검사하며 뿌리 방향 배치는 거부한다.
- 기존 주줄기·잎 보존. 원본 캡슐 fromto 순서로 축 방향을 복구하고, 송이 시작 캡슐 끝을 주줄기 표면에 접합한다. 접합 간격/지지 가지 전체의 주줄기 관통을 컴파일된 형상으로 검사하고 실패하면 생성 거부.
- 새 미리보기 `/root/docker_share/mujoko_debugging_data/20260923_tipward_attachment_final/index.html`. 전체 장면 충돌/로봇 수집 준비 완료를 의미하지 않는다.

## 최신 수정: 기존 주줄기와 잎 보존

- 조립기가 기존 `plant_mujoco_optimized.xml`의 주줄기 트리를 복사하고 기존 송이만 교체하도록 수정했다. 주줄기16몸체와 잎·잘린 가지 포함302형상을 유지하며 주줄기를 재생성하지 않는다.
- 기존 월드 배치·메시·색상·질량·관성·관절·충돌 정책을 보존한다. 새 송이의 위치·종류·회전만 바꾸며 새 하중에 따른 물리 응답까지 동일하다는 뜻은 아니다.
- 전체 식물/송이 확대 미리보기: `/root/docker_share/mujoko_debugging_data/20260923_preserved_stem_leaves/index.html`. 새 송이 물리와 수집기 연결은 여전히 미완료.

## 최신 추가: USD 송이 부착 장면 생성 미리보기

- `assemble_plant_scene.py` / `view_plant_scene.py`: 원본 송이 선택, 주줄기 호 길이 위치·국소 회전 변경, seed 재현, MuJoCo XML/JSON/미리보기 HTML 저장.
- 온실 USD 54식물/162부착 사례 확인. 162종 형상이라는 뜻은 아니다. L_00의3개 송이 배치로6장면 생성·렌더 완료.
- 2초 물리 발산은 없지만 초기 충돌 겹침1.03~2.82mm로6개 모두 겹침 기준 실패. **형태 검토용, 학습 수집 준비 미완료**. 기존 모델/데이터는 보존.
- 로봇·리프트·30후보 수집기 연결, 새 장면 분할 및 Depth+mask 학습은 아직 구현하지 않았다.
- 상세 범위/명령/검사: [PLANT_ASSEMBLY.md](mujoco-benchmark/PLANT_ASSEMBLY.md).
- 결과 `/root/docker_share/mujoko_debugging_data/20260923_attachment_preview_convex/index.html`.

## 최신 상태: RGB-D 진입각 학습·물리 검증 완료

- 브랜치 `mjlab-performance`. 수집/경로 변경 커밋 `3f636c2`, 다중 토마토 학습·검증 커밋 **`ed13ca9`**.
- 현재 방향은 고정 경로 후보를 RGB-D로 평가하는 지도학습이다. 아래 RL 기록과 구분한다.
- 데이터10대상×30경로/710관측, Tomato_05는 없음. 중심 정렬, 하부2mm, 방위각±90° Sobol, 전진45°상승→후퇴45°상승. 최종 상승32.5mm. 정상 물리에서 중심진입이면 열매 밀림과 무관하게 부분 성공.
- 8대상 학습(01,02,03,04,07,08,10,11),09검증,06테스트. DINOv2 RGB-D seed2 선택.06의64관측 모두 candidate_00001(−81.09°) 추천.
- 고유 추천1경로를 새 물리로1회 실행하여 중심진입 성공 확인, 열매 최대변위30.78mm. 64회 독립 물리 성공이나 꼭지 수확 성공이 아니다.
- 영상 없는 기준 모델도 top1성공이므로 RGB-D 우위 미입증. 확률 calibration도 불량(Brier약0.315); sigmoid1.0을 실제 성공확률100%로 해석하지 않는다.
- 결과: `/root/docker_share/mujoko_debugging_data/20260922_234403_multi_tomato_training/index.html`.
- 물리/재생: 같은 폴더 `physics/index.html`. 실행 명령·학습 분할·해석은 [learning/MULTI_TOMATO.md](mujoco-benchmark/learning/MULTI_TOMATO.md).
- 다음 판단: 재생으로 실제 진입 확인, 다른 토마토 holdout 비교, 영상 없는 모델 대비 이점 평가. 현재 모델은 저장된 관측 특징과 기존 후보를 사용하며 새 실물 이미지 입력/자유 경로 생성 기능은 아니다.

## 이전 단계: Tomato_06 RL (당시 상태 기록)


- 브랜치 `mjlab-performance`, 구현 커밋 `a085f47`. 최신 작업 요약은 [RL 작업 인수인계](mujoco-benchmark/RL_HANDOFF.md), 세부 설계는 [TOMATO06_RL.md](mujoco-benchmark/TOMATO06_RL.md).
- 팔6관절+리프트를 자유 제어하는 mjlab/MJWarp + RSL-RL PPO 구현. 고정 장면 GT 사용, 카메라 정책/RL 수확 성공은 아직 아니다.
- 256환경 10분 추가학습 완료: 170업데이트/139만 전이. 평가 최근접 거리56.16→43.17mm, 성공은 전후0. 더 가까이 접근한 뒤 머무르는 상태.
- 근접 보상이 정지에도 지급되는 문제가 유력한 원인. **보상 progress_v2 수정 완료, 새 학습은 미실행**이며 물리적 진입 가능성도 추가 확인 필요.
- 1,000회 장기 실행은 중단했고, 10분 실행은 완료했다. 아래 과거 기록의 실행 중/PID 표기를 현재 상태로 해석하지 말 것.
- 다음 작업: 저장 평가 재생 확인 → 보상과 탐색 개선 → 동일 기준의 10분 비교. 무조건 장시간 학습부터 늘리지 않는다.

## 이전 카메라 데이터 수집 단계 기록 (이후 완료, 아래 내용은 당시 상태)

1. Tomato_05의 1,000개 경로를 카메라 좌표 action v2로 저장하고 재실험 완료: 510.83초. 기존 결과와 일치.
2. 71개 RGB-D 관측과 71,000개 관측–후보 쌍의 좌표 변환 검증 완료. 목표는 부분 중심 진입과 최대 이동량이며, 꼭지 걸림 성공 판정은 아직 없다.
3. ResNet18 / DINOv2의 고정 영상 특징 + 작은 평가기, 영상 없는 기준 모델을 각각 3회 학습 완료. 검증 손실 기준 선택은 영상 없는 모델이다. AP는 실제 성공 확률이 아니다.
4. 모델 추천 21회(서로 다른 경로 3개)를 새 물리 실행으로 확인하고 2배속 영상 저장. 영상 모델은 각각 7/7 중심 진입 및 최대 이동 20mm 이하. 동일 장면 반복이며 실물 일반화 검증은 아니다.
5. 05를 제외한 10개 토마토의 수집 실행기 준비 완료. 각 대상 1후보 + 1시점 촬영 소량 검증 통과. 이후 본 수집을 진행하다 사용자 중단: 01~04 완료, 06은 정상 저장931개/남음69개. 아래 최신 재개 기능으로 이어갈 수 있다.

중단된 수집을 이어가는 명령:

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/collect_tomatoes.py \
  --resume --output /root/docker_share/mujoko_debugging_data/20260922_134834_multi_tomato_collection \
  --candidates 1000 --workers 48 --planning-workers 16 --postprocess-workers 8
```

- 상세 수집 명령과 카메라 의미: [MULTI_TOMATO_COLLECTION.md](mujoco-benchmark/MULTI_TOMATO_COLLECTION.md)
- 학습 비교와 재실행 방법: [learning/README.md](mujoco-benchmark/learning/README.md)
- 센서 저장 형식: [D435_RGBD.md](mujoco-benchmark/D435_RGBD.md), [OBSERVATION_DATASET.md](mujoco-benchmark/OBSERVATION_DATASET.md)
- 결과 데이터·모델 가중치·영상은 `/root/docker_share/mujoko_debugging_data/`에 있다. Git에는 코드와 문서를 저장한다. 계정/머신 이동 시 이 디렉터리와 기존 MuJoCo 모델·계획 자산·Python 환경도 별도 보존해야 한다.
- 카메라는 대상별 가상 시점이며 실제 로봇 관측 자세의 도달성을 검증하지 않았다. 다음 학습에서는 토마토 ID 단위로도 학습/평가를 분리해야 한다.

## 이전 측정: Isaac 로봇 팔 비용 비교

- `mujoco-benchmark/scripts/robot_cost_comparison.py` 추가. 후보49 첫10초, full/CPU240Hz/PGS64/4,
  파단 비활성. 실제 관절 모터 명령을 재생하고 측정한 고리 경로로 단독 조건과 비교했다.
- 실행 시간: 고리 headless9.87초 / 팔 headless10.78초 / 고리 GUI18.62초 /
  팔 물리+외형 숨김20.41초 / 팔 외형 표시21.24초. 준비/검사/저장 제외, 각1회 측정.
- 팔 물리/제어 약+0.91초, 팔 외형 약+0.83초. 이번 조건에서는 팔보다 전체 화면 갱신 부담이 큼.
- 팔 세 조건의 상태 완전 일치. 고리 경로 차이0.00015mm 미만. kinematic/dynamic 구동 차이로
  식물 반응 차이 최대1.22mm는 있음. 모두 접촉/관통·부착 검사 통과. 고리걸기 성공률 시험 아님.
- 결과 `mujoco-benchmark/outputs/20260922_044331_robot_cost/index.html`, CSV/JSON/스크린샷.
  요약 `mujoco-benchmark/validation/robot_cost.json`, 해석 `RESULTS.md`.
- benchmark_isaac.py에 --robot-trace/--robot-visuals/--camera-scale 추가. MuJoCo 로봇 이식 아님.
  기존 GUI는 --gui --trajectory fixture_entry_lift --hz240 사용; 조명은 stage open 전에 추가.
- timestamp 경로 입력 수정, 초기 고리 USD notice를 모든 모드에서 flush 후 reset하도록 수정.
  실패한 초기 실험은 최종 표에서 제외. 자동 테스트7개 통과. 기존 Isaac 원본 코드는 유지.

## 최신 추가: 원본 식물의 MuJoCo 비교 (2026-09-22)

- 별도 `mujoco-benchmark/`에 이식/benchmark/viewer/RGB-D/영상 생성기 추가. 기존 Isaac 코드와 USD 유지.
- 기준 `8e0d820`의 **full** 모델. 90강체/300 DOF/식물376+고리35 collision 유지.
  팔은 제외하고 world 고리 기록을 직접 재생. 양쪽 파단 비활성화한 Phase 1이다.
- CPU 원본120Hz 100개 순차 실행: Isaac 전체565.27초, MuJoCo539.83초. 병렬100환경이 아니다.
  부착 오차0.5mm 초과는 각각5/100,22/100. 엔진 변경만으로 큰 속도 이득 없음.
- MuJoCo 원본60Hz는6/6 폭주.240/480Hz는6개 fixture 검사 통과, 각각 RTF1.07/0.54.
- 별도 최적화는 잎/잘린 가지 충돌286개 비활성, 열매free/weld 병합(파단 불가).
  120Hz10개 RTF3.46, 검사 통과.60Hz6개 중2개 약1mm 관통으로 정상 채택 불가.
- GPU RGB-D10초5.63초(RTF1.78), 실제 DISPLAY=:0 viewer10초5.73초. scoped NVIDIA EGL 설정 사용.
- 결과 `mujoco-benchmark/outputs/20260922_002000_original_comparison/index.html`: 영상16개·RGB-D·CSV/JSON.
- 결론/사용법/구조 차이: `mujoco-benchmark/RESULTS.md`, `README.md`, `CONVERSION.md`.
  테스트6개, 입력 동일성/초기 상태/반복성 확인. raw 출력/대형 모델/venv는gitignore,
  작은 보존 요약은 `mujoco-benchmark/validation/engine_comparison.json`.
- 아직 전체 로봇/자동 detach/병렬 MuJoCo/RL/탐색 미구현. 원본120Hz 부착 오차를 먼저 해결해야 한다.
- 이 추가 작업의 커밋 여부는 `git status`로 확인한다.

## 최신 확인: 초경량 GPU 1·16·32·64환경 비교

- 같은 후보49 저장 경로를 모든 복제에 재생했다. 서로 다른 후보 탐색이나 계획 성능 시험은 아니다.
- 각 환경의 첫 10초 처리: 1개 22.26초 / 16개 62.96초 / 32개 106.88초 / 64개 200.27초.
  GPU 1환경 대비 합산 처리량 5.66 / 6.66 / 7.11배. 32→64의 추가 이득은 약 7%다.
- 전체 접촉 실행도 완료. 모든 복제는 GPU 1환경과 상태·접촉 결과가 일치하지만,
  동일한 0.521mm 관통으로 invalid_physics 처리됐다. 0.5mm 기준 유지. 정상 데이터로 사용 금지.
- reset/파단 API 통과, 환경 간 접촉 혼선 및 네이티브 오류 없음. 128환경은 미측정.
- 64개 전체 재생 376.16초 중 물리 56.87초, 쓰기·물리·읽기 외 비용 288.08초.
  CPU 약 110%, 전체 GPU 메모리 최대 3534MiB. 다음 병목은 환경별 판정·상태 처리다.
- 추가 도구 `ultralight_scale_benchmark.py`; 관련 자동 테스트 4개 통과.
  로그는 한글. 보고서 `runs/20260921_231100_ultralight_gpu_scale/benchmark.md` 및 CSV/JSON.
  보존 요약 `nvidia-sim/rl/validation/ultralight_gpu_scale.json`; 상세 `ULTRALIGHT_PLANT.md`.
- 모델/물리 설정은 이번 비교에서 변경하지 않았다. 사용자 세션은 유지했다.

## 이전 수정: 초경량 모델과 CPU 단일 환경 실시간 미리보기

- 이전 light 작업은 `a270554`로 커밋했다. 그 뒤 초경량 작업의 커밋 여부는 `git status`로 확인한다.
- `--plant-resolution ultralight` 추가. full/light와 원본 USD 유지. 기본값 full/GPU 유지.
- 식물 79강체/234자유도(full), 79/165(light) → **33/66(ultralight)**.
  줄기 캡슐 68개는 원래 위치·반경대로 유지하고 여러 조각을 한 강체에 묶는다.
  전체 활성 식물 충돌체 90개, 총질량 유지. 목표 꼭지 3분절, 열매 11개와 파단 유지.
  원거리 말단 힌지 병합으로 물리 응답과 충돌 제외 범위는 달라진다.
- `--physics-device cpu --num-envs 1` 추가. 실제 CPU PhysX/CCD, GPU 렌더링이다.
  120Hz/PGS64 유지, 매 스텝 0.5mm 관통 검사 유지. CPU/GPU 데이터·영상 비교를 분리한다.
- CPU 초경량 10초 처리 7.54초(계획·판정·저장 포함), full CPU 12.54초.
  화면을 켠 10초 고정 구간도 9.24초에 처리했다.
  초경량 화면 표시 후보49는 16.48초 분량을 13.71초에 실행. 초기 로딩/촬영 별도.
- CPU 정지·휘어짐/복원·파단·reset·4방향 선삽입 고리 유지 검사 모두 통과.
  실제 후보49는 excessive_displacement이며 수확 성공이 아니다. 관통 검사 통과.
- GPU 초경량 4환경×10초는 46.41초. 이전 light의 66.08초보다 개선됐지만 실시간은 아니다.
  GPU 60Hz/낮은 반복 수의 후보49는 관통 기준 초과. 그 설정을 기본값으로 채택하지 않았다.
- 상세 실행법/범위: `nvidia-sim/rl/ULTRALIGHT_PLANT.md`.
  실험: `nvidia-sim/rl/runs/20260921_ultralight/`, 요약: `validation/ultralight_plant.json`.

## 이전 수정: 경량 모델의 주줄기 잎·잘린 가지 충돌 제외

- `full`은 그대로 기본값이다. `light`는 이제 `--main-appendage-collisions ignore`가 기본이다.
  `light --main-appendage-collisions keep`으로 이전 경량 모델을 비교할 수 있다.
- 주줄기 부속 잎·잘린 가지 286개 convex 충돌체를 생성하지 않는다. 원본 시각 메시와
  주줄기 추종은 유지한다. 두꺼운 주줄기, TRUSS 송이 가지·꼭지, 열매 충돌은 유지한다.
- 송이는 이미 원거리 관절을 63 → 50개로 줄였다. 목표 주변의 충돌 형상은 유지한다.
- 정책을 config/backend/CSV/JSON/summary/대표 영상에 전달하고 정책이 다르면 resume 및
  영상 비교를 거부한다. 정책이 없는 이전 실행은 keep으로 해석한다.
- 실제 활성 충돌체 개수는 plant_model.json의 collision_inventory로 확인한다.
- 실제 집계 376 → 90개(줄기 68, 열매 11, 열매 꼭지 11). 4환경/10초 비교:
  full 초기화/실행 17.76/76.03초, light keep 17.67/68.55초, light ignore 9.85/66.08초.
  이번 잎 충돌 제외의 추가 실행 개선은 약 3.6%로, 주된 효과는 초기화와 충돌체 수 감소다.
- 관절·줄기 형상/질량·남은 충돌 필터는 light keep과 동일하고 초기 depth가 정확히 같다.
  관련 자동 테스트 39개 통과. 실행 폴더 `nvidia-sim/rl/runs/20260921_main_appendages_comparison`.
- ignore로 5·49 접촉 실행 완료: 과도 변위 2개, 관통 제외 0개(최대 0.222/0.317mm),
  잎/잘린 가지 접촉 이벤트 0개. 정지/휘어짐·복원/파손/리셋/네 방향 고리 fixture 유지 통과.
  검증 요약 `validation/main_appendage_light.json`. 수확 성공을 의미하지 않는다.
- 자세한 실행법/범위: [LIGHT_PLANT.md](nvidia-sim/rl/LIGHT_PLANT.md).

## 이전 수정: 원본을 유지하는 선택형 경량 식물 (잎 충돌 keep)

- 원본 USD와 기본 모델은 유지한다. GPU dataset의 `--plant-resolution full`이 기본값,
  `--plant-resolution light`로만 별도 경량 구조를 사용한다. 자동 출력 폴더는 `_light`로 구분.
- 충돌체·질량·열매 11개와 파손 연결은 유지. 목표 70mm 이내/목표 꼭지는 원래 관절 유지,
  먼 구간은 관절 4개당 약 1개만 휘게 하고 나머지는 부모에 고정 연결한다.
- Tomato_05 자유도 234 → 165. 주줄기의 탄성 연결 15 → 5개, 잎 충돌체 286개는 유지.
- 동일 4환경/10초 비교: 실행 75.43 → 66.86초(11.4% 단축), 물리 45.76 → 38.23초.
  64/256환경 처리량은 재검증하지 않았다. 초기화는 각각 18.06/17.77초.
- full의 4후보 처음 600스텝이 기존 원본 trace와 정확히 일치했다.
- light 0·1·5·49 완료: miss 1개/과도 변위 3개/관통 오류 0개. 5번은 원본의 부분 진입과
  달리 조기에 과도 변위로 중단되어 모델 간 판정 차이가 있다. 최종 후보는 원본 재검증 필요.
- 실제 모델·관절 목록을 plant_model.json에 저장하고 JSON/CSV/summary/영상에도 종류를 전달한다.
  서로 다른 모델로 resume/영상 비교를 통과시키지 않는다. 관련 자동 테스트 37개 통과.
- 경량 mechanics: 정지/휘어짐·복원/파손/리셋/고리 유지 통과. 0.2N 최대 변위 24.67mm,
  잔류 1.51mm, 연결부 최대 간격 0.022mm. 네 방향 고리 fixture 유지 통과.
  로봇의 진입 성공을 의미하지 않는다. 검증 요약 `validation/light_plant.json`.
- 자세한 범위/실행법: [LIGHT_PLANT.md](nvidia-sim/rl/LIGHT_PLANT.md).
  검증 실행: `nvidia-sim/rl/runs/20260921_plant_model_comparison`,
  경량 접촉 결과: `nvidia-sim/rl/runs/20260921_light_plant_4env`.

## 이전 수정: 단일 환경 GUI 속도 개선

- 동일 49번 GUI 실행 구간 150.34초 → 103.95초(약 31% 단축).
- 단일 환경은 원본 하우스를 표시하며, 다중 환경은 표시용 격자를 유지한다.
- 화면용 USD 동기화를 렌더 시점으로 제한하고 식물 skin 계산을 캐시/일괄 적용한다.
- 120Hz 물리/60Hz 제어/관통 검사 유지. 계획 명령, 제어 trace 995개,
  물리 접촉 trace 1,990개, 접촉 이벤트 306개 및 audit이 원본과 정확히 일치했다.
- 화면 시간 960Hz 고정값 수정. 화면 비용 기반 갱신율 조정 및 display_timings 추가.
- 실행법/검증 범위: [SINGLE_VIEW_PERFORMANCE.md](nvidia-sim/rl/SINGLE_VIEW_PERFORMANCE.md).
  대규모 환경 처리량 개선으로 일반화하지 않는다.

## 이전 수정: candidate_00049 접촉 관통 대응

- GPU dataset 기본값은 `contact120`: 120Hz 물리 / 60Hz 제어 / PGS 64/4 /
  armature 5e-4. 이전 practical60과 동일한 형상/물성/로봇 명령에서 timestep만 절반.
- 49번 동일 명령의 매 물리 스텝 겹침: 60Hz 최대 1.793mm → 120Hz 0.196mm.
  수정 후 줄기가 밀려 목표 변위 20mm 중단. 수확 성공으로 바꾼 것이 아니다.
- contact offset/CCD만 조정, 드라이브 강성 감소, velocity iteration 증가만으로는
  관통이 남았다. TGS 60Hz는 초기 파손. 120Hz 32/4도 0.5mm 선별 기준 미달.
- 새 dataset은 실제 USD/native pose의 고리 arc/rail 대 식물 capsule/sphere 겹침을
  매 스텝 검사한다. 0.5mm 초과면 `invalid_physics`로 중단/별도 저장/대표 영상 제외.
  mesh와 스텝 사이 연속 swept 검사까지 보장하는 것은 아니다.
- `physical_audit.json`, `tool_contact_trace.json`, CSV 물리 유효성 필드를 저장한다.
  오류를 숨기거나 고리/식물 pose를 인위적으로 보정하지 않는다.
- 문서: [CONTACT_PENETRATION_FIX.md](nvidia-sim/rl/CONTACT_PENETRATION_FIX.md).
  진단 원본: `nvidia-sim/rl/runs/20260921_contact_fix/`.
  기존 원본 데이터/영상은 변경하지 않고 새 결과와 구분한다. 실행 원본과 영상은
  git 제외 대상인 `runs/`에 로컬 보관하고 코드·문서·검증 요약을 커밋한다.
- 4환경에서 0·1·5·49 완료: 관통 선별 기준 초과 0개, 과도 변위 2개/miss 2개.
  5번 중심 진입 부분 성공 0.833초 유지, 완전 걸림 0. 모든 관절 명령은 원본과 동일.
  정지/탄성·복원/파손/리셋 및 미리 끼운 고리 네 방향 접촉 유지 fixture 통과.
- 49번 단독 측정: 60Hz guard 45.17초 / sim16.183초,
  120Hz 84.65초 / sim16.583초. sim 시간당 비용 약 1.83배, 초기화/렌더 제외.
  전체 64환경 처리량은 아직 재측정하지 않았다.
- 수정 전후 2배속 영상:
  `nvidia-sim/rl/runs/20260921_200605_candidate49_contact_fix_2x/index.html`.
  검증 요약 `validation/contact_penetration_fix.json`, 관련 자동 테스트 37개 통과.

## 이전 진단: candidate_00049 물리 관통 확인

- 사용자 영상 지적으로 저장된 native pose/접촉을 재검사했다. 49번 고리 segment_22가
  TRUSS_Rachis_08을 삽입 중 16.6~16.9초에 관통했다. 최대 겹침 1.793mm.
- 재실행 비교 통과와 물리적 유효성은 다르다. 이전의 49번 "정상 비목표 접촉 사례"
  해석을 정정하고 영상 모음에서 물리 오류로 분리했다. 대표 2개/진단 2개로 변경.
- 원본 CSV/판정은 보존하고 candidate_00049/physical_audit.json을 추가했다.
  당시에는 자동 제외 필터가 없었다. 후속 수정으로 새 dataset에 제외 검사를 추가했다.
- 근거: `nvidia-sim/rl/runs/20260921_candidate49_collision_audit/README.md`,
  `validation/candidate49_penetration.json`. 이 변경의 영상 분류 테스트 6개 통과.
- 다른 영상의 rachis 겹침 선별: 05 최대 0.266mm, 01 없음, 00 최대 0.701mm.
  전체 64개/모든 충돌체 검증은 아니다. 이 최초 진단 때는 물리 설정을 변경하지 않았다.

## 최신 후속 작업: 64환경 실측과 대표 영상 자동화

- 64환경/64개 서로 다른 staged6d 후보, CPU 경로 16 workers, PhysX CPU threads 8.
  practical60 물리/로봇 속도/중단 기준 유지. 실제 전체 실행 **814.70초 (13분 35초)**.
- rollout 740.32초: physics 481.42초, 실행/판정 192.93초, 명령 쓰기 38.61초.
  CPU 경로 작업 전체 10.59초. 명령 FK를 미리 계산하고 동일 tick 조회를 재사용했다.
- 이전 4환경/8후보 대비 후보당 관측 처리량 2.64배. 풀 크기/조기 중단 비율이 달라
  순수 코드 최적화 효과로 해석하면 안 된다. 64/64는 마지막 빈 슬롯 손실이 크다
  (active env-step fraction 0.414). 대량 실행은 continuous 재충전을 사용한다.
- CPU 초기화 최대 2801%, 경로 계산 시 최대 약 1892%, rollout 중앙값 117%.
  GPU 메모리 최대 6191MiB, 샘플 GPU 사용률 중앙값 25%. CPU 직렬 판정과
  physics 동기화 병목이 남아 있다. 전체 자원을 최대 활용하는 최적 설정을 찾았다는 뜻은 아니다.
- 54개 과도 변위, 8개 miss, 2개 비목표 접촉, 완전 걸림 0.
  candidate_00005만 안전 기준 내 중심 진입 1.22초 달성, 최종 miss.
- 이전 8개와 새 관절 명령 파일은 전부 수치적으로 동일. 최종 범주/부분 진입 여부도 일치.
  GPU 환경 수 차이로 과실 최대 변위 수치는 최대 0.294mm 차이가 있었다.
- 원본: `nvidia-sim/rl/runs/20260921_175044_staged6d_64env_videos/`.
  `performance_report.json`, `resource_samples.json`, `dataset/summary.json` 참고.
- `--representative-videos`로 완료 후 대표 사례 선정→저장 명령 재실행→원본 trace 비교→
  두 외부 시점 2배속 MP4/HTML 생성. 부분 성공을 완전 걸림으로 바꾸지 않는다.
  자세한 설명: [DATASET_REPRESENTATIVE_VIDEOS.md](nvidia-sim/rl/DATASET_REPRESENTATIVE_VIDEOS.md).
- 관련 자동 테스트 35개 통과. `representative_videos_2x/index.html` 영상 생성 완료:
  00005 부분 진입, 00001 과도 변위, 00049 비목표 접촉은 원본 비교 통과.
  00000은 최종 miss는 같지만 중간 주줄기 변위가 최대 15.29mm 달라 별도 진단 영상으로 분리.
  두 시점 2배속, 1280x612/30fps, 네 MP4 전체 디코드와 프레임 수 검사 통과.
  영상 후처리 추가 시간은 843.39초(약 14분). `validation/trajectory_64env.json`에 검증 요약.
  재실행이 원본 병렬 실행과 항상 같지는 않으므로 모든 원본 영상이 필요하면 본 실행에서
  물리 pose를 저장하는 방식으로 확장해야 한다. 현재 코드는 차이를 숨기지 않고 표시한다.

## 이전 작업: 떨어진 위치부터 접근하는 6D 소규모 시험

- 기준 커밋 `973806b` 이후 작업. [TRAJECTORY_PILOT.md](nvidia-sim/rl/TRAJECTORY_PILOT.md) 참고.
- `--trajectory-mode staged6d` 선택 시 170mm 떨어진 준비 위치→진입→수평 삽입→상승→1초 유지.
  기존 legacy 기본 동작은 그대로다. 새 모드는 접촉만으로 상승을 조기 종료하지 않는다.
- 실제 로봇 4환경·8개 Sobol 후보: 한 번의 완료 실행 269.06초, rollout 234.25초.
  6개 과도 변위, 2개 끝까지 실행 후 miss. candidate_00005는 GT 중심 진입 1.87초를
  달성했지만 꼭지 걸림은 아님. 이 새 후보의 영상 검증은 아직 하지 않았다.
- 동일 4환경/실패 비율 단순 예상: 64개 약 32분, 128개 약 63분, 2,048개 약 16시간 40분.
  영상 제외, 8개 표본의 제한이 있다. 새 경로의 16/32/64환경 처리량은 미측정이다.
- 완료 원본은 `runs/20260921_170634_staged6d_timing_pilot/env4_n8_final/`.
  앞선 두 폴더는 대화 중단으로 종료된 불완전 실행이다. 마지막 그래프 변수명 오류는
  물리 8개가 모두 저장된 뒤 발생했고 오프라인 후처리로 복구했다. 원래 오류 기록 보존.
- 관련 테스트 30개 통과. 새 source fingerprint 및 두 parameter schema의 plot 지원 추가.

## 이전 작업: 실용 60Hz

사용자가 960Hz와 완전히 같은 결과보다 밀림·걸림·휘어짐·복원을 유지하는 실용적
처리량을 우선하도록 기준을 변경했다. 따라서 아래의 이전 "960Hz 유지" 작업 지침은
후속 작업에서 대체됐다. 원래 성능/물리 이력은 삭제하지 않고 보존한다.

- 기준 커밋은 `bc84d41`; 실용 preset/검사/문서와 관찰 영상 도구는 후속 커밋에 포함했다.
  한글 커밋 제목은 `시뮬레이션: 실용 60Hz GPU 물리 설정과 접촉 관찰 영상 도구 추가`다.
- GPU dataset 기본은 `--physics-preset practical60`: 60Hz, armature 5e-4, PGS 64/4 iterations.
- `reference960`으로 원래 960Hz/1e-5 모델 선택 가능. `practical120`은 120Hz/3e-4 중간 비교용.
- 60Hz/5e-4에서 정지·실제 과실 하중/복원·원래 임계값 파손·리셋·네 방향 CAD 고리
  접촉 유지 검사가 통과했다. 고리 fixture는 미리 삽입된 별도 kinematic 도구이며 로봇 진입 성공은 아니다.
- 저장 로봇 경로 4환경 재생: 17.50초에 sim 4.4초 실행. 과거 960Hz는 168.69초/sim 4.333초였다.
  물리 모델이 다르므로 정밀 동등성 speedup이라고 부르지 않는다.
- 16환경 같은 경로 재생도 27.71초/sim 4.4초로 통과했고 native 접촉 라우팅/reset 오류가 없었다.
- 실제 dataset 4환경/서로 다른 4후보/8초 제한: RGB-D·계획·reset 통과. 결과 1개 과도 변위,
  3개 incomplete로 전체 수확 성공 검사는 아니다. 관련 자동 테스트 60개 통과.
- 30Hz/1e-3 잔류 변위가 커 제외. 60Hz/1e-3/32 iterations는 연결 간격 0.555mm로 0.5mm 기준 초과해 제외.
- 60Hz/3e-4 초기 검사는 idle을 과실만 검사했던 문제가 있었다. 주줄기도 포함하도록 수정했고
  최종 채택 모델은 5e-4다. 초기 `behavior60`의 passed 값을 그대로 최종 판정에 사용하지 않는다.
- 코드: `gpu_physics_presets.py`, `gpu_behavior_probe.py`, `gpu_low_hz_benchmark.py`, GPU runner/sim 변경.
- 새 검증 원본: `nvidia-sim/rl/runs/20260921_low_hz_practical/` (ignored).
- 최신 문서/실행법/레퍼런스: [GPU_PRACTICAL_PHYSICS.md](nvidia-sim/rl/GPU_PRACTICAL_PHYSICS.md).
- 60Hz 동작 관찰 영상: [GPU_PHYSICS_VIDEOS.md](nvidia-sim/rl/GPU_PHYSICS_VIDEOS.md).
  `runs/20260921_162920_practical60_contact_videos_2x/`에 실제 로봇 밀림, 하중/복원,
  미리 끼운 고리 유지의 두 시점 영상과 원본 pose/contact를 저장했다. 2배속 기본,
  고리 1배속 추가. 촬영 코드는 후속 커밋에 포함했고, 영상은 ignored runs에만 있다.
- 요약: [gpu_practical_physics.json](nvidia-sim/rl/validation/gpu_practical_physics.json).
- 다음에는 실제 로봇의 성공 진입/상승 경로, 타겟 1~11, 빠른 접촉과 더 많은 환경 수를 검증한다.
  아직 실측 보정이나 모든 상황의 비관통 보증은 없다. RL/DR을 추가하지 않았다.

## 1. 현재 결론과 바로 이어갈 일

- 기존 작업은 **`1305651`**까지 커밋했다. 브랜치는 **`nvidia-sim`**이다.
- 그 이후 **저주파 탄성 안정성 진단·설정 옵션·검증 자료를 이 문서와 함께 후속 커밋에 포함**했다.
  해당 커밋 제목은 `feat(sim): add elastic stability diagnostics and handoff`이며 해시는 `git log`로 확인한다.
- CPU 경로 계산 병렬화는 검증했고 기본 최대 8개 worker로 동작한다.
- GPU 물리 및 여러 환경의 동시 실행은 구현되어 있다. CPU 물리 worker pool과 혼동하지 않는다.
- 현재 핵심 미해결 문제는 **물리 결과를 유지하면서 960Hz를 낮추는 것**이다.
- 보조 관성(armature)을 키워 저주파의 정지 흔들림은 줄였지만, 접촉 변형과 파손 응답이 달라졌다.
  **60/240/480Hz를 기존 물리의 동등한 고속 대체 설정으로 채택하지 않았다.**
- 기본값은 **960Hz, armature 1e-5 kg·m², 로봇 제어 60Hz**를 유지했다.
- 이 파일을 작성할 때 관련 dataset/probe/planning worker는 실행 중이지 않았다.

다음 담당자는 [저주파 실험 상세](nvidia-sim/rl/GPU_ELASTIC_STABILITY.md)와
[수치 기록](nvidia-sim/rl/validation/gpu_elastic_stability.json)을 먼저 읽고,
후속 커밋의 코드를 검토한 뒤 탄성 관절·초기 하중·파손 연결부의 시간 간격 민감도를 조사하면 된다.
이미 실패한 설정을 다시 기본값으로 채택하거나, 같은 실패 라벨만 보고 물리 동등성을 선언하지 않는다.

## 2. 사용자 목표와 제약

사용자는 고리형 그리퍼로 방울토마토 아래에서 접근하고, 안으로 진입한 뒤 상승하여
꼭지 윗부분을 걸고 필요하면 당기는 동작을 연구 중이다. 주줄기를 밀면 송이와 과실도
움직여 고리걸기가 실패하는 상황까지 시뮬레이션되어야 한다.

현재 단계는 **고정 장면의 접근 pose 후보 → 실제 physics 실행 → 결과 데이터셋 생성**이다.
현재 성능/물리 진단 타겟은 `Tomato_05`이며, 장기적으로 Tomato_01~11 및 다양한 실험을 다룬다.

- 이번 데이터 생성 단계에서는 RL/IL, pose inference, segmentation 학습, domain randomization을 하지 않는다.
- 후보 생성과 사후 판정에는 simulator GT를 쓸 수 있다. 실제 perception이 구현되었다고 설명하지 않는다.
- RGB-D는 **진입 전** 촬영한다. 후보 실행 뒤 성공/실패 라벨을 붙인다.
  성공한 순간의 사진만 저장하는 방식이 아니다.
- 목표를 바라보는 관찰 자세, 원본 영상과 타겟 주변 문맥을 포함하는 crop을 사용한다.
  꼭지나 고리가 항상 보인다고 가정하지 않는다. 가림 상태도 실제로 존재한다.
- 장기 sim-to-real에서는 카메라와 joint 정보를 우선 고려하고, 힘 센서는 제외하고자 한다.
- 사용자가 제공한 house/식물/로봇 장면을 기반으로 한다. 무관한 단순 환경으로 바꾸지 않는다.
  멀티환경 표시/실행 최적화의 세부 범위는 기존 문서와 코드를 확인한다.
- 식물 위치 예: `--spawn-stem -0.75 0.55 0.32`.
  초기 자세는 ROS2 GUI의 left pick ready 계열, 리프트는 타겟 송이보다 0.4m 낮게 시작하는 요구가 있었다.
  실제 GPU runner의 cfg 설정을 기준으로 확인한다.
- 영상/실험 출력은 datetime으로 시작하고 간단한 설명이 붙은 폴더에 저장한다. 영상은 주로 2배속 요청이었다.
- 목표 처리량은 약 1,000개 후보/1시간이지만 **달성 검증은 안 됐다**.
- 최근 사용자는 큰 실험의 로딩을 피하려고 **1환경과 4환경으로 먼저 비교**하도록 요청했다.
  성능 검증 때 곧바로 256/1,000환경을 띄우지 않는다.

`--num-envs 32 --candidates 128`은 총 128개 pose를 최대 32개씩 실행한다.
32×128개의 후보를 생성하는 것이 아니다. `continuous`는 빈 슬롯에 다음 후보를 넣는 방식이다.

## 3. 실행 환경과 Git 상태

| 항목 | 값 |
|---|---|
| 저장소 | `/root/farmily_tomato` |
| 브랜치 | `nvidia-sim` |
| 인수인계 포함 후속 커밋 | `feat(sim): add elastic stability diagnostics and handoff` (해시는 `git log` 확인) |
| 성능 개선 기준 커밋 | `1305651 perf(sim): precompute candidate paths on CPU workers and benchmark timesteps` |
| 이전 관련 커밋 | `ec0c9db perf(sim): batch GPU commands and continuously schedule candidates` |
| 이전 관련 커밋 | `8995518 feat(sim): scale GPU candidate experiments and add live grid viewer` |
| 이전 관련 커밋 | `d14589d feat(sim): add parallel dataset workers, GPU probes and result dashboard` |
| 런타임 | Isaac Sim 5.1 + Isaac Lab |
| Python | `/root/isaaclab_env/bin/python` |
| GPU | RTX 3090 Ti, 약 24GB |
| CPU | Threadripper 3960X, 24코어/48스레드 |
| 물리 모니터 | `DISPLAY=:0` 사용 가능 |

`run_gpu_candidate_dataset.sh`는 Python 경로를 `FARMILY_ISAAC_PYTHON`으로 재지정할 수 있다.
기본은 위 Python이며 `PYTHONHOME`, `VIRTUAL_ENV`를 해제하고 `PYTHONPATH`를 설정한다.

후속 커밋에 포함한 파일(아래 상태 문자는 커밋 전 기록이며, 이 문서도 함께 포함):

```text
 M nvidia-sim/rl/GPU_TIMESTEP_AND_PLANNING.md
 M nvidia-sim/rl/elastic_plant.py
 M nvidia-sim/rl/gpu_dataset_runner.py
 M nvidia-sim/rl/gpu_dataset_sim.py
 M nvidia-sim/rl/gpu_probe.py
 M nvidia-sim/rl/gpu_probe_worker.py
 M nvidia-sim/rl/test_gpu_dataset_runner.py
 M nvidia-sim/rl/test_gpu_probe.py
?? nvidia-sim/rl/GPU_ELASTIC_STABILITY.md
?? nvidia-sim/rl/validation/gpu_elastic_stability.json
```

**계정 전환 시 보존:** 같은 워크스페이스를 사용하면 이 문서와 코드를 읽어 이어갈 수 있다.
코드와 요약 자료는 로컬 Git에 커밋했으며 push는 하지 않았다. 새 머신에서 원격 저장소만
clone하면 이 로컬 커밋이 포함되지 않을 수 있다. 또한 `nvidia-sim/rl/runs/`는 Git ignore
대상이므로, 다른 머신으로 옮길 경우 로컬 커밋과 필요한 실험 원본 폴더를 별도로 보존해야 한다.

## 4. 현재 실행 구조

| 파일 | 역할 |
|---|---|
| `nvidia-sim/run_gpu_candidate_dataset.sh` | 현재 GPU 데이터셋 실행 진입점 |
| `nvidia-sim/rl/gpu_dataset_runner.py` | CLI, config, resume 일치 검사, subprocess 관리 |
| `nvidia-sim/rl/gpu_dataset_sim.py` | 실제 장면, GPU 설정, 관측/검증, 후보 실행 스케줄 |
| `nvidia-sim/rl/dataset_scene.py` | 복제 환경/슬롯과 물리 실행 |
| `nvidia-sim/rl/dataset_motion.py` | 기존 후보 경로 계산 및 실행 |
| `nvidia-sim/rl/gpu_planning.py` | 실장면 모델 export 및 CPU 경로 사전 계산 서비스 |
| `nvidia-sim/rl/gpu_planning_worker.py` | Isaac을 띄우지 않는 CPU worker 진입점 |
| `nvidia-sim/rl/elastic_plant.py` | 탄성 식물 articulation, preload, 시각 모델 연결 |
| `nvidia-sim/rl/gpu_probe_worker.py` | 저장 명령을 재생하는 물리/정지/하중 분리 진단 |
| `nvidia-sim/rl/gpu_probe.py` | probe 실행/비교, 유효하지 않은 speedup 제외 |
| `nvidia-sim/rl/gpu_timestep_benchmark.py` | 1/4환경의 물리 Hz 비교 |
| `nvidia-sim/rl/gpu_planning_benchmark.py` | 동일 모델의 순차/병렬 CPU 경로 비교 |

현재 GPU dataset은 GPU dynamics + GPU broadphase + CPU tensor readback/native collider reports를 사용한다.
**완전히 CUDA tensor로만 처리하는 파이프라인이 아니다.** 접촉 object 식별을 위해 readback을 유지한다.
현재 GPU 설정은 PGS, position iteration 64, velocity iteration 4,
`gpu_max_num_partitions=1`, GPU CCD off다. 이전 CPU/TGS와 동일하다고 보증하지 않는다.
probe의 기본 solver/partition은 dataset과 다를 수 있으므로 재현 명령에서는 명시한다.

CPU planning은 실제 USD에서 추출한 FCL 도형 44개/자기충돌 검사 쌍 228개와 FK/IK 정보를 전달해
동일한 `dataset_motion.plan()`을 호출한다. IK·joint limit·자기충돌 검사를 생략하지 않는다.
최대 8개 worker로 다음 후보를 미리 준비하고 물리와 겹쳐 실행한다.
`--planning-workers 0`은 원래 순차 방식이다.

`DATASET READY`는 경로 계산 완료, `DATASET PHYSICS`는 물리 실행이다.
예전 `DATASET START`가 순차로 느리게 증가하던 큰 원인은 경로 계산이었다.
`planning_s`는 메인 프로세스의 대기 시간이며, worker 계산 시간의 합과 다르다.

타겟/성공 판정은 기존 object identity와 접촉 판정을 유지한다. probe의 `identity()`에서는
목표 distal `PedicelCollider` 및 proximal 마지막 collider, ring segment 08~23을 기록한다.
이 문서만 보고 성공 조건을 새로 추정하지 말고 `dataset_motion.py` 및 실제 target prim 정보를 확인한다.

## 5. 커밋된 성능 개선 결과

상세: [GPU_TIMESTEP_AND_PLANNING.md](nvidia-sim/rl/GPU_TIMESTEP_AND_PLANNING.md),
[검증 JSON](nvidia-sim/rl/validation/gpu_timestep_planning.json).

64개의 서로 다른 전체 후보 경로를 계산한 결과:

| 방식 | 경로 준비 시간 | 순차 대비 |
|---|---:|---:|
| 순차 | 86.14초 | 1.00배 |
| 4 workers | 23.86초 | 3.61배 |
| 8 workers | 13.57초 | 6.35배 |

명령 배열, waypoint, pose와 preflight 결과가 정확히 같았다. **전체 물리 실험이 6.35배 빨라진다는 뜻은 아니다.**
4환경/8후보/8제어 스텝의 통합 비교도 명령·trace·reset이 같았으며 경로 대기는 12.61→3.24초였다.
이 짧은 검사에는 접촉이 없었고 결과는 모두 `incomplete`다.

원래 armature 1e-5로 Hz만 낮추면:

- 60/120/240Hz는 접촉 전에도 식물이 밀려 조기 종료했다.
- 480/720Hz는 같은 접촉 구간을 진행했지만 960Hz 대비 1mm 회귀 기준을 초과했다.
- 높은 물리 스텝 수의 비용은 여전히 큰 병목이다. CPU 사용률만으로 GPU 물리 비용을 판단하지 않는다.

## 6. 후속 저주파 안정화 작업과 결과

추가한 코드:

- dataset CLI `--elastic-joint-armature` (기존값 1e-5, 유효 범위 0~0.01, 유한값 검사).
- 실제 PhysX DOF armature 적용 확인, config/backend/후보 metadata 저장, 설정이 다른 resume 거부.
- probe의 `--no-gravity`, `--no-preload`, `--preload-mode`, `--joint-armature`, `--pulse-force`.
- 정지 중 식물/과실 이동 및 joint 속도 기록. probe 예외 시 비정상 종료 코드 반환.
- 비교 시 다른 armature/preload 모델 또는 실패한 native API 검사는 유효한 speedup으로 인정하지 않음.
- production preload/reset은 원래 방식 그대로다. 실패한 drive/velocity 방식은 probe 안의 진단용 monkeypatch만 남겼다.

기존 모델은 articulation에 가는 줄기와 과실 연결 anchor를 구성하고,
초기 Jacobian으로 중력 보상 preload를 계산하여 고정 effort로 넣는다.
이번 진단에서는 geometry, 분할 수, 질량, 강성/감쇠, 마찰, 파손 임계값을 바꾸지 않았다.
단, **추가 armature 자체는 동역학을 바꾸는 추가 관성**이다.

240Hz에서 로봇 정지 2초 시험:

- 원래 모델의 최대 식물 바디 이동 약 58.5mm.
- 중력+preload 제거 시 약 0.072mm. 충돌만 제거 시 약 32.4mm.
- preload를 position/velocity drive로 옮기면 불안정/파손. 채택하지 않았다.
- armature 1e-4: 약 0.43mm. armature 1e-3: 약 0.10mm.
- 60Hz / armature 1e-3: 약 0.50mm. 정지 결과만으로 접촉 정확도를 보증하지 않는다.

저장된 `candidate_00003` 명령의 접촉 재생 시간(초기화 제외):

| Hz / armature | 1환경 wall | 4환경 wall | 실행 sim 시간 |
|---|---:|---:|---:|
| 960 / 1e-5 기존 | 156.25초 | 168.69초 | 4.333초 |
| 480 / 1e-4 | 79.54초 | 88.31초 | 4.350초 |
| 240 / 1e-4 | 40.83초 | 47.90초 | 4.383초 |
| 60 / 1e-3 | 11.74초 | 17.32초 | 4.350초 |

모두 비목표 잎을 접촉한 뒤 `excessive_displacement`로 종료했다.
성공 고리걸기/전체 60초 모션 처리량 검증이 아니다. 4환경은 같은 명령의 복제다.
물리 모델과 종료 시점이 다르므로 위 비율을 동등한 물리의 가속률로 발표하지 않는다.

0.2N에 해당하는 초기 Jacobian 고정 토크를 0.5~1.5초 가한 뒤 제거한 시험:
최대 목표 변위는 기존 약 37.64mm, 240Hz/1e-4 약 21.39mm, 60Hz/1e-3 약 16.56mm였다.
복원은 일어나지만 원래 변형 응답과 다르다. 변형 중인 지점에 일정 world force를 주는 시험은 아니다.

같은 armature끼리 시간 간격을 비교해도:

- 1e-4, 960 vs 240Hz: 고리 간격 지표 최대 차이 3.83mm, 종료 3제어 스텝 차이.
- 1e-4, 960 vs 480Hz: 고리 간격 지표 최대 차이 1.96mm, 종료 1제어 스텝 차이.
- 1e-3, 960Hz: **첫 제어 스텝 0.0167초에 로봇 접촉 없이 비목표 `HarvestJoint_05_4` 파손**.
  native API 종합 검사도 false. 이것은 안정한 비교 기준이 아니다.

회귀 허용치는 위치 지표 1mm / 로봇 joint 0.005rad / 종료 1제어 스텝으로 두었다.
이 기준은 실물 정확도나 tunneling 부재를 보증하는 값이 아니다.
960Hz도 실물 정답으로 검증된 모델은 아니다.

## 7. 실험 자료 위치와 검증 범위

모두 저장소 기준 `nvidia-sim/rl/` 아래다.

| 위치 | 내용 |
|---|---|
| `validation/gpu_timestep_planning.json` | 커밋된 CPU 병렬화/Hz 비교 요약 |
| `validation/gpu_elastic_stability.json` | 후속 안정성·접촉·복원·통합 시험 요약 |
| `runs/20260921_143244_gpu_timestep/` | 원래 모델 1/4환경 × 60~960Hz 시험 |
| `runs/20260921_144300_planning_validation/` | CPU 64경로 benchmark 및 순차/병렬 통합 비교 |
| `runs/20260921_150500_elastic_diagnosis/` | 중력/preload/contact/drive/armature 분리 진단 |
| `runs/20260921_151000_elastic_validation/` | 접촉 재생, pulse, 같은 관성의 Hz 비교, 데이터셋 통합 |
| `runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003/` | 저장된 재생 fixture |

fixture 명령 SHA256:
`cd300bcd280159339580dcba375ed759459f458eb8e84d03da730701aa5d2ad2`.

마지막 validation 폴더의 주요 하위 폴더:

- `motion_240_{1,4}`, `motion_60_{1,4}`, `motion_480_{1,4}`: 접촉 재생.
- `motion_240_stronger_{1,4}`: 240Hz / armature 1e-3 추가 진단.
- `reference_240_model_960`, `reference_60_model_960`: 각각 같은 armature의 960Hz 기준 시도.
- `pulse_960`, `pulse_240`, `pulse_60`, `pulse_240_stronger`: 하중/복원 시험.
- `*_harness_error`: 첫 pulse 구현 오류 때문에 실패한 자료. 유효 결과에서 제외했다.
  실제 joint effort 접근을 수정한 뒤 위 pulse 폴더로 재실행했다.
- `dataset_240`, `dataset_60`: 각 4환경/8개의 서로 다른 후보/16제어 스텝 통합 검사.
  초기 상태·독립 reset·camera·armature·planning 연동 통과. **각 8개 `incomplete`, dataset_complete=false**.
- `run_*.py`, `summarize.py`: 당시 실험 orchestration/요약 스크립트. `runs/` 내부라 Git 미추적이다.
  과거 폴더 완료 조건을 기다리는 스크립트도 있으므로 그대로 재실행하지 말고 명령을 읽어 새 폴더에 실행한다.

기록된 마지막 자동 검증은 관련 테스트 **55개 통과** 뒤 비교 도구에 API 실패 제외 테스트를
추가하고 해당 파일 **6개 통과**를 확인한 상태다. 새 테스트를 포함한 전체 묶음을 한 번에
다시 실행한 것은 아니다. `git diff --check`도 통과했다.

## 8. 재개 및 재현 명령

먼저 상태와 문서를 확인한다. 이 문서 작성 요청에서는 추가 시뮬레이션을 돌리지 않았다.

```bash
cd /root/farmily_tomato
git status --short
git log -4 --oneline
```

현재 기본 물리로 작은 dataset 실행(전체 후보 동작이므로 짧게 끝난다고 보장하지 않음):

```bash
./nvidia-sim/run_gpu_candidate_dataset.sh \
  --num-envs 4 --candidates 8 --schedule continuous \
  --physics-hz 960 --elastic-joint-armature 0.00001 --planning-workers 8
```

짧은 연결 검사만 하려면 `--max-control-steps 16`을 추가한다.
이때 중단된 후보는 `incomplete`이며 성공/실패 학습 데이터로 쓰지 않는다.
GUI 1환경 확인은 위 명령을 `--num-envs 1 --candidates 1 --gui`로 바꾸고 `DISPLAY=:0`을 사용한다.
새 계정에서 재현하기 전에 `--help`로 현재 옵션을 확인한다.

동일 fixture의 240Hz / armature 1e-4 진단 재현(**동등성 검증에 실패한 실험 설정**):

```bash
task_probe_dir="nvidia-sim/rl/runs/$(date +%Y%m%d_%H%M%S)_elastic_240_probe"
/root/isaaclab_env/bin/python nvidia-sim/rl/gpu_probe_worker.py \
  --headless --mode gpu --solver pgs --batched-io --native-replication \
  --gpu-partitions 1 --num-envs 1 --physics-hz 240 --joint-armature 0.0001 \
  --fixture nvidia-sim/rl/runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003 \
  --output "$task_probe_dir"
```

같은 설정으로 4환경을 검사하려면 `--num-envs 4`와 새로운 output 폴더를 사용한다.
동일 GPU에서 성능 시험을 동시에 실행하지 않는다. 초기화 시간과 motion wall time을 분리해 비교한다.

관련 테스트 재실행:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=nvidia-sim/rl \
/root/isaaclab_env/bin/python -m pytest -q \
  nvidia-sim/rl/test_gpu_planning.py \
  nvidia-sim/rl/test_gpu_step_profile.py \
  nvidia-sim/rl/test_gpu_throughput_benchmark.py \
  nvidia-sim/rl/test_gpu_replication.py \
  nvidia-sim/rl/test_gpu_scale_validation.py \
  nvidia-sim/rl/test_gpu_prim_lookup.py \
  nvidia-sim/rl/test_gpu_initialization.py \
  nvidia-sim/rl/test_gpu_batch_views.py \
  nvidia-sim/rl/test_gpu_dataset_runner.py \
  nvidia-sim/rl/test_gpu_validation.py \
  nvidia-sim/rl/test_gpu_probe.py \
  nvidia-sim/rl/test_dataset_pool.py
```

## 9. 다음에 검토할 작업 (미실시)

1. 후속 커밋 diff와 실험 기록을 검토하고 보존한다. 원래 preload/reset을 실험 실패 버전으로 되돌리지 않는다.
2. 보조 관성 1e-3/960Hz에서 접촉 전 파손되는 이유를 파손 연결부의 하중/초기화부터 분리 진단한다.
   원인이 엔진 버그라고 확정되지는 않았다.
3. 주줄기·꼭지·과실의 결합 및 하중 전달을 대상으로 작은 정지/토크/접촉 시험을 만든다.
   주파수를 바꿔도 변형·파손 응답이 수렴하는지 검사한다.
4. 실측 식물 변형 자료가 확보되면 모델의 응답을 보정한다. armature를 키우는 것만으로 sim-to-real을 보증하지 않는다.
5. 유효한 저주파 설정을 확보한 뒤 서로 다른 후보, 성공 trajectory, 타겟 1~11로 검증을 넓힌다.
6. 그 다음 대규모 환경 수에서 초기화/경로/physics/read-write/판정 시간을 다시 측정한다.

사용자가 요구한 충돌/탄성/파손 실패 사례를 없애거나 파손 임계값을 올려서 속도 시험을 통과시키지 않는다.
물리 정확도를 바꾸는 설정과 결과를 유지하는 코드 최적화는 기록에서 구분한다.

## 10. 이전 작업 문서 찾아보기

- [전체 시뮬레이터 README](nvidia-sim/README.md)
- [GPU 데이터셋](nvidia-sim/rl/GPU_CANDIDATE_DATASET.md)
- [GPU throughput](nvidia-sim/rl/GPU_THROUGHPUT.md)
- [GPU 병목 조사](nvidia-sim/rl/GPU_BOTTLENECK.md)
- [GPU 최적화](nvidia-sim/rl/GPU_OPTIMIZATION.md)
- [GPU probe](nvidia-sim/rl/GPU_PROBE.md)
- [후보 데이터셋/관측](nvidia-sim/rl/CANDIDATE_DATASET.md)
- [이전 CPU 다중 프로세스 pool](nvidia-sim/rl/CANDIDATE_DATASET_POOL.md)
- [탄성 식물](nvidia-sim/rl/ELASTIC.md)
- [고리걸기](nvidia-sim/rl/HOOKING.md)
- [pose 탐색](nvidia-sim/rl/POSE_SEARCH.md)
- [RGB 관찰/밀림 감지](nvidia-sim/rl/RGB_GUARD.md)

옛 문서의 CPU/TGS/worker pool 설정을 현재 GPU runner의 설정으로 착각하지 않는다.
충돌하는 수치는 최신 코드, 이번 검증 JSON, 마지막 두 성능/안정성 문서를 우선 확인한다.

## 새 대화에 붙일 문장

> `/root/farmily_tomato/HANDOFF.md`를 읽고 현재 Git diff와 연결된 최신 검증 자료를 확인한 뒤 이어서 작업해줘.
> 성능 개선 기준 커밋은 `1305651`이고, 저주파 안정화 진단과 이 문서는 후속 커밋에 포함했어.
> GPU 멀티환경과 CPU 경로 병렬화는 구현되어 있지만, 물리 결과를 유지하는 저주파 설정은 아직 검증하지 못했어.
> 그 이후 실용적인 물리 동작을 우선하는 기준으로 바뀌었고, 최신 기본값은 practical60이야.
> 이 문서 맨 위의 최신 후속 작업과 GPU_PRACTICAL_PHYSICS.md를 먼저 확인해줘.

## 2026-09-22 로봇 포함 MuJoCo CPU 기준선

- 현재 사용자 작업 브랜치 `mujoco-sim`에서 로봇 USD 관절/질량/관성/충돌 형상을 MuJoCo에 추가.
- `mujoco-benchmark/scripts/build_robot_model.py`, `robot_engine.py`, `robot_viewer.py`, `benchmark_robot_pool.py`.
- 기준 모델은 기존 최적화 식물, 로봇 7축, 전체 241 DOF. 고리는 동적 로봇 팔 끝에 고정. 하우스 없음.
- CPU 1/2/4/8/16/24/32/48/64 프로세스, 동일 49번 16.5초 기록을 각각 3회 실행.
- 48개 평균 6.443초/배치, 처리량 122.97 sim-s/s. 64개는 비슷한 처리량. 최대 검증64, 권장48.
- 모든 반복 발산0. 전체 영상/기하 검사 `outputs/20260922_robot_validation_final`, 순기구학 위치 오차 3.3e-8m, 9테스트 통과.
- 원본 시간/자원/코드 해시는 `outputs/20260922_robot_cpu_scaling{,64}`; 커밋용 요약은 `validation/robot_cpu_scaling.json`.
- 사용자는 CPU 기준선 커밋 후 새로운 브랜치에서 mjlab/MJWarp를 구축해 같은 조건의 GPU 성능 비교를 요청했다.

## mjlab GPU 비교 완료

- CPU 기준선 `38bac4d` 이후 `mjlab-performance` 브랜치 생성. 별도 `.mjlab-venv` 설치, 기존 `.venv` 보존.
- mjlab 1.6.0 / MuJoCo·MJWarp 3.11 / Warp 1.17 / Torch 2.7 cu128. `setup_mjlab.sh`, lock 파일 포함.
- 동일 모델 1/8/32/128/256/512/1024환경, 각 16.5초 × 3회. CUDA 제어·물리·평가 시간과 자원 시계열 저장.
- 가장 효율적인 GPU 설정 256: 평균 32.030초, 131.88 sim-s/s. 1024는 평균 179.960초, 93.89 sim-s/s.
- CPU 3.13/48프로세스 122.97 sim-s/s; 버전 맞춘 CPU3.11/24프로세스 75.62 sim-s/s.
- 모든 반복 비정상/overflow 0. GPU world0와 CPU 전체 스텝의 위치 오차는 validation/mjlab_robot_equivalence.json.
- 상세/한계/명령: mujoco-benchmark/MJLAB_PERFORMANCE.md. 비교 화면은 outputs/20260922_robot_backend_comparison/index.html.
- 파단은 비활성화, 동일49번 반복 성능 시험. 다양한 후보 탐색이나 RL 학습/수확 성공 평가기는 아님. mjlab Simulation 계층을 사용했으며 ManagerBasedRlEnv는 아직 추가하지 않음.

## 2026-09-22 CPU/GPU 운영 권장 정리

- GPU 구성 및 비교 구현 커밋: `115560b` (`mjlab-performance`). CPU 기준선: `38bac4d`.
- 대량 탐색 후속 구현 기본값은 GPU 256환경 권장. 기존 CPU 3.13/48 대비 처리량 약 7% 우세이며, 소수 경로 디버깅은 CPU 권장.
- 아직 동일 경로 재생 벤치마크이므로 다양한 후보 생성/저장까지 포함한 완료 시간은 미검증.
- autoreset 호환 변경은 별도 모델의 disable → enable 변경이며, MJWarp의 미지원 기능을 구현한 것이 아님. 원본과 오류 처리 의미가 같다고 설명하지 않는다.
- 상세 근거와 다음 단계: [MJLAB_PERFORMANCE.md](mujoco-benchmark/MJLAB_PERFORMANCE.md)의 운영 권장 절.

## MuJoCo 다양한 경로 생성 연결

- `mujoco-benchmark/scripts/candidate_experiment.py`: 기존 Isaac Sobol staged6d/IK/FCL 계획 → MuJoCo CPU 작업자별 서로 다른 경로 실행.
- 매 스텝 상태 저장, HTML/CSV, `replay_candidate.py`로 원본 상태 재생.
- 단순 벤치마크와 별도 실행기. 상세 명령/판정 한계: [CANDIDATE_SEARCH.md](mujoco-benchmark/CANDIDATE_SEARCH.md).
- 현재 부분 중심 진입/과도 변위/물리 이상 진단이며 꼭지 걸림 성공 평가기는 아직 아님.

- 경로 탐색 기본 출력 위치 변경: `/root/docker_share/mujoko_debugging_data/YYYYMMDD_HHMMSS_candidate_search/`. `candidate_experiment.py --output`으로 개별 지정 가능.

## D435 RGB-D 촬영

- `mujoco-benchmark/scripts/d435_capture.py`: 기존 ROS/Isaac URDF 광학 프레임 체인과69.4° FOV 사용, 저장된 후보 상태에서RGB/native depth/color-aligned depth와 행렬 저장.
- 사용법과 실물 센서 모사 한계: `mujoco-benchmark/D435_RGBD.md`.
- 후보87 preapproach/entry 샘플은 `/root/docker_share/mujoko_debugging_data/20260922_111838_candidate_search/candidates/candidate_00087/rgbd/`.

## Tomato_05 시점 변경 관측 준비 완료

- `prepare_observations.py`로 초기 상태 고정, 가상 D435 이동·회전, 자동 타깃 중앙정렬 없이71개RGB-D/GT목표표시/3배·6배크롭 촬영.
- 최종 데이터: `/root/docker_share/mujoko_debugging_data/20260922_122302_tomato05_observations/` (`index.html` 갤러리). 약192MiB.
- 기존 `20260922_111838_candidate_search`의1,000개경로/결과와 연결, per-view camera좌표라벨 제공. 당시 관측 생성 단계에는 새 물리시험0회. 이후 재실험·학습은 아래 기록 참조.
- `validate_observations.py`의71,000관측-후보pair좌표검증 및RGB/Depth/mask/crop검증통과. 대상gt마스크ID보간을피하기위해촬영MSAA비활성.
- 가상카메라 변환은 실제관절만으로복구불가. `camera.json`의유효tool_from_optical/world_from_optical 사용.
- 실제로봇도달성/새토마토일반화는검증하지않음. 상세: `mujoco-benchmark/OBSERVATION_DATASET.md`.

## 카메라 기준 action v2 및 1,000개 재실험 (2026-09-22)

- `camera_action.py`: 카메라 광학축 기준 진입 방향/고리 quaternion/거리/상승 방향의14차원 입력. 절대 시작 위치는 제외, 정확한 재연용 토마토 중심 상대 경유점은 별도 보존. 중력 방향도 관측별 저장.
- `candidate_experiment.py` → `action_frame.json`, `plan.json`/`result.json:action_camera`. 기존6개변수/월드경로 보존. 보고서 방위각은 명시적으로 월드 기준이라고 표시.
- 새 실험 `/root/docker_share/mujoko_debugging_data/20260922_124028_camera_action_v2_1000/`: CPU48/계획16,120Hz,seed0,1,000개 완료, 총510.83초. 부분중심진입36/과도변위908/miss56. 중심진입 자체는200개(과도변위 포함). 오류0. 이전111838실험과모든분류/최대변위동일.
- 기존71개관측은 `upgrade_observation_actions.py --run NEW_RUN`으로 새결과에연결. `actions_camera_v2.npz:features`를학습에사용하고 `candidate_ids`로`actions.json`과join. 원래초기카메라의단일action_camera를다른관측에그대로사용하면안됨.
- 사진과물리장면은보존. 모델해시/초기qpos동일확인. 71,000쌍좌표검증최대위치오차3.33e-16m. 카메라병진에대한불변성/회전에대한월드복원검증통과. v1원본메타는schema_v1_backup보존.
- 이 저장 형식 변경 이후 학습 완료(아래 참조). 꼭지걸림성공평가기는미구현. 동일Tomato05의시점변경만있으므로영상없는기준모델과비교필요.

## Tomato_05 카메라 기준 후보 학습 예비 비교 (2026-09-22)

- `mujoco-benchmark/learning/train.py`: 고정 DINOv2/ResNet18의 local/context RGB 특징 + metric Depth/validity/K + 카메라 action14/중력3 → 부분 중심 진입 확률/최대 이동량. 영상 없는 대조군 포함 3종×3seed 실제 GPU 학습, 41.21초.
- 결과 HTML/가중치/CSV/분할/코드 사본: `/root/docker_share/mujoko_debugging_data/20260922_130308_camera_action_training/`.
- 후보 train699/val149/test152와 관측 train50/val7/test7/잘림검사7을 별도 분리. 새 시점+새 후보가 주 평가. 모델/정규화/체크포인트 선택은 train/val만 사용.
- 테스트 AP: 영상 없음.837 / ResNet.849 / DINO.855. 이동 MAE 2.575/2.535/2.618mm. 검증 손실은 영상 없는 모델이 최소라 공식 선택도 영상 없음. 영상 모델의 확실한 우위 또는 sim-to-real 성공을 주장하지 않음.
- `predict.py`로 저장 관측의 후보 순위 재계산 가능. 세 모델 checkpoint 재로드/분할 비중복 확인 완료. 새 실물 이미지 배포는 미구현. 추가 physics 검증은 다음 절 참조.
- 모델 조사 근거와 명령: `mujoco-benchmark/learning/README.md`. DINOv3는 공식 가중치 접근 신청 필요해 미학습. 단일 장면 반복 관측의 한계/영상 없는 기준과의 비교를 계속 유지할 것.

## 학습 추천의 실제 물리 검증

- `learning/test_physics.py` / `render_physics.py` 추가. 평가용7시점×3모델(seed0)의1순위추천21회를독립reset후MuJoCo120Hz새실행. 중복제외3경로.
- `/root/docker_share/mujoko_debugging_data/20260922_model_recommendation_physics/index.html` 결과/상태/대표3개2배속영상. 물리41.52초(렌더제외).
- ResNet/DINO각진입7/7·20mm이하7/7. 영상없는모델진입7/7·20mm이하4/7. 초기상태동일/원본결과일치/불안정0확인.
- 기존카탈로그경로를추천해관절명령재실행. 새trajectory생성·새장면일반화·꼭지걸림성공실험은아님.

## Tomato01~11 다중 대상 수집 (05 제외)

- `candidate_experiment.py --target Tomato_XX`와 `RobotEngine(target=...)`로계획기하/과실진입판정/변위측정대상을선택. 기본05보존.
- `prepare_observations.py`는manifest대상으로GT과실마스크/라벨/action좌표자동연결. 가상카메라기준위치는05대비대상중심차만큼이동하고원래장착카메라pose와이동량을명시. 로봇실제관측자세검증은아님.
- `collect_tomatoes.py --targets 1,2,3,4,6,7,8,9,10,11 --candidates 1000 --workers 48 --planning-workers 16`으로대상순차물리시험→RGBD저장. 전체10,000개본수집은사용자가실행.
- 명령/출력/공간/카메라한계: `mujoco-benchmark/MULTI_TOMATO_COLLECTION.md`.
- 다중대상시상태재연카메라/HTML대상표시도연결. 기존데이터변경없음.

## 후처리 최적화와 재개 기능 (최신 추가)

- `camera_action.py`: 고정 기하 일괄 준비 / 카메라 변환·검증 벡터화. 관측별 압축·검증은 기본8 CPU 스레드. `prepare_observations.py`의 v1 중복 검증 제거, 최종v2 모든검사유지.
- 71,000쌍 후처리 131.81초→1.41초. 71장 촬영 포함65.61초(장면 초기화 제외). 기존RGB/Depth/mask각71파일완전일치, action오차1e-12이내. 물리 약8분은변경없음.
- `collect_tomatoes.py --resume --output 기존폴더` + `resume_candidates.py` 추가. 개별결과/상태/명령해시검사후누락만실행. 기존계획/완성관측재사용, Ctrl+C시단계프로세스그룹정리.
- 실제중단수집 `20260922_134834_multi_tomato_collection`: 01~04완료,06은정상931개/남음69개검사확인. 사용자대신본수집을다시시작하지않음. 재개명령은 MULTI_TOMATO_COLLECTION.md 최신절.
- 성능검증복사본 `20260922_144727_postprocess_benchmark/` 보존. 중단복구/원래물리결과일치/손상데이터검출확인.


## 2026-09-22 진입 탐색 범위 변경

새 staged6d 후보는 진입각 −45°~+45°, 하부 여유 0~10mm를 Sobol 샘플링한다. MuJoCo와 Isaac의 공통 생성기 `nvidia-sim/rl/trajectory_search.py`에 적용했다. 여유는 초기 열매 충돌 구의 아랫면과 고리 와이어 윗면 기준이다. 0mm는 기하학상 여유가 없는 조건이며 실제 접촉은 물리 엔진의 접촉 설정에도 영향을 받는다. Sobol은 끝값 조합을 반드시 포함하지 않는다. 기존 데이터/저장된 후보/재생은 변경하지 않았고, resume은 저장된 후보를 계속 사용한다. 같은 seed와 candidate_id라도 새 실행은 이전 범위와 다른 경로이므로 실행 폴더를 구분한다. 실제 새 범위 물리 실험은 아직 수행하지 않았다.

## 2026-09-22 Tomato_06 자유 동작 RL 파일럿

- `mujoco-benchmark/scripts/train_tomato_rl.py`: mjlab/MJWarp + RSL-RL PPO, 고정 Tomato_06 GT 장면에서 팔6관절+리프트7액션 학습. 회전 고정/직선 trajectory 제약 없음.
- 상세 실행·판정·제약: [TOMATO06_RL.md](mujoco-benchmark/TOMATO06_RL.md). 기존 데이터 수집/모델은 유지하고 RL 전용 모델에 자기충돌을 추가했다.
- 학습 완료 후 결정적 독립 평가 상태를 저장하며 `replay_tomato_rl.py`로 재생 가능. 짧은 검증은 학습 루프/물리 안정성 확인이며 수확 성공을 주장하지 않는다.

- 최종 16환경/20업데이트 검증: 학습28.88초, 준비·평가 포함45.93초(커널 캐시 사용). 10,240 전이, 학습32에피소드 및 독립평가 모두 부분 진입 성공0. 성공 정책이 아니라 실행 루프 확인 완료. 가중치 업데이트/reset/체크포인트 로드/재생 검증 통과. `mujoco-benchmark/validation/tomato06_rl_smoke.json`에 결과 보존.

## 2026-09-22 GPU 환경 확대 및 추가 학습 실행

기존 사용자 실행 `20260922_184128_tomato06_rl`의 200회 체크포인트에서 추가 학습을 시작했다. 짧은 동일 조건 비교(3초 에피소드, 32스텝, 4업데이트): 64환경 1,150전이/초, 256환경 2,352전이/초. 256환경에서 관측한 GPU 사용률 95%, 메모리4,414MiB(평균/최댓값 아님). 기존16환경 357전이/초는 30초 에피소드의 장기 측정이라 엄밀히 동일 조건 비교는 아니다.

- 실행 중: `/root/docker_share/mujoko_debugging_data/20260922_185257_tomato06_rl_gpu256`
- PID 286916 (종료/상태 확인 전 프로세스 명령을 다시 확인할 것)
- 환경256, 추가1,000업데이트, 32스텝, 에피소드30초. 총8,192,000전이. 기존 물리/보상 유지.
- 로그: 같은 경로 뒤 `.log`; 실행 인자/PID는 `.launcher.json`. 25업데이트마다 체크포인트, 완료 후 summary/평가 재생 데이터 저장.
- progress.jsonl에 처리량·누적 학습시간·잔여 예상분 추가. 약1시간 예상이나 접촉 복잡도에 따라 달라짐. 이 기록 작성 시 본 학습은 진행 중이며 성공 여부 미확인.
- 측정 보존: `mujoco-benchmark/validation/tomato06_rl_scaling.json`.

## 10분 학습 비교로 전환

사용자 요청으로 기존 1,000회 추가학습(PID286916)은 중단. 해당 실행의 model_00025.pt를 보존했으며 이후 미저장 업데이트는 보존되지 않았다. 비교의 출발점은 원래 사용자 200회 체크포인트다.

`--train-seconds 600 --compare-before --iterations 100000`으로 학습 구간만 600초 제한, 진행 중인 업데이트 종료까지 몇 초 초과 가능. 준비/기준평가/최종평가는 별도. 실제 완료 업데이트와 전이 수 기록, 마지막 체크포인트 저장. 2초 제한 검증에서 2.76초/2업데이트 후 자동 종료·저장·전후 비교 통과.

현재 실행: /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min

PID 291847; 로그: /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min.log

완료 후 summary.json의 comparison에 전후 성공수, 평균 최근접 거리, 평균 최대 열매 이동량 저장. baseline_world0.npz와 evaluation_world0.npz 저장. 실제 향상 여부는 완료 후 확인해야 한다.

## 10분 추가 학습 완료 및 진단

`20260922_185721_tomato06_rl_10min` 완료: 학습600.43초, 전체735.75초, 170업데이트/1,392,640전이. 학습2,306에피소드 성공0, 결정적 평가 전후 성공0. 평균 최근접 중심 거리56.16→43.17mm, 평균 최대 열매 이동량1.13→1.13mm. 결과 요약은 `mujoco-benchmark/validation/tomato06_rl_10min.json`에 보존.

평가 궤적은 약5초에 접근한 뒤30초까지 비슷한 위치에 머문다. 현재 근접 항은 정지 상태에서도 시간 비용 차감 후 약+0.024/제어스텝(다른 감점 전)을 지급한다. 성공 경험이 없는 정책에서 머무르기 보상과 밀림 실패 페널티가 정체를 유도했을 가능성이 크다. 성공 조건은 고리 평면2mm 이내/0.2초 유지이며 좁다. 다음 작업 후보는 정지 근접 보상 제거와 실제 정렬/삽입 진전 보상 설계, 물리적 진입 가능성 점검이다. **보상 수정은 아직 하지 않았다.**

이전 문단의 '실행 중'은 과거 기록이다. 1,000회 실행은 사용자 요청으로 중단했고 10분 비교 실행은 정상 완료했다. 학습 모델/대형 상태 기록은 docker_share에 유지하며 Git에는 코드·문서·작은 검증 요약만 포함한다.

## 최신 변경: 보상 progress_v2 (학습 실행 안 함)

사용자 요청으로 보상만 변경. 정지 근접 보상과 반복 내부 체류 보상을 제거했다. 기존 목표점 거리 감소20배 + 고리 좌표계의 포획 조건 위반 거리 감소20배로 진전을 보상한다. 포획 오차는 반원 내부/고리 평면/와이어 여유의 위반량이며 특정 세계 방향이나 경로를 고정하지 않는다. 최초 안전 부분 진입은 에피소드당1회 +2, 기존0.2초 유지 성공 +10. 기존 밀림/접촉/급격한 액션/시간 감점과 실패 -5 유지. 후퇴는 signed progress 감점이지만 물리적으로 금지하지 않는다.

물리·관측·행동·종료·성공 기준·학습기는 변경하지 않았다. manifest의 reward_version=progress_v2로 구분한다. 정지/진전/후퇴/왕복/최초 진입/포획 오차 산술 검증 통과. 수정 보상의 실제 학습 개선 여부는 아직 미검증. 학습은 사용자가 직접 실행한다.

2시간 추가 학습(이전10분 모델에서 시작):

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.mjlab-venv/bin/python -u \
  mujoco-benchmark/scripts/train_tomato_rl.py \
  --num-envs 256 --iterations 100000 \
  --steps-per-env 32 --episode-seconds 30 \
  --train-seconds 7200 --compare-before \
  --checkpoint /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min/model_00170.pt
```

학습 구간2시간이며 업데이트 경계까지 수 초 초과 가능, 준비·전후 평가는 별도다. 새 datetime 폴더로 저장. 기존 보상과 점수 크기가 달라졌으므로 보상 합보다는 독립 평가 성공/거리/변위를 비교한다.

## 최신 데이터 생성 조건: 각도만 탐색

새 staged6d/Sobol 후보는 1차원 scrambled Sobol로 진입 방위각 −45°~+45°만 바꾼다. GT 중심에 인식 오차 없음. 좌우 오프셋0mm, 하부 여유2mm 고정. 삽입42.5mm/수직 상승32.5mm는 이전 범위 중간값으로 고정, pre-hook170mm/roll0/elevation0 유지. 이 조건은 중심선을 향하는 계획이며 접촉/변형/제어 오차까지 제거하거나 실제 진입 성공100%를 보장하지 않는다. 삽입 깊이도 모든 후보에서 동일하고 열매 중심 도달을 자동 보장하지 않는다.

공통 trajectory_search 생성기와 새 실행 metadata에 sampling=scrambled_sobol_azimuth_only 기록. 파일/CLI trajectory_mode=staged6d 명칭은 호환성을 위해 유지. 기존 저장 후보/재생/계속 실행(resume)은 이전 경로를 유지하므로 새 조건 수집은 새 실행 폴더로 시작해야 한다. 이전 seed/candidate 번호와 새 경로는 다르다. RL 제어에는 적용하지 않는다.

생성/기하 검사5개 통과(1,000후보에서 각도만 다름, 좌우 중심 정렬, 와이어 여유2mm 확인). 물리 실행/데이터 수집은 실행하지 않았다.

## 두 구간 45° 상승 적용

새 후보에 `lift_profile=diagonal_45_return`을 저장한다. 진입 방향 기준 전진16.25mm+상승16.25mm 후, 후퇴16.25mm+상승16.25mm. 최종 위치는 이전 수직32.5mm 상승과 동일하며 고리 회전은 바꾸지 않는다. 경유점명은 `rise_mid`, 마지막은 `rise`; 실제 제어 단계는 두 구간 모두 rise로 처리하여 동일 상승 속도/접촉 판정을 적용한다. 이동 거리는 기존 상승의 √2배이므로 동일 속도에서는 상승 시간이 길어진다.

기존 저장 후보에 profile이 없으면 기존 상승을 유지한다. 새 방식은 waypoint5개를 카메라 좌표에도 모두 저장한다. 기존14개 compact features의 lift 방향/거리는 최종 순변위이며, 이14개만으로 직선 상승과 대각 상승을 구분할 수 없다. 두 방식을 학습용으로 섞으려면 phase_names/전체 waypoint 또는 별도 profile 입력을 사용해야 한다. 서로 다른 phase 구조를 한 batch로 섞으면 명시적으로 거부한다.

기하 검사6개 통과, 기존4점/새5점의 카메라 좌표 변환·복원 검증 통과.

## 중심 진입 단독 성공 기준

새 MuJoCo 데이터 수집의 classification_rule=center_entry_only_v2: 물리 오류는 invalid_physics, 정상 실행에서 중심 진입 이력이 있으면 열매 최대 밀림과 무관하게 partial_center_entry, 없으면 miss. target_center_max_displacement_m과 target_displacement_exceeded(20mm 초과)는 참고 지표로 별도 저장. 결과 JSON/CSV와 HTML의 판정 설명/밀림 집계를 갱신했다.

기존 데이터는 자동 덮어쓰지 않았다. 과거 manifest에 rule이 없으면 displacement_first_v1로 이어 실행하여 혼합 판정을 방지한다. 기존 ±90°/30개 데이터를 읽기 전용으로 재분류 검증하면 중심 진입 성공3개, 미진입27개다. RL 보상/종료 조건은 이번 변경 대상이 아니다.

## 다중 토마토 RGB-D 진입각 학습 완료

데이터10대상×30경로/710관측(05없음). 8대상 학습,09검증,06테스트로 분리. 진입 BCE만 학습하며 밀림 감점 제거. DINOv2 seed2 선택,06의64시점 모두−81.09° 후보00001 추천. 고유 경로1개 새 물리 실행에서 중심진입 성공(밀림30.78mm). action_only도동일 top1성공으로 영상 우위 미입증. 전체 후보 확률 calibration 불량(Brier약0.315). 상세/명령: [learning/MULTI_TOMATO.md](mujoco-benchmark/learning/MULTI_TOMATO.md). 결과 `/root/docker_share/mujoko_debugging_data/20260922_234403_multi_tomato_training/index.html`. 초기 요청 커밋3f636c2 이후 학습기 확장 작업.
