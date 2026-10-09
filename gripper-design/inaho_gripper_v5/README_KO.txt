FARMILY / inaho 형태 듀얼 벨트 그리퍼 V5.1

이 패키지는 조립 가능한 기구 시제품의 CAD·출력 파일입니다.
실제 출력·조립·파지력·작물 손상·반복 수명 시험은 수행하지 않았습니다.
inaho 생산 도면을 복원한 모델이 아니며, 공개 외형과 사용자의 동영상 관찰을 반영했습니다.

1. V5 반영 사항
- 좌우 본체 높이 차이 0 mm. 데크와 외측/후방 커버 높이 동일.
- 좌우 벨트 끝단 4 mm 비대칭. L Z9~61 / R Z13~65, 폭 52 mm.
  4 mm는 원본 실측값이 아닌 본 설계의 선택값입니다.
- 앞뒤 위치 차이 14 mm 유지. 전방은 +Y, 높이는 +Z, 개폐는 X.
- 롤러와 축 스페이서 재설계. 20T/32T 타이밍 홈 중심 Z86.75 정렬.
- 후방 커버 M3x12 세 개를 구조/모터 조절용 볼트와 분리.
- 실제 제조사 치수의 20T 풀리, 낮은 버튼헤드 캡 볼트, 원점 브래킷/작동 돌기 반영.
- 뷰어 정투상 높이 비교, 벨트/축 회전, 정지 중 불필요한 렌더링 중단.

V5.1 체결·분해 표시 정정
- 노즈 축 고정 나사 네 개: ISO4762 M3x6 + 작은 와셔에서
  ISO4026 M3x6 평끝 무두나사(AF1.5)로 변경. 포켓 M3 너트는 유지.
  머리·와셔의 착좌로 조임이 멈추던 조건을 없애 축 접촉 위치를 조정합니다.
- 기존 노즈 고정용 작은 와셔 네 개는 사용하지 않습니다.
- 텐셔너 잭 M3x25 두 개는 Bossard BN3 또는 동등한 전나사산 제품을
  지정합니다. 부분 나사산 제품은 머리 가까운 잭 잠금너트가 민축에 놓일 수 있습니다.
- C01 새들 두 개의 M5 힌지 너트와 포켓을 뒤로 1.5 mm 이동하여
  ISO7379 숄더 뒤 최대 2 mm의 민축 구간을 피했습니다.
  기존 V5.0 C01 출력물은 V5.1 STL로 다시 출력합니다.
- 상부 노즈 고정 나사는 상부 암 장착 후 설치하고, 축 스택을 맞춘 뒤
  두 끝을 고정합니다. 롤러 자유 회전과 축 미끄러짐 방지를 실물 확인합니다.
- 분해도에서 체결 부품이 장착 대상과 다른 위치에 남던 이동 조건을 정정.
  분해 위치는 장착 치수가 아닙니다. 실제 조립 순서는 PDF를 따릅니다.
- 누락된 C13 외측 커버와 C12 슬라이드 체결 절차를 조립 가이드에 추가.

2. 파일
inaho_gripper_v5_viewer.html : 오프라인 Three.js 뷰어. 내려받은 뒤 브라우저에서 열기.
models/inaho_gripper_v5_assembly.step : 전체 조립체, mm, 부품 이름/색 포함.
models/V5_L.step, V5_R.step, V5_fixed.step, V5_linkage.step : 하위 조립체.
models/inaho_gripper_v5_assembly.glb : m / Y-up 모델.
step_parts/ : 출력 부품 66개와 맞춤 시험편 5개의 개별 STEP. 조립 좌표 유지.
stl/ : 71개 개별 STL. mm, 출력 방향 및 바닥 정렬 적용.
docs/inaho_gripper_v5_assembly_guide_KO.pdf : 조립 순서, 축 스택, 치수, 검사 범위.
docs/PRINT_LIST.csv : 출력 파일별 재료/방향/크기.
docs/PURCHASE_BOM.csv : 모델에 포함된 구매 부품과 수량.
docs/AUXILIARY_BOM.txt : 케이블·타이·전장·호스트 인터페이스 등 별도 준비 항목.
docs/JOINT_STACKS.csv : 체결 길이·나사 물림·막힌 구멍 여유.
checks/ : 기구 및 내보내기, 뷰어, PDF 검증 결과.
source/ : 설계·검증·내보내기·뷰어·가이드 제작 소스.
FILE_SHA256.json : 패키지 내 파일 무결성 확인용 SHA-256.

3. 조립 전에
- PDF를 먼저 읽고 FIT_ 시험편 5종을 실제 출력 재료와 설정으로 출력합니다.
- PETG 구조물, TPU95A 접촉 벨트. 모터·축·베어링·레일·타이밍 벨트·나사는 구매품입니다.
- 제공 STL의 최대 외접 치수를 220x220x220 mm와 비교했습니다.
  브림·서포트·프린터 금지 영역은 슬라이서에서 추가 확인해야 합니다.
- C07 STL은 장착 전 자유 루프 C143.2, 조립 STEP/GLB는 장착 C144 형상입니다.
- 전체 STL을 임의 확대하면 규격품 피치가 달라집니다. 시험편에 맞춰 구멍 보정/후가공합니다.
- 모터와 레일 블록의 볼트 진입 깊이를 지킵니다. 비표준 길이 나사 및 샤프트의
  완성 길이는 BOM대로 준비하고 절단면의 버를 제거합니다.
- 노즈 무두나사의 조정으로 축 위치가 바뀌면 캐리어 정렬과 벨트 장력을
  다시 맞춥니다. 링크 부시·와셔 스택은 실측하고 너트 체결 후 회전을 확인합니다.
- 원점 스위치 전환 간격 95.6 mm를 실물로 교정합니다. 기계 스토퍼 간격은 96 mm입니다.

4. 검증 범위
- 개폐 39개 위치의 정밀 교집합: 0.01 mm³ 초과 간섭 0건.
  최종 CAD의 교집합 횟수와 상세 위치는 checks/interference_full.json을 참조합니다.
- 회전 16개 조합, 텐셔너/모터 조절 샘플, 주요 부품 삽입 및 공구 접근 검사.
- V5.1 체결 치수·와셔 정렬·노즈 무두나사 조정 여유는 checks/repair_audit.json,
  전체 부품의 분해 소속과 체결 대상 관계는 checks/explosion_audit.json을 참조합니다.
- STL 71개 폐곡면/일체형/양의 체적, STEP 76개 OpenCascade 재불러오기 검증.
- 출력 STEP에서 본체 높이 차이 0 mm와 벨트 끝단 4 mm를 별도 측정.
- STEP에는 SolidWorks 네이티브 메이트가 없습니다. SolidWorks 프로그램에서 직접 실행한
  검사는 아니므로 해당 버전의 가져오기 진단/간섭 검사를 별도로 수행할 수 있습니다.
- 검사한 위치와 명목 치수에 대한 결과입니다. 연속 전 구간, 공차 조합, 강도·피로,
  마찰·벨트 편주·TPU 크리프 및 실제 파지력을 보증하는 해석은 아닙니다.

5. 출력 후
무전원 수동 회전/개폐 -> 벨트 정렬/장력 -> 스위치 교정 -> 저속 무부하 ->
외부 힘계로 더미 파지력 -> 반복 개폐 -> 실제 작물 시험 순서로 진행합니다.
상세 판정과 기록 항목은 PDF에 있습니다.

6. 재현
Python 3.12에서 source/requirements.txt 설치 후 아래 순서로 실행합니다.
  python source/build_v5.py --export
  python source/verify_v5.py
  python source/verify_interfaces.py
  python source/verify_repair.py
  python source/verify_assembly_paths.py
  python source/check_exports.py
  python source/verify_double_check.py
  python source/build_viewer.py
  node source/test_explosion_ownership.cjs
  node source/test_viewer.cjs
  python source/build_guide.py
  python source/check_pdf.py --visual-reviewed
  python source/package_release.py
브라우저 검사는 source 폴더에서 npm install, npx playwright install chromium 실행 후 사용합니다.
V5_CHROMIUM_PATH와
V5_SPARTICUZ로 테스트 실행기의 경로를 지정할 수 있습니다.
--visual-reviewed는 PDF를 렌더링하여 전 페이지를 직접 확인한 뒤 사용합니다.
parameters.json은 설계 요약이며 모든 형상을 독립 제어하는 완전 파라메트릭 입력 파일은 아닙니다.
치수 변경 시 build_v5.py의 연결 형상/스택을 함께 바꾸고 전체 검증을 다시 실행합니다.
외부 라이브러리와 폰트의 사용 조건은 vendor의 LICENSE 파일에 있습니다.
