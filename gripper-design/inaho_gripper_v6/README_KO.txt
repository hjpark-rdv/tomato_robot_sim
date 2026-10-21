Inaho Gripper V6.0 — 정면 개방형 경사 U자 토마토 받침

V5.1의 벨트·모터·개폐 링크·외장 구조를 기준으로 하부 받침을 바꾼 시안입니다.
사진에서 관찰한 형태를 참고했습니다. 실제 inaho 양산 도면이나 실측 치수의 복원이 아닙니다.

변경 부품
  L_T01_sloped_receiver → L_T01_deep_U_receiver
  기존 좌측 유닛의 두 체결점, M3x16 볼트 2개와 M3 너트 2개를 유지합니다.
  체결점 (X,Y): (-54,46), (-50,144) mm. 조립 좌표 / 간격44 기준.
  개폐 시 받침 전체가 좌측 유닛과 함께 이동합니다.

형상 — 정면 개방 및 돌출 입구 수정
  첫 V6 시안의 반원형 정면 막음판을 완전히 제거했습니다.
  전방(+Y) 입구와 후방(모터 방향) 배출구가 모두 열린 U자형 통로입니다.
  본체 폭122 mm, 입구 최대 폭138 mm, 전체 길이188 mm, 기본 벽3 mm.
  입구는 기존 끝 Y174에서 Y202까지28 mm 돌출되고 좌우 각각8 mm 벌어집니다.
  입구 옆벽 선단은 측면에서 R18로 둥글게 내려가며 별도 막음 턱은 없습니다.
  본체 바닥은 열린 입구가 높고 모터/배출구 쪽이 낮은12도 경사입니다.
  본체 바닥: Y174에서 Z-134, Y14에서 약Z-168 mm.
  확장 입구 바닥도 같은 경사를 이어가며 앞쪽 중앙에 턱을 추가하지 않았습니다.
  본체 내측 반경 R58. 기둥 상단 Z-2, 전체 높이 약166 mm.
  경사 방향과12도는 사진을 참고한 설계 가정이며 실측값이 아닙니다.
  과실 이송·적체·배출은 로봇 자세와 과실 크기별 실물 확인이 필요합니다.

파일
  models/inaho_gripper_v6_assembly.step: 전체 조립체 / mm /356개 솔리드.
  models/inaho_gripper_v6_assembly.glb: 전체 웹 모델 / m /Y-up.
  models/V6_L.step 등: 하위 조립체.
  step_parts/L_T01_deep_U_receiver.step: 새 받침 단품 / 조립 좌표.
  stl/L_T01_deep_U_receiver.stl: 새 받침 출력 파일 / mm /앞면을 바닥으로 정렬.
  stl/: 전체 출력 부품66개 + 기존 맞춤 시험편5개.
  docs/inaho_gripper_v6_assembly_guide_KO.pdf: 변경부 조립 안내.
  docs/common_mechanism_guide_V5.1.pdf: 변경되지 않은 공통 기구의 기존 가이드.
    V5 받침 관련 페이지와 구형 받침 STL은 V6에 적용하지 않습니다.
  source/: 재현 가능한 CAD·뷰어·검증·문서·패키지 생성 코드.
  checks/: 새 받침 간섭, 내보내기, 브라우저 검증 결과.

조립
  기존 좌측 하부 데크의 볼트2개와 받침을 분리합니다.
  V6 받침의 옆으로 열린 포켓에 M3 너트2개를 넣습니다.
  받침의 기둥을 기존 데크 아래에 맞춘 후 기존 M3x16 볼트2개로 체결합니다.
  지지대의 폭과 깊이가 바뀌므로 호스트 로봇·작물·주변 구조물과의 여유를 확인합니다.
  앞면 기준 출력 방향입니다. 벌어진 입구, 경사 옆벽과 너트 포켓의 서포트를 슬라이서에서 검토하세요.
  좁은 접지면의 들뜸 방지를 위한 브림 및 방향별 지지 안정성도 확인하세요.
  새 받침의 출력 외접 크기는 약138×166×188 mm로220 mm 출력 영역에 들어갑니다.

검증 범위
  변경되지 않은351개 부품의 체적·경계·그룹·규격을 V5.1 결과와 비교했습니다.
  변경 받침을 다른 모든 부품과 개폐28~96 mm의35개 위치에서 교집합 검사했습니다.
  상세 판정 및 오차 기준: checks/v6_receiver.json.
  받침 STEP/STL과 전체 STEP/GLB를 다시 읽어 솔리드 및 메시를 확인했습니다.
  내보낸 STEP의 전방 개구부, 양쪽 돌출부 및 실제 바닥12도 경사도 검사합니다.
  명목 치수의 강체 CAD 검사입니다. 실물 제작·파지력·낙하 충격·공차·PETG 강도·과실 손상은 검증하지 않았습니다.

재현 (source/requirements.txt 환경)
  python source/build_v6.py --export
  python source/verify_v6.py
  python source/build_viewer.py
  python source/build_guide.py
  python source/check_browser.py
  python source/package_release.py
브라우저 검사는 Playwright와 Chromium 설치가 필요합니다. --url / --browser 옵션으로 경로를 지정합니다.
V5.1 원본 폴더가 함께 있으면 verify_v6.py가 그 파일들의 변경 여부도 확인합니다.
