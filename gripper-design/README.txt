Inaho Gripper 디자인 페이지

접속: http://192.168.222.27:8090/gripper-design/
프로젝트 목록: http://192.168.222.27:8090/

index.html: 프로젝트 목록 복귀, STEP/GLB/출력용 ZIP 다운로드, PDF 설명서 연결.
inaho_gripper/: V5.1로 동기화한 뷰어 HTML, STEP, GLB, 출력용 ZIP, PDF 설명서.
inaho_gripper_v5/: 수정 가능한 설계·뷰어 소스와 생성 모델, STL, BOM, 검사 결과.
REPAIR_REPORT_KO.txt: 전체 점검 결과와 실제 제작 시 교체할 부품 안내.
preview.webp: 홈 카드에 표시하는 실제 모델 화면 미리보기.

자체 포함 HTML 뷰어를 iframe에 표시합니다. 전체 부품의 분해 이동과 체결 규격을 점검하여 V5.1로 수정했습니다.
노즈 고정은 ISO4026 M3x6 네 개, 장력 잭은 전산형 M3x25 두 개를 사용합니다.
ISO7379 숄더 나사의 목 부분을 피하도록 좌우 C01 새들 너트 포켓을 수정했으므로 V5.1 STL로 다시 출력합니다.
소스 트리 README_KO.txt의 재현 순서대로 생성하고 검증한 후 source/package_release.py가 만든 전달 파일을 inaho_gripper/에 반영합니다.
설계 파일과 페이지는 farmily_tomato 저장소에서 함께 관리합니다.
통합 서버 /root/docker_share/farmily_design_test_v2/serve.py가 이 폴더를 /gripper-design/으로 제공합니다.
폴더 위치 변경 시 GRIPPER_DESIGN_ROOT를 설정합니다. 별도 서버나 파일 복사 작업은 필요 없습니다.
다른 버전 파일을 추가하면 index.html의 iframe 및 다운로드 경로를 함께 변경하세요.

검증 (통합 서버 실행 중):
  PYTHONPATH=/tmp/farmily-s300-tools/python python /root/docker_share/farmily_design_test_v2/tools/check_gripper_project.py
