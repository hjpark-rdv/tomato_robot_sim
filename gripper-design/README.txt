Inaho Gripper 디자인 페이지

접속: http://192.168.222.27:8090/gripper-design/
프로젝트 목록: http://192.168.222.27:8090/

index.html: 프로젝트 목록 복귀, STEP/GLB/출력용 ZIP 다운로드, PDF 설명서 연결.
versions.js: V5.1/V6 버전 전환과 현재 선택 버전의 다운로드 연결. 기본 V6.
직접 비교: /gripper-design/?version=v5 또는 /gripper-design/?version=v6
수확로봇 조립 탭: /gripper-design/?tab=robot
농장 수확 재생 탭: /gripper-design/?tab=replay
실제 Isaac 실행 기록을 선택해 전체 농장·로봇·가지 변형·열매 분리를 3D로 재생합니다.
시간 탐색, 일시정지와 배속을 지원합니다. 새 기록 등록은 harvest-replay/README.txt를 참고하세요.
로봇 탭은 S300 기체·리프트·RB5·현재 시뮬레이션 장착 그리퍼를 표시합니다.
그리퍼 설계의 V6와 달리 현재 장착 모델은 V5.1 기반 시뮬레이션 수정형입니다.
구성품 표시 전환, 정면/후면/측면/위 보기, 그리퍼 확대, 원본 GLB 다운로드를 제공합니다.
inaho_gripper/: V5.1로 동기화한 뷰어 HTML, STEP, GLB, 출력용 ZIP, PDF 설명서.
inaho_gripper_v5/: 수정 가능한 설계·뷰어 소스와 생성 모델, STL, BOM, 검사 결과.
inaho_gripper_v6/: V5.1 기구 기반의 개방형 경사 U자 받침. 정면 막음판 제거, 28 mm 돌출 입구, 모터 쪽으로 12도 하향 경사 시안. CAD·STL·뷰어·조립 안내·검증 결과.
REPAIR_REPORT_KO.txt: 전체 점검 결과와 실제 제작 시 교체할 부품 안내.
inaho_gripper_v6/preview.webp: 홈 카드에 표시하는 현재 V6 모델 미리보기. 루트 preview.webp는 기존 V5 미리보기.

자체 포함 HTML 뷰어를 iframe에 표시합니다. 전체 부품의 분해 이동과 체결 규격을 점검하여 V5.1로 수정했습니다.
노즈 고정은 ISO4026 M3x6 네 개, 장력 잭은 전산형 M3x25 두 개를 사용합니다.
ISO7379 숄더 나사의 목 부분을 피하도록 좌우 C01 새들 너트 포켓을 수정했으므로 V5.1 STL로 다시 출력합니다.
소스 트리 README_KO.txt의 재현 순서대로 생성하고 검증한 후 source/package_release.py가 만든 전달 파일을 inaho_gripper/에 반영합니다.
설계 파일과 페이지는 farmily_tomato 저장소에서 함께 관리합니다.
통합 서버 /root/docker_share/farmily_design_test_v2/serve.py가 이 폴더를 /gripper-design/으로 제공합니다.
폴더 위치 변경 시 GRIPPER_DESIGN_ROOT를 설정합니다. 별도 서버나 파일 복사 작업은 필요 없습니다.
다른 버전 파일을 추가하면 versions.js의 뷰어·다운로드 경로와 index.html의 선택 항목을 함께 추가하세요.

2026-10-21 실제 리프트 받침대 연결:
robot-assembly/index.html, viewer.js: 조립 뷰어. 기존 /s300/vendor/three/ (r180) 재사용.
가상 상판 제거. 원본 assy_lift의 받침판/뒤쪽 승강판/삼각 브래킷이 RB5/그리퍼와 함께 이동합니다.
기둥, 외측 바닥 지지대, 모터와 S300은 고정됩니다. 원본 120,881개 리프트 삼각형을 그대로 보존합니다.
0~750mm 상승량 슬라이더와 승강 보기 버튼을 제공합니다. 이것은 조립 기구 미리보기이며 라이브 물리 제어가 아닙니다.
GLB 전체 삼각형 2,938,929개. RB5 재질 색상 유지, 가상 상판 12개 삼각형만 제거했습니다.
원본 가시 메시 분리 후855개, GLB 약94.1MiB. gzip 압축 전송과 원본 GLB 다운로드를 지원합니다.
장착 그리퍼는 V5.1 기반 실제 시뮬레이션 수정형이며, 별도 V6 설계와 구분합니다.

수정 스크립트: nvidia-sim/harvest_s300/real_lift_mount.py
원본 USD를 보존하고 새 assembly를 생성합니다. 기존 리프트 관절·제어·질량 추정 유지,
RB5 장착높이 -2.931mm 보정, 기존 가상 판 collider는 실제 받침판 hull로 교체합니다.
나머지 리프트 CAD 부품은 이전처럼 시각 형상이며, 전체 CAD 충돌 인증은 아닙니다.
새 source/manifest/MJB로 재계획해야 하며 이전 수확 trace의 검증 결과를 승계하지 않습니다.

결과: /root/docker_share/mujoko_debugging_data/20261021_230320_real_lift_mount/
assembly/composed.usdc: 농장 배경까지 보존한 새 조립
assembly/robot_check: 기존 USD->MJCF adapter로 내보낸 로봇 단독 물리 모델
lift_cpu, lift_gpu: 각각 23초 실제 승강. GPU는 기존 Newton/MuJoCo Warp adapter 사용.
관통0, CPU/GPU 모두 완료. 기존 제어의 정지 높이 오차 약8.51mm는 남아 있습니다.
전체 농장 수확, 실물 스트로크/질량/처짐 검증을 의미하지 않습니다.

cd /root/farmily_tomato
./nvidia-sim/.asset-venv/bin/python gripper-design/robot-assembly/export_model.py \
  /root/docker_share/mujoko_debugging_data/20261021_230320_real_lift_mount/assembly \
  /root/docker_share/mujoko_debugging_data/20261021_230320_real_lift_mount/lift_cpu/states.npz

robot-assembly/models/model.json: source/상태/GLB SHA, 부품 분류와 리프트 기준 높이.
대형 robot-assembly/models/*.glb 및 *.glb.gz는 Git에서 제외합니다.
다른 서버에서는 위 export_model.py 명령으로 재생성하거나 현재 모델 파일을 별도 복사합니다.
robot-assembly/checks/real_lift_validation.json: 기구/CPU/GPU/브라우저 검증 요약.
수정 전 GLB/metadata는 위 결과 폴더의 previous_web_models/에 보존했습니다.

검증 (통합 서버 실행 중):
  PYTHONPATH=/tmp/farmily-s300-tools/python python /root/docker_share/farmily_design_test_v2/tools/check_gripper_project.py
