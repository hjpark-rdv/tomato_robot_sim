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

2026-10-22 카메라 SIM-GT 표시:

cd /root/farmily_tomato
DISPLAY=:0 ./nvidia-sim/run_s300_harvest.sh rgbd-gt

rgbd-test와 같은 승강/카메라에 renderer instance ID 기반 마스크를 추가합니다.
화면 없이 저장하려면 --headless. 기존 RGB-D 전용 명령은 그대로 유지합니다.
결과 폴더의 rgbd/index.html, rgbd/analysis.mp4에서 RGB / GT 표시 / 깊이를 비교합니다.
이 화면은 실제 취득한 카메라 데이터의 표시이며 인식 신경망을 사용하지 않습니다.
원자료는 각 frame_XXXXX 폴더의 semantic.npy(uint8 종류), instance.npy(uint32 객체),
depth.npy(float32 m), camera.json(K/실제 외부변환/시간), objects.json(보이는 객체)입니다.
gt_catalog.json은 클래스/객체 ID, 원본 mesh 경로, plant/truss 소속을 연결합니다.
objects.json의 class_name=fruit, red_proxy=true인 instance_id들을 instance.npy에서
선택하면 현재 보이는 빨간 열매 픽셀만 얻습니다. 같은 plant/truss로 묶을 수 있지만,
그 집단의 도달 가능성·수확 여부는 별도의 판단이 필요합니다.
열매 빨강/주황·노랑/초록은 USD에 지정된 색상 proxy이며 실물 숙도 인증이 아닙니다.
가려진 부분은 채우지 않습니다. 3D 위치는 관측 표면의 중앙값이며 열매 전체 중심이 아닙니다.
수확 영역 선택/도달 가능성 판정은 아직 실행하지 않습니다. 기존 수확 데이터와 분리합니다.
깊이 색상 표시만0.1~2m 고정 범위, 원본은0.1~5m입니다. 별도 GT 권한 옵션이며
이전 RGB-D 캡처를 수정하거나 새 마스크를 기존 학습 데이터에 섞지 않습니다.

2026-10-22 Isaac 하단 관절 세로 RGB-D:

DISPLAY=:0 ./nvidia-sim/run_s300_harvest.sh rgbd-test

실제 리프트 장면의 RB5 link1 바깥쪽 커버에 D435 외형을 세로 장착했습니다.
START 후 승강하면서 480x640 RGB + optical-Z depth(m,float32)를 2Hz로 저장합니다.
GUI의 RGB-D view 창에서 Robot/RGB-D 시점을 선택할 수 있습니다.
화면 없이 촬영하려면 --headless, 촬영 주파수는 --rgbd-fps 5 등으로 지정합니다.
주파수는 physics Hz(이 명령 기본120)를 나누어떨어지게 해야 합니다.
--physics-only는 렌더러를 끄므로 RGB-D 촬영과 함께 사용할 수 없습니다.

rgbd/frame_00000/rgb.png, depth.npy, camera.json 및 rgbd/frames.jsonl 저장.
depth_preview.png는 보기용 회색 깊이 영상이며 segmentation/mask가 아닙니다.
K/실제 카메라 pose/물리 step/time/로봇q를 함께 기록합니다.
이상적인 정렬 RGB-D이며 실물 D435 노이즈/스테레오 매칭/장치 보정은 아닙니다.
카메라/브래킷은 시각 형상이며 기존 물리 질량·충돌체를 바꾸지 않았습니다.

새 기본 장면:
/root/docker_share/mujoko_debugging_data/20261022_065240_shoulder_rgbd/assembly_exposure
결과/영상:
/root/docker_share/mujoko_debugging_data/20261022_065240_shoulder_rgbd/index.html
이번 카메라는 Isaac 모델에 추가했습니다. 웹 조립 탭의 기존 GLB와 과거 웹 재생은
기록 당시 모델을 유지하며 자동 변경하지 않았습니다.

2026-10-22 실제 Isaac 실행에도 리프트 수정 적용:

cd /root/farmily_tomato
DISPLAY=:0 ./nvidia-sim/run_s300_harvest.sh lift-test

전체 농장 로딩 뒤 Lift control의 START 버튼을 누릅니다.
실제 Isaac/PhysX가 받침판·뒤쪽 승강판·삼각 브래킷·RB5·그리퍼를 함께
상승/하강/복귀시키고 기록합니다. 기둥과 기체는 고정됩니다.
완료 후 창을 유지하며, 상태/검사/로그는 새 datetime 폴더에 저장합니다.
모델만 보려면 lift-test 대신 real-lift를 사용합니다.
화면 없이 검사: ./nvidia-sim/run_s300_harvest.sh lift-test --headless --physics-only

기본 수정 장면:
/root/docker_share/mujoko_debugging_data/20261022_062713_isaac_real_lift/assembly_verified_scene
다른 수정 장면/저장 위치는 S300_ISAAC_ASSEMBLY/S300_ISAAC_OUTPUT으로 지정합니다.
새 모델 생성에도 같은 리프트 수정기를 적용합니다. 기존 Newton GPU 입력이나
저장된 수확 경로를 자동 교체하지 않으며, 그 경로는 새 모델로 재검증해야 합니다.
23초 실제 승강 GUI/headless 저장 상태 정확 일치, 회귀24개 통과.
이 native 검사 속도는 headless0.430배/GUI0.149배이며 GPU 성능 검증이 아닙니다.
전체 결과 및 영상: /root/docker_share/mujoko_debugging_data/20261022_062713_isaac_real_lift/

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
