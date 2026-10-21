농장 수확 재생

웹: http://lab.farmily.ai:8090/gripper-design/?tab=replay
부모 페이지의 세 번째 탭 '농장 수확 재생'에서 기록을 선택합니다.
재생/일시정지, 처음으로, 시간 슬라이더, 0.25/0.5/1/2배속,
로봇/송이/농장/위에서 시점과 마우스 회전·확대를 제공합니다.

이 화면은 저장된 실제 상태를 재생합니다. physics를 새로 실행하지 않습니다.
물체의 위치·회전, 가지 스킨 변형, 그리퍼 연결부와 분리 열매를 기록대로 표시합니다.
기록 시점 사이에는 위치/회전을 보간합니다. 조작한 카메라가 저장된 결과를 바꾸지 않습니다.
렌더링 재질·조명은 브라우저용이므로 Isaac RTX 화면과 픽셀 단위로 같지는 않습니다.

현재 등록한 실제 Isaac Sim / PhysX TGS CPU 기록:
- isaac_truss_57s: 57초, 3,421프레임/277바디, 열매 분리 이벤트5개.
- isaac_move_83s: 82.8초, 4,969프레임/136바디, 레일 이동 약1.6424m, 분리 이벤트4개.
모두 원본 실행이 완료된 사례입니다. 분리는 포집·실물 수확 성공과 구분합니다.
이는 2026-10-11의 기록이며, 당시 가상 리프트 상판을 포함한 모델 그대로 재생합니다.
최근 실제 리프트 수정 모델은 '로봇 · 리프트 · 그리퍼' 탭에서 확인합니다.
수정된 리프트로 새로 실행한 기록은 새 assembly와 states.npz를 등록하면 됩니다.

다른 실제 실행을 등록하는 명령:

cd /root/farmily_tomato
./nvidia-sim/.asset-venv/bin/python gripper-design/harvest-replay/register_run.py \
  /절대경로/실행당시_assembly \
  /절대경로/states.npz가_있는_실행폴더 \
  --id 새_영문_ID --label '화면에 표시할 실행 이름'

ID는 영문/숫자/하이픈/밑줄을 사용합니다. 같은 ID나 출력 폴더는 덮어쓰지 않습니다.
assembly에는 원본 composed.usdc, motion_manifest.json과 skin*.npz가 필요합니다.
실행 폴더에는 states.npz(time,poses,body_paths,robot_q), runtime_settings.json,
summary.json, native_metadata.json, break_events.json을 사용합니다.
원본 파일의 해시와 실제 runtime의 composed USD 해시를 확인합니다.

구성:
- export_replay.py: 기존 USD 메시/재질 판독기와 MeasuredPoseDisplay,
  transmission_transforms를 재사용합니다. 새 물리/수확 파이프라인을 만들지 않습니다.
- scene.glb(.gz): 농장 전체 시각 형상, 반복 형상 GPU instancing, 가지 glTF skin.
- poses.bin.gz: 모든 원본 프레임의 xyz+quaternion xyzw, USD Z-up world.
- times.bin: 실제 원본 timestamp float64. 일정 FPS의 가상 시각으로 교체하지 않습니다.
- transmission.bin.gz: 실제 그리퍼 간격으로 계산한 연결부 자세.
- replay.json/replay_scene.json: 데이터 형식, 모델·원본 해시, 바디/스킨 매핑과 분리 시점.
- catalog.json: 웹 선택 목록. 새 기록은 register_run.py로 추가합니다.
- validate_replay.py: 원본 NPZ/모델/분리/가지 변형과 내보낸 자료를 독립 비교합니다.

검증:
./nvidia-sim/.asset-venv/bin/python gripper-design/harvest-replay/validate_replay.py \
  gripper-design/harvest-replay/assets/isaac_truss_57s

checks/recorded_data_*.json와 validator_negative_controls.json에 원본 비교를 기록합니다.
두 예제의 모든 timestamp·pose는 추가 손실 없이 보존됐고,
농장31,538개 가시 메시 인스턴스/53,997,552개 삼각형(복제 포함)을 보존했습니다.
브라우저 좌표는 Z-up→Y-up으로 바뀌지만 물체 간 배치와 시간은 같습니다.

assets/는 대형 실행 데이터이므로 Git에서 제외합니다. 다른 서버로 옮길 때 별도 보존합니다.
통합 서버가 gripper-design 아래 파일만 제공하므로 외부 경로로 향하는 symlink는 사용하지 않습니다.
새 웹 서버를 추가하거나 기존 서버를 재시작할 필요가 없습니다.
