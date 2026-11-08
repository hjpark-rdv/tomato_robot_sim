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

2026-10-22: 로봇 장착 카메라 PIP 자료
실행 폴더에 rgbd/frames.jsonl과 camera.json이 있으면 register_run.py가
RGB/GT 표시/깊이 색상 PNG, camera.json, objects.json을 웹 bundle 안에 복사합니다.
replay.json의 optional camera 항목과 camera/frames.json의 실제 timestamp로
3D 저장 상태와 카메라 화면을 함께 재생합니다. 과거 카메라 없는 기록도 유지합니다.
깊이·마스크의 수치 원본은 실행 디렉터리에 보존됩니다(2026-10-23부터 무손실 .npy.gz).
브라우저 표시용 복사본은 PNG/JSON만 포함합니다.
오른쪽 PIP에서 RGB / GT / Depth를 고르고 접기 버튼으로 숨길 수 있습니다.
PIP는 현재 재생 시각 이전의 가장 가까운 실제 촬영 프레임만 표시합니다.
시험 묶음을 선택하면 그 아래 구간 목록에서 이동·수확·중단·복구 기록을 고를 수 있고,
자동 다음 구간 옵션으로 실제 기록 순서대로 재생할 수 있습니다.

별도 저장 상태 렌더 결과를 연결할 때는 기존 명령에 --sensor-data /절대경로/rgbd를 추가합니다.
이 경우 해당 디렉터리의 provenance.json에 다음을 반드시 기록합니다.
{"mode":"measured_state_render","states_sha256":"states.npz의 SHA256",
 "scene_sha256":"실제 composed.usdc의 SHA256"}
이 자료는 저장 상태에서 후처리한 센서 렌더이며 동시 실시간 취득으로 표시하지 않습니다.
모든 camera frame은 원본 states의 정확한 timestamp, robot_q, 실제 카메라 장착 body pose와
일치해야 합니다. 잘못된 다른 실행의 센서 자료, 변경된 PNG, 외부 symlink는 거부합니다.
sensor_replay.py가 검사/복사를 담당하며 validate_replay.py도 같은 원자료와 독립 대조합니다.

카메라 exporter 회귀 검사:
cd /root/farmily_tomato/gripper-design/harvest-replay
../../nvidia-sim/.asset-venv/bin/python -m unittest -v test_sensor_replay.py

연속 시험 이력을 묶어서 등록:
각 실행을 register_run.py로 먼저 등록하고 아래처럼 실제 실행 순서대로 ID를 지정합니다.
./nvidia-sim/.asset-venv/bin/python gripper-design/harvest-replay/register_collection.py \
  --id 새_시험_ID --label '6송이 연속 시험' --runs 실행_ID_1 실행_ID_2 실행_ID_3
catalog.collections의 runs는 catalog.runs에 이미 있는 실행 ID의 순서 목록입니다.
각 segment는 자체 원본 scene·states·camera timestamp를 그대로 유지합니다.
서로 다른 물리 환경의 바디를 하나의 가상 states로 연결하지 않습니다.
같은 시험 ID 덮어쓰기, 없는 실행 ID, 중복 segment와 외부 symlink는 거부합니다.
--continuity measured_state_handoff|independent_trials|unknown으로 실제 연결 관계를 표시합니다.
기본 unknown은 초기화나 연속 물리 중 어느 쪽도 단정하지 않습니다.
최종 확인된 대표 시험에만 --make-default를 지정하면 같은 catalog 갱신에서
default_collection과 첫 segment인 default_run을 함께 설정합니다. 진단 시험은
이 옵션 없이 추가하며 기존 기본 선택을 유지합니다. 명시적인 URL 선택은 우선합니다.
--report 결과.json --diagnostics 실패.log 검사.json으로 지정한 실제 파일은
assets/<시험_ID>/ 안에 복사하며 report_url/diagnostics_urls와 SHA256을 목록에 기록합니다.
physics 이전 실패처럼 states가 없는 시도는 로그만 제공하고 가상 3D 재생을 만들지 않습니다.

카메라 기반 영역 선택 시험 (2026-10-22)
기존 run_row_harvest.py의 --camera-capture와 run_motion.py의 --camera-harvest를
함께 사용합니다. 별도 물리 실행기를 만들지 않습니다. mounted_rgbd가 있는 모델에서
--whole-truss --truss-order highest-first --single-fruit-when-remaining 0을 요구합니다.
camera_region_policy.py는 실제 촬영된 objects.json의 보이는 빨간 열매를 기존 익음 정책,
현재 식물, 레일 가까운 쪽 송이, 미분리·시도 예산과 교집합합니다. 기본 최소2개입니다.
진입 방향은 보이는 중심가지 마스크의 깊이 점군으로 구하며, 그 방향이 불확실하면
보이는 열매 추정 중심들의 PCA를 사용합니다. 보이지 않는 목표 중심은 사용하지 않습니다.
축 부호는 카메라→관측 중심 방향을 향하게 하고, 수직에서15도 안의 가지는 관측 중심의
수평 방향을 사용합니다. --camera-heading-offsets-deg의 기본0,15,-15,30,-30도 후보마다
보이는 열매의 footprint와 중앙 보정을 다시 구한 뒤 기존 IK/전체 환경 검사를 수행합니다.
각 방향의 계획 거절 이력은 독립적이며 실제 동작을 시작하기 전 거절은 수확으로 세지 않습니다.
열매 중심은 보이는 표면 중앙값+알려진 모델 반경의 근사치이며 실물 인식 모델이 아닙니다.
기존 IK/전체 자가충돌 검사/물리 접촉 감시는 그대로 실행합니다. 계획 거절은 수확 시도와
별도로 기록합니다. camera_scan_prefix는 수확으로 세지 않습니다.
정차 후 실제 base 관절을 움직여 카메라를 지도상의 촬영 위치로 향하게 하는 스캔 pan도
같은 경로 검사·구동을 거칩니다. 지도는 촬영 방향 준비에만 쓰고, 진입 목표는 도착 후
실제로 촬영한 관측에서 다시 정합니다. 카메라의 고정 장착 transform은 바꾸지 않습니다.
렌더 카메라 행렬이 지연되면 같은 물리 시각에서 RGB·깊이·카메라 자료를 최대3회 다시
렌더합니다. 기존 좌표 오차 기준은 유지하며 물리 시간이 바뀌면 즉시 중단합니다.
RGB·깊이·카메라 파라미터·renderer instance ID/label 대응을 같은 촬영 묶음에서
CPU 소유 복사본으로 읽습니다. renderer가 재사용하는 텍스처를 파일 저장 뒤에 늦게
읽지 않습니다. 잘못된 ID/형태/자료형은 해당 원본·label·오류를 보존하고 같은 물리
시각에서만 최대3회 다시 촬영합니다. 알 수 없는 픽셀을 배경으로 바꾸지 않습니다.
객체 catalog의 실제 매핑 오류와 물리 시간 변화는 이 재시도로 우회하지 않습니다.

현재 joint/ready-opening 카메라 시험은 기존 준비·개방·레일·스캔 명령만 먼저 만듭니다.
실제 카메라 촬영 뒤에 선택된 수확 영역의 준비/삽입 IK와 전체 환경 검사를 수행합니다.
촬영에 쓰지 않는 가상의 수확 준비 IK 때문에 스캔 자체가 차단되지 않도록 분리했으며,
카메라를 쓰지 않는 기존 동작 계획과 실제 수확 명령·물성·접촉 기준은 유지합니다.
행 실행기의 --allow-camera-scan-only-proposals는 명시적인 카메라 시험에서만 쓸 수
있습니다. 과거 지도 수확 경로가 막혔어도 그 정차점을 새 촬영 제안으로 사용할 수
있으며, 원래 차단 상태와 원본 해시는 그대로 기록합니다. 그 옛 경로를 실행하거나
통과한 것으로 바꾸지 않습니다. 새 장면 초기 검사, 실제 촬영 준비 경로, 촬영 뒤의
수확 경로에 대한 기존 자가충돌·전체 환경·실제 접촉 검사는 각각 그대로 수행합니다.

수락된 실행 영역은 원본 마스크의 현재 보이는 픽셀 안쪽에 얇은 분홍 테두리로 표시합니다.
harvest_region.png/analysis_region.png/region.json은 파생 자료이며 원본 RGB·깊이·마스크는
그대로 남습니다. 웹 PIP의 GT 모드와 실제 시각으로 만든 분석 MP4에서 확인할 수 있습니다.
camera_campaign_report.py DATA_ROOT는 기존 원자료를 읽어 스캔/계획/실제 접근/벨트 구간/
완주/새 조인트 분리를 구분한 JSON·HTML 보고서를 만듭니다. 접촉·분리를 실제 수확·포집
성공으로 승격하지 않습니다. 연속 window에서는 이전 로봇 상태와 분리 과실을 넘기고,
지나간 부착 식물은 측정 변형 상태로 동결하는 기존 근사를 명시합니다.
정차 순서는 사전 지도상의 시험 제안이며 최적 수확 순서를 학습한 결과가 아닙니다.
시험당 집단 진입1회와 닫기·열기2회를 완료해도 남은 익은 열매가 있을 수 있습니다.
새 조인트 분리, 송이 전체 완료, 포집, 실물 수확 성공을 서로 구분해서 읽어야 합니다.

소프트웨어 예외로 남은 실제 부분 기록
export_replay.py/register_run.py/validate_replay.py의 --allow-partial-diagnostic은
states_partial.npz + partial_trace.json + runtime_error.json + 실제 native 메타데이터가
정확히 결합된 경우에만 불완전한 기록을 표시합니다. 기본 실행은 이 모드를 사용하지 않습니다.
기존 장면/스키닝/측정 바디/시각 검사를 재사용하며 summary.json이나 정상 checkpoint를
만들지 않습니다. 원본 부분 배열·모델·manifest·오류의 해시와 모든 바디 경로를 검증합니다.
record_kind=partial_diagnostic, diagnostic_only=true, checkpoint_eligible=false,
harvest_credit=0, training_eligible=false를 유지합니다. 최종 카메라 메타데이터가 없으면
다른 관측을 끼워 넣지 않고 카메라 없음으로 표시합니다. 일부 기록의 끝에서는 자동으로
다음 구간을 재생하지 않습니다. 목록에 추가할 때도 명시적인 진단 허용 옵션이 필요하며,
독립 진단 collection에만 넣을 수 있고 기본 대표 시험으로 지정할 수 없습니다.
실패 말단의 속도/재개 상태가 없으면 이후 정상 스냅샷 재시험은 독립 분기로 표시합니다.
collection.provenance.continuity_note와 history_graph가 이 관계를 설명하며,
실패 말단에서 계속된 동작이나 중단 없는 연속 물리 실행으로 해석하지 않습니다.

2026-10-23: 카메라·웹 기록 저장 최적화

새 카메라 기록은 float32 깊이/uint32 객체 ID/uint8 semantic 및 renderer ID를
NPY 원래 바이트 그대로 gzip(level 1) 저장합니다. NaN payload·dtype·형상도 유지합니다.
프레임별 파일 쓰기만 최대4개 I/O 스레드로 묶고, 완료 후 camera.json/frames.jsonl을
기록합니다. 렌더링/물리 적분·제어·촬영 시각을 스레드에서 실행하지 않습니다.
renderer_labels.json은 같은 RGB-D 디렉터리의 shared_labels/<SHA>.json을 상대 링크로
참조합니다. 동일 사전만 공유하며 객체 매핑이 바뀌면 별도 사전을 남깁니다.
RGB/GT/Depth/분석 PNG와 기존 MP4는 화질 변경 없이 유지합니다.

새 웹 export는 scene.glb.gz만 보관합니다. gzip 해제 SHA가 기존 GLB SHA와 같음을
확인하며 validate_replay.py는 예전 raw GLB와 새 gzip-only 형식을 모두 읽습니다.
동일한 웹 모델·카메라 자산은 assets/_shared 안의 내용 해시 파일과 hardlink로 공유합니다.
다른 모델을 억지로 공통 모델로 합치지 않으며 원본 NAS 자료와 웹 파일은 hardlink하지 않습니다.
공유 자산은 불변 파일입니다. 변경하려면 새 파일을 만든 뒤 atomic replace하십시오.
NAS RGB-D를 옮길 때는 shared_labels를 포함한 rgbd 전체를 함께 복사하십시오.

기존 결과 압축(기본은 dry-run, 중지된 완료 기록에서만 --apply):
PY=/root/farmily_tomato/nvidia-sim/.asset-venv/bin/python
$PY nvidia-sim/harvest_s300/compact_camera_storage.py DATA_ROOT --report NEW_AUDIT.json
$PY nvidia-sim/harvest_s300/compact_camera_storage.py DATA_ROOT --apply --report NEW_APPLY_AUDIT.json
$PY nvidia-sim/harvest_s300/compact_camera_storage.py gripper-design/harvest-replay/assets --web --apply --report NEW_WEB_AUDIT.json

기존 원본 frames.jsonl/camera.json/objects.json 및 물리 상태·결과 SHA는 변경하지 않습니다.
따라서 옛 로그의 논리적 depth.npy 경로는 실제 depth.npy.gz에 대응할 수 있습니다.
이관한 프레임의 storage.json에 원본 NPY SHA·압축 파일 SHA·크기를 남깁니다.
내부 판독기는 recording_storage.load_array(path)로 두 형식을 모두 지원합니다.
외부 프로그램에서 np.load(path)만 직접 호출하던 경우에는 다음처럼 읽으십시오:
  import gzip, numpy as np
  with gzip.open('depth.npy.gz', 'rb') as stream:
      depth = np.load(stream, allow_pickle=False)
또는 gzip -dk depth.npy.gz 로 이전 NPY 바이트를 정확하게 복원할 수 있습니다.
전체 이관 배열 복원은 같은 도구의 --restore-arrays --apply를 사용합니다.
압축 모델도 gzip -dk scene.glb.gz로 원래 GLB를 복원할 수 있습니다.

실제 이관 감사·테스트·성능 측정:
/root/docker_share/mujoko_debugging_data/20261022_storage_compaction/
초기화/물리 성능이나 수확 성공률을 변경한 시험이 아닙니다.

2026-10-23: 전체 기록 탐색과 정면 스캔

테스트 묶음을 선택하면 아래에 '전체 기록' 막대가 표시됩니다. 기존 '현재 구간'
막대와 별개로, 여러 저장 구간의 합산 시간에서 원하는 위치로 직접 이동합니다.
camera_region_route_20261022는 14구간, 총499.5초입니다. 이는 독립 재시험을
포함한 재생 목록의 합계이며, 하나의 중단 없는 물리 실행 시간으로 주장하지 않습니다.
구간 간 상태를 보간하거나 새 동작을 만들어내지 않습니다. 독립 분기의 자동 정지
규칙은 유지하고, 사용자가 그 구간을 직접 선택하면 독립 재시험임을 표시합니다.
HARVEST_REPLAY_API.seekCollection(seconds)도 같은 실제 구간 선택 동작을 사용합니다.

새 run_motion.py --camera-harvest의 기본 --camera-scan-view는 row-front입니다.
고정 장착 카메라의 실제 base yaw를 레일에 수직인 농장 쪽으로 돌립니다.
--camera-scan-view target-point는 이전의 특정 송이 지점을 향하는 사선 스캔입니다.
이미 저장한 영상·자세·카메라 행렬은 변경하지 않습니다. 수확 중에는 link1에
고정된 카메라가 로봇 몸통 회전과 함께 움직이며 독립 짐벌처럼 고정하지 않습니다.
정면 촬영 정차점과 팔의 수확 정차점은 같은 의미가 아니며, 새 자세/경로 검사가
필요합니다. station별 1회만 실행한 과거 시험은 --camera-max-harvest-cycles 1의
명시적 예산 때문입니다. 0은 기존 반복 선택 방식, 양수는 실제 수확 사이클 예산이며,
--max-truss-cycles는 거절된 계획까지 포함하는 별도 유한 탐색 예산입니다.

새 정면 촬영 실제 기록:
http://lab.farmily.ai:8090/gripper-design/harvest-replay/?run=camera_front_scan_20261023
원본/검사: /root/docker_share/mujoko_debugging_data/20261023_front_camera_review/

2026-10-23: 측정된 그리퍼 추종 초과의 표시 진단

export_replay.py --allow-measured-linkage-diagnostic은 실제 저장된 간격이
기존 CAD 명목 범위27.9..96.1mm를 잠깐 벗어난 중단 기록의 검토 옵션입니다.
25/45mm 연결봉의 실제 삼각형이 닫히는 범위에서 같은 CAD 식을 적용하고,
모든 초과 프레임·실제 최솟값/최댓값을 transmission 메타데이터에 남깁니다.
원본 robot_q/바디 자세/간격을 보정하거나 자르지 않습니다. 물리·명령 범위도
바꾸지 않으며 기존 계획·실행의 기본 transmission_transforms 검사는 유지됩니다.
validate_replay.py는 원본 간격과 초과 프레임 전체를 대조합니다. 이 옵션을
사용한 표시는 실제 하드웨어의 초과 구동이 안전하거나 유효하다는 의미가 아닙니다.
