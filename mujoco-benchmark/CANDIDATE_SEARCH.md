# MuJoCo 다양한 경로 데이터 수집

기존 Isaac의 `trajectory_search` Sobol 6변수와 `dataset_motion.plan` IK·FCL 자기충돌 검사를 사용한다. Isaac 앱은 실행하지 않는다. 실제 MuJoCo 초기 목표 중심/neck/반지름과 로봇 초기 관절값을 계획기에 넣고 고리 기준점 FK를 비교한다. 기존 exported FCL 모델의 로봇 충돌 구조를 사용하므로 다른 로봇으로 교체하면 계획용 충돌 모델도 다시 추출해야 한다.

## 실행

```bash
cd /root/farmily_tomato
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/candidate_experiment.py \
  --candidates 8 --workers 4 --planning-workers 4
```

총 8개의 **서로 다른 경로**를 생성한다. 최대 4개의 CPU 물리 작업자가 빈 순서대로 후보를 맡는다. 모든 후보는 동일 초기 상태로 reset한다. 계획은 별도 CPU 프로세스에서 수행하며 물리 실행 전에 완료한다. 기존 CPU 벤치마크의 동일 경로 반복과 다르다.

기본 계획 런타임은 `/root/isaaclab_env/bin/python`(torch/FCL 설치 환경), 물리는 `.venv` MuJoCo이다. `--planning-python`과 `--planning-model`로 변경할 수 있다. 기본 계획 모델은 `20260922_054612_gpu_env8_matched12` 실행에서 추출한 로봇 모델이다. 지정한 pickle은 신뢰하는 로컬 파일만 사용한다.

## 저장 및 확인

- `index.html`: 후보 결과, 계획·결과 JSON 링크, 개별 재연 명령.
- `candidates.json`: seed와 Sobol 후보 변수. RL·인지·장면 랜덤화 없음.
- `manifest.json`, `planning_inputs.json`: 버전·Hz·작업자 수·입력·총 시간·하드웨어·파일 해시.
- `replay_assets`: 실제 MJB, 초기 trace, reference, 계획 모델 및 코드 사본.
- `candidates/candidate_XXXXX/plan.json`: 경유점/각 단계 길이/계획 시간/IK·충돌 검사 결과.
- `trace.json`: 해당 후보의 실제 관절 명령(60Hz).
- `states.npz`: 매 physics step qpos/식물·고리 pose/시각. 후보마다 저장하므로 디스크 사용량은 기존 타이밍 벤치마크보다 크다.
- `contacts.json`: 스텝별 고리 접촉 후보의 geom 이름 쌍. 비활성 접촉 후보도 포함하며 접촉력 데이터는 아님.
- `result.json`, `results.json`, `results.csv`: 물리 시간·진단·부분 진입·최대 열매 중심 변위·최초 접촉 후보와 원본 prim 경로.

## 재연

```bash
DISPLAY=:0 mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/replay_candidate.py \
  /결과/폴더 --candidate candidate_00000
```

모델 및 상태 해시/버전을 확인한 뒤 원본 저장 상태를 약 30fps로 표시한다. 물리 계산을 다시 실행하지 않는다. `--check-only`는 파일 검증만 한다. 영상 MP4를 자동 생성하는 기능은 아직 연결하지 않았다.

## 결과 해석

- `ik_or_planning_failure`: 물리 실행 전에 IK/관절 제한/속도/자기충돌 검사 탈락.
- `invalid_physics`: 비정상 상태 또는 MuJoCo 경고.
- `excessive_displacement`: 초기 대비 목표 열매 중심 최대 변위 20mm 초과.
- `partial_center_entry`: 처음에는 고리 중심 영역 밖에 있다가 열매 중심이 고리 내부 판정 영역에 진입한 이력. 안전한 꼭지 걸림 성공 판정이 아니다.
- `miss`: 위 조건에 해당하지 않음.

기본 목표는 Tomato_05, 최적화 식물과 원본 STL 시각 형상/기존 분할 충돌체, 파단 비활성이다. 힘 기반 비목표 접촉 위험도, 유지 시간, 정확한 꼭지 걸림 성공 판정은 아직 미구현이다. 결과의 `hook_success`는 null로 둔다. 이 데이터를 곧바로 성공/실패 학습 정답으로 사용하지 않는다. 먼저 후보 재연을 통해 물리와 판정을 검토한다.

## 소규모 검증 (2026-09-22)

`outputs/20260922_candidate_search_validated`: 2후보/2물리 작업자/2계획 작업자, 전체 24.43초.
후보0은 42.07초 동작, 목표 중심 최대 변위27.15mm; 후보1은 34.17초 동작, 최대32.96mm. 둘 다 `excessive_displacement`, 물리 비정상0.
두 관절 명령 trace의 해시가 다름을 확인했다. 후보0의 5049프레임 상태 파일 검증 통과. 이 소규모 처리량을48작업자로 선형 환산하지 않는다.
첫 4후보 실행은 물리 후 결과 가공의 읽기 전용 배열 오류로 중단되었으며, 수정 후 위2후보를 재실행했다. 원본 실패 폴더는 `20260922_candidate_search_smoke`로 보존한다.

## 대화형 HTML 보고서 (2026-09-22 갱신)

`candidate_report.py`와 `report_assets/candidate.{html,css,js}`가 외부 의존성 없이 실행되는 단일 HTML을 생성한다. 실험 종료 후에도 같은 결과를 다시 렌더링할 수 있다.

```bash
python3 mujoco-benchmark/scripts/candidate_report.py \
  mujoco-benchmark/outputs/20260922_093539_candidate_search
```

요약 카드, 6변수 선택형 산점도, 결과 분포 필터, 후보 번호 검색, 진입 이력 필터, 정렬 및20개 단위 페이지, 조건별 CSV 다운로드를 제공한다. 후보 선택 시 상세 패널에서 파라미터·계획 경유점 X–Z 투영·최초 접촉 후보·시간·원본 링크와 재연 명령을 확인한다. 재연 명령 복사 및 쉘 스크립트 다운로드도 지원한다. HTML에서 임의 명령을 직접 실행하지 않는다.

기본 정렬은 부분 진입 후보를 먼저 보여주고 그 안에서 최대 변위가 작은 순서다. 이는 검토 순서이며 성공 확률 순위가 아니다. 진입 이력과 최종 판정은 분리한다. 실측 1000후보는 부분 진입36, 과도 변위908, 미진입56이며 중심 진입 이력200개 중164개는 과도 변위다. 총 시간은 manifest의543.02초(9분3초)다.

기존 결과 데이터는 변경하지 않고 index.html만 재생성했다. Chromium에서 1440px/390px 화면, JS 오류 없음,36개 필터,164개 복합 필터, 후보49 검색, 페이지 이동, CSV 다운로드, 상세·재연 명령 표시를 확인했다. 실험 실행기도 새 생성기를 사용하며 최종 manifest 저장 후 보고서를 한 번 더 갱신한다.

## 저장 용량 및 자유 카메라 개선

새 실험은 물리·판정은120Hz 그대로 유지하고, states.npz에30Hz qpos와 시간을 저장한다. poses는 판정 계산에 사용한 후 디스크에 저장하지 않는다. 마지막 프레임은 반드시 남긴다. 기존 결과의 전체 상태 파일은 변경하지 않았다.
샘플 candidate_00087 상태 파일은25,167,833바이트에서1,956,039바이트로 감소했다. 이 비율은 후보에 따라 달라진다.
재연기는 실제 저장 시각을 사용하므로 기존120Hz와 새30Hz 파일 모두 같은 길이로 재생된다. 모든 물리 스텝을 사후 복구하는 것은 아니다.
GUI 기본 카메라는 Tomato_05를 중심으로 자유 카메라(거리0.32m, 방위각-45도)로 시작한다. 마우스 왼쪽 드래그 회전, 오른쪽 드래그 이동, 휠 확대/축소가 가능하다. 반복마다 카메라를 강제로 원위치하지 않는다.

## 기본 결과 저장 위치

새 경로 실험은 기본적으로 `/root/docker_share/mujoko_debugging_data/YYYYMMDD_HHMMSS_candidate_search/`에 저장한다. HTML, CSV, 후보별 결과와 재연 데이터 모두 같은 실행 폴더에 모인다. `--output`을 지정하면 그 경로를 우선 사용한다. 기존 결과는 이동하지 않는다.
