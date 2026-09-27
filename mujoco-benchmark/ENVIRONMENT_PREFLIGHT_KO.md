# 환경 충돌 사전검사: 첫 확인 방법

## 이번 변경의 범위

기준은 `mjlab-performance`의 `da04a0d`이며, 작업 브랜치는
`codex/mujoco-environment-preflight`다. 기존 직접 예측각, 궤적, 물리 설정,
중심 진입 판정은 변경하지 않는다. 기본 실행에서는 새 검사가 꺼져 있다.

이번 기능은 **기존 경로를 실행하기 전에 어디에서 환경과 접촉하는지 검사하는 것**이다.
우회 경로 생성, 실행 중 감속/중단, 꼭지 걸림 판정, 실물 안전 검증은 아직 아니다.

## 권장 첫 시험: 기존 결과를 수정하지 않고 검사

기존 `test_direct_angle.py` 실행의 대상별 `physics` 폴더를 사용한다.
훈련 결과 폴더나 장면 생성 폴더가 아니라 다음 파일이 있는 폴더다.

```text
manifest.json
replay_assets/model.mjb
replay_assets/reference.json
replay_assets/initial_trace.json
candidates/predicted_00000/plan.json
candidates/predicted_00000/trace.json
```

기존 모델 바이너리·실행 기록은 Git에 없을 수 있으므로 해당 자료가 있는 사용자 서버에서 실행한다.
아래 RUN만 실제 저장된 대상별 physics 경로로 바꾼다.

```bash
cd /root/farmily_tomato
RUN="/실제/실험폴더/targets/Tomato_01/physics"
OUT="/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_environment_audit"

DISPLAY=:0 ./mujoco-benchmark/.venv/bin/python \
  mujoco-benchmark/scripts/audit_environment_preflight.py "$RUN" \
  --candidate predicted_00000 \
  --policy mujoco-benchmark/config/environment_preflight.example.json \
  --output "$OUT" --gui
```

결과 JSON/HTML은 새 OUT에만 저장한다. 원래 실행 결과·상태 파일은 수정하지 않는다.
종료 코드 0은 검사한 샘플에서 통과, 2는 접촉/거리 위반, 3은 검사 불충분이다.
따라서 코드 2/3이 반환되는 것 자체가 프로그램 실행 오류라는 뜻은 아니다.

`--gui`는 처음 위반한 **계획상 관절 자세**에서 멈춘 화면을 보여준다.
빨간 점은 거리 조회가 반환한 두 점이다. 마우스로 카메라를 회전할 수 있다.
이 화면은 실제 물리 실행 영상이나 실제로 발생한 관통의 증거가 아니다.
위반 샘플이 없으면 표시할 실패 자세도 없으므로 뷰어를 띄우지 않는다.
이번 변경에는 MP4 저장 기능을 추가하지 않았다.

## 결과 해석

- `blocked`: 검사한 명령 자세에서 설정 거리 이하인 물체 쌍을 찾음.
- `sampled_clear`: 지정된 이산 샘플 검사는 통과. 연속 경로 전체의 무충돌 보증은 아님.
- `inconclusive`: 시간/계산 예산 초과, 지원하지 않는 모델, 입력 문제 등으로 결론을 내리지 못함.

`first_violation`에서 시각, 동작 단계, 로봇 geom, 환경 geom, 거리,
명령 관절값을 확인한다. `inventory`에서 실제 검사 대상 목록도 확인한다.

예제의 `clearance_m=0`은 우선 접촉/겹침을 확인하는 진단 설정이다.
센서·추종 오차나 정지거리를 반영한 안전 여유값이 아니다.

목표 꼭지 접촉까지 자동 허용하지 않는다. 따라서 의도된 걸림이나
초기 로봇 베이스–바닥 접촉도 거절할 수 있다. **거절된 실제 물체 쌍과
단계를 확인한 뒤** 허용 정책을 정해야 한다. 성공률을 높이려고 전체 목표
송이나 모든 접촉을 허용하면 안 된다. 허용 정책은 실제 물리 충돌을 끄지 않는다.

## 새 실행에 선택적으로 연결

먼저 읽기 전용 검사로 결과를 확인한 뒤 사용한다.

```bash
export FARMILY_ENV_PREFLIGHT_POLICY="$PWD/mujoco-benchmark/config/environment_preflight.example.json"
# 기존 test_direct_angle.py 실행 명령을 새 --output 경로로 실행
# 이후 새 실험에서 끄려면:
# unset FARMILY_ENV_PREFLIGHT_POLICY
```

검사 결과가 blocked/inconclusive면 물리는 실행하지 않는다.
`physics_executed=false`, `training_eligible=false`로 남기며 실제 실패 라벨로 바꾸지 않는다.
통과한 경우에만 기존 명령을 그대로 실행하고 기존 결과 판정을 유지한다.
이미 정책이 기록된 run은 환경변수를 지웠다고 검사가 꺼지지 않는다.
저장 정책과 다른 정책을 주면 오류로 중단한다.

대상별 리포트에 `environment_preflight_index.html` 링크를 추가했다.
여기서는 검사한 후보 수, 보류 수, 실제 물리 실행 수를 구분한다.
기존 최상위 직접각도 보고서는 아직 모든 판정을 '시도'로 부를 수 있으므로
새 보조 보고서의 물리 실행 수를 사용한다.

## 검증 범위

구현 커밋 `125b6c5`의 GitHub Actions 첫 실행에서 Python 3.11.16,
MuJoCo 3.14.0, NumPy 2.4.6, SciPy 1.17.1으로 **32개 테스트 모두 통과**했다.
실행: https://github.com/hjpark-rdv/tomato_robot_sim/actions/runs/36310197985

여기에는 작은 구/상자 모델의 실제 MuJoCo 거리 조회 2개 테스트가 포함된다.
기존 온실 모델 전체, 기존 물리 실행 기록, 그래픽 뷰어, 실물 로봇은
이 CI 시험에서 검증하지 않았다. 후속 CI는 3.13.0/3.14.0을 각각 검사한다.
현재 커밋별 결과는 GitHub Actions에서 따로 확인한다.

```bash
./mujoco-benchmark/.venv/bin/python -m pytest -v -rs \
  mujoco-benchmark/tests/test_environment_preflight.py
```

## 남아 있는 한계

현재 검사는 SIM GT와 초기 고정 환경을 사용한다. 움직이는 식물,
관절 추종 오차, 샘플 사이 충돌, 시각 전용 물체까지 보장하지 않는다.
로봇 자기충돌은 기존 계획기에 맡기며, 고정 환경끼리의 초기 겹침도
이 검사 범위가 아니다. 충돌체가 없는 주변 잎/송이는 여전히 미검사다.

따라서 다음 사용자 환경 검증은 기존 경로 몇 개만 대상으로 한다.
명백히 비접촉인 경로, 거터를 가로지르는 경로, 고리는 통과하지만
팔꿈치가 줄기에 닿는 경로, 상승 단계의 접촉을 구분해 확인한다.
그 결과를 보기 전에는 대규모 데이터 생성이나 실제 로봇에 적용하지 않는다.

상세 정책과 제한은 `ENVIRONMENT_PREFLIGHT.md`를 참고한다.
