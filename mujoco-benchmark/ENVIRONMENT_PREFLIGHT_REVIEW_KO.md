# 환경 충돌 사전검사: 최종 검토 브랜치

작성: 2026-09-27. 검토 브랜치: `codex/environment-preflight-20260927`.
원본 기준: `mjlab-performance`의 `da04a0dbb16a2b6c661beb8f5ffff1bac355689b`.

## 브랜치 관계

기존 `codex/mujoco-environment-preflight`의 사전검사 구현 및 보강 커밋
`28d3e315d2a8264aff4ed5063968d885e6cbb49d`를 그대로 포함한다.
이 검토 브랜치에는 별도의 두 번째 사전검사기를 만들지 않았다.
추가 변경은 실제 MuJoCo로 실행하는 작은 형상 테스트 11개와 CI 설정,
이 검증 기록이다. `mjlab-performance`에는 병합하지 않았다.

환경변수, 스크립트, 정책 파일은 기존 구현과 동일하다.
- 설정: `FARMILY_ENV_PREFLIGHT_POLICY`
- 검사: `scripts/environment_preflight.py`
- 읽기 전용 검사/정적 자세 뷰어: `scripts/audit_environment_preflight.py`
- 설명: `ENVIRONMENT_PREFLIGHT.md`, `ENVIRONMENT_PREFLIGHT_KO.md`

## 구현 범위

기존 RGB-D 예측각과 경로를 변경하지 않고, MuJoCo 실행 전에 같은 모델의
활성 로봇/환경 충돌 형상 사이 거리를 검사한다. 기존 IK/자기충돌 검사는 유지한다.
검사는 명시적으로 활성화할 때만 실행한다.

- `sampled_clear`: 검사한 명령 자세들에서는 거리 조건 위반이 없었다.
- `blocked`: 검사한 자세에서 금지된 거리 조건 위반이 발견됐다.
- `inconclusive`: 지원 범위, 입력, 시간/샘플/조회 예산 등의 문제로 완료하지 못했다.

`blocked`와 `inconclusive`는 물리 실행을 하지 않고 별도로 기록한다.
실제 실행 실패나 수확 불가능 라벨로 사용하면 안 된다.
현재 중심 진입 판정, `hook_success=None`, actuator/물리 설정은 변경하지 않았다.

## 실제 확인한 자동 테스트

테스트한 코드/CI 커밋: `c0a703646627c9192ef3c52fd64eaf4da3836804`.
실행 기록: https://github.com/hjpark-rdv/tomato_robot_sim/actions/runs/36310992839

| CPU 환경 | 결과 |
|---|---|
| Python 3.11 / MuJoCo 3.13.0 | 46개 통과, skip 없음 |
| Python 3.11 / MuJoCo 3.14.0 | 46개 통과, skip 없음 |

기존 35개 검사에 실제 MuJoCo 형상 테스트 11개를 추가했다.
추가 검사는 팔꿈치만 충돌하는 경로, 원래 contact mask와 별개인 거리 조회,
명시 contact pair의 0-mask 형상 포함, 초기 겹침, 단계별 허용 접촉,
환경 충돌체가 없는 경우의 판단 유보, 엔진 상태 불변성, 고리 내부 공간 유지,
평면 거리 여유, 회전 도중의 충돌, convex mesh/box 거리, 샘플 예산을 확인한다.

이것은 생성한 작은 모델의 기하 검사다. 사용자의 실제 고리 CAD와 온실 모델,
동적 접촉/걸림 성공, 전체 경로 처리속도, GUI 표시를 검증한 것은 아니다.

## 서버에서 첫 확인

기존 작업 파일을 강제로 초기화하지 않는다. 미커밋 변경을 보존한 뒤 브랜치를 받는다.

```bash
cd /root/farmily_tomato
git status --short
git fetch https://github.com/hjpark-rdv/tomato_robot_sim.git codex/environment-preflight-20260927
git switch -c review/environment-preflight-20260927 FETCH_HEAD
```

기존 실행 결과에 대한 읽기 전용 검사가 먼저다. `RUN`은 실제 대상별 physics 폴더로
바꾼다. `manifest.json`, `replay_assets`, `candidates/predicted_00000/plan.json` 및
`trace.json`이 있어야 한다. output은 기존 결과 밖의 새 폴더여야 한다.

```bash
RUN="/실제/기존결과/targets/Tomato_01/physics"
OUT="/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_environment_audit"
DISPLAY=:0 ./mujoco-benchmark/.venv/bin/python \
  mujoco-benchmark/scripts/audit_environment_preflight.py "$RUN" \
  --candidate predicted_00000 \
  --policy mujoco-benchmark/config/environment_preflight.example.json \
  --output "$OUT" --gui
```

원래 물리 결과와 상태를 덮어쓰지 않는다. `--gui`는 처음 거절된 **계획 자세**를
보여준다. 새로운 물리 실행 영상이 아니다. 위반 자세가 없으면 뷰어를 띄우지 않는다.
HTML/JSON에는 거리 조건, 대상 쌍, 단계, 시점, 관절 명령, 검사 목록/예산 등이 나온다.
종료코드 0/2/3은 각각 검사 통과/거절/판단 유보다. 2 또는 3이라고 무조건 프로그램
오류라고 해석하지 않는다.

## 아직 남은 한계

- 초기 식물을 고정한 이산 명령 자세 검사다. 검사 지점 사이 충돌, 실제 추종 오차,
  식물 변형, 정지 거리까지 보증하지 않는다.
- 주변 시각 전용 잎/송이는 검사 대상이 아니다. 해당 충돌 형상 생성도 추가하지 않았다.
- 목표 꼭지 접촉은 기본 허용하지 않는다. 기본 정책이 의도된 걸림 또는 기존
  베이스-지면 접촉도 막을 수 있다. 실제 쌍/단계를 검토한 뒤 명시적으로 허용한다.
- 예제 `clearance_m=0`은 진단값이며 실물 안전 여유가 아니다.
- 아직 우회 경로 생성, Roll/Pitch 경로 최적화, RL, 고리 포획/유지 판정은 없다.
- 원본 온실 모델 및 결과는 사용자 서버에 있으므로 해당 환경에서 기존 정상 경로,
  거터 교차, 팔꿈치-줄기, 상승 중 비목표 과실 충돌을 먼저 비교해야 한다.

다음 구현은 이 비교로 실제 막히는 구간을 확인한 뒤 정한다.
검사를 통과시키려고 충돌 형상을 끄거나 허용치를 임의로 넓히지 않는다.
