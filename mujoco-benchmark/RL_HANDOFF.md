# Tomato_06 RL 작업 인수인계

작성: 2026-09-22. 코드 기준 `mjlab-performance` 브랜치의 `a085f47`. 이 문서 정리는 해당 커밋 이후 변경이다.

## 사용자 목표와 현재 범위

직선 삽입/상승 경로만으로 성공하지 못하는 경우를 해결하기 위해 RL로 자유로운 동작을 찾는다. Tomato_06에 집중하며 고리 회전과 리프트도 고정하지 않는다. 꼭지 수확까지 요구하지 않고, 토마토 중심까지 부분 진입하면서 열매 이동이 작은 동작을 우선한다.

현재는 동일 초기 장면, GT 관측, 기존 최적화 식물과 로봇팔/원본 STL 외형을 사용한다. RGB-D 정책, domain randomization, 실물 일반화 검증은 하지 않았다. STL 외형과 물리 충돌 형상은 같지 않으며 고리는 기존 분할 충돌체를 유지한다.

## 구현 파일

| 파일 | 역할 |
|---|---|
| `scripts/train_tomato_rl.py` | mjlab Simulation/MJWarp + RSL-RL PPO, 7관절 명령, 시간 제한, 전후 평가 |
| `scripts/replay_tomato_rl.py` | 저장된 독립 평가 첫 환경의 qpos 재생 |
| `TOMATO06_RL.md` | 상세 물리·보상·실행 설명 |
| `validation/tomato06_rl_smoke.json` | 짧은 검증 결과 |
| `validation/tomato06_rl_scaling.json` | 64/256환경 처리량 측정 |
| `validation/tomato06_rl_10min.json` | 완료한 10분 학습 결과 |

물리120Hz / 정책20Hz. 팔6관절+리프트의 속도 의도를 position drive 목표로 적분한다. 실험용 명령 상한은 팔0.25rad/s, 리프트0.04m/s이며 실제 하드웨어 검증값이 아니다. 기존 관절 한계 적용. RL 전용 모델에 비인접 링크 자기충돌을 추가했고 기존 데이터 수집 모델은 변경하지 않았다.

부분 진입 판정: 목표 열매 구 단면이 고리 rear 내부에 들어오고 고리 평면에서2mm 이내, 0.2초 유지. 전체 열매 최대 이동20mm 초과/로봇 관련 관통2mm 초과 시 종료. 꼭지 걸림 성공 판정과 다르다. reset, 비정상 물리 상태/overflow 검사, 체크포인트 재시작 검증 완료.

## 실제 실험 결과

결과 루트: `/root/docker_share/mujoko_debugging_data/`

| 실행 폴더 | 상태 및 결과 |
|---|---|
| `20260922_182738_tomato06_rl` | 16환경20업데이트 검증, 전체45.93초, 성공0 |
| `20260922_184128_tomato06_rl` | 사용자200업데이트, 전체321.72초, 학습208에피소드 성공0; 기준 모델 `model_00200.pt` |
| `20260922_185257_tomato06_rl_gpu256` | 1,000업데이트 계획을 사용자 요청으로 중단. `model_00025.pt` 보존, 이후 미저장 업데이트 손실 |
| `20260922_185721_tomato06_rl_10min` | 기준200회 모델에서 새로10분 학습, 정상 완료. 최종 `model_00170.pt` |

10분 실행: 학습600.43초, 전체735.75초(준비/전후 평가 포함), 170업데이트, 1,392,640전이, 약2,319전이/초. 학습2,306에피소드: 성공0, 과도한 이동34, 시간 제한2,272.

| 동일 조건 결정적 평가 | 전 | 후 |
|---|---:|---:|
| 부분 진입 성공수 | 0 | 0 |
| 평균 최근접 고리–열매 중심 거리 | 56.16mm | 43.17mm |
| 평균 최대 열매 이동량 | 1.13mm | 1.13mm |

256개 평가 환경은 동일 장면/동일 정책의 복제이며 독립적인 일반화 표본이 아니다. 성공률 개선을 달성했다고 주장하지 않는다.

64/256환경의 짧은 동일 조건 처리량: 1,150 / 2,352전이/초. 256환경에서 관측한 GPU95%, 메모리4.4GB는 순간값이며 평균/최댓값이 아니다. 메모리 여유가 추가 처리량을 보장하지 않는다.

## 정체 원인과 다음 작업

확인된 사실: 약5초에 접근 후30초까지 비슷한 위치에 머문다. 정지해도 현재 근접 항과 시간 비용의 합이 약+0.024/스텝이다(다른 감점 전). 성공 경험은0이다.

유력한 해석: 삽입을 시도하다 밀림 페널티/종료를 받는 것보다 근처에서 보상을 계속 받는 정책으로 정체될 수 있다. 2mm 평면 정렬/0.2초 유지라는 좁은 성공 조건도 탐색을 어렵게 한다. 보상 구조만이 원인이라고 입증한 것은 아니며 실제 장애물/도달성 확인이 필요하다.

다음 작업 후보:

1. 저장 평가에서 자세·장애물 간격·진입 가능성을 확인한다.
2. 정지 근접 보상 제거, 정렬/삽입 진전 중심 보상을 검토한다. 후퇴/재접근 탐색을 과도하게 막지 않도록 한다.
3. 성공 판정/물리와 같은 기준으로 기존 모델 대비10분 비교한다. 보상 합만으로 개선을 판단하지 않는다.

**아래 최신 변경에서 progress_v2 구현 완료. 새 학습은 사용자가 실행한다.** 기존 결과를 덮어쓰지 않는다.

## 재생 명령

```bash
cd /root/farmily_tomato
DISPLAY=:0 ./mujoco-benchmark/.mjlab-venv/bin/python \
  mujoco-benchmark/scripts/replay_tomato_rl.py \
  /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min
```

## 같은 기준에서 10분 비교 재실행

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.mjlab-venv/bin/python -u \
  mujoco-benchmark/scripts/train_tomato_rl.py \
  --num-envs 256 --iterations 100000 \
  --steps-per-env 32 --episode-seconds 30 \
  --train-seconds 600 --compare-before \
  --checkpoint /root/docker_share/mujoko_debugging_data/20260922_184128_tomato06_rl/model_00200.pt
```

600초는 학습 구간만 측정하며 현재 업데이트 완료까지 수 초 초과할 수 있다. 준비/평가는 별도. `iterations`는 상한이며 실제10만회 실행하지 않는다. `model_00170.pt`를 checkpoint로 주면10분 결과에서 이어 학습하지만 위의 기준 모델 비교와는 실험 조건이 다르다. checkpoint 로드는 모델/옵티마이저를 복원하며 환경/난수 상태의 완전 중단 복원은 아니다.

각 실행은 새 datetime 폴더에 `manifest.json`, 실제 `rl_model.xml/mjb`, 코드 사본, `progress.jsonl`, 체크포인트, `baseline.json`, `summary.json`, `baseline_world0.npz`, `evaluation_world0.npz`를 저장한다. 전후 수치는 `summary.json`의 `comparison`을 확인한다. 영상 파일이 아니라 재생 가능한 상태 기록이다.

대형 모델/체크포인트/상태는 Git 밖 docker_share에 있으므로 계정 또는 머신 변경 시 별도 보존한다. 이전 PID와 실행 중 표기는 역사 기록이며 재개 전 실제 프로세스 상태를 확인한다.

## 최신 변경: 보상 progress_v2 (학습 실행 안 함)

사용자 요청으로 보상만 변경. 정지 근접 보상과 반복 내부 체류 보상을 제거했다. 기존 목표점 거리 감소20배 + 고리 좌표계의 포획 조건 위반 거리 감소20배로 진전을 보상한다. 포획 오차는 반원 내부/고리 평면/와이어 여유의 위반량이며 특정 세계 방향이나 경로를 고정하지 않는다. 최초 안전 부분 진입은 에피소드당1회 +2, 기존0.2초 유지 성공 +10. 기존 밀림/접촉/급격한 액션/시간 감점과 실패 -5 유지. 후퇴는 signed progress 감점이지만 물리적으로 금지하지 않는다.

물리·관측·행동·종료·성공 기준·학습기는 변경하지 않았다. manifest의 reward_version=progress_v2로 구분한다. 정지/진전/후퇴/왕복/최초 진입/포획 오차 산술 검증 통과. 수정 보상의 실제 학습 개선 여부는 아직 미검증. 학습은 사용자가 직접 실행한다.

2시간 추가 학습(이전10분 모델에서 시작):

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.mjlab-venv/bin/python -u \
  mujoco-benchmark/scripts/train_tomato_rl.py \
  --num-envs 256 --iterations 100000 \
  --steps-per-env 32 --episode-seconds 30 \
  --train-seconds 7200 --compare-before \
  --checkpoint /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min/model_00170.pt
```

학습 구간2시간이며 업데이트 경계까지 수 초 초과 가능, 준비·전후 평가는 별도다. 새 datetime 폴더로 저장. 기존 보상과 점수 크기가 달라졌으므로 보상 합보다는 독립 평가 성공/거리/변위를 비교한다.
