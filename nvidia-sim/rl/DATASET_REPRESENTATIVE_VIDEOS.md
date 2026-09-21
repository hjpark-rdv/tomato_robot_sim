# 병렬 trajectory 시험과 대표 영상

후속: 49번의 관통 확인으로 새 GPU dataset 기본값은 `contact120`이다.
[접촉 수정 및 범위](CONTACT_PENETRATION_FIX.md)를 먼저 참고한다.
아래 64환경 성능 수치는 이전 60Hz 실행 기록이며 새 설정의 속도 예측에 그대로 쓰지 않는다.

64개 GPU 환경과 CPU 경로 계산 16개 작업자를 사용한 실측은
`runs/20260921_175044_staged6d_64env_videos/`에 저장한다.
물리/동작 속도/접촉 중단 기준은 practical60 소규모 시험과 동일하다.

실측 64개 전체 실행은 **814.70초 (13분 35초)**, 후보당 12.73초다.
이전 4환경/8후보의 후보당 33.63초 대비 관측 처리량은 2.64배지만,
후보 구성과 조기 중단 비율이 다르므로 순수 최적화 speedup은 아니다.
rollout 740.32초 중 physics 481.42초, 실행/판정 192.93초였다.
64개 경로 계산은 16 workers에서 10.59초; GPU 메모리 최대 6191MiB.
CPU 경로 계산 최대 약 1892%, rollout 중앙값 117%로 직렬 병목은 남아 있다.
완전 걸림 0, miss 8, 과도 변위 54, 비목표 접촉 2. 중심 진입 부분 성공 1개다.
`performance_report.json`의 첫 8개 비교에서 관절 명령은 이전 시험과 모두 같다.

64/64 단일 묶음에서는 먼저 끝난 환경이 빈 채로 남는다. 이번 활성 env-step 비율은
0.414였다. 아래 128개 예제처럼 후보가 더 많으면 빈 환경을 다음 후보로 채울 수 있다.
이번 수치를 근거로 2,048개가 특정 시간 안에 끝난다고 보장하지 않는다.

## 실행

```bash
./nvidia-sim/run_gpu_candidate_dataset.sh \
  --trajectory-mode staged6d --physics-preset contact120 \
  --num-envs 64 --candidates 128 \
  --planning-workers 16 --physics-threads 8 \
  --schedule continuous --representative-videos --video-limit 4
```

하나의 Isaac Sim에서 서로 다른 후보를 동시에 실행한다. 후보 128개를 각 64번
반복한다는 뜻이 아니다. 빈 환경은 다음 후보로 바로 교체한다.
카메라 영상은 전체 후보 실행 중 렌더링하지 않고, 시험 종료 후 대표 사례만
녹화·렌더링한다. 영상은 별도 추가 시간이 든다.

`--representative-videos`는 Tomato_05, staged6d, GPU PGS를 지원하며 원본 Hz/반복 횟수/
접촉 설정을 그대로 재생한다. `invalid_physics` 후보는 대표 사례에서 제외한다.
기본 사진/RGB-D 데이터 저장 동작은 기존과 같다. headless 시험 후 영상 렌더링에
NVIDIA X display를 사용한다 (`--video-display :0`).

기존 완료 데이터셋의 영상만 생성하려면:

```bash
/root/isaaclab_env/bin/python nvidia-sim/rl/dataset_videos.py \
  --run-dir /절대경로/완료된_dataset --max-videos 4 --display :0
```

날짜와 시각으로 시작하는 `*_representative_videos_2x` 폴더에 MP4와 `index.html`을
생성한다. 실패 후 동일 출력 폴더를 재사용하려면 `--output ... --resume`을 지정한다.

## 속도 개선 범위

- CPU 16개 프로세스에서 IK/자가충돌 검사 및 명령별 FK를 미리 계산한다.
- 물리 실행 중 명령 FK를 매 제어 주기마다 반복하지 않는다.
- 동일 제어 주기의 동일 비목표 꼭지 좌표 검사와 고리 geometry 조회를 재사용한다.
- 물리 60 Hz, PGS 64/4, armature 5e-4, 접근 35mm/s와 상승 2mm/s는 유지한다.
- 메모리를 많이 쓰는 것 자체가 빨라지는 것은 아니다. CPU 직렬 접촉 판정과
  GPU physics 동기화가 남아 있어 CPU/GPU 사용률이 항상 100%가 되지는 않는다.

## 대표 사례와 검증

가능한 범주에서 완전 걸림, 중심 진입 부분 성공, miss, 허용 변위 초과,
비목표 위험 접촉 순으로 서로 다른 후보를 선택한다. 기본 최대 4개다.
**완전 성공이 없으면 성공 영상을 만들었다고 표시하지 않는다.**
부분 성공은 열매 중심이 실제 고리 평면의 내부 영역을 통과한 기록이며,
그 뒤 최종 판정이 실패일 수 있다. 수확 성공/실물 안전을 뜻하지 않는다.

대표 영상은 저장된 `planned_commands.npy`를 동일 물리 설정의 단일 환경에서
재실행하고 실제 강체/줄기 변형 상태를 녹화한다. 원본 병렬 실행과 비교한다:

- 명령 파일 SHA256, 최종 판정, 걸림/부분 진입/파손/상승 완료/중단 사유
- 첫 접촉 객체 (복제 환경 번호만 정규화)
- 최대 변위 차이 2mm 이내, 종료 시점 차이 6 tick(0.1초) 이내
- 공통 제어 tick의 phase 일치, 전체 trace의 관절 차이 0.02rad 이내,
  리프트와 목표/주줄기 변위 차이 2mm 이내

허용 오차는 데이터셋의 성공/실패 기준을 바꾸지 않는다. 일치하지 않으면
`REPLAY MISMATCH`로 표시하며 대표 성공으로 사용하지 않는다. 완전한 bitwise
동일성이나 원본 실행을 직접 촬영했다는 주장은 하지 않는다.

두 외부 카메라로 접근 전체와 고리 근접 상태를 함께 보여준다. 원본 모델의
물체를 숨기지 않는다. Tomato_05는 노란색 GT 표식, 07은 청록색 표식이다.
저장한 물리 pose를 Blender로 표시하며 추가 물리 시뮬레이션을 하지 않는다.
30fps, 2배속, 마지막 상태 1초 정지. ffmpeg 전체 디코드 및 프레임 수 검사를 한다.

`manifest.json`, 각 replay의 `comparison.json`, `report.json`, `recording.json`,
`*.ffprobe.json`으로 원본/재실행/영상 검증을 추적할 수 있다.

## 이번 저장 영상 (2026-09-21)

`runs/20260921_175044_staged6d_64env_videos/representative_videos_2x/index.html`

| 후보 | 영상 내용 | 원본 비교 | 길이(2배속+마지막 1초) |
|---|---|---|---:|
| 00005 | 열매 중심 부분 진입, 최종 miss | 통과 | 21.10초 |
| 00001 | 과도한 밀림 | 통과 | 9.10초 |
| 00049 | 비목표 접촉 | 통과 | 20.17초 |
| 00000 | 추가 진단, 최종 miss는 같으나 중간 물리 움직임 차이 | 불일치 | 23.37초 |

00000은 목표 최대 변위 3.12mm 차이, 공통 tick의 주줄기 변위 최대 15.29mm 차이로
선언한 허용 오차를 넘었다. 영상 제목 `REPLAY MISMATCH` 및 별도 진단 섹션으로
구분했으며 대표 영상 수에는 포함하지 않는다. 이것을 해결하려고 허용 오차나
성공 기준을 늘리지 않았다. 나머지 세 후보는 원본 판정/첫 접촉/관절/변위 비교 통과.

네 파일 모두 1280x612, 30fps, ffmpeg 전체 디코드 및 예상 프레임 수 검사 통과.
측면과 근접 시점의 중간/최종 프레임을 확인했고, 근접 시점에서 목표와 고리가 보인다.
로봇을 숨기지는 않았으므로 측면에서는 일부 가려질 수 있다.
영상 후처리 추가 시간은 843.39초. 물리 시험 814.70초와 별도다.

두 실패 사례 녹화는 후처리 중 독립 GPU 프로세스로 미리 병렬 실행했다.
본 시험의 시간·자원 측정과 겹치지 않았다. 일반 영상 runner는 순차 실행하며,
이미 완료된 동일 명령 녹화가 있으면 검증하고 재사용한다.

관련 자동 테스트는 합계 35개 통과했다. 최초 pytest 실행은 ROS 플러그인 자동 로드와
자식 프로세스 PYTHONPATH 설정 때문에 실패했고, 아래 환경으로 재실행해 통과했다.

```bash
cd nvidia-sim/rl
PYTHONPATH="$PWD" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /root/isaaclab_env/bin/python -m pytest -q \
  test_dataset_videos.py test_trajectory_search.py test_gpu_planning.py \
  test_gpu_dataset_runner.py test_gpu_physics_presets.py
```
