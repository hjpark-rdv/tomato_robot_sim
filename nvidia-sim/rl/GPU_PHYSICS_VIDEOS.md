# 60Hz 물리 확인 영상 (2026-09-21)

결과 폴더: `runs/20260921_162920_practical60_contact_videos_2x/`

- [브라우저 영상 모음](runs/20260921_162920_practical60_contact_videos_2x/index.html)
- [전체 요약, 2배속](runs/20260921_162920_practical60_contact_videos_2x/videos/00_overview_2x.mp4)
- [실제 로봇의 밀림](runs/20260921_162920_practical60_contact_videos_2x/videos/01_robot_push_2x.mp4)
- [과실 하중과 복원](runs/20260921_162920_practical60_contact_videos_2x/videos/02_elastic_recovery_2x.mp4)
- [미리 끼운 고리의 접촉 유지](runs/20260921_162920_practical60_contact_videos_2x/videos/03_hook_retention_2x.mp4)
- [고리 시험 1배속](runs/20260921_162920_practical60_contact_videos_2x/videos/03_hook_retention_1x.mp4)

기존 `practical60` 조건을 실제 GPU 물리로 재실행했다. 카메라는 후보 생성용 RGB-D가
아닌 관찰용 외부 카메라다. 로봇/잎을 숨기지 않고 여러 방향을 미리 렌더링한 뒤,
측면 전체 장면과 위쪽 사선 확대 화면을 골랐다. 각 영상에 목표 GT 이름, phase,
물리 시간, 목표/주줄기 변위를 표시한다. 고리 시험에는 native target contact,
rear 영역 간격, seated 여부를 추가했다.

## 해석 범위

| 영상 | 실제로 수행한 시험 | 확인 결과 |
|---|---|---|
| 실제 로봇 | 저장된 candidate_00003 명령 실행 | 주줄기에 붙은 잎을 먼저 접촉. 주줄기 30.28mm, 과실 18.95mm 이동 후 과도 변위 중단. 수확/고리 진입 성공 아님 |
| 휘어짐·복원 | 준비 자세 로봇, 과실에 0.2N 하중 1초 후 제거 | 과실 최대 24.24mm, 제거 4초 뒤 잔류 1.32mm. 접촉으로 가한 하중은 아님 |
| 고리 걸림 | 별도 kinematic 반원 고리+rail을 GT 꼭지에 미리 삽입 | 네 회전각에서 1초 유지 구간 접촉·seated 조건 통과. 이후 뒤로 풀고 복원. 로봇 진입/IK/전체 그리퍼 시험 아님 |

촬영 실행의 단계별 공통 수치, native 접촉 이벤트, 네 고리 결과, 로봇 결과가
기존 무촬영 60Hz 실행과 동일함을 `physics_recording_check.json`으로 확인했다.
전체 MP4 검사 결과는 폴더의 `video_validation.json`에 저장한다.
원본 대용량 자료와 영상은 ignored `runs/`에 있으며 Git만 옮기면 영상은 따라오지 않는다.

## 촬영 코드와 재실행

`gpu_probe_worker.py --record-physics-video`로 한 환경의 60Hz motion/behavior probe를
기록한다. 선택 preset과 같은 설정은 명시적으로 지정해야 한다. 진단 runner의
기본값은 960Hz이며, 이 옵션이 물리 설정을 자동으로 바꾸지는 않는다.

```bash
/root/isaaclab_env/bin/python -u nvidia-sim/rl/gpu_probe_worker.py \
  --headless --mode gpu --solver pgs --batched-io --native-replication \
  --gpu-partitions 1 --num-envs 1 --physics-hz 60 \
  --joint-armature .0005 --position-iterations 64 \
  --record-physics-video --behavior-probe \
  --fixture nvidia-sim/rl/runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003 \
  --output nvidia-sim/rl/runs/NEW_DATETIME_mechanics_capture
```

로봇 명령을 촬영할 때는 `--behavior-probe`를 빼고 별도 출력 폴더를 지정한다.
결과의 `captures/<clip>/`에 geometry, native 몸체 위치·회전, trace, metadata가 저장된다.
위 완료 폴더의 `commands.json`, `render_commands.json`, `views/*.json`에는 실제 사용한
명령과 카메라 위치가 있다. 새 경로로 복사/수정한 뒤 재사용한다.

```bash
DISPLAY=:0 __GLX_VENDOR_LIBRARY_NAME=nvidia blender -t 4 \
  --python nvidia-sim/rl/pose_video_render.py -- \
  --scene CAPTURE/scene.npz --capture CAPTURE --output VIDEO.mp4 \
  --gpu --views VIEWS.json --playback-speed 2
```

`--preview-times 0,1.6,4.8`로 실제 물리 시간별 카메라 미리보기를 만들 수 있다.
`--playback-speed 1`은 같은 상태의 1배속 영상이다. 물리를 다시 실행하지 않는다.

`physics_video_capture.py`는 실제 pose를 15 simulation fps로 읽는다. 출력은 30fps,
기본 2배속이며 낮은 배속에서는 동일 프레임을 반복한다. 보간으로 접촉을 만들지 않는다.
원래 USD geometry/탄성 스키닝을 Blender studio shading으로 표시하므로 RTX 조명과
재질은 다르다. 별도 fixture의 표시 색만 은색으로 바꿨다.

native teleport 직후 kinematic fixture의 USD transform이 아직 주차 좌표에 남는 경우를
확인했다. exporter는 추가 몸체의 USD-local geometry를 **실제 native rest pose**에
변환해 묶는다. 물리 상태 자체를 이동시킨 것이 아니다. 이 처리를 빼면 고리가 영상에서
사라지므로 렌더링과 접촉 로그를 함께 검사해야 한다.
