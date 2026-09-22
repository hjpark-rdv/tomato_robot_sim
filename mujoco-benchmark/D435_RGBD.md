# ROS/Isaac 장착 위치의 D435 RGB-D

`d435_capture.py`는 저장된 MuJoCo 후보의 관절 상태를 불러오고 그리퍼 Hook body에 카메라를 붙여 촬영한다. 물리를 다시 실행하지 않으며 원본 모델/상태 파일을 변경하지 않는다. 카메라 슬롯만 촬영 프로세스 안에서 사용한다.

## 촬영

```bash
cd /root/farmily_tomato
mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/d435_capture.py \
 /root/docker_share/mujoko_debugging_data/20260922_111838_candidate_search \
 --candidate candidate_00087 --phase preapproach
```

`--phase initial|preapproach|entry|insert|rise`는 각 단계가 끝난 저장 프레임을 선택한다. 기본 preapproach는 원거리 접근 준비가 끝난 뒤 본격 진입하기 전이다. `--time 5.0`으로 기록 시각을 지정할 수도 있다. 30Hz 상태는 가장 가까운 프레임을 선택하고 요청/실제 시각을 모두 기록한다. `--output`이 없으면 후보 폴더의 `rgbd/날짜시간_단계/`에 저장한다. 모든 후보 자동 촬영이나 ROS 토픽 송신은 이번 변경에 포함하지 않았다.

## 동일 장착 위치의 근거

- ROS xacro: `rbpodo_description/robots/rb5_farmily.urdf.xacro`의 d435_mount 기본값.
- Isaac URDF: `nvidia-sim/robot_usd/rb5_farmily.urdf`.
- 그리퍼→bottom_screw: xyz `[-0.04915, 0.0415, 0.0125]`, rpy `[-1.57, 0, 3.141592]`.
- URDF의 bottom_screw/link/color/depth/optical 전체 고정 체인을 곱한다. color와depth 프레임15mm 오프셋 유지.
- USD `robot_export_only.usda`와 비교한 상대 변환 행렬 최대 성분 차이1.283e-7.
- ROS optical(+Z 전방,+Y 아래)에서 MuJoCo 카메라(-Z 전방,+Y 위)로 local X180도 변환.
- 실제 ROS에서 launch 인자로 기본 장착값을 바꿨거나 실측 보정을 적용했다면 그 값을 별도로 반영해야 한다.

## 영상/깊이 규격

기존 Isaac `run_farm_simulation.py`의 카메라와 동일한640×480, 수평FOV69.4°, clip0.10~20m. 내부행렬은 이 FOV에서 계산한 값으로 실제 장치 보정값은 아니다. 노이즈/스테레오 매칭/반사/실물 최소 검출 거리까지 모사하지 않는 이상적인 pinhole depth이다.

- `rgb.png`: color optical frame 영상.
- `depth_m.npy`: depth optical frame의 광축 Z거리(float32, m). 무효는NaN.
- `depth_valid.png`: 원본 깊이 유효 mask.
- `depth_aligned_to_color_m.npy`: 원본depth 점을color로 투영하고 z-buffer로 합친 깊이. 가림/시야차의 빈 영역은NaN.
- `aligned_valid.png`: 정렬 깊이 mask.
- `depth_preview.png`: 0~1m를 밝기로 표시하는 미리보기(수치 분석용 아님).
- `camera.json`: K, 해상도, optical world pose, tool pose, source hash, 시각 및 후보 정보.

가림과10cm clipping을 그대로 유지하므로 가까운 고리/꼭지가 보이지 않을 수 있다. 관찰 대상을 보기 위해 카메라를 자동으로 돌리지 않는다. 배경이 없는 장면은 배경depth가무효이다.

검증: 후보87 preapproach/entry 실제 이미지 저장. synthetic평면 광축거리0.74m의 렌더 결과0.73999995m. 코드 축 변환 근거: https://mujoco.readthedocs.io/en/stable/programming/visualization.html .
