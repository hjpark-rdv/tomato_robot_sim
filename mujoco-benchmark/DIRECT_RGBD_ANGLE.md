# RGB-D 한 장에서 진입각 하나 예측

이 실험은 타깃 열매의 **실행 전 RGB-D 한 장**을 모델에 넣어 세계 좌표의 진입 방위각 한 개를 출력한다. 예측 전에 Sobol 후보를 생성하거나 여러 후보를 모델로 채점하지 않는다. 예측 각도로 기존 staged6d 경로를 한 번 계획하고, 계획이 통과하면 기존 MuJoCo 실행기에서 한 번 물리 시험한다.

## 데이터와 학습

완료된 `20260923_150828_training_dataset`의 `training_dataset_v1`을 사용한다. 이 수집은 GLB 5장면, 장면당 열매 10개, 열매당 물리 후보 10개와 RGB-D 20시점을 갖는다. 학습에서는 각 열매의 기준 시점 `view_0000` **한 장만** 사용한다. 유효한 중심 진입 후보 중 열매 최대 변위가 가장 작은 후보의 진입각을 지도 라벨로 택한다. 따라서 라벨은 측정된 성공각 하나이며 유일한 최적각이나 꼭지 수확각을 뜻하지 않는다. 유효한 진입 후보가 전혀 없는 열매에는 직접 각도 라벨을 만들지 않는다.

RGB와 실수 Depth는 기존 `learning/features.py`의 고정 ResNet18/DINOv2 및 Depth 특징 추출을 사용한다. 모델 입력에는 후보 각도와 성공/실패 결과가 들어가지 않는다. 해당 특징은 GT 열매 중심으로 만든 local/context crop을 사용한다. 이는 현재 시뮬레이터의 타깃 위치를 활용하며, 실물 RGB-D만으로 타깃 검출·크롭하는 기능은 아직 없다. 가상 카메라는 기존 로봇 nominal 시점을 대상 중심에 맞게 **병진 이동**한 것이고 실제 로봇 카메라 도달성은 검증되지 않았다.

장면 단위 분할: cyan/green/red 학습, rotated90 검증, white 보류 테스트. 같은 장면의 다른 열매를 다른 분할에 넣지 않는다. 모델은 사진 특징 → 방위각 스칼라 한 개를 회귀하고, 검증 각도 MAE로 모델·seed를 선택한다. 보류 장면의 각도 오차와 일정각 기준을 함께 저장한다. 독립 식물 장면은 총 5개뿐이라 일반화 수치는 제한적이다.

```bash
cd /root/farmily_tomato
/root/isaaclab_env/bin/python mujoco-benchmark/learning/train_direct_angle.py \
  /root/docker_share/mujoko_debugging_data/20260923_150828_training_dataset \
  --output /root/docker_share/mujoko_debugging_data/NEW_direct_angle_training
```

## 새 랜덤 장면 물리 시험

`test_direct_angle.py`는 기존 `generate_random_glb_scenes.py`로 송이 위치와 GLB Y 회전을 설정하고, 각 열매에 대해 다음 순서를 따른다.

1. `candidate_experiment.py --scene-only`로 원래 모델·로봇·계획 자산을 저장한다. 이때 진입각 후보는 생성하지 않는다.
2. `prepare_observations.py --views 1`로 타깃 기준 RGB-D 한 장을 촬영한다.
3. 학습 모델이 그 한 장에서 진입각 하나를 예측한다. 모든 각도 예측을 `predictions.json`에 물리 실행 전에 확정한다.
4. 기존 `plan_candidates.py`에 그 각도 하나를 전달해 IK/FCL 계획을 수행한다. 통과하면 기존 `candidate_experiment.execute`에서 물리 실행·판정·재생 상태를 저장한다. 실패하면 계획 실패를 기록하며 다른 각도로 대체하지 않는다.

```bash
/root/isaaclab_env/bin/python mujoco-benchmark/learning/test_direct_angle.py \
  /root/docker_share/mujoko_debugging_data/NEW_direct_angle_training \
  --seed 1002 --angle-min 0 --angle-max 90 \
  --output /root/docker_share/mujoko_debugging_data/NEW_direct_angle_test
```

열매당 결과는 `targets/Tomato_XX/prediction.json`, `observation/`, `physics/`에 저장한다. 전체 `summary.json`과 `index.html`은 예측각·계획 통과·물리 판정을 연결한다. 훈련 seed 42와 같은 seed는 거부한다. 장면 초기 물리 검사에 실패하면 해당 장면은 실행하지 않고 실패를 저장한다.

연속 시험은 `--repeat`로 지정한다. 장면 하나의 열매를 모두 처리하고 저장한 뒤 다음 seed의 장면을 생성한다. 각 장면 결과는 `test_0000/`, `test_0001/`처럼 분리한다.

```bash
/root/isaaclab_env/bin/python mujoco-benchmark/learning/test_direct_angle.py \
  /root/docker_share/mujoko_debugging_data/NEW_direct_angle_training \
  --seed 1002 --repeat 10 --angle-min 0 --angle-max 90 \
  --output /root/docker_share/mujoko_debugging_data/NEW_direct_angle_repeated_test
```

현재 물리 결과는 `partial_center_entry`, `miss`, `invalid_physics`, `ik_or_planning_failure`다. `partial_center_entry`는 중심 진입 이력으로, 꼭지 걸림이나 수확 성공이 아니다. 물리 겹침 기준과 저장 형식은 기존 실행기를 따른다. 사진만으로 예측한 각도가 실제 로봇에 도달 가능한지는 계획 및 물리 단계에서 새로 확인한다.
