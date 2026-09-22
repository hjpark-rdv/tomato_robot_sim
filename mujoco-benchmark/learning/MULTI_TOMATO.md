# 여러 토마토의 RGB-D 진입각 학습

## 이번 데이터와 실험

원본: `/root/docker_share/mujoko_debugging_data/20260922_232150_multi_tomato_collection`

10개 토마토(01,02,03,04,06,07,08,09,10,11), 각각30개 각도 후보/71개 가상 RGB-D 시점. 총300개 물리 경로,710시점. Tomato_05는 이번 경로 조건의 데이터가 없어서 과거 데이터를 섞지 않았다. 중심 정렬/하부2mm/±90° Sobol/대각45°상승 후 반대45°상승/중심진입 단독 성공 기준이다.

학습8대상(01,02,03,04,07,08,10,11), 검증09, 테스트06. 테스트 대상의 어떤 시점이나 성공 라벨도 학습/정규화/모델 선택에 사용하지 않는다. 후보 격자는 모든 대상에 동일하며, 새로운 각도 생성 성능이 아니라 학습하지 않은 **동일 식물의 다른 열매**에서 기존30개 경로 순위를 매기는 실험이다. GT 타깃 크롭과 시뮬레이터 카메라 정보가 필요하다.

마스크는 입력에 사용하지 않는다. 영상 안 유효 타깃·local/context crop이 있는 관측만 사용하고 visibility_test를 제외: 총617시점(학습489/검증64/테스트64). 제거된 시점의 성공률은 주장하지 않는다.

## 학습기

기존 작은 Scorer의 출력만1개로 구성. 후보 camera action14 + gravity3와 고정 RGB-D 특징을 입력한다. action_only / ImageNet ResNet18 / DINOv2 ViT-S14를 seed0,1,2로 비교. 손실은 중심진입 BCE, 추천점수는 진입 sigmoid만 사용한다. **열매 밀림은 점수나 손실에 넣지 않으며 결과 표시 지표로만 남긴다.**

고정 상승 profile이 섞이면 학습을 거부한다. compact action14만으로 수직/대각 상승을 구별할 수 없기 때문이다. 전체5개 waypoint는 사진 데이터와 추천에 보존한다. 불안정/계획실패 실행도 임의로 음성으로 취급하지 않고 입력 오류로 중단한다.

모델 계열은3seed 평균 검증 BCE, 해당seed는 그 계열의 검증 BCE로 선택한다. 테스트 결과를 보고 모델을 바꾸지 않는다. 체크포인트 inference는 성공 라벨을 읽지 않는다. 현재 predict_multi는 저장된 테스트 관측의 특징을 사용하며 새 실물 이미지 입력 API가 아니다.

## 실행

```bash
cd /root/farmily_tomato
/root/isaaclab_env/bin/python mujoco-benchmark/learning/train_multi.py \
 /root/docker_share/mujoko_debugging_data/20260922_232150_multi_tomato_collection \
 --test-target Tomato_06 --validation-target Tomato_09
```

결과: `/root/docker_share/mujoko_debugging_data/20260922_234403_multi_tomato_training`

- `index.html`: 모델 비교/선택 모델 추천/제약
- `manifest.json`: 대상 분할/해시/선택 기준, `results.json`:9개 모델 결과
- `features/Tomato_XX`: 입력 해시·특징, `backbones`: 중복 저장하지 않는 고정 백본
- 각 모델 폴더: 체크포인트/학습 이력/테스트 점수 배열
- `recommendations.json`: 선택 모델의 확정 추천
- `physics/`: 서로 다른 추천 경로를 초기화하여 새 물리 실행한 결과

추천:

```bash
/root/isaaclab_env/bin/python mujoco-benchmark/learning/predict_multi.py \
 /root/docker_share/mujoko_debugging_data/20260922_234403_multi_tomato_training \
 --view view_0000 --top 5
```

새 학습 폴더에서 추천을 물리 검증하려면(기존 physics 폴더는 덮어쓰지 않음):

```bash
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/learning/test_multi_physics.py \
 /학습/결과/폴더 --workers 8
```

## 결과와 해석

학습/특징 추출23.36초. 검증 기준으로 DINOv2 RGB-D seed2 선택. Tomato_06의64시점 모두 candidate_00001(−81.09°) 추천, 기존 결과 조회상 중심진입64/64. 균등 무작위 후보의 진입 기대비율은3/30=10%. 하지만 **영상 없는 action_only도64/64**이므로 카메라 모델의 우위는 입증되지 않았다. 영상 모델의 다른 관측 RGB-D 대체 시 top1은0%였지만 이것만으로 일반화나 이미지의 인과적 기여를 확정할 수 없다.

선택 모델의 테스트 Brier는 약0.315로 확률 예측 품질은 좋지 않다. sigmoid가1.0에 가까워도 교정된100% 확률로 해석하면 안 된다. 1순위 선택 결과와 전체 후보 확률의 품질은 별개다.

고유 추천은1개이므로 해당 경로를 **새 물리로1회** 실행: 중심진입 성공, 목표 열매 최대 밀림30.78mm, 물리 검증17.60초.64시점별 독립 물리 성공64회가 아니다. 모델은 저장된 경로를 선택했으며 새로운 관절 경로를 생성한 것이 아니다. 꼭지걸림/수확/실물 성공은 미검증.

실제 새 물리 재생:

```bash
DISPLAY=:0 ./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/replay_candidate.py \
 /root/docker_share/mujoko_debugging_data/20260922_234403_multi_tomato_training/physics \
 --candidate candidate_00001
```
