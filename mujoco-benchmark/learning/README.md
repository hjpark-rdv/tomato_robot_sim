# Tomato_05 영상 기반 후보 경로 평가기

## 선택 근거 (2026-09-22 조사)

현재 데이터는 한 물리 장면의 1,000개 후보 결과와71개 가상 카메라 관측이다. 71,000개쌍을독립실험으로보면안된다. 전체trajectory를생성하는대형정책보다 **사전학습RGB특징 + Depth + 카메라기준후보 → 진입확률/최대이동량**을예측하는작은평가기부터검증한다. 이것은현재데이터규모에대한설계판단이며특정논문이본고리작업에서우수함을보장하는것은아니다.

| 계열 | 공식 자료 | 이번 판단 |
|---|---|---|
| DINOv2 ViT-S/14 | [논문](https://arxiv.org/abs/2304.07193), [공식 구현](https://github.com/facebookresearch/dinov2) | 사전학습특징을고정하고작은헤드만학습가능.22M RGB backbone, 이번비교대상 |
| DINOv3 ViT-S/16 | [2025 논문](https://arxiv.org/abs/2508.10104), [공식 구현](https://github.com/facebookresearch/dinov3) | 최신dense특징이유망.공식가중치접근신청필요;이번에는미학습.현작업우월성미검증 |
| ResNet18 | [Torchvision 공식 문서](https://docs.pytorch.org/vision/stable/models/generated/torchvision.models.resnet18.html) | 가벼운ImageNet사전학습비교모델.작은데이터에서DINO보다불리하다고단정하지않음 |
| Contact-GraspNet | [논문](https://arxiv.org/abs/2103.14127), [NVIDIA 구현](https://github.com/NVlabs/contact_graspnet) | 점군→일반6DoF파지.현재고리진입/탄성/이동량점수와출력목적이다르므로직접교체하지않음 |
| Diffusion Policy / DP3 | [공식 프로젝트](https://diffusion-policy.cs.columbia.edu/), [DP3 논문](https://arxiv.org/abs/2403.03954) | 시연기반정책/시계열동작학습.현재단일관측+실패포함후보평가에비해문제가큼.추후다양한성공시연과폐루프제어단계에서재검토 |

DINO/ResNet은RGB처리기일뿐RGB-D용사전학습모델이아니다. Depth는별도metric특징으로결합한다.고리성공을학습한기존가중치를받는것도아니다.

## 실행

기존CUDA PyTorch환경을사용하며IsaacSim을켜지않는다. 최초실행은공식DINOv2/ResNet18가중치를다운로드한다.

```bash
cd /root/farmily_tomato
/root/isaaclab_env/bin/python mujoco-benchmark/learning/train.py \
 /root/docker_share/mujoko_debugging_data/20260922_122302_tomato05_observations
```

기본150epoch상한,검증손실30epoch미개선시중단,seed0/1/2각각학습.백본은고정해관측별특징을한번만계산한다.출력은datetime_camera_action_training폴더.기존시뮬레이션/데이터는변경하지않는다.

## 입력과 목표

- target주변3배/6배crop의원본RGB각224×224,ImageNet정규화.표시된미리보기/GT마스크는학습하지않는다.화면가장자리crop도사용하고K를crop크기로정규화해함께제공한다.목표지정crop자체는GT기반이므로실제검출오차는이번평가에없다.
- aligned depth를crop후16×16mask-aware평균pooling.거리m와유효율을입력.무효값을가짜0거리로해석하지않는다.2m범위clip.원본센서노이즈모사는없다.
- 카메라축기준14개action + 관측중력방향3개.중력은시뮬레이터캘리브레이션값,실물에서도해당방향추정/외부파라미터가필요하다.
- 모델입력에후보ID/월드pose/실제결과/GT토마토3D중심을넣지않는다.
- BCE: `center_entered` (과도변위후진입도양성).회귀:전체후보의최대이동량/20mm, smooth-L1.결합손실BCE+0.25×회귀.
- 추천점수: `P(center_entry) * exp(-max(predicted_displacement_mm,0)/20)` 고정.확률은아직별도calibration하지않았다.진입하지않고가만히있는후보를이동량만보고좋다고판정하지않는다.

## 누수 방지와 평가

- 기존시점분할:train50/validation7/test7/visibility_test7.가시과실3pixel인view_0066은visibility_test에만포함하며학습하지않는다.
- 후보도결과종류별고정seed층화70/15/15분할.같은후보는다른시점에서도train/test를넘나들지않는다(주평가:새시점+새후보).
- train시점×train후보만학습.정규화통계도이조합에서만추정.검증시점×검증후보손실로checkpoint선택.모델종류선택은3seed검증손실평균.테스트점수로모델/seed선택하지않는다.
- 보조평가:새시점+기존후보,화면잘림stress관측.전체예측CSV저장.
- action_only(카메라action+중력),ResNet18_RGBD,DINOv2_RGBD비교.각영상모델은동일action을유지하고RGBD관측만다른시점으로바꿔민감도검사.이검사는RGB/Depth/K를함께바꾸므로RGB단독기여를증명하지않는다.
- AP,이동량MAE,1순위실제중심진입률,1순위진입+20mm이하변위율.후보순위는학습모델예측으로정하고실제결과는그후확인.
- 고정장면에서는이미측정된가장좋은train후보를암기하는것만으로도매번같은결과를얻는다.보고서에그기준도명시한다.카메라분석의필요성은식물/배치가다른holdout에서추가검증해야한다.
- 테스트7시점/3seed수치는새로운식물7개/3회독립물리시험이아니다.성공률의실물신뢰구간으로해석하지않는다.

## 저장

`manifest.json`(정확한분할/설정/해시),`features.npz`,백본가중치,feature_manifest입력파일해시,`source/`코드사본,모델별`model.pt`,history/metrics.json,predictions.csv,전체results.json,index.html을저장한다.모든시간은wall clock.모델score는기존물리결과와오프라인비교하며새물리rollout이나실제로봇작동은수행하지않는다.

## 이번 실행 결과

`/root/docker_share/mujoko_debugging_data/20260922_130308_camera_action_training/index.html`

특징 추출 및 3모델×3seed 전체 41.21초. 후보 분할 train699/validation149/test152. 새 시점7개×새 후보152개를 주 평가로 사용했다.

| 모델 | 진입 AP (3seed 평균) | 최대 이동량 MAE | 1순위 진입+20mm 이하 (7시점×3seed) |
|---|---:|---:|---:|
| 영상 없음 | 0.837 | 2.575 mm | 80.95% |
| ResNet18 RGB-D | 0.849 | 2.535 mm | 100% |
| DINOv2 RGB-D | 0.855 | 2.618 mm | 95.24% |

검증 손실 평균은 영상 없음0.2231 / ResNet0.2419 / DINO0.2324로, 미리 정한 선택 규칙에 따라 **영상 없는 모델이 선택됐다**. 테스트 표만 보고 ResNet을 최종 승자로 바꾸지 않았다. 영상 관측을 바꾸면 AP가 ResNet0.691 / DINO0.698로 떨어져 관측 정합성에 대한 의존은 보이지만, 단일 장면이며 Depth/K도 함께 바꾼 검사라 RGB 기반 일반화 증거는 아니다.

이번 데이터에서 대형 영상 모델의 확실한 우위를 확인하지 못했다. 이미지 기반 개발을 계속할 때는 가벼운 ResNet18과 DINOv2를 비교 대상으로 유지하되, 먼저 다른 물리 장면에 대한 검증 데이터를 확보하는 것이 중요하다. 알려진 단일 장면에서는 train후보87을 기억하기만 해도 9.65mm 이동으로 진입한다.

저장 모델로 기존 관측의 후보를 다시 추천하기(실제로 로봇을 움직이지 않음):

```bash
/root/isaaclab_env/bin/python mujoco-benchmark/learning/predict.py \
 /root/docker_share/mujoko_debugging_data/20260922_130308_camera_action_training \
 --view view_0015 --model dinov2_rgbd --scope test --top 5
```

이 명령은 저장된 관측 특징을 사용한다. 새 실물 RGB-D를 직접 입력하는 배포기는 이번 범위가 아니다. `--scope all`은 학습에서 본 후보까지 포함하므로 독립 테스트 점수로 해석하지 않는다. 출력은 카메라 기준14개값과타깃상대경유점이며 실제 좌표 변환·IK 확인은 별도다.

## 추천 경로 실제 물리 재실행

`test_physics.py prepare`를 PyTorch환경에서 실행해 seed0의 세모델×test시점7개 추천을 결과조회 없이 고정한다. 카메라 상대경로→월드경로 일치를 검사하고, 해당 후보의 기존 IK/충돌검사된 관절 명령을 보존한다. `execute`는 MuJoCo환경에서 매회 reset 후 새 physics를 계산한다. 모델이 새 연속 trajectory를 생성하는 실험은 아니다.

```bash
/root/isaaclab_env/bin/python mujoco-benchmark/learning/test_physics.py prepare OUTPUT --training TRAINING_RUN
mujoco-benchmark/.venv/bin/python mujoco-benchmark/learning/test_physics.py execute OUTPUT --workers 8
mujoco-benchmark/.venv/bin/python mujoco-benchmark/learning/render_physics.py OUTPUT
```

실제 완료: `/root/docker_share/mujoko_debugging_data/20260922_model_recommendation_physics/index.html`.
21회 / 서로다른경로3개 / 물리계산41.52초(영상제외). ResNet18과DINOv2각각중심진입7/7,진입+최대이동20mm이하7/7. 영상없는모델중심진입7/7,20mm이하4/7. 후보807=19.21mm,631=20.96mm,920=17.50mm. 모든 초기qpos동일/원본 물리 결과와 일치/불안정0.

상태30fps와원본120Hz진단저장. 대표3경로2배속영상. 카메라는전체(-45°,-15°),근접(-90°,-30°)로고정해로봇뒤가림을줄임. 영상은이번새실행의기록상태를재생하며추가물리계산없음. 동일장면결정론적재실행이며독립21개토마토에대한성공률이아니다.
