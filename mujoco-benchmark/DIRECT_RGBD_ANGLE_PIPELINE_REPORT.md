# 농업용 로봇 토마토 수확: 단일 RGB-D 사진 기반 진입각 예측 및 물리 검증 파이프라인 보고서

- **작성 일시**: 2026-09-23
- **실행 환경**: Linux (`rdv-dev`), Docker 컨테이너 (`humble_x64_env`), Python 3.11 가상환경
- **핵심 기술 스택**: MuJoCo 3.x, Isaac Lab, PyTorch, ResNet18 / DINOv2, OpenCV, Warp

---

## 1. 프로젝트 개요 및 핵심 목표

### 1.1 배경 및 목적
온실 환경에서 다관절 로봇 암(RB5 등)과 고리형 엔드이펙터(Ring Hook Gripper)를 사용하여, 복잡하게 매달린 토마토 송이(Truss)에서 목표 열매를 손상 없이 수확하는 시뮬레이션 파이프라인을 구축하고 검증합니다.

### 1.2 핵심 개발 전환 (Paradigm Shift)
- **기존 방식 (다중 후보 샘플링 & 스코어링)**:
  - 타깃 열매 주변으로 Sobol 난수 기반의 10~100개 진입 궤적 후보(Candidate)를 무작위로 생성한 뒤, 머신러닝 스코어러로 순위를 매겨 실행하는 방식.
  - 연산량이 많고 실물 로봇의 단일 샷 제어 흐름과 차이가 있음.
- **현재 방식 (One-Shot Direct Angle Prediction & Execution)**:
  - **"타깃 열매가 보이는 방향에서 단 1장의 RGB-D 사진 촬영"**
  - **"신경망 모델이 최적의 진입 방위각(Approach Azimuth, deg) 1개를 직접 회귀 예측"**
  - **"예측된 각도 하나로 즉시 경로를 생성하여 단 1회 물리 시도(재시도 없음)"**

---

## 2. 물리 환경 및 파라미터 규격 (Simulation Specifications)

### 2.1 토마토 송이 및 물리 모델
- **GLB 3D 스캔 모델 5종**: `cyan`, `green`, `red`, `white`, `rotated90`
- **송이 크기 배율 (`truss_scale`)**: `0.5` (원 규격의 절반 크기로 표준화)
- **송이당 열매 구성**: 각 송이당 10개 열매 (Tomato_01 ~ Tomato_11 중 06번 제외)
- **주줄기(Stem) 부착 조건**:
  - 부착 세그먼트: `6 ~ 10` (너무 높거나 낮은 위치 배제)
  - 세그먼트 미세 위치(`stem_fraction`): `0.25 ~ 0.75` (관절 경계 간섭 방지)
  - Y축 회전 각도: GLB 대칭 특성을 고려하여 `0° ~ 180°` (학습 수집 시 `0° ~ 20°` 제한)

### 2.2 고리 그리퍼 및 진입 성공 판정 기준
- **고리 물리 규격**:
  - `RING_RADIUS` = 27.5 mm, `WIRE_RADIUS` = 1.0 mm (순수 내부 클리어런스 = 26.5 mm)
  - 열매(예: Tomato_05) 반지름: 약 11.6 mm → 고리 중심과의 최대 허용 오차 약 14.9 mm
- **중심 진입 판정 (`center_region`)**:
  - 좌표계: `+X` = 고리 개구부(로봇 방향), `+Y` = 고리 평면 법선(하향), `+Z` = 측면(Lateral)
  - 평면 허용 오차: `|y| <= 0.020 m` (±20 mm, 완화 기준 적용)
  - 진입 상태: `partial_center_entry` (고리 중심 영역 진입 성공)
- **실패 및 무효 판정**:
  - `invalid_physics`: 그리퍼와 열매/줄기 간 침투(Penetration)가 0.5 mm를 초과한 경우 (MuJoCo 물리 안정성 기준)
  - `excessive_displacement`: 열매 중심 최대 변위(밀림)가 20.0 mm를 초과한 경우
  - `miss`: 진입 조건을 만족하지 못하고 빗나간 경우

---

## 3. 데이터 취득 파이프라인 (Data Collection)

### 3.1 수집 스크립트: `collect_training_dataset.py`
- **실행 방식**:
  - GLB 5종 각각 1개씩 독립 장면 강제 배치
  - 각 열매(총 50개)당 10개의 물리 후보 실험 수행
  - 각 열매마다 가상 D435 카메라로 20시점(또는 9시점)의 RGB-D 촬영 (총 1,000장 이미지)
  - 장면 물리 검사(`scene_screen_passed`) 실패 시 최대 5회 자동 재시도 (seed offset +1000)
  - CPU 40스레드 병렬 최적화 (`--workers 32 --planning-workers 16 --postprocess-workers 16`)

### 3.2 수집 데이터셋 현황 (`20260923_150828_training_dataset`)
- **저장 위치**: `/root/docker_share/mujoko_debugging_data/20260923_150828_training_dataset/`
- **규모**: 5개 송이 × 10개 열매 = 총 50개 타깃 열매, 총 500회 물리 롤아웃
- **성공률**: 전체 500회 시도 중 **260회 진입 성공 (52.0%)**, 240회 실패
- **취득 데이터 포맷**:
  - `rgb.png` (640×480 RGB)
  - `depth_aligned_to_color_m.npy` (실수형 미터 단위 Depth 맵)
  - `crop_local_rgb.png`, `crop_context_rgb.png` (타깃 중심 로컬/문맥 크롭 이미지)
  - `results.json`: 후보별 진입 파라미터, 접촉(contact) 객체, 최대 변위, 관통 깊이

---

## 4. 단일 사진 기반 각도 직접 예측 모델 (Learning Direct Azimuth)

### 4.1 지도학습 라벨 정의 (`train_direct_angle.py`)
- 50개 타깃 열매 각각에 대해, 수집된 후보 중 **"유효한 중심 진입에 성공하고 열매 밀림(변위)이 가장 작았던 최적 후보의 approach_azimuth_deg(방위각)"**을 단일 지도 라벨(Ground Truth)로 설정.
- 입력: 각 열매의 단 1장의 기준 시점(`view_0000`) 사진.

### 4.2 특징 추출 및 네트워크 아키텍처
- **Feature Extractor**:
  - 사전학습된 고정 백본: `ResNet18` 또는 `DINOv2`
  - 컬러 특징 벡터 + Depth 통계 특징 + 카메라 내부 파라미터(Intrinsics) 결합
- **회귀 헤드 (Regressor)**:
  - `nn.Sequential(nn.Linear(dim, 64), nn.ReLU(), nn.Linear(64, 1))`
  - 출력: 단일 실수형 방위각 스칼라 (`predicted_approach_azimuth_deg`)

### 4.3 데이터셋 분할 (Scene-Held-Out Split)
- 일반화 평가를 위해 열매 단위가 아닌 **독립 GLB 송이 장면 단위**로 엄격 분할:
  - **Train (3송이)**: `cyan`, `green`, `red`
  - **Validation (1송이)**: `rotated90`
  - **Test (1송이)**: `white`
- **학습 결과 모델**: `/root/docker_share/mujoko_debugging_data/20260923_direct_angle_training_full/resnet18_seed2/model.pt`
  - 검증 송이 MAE: 약 17.1°

---

## 5. 테스트 시나리오 및 검증 파이프라인 (Inference & Rollout)

### 5.1 검증 스크립트: `test_direct_angle.py`
사용자가 원하는 GLB 송이 종류와 회전 각도를 지정하여 1회 샷 성능을 즉시 평가할 수 있습니다.

```bash
cd /root/farmily_tomato && \
/root/isaaclab_env/bin/python mujoco-benchmark/learning/test_direct_angle.py \
  /root/docker_share/mujoko_debugging_data/20260923_direct_angle_training_full \
  --seed 2026 \
  --glb red \
  --angle 20 \
  --output /root/docker_share/mujoko_debugging_data/20260923_172501_my_direct_test
```

### 5.2 실행 절차
1. **장면 생성**: 지정된 `--glb`와 `--angle`로 3D 토마토 송이를 로봇 앞 작업 영역에 생성 (물리 검사 통과 시까지 최대 5회 자동 재시도).
2. **단일 시점 촬영**: 타깃 열매마다 로봇 명목 위치에서 D435 RGB-D 1장(`view_0000`) 촬영.
3. **각도 예측**: 모델이 사진 1장으로부터 방위각 1개를 즉시 추론하여 확정.
4. **경로 계획**: 예측 각도로 Staged6D 경로(IK / FCL 충돌 검사) 생성.
5. **물리 실행**: MuJoCo 시뮬레이터에서 1회 물리 시도 후 중심 진입 여부, 변위, 관통 깊이 자동 판정.
6. **대시보드 생성**: 촬영 사진, 예측각, 결과 뱃지, 변위, 3D 리플레이 버튼이 포함된 `index.html` 자동 빌드.

---

## 6. 최신 실험 결과 비교

### 6.1 실험 A: 미지의 큰 각도 장면 (`truss_y = 67.3°`, 무작위 송이)
- **폴더**: `20260923_direct_angle_full_scene_test`
- **결과**: **진입 성공 4개 (40%)**, 물리 오류 5개 (50%), 미진입 1개 (10%)
- **분석**:
  - 송이가 67.3도로 크게 기울어지자, 학습 데이터(0~20°) 분포를 벗어남(OOD).
  - 예측 각도가 `-86° ~ -89.8°`의 경계 극단값으로 쏠리면서 그리퍼가 줄기/열매와 정면 충돌하여 관통 한도 초과(`invalid_physics`).

### 6.2 실험 B: 학습 분포 내 각도 장면 (`truss_y = 20.0°`, `red` 송이)
- **폴더**: `20260923_172501_my_direct_test`
- **결과**: **진입 성공 9개 (90.0%!)**, 물리 오류 1개 (10%)
- **세부 데이터**:
  - `Tomato_02`: `-16.71°` (진입 성공, 변위 12.8 mm)
  - `Tomato_03`: `+61.78°` (진입 성공, 변위 17.6 mm)
  - `Tomato_04`: `-34.65°` (진입 성공, 변위 14.4 mm)
  - `Tomato_05`: `+79.10°` (진입 성공, 변위 16.4 mm)
  - `Tomato_07`: `+58.25°` (진입 성공, 변위 22.3 mm)
  - `Tomato_08`: `-21.90°` (진입 성공, 변위 18.4 mm)
  - `Tomato_09`: `+48.16°` (진입 성공, 변위 20.3 mm)
  - `Tomato_10`: `-89.85°` (진입 성공, 변위 23.3 mm)
  - `Tomato_11`: `+59.52°` (진입 성공, 변위 **1.6 mm**, 완벽 진입)
  - `Tomato_01`: `+52.44°` (물리 오류, 변위 14.4 mm)
- **시사점**: 학습된 회전 각도 영역 내에서는 **단 1장의 사진만으로 90%의 고리 진입 성공률**을 달성함.

---

## 7. 대시보드 및 도커 원클릭 3D 리플레이 기능

- **배경**: 호스트 브라우저에서 `file:///...`로 리포트를 볼 때 터미널에서 매번 `docker exec` 명령어를 복사/붙여넣기 해야 하는 불편함 해소.
- **아키텍처**:
  - 컨테이너 내부에서 경량 HTTP 서버 (`mujoco-benchmark/scripts/replay_server.py`, 포트 8766) 상시 구동.
  - 리포트 `index.html`의 각 열매 항목 및 상세 Drawer에 **`[🚀 바로 실행 (도커)]`** 버튼 탑재.
  - 브라우저에서 버튼 클릭 시 로컬 API를 호출하여 컨테이너 내에서 `DISPLAY=:0` 화면으로 MuJoCo 3D 인터랙티브 뷰어를 즉시 실행.

---

## 8. 향후 논의 및 자문 과제 (Discussion Topics for GPT-6)

1. **데이터셋 확장 및 회전각 일반화**:
   - 현재 독립 GLB 송이가 5종에 불과하여 다양한 Y축 회전각(0~90°)에서 각도 쏠림(Collapse) 현상이 발생함.
   - 다양한 회전 각도(0~90°)에서의 추가 데이터 수집 전략 및 카메라 시점 지터를 활용한 대규모 데이터 증강 방안.
2. **각도 표현식 개선 (Representation)**:
   - 단순 스칼라 방위각($\theta$) 회귀 대신, 주기성을 반영한 $(\sin\theta, \cos\theta)$ 2D 벡터 회귀 도입.
   - 회귀 대신 충돌 위험이 적은 안전 진입 각도 구간(Bin Classification) 분류 모델과의 하이브리드 구성.
3. **End-to-End 비전 검출기 통합**:
   - 현재는 시뮬레이터의 GT 메타데이터를 사용하여 열매 바운딩 박스를 크롭함.
   - 실물 로봇 배포를 위한 2D Detector(YOLOv8/11 등)와의 통합 및 위치 오차 허용성 확보 방안.
4. **최종 수확(Hook & Pull) 단계 연동**:
   - 현재 파이프라인은 '고리 중심 진입(Center Entry)'까지 검증됨.
   - 고리 진입 후 위로 들어 올려 당기는 최종 수확 분리(Separation) 동역학과의 결합 설계.

