# 새 줄기 GLB 외형 선택

`nvidia-sim/env_usd/tomato_stem_leaves_v9.glb`를 기존 16분절 탄성 주줄기에 연결한다. 기본 줄기와 기존 데이터는 그대로이며 `--stem-glb` 지정 시에만 적용한다.

새 GLB는 기존 주줄기와 같은 중심선/반경을 가지고 아래쪽 잎을 추가한 자산이다. GLB Y-up → 시뮬레이션 Z-up 변환 및 0.5배율을 적용한다. 223개 mesh primitive, 524,809개 삼각형을 모두 유지한다. 동일 이름의 여러 primitive도 빠뜨리지 않는다. 기존 중심선과 최대 약 0.00042mm 차이, 반경 약 0.000007mm 차이다.

## 랜덤 장면 생성

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/generate_random_glb_scenes.py \
  --scenes 1 --trusses 1 --seed 2026 --glb red \
  --angle-min 20 --angle-max 20 \
  --stem-glb nvidia-sim/env_usd/tomato_stem_leaves_v9.glb
```

하우스는 `--house` 추가. 기존 `learning/test_direct_angle.py`에도 동일한 `--stem-glb PATH` 옵션을 추가했으므로 새 테스트 생성 장면에 적용할 수 있다. 기존 학습 가중치를 사용한 새 잎 배경의 예측 정확도는 미검증이다.

## 변환 범위

- 기존 STEM 외형을 숨기고 새 외형을 기존 관절 body에 연결한다. 주줄기 표면은 기존 16분절에 나눠 연결한다.
- 잎/잎자루는 주줄기와 가장 가까운 정점으로 소속 분절을 정해 해당 분절을 따라 움직인다. 부드러운 skinning이나 잎 자체의 탄성은 아니다.
- 기존 질량/관성/강성/감쇠/충돌 형상은 유지한다. 새 잎은 시각 전용이며 충돌·질량·별도 관절을 추가하지 않는다.
- 새 줄기 형태가 기존 충돌 골격과 0.01mm 이상 다르면 변환을 거부한다. 다른 형상의 줄기는 별도 물리 변환이 필요하다.
- `model.stem.json`에 원본 해시·좌표변환·primitive별 소속 body·삼각형 수를 저장한다.

## 확인 자료

`/root/docker_share/mujoko_debugging_data/20260927_stem_leaves_v9/index.html`

기존/신규 외형 비교. 기존과 신규 모델 240스텝 실행 qpos 차이 0 확인. 이는 짧은 동등성 검사이며 새 잎과의 충돌 시험이 아니다. 랜덤 생성 통합 확인은 `20260927_stem_v9_generator_smoke`에 저장한다. 원본 GLB는 별도로 보존해야 한다.

## 기존 줄기 색상 통일

사용자 요청으로 새로 변환/생성하는 기존 줄기도 v9 GLB의 기본 색상을 사용한다. `config/stem_v9_colors.json`에 원본 해시와 기존 302개 geom의 대응 색상을 보존했다. 주줄기·잘린 가지·잎별 색상을 적용하며, 여러 재질이 합쳐진 기존 잎은 원본 객체의 삼각형 면적 가중 평균색을 사용한다. GLB의 재질 세부 표현까지 동일한 것은 아니다. 원본 저장 실험은 변경하지 않는다. 물리 속성/조명은 유지한다.

비교: `/root/docker_share/mujoko_debugging_data/20260927_stem_color_matched/index.html`
