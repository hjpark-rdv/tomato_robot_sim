# USD 기반 주줄기·송이 조립 미리보기

2026-09-23. 기존 수집 모델과 USD는 변경하지 않는다. **이번 구현은 조립 장면 생성·관찰 단계이며 30후보 자동 수집기로 연결된 상태가 아니다.**

## 현재 기준: 사용자 지정 단계로 롤백

사용자 요청으로 `20260923_tipward_surface_attachment` 단계의 생성 동작으로 복원했다. 짧은 시작 가지 기준 회전과 당시 표면 부착 계산을 사용한다. 이후 캡슐 축 순서 수정, 엄격한 접합 거부/재샘플링, 긴 중심가지/열매 중심 각도 변경은 현재 코드에서 되돌렸다. 아래 후속 변경 기록은 과거 이력이며 현재 동작이 아니다. 기존 주줄기·잎과 모든 결과 폴더는 보존한다. 이 단계의 알려진 초기 겹침도 복원되므로 수집 검증 완료로 해석하지 않는다.

기준 결과: `/root/docker_share/mujoko_debugging_data/20260923_tipward_surface_attachment/index.html`.

## 구현

- `scripts/assemble_plant_scene.py`: 온실 USD의 `/World/Plants`에서 송이를 목록화한다. 현재 파일은 54식물 × 3송이 = 162개 부착 사례. 이것을 162종의 독립 형상으로 해석하지 않는다. catalog의 prototype_signature는 참조 조합이며 형상 동등성 판정이 아니다.
- 기존 `models/plant_mujoco_optimized.xml`의 `STEM_MainStem_00` 하위 트리를 복사하고 기존 TRUSS만 제거한다. 주줄기 16개 몸체와 잎·잘린 가지를 포함한 302개 geom, 메시, 관절, 질량, 관성, 색상, 충돌 설정을 유지한다. 주줄기를 새로 만들지 않는다.
- 주줄기 호 길이 방향으로 부착점을 이동하고 원래/새 접선 사이 최소 회전을 적용한다. 추가 yaw는 새 주줄기 접선 축, tilt는 주줄기와 가지 방향에 수직인 축이다. 둘 다 월드 Euler 각이 아니다.
- 송이 전체에 동일한 강체 변환을 적용하여 가지·열매·꽃받침의 상대 배치를 보존한다. 원본 송이의 상대 부착 호 길이를 기존 주줄기에 대응시키고, 선택한 이동량을 적용한다. `--plant`는 송이 공급원을 선택하며 주줄기를 교체하지 않는다.
- 송이 원본 배율은0.5이며 주줄기는 기존 모델의 월드 위치와 회전을 그대로 유지한다.
- 기본 위치 변화 ±80mm, yaw ±10°, tilt ±5°. 이는 보수적인 **미리보기 설정**이지 USD에 기록된 생물학적 허용범위를 추출한 값이 아니다. 원본 생장 규칙 생성기는 아직 발견하지 못했다. CLI는 yaw ±20°, tilt ±10°를 넘기지 못하게 한다.
- `--plant`, `--truss`로 원본 종류 선택. `--shift`, `--yaw`, `--tilt`로 고정값 지정하거나 seed와 범위로 재현 가능한 무작위 장면 생성.
- `scene.xml`, `scene.json`에 실제 MuJoCo 모델·부착 변환·원본 경로·열매 위치 저장. manifest는 seed/실행인자/USD 해시를 보존한다.

## 이번 물리 근사 범위

주줄기는 기존16분절을 보존한다. 새 송이는 지지 가지5, 송이 중심14, 열매별 proximal3/distal1 분절을 사용한다. 주줄기 밑만 고정하고 연결된 분절에 회전 스프링을 둔다. 열매는 원본 메시의 convex hull 충돌과 원본 시각 메시를 사용한다. 기존 주줄기의 잎·잘린 가지 메시와 기존 충돌 비활성 정책을 보존한다. 새 송이의 가지는 캡슐이며 송이 털·빈 꼭지는 생략했다. 과실 질량25g, 스프링/감쇠는 기존 코드 값을 참고한 **실험 설정**이다. 기존 경량 모델의 동역학과 같다고 보증하지 않는다. 로봇·파단·리프트·카메라 수집은 이 미리보기에 포함하지 않는다.

## 실행

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/assemble_plant_scene.py \
  --count 6 --seed 0 --render \
  --output /root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_attachment_preview
```

특정 송이와 부착 설정:

```bash
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/assemble_plant_scene.py \
  --plant TOMATO_STEM_L_00 --truss Truss_02 --count 1 \
  --shift 0.05 --yaw 8 --tilt 0 --render \
  --output /root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_attachment_fixed
```

실제 생성된 장면의 마우스 조작 가능한 화면:

```bash
DISPLAY=:0 ./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/view_plant_scene.py \
  /root/docker_share/mujoko_debugging_data/20260923_preserved_stem_leaves/scene_0001
```

기본은 정지 형태 관찰이다. `--simulate`를 붙이면 초기 중력 보상 토크를 유지한 240Hz 물리를 실시간 속도로 실행한다. 초기 겹침이 있는 미검증 모델이므로 이를 정상 학습 물리로 해석하지 않는다.

## 기존 주줄기 보존 수정

- 최신 결과: `/root/docker_share/mujoko_debugging_data/20260923_preserved_stem_leaves/index.html`. 세 송이 배치와 전체 식물/송이 확대 미리보기.
- 주줄기 형상·물성 보존과 새 송이 물리 유효성은 별도다. 새 송이를 붙이면 하중과 동역학 응답은 달라진다. 초기 겹침 및 수집 준비 여부는 각 장면의 `validation.json`을 확인한다.

## 이전 재생성 주줄기 샘플 검사 (현재 방식과 구분)

- 결과: `/root/docker_share/mujoko_debugging_data/20260923_attachment_preview_convex/index.html`.
- L_00의3개 송이 배치에서 각2장면, 총6개. 생성/2초 정지 물리/렌더 약15.48초, USD 로딩·목록화 제외.
- 6개 모두2초간 비정상 초기화/발산 없이 실행. 강체107, 자유도282로, 경량화 성능을 검증한 모델은 아니다.
- 지지가지–송이 및 proximal–distal 원본 접합면의 최대 간격 약0.0086mm 미만. 전체 접촉/변형 연결 보증과는 다르다.
- 정지 중 최대 바디 이동0.93~6.44mm. 초기 최대 충돌 겹침1.03~2.82mm. **6개 모두0.5mm 초기 겹침 검사를 통과하지 못했다.**
- 자기 충돌을 전부 끄거나 이웃 열매/가지를 임의로 제외하지 않았다. validation.json에 문제 geom 쌍을 기록했다. 메타데이터 `collection_ready=false`, `candidate_physics_validated=false`.
- 최초 sphere 근사 샘플은 별도 `20260923_attachment_preview`에 보존하며 최종 convex 샘플과 구분한다.
- 중심선 투영·호 길이 복원·회전 거리 보존·범위 검사4개 통과. 두 대표 PNG 직접 확인.

## 다음 연결 작업

원본 시각 메시 자체의 겹침인지 충돌 근사 문제인지 분리하고, 접합부의 정당한 충돌 제외와 충돌 형상을 검증한다. 그 후 기존 로봇/경로 계획 참조 모델·GT target 정보를 장면별로 갱신하고, 송이 높이 기반 리프트 초기화와 관측을 붙인다. 기존 action14/gravity/waypoint 형식은 변경하지 않았다. 이번 출력은 해당 학습 형식의 데이터가 아니므로 기존 학습기로 직접 읽지 않는다. 수천 장면/분할/Depth+mask 비교/REAL 테스트는 후속 단계다.

## 표면 부착과 허용 방향

주줄기 캡슐의 내부 축 부호 대신 원본 `fromto` 시작→끝 순서를 사용한다. 가지 시작 캡슐의 둥근 끝이 주줄기 표면에 닿도록 전체 송이를 강체 이동한다. 실제 컴파일된 MuJoCo 형상에서 접합 간격과 송이 지지 가지5개 대 주줄기16개 캡슐의 부호 있는 거리를 검사한다. 허용 수치 오차는1e-7m이며 연결이 끊기거나 관통하면 장면 생성을 거부하고 `attachment_validation.json`에 기록한다. 잎·열매 등 전체 장면의 충돌 유효성을 보증하는 검사는 아니다.

최종 사용자 기준은 **주줄기 위쪽 끝 방향0°, ±90° 허용**이다. 3차원에서는 국소 주줄기 끝 방향과 시작 가지 방향의 내적이0이상인 반구로 적용한다. 원본 송이가 뿌리 방향이면 전체 송이를 강체 회전하여 위쪽 반구로 옮긴 뒤 yaw/tilt 변화를 적용한다. 최종 시작 가지가90°를 넘으면 거부한다. 열매가 아래로 늘어지는 것 자체를 금지하는 조건은 아니다. `scene.json`에 실제 각도를 저장한다. CLI yaw/tilt는 이0° 기준 각도와 다른 추가 회전값이다.

확정 기준 샘플: `/root/docker_share/mujoko_debugging_data/20260923_tipward_attachment_final/index.html`. 기존 결과는 보존한다.

## 최신 각도 기준: 긴 중심가지

사용자 시각 기준에 맞춰 v4는 부착부의 짧은 지지 가지 대신 원본 `Rachis` 중심선의 시작점→끝점 방향을 사용한다. 부착 위치의 주줄기 위쪽 접선과 이루는3차원 각도가0~90도인지 확인한다. 전체 송이에 동일 회전을 적용하여 내부 모양을 유지하고, 표면 접합/주줄기 비관통 조건은 별도로 검사한다. 이전 v3의32도 등은 짧은 지지 가지 기준이므로 새 각도와 비교하지 않는다.

`--rachis-angle 60`으로 긴 중심가지 각도를 지정한다. 생략시 seed로15~85도에서 샘플링한다. yaw/tilt는 중간 배치 회전이며 최종 각도는 긴 중심가지 기준으로 맞춘다. HTML에 실제 긴 중심가지 각도를 표시한다. 새 샘플9개: `/root/docker_share/mujoko_debugging_data/20260923_rachis_angle_samples_checked/index.html`. 위치와 원본 송이3종을 함께 바꾼다. 실패한 부착은 rejected 폴더에 형상/검사/인자를 보존하고 새 배치를 최대30회 뽑는다. 실패 배치를 정상 샘플로 포함하지 않는다.

새9장면의 컴파일된 Rachis 시작/끝 위치에서 독립 재계산한 각도는18.08~84.23도이며 메타데이터와1e-5도 이내 일치했다. 접합/지지 가지 비관통9개 통과, 부적합2개는 rejected에 보존했다. 2초 정지 물리는9개 발산 없음. 전체 장면 초기 겹침은9개 모두 남아 있으며 정상 학습 데이터가 아니다. `angle_verification.json` 참조. 관련 기하 테스트8개 통과.

## 최종 정정: 부착점→열매 평균 중심(v5)

현재 각도는 주줄기 표면 부착점에서 열매 중심 좌표들의 산술평균을 향하는 방향과 국소 주줄기 위쪽 접선 사이의 각도다. 각 열매 중심은 원본 메시 경계상자 중심을 송이와 함께 강체 변환한다. 이전의 짧은 가지/긴 중심가지 각도는 과거 정의다.

CLI는 `--fruit-center-angle`로 변경했다. 생략시15~85도 방향을 샘플링한 후 표면 접합 보정을 적용하고 실제 최종 각도를 기록하므로 요청값과 실제값은 약간 다를 수 있다. HTML에 실제 각도를 표시한다. 새9개 샘플은 `/root/docker_share/mujoko_debugging_data/20260923_fruit_center_angle_samples/index.html`. 컴파일된 열매 body 위치에서 각도를 독립 재계산하여 메타데이터와1e-5도 이내 일치 확인. 접합/지지 가지 비관통9개 통과, 실패2개는 rejected 폴더 보존. 기존 데이터는 변경하지 않았다.

## 현재 GLB 구현 및 초기 겹침 검사

현재 사용자는 `nvidia-sim/env_usd/tomato_rotate_glb`의5원본과 **GLB Y축0~180° 회전만** 사용하도록 확정했다. 위의 시작가지/열매중심/반구 추정은 이전 시행착오다. `assemble_glb_plant.py --output NEW_DIR --seed 23 --view`로5종을 각각1개씩 기존 주줄기의 구간별 무작위 위치에 붙인다. 원본 크기1, GLB Y-up→MuJoCo Z-up 변환 후 Y축 회전. 아직 시각 전용이다.

검사 명령:

```bash
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/audit_glb_collisions.py   /root/docker_share/mujoko_debugging_data/20260923_multi_glb_y_random   --output /root/docker_share/mujoko_debugging_data/NEW_collision_audit
```

접촉 마스크와 관계없이 convex hull의 명시적 거리를 계산한다. 검사는 초기 상태만 대상으로 하며 메시 삼각형 그대로의 관통 보증이 아니다. 줄기/꽃받침/열매/털의 의도된 연결과 convex 과대근사 때문에 내부 겹침이 많다. 털 포함 여부를 별도 기록한다. 임계값0.5mm에서 서로 다른 송이0쌍, 털 제외 송이–주줄기7쌍(최대3.042mm), 송이–잎/잘린가지1쌍(최대3.201mm). 형상을 바꾸거나 충돌을 추가 제외하지 않았다. 상세 `validation/glb_initial_overlap.json`.
