# 주변 장애물 충돌 커버리지 보강 (2026-09-28)

## 목적

기존 사용자 접촉 정책은 이미 명확하다.

- 목표 과실: 고리 와이어 접촉 허용
- 목표 꼭지/pedicel: 작업에 필요한 접촉 허용, 후방 와이어 접촉은 별도 걸림 증거 후보
- 목표 송이 자신의 Rachis: 고리 와이어 접촉 허용
- 다른 과실, 다른 송이 Rachis/pedicel/peduncle, main stem, gutter, 로봇 arm/mount: 장애물/비허용 접촉

문제는 정책이 아니라 장면 커버리지였다. `preview_neighbor_plants.py`의 배경 식물/송이는 명시적으로 `contype=0, conaffinity=0`인 시각 전용 형상이며, 기존 보강은 주변 main stem capsule만 물리 장애물로 추가했다. 따라서 밀집 배경 송이의 과실/Rachis/pedicel/peduncle이 보이지만 planner/runtime에는 빈 공간일 수 있었다.

이번 변경은 **기존 접촉 허용 정책을 바꾸지 않고**, scene.json에 이미 배치된 주변 송이를 fixed collision proxy로 추가한다.

## 추가된 도구

`mujoco-benchmark/scripts/add_neighbor_truss_obstacles.py`

각 주변 송이의 원본 GLB와 placement metadata를 사용해 기존 `build_glb_physics.py`와 동일한 authored collision topology에서 다음을 추출한다.

- fruit: 원래 convex collision mesh
- rachis: capsule chain
- pedicel: proximal/distal capsule chain
- peduncle: capsule chain

이들을 배경 송이 위치에 fixed world geom으로 배치한다. 배경 장애물은 target 후보가 아니므로 elasticity를 추가하지 않는다. 기존 active target plant, target contact allowlist, friction/material/robot actuator는 변경하지 않는다.

새 geom 이름은 다음 의미를 보존한다.

- `neighbor_truss_collision_fruit_*`
- `neighbor_truss_collision_rachis_*`
- `neighbor_truss_collision_pedicel_*`
- `neighbor_truss_collision_peduncle_*`

`environment_preflight.py`는 위 네 종류를 진단 class로 별도 집계한다. 이는 allow/gate 의미를 바꾸지 않는다.

## 잎

이 변경에서 **잎은 visual-only로 유지한다.** 현재 소스의 사용자 접촉 정책은 다른 과실/송이/주줄기/거터에 대해서는 명시적이지만, 배경 잎을 hard obstacle로 취급하라는 동일 수준의 근거는 없다. 잎을 강체 장애물로 임의 승격하지 않는다.

## 권장 파이프라인

기존 dense layout을 사용할 때 예:

```bash
# 1) 기존 visual neighbor layout에 주변 main stem obstacle 추가
python mujoco-benchmark/scripts/preview_neighbor_plants.py \
  "$ACTIVE_SCENE" \
  --stem-obstacles-layout "$VISUAL_LAYOUT" \
  --output "$STEM_SCENE"

# 2) scene.json에 기록된 모든 주변 송이의 물리 obstacle 추가
python mujoco-benchmark/scripts/add_neighbor_truss_obstacles.py \
  "$STEM_SCENE" \
  --layout "$VISUAL_LAYOUT" \
  --output "$COVERED_SCENE"

# 3) planner/RL 전에 fail-closed coverage audit
python mujoco-benchmark/scripts/add_neighbor_truss_obstacles.py \
  "$COVERED_SCENE" --audit \
  --require-neighbor-stems \
  --require-gutter
```

활성 물리 장면의 `scene.json`과 visual-neighbor layout이 다른 경우에는 `--layout`을 반드시 명시한다. 도구는 일반 랜덤 물리 장면의 `placements`를 주변 visual 송이로 오인해 중복 collider를 만들지 않도록 fail-closed한다.

거터가 없는 의도적 단순 장면이면 `--require-gutter`를 사용하지 않는다. 주변 stem이 없는 단일 식물 장면이면 `--require-neighbor-stems`를 사용하지 않는다. 요구한 항목이 하나라도 빠지면 audit는 exit code 2를 반환한다.

새 collision을 추가한 뒤에는 반드시 초기 penetration/idle screen을 다시 실행해야 한다. `scene.json`은 기존 validation을 자동으로 유효하다고 재사용하지 않고 `scene_screen_passed=false`로 되돌린다.

## 현재 결과에 대한 해석

`d1fdc7c`의 새 6장면은 원래부터 main stem + 실제 물리 GLB 2송이 + gutter collision으로 생성되어, 그 2송이의 fruit/Rachis/pedicel은 collision을 가진다. 대표 nominal 영상에서 충돌 이후에도 로봇이 송이를 관통해 보이는 것은 명목 command를 끝까지 렌더했기 때문이며 물리 실행 성공이 아니다.

이번 수정이 직접 해결하는 것은 **기존 dense greenhouse의 추가 주변 visual-only 송이**를 planner/physics가 빈 공간으로 보는 문제다. 따라서 이 변경을 적용했다고 `d1fdc7c`의 239개 차단 결과가 자동으로 통과하거나 수확 성공률이 오른다고 주장하지 않는다.

## 다음 단계 gate

아래를 만족하기 전에는 dense scene에서 대규모 motion-family 증액이나 RL 학습을 시작하지 않는다.

1. 주변 main stem + 주변 truss + 필요한 gutter coverage audit 통과
2. 새 collision 포함 초기 penetration/idle screen 통과
3. 대표 경로에서 obstacle class가 기대대로 report되는지 확인
4. 실제 물리 qpos 영상과 nominal-only 영상을 분리

그 뒤 접근 planner와 국소 RL/trajectory 비교를 진행한다.
