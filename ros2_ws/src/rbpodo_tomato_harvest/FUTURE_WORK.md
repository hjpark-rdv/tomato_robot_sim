# 추후 작업 기록

## 검증된 IK 후보를 이용한 constrained OMPL 가속

- 기록일: 2026-07-24
- 상태: 보류 — IK 후보 방식은 현재 코드에서 제거
- 대상: 수확 모션의 OMPL Pose goal 계획 및 Cartesian 실패 후 OMPL fallback

### 문제

현재 TCP Pose goal을 OMPL에 직접 전달하면 MoveIt goal sampler가 IK 후보를
반복해서 생성한다. 관절 constraint, collision, Pose 오차 조건을 만족하지 못하는
후보가 많을 때 다음 경고와 함께 계획 시간이 길어진다.

```text
More than 80% of the sampled goal states fail to satisfy the constraints
```

현재 적용 중인 아래 조건은 성능 개선 이후에도 유지해야 한다.

- 각 OMPL 단계 시작 자세 기준 관절 범위 `±120°`
- 제한 대상: `base`, `shoulder`, `elbow`, `wrist1`, `wrist2`
- `wrist3`는 `±120°` 제한에서 제외
- 기존 로봇 관절 한계 및 collision 검사
- 기존 planning time, planning attempts, 속도 및 가속도 scaling
- 기존 TCP Pose 허용 오차
  - 위치: `5 mm`
  - 자세: 축별 `0.05 rad`

### 제안하는 계획 순서

1. OMPL에 Pose goal을 바로 전달하기 전에 서로 다른 seed로 IK 후보를 여러 개
   생성한다.
2. IK 요청의 목표 링크는 SRDF 기본 tip에 맡기지 않고 반드시 `tcp`로 명시한다.
3. 다음 조건을 모두 통과한 IK 후보만 남긴다.
   - 로봇 관절 한계
   - 단계 시작 자세 기준 `±120°` 관절 constraint
   - collision-free 상태
   - TCP 위치 및 자세 허용 오차
4. 유효 후보를 현재 관절 자세와의 거리로 정렬한다.
   - continuous joint는 `2π` wrapping을 고려한다.
   - 불필요한 관절 회전을 줄이도록 관절별 가중치를 적용할 수 있다.
5. 현재 자세와 가장 가까운 후보를 joint goal로 선택한다.
6. 선택한 joint goal로 OMPL RRTConnect 계획을 실행한다.
   - 경로 전체에는 기존 시작 자세 기준 `±120°` path constraint를 그대로
     적용한다.
   - collision 검사도 그대로 유지한다.
7. 첫 후보가 실패하면 필요에 따라 두 번째 또는 세 번째 후보까지 시도한다.
8. 사전 계산된 joint goal 방식이 모두 실패하면 기존 Pose goal 방식으로
   fallback한다.

### 사용할 MoveIt 인터페이스

- `/compute_ik` (`moveit_msgs/srv/GetPositionIK`)
  - 서로 다른 seed를 사용해 복수 IK branch 생성
  - `ik_link_name: tcp`
  - `avoid_collisions: true`
- `/check_state_validity` (`moveit_msgs/srv/GetStateValidity`)
  - collision 및 constraint 최종 확인
- `/compute_fk` (`moveit_msgs/srv/GetPositionFK`)
  - 후보 관절값의 실제 TCP Pose 오차 확인
- `/move_action`
  - 검증된 joint goal을 기존 constrained OMPL 요청으로 계획

### 시간 제한 주의사항

후보마다 기존 planning time 전체를 새로 부여하면 최악의 경우 오히려 더 오래
걸릴 수 있다. 다음 중 하나를 적용한다.

- 후보 전체가 하나의 planning time 예산을 공유
- 가장 가까운 후보부터 최대 2~3개까지만 시도
- IK 후보 생성 시간과 OMPL 계획 시간을 별도로 측정하여 결과 로그에 기록

기존 Pose goal fallback까지 실행되는 최악의 경우에는 총 시간이 증가할 수
있으므로 fallback 진입 조건과 남은 시간 예산을 명확히 정의해야 한다.

### Trajectory 품질 관련 판단

동일한 constraint와 collision 검사를 유지할 수는 있지만 기존 Pose goal 방식과
완전히 동일한 trajectory를 보장할 수는 없다. OMPL의 랜덤성과 선택된 IK branch가
달라질 수 있기 때문이다.

다만 현재 자세와 가까운 IK branch를 우선 선택하면 다음 효과를 기대할 수 있다.

- 팔이 과도하게 말리는 동작 감소
- 불필요한 관절 회전 감소
- goal sampler의 반복 실패 감소
- 계획 시간 단축

가장 가까운 IK가 항상 가장 계획하기 쉬운 경로는 아니므로 단일 후보 고정보다는
상위 후보 2~3개와 기존 Pose goal fallback을 유지하는 것이 안전하다.

### 구현 위치

주요 수정 대상:

- `rbpodo_tomato_harvest/harvest_planner.py`
  - `_plan_pose_target()`
  - `_plan_cartesian_with_ompl_fallback()`
  - IK 후보 생성, 필터링, 점수 계산용 신규 함수
- 자동 테스트 결과
  - IK 후보 생성 수
  - 유효 후보 수
  - 선택된 후보의 관절 거리
  - IK 준비 시간
  - joint-goal OMPL 시간
  - 기존 Pose goal fallback 사용 여부

### 완료 조건

- 기존 `±120°` constraint와 collision 검사를 위반하지 않는다.
- TCP 목표 Pose 오차가 기존 허용 범위 안에 있다.
- 기존 Pose goal 방식과 비교해 성공률이 낮아지지 않는다.
- 대표 실패 케이스에서 평균 및 95백분위 planning 시간이 감소한다.
- 과도한 관절 말림이나 불필요한 회전이 증가하지 않는다.
- 모든 사전 IK 후보 방식이 실패하면 기존 Pose goal 방식이 정상 동작한다.

### 현재 결정 (2026-07-24)

1차 적용에서 계획 시간은 줄었지만, 가까운 IK endpoint만 선택하고 실제 OMPL
trajectory 품질을 평가하지 않아 관절이 크게 우회하는 경로가 발생했다.
따라서 IK 후보 기반 계획은 현재 코드에서 제거하고 기존 Pose goal +
시작 자세 중심 `±120°` constrained OMPL 방식으로 복원했다.

당분간은 `rbpodo_moveit_config/config/ompl_planning.yaml`에서
`mainpulation` 그룹의 RRTConnect 설정을 명시하고, 수확 planner의 OMPL Pose
goal 제한 시간을 `2초`, planning attempts를 `2회`로 사용한다. IK 후보 방식은
허용오차 영역 샘플링과 실제 trajectory 품질 평가 기준이 마련된 뒤 다시
검토한다.
