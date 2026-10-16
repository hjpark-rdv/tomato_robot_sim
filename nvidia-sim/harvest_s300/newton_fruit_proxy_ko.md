# 열매 convex collision proxy 실험

2026-10-17. 기존 준비 입력을 사용하는 선택형 collision approximation이다. 기존 입력이나
기본 실행 옵션을 덮어쓰지 않는다. 이 파일의 수치는 모델 준비·기하 검증 결과이며,
전체 수확 속도나 성공의 검증은 별도 실행 결과를 사용한다.

기준 입력은 `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/inputs_coarse4`다.
coarse4 자체의 물리 표현 변경은 metadata의 `upstream_physics_model_variant`에 보존한다.
이 추가 실험은 과실10개의 mesh만 변경한다. 다른 collider, 질량·관성, 관절·spring·damping,
equality, friction, margin/gap, actuator, dt와 runtime_dof_length는 source와 정확히 비교한다.
initial_state.npz·commands.npz·sampled_plan.json도 그대로 복사한다. 1mm 물리 중단 기준은
이 도구가 수정하지 않는다.

## 보수적 외형 오차의 증명

기준 충돌체는 MuJoCo의 compiled `mesh_graph`가 지정하는 convex hull이다. 단순 visual
mesh를 기준으로 오차를 주장하지 않는다. 그 hull의 source vertex에서 방향별 극점을
선정한다. hull 내부점 c를 중심으로 선택점 p를 `c+s*(p-c)`로 확장한다.

1. 작은 hull의 모든 면 halfspace에 전체 source mesh vertex를 대입해서 필요한 s를
   계산한다. 따라서 source 전체가 확장 hull 안에 있다.
2. 확장된 각 vertex는 기준 convex hull 안의 원래 선택점 p에서 `(s-1)*|p-c|`만큼만
   이동했다. 모든 vertex의 상한은 `(s-1)*R`이다.
3. 기준 hull과 반경 오차 공의 Minkowski 합도 convex이므로, vertex의 상한은 proxy
   전체의 outward Hausdorff 상한이 된다.
4. MuJoCo가 mesh를 다시 중심 정렬·회전한 뒤 실제 compiled vertex를 body frame에서
   확인한다. ideal proxy vertex와의 최대 오차를 위 상한에 더하고, 실제 compiled hull이
   모든 source vertex를 포함하는지도 다시 검사한다.

방향 수128→256→512→1024를 순서대로 시험한다. 주어진 한도 안에서 증명이 안 되면
오류로 종료한다. 후보 개수나 error limit를 몰래 늘리지 않는다. 명시적인 `--budgets`
옵션은 지원한다. float/compile 여유는 기하 재현성용이며 1mm 물리 판정과 별개다.

새 mesh는 source의 이미 compiled된 mesh-local vertex로 만든다. XML geom의 pose는
source compiled geom의 body-relative pose를 사용한다. MuJoCo의 mesh recentering을 두 번
적용하지 않도록 한 것이며, 틀린 transform은 source containment 및 outward bound 검사에서
실제로 실패한다.

## 준비된 결과

| 항목 | 원본 | 0.1mm 한도 | 0.25mm 한도 |
|---|---:|---:|---:|
| 열매10개 mesh vertex 합 |26,240|9,994|3,058|
| 사용 방향 수 | 해당 없음 | 모두1024 |256 또는512|
| 실제 최대 outward 상한 |0|0.086870mm|0.248048mm|
| compiler body-frame 재현 오차 최대 | 해당 없음 | 약1.08nm | 약1.10nm |
| 비열매 collider·물성·동역학 검사 | 기준 |PASS|PASS|

두 변형의 입력 위치:

- `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/inputs_coarse4_fruit_proxy_01`
- `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/inputs_coarse4_fruit_proxy_025`

각 입력의 `fruit_proxy_proof.json`에 열매별 vertex 수·선택한 방향 수·scale·외형 상한·실제
compiled containment·원본 불변 검사 결과가 있다. 실제 source vertex는 모두 compiled
proxy 안에 있으며, 면까지의 최소 여유는 약10nm다. 0.25mm 후보는 별도로 선언한 더 큰
근사이며, 0.1mm 후보와 동일한 geometry라고 주장하지 않는다.

| 열매 | 원본 vertex | 0.1mm vertex | 상한(mm) | 0.25mm vertex | 상한(mm) |
|---|---:|---:|---:|---:|---:|
|01|2624|994|0.077829|255|0.240114|
|02|2624|1001|0.086870|511|0.133935|
|03|2624|1004|0.076243|254|0.237107|
|04|2624|1002|0.083767|510|0.141068|
|05|2624|998|0.076618|254|0.224104|
|07|2624|996|0.065048|255|0.195164|
|08|2624|1006|0.080293|255|0.248048|
|09|2624|994|0.064223|255|0.201235|
|10|2624|998|0.071048|255|0.213892|
|11|2624|1001|0.073708|254|0.219370|

기하 외형이 조금 커지므로 실제 접촉 시점·manifold·동역학은 달라질 수 있다. 보수적인
외형 포함 증명이 물리 궤적이나 수확 결과의 동등성 증명은 아니다. `training_eligible=false`,
`harvest_success=null`로 준비하며, 기존 executor로 전체 동작을 별도 검증해야 한다.

## 재현

```bash
cd /root/farmily_tomato
nvidia-sim/.newton-venv/bin/python nvidia-sim/harvest_s300/newton_fruit_proxy.py \
  /root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/inputs_coarse4 \
  /root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/new_fruit_proxy_inputs \
  --max-error-m .0001
```

기존 prepared-input 형식과 실행기를 그대로 사용한다.

```bash
./nvidia-sim/run_s300_newton.sh gpu \
  /root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/new_fruit_proxy_inputs
```

`test_newton_fruit_proxy.py`5개가 PASS다. 전체 source vertex 포함/오차 증명, 틀린 compiled
transform 거절, NaN/Inf/퇴화 형상·한도 미충족 거절, 회전·scale·refquat를 가진 source의
실제 MJB 재컴파일, runtime 상수와 질량·관성 보존, shared mesh 오수정·덮어쓰기 거절을
검사한다.

mesh recentering 및 maxhullvert는
[MuJoCo mesh 문서](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh),
compiled convex graph의 layout은
[MuJoCo convex hull 문서](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#convex-hulls)를
확인했다.
