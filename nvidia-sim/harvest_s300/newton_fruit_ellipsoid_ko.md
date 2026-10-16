# 분석적 타원체 과실 충돌체 추가 후보

이 후보는 과실 convex mesh10개를 그 외형을 포함하는 분석적 타원체10개로 바꾼다.
원본 모델과 같은 물리가 아니다. 기존 133DOF coarse4 입력을 사용하므로 해당 입력의
좌표 축소·탄성 보정·강체 내부55쌍 충돌 억제도 그대로 상속한다.

설치된 MuJoCo Warp3.15의 `collision_driver.py`와 `io.py`에서 ELLIPSOID를 지원하며,
과실↔mesh/capsule/ellipsoid는 분석적 support mapping을 사용하는 generic convex
접촉 경로다. 정점 수천 개의 support 검색을 피할 수 있지만, mesh와 다른 접촉점 수·방향이
발생할 수 있다. 단순히 더 정확하거나 같은 물리라고 보지 않는다.

## 보존·기하 증거

출력: `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/inputs_analytic_fruit`

`analytic_fruit_proof.json`에 각 과실의 중심·축·회전·포함 검증·오차 표본을 저장했다.
컴파일된 원래 충돌 hull 정점을 모두 포함하도록 SPD affine ellipsoid를 맞췄다.
타원체가 convex이고 모든 원래 정점의 quadratic level이1 미만이므로 원래 convex hull 전체를
포함한다. 실제 컴파일 후 모든 원래 mesh 정점도 다시 검사했으며 최대 level은 약0.999998이다.
최적화10개 모두5~7회 iteration에서 수렴했다.

4096개 방향의 support gap 최대는0.846mm다. 이는 기하 차이를 설명하는 표본 지표이며
전체 외형 오차의 엄밀한 전역 상한이 아니다. 앞선 convex proxy의0.1mm 상한을 이 후보에
잘못 적용하지 않는다. 포함을 위해 바깥으로 확장했고 충돌을 없애기 위한 축소는 하지 않았다.

질량·관성·질량중심·관절·구동·물성·마찰·margin/gap·나머지 충돌체에 대해7504개 보존 검사를
통과했다. 초기 상태·명령·기존 계획 파일은 byte 단위 복사다. MuJoCo compiler는 쓰지 않는
과실 physics mesh asset10개를 제거해810→800개가 되며, 나머지 mesh는 원래 이름과 내용으로
검증했다. 전체 농장/과실의 Isaac 시각 자산은 원본 그대로다.

초기 active contact 최대 침투는0.328mm로 기존 coarse4의0.223mm보다 증가했지만 유지한
1mm 기준을 통과한다. 이 검사는 물리 적분 전 `mj_forward` 검사이며 수확 성공 증거가 아니다.
초기 과실은 모두 부착 상태라 분리 과실의 예외를 적용하지 않는다.

## 검증·재현

CPU 단위 시험3개는 비대칭 hull 포함, 퇴화/비유한 입력 거부, 임의 source mesh scale/refquat 및
geom 회전을 가진 fixture의 컴파일 후 포함·질량/물성 보존·원본 불변·hash 검사를 확인한다.
이 시험은 GPU 성능 증거가 아니다.

```bash
cd /root/farmily_tomato
OPENBLAS_NUM_THREADS=1 nvidia-sim/.newton-venv/bin/python \
  nvidia-sim/harvest_s300/test_newton_fruit_ellipsoid.py
```

새 입력 생성 시 기존 출력을 덮어쓰지 않는다.

```bash
cd /root/farmily_tomato
OPENBLAS_NUM_THREADS=1 nvidia-sim/.newton-venv/bin/python \
  nvidia-sim/harvest_s300/newton_fruit_ellipsoid.py \
  /root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/inputs_coarse4 \
  /root/docker_share/mujoko_debugging_data/NEW_analytic_fruit
```

실행 전 proof의 `initial_active_1mm_passed`를 확인한다. 이번 실험은 이를 통과한 입력에서
기존 canonical GPU 실행기1회만 시험하며, 별도 완화·추가 후보는 만들지 않는다.
`training_eligible=false`, `harvest_success=null`을 유지한다.

## 단1회 전체 GPU screen 결과

실행 결과는 `/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/trial_analytic_fruit`에
있다. 요약·단계별 시간·원시 상태·파단 이벤트·원래 명령을 기존 형식 그대로 저장했다.
다른 후보와의 상세 비교는 같은 결과 루트의 `analytic_fruit_assessment.json`에 있다.

| 모델 | 전체 RTF | 접근 | 닫기 | 재닫기 | 마지막 벨트 | 활성 최대 침투 | 전체 접촉력 최대 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 원본385DOF |1.556|0.873|0.499|0.652|0.378|0.794mm|18.445N|
| coarse4 원본 과실 mesh |2.436|1.358|0.769|1.057|0.596|0.658mm|13.989N|
| coarse4+0.1mm proxy |2.503|1.396|0.808|1.064|0.621|0.657mm|15.157N|
| coarse4+타원체 |2.500|1.471|0.936|0.989|0.650|0.705mm|49.866N|

모두 같은 기존 명령의13,806스텝을 완주했고 각각6개 연결이 해제됐다. 이것은 과실 수거·송이 전체
수확 성공을 뜻하지 않는다. 타원체의 GPU 실제 시간은82.793초, loop33.120초다. cold setup64.316초에는
새 접촉 kernel 컴파일이 포함되며 clean loop RTF와 구분한다. 마지막 벨트는 여전히 실시간보다 느리다.

전체 접촉력 최대는 **분리 과실을 포함한 모든 접촉 norm의 최대**다. 타원체의49.866N은55.1463초/close
구간이며 그 순간 활성 침투0.0992mm, 분리 과실 침투0.7700mm, 꼭지 연결력 최대0.192N이었다.
저장된 global metric은 최대 힘의 접촉 pair를 식별하지 않으므로 이것을 목표 과실 하중·손상으로
단정하지 않는다. 반대로1mm 통과나 연결 해제 수가 같다는 이유로 힘 차이를 숨기지도 않는다.

분리 ID는02/03/04/05/07/08로 같지만 시점은 다르다. 특히07번은proxy에서53.9145초/첫 닫기에
분리됐고, 타원체에서는63.1519초/재닫기에 분리돼 약9.24초 늦었다. 접촉 동작이 같은 후보가 아니다.

**0.1mm proxy 권장안을 유지한다.** 타원체는 전체 시간 이득이 사실상 없고, 형상 차이가 더 크며,
힘과 분리 시점도 바뀌었다. 이 후보의 반복 재현성·GUI 일치는 시험하지 않았으며 추가 시험·추가 후보는
진행하지 않았다. 새 수확 알고리즘·RL·관통 기준 변경은 없다.
