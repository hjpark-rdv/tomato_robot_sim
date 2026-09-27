# 환경 사전검사 실제 서버 검증 결과

작성: 2026-09-27. 검증 대상 코드 기준은 `38cdb6a`이며, 실제 자산은
`rdv@192.168.222.27`의 `humble_x64_env` 안 `/root/farmily_tomato` 및
`/root/docker_share/mujoko_debugging_data`를 사용했다.

이 문서는 실제 서버에서 생성된 `ENVIRONMENT_PREFLIGHT_PRO_FEEDBACK.zip`의
`SERVER_REVIEW_KO.md`, `results.json`, `fresh_success_diagnostic.json`,
렌더 이미지와 재현 스크립트를 검토한 뒤 핵심 결과와 해석을 저장한다.

## 확인된 사실

- 실제 온실 모델의 알려진 거터/주변 주줄기 문제 경로를 사전검사가 실행 전에 잡았다.
- 실제 gate 연결에서 `blocked`와 `inconclusive`는 rollout을 호출하지 않았고,
  `physics_executed=false`, `training_eligible=false`로 저장됐다.
- 검증한 기존 7경로는 모두 첫 기하 위반에서 `blocked`였다.
- 기존 중심 진입 성공 기록도 사전검사에서 막혔다. 따라서 현재 검사는 안전 측면에서는
  보수적이지만, 첫 위반 하나만으로는 이후의 다른 장애물을 알 수 없다.
- 운영 장면의 주변 송이 106개와 잎은 여전히 시각 전용이다. 주변 주줄기 1,088개와
  거터 18개만 추가 충돌 형상으로 확인됐다.

## 기존 7경로의 첫 위반

| 사례 | 기존 결과 | 첫 위반 s | 단계 | 대상 쌍 | 거리 mm |
|---|---|---:|---|---|---:|
| original02 | partial_center_entry | 15.4750 | insert | g385 / glb_col_TRUSS_Rachis_05 | -0.1781 |
| visual02 | partial_center_entry | 15.2389 | insert | g390 / glb_col_TRUSS_Rachis_05 | -0.0260 |
| gutter02 | partial_center_entry | 15.2389 | insert | g390 / glb_col_TRUSS_Rachis_05 | -0.0260 |
| gutter09 | miss | 7.6833 | preapproach | g410 / gutter_collision_GutterFoldedLip_001 | -0.4116 |
| stem01 | invalid_physics | 4.1333 | preapproach | g410 / neighbor_stem_collision_18_05 | -0.3220 |
| stem02 | invalid_physics | 4.4333 | preapproach | g410 / neighbor_stem_collision_18_05 | -0.4099 |
| stem09 | invalid_physics | 5.3833 | preapproach | g410 / neighbor_stem_collision_18_05 | -0.1827 |

g385/g390/g410은 모두 Hook 바디의 충돌 형상이다. 표는 **최초 위반**만 뜻한다.

## 명목 계획과 실제 저장 상태의 차이

성공 기록 original02/visual02/gutter02에서는 초기 식물을 고정한 명목 command 자세가
target plant의 `TRUSS_Rachis_05`와 미세하게 겹쳤지만, 저장된 실제 qpos에서는 같은
쌍이 0.62~0.87 mm 이상 떨어져 있었다. 별도 fresh replay한 gutter02에서도 같은 쌍의
최소 거리는 약 0.866 mm였고 물리 유효성 기준을 통과했다.

반면 stem01/02/09는 실제 저장 상태에서도 나중에 주변 주줄기와 겹침이 나타났고
기존 결과도 `invalid_physics`였다.

이 차이를 이유로 clearance를 줄이거나 target plant 전체를 허용하지 않는다.
계획 command, 실제 actuator tracking, 식물 변형, MuJoCo contact margin/constraint,
실제 접촉력은 별도 신호로 다뤄야 한다. 특히 계획이 고정 장애물을 관통하지만 추종
오차 때문에 실제 qpos가 비켜갔다는 사실은 안전 근거가 아니다.

## 이번 결과로 결정하지 않은 것

- `TRUSS_Rachis_05`가 수확에 의도적으로 허용해야 하는 target pedicel인지,
  금지해야 하는 rachis/branch인지 아직 코드/자산 의미를 확인해야 한다.
- 주변 과실/송이와 잎까지 무접촉 대상으로 만들지는 아직 확정하지 않았다.
- 전체 길이를 `sampled_clear`로 통과한 경로가 실제 fresh rollout에서도 무접촉인지
  아직 비교하지 못했다.
- 실제 팔꿈치-only 충돌 및 상승 중 비목표 과실 접촉 사례는 아직 서버에서 확보되지 않았다.

## 코드 변경: 첫 위반이 뒤의 장애물을 가리지 않도록 진단 확장

서버 결과에서 가장 직접적인 구현 문제는 **첫 위반에서 즉시 반환하기 때문에**
target plant의 작은 nominal overlap이 이후 거터/주줄기 같은 더 중요한 위반을 가릴 수
있다는 점이다.

따라서 production gate의 빠른 fail-closed 동작은 그대로 두고, 읽기 전용 audit에서만
전체 경로를 계속 검사할 수 있도록 `--all-violations` 옵션을 추가했다.

- 기본 실행/gate: 첫 금지 위반에서 즉시 `blocked` — 기존 동작 유지.
- `audit_environment_preflight.py --all-violations`: 전체 경로를 끝까지 검사하고
  unique robot/environment/phase 위반을 최대 64개 기록.
- 이름 기반 진단 분류 `gutter`, `neighbor_stem`, `glb_plant`, `other`를 추가.
  이 분류는 허용/차단 정책을 바꾸지 않는다.
- 전체 audit가 하나라도 위반을 찾으면 결과는 여전히 `blocked`다.

이 기능의 목적은 정책을 완화하는 것이 아니라, **target plant의 첫 위반 뒤에
실제 고정 장애물 위반이 더 있는지 확인하는 것**이다.

## 다음 실제 서버 검증

기존 7경로를 새 출력 폴더에 다시 읽기 전용 검사한다.

```bash
DISPLAY=:0 ./mujoco-benchmark/.venv/bin/python \
  mujoco-benchmark/scripts/audit_environment_preflight.py "$RUN" \
  --candidate predicted_00000 \
  --policy mujoco-benchmark/config/environment_preflight.example.json \
  --output "$OUT" \
  --all-violations
```

검증 시 다음을 별도로 기록한다.

1. 각 경로의 위반 class와 pair 전체 목록.
2. 성공 기록에서 `glb_plant` 뒤에 gutter/neighbor_stem 위반이 존재하는지.
3. `TRUSS_Rachis_05`의 reference/GLB 의미와 target pedicel 관계.
4. MuJoCo contact 생성 여부와 `mj_contactForce` 기반 실제 접촉력.
5. 주변 송이/과실 충돌 형상을 추가했을 때 초기 배치 겹침 여부와 성능 비용.

정책을 통과시키기 위해 collision geom을 끄거나 허용 범위를 넓히지 않는다.
