# Newton 그리퍼 충돌 형상 개별 시험

2026-10-17, `codex/s300-newton-independent-tests-20261017`.

원본 385DOF/접촉400Hz/이동60Hz를 유지하고 그리퍼의 충돌 표현만 바꾼다. 관절 축소, 열매 proxy, 부드러운 접촉, Hz 변경은 함께 적용하지 않았다. 기존 `benchmark_solver_compare.py`와 GPU 실행기 및 공유 GPU lock을 사용한다. `training_eligible=false`, `harvest_success=null`이다.

## 원본과 변경 범위

원본은 `20261016_s300_newton/inputs_render_separated`다. 전체901충돌체/810메시, 그리퍼의 비벨트 본체3개는334메시(`Tool_fixed`92 + `Tool_L`121 + `Tool_R`121), 벨트 본체2개는 메시2개+박스2개다.

1. **안쪽에 완전히 묻힌 충돌체 제거**: 같은 강체의 남겨둔 볼록 충돌체가 작은 충돌체를 완전히 포함하고, 접촉 파라미터도 같을 때만 제거한다. 최소1mm 여유에서는115개(901→786), 최소1µm에서는132개(901→769)를 제거한다. 실제 컴파일된 collision graph의 모든 꼭짓점이 포함 충돌체의 모든 정규화 평면 안쪽에 있는지 검증했다. 충돌체의 외부 합집합은 같지만 깊게 겹칠 때 접촉 manifold/기여 힘과 수치 순서는 달라질 수 있다.
2. **ID를 보존하는 대조군**: 같은115개의 contype/conaffinity만 끄고 원본 geom/mesh ID와 BVH 저장 순서를 유지한다. 남아 있는 외부 충돌 형상은 모두 유지된다. MJB가 실제 물리 모델이며, XML도 같은 활성 접촉 정책을 기술한다. XML의 discardvisual은 꺼진 충돌체의 메타데이터 저장을 생략할 수 있다.
3. **실제로 접촉하는 외부3개 메시의 proxy**: 원본에 부착 중인 Fruit08이 접촉하는 `s300_g0581`, `s300_g0606`, `s300_g0620`만 바꾼다. 기존 `newton_fruit_proxy.py`의 보수적 enclosure/오차 증명을 재사용한다. 벨트 접촉면, 벨트 사이 개방 폭, 나머지 모든 충돌체, 질량/관성/관절/명령/마찰은 유지한다. 지정한 외부 부품 자체는 최대0.1mm 범위에서 바깥쪽으로 커질 수 있다.

외부3개 proxy는 원래 **컴파일된 충돌 graph399꼭짓점→377**이다. CAD의 전체5384메시 꼭짓점을 377개로 줄여 그 비율만큼 런타임이 개선됐다고 계산하지 않는다. 각 compiled outward bound는 g0581=0.009318mm, g0606=0.051015mm, g0620=0.0000548mm다. g0620은80→80으로 런타임 graph 정점 감소가 없다.

원본 body 질량/관성/COM, 관절/DOF/actuator/equality/초기 상태와 남아 있는 모든 충돌체의 파라미터를 수치적으로 정확히 비교한다. 초기 body pose도 정확히 같다. 처음 두 immutable 입력을 만든 뒤, 검증을 all-mesh convex hull에서 **실제 compiled collision graph**로 강화했으며, 같은115/132개 제거가 유효함을 별도 재검증했다. 남은 메시의 vertices/faces/normals/compiled graph도 각각26223/25645개 검사를 통과했다.

## 측정 결과

결과 원자료 루트:
`/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests`

| 경우 | 완주/중단 | 전체 RTF | close | reclose | belt_finish | 분리 조인트 수 | 최대 감시 겹침 |
|---|---|---:|---:|---:|---:|---:|---:|
| 원본400Hz 관찰 기준 |82.795s 완주|1.556|0.499|0.653|0.378|6|0.794mm|
|115개 제거, 기존1mm중단|56.47s 중단|prefix1.962|0.488|미완료|미완료|5|5.293mm|
|132개 제거, 기존1mm중단|56.47s 중단|prefix1.982|0.487|미완료|미완료|5|5.293mm|
|115개 제거, 겹침 관찰 계속|82.795s 완주|1.506|0.490|0.604|0.370|5|5.293mm|
|115개 비활성화, 원본 ID 유지|56.47s 중단|prefix1.967|0.483|미완료|미완료|5|5.293mm|
|외부3개 보수적 proxy|82.795s 완주|1.537|0.499|0.618|0.372|6|0.708mm|

중도 중단의 prefix RTF는 전체 수확 처리량이 아니다. 관찰 계속 실행도 겹침이 사라졌다는 뜻이 아니다. 묻힌 CAD 부품 제거와 외부3개 proxy 모두 전체/접촉 구간 속도가 개선되지 않았다. **이번 조합 후보에 그리퍼 변경을 추가할 근거는 없다.** 외부3개 proxy는 기존1mm 기준에서도 완주하지만 global contact force 최대39.31N(원본18.44N)으로 달라졌다. 이 값에는 분리된 열매가 포함되어 타깃 손상 힘으로 해석하지 않는다. 분리 수가6개로 같아도 경로별 반응이 같다는 뜻은 아니다. Fruit07 분리는 원본54.960s→proxy62.7175s로 약7.76s 늦어졌고, rod 최대 위치 차이17.22mm/두 실행에서 모두 부착 중인 열매 위치 차이15.48mm가 나타났다.

두 compact 수준은 저장된7356스텝 전체 상태가 정확히 같다. 원본과는54.000s까지 qpos/qvel/robot_q/pose가 정확히 같고,54.0025s에 처음 갈라진다. 같은 baseline q에서 CPU collision query를 시행한53.9975/54/54.0025s에는 각각17/20/18개 접촉의 pair 이름/거리/위치가 원본과 정확히 같고 제거한 충돌체의 접촉은0개다. 따라서 잘못된 외부 형상 transform으로 단정할 근거는 없다. 원본 ID/BVH를 유지한 mask 대조군도 compact 두 수준과 저장 상태 전체가 정확히 같으므로 **geom ID compaction만으로 차이가 생겼다는 설명은 배제**한다. CPU 접촉 조회의 일치는 GPU 접촉/제약 force 동등성 증거가 아니다. 외부 합집합이 같아도 중복 collider가 접촉 margin 및 제약 구성에 기여하는 방식은 다를 수 있으며, 실제 분리 결과도 달라졌다.

첫 기준 초과는 **이미54.590s에 분리된 Fruit08↔거터 `farm_structural_13`**이며,56.450s에4.594mm로 나타나고 같은 중단 batch에서5.293mm까지 간다. 직접적인 벨트-부착열매 겹침 중단과 구분한다. 원본 대비 rod 위치 차이 최대58.16mm, 두 실행에서 모두 부착 중인 열매 위치 차이 최대34.98mm가 나타났으므로 원본 반응과 동일하다고 주장하지 않는다. global 최대 contact force에는 분리된 열매도 포함돼 타깃 손상 힘으로 해석할 수 없다.

## 재현

묻힌 부품 제거:
```bash
./nvidia-sim/.newton-venv/bin/python nvidia-sim/harvest_s300/newton_gripper_variant.py "$SRC" "$NEW_OUT" --minimum-clearance-m 0.001
```

ID 유지 대조군:
```bash
./nvidia-sim/.newton-venv/bin/python nvidia-sim/harvest_s300/newton_gripper_variant.py "$SRC" "$NEW_OUT" --minimum-clearance-m 0.001 --keep-disabled-geom-ids
```

외부3개 proxy(묻힌 부품 제거와 독립):
```bash
./nvidia-sim/.newton-venv/bin/python nvidia-sim/harvest_s300/newton_gripper_variant.py "$SRC" "$NEW_OUT" --outer-contact-proxy --max-outward-m 0.0001
```

어떤 prepared parent에서도 원래 body/DOF/명령을 유지하는 `prepare(source, output, ...)`, `prepare_contact_proxy(source, output, ...)`를 호출할 수 있다. 조합은 개별 시험 완료 후 새 output에서 시행한다.

## 테스트와 증거

- `test_newton_gripper_variant.py`:6개 통과. 엄격 포함/경계, 접촉 재질 불일치, 회전·병진, 원본 보존, ID 유지 mask, belt proxy 차단.
- `test_newton_fruit_proxy.py`:7개 통과. 기존 fruit default 회귀, 명시적 이름 선택, 가중 support 선택/전체 hull fallback의 포함·거리 증명, 잘못된 transform/범위/공유 메시 차단.
- `gripper_compiled_graph_reaudit.json`: 원본 collision graph로115/132개 재검증.
- `gripper_retained_collision_graph_verification.json`: 남겨둔 메시의 실제 compiled graph 비교.
- `gripper_first_divergence_cpu_collision_audit.json`: 같은 q의 접촉 pair/거리/위치 조회.
- `gripper_individual_evidence.json`: 입력/모델/상태/SHA, 단계별 속도, 분리 이벤트, 로봇 추종/rod·부착열매 반응 차이.
- `build_gripper_evidence.py`: 저장 결과만 읽는 위 JSON 재생성 도구. 물리를 실행하지 않는다.

이 실험 자체는 별도 동영상을 생성하지 않는다. 통합 보고에서 선택된 실제 상태의 비교 영상을 연결하며, 명목 계획을 실제 physics처럼 제시하지 않는다.

초기 outer3 준비에서 전체 CAD 꼭짓점 포함과0.1mm bound를 만족하지 못한 시도는 물리 실행 전 거절됐고 원자료를 보존했다. 이후 실제 compiled collision graph를 기준으로 명시했다. 가중 support를 쓰지 않은 후보는399→399로 줄지 않아 GPU 실행하지 않았다. 이 둘은 수확 실패나 IK 실패로 집계하지 않는다.
