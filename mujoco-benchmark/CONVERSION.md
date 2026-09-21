# 원본 식물 → MuJoCo 변환 기준

Isaac 원본 코드의 `full`을 기준으로 한다. `light`, `ultralight`를 원본으로
취급하지 않는다. 기준 커밋은 추출 JSON에 기록하며 기존 USD·Isaac 코드는 수정하지 않는다.
추출 프로그램이 실제 구동한 PhysX의 질량·관성·COM을 읽는다. 길이·반경·좌표·관절별
수치는 `assets/reference/reference.json`에 모두 저장한다.

| Isaac 구성 | MuJoCo 구성 | 유지한 값 | 차이/범위 |
|---|---|---|---|
| 주줄기 | 강체 16개, 기저 고정 | K=100 N·m/rad, D=3 N·m·s/rad | 분절 수 유지 |
| 송이 연결 가지 | 강체 5개 | K=25, D=1 | 분절 수 유지 |
| 송이 중심 줄기 | 강체 14개 | K=2, D=0.15 | 분절 수 유지 |
| 근위 꼭지 11개 | 각각 강체 3개 | K=0.25, D=0.02 | 목표·비목표 모두 유지 |
| 말단 attachment | 강체 11개 | 각 0.0005 kg, 관성 1e-9 kg·m² | 말단 힌지 유지 |
| 탄성 관절 | 같은 위치의 XYZ hinge 3개 | 총 234 DOF, 각 ±45°, armature 0.0005 kg·m² | 원본 구면/D6 좌표와 큰 회전에서 동일하지 않음 |
| 드라이브 상한 | passive spring/damper | 원본 maxForce 100은 추출 보존 | MuJoCo passive force에는 동일 상한을 적용하지 않음. 큰 충격 동등성 미보장 |
| 줄기 관성 | 명시적 inertial | 원본 native mass/COM/principal inertia | geometry에서 재계산하지 않음 |
| 열매 11개 | free body + weld 11개 | 원본 질량·초기 자세·충돌 sphere | 동일한 66 자유도와 부착 제약. weld의 수치적 강성은 엔진별 차이 |
| 원본 파단 조인트 | Phase 1에서 고정 weld | 원본 force/torque는 추출 JSON에 보존 | **양쪽에서 파단 비활성화**. 수확 성공 시험 아님 |
| 줄기 충돌 | capsule 68개 | 길이·반경·배치 | 근사 축소 없음 |
| 잎/잘린 가지 | convex mesh 286개 | 원본 모든 입력 정점 | 각 엔진 convex hull cooking은 다름 |
| 열매/말단 충돌 | sphere 11개 + capsule 11개 | 원본 반경·위치 | 총 식물 충돌체 376개 |
| 고리 | capsule 32개 + rail 2개 + proximal hull | 반경 27.5mm, 와이어 반경 1mm, 기존 CAD-derived 좌표 | 전체 고리를 하나의 convex hull로 만들지 않음 |
| 고리 제어 | world mocap / PhysX kinematic | 같은 위치+회전 보간 궤적 | 양쪽 로봇 암 제거. MuJoCo mocap의 상대 접촉 속도는 PhysX kinematic target과 다름 |
| 중력/프리로드 | 중력 + 고정 generalized torque | (0,0,-9.81), 초기 하중으로 계산 | 실시간 중력 보상이 아님. 좌표 차이로 엔진별 초기 Jacobian 사용 |
| 마찰 | sliding friction 0.4 | 원본 dynamic 0.4 | 원본 static 0.5를 별도로 재현할 수 없음 |
| 반발/접촉 | MuJoCo solref/solimp | 원본 restitution=0 참조 | 숫자 일대일 변환 불가. solref=.002,1 / solimp=.99,.999,.001 명시 |
| 접촉 여유 | margin=.0005, gap=.0005 | 원본 contact offset .5mm, rest offset 0 참조 | MuJoCo는 gap 이내 비활성 후보; PhysX speculative 접촉과 동등하지 않음 |
| 자기 충돌 제외 | bit masks + body exclude | 잎끼리 제외, 관절 연결·명시적 제외 | 미변환 필터가 있으면 변환 중단 |
| 시각화 | 기존 열매/꽃받침 mesh 99개 + 줄기/잎 collision 형상 | 물리 형상 우선 | 원본 줄기 skin/세부 잎 shading/털은 재현하지 않음 |

질량 합계는 식물 articulation과 열매를 합쳐 약 **0.720472kg**, 식물 강체 90개다.
MuJoCo world와 kinematic hook까지 포함하면 `nbody=92`, `nv=300`이다.
원본도 탄성 DOF 234개와 별도 열매 free-body DOF 66개를 갖는다.

## 계산 조건

- Isaac: CPU PhysX, PGS, 120Hz, 위치 64회/속도 4회, CCD 활성화.
- MuJoCo: CPU, `implicitfast`, Newton, 최대 64회, sparse Jacobian,
  elliptic friction cone. 60/120/240/480Hz와 Euler 비교가 실행 가능하다.
- 반복 수가 같다고 두 solver의 정확도/작업량이 같다는 의미는 아니다.
- MuJoCo의 `nativeccd`는 convex collision 알고리즘 명칭이다.
  PhysX의 시간축 continuous collision detection과 혼동하지 않는다.
- 고리 궤적은 기존 기록의 실제 joint 값으로 FK를 계산한다. 경로를 새로 계획하지 않는다.
  이전에 중단된 기록은 그 지점까지의 prefix로 보존하며 뒷부분을 성공 동작으로 만들지 않는다.
- MuJoCo mocap은 동역학 자유도가 없어 입력 위치를 바꿔도 일반 동적 강체의 속도로
  취급되지 않는다. PhysX kinematic target은 이동 속도를 접촉에 반영한다. 따라서
  **동일 궤적이어도 접촉 마찰·감쇠까지 완전히 같은 경계 조건은 아니다.** 이번은
  요청에 허용된 mocap 기반 1차 비교이며, 정밀 동등성을 주장하려면 후속으로
  양쪽의 동적 고리+동일 구동기 비교가 필요하다. 이번 속도로 이식을 확정하지 않는다.
- 기록 경로 외에 miss/fruit/pedicel/stem push/entry-lift/큰 밀림의 6개 결정적 fixture를 추가한다.
  모든 엔진이 동일한 fixture를 사용하며, 학습이나 랜덤 탐색은 하지 않는다.
- 관통은 기존 0.5mm 기준을 유지한다. capsule/sphere 분석만으로 mesh까지 안전하다고
  보장하지 않는다. 스텝 사이 보간 충돌은 `tunneling_suspect`로만 보고한다.

## 공식 자료

- [MuJoCo integration 및 collision](https://mujoco.readthedocs.io/en/stable/computation/)
- [MuJoCo joint, weld, mocap, solver XML](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo contact parameter 모델링](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [mocap과 동적 강체 접촉의 차이](https://mujoco.readthedocs.io/en/3.2.3/modeling.html#mo-cap-bodies)

이 자료를 근거로 implicitfast를 포함하고 visual/collision 및 접촉 파라미터를 분리했다.
이식 모델의 동등성이나 실물 타당성을 문서만으로 보장하지 않는다.

## 파단을 추가할 때의 후속 설계

추출된 원본 열매 조인트 11개의 임계값은 각각 force **3N**, torque **0.08N·m**다.
이번 standalone USD에서는 두 값을 무한대로 올리고, MuJoCo weld도 항상 활성화한다.
즉 원본 파단값을 잃어버린 것이 아니라 1차 비교에서 명시적으로 끈 것이다.

후속 원본 모델에서는 각 weld의 제약 반력을 해당 조인트 좌표계의 힘/토크로 환산한 뒤
원본 기준을 넘으면 `data.eq_active[weld_id] = 0`으로 해제하는 방식을 검토할 수 있다.
`efc_force`의 요소를 바로 N/N·m로 간주하면 안 되며 Jacobian/제약 종류/회전 스케일을
확인해야 한다. 이 반력 환산과 파단 시점 동등성은 아직 구현·검증하지 않았다.
reset은 이미 equality 활성 상태까지 초기화한다. Optimized 모델은 free joint와 weld를
제거했으므로 이 방식으로 파단을 추가할 수 없고 원본 모델 구조가 필요하다.
