# Newton 독립 시험5·7: 연결 표현과 GPU 실행 최적화

모든 표는 원본 **385DOF**·동일 장면·같은82.795초 수확 명령을 사용한다. 접촉400Hz/이동60Hz/활성1mm는 이번 개별 대조의 출발점이며 프로젝트 전체의 변경 불가 조건이 아니다. 다른 시험의 개선을 누적하지 않았다.133DOF/proxy 결과를 이 표에 섞지 않는다.

기존 `run_s300_newton.sh gpu`와 `benchmark_solver_compare.py`를 재사용했다. 물리 실행은 실제 Newton→MuJoCo Warp CUDA이며 CPU는 별도 반응/산술 대조뿐이다. 공통 GPU lock으로 각 실행을 직렬화했다. 모든 결과는 `training_eligible=false`, `harvest_success=null`이다. 연결 해제 수는 수거 성공이나 송이 전체 수확 성공이 아니다.

## 개별 전체 실행

| 변경 | 결과 | 전체RTF | 닫기 | 마지막 벨트 | 활성 최대겹침mm | 연결 해제 |
|---|---|---:|---:|---:|---:|---:|
| 공통 원본385DOF | 완주 | 1.556 | 0.499 | 0.378 | 0.794 | 6 |
| CONNECT direct 형식 대조 | 완주 | 1.497 | 0.491 | 0.362 | 0.600 | 5 |
| CONNECT 위치 stiffness 유지·감쇠2배 | 완주 | 1.547 | 0.484 | 0.373 | 0.773 | 6 |
| AABB filter만 추가 | 완주 | 1.560 | 0.499 | 0.380 | 0.794 | 6 |
| 동일 Hessian factor cache만 추가 | 완주 | 1.544 | 0.487 | 0.361 | 0.794 | 6 |
| graph batch10→20만 변경 | 완주 | 1.563 | 0.500 | 0.382 | 0.794 | 6 |
| 접촉/broadphase capacity128→64 | solver_capacity_overflow | — | 0.656 | — | 0.290 | 0 |
| constraint capacity256→128 | 완주 | 1.561 | 0.501 | 0.381 | 0.794 | 6 |

capacity64의 부분 실행RTF는 표에서 제외했다. 나머지 전체 실행은13,806physics step을 기록했다. 각 후보는1회 선별 시험으로, 미세한 속도 차이를 통계적으로 확립한 개선이라고 주장하지 않는다.

## 시험5의 범위와 실제 연결 구조

원본 과실은 고정 weld가 아니다. **열매 free6DOF + 위치 CONNECT3행 + 별도3축 angular spring/상대 damping/1° deadband**다. 위치 연결은 유지하면서 회전하고, 기존3N 또는0.008Nm 조건으로 CONNECT가 해제된다.

이번 구현 범위는 CONNECT의 reference coefficient 표현 대조와 위치 stiffness를 유지한 감쇠2배 시험이다. 질량·관성·충돌 형상·위상·각도 spring·deadband·분리 임계값은 그대로다. 준비 후 모델의 모든 ndarray를 대조해 `eq_solref`만 달라졌음을 확인한다.

ball joint로 교체한 실험은 하지 않았다. ball은 부착 상태 자유도를 줄이지만 분리 뒤에는 free motion이 필요하므로, 단순 교체로는 기존의 실제 분리를 유지할 수 없다. DOF/위상 교체와 실행기 지원까지 필요한 별도 구현을 이번 작은 시험에 완료했다고 주장하지 않는다.

### DISCRETE를 반영한 계수 검증과 수정 이력

현재 source integrator는 enum4 **DISCRETE**다. 이 모드의 CONNECT는 positive `solref=(.002,1)`에 continuous integrator의 `2*dt` clamp를 적용하지 않는다. 따라서 위치 reference coefficient는 `1/.002²=250000`, damping coefficient는 `2/.002=1000`이며 direct 형식은 `(-250000,-1000)`이다. 감쇠2배는 `(-250000,-2000)`이다. 이는 정규화된 reference acceleration 계수이며 실물 N/m stiffness로 해석하지 않는다.

첫 준비에서 continuous 방식의 clamp를 잘못 가정해 `(-40000,-400)`을 사용했다. 두 전체 실행을 모두 보존하고 **의도한 동일 stiffness 시험에서 제외**했다. 해당 원자료를 성공 후보로 재명명하지 않았다. `attachment_first_variants_invalidated.json`에 정확한 원인을 남겼다. 최초 입력복사에서 보존 proof 파일을 빠뜨린 준비 오류도 물리 실행 이전 오류로 구분했다.

수정 후 CPU 계수 검사4개와 실제 CUDA 검사3개를 실행했다. CUDA 검사는 설치된 native `_contact_kbimp`를 호출하고 실제 서버 MJB를 읽는다. source K/B는 약250049.969/1000.099976, direct는250050.016/1000.100037이다(dmax 포함). 최대 상대 차이1.875e-7은 사전에 정한 float32 산술 대조 범위2e-7 안이다.60/120/400Hz 모두 CONNECT 계수가 같음을 확인했다. 접촉 행의 별도 DISCRETE REFSAFE 동작까지 이 결과로 대체하지 않는다.

CPU 동일 하중의 idle·밀기·당기기·비틀기4개에서는 수정 direct와 원본의 최대 위치 차이가4.69e-15m, normalized quaternion 자세 차이는2.76e-11°였다. 감쇠2배는 최대1.27µm/0.00192° 차이였다. 연결 사이트 간격도 별도 기록했다. 이것은 plant 반응 대조다. 이 idle fixture에는 원본에서도 Tool_R↔lift platform26.835mm 초기 접촉이 있어 whole-robot validity 증거로 사용하지 않는다. 실제 수확의 활성 접촉 유효성은 위 GPU 전체 실행에서 따로 확인했다.

수정 direct의 CPU 계수/반응은 같아도 GPU 전체 접촉 궤적까지 같지는 않았다. float32 계수 경로의 작은 차이 뒤 연결 해제 결과가 바뀌었다. 이를 동일 궤적의 실행 최적화로 승격하지 않는다.

## 시험7의 결과 해석

AABB, graph batch20, constraint capacity128의 속도 차이는 모두 기준 대비0.5% 미만인 단일 선별 결과다. 유의한 이득으로 판단하거나 새 기본값으로 바꾸지 않는다. cache는 전체가 오히려 느렸으며51,176solve 중8,312회(16.24%) factor를 재사용했다.

유효하게 완주한 GPU 실행 설정4개는 **전체 상태·접촉/힘·분리 이벤트가 원본과 정확히 일치**했다. 기존 비교기의 strict metadata 판정은 의도적으로 바꾼 AABB/cache/batch/capacity까지 같아야 하므로 false가 될 수 있다. 원본 비교JSON을 보존하고 실제 배열·전체 coverage·의도된 설정 차이를 나란히 기록했다. tolerance를 늘려 일치시킨 것이 아니다. graph20의 capture call 수44→84도 실제 물리 차이와 구분한다.

capacity64는 실제 접촉점이 적어도 충분하지 않았다.53.27초,6,076step에서 `BROADPHASE` overflow bit4가 발생했다. 후보 pair65개가64개 버퍼를 넘었으며 당시 실제 접촉은12개였다. 원본 최대 접촉42개만으로 broadphase 용량을 정하면 안 된다. 이 실행은 용량 실패이며 수확 불가능이나 물리 관통 실패로 분류하지 않는다.

이전 MathDx, FP64, equilibration, blocked factor, sparse factor, cache/AABB/profile 혼합 시험도 조사했다. 일부는60step 연결 시험 또는 이전 최적화 조합이므로 지금의 독립 전체 실행과 같은 성능표에 넣지 않았다. 새 GPU 옵션을 누락했다고 가정하고 똑같은 시험을 반복하지 않았다.

## 저장된120Hz 실행의 추가 읽기 전용 검토

`low_hz_attachment_fidelity_audit.json`은 새 physics 없이 원본400/120Hz의 저장 q/v/pose/force를 검토한 자료다. 모든 열매의 eq 비활성화가 첫3N/0.008Nm 초과 시점과 정확히 맞으며 이벤트 force/torque도 원자료와 같다. 저장 GPU q/v에서 CPU native kinematics로 angular law를 다시 계산한 오차는 최대4.92e-9Nm다. 저장하지 않은 전체 efc row force까지 독립 재계산했다고 주장하지 않는다.

120Hz의7개 분리는 모두 torque 조건이다. 일부는 축 회전보다 휘어짐이 크다. 예를 들어 Fruit05는 twist 약−0.77°/swing16.0°, Fruit07은−1.02°/16.17°이고 후자는0.01136Nm까지 한 step에서 넘었다. 원본400Hz에도 휘어짐/damping 중심 분리가 있다. 따라서400Hz를 실물 정답으로,120Hz의7개를 무조건 더 좋은 수확으로 각각 해석하지 않는다. 이 결과는 현재 분리 proxy가 혼합 angular load이며 시간 간격에 따라 분리 양상이 바뀐다는 증거다. 실제 안전 한계·손상 기준은 미보정이다.

## 원자료와 검증

- 원자료: `/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests`
- compact Git evidence: [s300_newton_attachment_gpu_independent.json](../validation/s300_newton_attachment_gpu_independent.json)
- 전체 비교/입력/명령/상태 hash와 invocation: 각 case 폴더 및 `attachment_gpu_independent_evidence.json`
- CPU host4tests: `attachment_variant_tests_v2.log`
- 실제 CUDA3tests: `attachment_gpu_coefficients_tests.log`
- 준비 계수 오류 보존: `attachment_first_variants_invalidated.json`
- GPU별 전체 배열 비교: `gpu_*_original385_complete_comparison.json`
- source joint4반응 재사용은 기존 `response_v2_baseline/leaves_fixed/coarse2/coarse4`와 별도로 해석한다. 네 MJB hash가 현재 입력과 모두 맞는지 확인했다. 오래된 `response_*`의 pre-step pose 자료는 사용하지 않는다.

이 작은 집단에서 GUI/N환경 최종 자격 시험이나 새 MP4를 생성하지 않았다. 해당 검증과 실제 상태 영상은 전체 독립 시험 후 선택한 조합에 대해 수행한다. 접촉 단계 실시간 목표나 실물 정확도를 이 보고서만으로 완료 처리하지 않는다.
