# 새 정상 시작에서 송이 준비 경로 검사

2026-10-11. 이전 Truss01 시험은 준비 관절 이동 중 외부 Plant34의 잎과 충돌해 멈췄다. 기존 검사는 로봇·그리퍼 자기충돌을 확인했지만, 해당 시험에 rail_move가 없어 농장 환경 검사는 수행하지 않았다. 자기충돌 통과를 농장 장애물 통과로 해석할 수 없다.

`screen_fresh_truss_preparation.py`는 기존 `plan_motion`, `MotionSelfCollision`, `check_plan`, `rail_environment_preflight.audit_rail_transit`를 사용한다. 새 실행기나 물리 모델을 만들지 않는다. 새로운 독립 실험의 정상 초기 상태에서 Truss01을 먼저 선택하고, 기존 joint seed 0~5와 Cartesian 두 방식만 검사한다. 총 8개 경로 제안이며 후보 재생성은 없다.

- 초기 관절은 새 assembly의 `initial_robot_q` 그대로 사용한다. 이전 성공/실패 checkpoint와 native 파단을 가져오지 않는다.
- 기존 highest-first 선택기로 최초 과실 그룹을 고른다. 송이 방향은 기존 XY 주축/부호 규칙을 쓰고 진입 자세는 수평으로 유지한다.
- 8Hz 자기충돌과 준비 구간 환경 검사를 먼저 한다. 통과한 제안만 120Hz로 같은 검사를 반복한다.
- 환경 검사는 첫 approach 전의 모든 준비 단계와 staging까지 포함한다. 정적인 외부 잎도 물리 장애물이며, 이전 rail 검사에서 사용한 ‘모든 잎은 진단만’ 규칙을 준비 구간 통과 근거로 쓰지 않는다.
- 물체의 위치, 충돌체, 물성, 관통 기준과 조인트를 변경하지 않는다. 준비 구간 통과가 실제 추종·접촉·수확 성공을 뜻하지 않는다.
- 모든 계획 실패, 자기충돌 차단, 환경 차단, 미완료 검사를 개별 폴더와 summary.json에 보존한다. 유한 탐색 실패를 절대 수확 불가능으로 기록하지 않는다.

새 자산의 검증과 환경 검사 API가 준비된 뒤 실행한다. 출력 폴더가 이미 있으면 덮어쓰지 않는다.

```bash
cd /root/farmily_tomato
OPENBLAS_NUM_THREADS=1 nvidia-sim/.asset-venv/bin/python -u \
  nvidia-sim/harvest_s300/screen_fresh_truss_preparation.py \
  /root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/assembly_plant22_fresh_near20_truss01 \
  /root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/fresh_truss01_preparation_screen \
  --ready-lift-offset-mm -135 --grasp-depth-mm 65 --grasp-height-offset-mm -1.5
```

이 명령은 물리 시뮬레이션을 실행하지 않는다. 실제 실행에서는 별도로 새 초기 상태의 native 유효성, 실제 0.5mm 관통, 접촉력, 조인트 파단, 실제 추종과 수확 단계를 확인해야 한다. 정지한 과거 invalid terminal을 정상 시작으로 바꾸거나 이전 유효 checkpoint로 되돌아간 결과를 연속 수확으로 집계하지 않는다.

## 실제 서버의 제한된 검사 결과

처음 8개는 자기충돌/2mm 여유 검사 또는 Cartesian 준비 IK에서 모두 차단됐다. 이어서 기존 seed6/7에 손목의 2π 동등 회전을 적용하고 리프트 −135/−170mm를 비교한 6개도 실행 가능한 전체 준비 경로를 찾지 못했다. seed6는 자기충돌은 통과했지만, 직접 관절 연결이 다른 줄기의 잎과 온실 와이어를 가로질렀다. 손목 회전을 짧게 줄여도 팔꿈치가 약 +2.340rad에서 −2.267rad로 이동하는 큰 회전은 남았다.

정확한 끝 자세를 따로 확인했더니, 기존 100mm 후퇴 staging에서 44mm 폭은 통과하지만 사용자가 요구한 완전 개방 96mm에서는 Plant20의 정적인 잎과 겹쳤다. 따라서 그 위치까지 OMPL로 가는 것만으로는 문제를 해결할 수 없다.

`plan_motion(..., approach_distance_m=.10)`을 기본값 그대로 두고, 수평 벨트 모션에서만 50~300mm의 명시적 staging 후퇴를 허용했다. 최종 과실 중심, 진입 yaw/pitch, 완전 개방 96mm는 유지한다. 다른 후퇴 거리는 실제 준비 위치와 Cartesian 진입 경유점을 바꾼다. 기존 기본 command 배열 SHA와 의미 있는 FK/연속성/기계 한계 검사를 포함한 56개 테스트가 통과했다.

- 150/200/250/70mm × 리프트 −135/−170mm, seed6의 8개 끝 자세 검사: 합격 0개. 250mm에서는 환경 충돌이 사라졌지만 그 IK branch는 자기충돌했다.
- 250mm에서 기존 IK seed0~8 × 리프트 −135/−170mm를 끝 자세만 검사: 18개 중 seed1/7의 네 자세가 완전 개방 96mm 자기충돌·환경 검사 모두 통과했다.
- 네 자세의 기존 `prepare_open → approach → 벨트 사이클 → release_open → withdraw`는 120Hz 자기충돌 검사 3,190표본을 모두 통과했다. 차단된 기존 직접 관절 준비/복귀는 이 suffix 검사에서 명시적으로 제외했고, 충돌 회피 연결로 교체해야 한다.

이 결과는 실행할 준비 경로를 확보한 것이 아니라 **충돌 회피 연결이 향할 수 있는 정상 목표 자세 네 개를 확보한 것**이다. 준비 경로 전체의 120Hz 환경 검사와 실제 native 물리 실행은 후속 작업이다. 아직 수확 성공으로 집계하지 않는다.

상세 원자료는 공유 결과 폴더의 `fresh_truss01_preparation_screen`, `fresh_truss01_turn_normalization_screen`, `fresh_truss01_staging_distance_probe`, `fresh_truss01_staging_branch_probe`에 있고, 작은 요약과 SHA는 `validation/fresh_truss01_preparation_20261011.json`에 있다. 검사 중 환경 helper의 디스크 코드가 다른 에이전트 수정으로 바뀐 경우, 원본 보고서의 `source_code_unchanged=false`를 그대로 유지했다. 실제 실행된 환경 helper import SHA와 정확한 코드 바이트를 별도로 보존했으며 결과를 새 코드의 실행으로 바꿔 쓰지 않았다.

250mm staging가 통과하더라도, 250→0mm 완전 개방 진입은 이전의 100mm 차단 위치를 지나간다. 같은 원본 장면에서 실제 계획의 100/50/0mm 경유점을 각각 검사했다. 자기충돌은 모두 없었지만, 외부 Plant20의 `CompoundLeaf01/02`에 해당하는 mesh127/128/131/132만 각각 6/5/9쌍의 단단한 장애물 겹침으로 남았다. 이 세 대표 상태에서는 다른 과실·Rachis·주줄기·거터·온실 구조물의 hard hit는 발견되지 않았다. 다만 경로 전체 검사가 아니라 대표 상태 검사다.

따라서 후속 실험은 이 두 잎 그룹만 기존 유연 D6 잎 모델로 만드는 **별도의 물리 가설**과 충돌 회피 준비 연결을 함께 검증해야 한다. 원본의 정적 잎을 접촉 허용으로 바꾸거나, 겹침을 없다고 해석한 것은 아니다. 모든 원본 실패와 0.5mm 물리 기준은 그대로 유지한다.

## 두 잎의 유연 모델을 적용한 독립 진단

두 외부 잎 그룹만 기존 world-D6 잎 모델로 바꾼 새 자산에서 3초 엄격 idle은 정상 완료됐다. 초기 상태 유효, 파단 0개이며 수확 시험은 아니다. 이 자산에서 seed1/100mm, seed1/250mm, seed5/100mm의 3개 기존 직접 연결을 확인했다. 준비 구간 환경 8Hz는 세 경로 모두 끝까지 검사해 hard hit 0개였지만, 자기충돌 때문에 실행 후보로 사용할 수 없었다.

100mm seed1/5는 처음 2mm 여유 기준에 걸릴 때 약 0.346/0.347mm의 양수 간격이었다. 이것만으로 실제 관통이라고 해석하지 않고, seed5를 기존 2mm 결과와 별도로 ‘교차만 검사하는 0mm 명시적 진단’에서 120Hz로 끝까지 검사하려 했다. 조금 뒤 준비 구간 0.297222 지점에서 같은 `L_C14_rear_drive_cover ↔ link3`가 실제 기하 교차해 멈췄다. 따라서 이번 경로는 2mm 여유 수치만 줄여 해결되는 경로가 아니다. 기본 2mm 계획 기준과 0.5mm native 물리 기준은 변경하지 않았다.

250mm seed1의 완전 개방 목표는 여전히 정상이다. 다만 그곳까지의 직접 관절 연결은 고리 뒷판 `F01_rear_rail_backbone ↔ link1`과 실제 교차했다. 다음 작업은 목표 자세를 다시 바꾸는 것이 아니라 이 중간 구간을 피하는 충돌 회피 연결이다.

3초 idle의 속도는 57.04초였고, simulate 56.83초에는 접촉 callback 22.56초가 포함됐다. 접촉점 1,199,596개가 기록됐다. CPU 병행 작업 중의 기능 검증이므로 독립 속도 비교로 쓰지 않는다. 유연 잎 두 개의 추가가 초기 유효성을 통과하더라도 빠른 실행까지 검증됐다는 뜻은 아니다. 작은 비교 자료는 `validation/fresh_truss01_two_passive_leaves_20261011.json`에 있다.
