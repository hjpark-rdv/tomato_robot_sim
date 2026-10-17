# 원본 Hz 시험의 꼭지/가지–그리퍼 형상 검토

저장된 실제 GPU qpos를 읽어 native CPU kinematics 및 `mj_geomDistance`를 적용한 **읽기 전용 진단**이다. 새 물리 실행, 실제 GPU 접촉력, 실물 압축·손상 판정과 구분한다. `training_eligible=false`, `harvest_success=null`을 유지한다.

1mm를 universal hard gate로 쓰지 않고, 원본120Hz에서 확인된 정확히6개 capsule–그리퍼 본체 pair를 비교한다. 이6pair의 manipulation 구간 signed distance를 찾고, pair별 최악 접촉 전후±0.15s에서 실제 compiled convex collision hull과 capsule 중심선을 비교한다. 다른 pair의 부재나 전체 scene의 유효성까지 증명하지 않는다.

## 방법과 대조 시험

`newton_contact_geometry_review.py`는 정규화된 convex hull halfspace에 capsule 중심선 선분을 clipping한다. 반경+10nm만큼 안쪽으로 이동한 모든 평면을 만족하는 선분 부분이 있으면 **그 위치의 capsule 반경 구 전체가 rigid-tool convex proxy 안에 들어간 상태**다. 이 결과는 1mm 초과와 별개다.

반대로 반경 구가 완전히 들어가지 않았다는 것만으로 비관통을 증명하지 않는다. 국소적으로 얇은 부분이나 모서리에서는 capsule 중심선이 solid를 가로질러도 반경 구 전체는 들어가지 않을 수 있다. 실제 고정 파트의 전체 형상이 얇다고 단정하지 않는다.

6개 테스트 통과: 완전 반경 구 포함 양성 / 충분히 떨어진 음성 / 한 점 모서리 접촉과 실제 교차 구분 / 한쪽 끝 삽입과 양끝 바깥 교차 구분 / 국소 얇은 solid 교차인데 완전 구 포함은 없는 대조 / 강체 변환 불변성·잘못된 단위 및 비유한 값 차단.

## 현재 결과

아래 penetration은 저장 GPU q에서 nativeCPU `mj_geomDistance`로 재구성한 값이며, GPU contact buffer의 signed distance와 혼동하지 않는다.

| 원본 pair | capsule 지름 |120Hz 최대|200Hz 최대|240Hz 최대|300Hz 최대|400Hz 최대|
|---|---:|---:|---:|---:|---:|---:|
|distal06 g0166–Tool_L g0620|1.650mm|1.788mm|0(비교 구간 비접촉)|0|0.640mm|0.539mm|
|distal03 g0154–Tool_R g0707|1.650mm|1.751mm|0.840mm|0.706mm|0.495mm|0.340mm|
|proximal06 g0165–Tool_L g0620|1.250mm|1.588mm|0|0|0.440mm|0.339mm|
|proximal03 g0153–Tool_R g0707|1.250mm|1.551mm|0.640mm|0.506mm|0.295mm|0.140mm|
|Rachis13 g0126–Tool_fixed g0473|1.135mm|1.535mm|0.745mm|0.259mm|0.430mm|0.281mm|
|distal03 g0154–Tool_R g0710|1.650mm|1.159mm|0.303mm|0.159mm|0|0|

120Hz에서는 첫5pair의 최악 접촉 부근에 반경 구 전체가 proxy 안에 들어간 표본이2~4개 있다. 묻힌 중심선 길이는 최대0.40~0.64mm다. 따라서 **120Hz의 큰 겹침을 전부 벨트 탄성에 의한 눌림으로 설명할 수 없다.**

다만 가장 큰 distal06/proximal06은 제거된 Fruit06의 남은 가지이며 현재 attachment 목록에 Fruit06이 없다. distal03/proximal03의64.233s 접촉에서는 Fruit03이 이미 분리됐다. g0620/g0707의 해당 endpoint는 같은 방향 면 쪽으로 되돌아 나오는 형상이다(진입 전/이탈 후 closest-plane normal dot0.971/1.0). 이것만으로 연결된 과실의 잡기·당김 실패나 가지 전체가 금속 반대편까지 통과했다고 단정하지 않는다.

g0154–g0710은 별도로 capsule 중심선의 양 끝이 밖에 있으면서 convex solid를 지나가는 표본4개가 있다. 반경 구 전체 포함은 없다. **국소 convex hull 교차**로 기록하며, 실제 CAD의 concavity를 메운 proxy 근사 가능성과 힘/접촉 방향을 추가 해석해야 한다. g0710 전체 hull 크기는 약16×64×89mm여서 전체 파트를 얇은 판이라고 부를 근거는 없다.

200/240/300/400Hz의 같은6pair에서는 diameter 초과 표본이0개이며, pair별 최악 접촉 전후의 완전 반경 구 포함 및 양 끝 바깥 중심선 교차가 모두0이다. 현재 조사한6pair에 한정하면200Hz가120Hz보다 위 현상을 줄인다. **모든 pair가 깨끗하다거나200Hz가 필요한 최저 주파수라는 전역 결론은 아니다.**120Hz의 실제 접촉력/법선/traction과 실물의 연한 가지 압축 근사로 수용 가능한지는 별도 판단한다.

240Hz의 전체 감시 최대10.081mm는 위 capsule 표와 다른 지표다. 첫 큰 이벤트는56.4875s의 **이미54.1875s에 분리된 Fruit08–거터2** 접촉(9.653mm)이다. 이 단일 최대값으로 주줄기 불안정이나 부착 중인 열매/꼭지 문제라고 해석하지 않는다. 기존 monitor의 `active`라는 이름은 반드시 부착 중인 열매를 뜻하지 않으며, 전역 최대 pair와 첫 이벤트가 항상 같은 pair라는 보장도 없다.

## Hz 비교의 솔버 조건

이 원본은 `DISCRETE` 적분 및 `REFSAFE`를 사용한다. 현재 설치된 MuJoCo Warp의 discrete 제약 행 계산은 양수 형식 접촉 계수가 시간 간격에 비해 클 때 `h²·k·imp`를 제한하고 damping도 함께 바꾼다. 따라서400→120Hz 시험은 단순히 스텝 개수만 줄인 것이 아니다. 시간 간격에 따라 유효 접촉 regularization이 달라지는 효과도 포함한다. 여기서 `k`는 제약 가속도 계수이며 직접적인 N/m 재료 강성과 혼동하지 않는다.

별도 contact 실험의 `contact/gpu_contact_row_coefficients.json`에는120/240/400Hz와 접촉 깊이0/0.5/1/2/3mm 등150개 실제 CUDA probe가 있다. 양수 `solref` 형식과 source/belt material mixing을 유지해 행 계수를 확인한 자료다. 본 문서의 native CPU 형상 재구성과 해당 GPU 행 계수·접촉력 측정은 서로 다른 증거이며, 어느 하나로 다른 항목을 대체하지 않는다.

## 개별 시험 완료 뒤의 조합3건

아래는 위 원본 Hz 개별 시험과 **별도**다. 실제 사용한 `inputs_coarse2`(190DOF)/`inputs_coarse4`(133DOF)의 MJB에서 저장 q를 재구성했다. `summary → inputs.json → input_model.mjb` SHA 연결과 q 차원을 확인했다. 화면 USD hash는 물리 MJB hash와 범위가 다르므로 혼동하지 않았다. 충돌 형상901개는 유지되지만, 관절 구조와 Hz가 함께 달라진 조합이다.

| 조합 | 전체 RTF | 가장 느린 phase RTF | 기존 active 범주 최대 겹침 | 첫1mm 초과 pair / 시각 | 분리 수 / 벨트 전 분리 |
|---|---:|---:|---:|---|---:|
|coarse2 /200Hz|3.503|belt_finish0.985|1.673mm|부착 중 Fruit05↔오른쪽 벨트,53.960s|6 /0|
|coarse4 /200Hz|3.944|belt_finish1.170|1.445mm|부착 중 Fruit05↔오른쪽 벨트,53.200s|6 /0|
|coarse4 /240Hz|3.475|belt_finish0.966|1.232mm|Fruit04의 남은 distal 꼭지↔왼쪽 벨트,54.904s|6 /0|

3건 모두 finite 상태로 전체 trace를 마쳤으나 **diagnostic-continue**였다. 기존1mm 감시를 통과한 실행이라고 보고할 수 없다. coarse4/200Hz만 모든 phase가1배속 이상이지만, 이 조건 하나로 최종 선택을 확정하지 않는다. 첫 초과 시각/깊이와 전체 최대 깊이는 서로 다른 표본이며, 표에 이를 구분했다.

240Hz의 첫 pair에서 Fruit04는54.017s에 이미 분리됐다. 그러나 그 꼭지는 여전히 식물에 연결돼 있다. **분리된 과실이 거터에 떨어진 접촉과 같은 종류로 분류하면 안 된다.** 3건의 released 범주 최대16.97~20.42mm도 별도 저장했으며 active 범주 최대의 원인으로 잘못 설명하지 않는다.

같은6개 capsule–본체 pair의 재구성 최대 겹침은 coarse2/200Hz에서 `[1.144,0.762,0.945,0.562,0.644,0.435]mm`, coarse4/200Hz에서 `[0.969,0.590,0.770,0.390,0.636,0.234]mm`, coarse4/240Hz에서 `[0.710,0.633,0.510,0.433,0.619,0.140]mm`다. **세 조합 모두 조사한6pair의 최악±0.15s에서는 완전 반경 묻힘0, 양 끝 바깥 중심선 교차0**이다. 이것은 다른 모든 충돌체나 벨트 접촉이 검증됐다는 뜻은 아니다.

최초 벨트 속도 발생은48.800s이고, 그 전 분리는 모두0이다. 벨트 전 주줄기 최대 이동은0.086~0.092mm, 모든 가지0.202~0.210mm, 과실 최대0.519~0.528mm였다. 조합 초기 상태가 접촉 없이 크게 무너져 분리됐다는 증거는 없다. 수확 중 주줄기 끝 이동은89.4~94.3mm/주줄기 방향 변화 약5.0~5.2°, 가지 최대 이동110.9~112.7mm였다. 부착 중 과실 최대 이동도111.5~113.4mm까지 있으므로 "작은 겹침만 있고 식물 반응은 동일하다"고 단정하지 않는다. 이는 관찰 수치이며 임의의 universal5mm gate로 탈락시키지 않았다. 원본 물성 보존과 실물 변형의 정확성은 별개이며, 조합 모델의 가압·당김 반응 대조와 실제 qpos 영상으로 평가해야 한다.

`combo_geometry_review.json`에 전체 phase RTF, 최초 이벤트의 eq 상태와 분리 시각, 모든 열매의 부착 중 이동·속도, 주줄기/가지 이동과 회전, 관절별 추종 오차가 있다. 상세6pair 표본은 `combination_screen/<case>/geometry_review.json`이다. 읽기 전용 재현 script는 `review_combination_geometry.py`이고 새 물리 실행을 만들지 않는다. Git 요약은 `nvidia-sim/validation/s300_newton_combo_geometry.json`이다.

## 원자료

루트 `/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests`:

- `original120_capsule_geometry_review_v2.json`
- `original200_capsule_geometry_review_v2.json`
- `original240_capsule_geometry_review_v2.json`
- `original300_capsule_geometry_review_v2.json`
- `original400_capsule_geometry_review_v2.json`
- `review_hz_capsule_geometry.py`: 기존 실행 결과에 대한 재현용 읽기 전용 script.
- `hz_contact_geometry_summary.json`

Git의 작은 요약은 `nvidia-sim/validation/s300_newton_hz_contact_geometry.json`이다. 입력 MJB/상태/원자료 SHA와 표본 timestamp, capsule 치수, 실제로 검사한 pair/시간 범위를 보존한다. 기존 raw 결과를 덮어쓰거나 학습 데이터로 사용하지 않았다.
