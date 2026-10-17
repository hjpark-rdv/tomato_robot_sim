# 벨트 접촉·중단 정책 개별 시험

새 GPU 전체 실행14개와 별도reference2개, 실제120Hz 접촉 기록의 중단 정책10조건 후처리를 완료했다. 벨트 물성을 바꾼4조건은 모두 원본보다 느렸고, 겹침 기준만 완화해도 다른 고정 부품 접촉이 남았다. 닫기→열기 잔류 허용은 이번 실제 궤적에서 사용되지 않았다. 따라서8–11번에서 채택할 속도 개선 요인은 발견하지 못했다.

이 보고서는 개별 시험 8–11의 근거다. 원본385DOF, 접촉400Hz/이동60Hz, 동일 초기상태·명령을 사용한다. 각 변경은 누적하지 않는다. `--diagnostic-continue`는 전체 동작을 관찰하기 위한 설정이며, 완료 여부를 물리 유효성 또는 수확 성공으로 승격하지 않는다.

항목 번호: **8** 벨트–과실 겹침, **9** 가는 꼭지 및 정확한 비벨트 쌍, **10** 벨트 K/D 물성, **11** 닫기→열기 잔류 눌림.

## 접촉 대상을 정확히 구분

- 실제 표면속도가 적용되는 `Tool_L_belt`/`Tool_R_belt`의4충돌체(`s300_g0813`, `s300_g0814`, `s300_g0815`, `s300_g0816`)만 벨트로 지정했다. CAD벨트메시와PrototypeTread박스를 모두 포함한다. `Tool_L/R/fixed`의다른CAD구성품은제외한다.
- 열매는 기존attachments목록의 정확한body, 꼭지는44개Pedicel캡슐로 구분한다. 꼭지반경0.625–0.825mm(직경1.25–1.65mm)다.
- 기본정책은 닫기/재닫기만 벨트–열매 또는 벨트–꼭지 기준을 변경한다. 다른 접촉은1mm참고를 유지하며, 정확한 비벨트geom쌍을 명시하면 그쌍·그단계만 별도기준으로 시험할 수 있다.
- 기존1mm원자료·최초위반·힘·분리규칙은 항상 보존한다. 새기준은 별도first_policy_event로 기록한다. 충돌마스크·마찰·형상·실제접촉력을 끄지 않는다.

## 닫기→열기 잔류 눌림

닫는 동안 실제1mm를 넘었던 정확한쌍만 잔류 허용을 획득한다. 열기단계에서최대0.5초만 같은상한을 유지하며, 접촉상실/1mm이하복귀/다른단계/기한초과면 즉시해제한다. 다른쌍이나 새롭게생긴 열기접촉에는 적용하지 않는다. 두번째재닫기뒤 `belt_finish`는 열기단계가 아니므로 이정책이 자동확대되지 않는다. 시간판정은 기록된simulation timestamp를 사용한다.

## 벨트 탄성 근사

실제GPU물성시험은 원본positive-solref형식을유지한다. 지오메트리K목표는tau고정,zeta=zeta0/sqrt(alpha), D목표는tau=tau0/beta,zeta=zeta0*beta로만든다. 다른480개모델배열,마찰·solimp·priority·명령·초기상태는그대로다. 단,원본모든geom의solmix1/priority0때문에실제벨트–열매solref는양쪽material평균이며,DISCRETE CONTACT의positive-only REFSAFE행클램프도작용한다. 따라서지오메트리K/D목표하나를변경해도실제접촉행K/B는함께변할수있다. 이효과를숨기지않고아래표와actualGPU행계수대조로기록한다. 초기Euler클램프오적용자산과,rawKB만맞췄지만CONTACT행클램프를우회하는negative자산은준비오류대조로보존했으며GPU수확시험에는실행하지않았다.

## 실제 GPU 전체 실행

|시험|상태|전체RTF|닫기RTF|벨트마무리RTF|활성최대mm|분리수|새정책통과|
|---|---|---:|---:|---:|---:|---|---|
|policy_reference|completed|1.556|0.498|0.381|0.794|[6]|True|
|material_positive_control|completed|1.555|0.497|0.380|0.794|[6]|None|
|material_positive_damping05|completed|1.504|0.486|0.355|0.598|[5]|None|
|material_positive_damping2|completed|1.475|0.487|0.367|0.639|[5]|None|
|material_positive_stiffness025|completed|1.498|0.493|0.350|0.713|[6]|None|
|material_positive_stiffness05|completed|1.471|0.480|0.361|0.859|[5]|None|
|original120_other_Fruit08_part147_2mm|completed|3.456|1.563|1.161|2.929|[7]|False|
|original120_reference|completed|3.467|1.569|1.168|2.929|[7]|False|
|policy_fruit2|completed|1.548|0.496|0.376|0.794|[6]|True|
|policy_fruit3|completed|1.549|0.496|0.377|0.794|[6]|True|
|policy_fruit3_residual05|completed|1.548|0.496|0.376|0.794|[6]|True|
|policy_fruit5|completed|1.549|0.496|0.376|0.794|[6]|True|
|policy_pedicel125|completed|1.549|0.496|0.376|0.794|[6]|True|
|policy_pedicel15|completed|1.549|0.495|0.376|0.794|[6]|True|
|policy_pedicel5_risk_control|completed|1.549|0.496|0.376|0.794|[6]|True|
|policy_reference_v2|completed|1.555|0.497|0.378|0.794|[6]|True|

## 실제 contact material 혼합·DISCRETE행계수

다음은400Hz,impedance평가깊이0.5mm의정규화된 솔버 행 계수다 (K는 s^-2, B는 s^-1). 실측 N/m 또는 N·s/m 물성값이 아니다. 실제벨트와원본열매material을혼합한계수이며,source/negative행은형식대조다. 계산장치: cuda:0

|조건|raw K|raw B|실제 normal행 K|실제 normal행 B|
|---|---:|---:|---:|---:|
|source_positive|250500.72|1001.00|160884.85|802.21|
|negative_raw_reference|250500.73|1001.00|250500.73|1001.00|
|mixed_pair_stiffness05|171916.50|1001.00|160884.85|968.35|
|mixed_pair_stiffness025|111333.66|1001.00|111333.66|1001.00|
|mixed_pair_damping05|197926.50|667.33|160884.85|601.66|
|mixed_pair_damping2|197926.50|1334.67|160884.85|1203.31|

물성 계수 변경은 접촉 응답을 바꾼다. 네 material 변형 모두 전체 RTF가1.471–1.504로 원본1.555보다 느렸다. K0.25는6개 분리를 유지했지만, 나머지3조건은5개였다. 이 분리 수를 실물 수확 정확성으로 해석하지 않는다.

현재3.15의gap은형상밖의비활성탐지대역이다. source각geom margin0/gap0.5mm이며 pairgap1mm에서먼저탐지하고 signed distance<0부터활성제약을만든다. 예전gap관례를적용해0.5mm관통후에야힘이생긴다고해석하지않는다. 해당현재동작은nativeCPU기하대조와설치된GPUcollision_core소스로검사했다.

## 작은 대조 시험

contact/material/replay의20개 자동시험에 정확한쌍분류, 분리열매예외, 잔류허용timestamp만료, 접촉소실후재접촉차단, 비벨트정확쌍만허용, K/D개별불변성, 실제마찰끌어들이기를 포함한다. 결과로그는 rawroot/contact/tests_v3.log (geometry 검증6개 포함 총26개 통과)다.

Euler적분기NativeMuJoCo CPU의 작은 양쪽벨트마찰대조에서는 각0.5mm눌림·벨트0.15m/s로0.2초실행해, 원본positive와direct대조가 같은29.9991mm전진과최종0.15m/s를보였다. K0.25도 끌어들임이유지됐다. D2는전진하지만속도가약0.00146–0.29927m/s로진동했다. 감쇠가높다고무조건안정적이지않다. 이는GPU수확성능자료나DISCRETE소재등가성증거가아니다.

직경1.4mm꼭지와두께5mm벨트의기하학적mocap스윕에서는최대겹침3.2mm만으로벨트가꼭지를완전히지나가고접촉법선이반전됐다. 따라서5mm허용은 이런통과를차단하지못한다. 동적실행에서실제로발생했다는뜻은아니며,5mm꼭지정책은위험음성대조이지추천값이아니다. 이전world고정geom갱신실패fixture도별도이름으로보존했다.

## 실제 접촉 witness

별도120Hz의 정확한 Fruit08–CADpart147 시험 한 개에만 read-only physics-frequency witness를 붙였다. 실제GPU접촉쌍·signed distance·법선·월드힘/토크·양쪽geom국소좌표의상대중심·제어 콜백의 cached c.attached 상태를 저장한다. 같은 스텝에서 release_attachments가 eq_active를 바꿨더라도 cached 상태는 다음 제어 콜백까지 유지된다. 기존물리는그대로이며이시험RTF는계측부하가있어clean성능순위에서제외한다. 별도 reference120 실행과의 물리 배열도 정확히 일치했다. 순수 정책·대조·계측의 물리 배열 비교는 총11건 모두 통과했다. 이추가자료는단순히중단기준이통과했다는이유로견인성공을주장하지않기위한것이다.

## 기록 시점의 구분

접촉 거리·법선·접촉점·힘은 적분 전 solve의 기록이다. 국소 중심 좌표는 적분 후 갱신된 geom 자세이며, c.attached는 제어 콜백 시점의 캐시이고 timestamp는 실제 step-end다. 이를 한 시점의 자세로 간주하지 않는다. 원본 schema를 보존하고 contact_witness_time_scope_addendum.json에 명시했다. 중단 판정 재평가는 GPU와 같은 거리·캐시·시계를 사용하므로 이 구분이 결과를 바꾸지는 않는다.

## 판정의 한계

- 400Hz원본에서벨트겹침이이미1mm이하라면 2/3/5mm정책은 실제허용을한번도사용하지않는다. 그런결과를성능개선으로해석하지않는다.
- 정책만변경한경우 실제물리배열은같아야한다. 같은모델/초기상태/명령의exact비교결과는contact_evidence.json에저장한다. 의도적으로다른설정메타데이터까지동일하다고주장하지않는다.
- 전역최대접촉력은분리열매접촉도포함하므로 타깃열매손상력으로해석하지않는다.
- 조인트분리수는수거완료나송이전체수확성공과다르다. training_eligible=false,harvest_success=null유지.

원자료: /root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/contact

재생성: `nvidia-sim/.newton-venv/bin/python /root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/contact/build_contact_report.py`


## 120 Hz 실접촉 기록에서 중단 정책만 다시 평가

추가 physics를 실행하지 않았다. 같은 GPU 접촉 기록을 읽어 중단 여부만 계산했다. 1mm 기준의 최초 이벤트가 실제 GPU 원자료와 정확히 일치했고, 저장된 정책의 최초 이벤트·단계별 최대 겹침·허용 횟수도 모두 일치했다.

|정책|최초 중단 시각(s)|단계 / 쌍|과실 허용 pair-step|꼭지 허용 pair-step|열기 잔류 허용|
|---|---:|---|---:|---:|---:|
|reference_1mm|53.026|close g0355 / s300_g0816|0|0|0|
|fruit2|53.551|close g0355 / s300_g0816|17|0|0|
|fruit3|53.851|close g0373 / s300_g0581|19|0|0|
|fruit5|53.851|close g0373 / s300_g0581|19|0|0|
|pedicel125|53.026|close g0355 / s300_g0816|0|68|0|
|pedicel15|53.026|close g0355 / s300_g0816|0|113|0|
|pedicel5_risk_control|53.026|close g0355 / s300_g0816|0|194|0|
|fruit3_residual05|53.851|close g0373 / s300_g0581|19|0|0|
|pedicel15_residual05|53.026|close g0355 / s300_g0816|0|113|0|
|other_Fruit08_part147_2mm|53.026|close g0355 / s300_g0816|0|0|0|

횟수는 진단용 전체 궤적에서 계산하며 최초 중단 이후도 포함한다. 정책을 통과했다는 이유로 이후 상태가 새로 생긴 것은 아니다. 각 실험은 원본 궤적을 보존한 순수 후처리이며, RTF나 새 물리 성공으로 집계하지 않는다.

과실2mm는 최초 중단을53.026→53.551초로 늦추지만 Fruit05–벨트가 다시 기준을 넘는다. 3/5mm는53.851초의 Fruit08–고정된 Tool_L 형상 접촉까지 진행할 수 있다고 판정한다. 이때는 벨트가 아닌 다른 부품이므로 벨트 눌림만 완화해서 해결되지 않는다.

꼭지 최대2.929mm는 belt_finish 단계의4번 꼭지에서 발생했다. 그 시점에는 Fruit04가 이미 분리되어 CONNECT 힘은0N이고, 실제 벨트 접촉력은 약3.988N이었다. 따라서 이를 아직 붙어있는 열매의 견인 실패라고 단정하지 않는다. 별도로 확인된 rigid CAD와 가는 가지의 침투는 벨트 눌림으로 간주하지 않는다.

열기0.5초 잔류 규칙은 이120Hz 궤적에서도 실제 사용 횟수0이다. 기계 동작에 이점이 입증된 것이 아니라, 준비된 시간/쌍 제한 로직과 경계 조건이 검증된 상태다.

관련 원자료: /root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/contact/original120_posthoc_policy_tests.json
