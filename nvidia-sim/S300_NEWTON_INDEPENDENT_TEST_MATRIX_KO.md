# Newton GPU 개별 시험 11항목: 결과와 해석 범위

2026-10-17, `codex/s300-newton-independent-tests-20261017`.

이 문서는 대안을 **개별적으로 먼저 확인한 결과표**다. 조합의 최종 추천, GUI/headless 최종 비교, 다중 환경의 최종 성능은 별도 통합 보고서에서 다룬다. 8–11번은 새 전체 실행14개와 별도120Hz 실접촉 기록의 중단 정책 후처리10조건까지 완료했다. 11항목 모두에 대해 제한된 수준의 구현·실행·검토를 배정했지만, 모든 가능한 표현·계수 조합의 전역 최적값을 찾았다는 뜻은 아니다.

## 비교 기준

- 원본 입력: `/root/docker_share/mujoko_debugging_data/20261016_s300_newton/inputs_render_separated`.
- 원본 MJB SHA256: `2029e0e4cb5c9775b2138abadc6cd290045d89fefcd79b5065e5cedf2a5377c0`.
- 원자료 루트: `/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests`.
- 공통 원본: 385DOF, 접촉 400Hz / 이동 60Hz, 같은 초기 상태와 약82.795초의 수확 명령, 실제 Newton → MuJoCo Warp CUDA 실행.
- 전체 농장 외형, 타깃 송이·주줄기 중심의 활성 충돌 범위, 벨트 방향, 닫기·열기 반복, 기존 3N/0.008Nm 분리 조건을 유지한다. 변경한 항목은 별도 입력/명령/결과로 보존한다.
- 기준 1mm의 원자료는 계속 기록한다. 사용자가 허용한 기준 변경 시험은 별도 정책으로 판정하며, 더 큰 허용값 자체를 접촉 물리 정확성이나 실물 안전성으로 해석하지 않는다.
- RTF = 실제 시뮬레이션 시간 / 실제 경과 시간. 부분 실행, 준비 실패, 계측 실행은 완주한 clean 처리량과 구별한다. GPU 시간은 완료 동기화를 포함하며, 초기 로딩/JIT/graph capture는 loop 성능과 분리한다.
- 연결 해제 수는 바닥 낙하·수거·송이 전체 수확 성공과 다르다. 이 자료는 `training_eligible=false`, `harvest_success=null`이다.

새로 측정한 원본400Hz는 **전체RTF 1.556, 닫기 0.499, 마지막 벨트 0.378**, 감시 최대 겹침0.794mm, 연결 해제6개다. 평균은 실시간보다 빠르지만 접촉 구간 실시간 목표는 충족하지 못한다. 이 차이를 숨기지 않는다.

## 11항목 개별 결과표

| 번호 | 개별 변경 | 실제 시험/증거 | 현재 결과 | 해석 및 남은 범위 |
|---|---|---|---|---|
| 1 | 물리Hz. 접촉Hz와 이동Hz를 분리 | 원본 접촉60/120/200/240/300/400Hz. 이동은60Hz 고정. 별도로 접촉400Hz 고정에서 이동30/60/120Hz | 접촉120Hz 전체3.471/닫기1.568/마지막벨트1.167RTF이나 최대 겹침2.929mm. 접촉60Hz는 nonfinite. 이동30Hz는 접근 전10개가 모두 분리 | 속도뿐 아니라 실제 접촉/분리 상태가 달라진다. 아래 세부표와 실패 분류 참조. 400Hz나1mm를 변경 불가능한 정답으로 취급하지 않는다 |
| 2 | 실제 접촉 기반 적응Hz | 접촉 활동을 관찰해400/60Hz를 선택. 같은 연속 명령을 해당 시각에 샘플링 | 수정 후 전체2.0955RTF/닫기0.572/마지막벨트0.367, 최대0.716mm,6개 분리,8,451step 완주. GPU 명령/벨트 버퍼 readback 및 tick 검사 PASS | 원본 형상·DOF를 유지한 속도 개선. 마지막 벨트 구간은 여전히 느리다. 적분 스텝이 달라지므로 원본과 동일 궤적이라고 주장하지 않는다 |
| 3 | 그리퍼 충돌 표현 | 완전히 묻힌115/132개 collider 제거, 원본ID를 보존한 비활성 대조, 실제 접촉하는 외부3개 메시만 보수적proxy | 외부3개proxy 전체1.537RTF/0.708mm/6개. 묻힌115개 제거 관찰완주1.506RTF/5.293mm/5개. 기존1mm 중단 실행과ID유지 대조도 보존 | 속도 이득이 없고 반응/분리 시점 차이가 있다. 외부 형상의 합집합 보존만으로 동적 결과 동등성을 가정할 수 없다 |
| 4 | 식물 관절 수/표현 해상도 | 기존385→304→190→133DOF 전체 GPU 실행과 `response_v2_*` 하중 시험을 source hash 확인 후 재사용 | 전체RTF1.556→1.678→2.069→2.436.133DOF의 닫기0.769/마지막벨트0.596, 최대0.658mm/6개 | 단계형 모델 표현 변경이다. 잎 고정/분절 병합을 포함하므로 원본과 같은 물리로 주장하지 않는다. 작은 하중 비교와 그 한계를 아래 명시 |
| 5 | 열매–꼭지 연결 표현 | DISCRETE에 맞춘 CONNECT direct 계수 대조와 같은 위치stiffness에서 damping만2배 | direct 전체1.497RTF/0.600mm/5개. damping2배1.547RTF/0.773mm/6개. CPU4개/CUDA3개 검사 PASS | 원본은 이미 free6DOF+CONNECT+3축 angular spring이다. 이번에는 계수 표현/감쇠를 시험했으며 ball joint 기반 분리 모델 교체까지 구현한 것은 아니다 |
| 6 | 분리 완료 열매의 계산 부하 / native sleep | 열매 free tree만 native sleep 허용, 나머지 로봇/식물은 항상 awake. native backend에서 sleep off/on을 각각 완주 | off0.498259RTF/5개/3.851mm, on0.495689RTF/6개/1.016mm. 실제 asleep fruit-pair-step31,190, attached-asleep0 | 열매 삭제·강제고정·충돌 제거 없이 sleep 기능 작동은 확인. 속도 이득은 없고 두 결과 모두 기존1mm 참고 초과. 최적화 backend와 호환되지 않아 더 느린 native 대조 경로를 사용했음을 분리 |
| 7 | GPU 계산/실행 최적화 | AABB만, factor cache만, graph batch10→20만, contact capacity128→64만, constraint capacity256→128만 변경 | 완주한4설정은 원본 상태/물리배열/분리이벤트 정확히 일치. RTF1.544–1.563으로 유의한 이득 미확립. contact capacity64는 broadphase65개에서 overflow | 설정만 다른 strict metadata 실패와 실제 물리 차이를 분리. 중단된 capacity64의 부분RTF는 성공 성능에서 제외 |
| 8 | 벨트–열매 중단 기준 | 닫기/재닫기의 정확한 벨트–열매 쌍만1/2/3/5mm. 실제 힘·형상은 그대로 | 400Hz의2/3/5mm 새 실행은 원본 물리배열과 정확히 같고 허용 횟수0.120Hz 실접촉 후처리에서2mm는 최초 중단53.026→53.551초,3/5mm는53.851초로 늦춤 | 이후 Fruit08–고정 Tool_L 접촉이 남는다. 순수 중단 정책은 견인 물리나 속도를 개선하지 않는다. 후처리를 새 physics로 집계하지 않음 |
| 9 | 가는 꼭지·명시한 다른 쌍의 겹침과 관통 검토 | 벨트–꼭지1/1.25/1.5/5mm, 정확한 Fruit08–CADpart147 쌍의2mm, 실제 GPU 접촉력 witness | 400Hz 꼭지 정책은 허용 횟수0.120Hz 후처리1.25/1.5/5mm는68/113/194 pair-step 허용하나 첫 과실 중단은 그대로. 실제 비벨트 쌍 시험도 완주/원본 물리배열 동일하지만 다른 충돌이 남음 | 최대2.929mm 벨트–4번 꼭지는 Fruit04 분리 이후. 실제 고정 CAD 침투는 벨트 눌림으로 보지 않는다.5mm 기하 음성대조는 직경1.4mm 꼭지 완전 통과를 숨길 수 있음 |
| 10 | 벨트 soft contact의K/D | 벨트geom의positive-solref 형식에서 K목표0.5/0.25, D목표0.5/2를 각각 변경. source control 포함5개 실제 전체 실행 | source1.555RTF/6개/0.794mm. K0.5:1.471/5개/0.859mm, K0.25:1.498/6개/0.713mm, D0.5:1.504/5개/0.598mm, D2:1.475/5개/0.639mm | 모두 source보다 느리고 접촉 구간 실시간 미달. 일부 겹침은 감소하지만 채택할 속도 이점 없음. 실제CUDA150조건에서 solmix·DISCRETE REFSAFE 행계수 결합을 확인. 행계수는 실물N/m가 아님 |
| 11 | 닫기→열기 잔류 눌림 처리 | 닫는 동안1mm를 넘은 정확한 쌍만 열기에서0 또는최대0.5초 허용. 실제timestamp로 기한 검사 | 새400Hz 전체 실행과120Hz 실접촉 후처리 모두 잔류 허용 사용0.단위시험에서 소실·1mm복귀·기한·phase 전환 해제와CPU/Warp 결과 일치 | 로직/범위는 검증했지만 이 궤적에서 기계적 이점은 미입증. 새 쌍·다른 phase·belt_finish까지 허용을 자동 확대하지 않음 |

8–11의 최신 실제 집계는 [벨트 접촉 개별 보고서](harvest_s300/newton_contact_experiments_ko.md)와 raw `contact/contact_evidence.json`에 있다. 전체 입력·코드 hash, 실제 접촉력, 순수 정책의 물리배열 정확 비교11건, GPU 계수 대조 및 기록 시점의 한계를 보존했다. compact 근거는 [접촉 개별 evidence](validation/s300_newton_contact_independent.json)에 있다.

## 1. 접촉Hz와 이동Hz: 속도와 물리 변화를 함께 기록

아래는 **원본385DOF**이며 다른 개선을 섞지 않았다. 겹침은 기존 monitor의 `active` 범주 최대다. 이 범주는 ‘아직 부착 중인 열매’와 동의어가 아니므로 pair별 원인을 따로 읽어야 한다.

| 접촉/이동Hz | 전체RTF | 닫기RTF | 마지막벨트RTF | 감시 최대mm | 분리수 | 결과 및 주의 |
|---|---:|---:|---:|---:|---:|---|
|60/60|—|—|—|미확정|미확정|약74.13초에 fixed-reduction failure, 저장q/v/pose nonfinite. 실패처리 중NaN JSON오류로summary가running에 남았으나 assessment exit1과log로 실패 확인|
|120/60|3.471|1.568|1.167|2.929|7|관찰계속으로 완주. 기존1mm 참고는 초과하며, 가는 가지/그리퍼 기하와 분리 회전 양상 별도 검토|
|200/60|2.543|0.972|0.728|1.395|6|평균만 실시간 이상. 마지막벨트는 실시간 미달|
|240/60|2.185|0.808|0.628|10.081|7|큰 최대 겹침은 분리된Fruit08↔거터. 이를 주줄기 충돌 또는부착열매 벨트 눌림으로 오인하지 않음|
|300/60|1.909|0.656|0.493|1.006|6|1mm 경계에 가까우나 정밀도·분리·기하의 실물 동등성은 별도 문제|
|400/60|1.556|0.499|0.378|0.794|6|기존1mm 참고 안에서 완주. 접촉 구간은 여전히느림|
|400/30|2.110|0.755|0.419|1.549|10|settle에서 이미모두분리. 분리조건 평가시각0.100–0.166667초, 해제효력은각step끝0.133333–0.200000초. 접근/수확으로 딴10개가 아님|
|400/120|1.241|0.496|0.355|0.714|6|완주했으나 더느림. 이동Hz를 낮추는것이 항상 안전한 것도, 높이는것이 항상 유리한 것도 아님|

60Hz의 저장 오류는 실패를 감추거나 성공으로 재해석하지 않고 보존했다. 이후 runner의 실패 보고기는 nonfinite값을 명시한 문자열로 기록하도록 수정하고 회귀시험을 추가했다. 원본 실패폴더를 덮어쓰지 않았다.

120/400Hz의 저장 q/v로 angular spring·damping을 독립 재계산한 차이는 최대4.92e-9Nm였다. 모든 연결 해제는 최초3N/0.008Nm 초과step과 맞았다. 그러나120Hz의7개 분리는 모두torque조건이며 일부는 축회전보다swing이 크다.400Hz도 휘어짐/damping 중심의 분리가 있으므로 **400Hz=실물정답, 7개=6개보다 정확한수확**으로 각각 단정하지 않는다. 저장하지 않은 전체efc행force를 재계산한 검증은 아니다.

Hz 기하 검토: [접촉 형상 검토](harvest_s300/newton_contact_geometry_review_ko.md), [compact evidence](validation/s300_newton_hz_contact_geometry.json). 원자료는 `hz_contact_geometry_summary.json`, `low_hz_attachment_fidelity_audit.json`이다.

## 2. 적응Hz의 실제 명령·시계 확인

`adaptive_bookkeeping_verified`는 한physics실행을 끝까지 새로 수행한 결과다. 저장영상 재생이나 명목계획 계산이 아니다. 이전의 `adaptive_campaign/original_contact_adaptive400_60`도 별도 보존했다.

- 정수master clock으로 phase경계와 최종시간을 관리하며, 실제 실행한dt에 맞게 같은 연속명령을 샘플링한다.
- 각physics step에서 nativeGPU timestep을 업로드한다. phase별 simulated time은 실제 실행한dt의 합이다.
- 종료후 GPU의command/belt버퍼를 읽어 예정된float32명령과 정확히 비교했고, controller tick이실제8,451step과 같은지도 확인했다. 두검사모두PASS다.
- 새저장dt는 `np.diff(time_edges)`로기록한다. 앞선기록의약1e-14초 subtraction차이를 고친 bookkeeping이며 물리dt를 사후에 바꾼것이 아니다.
- 활동feedback은 최근step의접촉상태다. 높은주파수graph의모든중간step에접촉이있었는지에대한누적증거로과장하지않는다.

실제끝시각82.795249939초와명목82.795초차이는float32clock누적의진단값이다. 이결과만으로원본400Hz의궤적과동일하다고판정하지않는다. 전체속도는좋아졌지만마지막벨트0.367RTF의병목은남아있다.

## 3–5. 표현 변경과 GPU 옵션 변경을 구별

그리퍼 proxy, 식물 관절 병합, damping변경은 실행옵션최적화와 다르다. 좌표와반응이달라질수있으므로 각각어떤물리기능이유지되는지기록한다.

그리퍼의묻힌충돌체제거는 원본ID유지대조에서도 같은분리·낙하결과차이가발생했다. 단순geom번호변경만이원인이라는설명은배제한다. 첫큰겹침은이미분리된Fruit08↔거터지만, 다른부착열매/가지반응도원본과달라졌으므로‘겉형상은같아완전히동일물리’라고하지않는다. [그리퍼 개별 보고서](harvest_s300/newton_gripper_variant_ko.md)에compiled collision graph와첫분기시점검사가있다.

4번관절수시험은이전실행의무조건재인용이아니다. 현재원본400Hz와이전baseline이전체기록배열/이벤트에서정확히같고, 각입력MJB와수정후반응fixture의MJB hash가맞는지확인했다. 재사용증거는 `joint_count_independent_reuse.json`이다. `response_v2_baseline/leaves_fixed/coarse2/coarse4`의code SHA256은모두 `281b99d2aa9b6f2d9c15207231979f93e09f8dedbaacd7033d793142fa64d650`이다.오래된pre-step pose fixture는사용하지않는다.

| 표현 | DOF | 원본대비작은하중 최대위치차이 | 최대자세차이 | 제한 |
|---|---:|---:|---:|---|
|잎고정|304|0.556mm|0.091°|잎의개별자유도제거가반응을바꿈|
|분절coarse2|190|0.561mm|0.595°|잎고정과분절병합을포함한표현수준이며순수DOF숫자만의효과가아님|
|분절coarse4|133|0.579mm|1.112°|빠르지만같은물리라는증거가아니며큰접촉에서차이가더커질수있음|

이하중fixture는nativeMuJoCo CPU의idle/주줄기0.2N밀기/과실0.05N당기기/0.002Nm비틀기대조다. 연결을유지한반응진단이며실제수확의분리/부하한계대조를대체하지않는다. 원본부터Tool_R↔리프트플랫폼26.835mm접촉이존재하므로whole-robot물리유효성으로쓰지않는다. 실제수확의GPU접촉결과는각전체 실행에서따로판정한다.

5번의첫잘못된direct계수는DISCRETE를continuous모드처럼가정한준비오류였다. 잘못된두전체 실행을제외하고, 실제설치CUDA `_contact_kbimp`와서버MJB를쓰는검사후수정실행했다. source/direct CONNECT K/B는float32상대차이1.875e-7로맞지만, 전체GPU접촉결과까지정확히같지는않았다. [연결·GPU 개별 보고서](harvest_s300/newton_attachment_gpu_independent_results_ko.md)와 [compact evidence](validation/s300_newton_attachment_gpu_independent.json)에입력·수정이력·실패를모두남겼다.

## 6. native sleep의 확인 범위

처음기본최적화경로에서sleep를켜면기존deterministic bound proof가sleep/compacted solve를지원하지않아시작전거절된다. 이호환성오류를수확실패로세지않았다. sleep지원native backend에서off/on을각각다시완주해기능과효과를비교했다. 따라서off/on약0.5RTF를원본최적화backend1.556RTF와직접비교해‘sleep의속도손해’로계산하지않는다.

source의fruit free-tree만native자동sleep를허용한다. 모든다른로봇/식물tree는항상awake이고, native contact/equality wake-up규칙을쓴다. 실제awake기록에서31,190은열매×physics-step표본수이며시간초나서로다른열매수아니다. 부착중asleep표본은0개다. 잠든열매도질량·중력·충돌체를유지하며삭제/teleport/강제고정하지않았다.

기능이작동해도속도이득은보이지않았다. sleep on/off에서상태·분리수가달랐고원래1mm참고도초과하므로이번결과만으로채택하지않는다. 실제입력과결과는 `released_sleep_native_campaign/native_sleep_off_reference`, `native_fruit_sleep_on`에있다.

## 7. GPU 옵션의 작은 속도차이를 과장하지 않음

| 독립 옵션 | 전체RTF | 상태/물리배열/이벤트 | 결과 |
|---|---:|---|---|
|AABB filter만|1.560|원본과 전체 정확히 같음|0.5%미만단일 측정 차이|
|factor cache만|1.544|원본과 전체 정확히 같음|더느림.51,176solve중8,312factor재사용|
|graph batch10→20|1.563|원본과 전체 정확히 같음|0.5%미만단일 측정 차이|
|constraint capacity256→128|1.561|원본과 전체 정확히 같음|0.5%미만단일 측정 차이|
|contact/broadphase capacity128→64|—|전체미완료|후보 pair65>64에서BROADPHASE overflow.부분RTF를성공성능에서제외|

기존 strict comparator는 의도적으로 다른 실행 옵션도 동일해야 PASS이므로 false일 수 있다. 이 metadata 차이와 실제 물리 배열/전체 coverage를 나란히 기록했고, tolerance를 늘려 같다고 만들지 않았다. 모든 설정이 유의하게 빠르다고 결론내리지 않는다.

## 읽는 순서와 미완료 범위

[이전 솔버 비교](S300_NEWTON_SOLVER_COMPARISON_KO.md)는다른솔버/연결기능의지원범위를다룬자료다. 이번 11항목에독립적인새솔버를추가시험했다고중복계산하지않는다.

현재 개별 시험으로유망하거나불리한요인을가를수는있지만, 최적조합·접촉전구간실시간·실물 정확성의완료증거는아직아니다. 선택한조합의새전체 실행, 실제 GUI카메라이동중비교, 검증된경우N환경확장은통합보고서에서별도로완료여부를확인해야한다. 원본과다른모델/정책의통과를원본과완전히같은물리로포장하지않는다.
