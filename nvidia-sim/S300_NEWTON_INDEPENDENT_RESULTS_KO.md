# Newton GPU 개별 시험과 조합 검증 결과

2026-10-17. 브랜치 `codex/s300-newton-independent-tests-20261017`.

## 현재 확보한 결과

11개 대안을 개별적으로 시험한 뒤, 원래의 약82.795초 수확 시퀀스를 **실제 Newton → MuJoCo Warp CUDA**로 끝까지 실행하는 조합 두 개를 확보했다. 전체 Isaac 농장의 외형과 기존 로봇·리프트·기체·벨트 명령을 유지했다. CPU 실행이나 저장 영상 재생을 GPU 물리 결과로 집계하지 않았다.

| 조건 | 전체 RTF | 가장 느린 단계 RTF | GUI 전체 / 가장 느린 단계 | 연결 해제 | 접촉 정책 |
|---|---:|---:|---:|---:|---|
| 원본385DOF,400/60Hz |1.556|0.378|이전 검증 자료 참조|6|기존1mm|
| 원본385DOF,120/60Hz 관찰|3.471|1.167|미측정|7|최대2.929mm; 단순 채택하지 않음|
| 190DOF+fruit proxy,200/60Hz 반복001|3.518|1.015|3.058 / 0.889|5|벨트만2mm, 나머지1mm; 전체 통과|
| 133DOF+fruit proxy,220/60Hz 반복001|3.857|1.132|3.369 / 0.982|6|같은 정책; 전체 통과|

표의1mm/2mm는 감시 쌍의 기준이다. 기존 분리 열매의 바닥·기체·자기 송이 접촉 예외는 유지하며, 이 예외까지1mm 이하였다는 뜻은 아니다.

RTF=시뮬레이션 시간/실제 경과 시간. 반복001은 같은 프로세스에서 모델을 유지하고 초기상태를 완전히 복원한 두 번째 실행이다. 초기 로딩·JIT·graph capture·저장·영상 렌더는 loop RTF와 구분한다. GUI 결과는 별도의 새 물리 실행이며, 카메라를 실제로 움직였다. 짧은 성공 구간이나 중단된 prefix의 속도를 전체 완료 성능으로 사용하지 않았다.

권장 조건의 physics는 접촉220Hz/이동60Hz이고, GUI 표시 목표는15FPS다. 저장 비교 영상은10FPS·1배속이다. 표시 프레임 수와 물리 적분 횟수는 별개다.

133DOF는 화면 없는 모든 phase가 실시간 이상이고, GUI의 가장 무거운 벨트 단계도 약0.98배속이다. 190DOF는 관절과 기존 내부 충돌 관계를 더 유지하지만 GUI 접촉 단계는 약0.89배속이다. 따라서 두 조건을 성능/표현 보존의 선택지로 남긴다. 원본385DOF 그대로 접촉 실시간을 확보한 것은 아니다.

단순히 관절을 가장 적게 만든 모델을 추천한 것은 아니다.190DOF도 먼저 전체 검증했으며,133DOF는 GUI 접촉 성능과 비벨트 겹침 여유가 더 좋았기 때문에 별도 실행 preset으로 남겼다. 기존385DOF/400Hz 경로와190DOF 대조도 계속 실행할 수 있다. 다른 장면에서도 이 순서가 유지된다는 보장은 없다.

## 실행

현재 서버/NAS에 보존된 입력으로 권장 조합을 실행한다.

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh gpu-realtime --gui
```

화면 없이 반복 측정:

```bash
./nvidia-sim/run_s300_newton.sh gpu-realtime --repeats 3
```

관절을 더 유지한 대조:

```bash
./nvidia-sim/run_s300_newton.sh gpu \
  /root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/inputs_coarse2_fruit_proxy01 \
  --hz 200 --free-hz 60 \
  --contact-policy /root/farmily_tomato/nvidia-sim/harvest_s300/profiles/belt_strokes_2mm.json \
  --gui
```

기존 `gpu`와 `gpu-fast` 기본값은 유지했다. `gpu-realtime`은 변경 표현과 정책을 명시적으로 선택하는 새 preset이다. 입력이 없는 머신에서는 준비된 입력을 별도로 옮겨야 하며 자동으로 다른 모델을 대신 실행하지 않는다. GUI는 Isaac의 별도 프로세스가 실시간 측정 상태를 받아 표시한다. Isaac 내부의 PhysX를 Newton으로 교체한 공식 extension 구성이나 저장 영상의 실시간 재생이라는 뜻이 아니다.

## 무엇을 변경했고 무엇을 검증했는가

- 개별 시험은 [11항목 표](S300_NEWTON_INDEPENDENT_TEST_MATRIX_KO.md)에 정리했다. 초기부터 여러 변경을 섞어 효과를 추정하지 않았다. 원본 Hz, 이동 Hz, 접촉 기반 적응 Hz, 그리퍼 충돌 표현, 관절 수, 연결/감쇠, 해제 열매 sleep, GPU 실행 최적화, 벨트/꼭지 기준, 재질 K/D, 열기 잔류 정책을 각각 확인한 뒤 조합했다.
- 190/133DOF는 원래385DOF의 식물 좌표 표현을 줄인 별도 모델이다. 질량·관성·원래 충돌 형상 위치는 보존하지만, 관절을 묶으면 강체 내부 충돌 가능성이 달라진다. 190DOF에서는 새로 억제된 pair가 이미 source에서 제외된 관계였고,133DOF에서는 원래 가능했던 내부55pair가 추가로 억제된다. 작은 하중/복원/회전 대조는 보존했으나 실제 식물의 물성 보정은 미완료다.
- fruit proxy는 외향 오차 상한0.1mm인 보수적 형상이다. 실제 컴파일 형상 상한은 약0.086870mm이며, 관절 축·로봇·초기 상태·명령을 그대로 둔 준비 불변량을 확인했다. 이 변경을 원본과 동일한 물리 모델의 가속으로 설명하지 않는다.
- 기존 positive-solref 수치는 보존했다. 하지만 이 모델의 DISCRETE 접촉 행은 dt에 따라 REFSAFE 계수가 달라진다.400→200/220Hz는 단순히 같은 계산을 덜 하는 변경이 아니다. 고주파가 실물 정답이라는 가정도 하지 않았다. 실제 접촉·분리와 기하를 함께 비교했다.
- 새2mm는 **정확히 매핑된 이동 벨트↔열매/꼭지**의 중단 판정만 바꾼다. 닫기, 닫힌 압력 유지, 두 열기 stroke에만 적용한다. 접근·이동·완전히 열린 hold와 상판/지지대/본체/다른 물체는1mm다. 열매 분리3N/0.008Nm, contact mask, margin/gap, 재질 계수, actuator를 이 정책으로 바꾸지 않았다. 실제 탄성 벨트를 계산한 모델은 아니므로2mm의 실물 재현성은 별도 검증 대상이다.
- 처음 시도한 closing-only 및0.5초 잔류 latch는 열기에서 새로 생기는 꼭지 접촉을 허용하지 않았다. 이를 통과했다고 보고하지 않았다. 명시적 opening-stroke band를 별도 opt-in으로 만들고 원래 정책의 동작은 보존했다. 과거 물리 결과와 새 판정의 원자료를 모두 남겼다.
- 190/200의 class 최대값은 other0.997469mm / belt-fruit0.939703mm / belt-pedicel1.674869mm다.133/220은0.799010 /0.847600 /1.304378mm다. 최대1.44mm를 열매라고 설명했던 중간 표현은 정정한다. 실제 해당 최대는 꼭지–벨트였다.
- 낮추기만 하면 좋아지는 것도 아니다.190DOF/190Hz strict 실행은 그리퍼 본체 접촉으로 중단됐고, 제거된06번 열매에 남은 가지가 국소적으로 묻힌 표본이 있었다. 실패를 삭제하거나 faster prefix로 채택하지 않았다.

## GUI와 화면 없는 실행

각 모델에서 headless 두 번과 GUI 한 번을 같은 초기상태·명령·스텝으로 새로 실행했다. 관절 위치/속도, 바디 위치/회전, 접촉/힘/침투, equality 상태, 분리 대상·시점과 전체 phase를 모든 physics step에서 비교했다. 두 모델 모두 전체 배열과 이벤트가 정확히 일치했다. 카메라의 실제 render 후 변환 기록, stopped timeline, 최종 측정 프레임 ACK도 확인했다. GUI가 물리를 두 번째로 적분하지 않는다.

증거: raw root의 `headless190_comparison.json`, `gui190_comparison.json`, `headless133_comparison.json`, `gui133_comparison.json`. CUDA float32 시간 누적과 별도로 저장한 실제 solver step/명령 schedule의 전체 coverage를 확인했다. 사후에 상태 허용오차를 늘려 일치시킨 것이 아니다.

초기 로딩과 저장까지 고려한 사용 시간도 구분한다. 새 명령의133DOF cold headless는 setup8.88초+physics loop21.40초, cold GUI는28.92초+24.48초였다. `invocation.json` 생성부터 최종 `summary.json` 기록까지의 파일시각 차이로 추정한 실행 구간은 각각 약31.7초/57.7초다. 이는 Python import·shell lock 대기와 summary 이후 정리를 제외한 참고치이며, 동기화된 monotonic physics timer를 대신하지 않는다. raw `final_run_wall_window_estimates.json`에 산식과 범위를 남겼다. 초기 로딩이나 NPZ 압축을 숨기기 위해 전체 운영 시간도3.9배속이라고 표현하지 않는다.

권장133DOF의 같은 프로세스 반복001과 새 GUI 반복002를 단계별로 비교한다. 대기·복귀를 포함한 전체21단계의 원래 시간/step 수는 각 `summary.json:phase_performance`와 canonical 검증 JSON에 보존했다.

| 단계 | headless RTF | GUI+카메라 이동 RTF |
|---|---:|---:|
|기체 레일 이동|7.685|6.717|
|리프트 준비|7.548|6.622|
|팔 준비 자세|8.661|7.476|
|접근|2.503|2.173|
|열린 상태 벨트 시작|2.500|2.206|
|첫 닫기|1.481|1.290|
|첫 열기|1.601|1.388|
|재닫기|1.979|1.717|
|마지막 벨트 구간|1.132|0.982|
|최종 열기|1.852|1.610|
|후퇴|2.550|2.221|
|리프트 복귀|7.499|6.434|

## GPU 병렬 확장

배치는 기존 실행기의 native Warp 계산 경로를 사용한다. canonical 단일 환경의 cuSOLVER/dense-Hessian 전용 처리와 연산 순서가 달라, 배치용 N1을 새로 완주한 뒤 모든 world를 그 기준과 비교했다. canonical N1의3.86RTF와 배치 N1의1.91RTF를 같은 수치 경로인 것처럼 합치지 않는다.

| 환경 수 / contact capacity | 환경당 전체 RTF | 합산 sim초/실제초 | GPU 최대 표본 MiB | 모든 world의 전체 N1 일치 |
|---|---:|---:|---:|---|
|1 /128|1.911|1.911|1,383|PASS|
|2 /128|1.704|3.408|1,895|PASS|
|4 /128|1.424|5.695|2,855|PASS|
|8 /128|1.058|8.465|4,817|PASS|
|16 /128|0.701|11.215|8,743|PASS|
|32 /128|0.410|13.129|16,575|PASS|
|40 /96|0.343|13.730|20,447|PASS; 고정 입력의 용량 시험|

새 밀도·장면을 시험할 때 보수적인 출발점은 **32환경/capacity128**이다.40/128은 접촉 정렬의 합계 상한4096을 넘어 초기화 때 중단됐다. 이는 물리 실패나 VRAM OOM이 아니다. 기존 옵션의 capacity96만 바꿔 N1을 다시 실행하고,128의 N1과 모든 기록이 일치하는지 확인한 뒤40/96을 완주했다.40/96은 broadphase3840/3840으로 여유0에 도달하므로 다른 장면에 공통 추천하지 않는다. 실행기 기본128은 유지했다.32→40의 합산 증가는4.58%이며 최대 환경 수만으로 가장 좋다고 판단하지 않는다.

모든 world는 같은 초기 장면·명령의 복제이며 각각6개가 분리됐다. 서로 다른 장면이나 독립적인 world별 명령을 검증한 시험은 아니다.32/128의 마지막 벨트 단계는 환경당0.081RTF,40/96은0.067RTF다. 합산이 빨라도 각 환경의 접촉까지 실시간이라는 뜻은 아니다.

별도 cuSOLVER32-batched 경로는 N1의55.64초에 비벨트06번 가지–고정 본체의1.0219mm 접촉으로 중단됐다. 마지막 국소 형상에는 완전 통과가 확인되지 않았으며, 임계값에 민감한 미해결 중단으로 보존한다. 미실행 후반을 통과로 집계하거나 N8을 추가 시험하지 않았다. 자원·초기화·모든 world 비교·실패 로그는 [GPU 배치 보고서](harvest_s300/newton_gpu_batch_independent_results_ko.md), [검증 JSON](validation/s300_newton_gpu_batch_independent.json), [국소 형상](validation/s300_newton_cusolver_batch_geometry.json)에 있다.

## 같은 입력의 CPU 대조와 실제 접촉

| 동일 입력/Hz | native CPU16thread 전체 RTF | canonical GPU 전체 RTF | CPU 연결 해제 / GPU | CPU의 선택 정책 후처리 |
|---|---:|---:|---:|---|
|190DOF+proxy,200/60Hz|4.013|3.518|5 /5|32개 비예외1mm 초과 event 중27개는 belt band 안,5개는 Fruit07–본체 CAD; 최대1.104mm로 잔여 위반|
|133DOF+proxy,220/60Hz|5.549|3.857|5 /6|4개 비예외1mm 초과 event 모두 선언한 belt band 안; 잔여 위반0|

**새 모델의 단일 환경에서는 CPU가 더 빠르다.** GPU가 CPU보다 빨라졌다고 주장하지 않는다. 이번 목표는 Isaac 화면을 유지하면서 GPU에서 접촉 구간 실시간과 반복 일관성을 확보하고 GPU 배치로 확장하는 것이다. CPU 실행은 대조 자료로만 사용했다. CPU는 실제로 기존1mm `diagnostic_continue`로 끝까지 실행했고, 모든1mm 초과 contact log를 같은 정확한 pair/phase 규칙으로 후처리했다. GPU용 정책을 CPU에서 직접 실행한 것은 아니다. 같은 모델/명령이어도 CPU/GPU 계산 정밀도·순서와 접촉/분리 결과는 달라질 수 있으며,5대6 차이를 정확도 향상으로 해석하지 않는다.

별도의 전체 contact witness 실행은 clean 실행과 상태·기존 물리 metric·분리 이벤트가 정확히 일치했다. 실제 접촉력은 solve 시점에 기록했고, 계측 실행은 clean 속도 순위에서 제외했다.

- 190/200: 최대 belt-pedicel 겹침1.674869mm는 Fruit04가 분리된 뒤의 distal04–왼쪽 belt다. 그때 접촉력5.788N이며, capsule 중심선은5mm 두께 tread의 앞면에서0.849854mm 안쪽이었다. 반경을 포함한 envelope1.674854mm도2.5mm 중앙면을 넘지 않았다. 변형 없는 CAD box 내부에 완전히 묻힌 반경 표본2개는 숨기지 않으며, 선언한 벨트 눌림 근사 범위로 구별한다. 다른 rigid CAD 관통으로 자동 허용하지 않았다.
- 133/220: 최대 belt-pedicel 겹침1.304378mm는 Fruit05 분리 뒤의 distal05–오른쪽 belt이며, 해당 접촉력2.361N이다. 부착 중 과실–벨트 최대력은4.871N으로, 같은 순간의 CONNECT 힘1.218N 및 회전 토크0.008352Nm와 구별한다. 전체 최대10.427N은 이미 분리된 Fruit03–고정 그리퍼 접촉이므로 부착 타깃의 힘이라고 보고하지 않는다.
- d.contact 거리/프레임/해결 힘은 적분 전 solve 시점, native geom/body transform은 step 뒤 갱신 시점이다. `qpos[t]`와 `qpos[t+1]` 재구성 결과를 구분했다. controller의 attached cache는 같은 step의 release 직전 값이며 equality disable과 갱신 시점도 별도다.
- display USD SHA, 물리 MJB SHA, 물리 model-array SHA는 서로 다른 항목으로 기록한다. 표시용 `model_sha256`을 물리 MJB hash로 오해하지 않는다.

상세 힘·재현·CPU 후처리는 별도 [최종 접촉 보고서](harvest_s300/newton_final_contact_review_ko.md)에 연결한다.

## 원자료와 해석 한계

원자료 루트:

`/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests`

기존 결과는 덮어쓰지 않았다. 잘못 준비한 초기 direct-contact 실험, nonfinite60Hz 실행, 전체를 관찰하려고 중단을 비활성화한 diagnostic 실행, 엄격한 새 policy 실행을 구별해 보존했다. 조합12개와 지정 기하 window의 pair별 검토는 [작은 조합 증거](validation/s300_newton_combination_proxy_review.json)에 있다. 지정 window에서 전체 반경 묻힘이 없다는 결과를 장면 전체의 무관통 증명으로 확대하지 않는다.

이 작업은 기존 명령의 실행 성능/일관성 시험이다. **5개 또는6개 연결 해제는 송이10개 전체 수확 성공이 아니다.** 떨어진 열매의 기존 자체 송이/기체/바닥 예외를 유지했으므로 그 예외에서 발생한 큰 바닥 겹침을 정확한 낙하/수거 검증으로 해석하지 않는다. `training_eligible=false`, `harvest_success=null`을 유지한다. RL이나 새로운 수확/경로 알고리즘, 주변 전체 식물의 탄성화는 추가하지 않았다.

전체 단일 환경 근거는 [canonical 검증 JSON](validation/s300_newton_canonical_realtime.json), 접촉/CPU 후처리는 [접촉 검증 JSON](validation/s300_newton_final_contact_review.json)에 있다.

## 표시·영상 문제의 별도 처리

첫 offline 렌더는 도구 환경의 `DISPLAY=:182`를 상속해 Vulkan NVIDIA 장치를 찾지 못했다. CUDA 물리와 실제 GUI 검증은 wrapper가 `DISPLAY=:0`을 명시하므로 영향을 받지 않았다. 실패 로그·중단 이유를 남기고 offline 렌더만 `DISPLAY=:0`으로 재실행했다. 실제 GPU 초기화 오류를 물리 계산이 느린 현상으로 분류하지 않는다.

저장 상태 렌더러도 live viewer의 기존 `prepare_display_meshes`를 재사용해 명시적으로 비활성인 표시 메시의 dormant collision schema를 session layer에서만 정리하도록 맞췄다. enabled collider나 source USD는 바꾸지 않는다. viewport capture가 그래픽 초기화 실패로 영원히 기다리지 않도록60초 제한과 원인 메시지도 추가했다. 원래 저장 상태와 물리 결과는 그대로다. 표시 helper의 기존3개 회귀시험은 Isaac에 포함된 USD 라이브러리 환경에서 통과했다.

## 실제 상태 영상

첫 영상은 전체 농장 외형을 유지한 전체 시퀀스다. 나머지 상세 비교는 같은 top camera이며,40–72초 또는48–68초 clip을 사용한다. 상세 cutaway의 잎·배경 숨김은 렌더 표시만의 변경이다. 영상은 실제 저장 상태를1배속으로 표시하고 작은 자막을 사용한다. raw `index.html`에서 실제48/54초 기준으로 함께 재생할 수 있다.

- [/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/final_session_133/repeat_001/actual_133_220_full_farm_1x.mp4](/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/final_session_133/repeat_001/actual_133_220_full_farm_1x.mp4)
- [/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/final_session_133/repeat_001/actual_133_220_contact_1x.mp4](/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/final_session_133/repeat_001/actual_133_220_contact_1x.mp4)
- [/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/final_session_190/repeat_001/actual_190_200_contact_1x.mp4](/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/final_session_190/repeat_001/actual_190_200_contact_1x.mp4)
- [/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/final_session/repeat_001/actual_gpu_contact_1x.mp4](/root/docker_share/mujoko_debugging_data/20261016_s300_newton_gpu_contact_recovery/final_session/repeat_001/actual_gpu_contact_1x.mp4)
- [/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/hz_original_campaign/original_hz120_observe/actual_original120_contact_1x.mp4](/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/hz_original_campaign/original_hz120_observe/actual_original120_contact_1x.mp4)

## 코드·시험·보존

출발 코드는 `4112bda06f04548df3938bab51a72a1788f7c2ca`이며 기존 사용자 gripper 변경과 대형 자산은 보존했다. 구현은 별도 branch에서 진행했고 기존 학습 데이터는 변경하지 않았다.

- 개별 시험 구현 `8aef380`, 개별 결과/조합 초기 증거 `a0d67d1`.
- 명시적 opening-stroke API `295b348`, 모든 배치 world의 정책 비교 `e8287d5`.
- 새 실행 preset/조합 기하 `2f65bcf`, 단일 환경 검증 `e356c848`.
- 실제 접촉/CPU 후처리/hash 구분 `8c1acaa`, 표시 메시 준비 `48f9732`, capture 무한대기 차단 `0a37924`.
- 배치 NPZ 비교의 반복 압축해제 제거 `1c97bf2`, 모든 배치 증거·용량 경계 `8c3a582`.
- 서버 core 회귀226개 OK(22개 skip), 실제 CUDA attachment3개/contact3개 별도 통과. 이후 변경에 대해 launcher1개, contact policy14개, replay/hash3개, batch comparison7개, benchmark15개, compare17개와 표시 helper3개 등의 해당 회귀를 실행했다. 서로 겹치는 테스트 수를 더해 독립 시험 수처럼 보고하지 않는다.
- 표시 helper는 Newton 전용 Python에 `pxr`가 없어 그 환경에서 import 실패했다. Isaac bundled USD의 Python/공유라이브러리 경로를 적용한 실제 USD 환경에서3개가 통과했다. 이 환경 연결 실패를 식물/경로 실패로 집계하지 않았다.

각 실행의 원래 CLI, source model-array/input/명령 hash, 전체 상태, contact metric, 분리 이벤트, 자원 기록과 실패 로그는 raw root에 있다. Git에는 코드·문서·작은 검증 증거를 저장하고 대형 MJB/상태/MP4는 공유 폴더에 보존한다. 새로운 머신은 공유 자산도 별도로 옮겨야 한다.

마지막 읽기 전용 집계는 원자료78실행을 오류 없이 읽었다. 관찰 완주65개, 중단/미완료10개, 준비 오류로 무효화한2개, nonfinite 실패1개를 구분한다. 관찰 완주 수는 물리 검증 통과 수와 다르다. 준비 실패와 중단 prefix의 속도는 HTML의 전체 완료 속도로 표시하지 않는다. 영상5개의 전체 디코딩도 모두 통과했다. 파일/SHA/시험 결과의 최종 목록은 [통합 검증 JSON](validation/s300_newton_independent_tests.json)에 있다.
