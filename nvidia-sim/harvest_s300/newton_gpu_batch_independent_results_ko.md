# Newton GPU 배치 확장 결과

완료한 시험 중 합산 처리량이 가장 높은 것은 **N=40, 13.730 sim초/실제초**다. 환경 하나의 속도는 **0.343배**이며 접촉 단계까지 실시간이라는 뜻은 아니다. 각 world는 전체 동작을 완주하고 자기 계산 집단의 N1과 상태·접촉·분리 기록이 정확히 일치했다.

선택 모델은 **133DOF + 과실 외측오차 0.1mm 이내 proxy, 접촉 220Hz / 이동 60Hz**다. 실제 Newton → MuJoCo Warp CUDA로 같은 장면·초기 상태·명령을 복제했다. 원본 385DOF나 이전 133DOF/400Hz 결과와 섞지 않는다. `training_eligible=false`, `harvest_success=null`이며 연결 해제는 수거 성공과 다르다.

새로운 밀도·장면을 시험할 때의 보수적 출발점은 **N32 / capacity128**이다. **N40 / capacity96은 이 고정 입력·trace의 용량 탐색**이며 broadphase 여유가 0인 순간이 있다. 완주하더라도 임의 장면에 안전한 기본값으로 추천하지 않는다. 실행기 기본 capacity128은 그대로다.

## 계산 경로와 접촉 정책

Canonical 단일 환경의 cuSOLVER inertia·dense Hessian·fixed reductions는 N1 전용이다. 배치는 기존 `preset=batched`, native Warp inertia/선형대수, fixed tree sums/contact order, `--deterministic-scope constraints`를 사용한다. 연산 순서가 다르므로 새로운 배치 N1을 먼저 검증했으며 canonical N1의 성공으로 대신하지 않았다.

정확한 벨트–목표 과실/꼭지 쌍은 닫기·재닫기·닫힌 압력 유지·열기 스트로크에서만 2mm를 허용한다. 접근·운반·완전히 열린 유지에서는 원래 감시 범위를 유지하며, **기존 분리 열매 접촉 예외를 제외한 다른 감시 쌍은 1mm**다. 실제 충돌·힘·마찰·분리 3N/0.008Nm는 바꾸지 않았다. 원래 1mm 진단도 보존한다. 정책 통과를 실물 물리 정확도 검증으로 확대하지 않는다.

## 계산 집단 1

Native Warp inertia/constraint factorization with existing fixed tree sums, contact order and constraints-scope determinism; distinct from canonical cuSOLVER/dense-Hessian/fixed-reduction single-world arithmetic.

기준: `/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/batch_n1/native133_hz220_n1`

| N | 전체 상태 | 환경당 RTF | 합산 sim초/초 | 닫기 RTF | 마지막 벨트 RTF | 분리수 | 최대 감시 mm | 자기 N1 전체 일치 | GPU 최대 MiB |
|---:|---|---:|---:|---:|---:|---|---:|---|---:|
|1|완주|1.911|1.911|0.704|0.497|6씩|1.310|PASS|1383|
|2|완주|1.704|3.408|0.613|0.425|6씩|1.310|PASS|1895|
|4|완주|1.424|5.695|0.498|0.334|6씩|1.310|PASS|2855|
|8|완주|1.058|8.465|0.360|0.231|6씩|1.310|PASS|4817|
|16|완주|0.701|11.215|0.231|0.143|6씩|1.310|PASS|8743|
|32|완주|0.410|13.129|0.133|0.081|6씩|1.310|PASS|16575|
|40|초기화: capacity 범위 초과|—|—|—|—|—|—|미완료|—|

| N | 초기화 초 | graph capture 초 | 물리 loop 초 | 파일 mtime 창 초(근사) | 최저 단계 RTF | CPU 평균 % | RSS 최대 GiB | GPU 평균 % |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|1|8.89|2.16|43.326|53.983|0.497|99.2|1.34|95.6|
|2|8.97|2.14|48.602|59.791|0.425|99.1|1.34|95.8|
|4|8.96|2.16|58.159|70.260|0.334|99.1|1.34|76.5|
|8|9.04|2.17|78.259|92.221|0.231|99.1|1.34|93.4|
|16|9.20|2.19|118.139|135.807|0.143|99.1|1.35|90.2|
|32|9.29|2.20|201.833|226.762|0.081|99.0|1.39|83.5|

## 계산 집단 2

Existing cusolver32-batched constraint factorization with native Warp inertia; distinct from both canonical single-world full hooks and native Warp factorization.

기준: `/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/batch_scaling_cusolver133_n1/cusolver133_hz220_n1`

| N | 전체 상태 | 환경당 RTF | 합산 sim초/초 | 닫기 RTF | 마지막 벨트 RTF | 분리수 | 최대 감시 mm | 자기 N1 전체 일치 | GPU 최대 MiB |
|---:|---|---:|---:|---:|---:|---|---:|---|---:|
|1|contact_policy_violation|—|—|—|—|—|—|미완료|—|

| N | 초기화 초 | graph capture 초 | 물리 loop 초 | 파일 mtime 창 초(근사) | 최저 단계 RTF | CPU 평균 % | RSS 최대 GiB | GPU 평균 % |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|

## 계산 집단 3

Native Warp arithmetic, same fixed input/trace; buffer nconmax96 only. NewN1 verified exactly equal to native capacity128 N1, thenN40 compared against96N1. Specific capacity probe; broadphase has zero headroom at peak.

기준: `/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests/batch_scaling_native133_capacity96_n1/native133_hz220_n1_capacity96`

| N | 전체 상태 | 환경당 RTF | 합산 sim초/초 | 닫기 RTF | 마지막 벨트 RTF | 분리수 | 최대 감시 mm | 자기 N1 전체 일치 | GPU 최대 MiB |
|---:|---|---:|---:|---:|---:|---|---:|---|---:|
|1|완주|1.916|1.916|0.707|0.500|6씩|1.310|PASS|1371|
|40|완주|0.343|13.730|0.110|0.067|6씩|1.310|PASS|20447|

| N | 초기화 초 | graph capture 초 | 물리 loop 초 | 파일 mtime 창 초(근사) | 최저 단계 RTF | CPU 평균 % | RSS 최대 GiB | GPU 평균 % |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|1|8.91|2.14|43.221|53.883|0.500|99.2|1.34|84.0|
|40|9.20|2.18|241.252|269.268|0.067|99.1|1.39|85.9|

## 실패와 용량 경계

기본 capacity 128에서 N40은 physics 시작 전에 중단됐다. 기존 deterministic contact order가 전체 contact capacity 4096까지만 지원하는데 40×128=5120이기 때문이다. 관통 실패나 VRAM OOM으로 분류하지 않으며 속도에도 포함하지 않는다. 원래 실패 로그를 그대로 보존했다.

복구는 기존 옵션 `--nconmax 96`만 사용했다. 40×96=3840이 지원 범위에 들어온다. 먼저 새 N1/96을 완주하고 기존 N1/128과 모든 상태·물리·정책 배열의 정확한 일치를 확인한 뒤에만 N40/96을 실행했다. 분리조건·물성·명령·충돌 정책이나 overflow guard는 바꾸지 않았다. 단, N1의 broadphase 후보수는 96까지 도달한다. 기존 128 실행도 같은 시점에 96이며 상태는 같지만 이 장면에서 버퍼 여유가 없으므로 다른 장면에 공통 적용할 설정으로 일반화하지 않는다.

별도 cuSOLVER32-batched N1은 9127스텝 중 5077스텝에서 정책 중단했다. 최초 감지 tick 5072는 `Truss_00_Pedicel_distal_06_0`의 `g0166`와 `Tool_L`의 `s300_g0620` 접촉이다. 이 쌍은 벨트가 아니어서 1mm가 유지되며 GPU 감지 깊이는 약 1.0219mm다. 접촉 임계값에 민감한 미해결 실패로 보존한다. 마지막 약 0.15초 기하 검사에서 완전 매몰이나 축의 외부→외부 관통은 확인되지 않았으나 미실행 suffix의 통과를 뜻하지 않는다. 자기 N1을 통과하지 않아 해당 집단 N8은 실행하지 않았다. 중도 구간 RTF 2.363은 전체 처리량으로 사용하지 않는다. [별도 기하 검사](../validation/s300_newton_cusolver_batch_geometry.json)를 함께 본다.

16→32의 합산 처리량은 17.07% 늘었다. 측정 메모리 선형 추정은 N40 20,491MiB, N48 24,407MiB, N64 32,239MiB다. 장치 용량은 24,564MiB다. N48은 물리 용량보다 조금 작지만 여유가 거의 없고 이번 22GiB 시험 예산을 넘는다. N64는 물리 용량을 넘으므로 둘 다 실행하지 않았다. 시험한 최대와 이론적인 최대 world 수를 혼동하지 않는다.

## 기록과 비교 범위

GPU 공유 lock으로 실제 계산을 직렬화한다. RTF는 실제 GPU 시뮬레이션 clock / 물리 loop 경과시간이며 로딩·JIT·graph capture·잠금 대기·압축 저장·영상 렌더를 섞지 않는다. 초기화에는 graph capture가 포함되므로 두 시간을 합산하지 않는다. CPU 100%는 논리 코어 한 개다. GPU 메모리/사용률은 장치 전체의 표본이며 독점 할당량이나 연속 측정한 절대 peak가 아니다.

파일 mtime 창은 summary mtime−invocation mtime의 벽시계 근사다. 초기화·physics loop·최종 D2H/NPZ 압축/보고 저장 일부를 포함한다. 이전 Python import·잠금 대기·summary 이후 정리는 포함하지 않는다. 단조시계 측정이 아니므로 성능 순위에 쓰지 않는다. 물리 loop RTF만으로 최초 실행 대기 전체를 표현하지 않는다.

모든 world의 매 physics-step qpos/qvel/eq_active·실제 timestamp, 접촉·힘·관통·분리 배열과 break event를 보존했다. 명시 정책 config/input SHA·스냅샷·class-depth/first-event/허용 pair-step/residual 배열도 각 world마다 자기 N1과 정확히 대조했다. GPU 버퍼의 실제 float32 명령/벨트속도·hash·tick, native float32 clock 누적, 원래 분리조건 최초 초과와 equality 해제 시점도 읽기 전용으로 확인했다. 벨트 동작 전 조기 분리는 없다.

각 N은 한 번씩 측정했다. 근소한 속도 차이의 통계적 유의성이나 전역 최적 조합을 증명하지 않는다. 같은 입력 복제의 정확한 일치를 확인한 결과다. 서로 다른 장면이나 독립된 제어명령을 배치로 실행하는 검증은 아직 하지 않았다. float grouping 차이는 정확한 일치로 보고하지 않으며 차이만으로 환경간 충돌이라고 단정하지도 않는다. N 증가로 합산 처리량이 높아져도 각 환경의 접촉 단계는 느려진다.

오프라인 comparator의 NPZ 중복 압축해제만 줄인 커밋은 `1c97bf2`다. 7개 단위시험과 실제 이전 N1→N8 비교가 통과했고, 이전 비교 결과와 구조적으로 완전히 같다. 물리 실행 코드는 바꾸지 않았다.

## 재현과 영상

새 파이프라인 없이 기존 `harvest_s300/benchmark_solver_compare.py`에 raw manifest와 새 `--output`을 지정한다. 기존 결과 덮어쓰기는 거부한다. 코드 hash·실제 argv·입력 hash는 Git 보존 JSON과 각 invocation에 있다. 이번 실행은 화면 없는 배치 계산이며 N-world GUI를 검증하지 않았다. 별도 배치 영상은 만들지 않았다. 실제 저장 상태의 단일 환경 영상 및 GUI/headless 비교는 [통합 결과 보고서](../S300_NEWTON_INDEPENDENT_RESULTS_KO.md)에 있다.

원자료: `/root/docker_share/mujoko_debugging_data/20261017_103107_newton_independent_tests`

- `batch_scaling_native133_evidence.json`: 전체 개별 결과·입력/코드 SHA·분리조건 검사.
- `batch_scaling_native133_n*_comparison.json`: native 집단의 전체 world 비교.
- `batch_scaling_cusolver133_evidence.json`: 별도 cuSOLVER 실패와 미실행 범위.
- `batch_scaling_native133_capacity96_evidence.json`: 버퍼96 N1→N40과 기존128 N1 대조.
- `batch_scaling_resource_decisions.json`: 측정 메모리 추정과 후속 실행 결정.
- [작은 Git 보존 요약](../validation/s300_newton_gpu_batch_independent.json).
