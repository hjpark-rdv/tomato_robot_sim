# Newton GPU 배치 확장 검증 · 2026-10-17

실제 Newton → MuJoCo Warp CUDA에서 **동일한 수확 장면을 1/2/4/8개 동시에 계산**했다.
8환경의 합산 처리량은 1환경 배치 경로 대비 4.36배였으며, 모든 환경의 전체 상태와
접촉·분리 기록이 1환경 기준과 정확히 같았다. 다만 8환경 각각은 평균 0.669배속이고,
접촉 구간은 더 느리다. **실시간 단일 수확과 자원 한도까지의 최대 병렬 수를 달성한 결과는 아니다.**

이번 표는 `inputs_coarse4`의 **과실 메시를 바꾸지 않은 133DOF 모델**이다.
별도 최종 보고서가 권장하는 보수적 0.1mm 과실 proxy 모델의 N환경 성능으로 해석하면 안 된다.
결과는 SIM-GT 진단이며 `training_eligible=false`, `harvest_success=null`이다.
6개 분리는 조인트 분리 기록이며 수거 완료·전체 송이 수확 성공 판정이 아니다.

## 입력과 측정 범위

- 장비: RTX3090Ti 24GB, Threadripper3960X. Newton1.7.0 / MuJoCo3.15.0 /
  MuJoCo Warp3.15.0 / Warp1.17.0.
- 132body / 143qpos / 133DOF / 901collision geom / 10actuator / 10열매 연결.
  원본385DOF와 같은 운동 방정식이라고 주장하지 않는다. 별도 정적·동적 응답 검사로
  평가한 줄기 관절 축약 모델이며, 원본 외형·질량·충돌체는 보존한다.
- 기존 타깃 송이·자기 주줄기 충돌 정책, 잎 무시 및 분리 열매 처리,
  활성 겹침1mm, 분리3N/0.008Nm, 물성·마찰·actuator를 유지했다.
- 고정된 동일 명령의 전체13,806step, 명목82.795초를 완료했다.
  접촉400Hz / 비접촉 이동60Hz이며 float32 누적 실제 clock은82.793121초다.
- 표의 RTF는 실제 GPU 누적 sim 시간82.793121초 / 실제 물리 loop 시간이다. 명목 명령표82.795초와의 clock 차이는 따로 보존한다. 로딩·JIT·입력 준비·잠금 대기·저장·
  별도 영상 렌더를 제외한다. 단계별 시간은 해당 물리 loop 내 측정이다.
- 모든 GPU 작업은 `/tmp/s300_solver_compare_gpu.lock`의 배타적 잠금을 사용했다.

| 입력 항목 | SHA256 |
|---|---|
| `input_model.mjb` | `e7e7a3d494e18350dc7bcb2795ffc7881c81ff03bc11fc074de7f4389adb2304` |
| 런타임 source model 배열 | `160360ec75d9f0dda0d5533fc6820346a5aa79d04f0dbdf9b132aae10ff372f7` |
| 실제 실행 명령 `executed_commands.npz` | `d187adac43e87b05c8d1e6588bd094f9260451eaab2456f80d7984176350da0a` |
| `inputs.json` | `869543a3d018e295fa03990ac8ff32667267c550f5bd77c85983692d41f2e2e7` |

## 실제 GPU N환경 결과

| 환경 수 | 물리 loop초 | 환경당 RTF | 합산 sim초/실제초 | 환경별 분리 | 활성 최대 겹침 | 전체 N1 비교 |
|---:|---:|---:|---:|---|---:|---|
|1|67.361|1.229|1.229|6|0.683mm|기준|
|2|75.731|1.093|2.187|각6|0.683mm|PASS|
|4|91.265|0.907|3.629|각6|0.683mm|PASS|
|8|123.669|0.669|5.356|각6|0.683mm|PASS|

전체 비교는 각 world의 모든13,807개 qpos/qvel/연결 상태 표본과13,806개 timestamp,
기존8개 접촉·힘·겹침·분리 지표 배열, 원래 물리 break event를 확인했다.
N2/N4/N8의 각 world가 native Warp N1과 **전체 구간에서 정확히 같다**.
연결된 대상과 분리 개수만 같은 비교가 아니다. 모든 환경의 finite·capacity·qacc·활성 겹침
검사를 통과했다. 같은 장면을 복제한 검증이며 서로 다른 장면·개별 명령의 병렬 검증은 미실시다.

| 환경 수 | 접근 | 닫기 | 첫 열기 | 재닫기 | 마지막 벨트 | 후퇴 |
|---:|---:|---:|---:|---:|---:|---:|
|1|0.680|0.386|0.401|0.515|0.281|0.702|
|2|0.620|0.339|0.348|0.450|0.240|0.628|
|4|0.529|0.278|0.281|0.368|0.188|0.535|
|8|0.407|0.201|0.200|0.267|0.131|0.406|

각 셀은 **환경당 단계 RTF**다. 빠른 이동 구간 때문에 전체 평균이1배속이어도
닫기·벨트는 실시간이 아니다. N8은 처리량 측면에서 유리하고, 사람이 한 환경을 실시간으로
확인하는 용도에서는 검증된 단일 환경 전용 최적화 경로가 더 빠르다.

이 배치 경로는 native Warp 선형대수와 제약 결정성 모드를 사용한다.
단일 환경 전용 cuSOLVER·dense Hessian·고정 reduction hook을 사용하는 canonical 경로와
산술 순서가 다르다. 같은 `coarse4`의 canonical N1은2.436RTF였다.
따라서 N8 합산5.356은 canonical N1 대비2.20배이며, 4.36배는 **native 배치 경로 N1 대비**다.
두 수치를 섞거나 native N배치와 canonical N1의 정확한 궤적 일치를 주장하지 않는다.

## 자원과 추가 선형대수 시험

| 환경 수 | CPU 평균% | RSS 최대 GiB | GPU 평균% | GPU 메모리 관측 최대 MiB | 샘플 수 |
|---:|---:|---:|---:|---:|---:|
|1|99.0|1.39|87.0|1411|13|
|2|99.1|1.36|80.4|1927|14|
|4|99.1|1.34|86.2|2887|17|
|8|99.1|1.35|93.6|4879|24|

물리 loop 중 약5초 간격으로 샘플링했다. CPU100%는 논리CPU1개이며, CPU16개를
이 표의 단일환경 계산에 쓴다는 뜻이 아니다. GPU 수치는 데스크톱 등 다른 유지 프로세스를
포함한 장치 전체 값이다. 전용 프로세스 할당량이나 연속 관측한 절대 peak가 아니다.
별도 controller 자원 로그에는 잠금 대기·준비·저장도 포함되므로 위 값과 구분한다.
N8은 이번 시험 예산의 상한이다. 관측 GPU 메모리가 약4.9GB이므로 최대 메모리 한도를
찾았다거나 N8이 장비의 절대 최대라고 주장하지 않는다.

동일 `coarse4`에서 native inertia를 유지한 **batched cuSOLVER N1**도 전체 실행했다.
72.010초 / 1.150RTF / 6개 분리 / 최대0.707mm로 물리 검사를 통과했지만,
native Warp N1보다 wall time이6.9% 늘었다. 조건부 추가 예산의 전제인 N1 개선이 없어
N4 추가 실행은 하지 않았다. N>1 batched cuSOLVER가 느리다는 결론은 미검증이다.

## baseline 계측과 채택하지 않은 CG

원본385DOF canonical GPU를 다시 측정한 baseline은53.226초 / 1.556RTF,
6개 분리 / 최대0.794mm였다. GPU event 계측 실행은54.667초였고 물리 배열과 이벤트는
정확히 같았다. 다만 이 두 실행 사이 계측 코드의 Nworld metadata가 바뀌었기 때문에
엄격한 비교 JSON의 전체 판정은 metadata 항목에서 FAIL이다. 이를 정식 전체 PASS로
집계하지 않았다. 현재 기본 N1 metadata는 보존하도록 수정했다.

`coarse2` 계측은 원래 실행과 엄격한 전체 비교 PASS였다.
대표 닫기 표본의 solver 시간은 baseline4.393ms→coarse2 2.152ms,
collision은1.402ms→1.418ms였다. 마지막 벨트 표본은 solver4.959ms→3.370ms,
collision1.482ms→1.374ms였다. 서로 다른 표현이 만든 실제 다른 상태의 대표 표본이므로
이를 동일 상태의 커널만 바꾼 속도 비교라고 해석하지 않는다.

CG는 명시적 opt-in으로만 열었으며 기본 Newton 제한과 검사를 유지했다.
작은12DOF CONNECT의 native C/CPU 및 CUDA 한 스텝 대조 시험은 통과했다.
그러나 전체 식물에서 허용한 두 파일럿은 모두 초기 settle 중 중단됐다.

| CG 파일럿 | 실제 실행 | 첫 주요 문제 | 최대 활성 겹침 |
|---|---|---|---:|
|coarse2|30step / 0.5초|step1의 Fruit02 torque분리, 이후 구조물 접촉|14.122mm|
|coarse4|20step / 0.333초|step10 Rachis4–Fruit02 접촉, 접근 전 분리|1.862mm|

CG native Warp inertia와 cuSOLVER inertia의 첫3step 대조에서도 거의 같은 조기 분리가
발생해 **cuSOLVER inertia만의 오류라는 가설은 배제**했다. Native C CG의 첫3step은
분리 없이 안정적이었다. CPU/GPU 정밀도·허용오차 및 큰 모델 조건수 차이 중 정확한 원인은
완전히 분리하지 못했다. 모든 CG가 부적합하다고 일반화하지 않고 이번 두 모델 설정을
미채택했다. 짧은 대조 실행의 `completed`를 전체 수확 완주나 처리량으로 집계하지 않는다.

## 재현과 원자료

기존 실행기를 재사용하는 [benchmark_solver_compare.py](harvest_s300/benchmark_solver_compare.py)가
입력 검증·배타적 잠금·호출 인자 저장·집계를 담당한다. 새 물리 실행 파이프라인은 없다.
실제 배치 설정은 `preset=batched`, `--deterministic-scope constraints`,
`--all-world-trace`다. N1 전용 solver hook을 N환경에 잘못 적용하지 않는다.

저장된 N4 manifest로 **새 폴더**에 재실행하는 예:

```bash
cd /root/farmily_tomato
./nvidia-sim/.newton-venv/bin/python \
  nvidia-sim/harvest_s300/benchmark_solver_compare.py \
  /root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/coarse4_native_n4_manifest.json \
  --output /root/docker_share/mujoko_debugging_data/new_coarse4_native_n4_campaign
```

이 manifest는 실제 서버의 prepared input을 참조한다. Git 코드만으로 대형 모델·명령 자산이
포함됐다고 가정하지 않는다. 결과 폴더가 있으면 덮어쓰기를 거부한다.
`--summarize`는 기존 결과를 읽어 CSV를 만들고, `--compare-worlds N1_RUN N_RUN`은
전체 상태/접촉/이벤트 비교를 새 폴더에 저장한다.

- 원자료 집계:
  [/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/native_batch_scaling_evidence.json](/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/native_batch_scaling_evidence.json)
- N2 전체 비교:
  [/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/coarse4_native_n2_comparison_complete/comparison.json](/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/coarse4_native_n2_comparison_complete/comparison.json)
- N4 전체 비교:
  [/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/coarse4_native_n4_comparison/comparison.json](/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/coarse4_native_n4_comparison/comparison.json)
- N8 전체 비교:
  [/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/coarse4_native_n8_comparison/comparison.json](/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/coarse4_native_n8_comparison/comparison.json)

회귀 시험은 deterministic/CG14개 + controller6개의 **중복 없는20개 PASS**다.
별도 CUDA에서는 N2/N4의 per-world row 순서·빈 world·반복 graph 시험과12DOF CG 실제
CONNECT 대조를 통과했다. 이전 혼합 시험 로그의 중복 항목을 전체 수에 다시 더하지 않는다.
Controller6개는 마지막 CPU 계측 표기 수정 후에도 다시 통과했다.
짧은3step 실행을13,806step 전체 완주로 오인하지 않는 회귀시험과, world1만 다른 상태를
탐지하는 시험, viewer용 metadata가 다른 이벤트와 원래 물리 이벤트를 구분하는 시험을 포함한다.

이번 배치 전용 시험에서는 영상을 새로 만들지 않았다. 실제 단일 환경 영상과 GUI/headless
비교는 [주 실험 보고서](S300_NEWTON_SOLVER_COMPARISON_KO.md)에 모았다.
미완료 사항은 **단일 환경 접촉 전 구간 실시간**, 서로 다른 scene/명령의 GPU 병렬 검증,
Nworld GUI 검증, 그리고 N8을 넘어선 자원·처리량 한도 측정이다.
