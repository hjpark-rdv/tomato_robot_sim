# Isaac Sim 공식 Newton 연결 진단

2026-10-17. `codex/s300-newton-solver-compare-20261017`의 비교 후보 B다.

공식 `isaacsim.physics.newton` 경로를 원래 S300 USD 자산으로 시험했다. 이 경로의
MuJoCo 설정은 독립적인 새 솔버가 아니라 Isaac의 USD import·Fabric·제어 연결 방식이다.
독립 실행기의 Newton1.7/Warp1.17 환경과 Isaac6.1 동봉 Newton1.5/Warp1.16 환경을
서로 덮어쓰지 않았다.

## 입력과 보존

원본:

`/root/docker_share/mujoko_debugging_data/20261011_194707_plant_interaction_comparison/D_target_only/composed.usdc`

SHA256: `e5697c6111478627272ca1d6e9f62fc0a72d195bce64707223def4eaaa9d9971`.

`native_newton_prepare.py`는 원본을 sublayer로 참조하는 새 USD를 만든다.
원본 파일의 전후 해시를 비교하고, 원본에는 저장하지 않는다. 기존 출력 폴더 덮어쓰기도
거부한다. 변환 중 오류가 난 첫 `prepared/` 폴더도 진단 기록으로 보존했다.

진단용 derivative의 변경은 다음과 같다.

- 표시 전용 Gprim1,975개를 비활성화한다. body·joint·활성 collider는 제거하지 않는다.
- 원본의 collider/body/joint가 전혀 없는 `/World/Greenhouse` 표시 subtree를 물리 전용
  derivative에서 비활성화한다. 원래 전체 농장 표시 자산은 그대로 남는다.
- convexDecomposition mesh781개는 per-mesh convexHull로 바꾼다. 이는 기존 portable
  MuJoCo adapter가 이미 사용한 근사 방식이다. 원본 PhysX convex decomposition과
  동일하다는 뜻은 아니다.
- 원본 body136개, joint393개 중 enabled131개, enabled collider937개를 보존한다.

현재 authoritative portable MJB는 `nbody=132`, `nv=385`, `ngeom=901`, `neq=10`이다.
원본 USD의 body/joint/collider 정의와 이 MJB의 정의·역사적 loose-fruit 제외·ground
표현 등이 다르다. 따라서 이 raw USD 진단의 속도를 동일 모델 수확 속도와 직접 비교하지
않는다. 이 파일은 모델 차이를 숨겨 순위를 매기는 용도로 사용하면 안 된다.

## 실행과 관찰

결과 root:

`/root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/native/`

| 시험 | 변경 | 관찰 |
|---|---|---|
| native_hulls_mujoco | 표시 Gprim 제거 + mesh hull, 기본 renderer | 약155초에 외부 중단. source-open 단계, physics0스텝 |
| native_hulls_minimal | 추가로 비물리 greenhouse subtree 제외, 공식 MinimalRendering, headless viewport-off | bootstrap30.966초, source-open0.988초. 공식 Newton parser/finalize까지 진행. 180초 예산 내 physics0스텝, timeout124 |
| native_hulls_instrumented | 같은 입력·설정, 주기적 Python stack 기록 | 초기화32.033초. 첫 CUDA convex kernel 컴파일 중180초 예산 종료, 완료physics0스텝 |
| native_unordered_bodies | source joint 수정 없이 누락된 loop endpoint body/root 복원 | 초기화34.499초. 실제GPU120스텝/0.3sim초 완료, 전체수확 미검증 |

두 번째 시험의 native parser 로그는 역사적으로 분리된 RouteFrozen 열매의 filteredPair
일부가 import되지 않았다는 경고와, 작은 꼭지 관성의 자동 보정 경고를 포함한다. 이를
그대로 보존했다. 표시 로딩을 줄여 source-open 병목은 해결했지만, 그것이 수확 실행이나
물리 동등성의 통과를 의미하지 않는다.

세 번째 시험에서 실제 구현 오류를 발견했다. 기본 `bodies_follow_joint_ordering=True`
설정은 body 생성을 joint graph 순서까지 미룬다. source의 target-fruit D6 joint는
`physics:excludeFromArticulation=True`이므로 graph에서 빠졌고, 그 loop joint에만 연결된
열매 body10개는 생성되지 않았다. collider10개는 exported MJCF의 worldbody로 들어갔다.
질량이나 rigidBodyEnabled가 없는 자산이 아니었다. 이 상태는 수확용으로 사용할 수 없다.

`--preserve-unordered-bodies`는 public parser 옵션을 False로 설정하고, 원래 loop joint에만
연결되어 tree/world-root가 없는 양의 질량 body에 free world root를 생성한다. 설치 패키지나
source joint의 exclude flag/limit를 변경하지 않는다. 기존 별도 D6 joint는 그대로 남아
SolverMuJoCo의 CONNECT/WELD로 변환된다. 수정 후 target fruit10개 모두 body mapping과
동적 body에 복원되었다. S300Carrier의 별도 fixed-loop endpoint도 복원하여 총136body,
147joint,451NewtonDOF,937활성collider가 되었다. 전체 shape1720개는 hull 생성 전 표시
shape도 포함하므로 활성 collider 수와 다르다. 누락된 target fruit가 있으면 최신 probe는
물리 실행 전에 오류로 중단한다.

네 번째 실행은 bootstrap31.320초/open0.976초/init34.499초 후 실제120스텝을 완료했다.
첫 스텝62.410초는 cold CUDA kernel compile, 두 번째0.0974초는 graph capture를 포함한다.
이 두 스텝을 별도 제외한118스텝의 평균7.360ms, 중앙값7.071ms, P95=9.810ms이며
400Hz 기준 warm physics RTF는0.340이다. raw loop63.409초/0.3sim초=0.00473이다.
이 수치는 idle 연결 진단이며, 다른 입력·제어를 사용하는 전체 수확 RTF와 비교하지 않는다.

idle에서 active fruit 최대34.58mm, target plant의 leaf127.33mm, robot/tool1.143m,
historical released fruit445.17mm 이동을 기록했다. source drive/initial pose와 현재 controller,
feedforward/preload가 연결되지 않은 상태다. 유한 상태를 저장했지만 물리 유효성 통과나
수확 동작으로 인정하지 않는다. 1mm penetration/contact force 판정도 이 idle probe에서는
평가하지 않았다. 상세 수치와 원자료 경로는 `native/summary.json`에 있다.

외부 py-spy attach는 container의 ptrace 권한 제한으로 실패했다. 서버 권한을 바꾸지
않고, Python 기본 faulthandler의 `--stack-dump-seconds`를 추가했다. 외부 시간 제한은
각 실행에 적용되며, GPU lock 대기 시간은 실행 예산에서 제외한다.

## 수확 의미의 미연결 항목

원본에는 authored breakForce/breakTorque 각47개, belt surfaceVelocity와 local-space
속성 각2개가 있다. 설치된 공식 Newton importer/stage 소스에서 이 PhysX 속성을
처리하는 코드를 찾지 못했다. 단순 source import가 기존 벨트와 파단 동작을 이어받는다고
가정하지 않는다. 고정 preload, 현재 qfrc_bias 기반 feedforward, passive fruit spring,
3N/0.008Nm 분리, 분리 후 접촉 정책은 기존 실행기 adapter와 연결해야 한다.

이 진단에서는 원본 USD의 관절을 제거하거나, 물성을 약하게 하거나, 열매를 강제로
분리해 import 성공을 만들지 않았다. `training_eligible=false`, `harvest_success=null`이다.
idle 또는 import 진단만으로 전체 수확 성공/RTF/GUI 일치를 주장하지 않는다.

## 재현

OpenUSD를 독립적으로 사용할 때 이 서버에서는 Isaac의 동봉 USD 경로를 명시한다.

```bash
cd /root/farmily_tomato
PYTHONPATH=/root/isaac-sim-6.1.0/extscache/omni.usd.libs-1.0.3+00c488ae.lx64.r.cp312 \
LD_LIBRARY_PATH=/root/isaac-sim-6.1.0/extscache/omni.usd.libs-1.0.3+00c488ae.lx64.r.cp312/bin \
/root/isaac-sim-6.1.0/python.sh nvidia-sim/harvest_s300/test_native_newton_prepare.py
```

`python.sh`만 실행하면 SimulationApp 시작 전에는 `pxr`가 검색되지 않는다.
위 두 경로는 앱을 켜지 않고 동봉 OpenUSD 라이브러리만 사용하는 CPU 시험에 필요하다.
나머지 5개 importer 시험은 별도 Newton 환경에서 실행한다.

```bash
cd /root/farmily_tomato
nvidia-sim/.newton-venv/bin/python nvidia-sim/harvest_s300/test_native_newton_import.py
```

3개 보존 시험은 original SHA, body/joint/활성 collider 보존, 표시 subtree 제거 시 정적
장애물 유지, material GeomSubset 무효화 처리, 출력 덮어쓰기 거부를 확인한다.
추가 `test_native_newton_import.py`5개는 scoped override 복구, 기존 root 보존 및 loop-only
free root 복원을 확인한다. 실제 Newton1.7→MuJoCo CPU fixture에서는 free fruit6DOF와
CONNECT1개가 생성되고, 연결 중에는 중력에 유지되며 constraint를 끄면 낙하함을 검사했다.
이 단위 fixture는 native full harvest나 GPU 속도의 증거가 아니다. 합계8개PASS다.

새 출력 경로로 실행한다.

```bash
cd /root/farmily_tomato
flock -x -w 900 /tmp/s300_solver_compare_gpu.lock \
  timeout --signal=TERM --kill-after=20 180 \
  /root/isaac-sim-6.1.0/python.sh nvidia-sim/harvest_s300/newton_probe.py \
  /root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/native/prepared_v3/native_physics_hulls.usda \
  --output /root/docker_share/mujoko_debugging_data/20261017_newton_solver_compare/native/new_attempt \
  --steps 120 --hz 400 --minimal-rendering --stack-dump-seconds 30 \
  --preserve-unordered-bodies
```

공식 연결 구조와 패키지 호환 원칙은
[Isaac Sim6.1 Newton 문서](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/physics/newton_physics.html)에
설명되어 있다. 설치된 `isaacsim.physics.newton/impl/newton_stage.py`, `solver_config.py`,
동봉 Newton의 `utils/import_usd.py`를 실제 구현 확인에 사용했다.
