# Newton 수확 화면: 준비 후 START, 1배속 실행

2026-10-17. 기존 GUI는 첫 화면 update 뒤 최대 속도로 물리를 실행해, 사용자가 로딩 완료를 확인할 때는 동작 상당 부분이 이미 끝나는 문제가 있었다.

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh gpu-realtime --gui
```

이 명령은 다음 순서로 진행한다.

1. 물리 입력과 Isaac 농장 장면을 준비한다.
2. USD 로딩/streaming이 끝나고 초기 viewport의 PNG가 실제로 완성된 것을 확인한다.
3. `Harvest control` 창의 **START (1x)** 버튼에서 기다린다. 이때 실제 물리 시간과 실행 step은0이다.
4. 버튼을 누르면 제어 창을 숨기고 같은 고정 physics step·명령을1배속으로 실행한다. 느린 구간이 생겨도 이를 만회하려고 갑자기 빨리 진행하지 않는다.
5. 동작 완료 후 결과를 저장하고 마지막 화면을 유지한다. Isaac 창을 닫으면 실행기도 종료한다.

Isaac 좌측 timeline Play 버튼은 사용하지 않는다. 물리 계산은 Newton 프로세스가 담당하고 Isaac은 실제 측정 상태를 표시한다. 시작 버튼은 이 실행기에 시작 신호만 전달한다.

화면 없는 성능 측정은 기존 명령 그대로다.

```bash
./nvidia-sim/run_s300_newton.sh gpu-realtime
```

GUI를 포함한 **자동 시작·최대 처리 속도** 측정:

```bash
./nvidia-sim/run_s300_newton.sh gpu-realtime --gui --no-interactive-viewer
```

1배속 모드는 `interactive_viewer=true`와 대기 시간을 기록한다. `loop_wall_s` 및 단계별 시간에 의도적인 속도 제한 대기가 포함되므로, 이 모드의 약1.0RTF를 기존 최대 처리 성능3.4RTF와 비교해 성능 저하라고 해석하지 않는다. 사용자 START 대기는 `operator_wait_s`, 속도 제한 대기는 `presentation_wait_s`로 구분한다. 물리 dt·솔버·접촉 정책·분리 조건과 저장 명령은 바꾸지 않는다.

`--interactive-viewer`는 단일 GPU 실행+GUI에서 사용한다. 여러 반복을 자동 측정할 때는 `--no-interactive-viewer`를 지정한다. 기존 `gpu`/`gpu-fast`의 자동 실행 기본값은 유지한다.

## 실제 서버 검증

- 출력: `/root/docker_share/mujoko_debugging_data/20261017_230835_newton_interactive_validation`.
- 초기 장면 캡처 후 실제 START 버튼 클릭까지 99.33초 대기했다. 그동안 물리 시간/실행 step은0이었다.
- 실제 버튼 클릭으로 9,127/9,127 step, 시뮬레이션82.807초를 완료했다. 속도 제한 대기58.10초가 포함되며 강제 중단은 없었다.
- 기존 검증된 headless `final_profile_session/repeat_000`과 전체 기록 상태·접촉·분리 시계열이 정확히 일치했다. `comparison_with_verified_headless.json`의 `passed`, `full_requested_physics_trace_matches`, `gui_physics_equivalence_verified` 모두 true다.
- 연결 해제6개는 기존 결과와 동일하며 송이 전체 수확 성공을 의미하지 않는다. `training_eligible=false`, `harvest_success=null` 유지.
- 완료 결과 저장 후에도 viewer 생존과 마지막 실제 상태 표시를 확인하고, 검증 창만 종료했다. 기록: `interactive_wait_check.json`, `interactive_start.json`, `interactive_completion_check.json`.
- 관련 회귀시험20개 통과. GUI의 START/1배속 대기만 변경했으며 물리 모델·명령·판정은 유지했다.
