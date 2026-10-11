# S300 수확 상태를 보존한 송이·레일 연속 실행

이 문서는 기존 `run_motion.py`, `benchmark_motion.py`, `run_row_harvest.py`를 연결하는 상태 인계 규칙을 설명한다. 별도 수확 실행기나 새 물리 모델을 추가하지 않는다. SIM-GT 실험이며 학습 데이터에 넣지 않는다.

## 실제 마지막 상태에서 이어가기

`cycle_checkpoints/NNN`에는 실제 바디 위치·회전·속도, 로봇과 식물의 관절 위치·속도, preload, native 분리 이벤트, 로봇 drive의 위치·속도 목표를 저장한다. 측정된 관절 위치와 drive 목표는 서로 다른 값이며, 재개 시 둘을 각각 보존한다.

행동이 끝난 checkpoint를 만든 뒤 실행이 계속되어 물리 오류로 멈췄다면, 앞의 checkpoint는 독립 진단용으로 사용할 수 있다. 실제 레일 수확 이력의 인계에는 최종 정상 timestamp와 같은 checkpoint만 사용한다. 오류 뒤에 이전 상태로 돌아가 연속 성공으로 집계하지 않는다.

`run_row_harvest.py --initial-native-checkpoint`는 같은 assembly·manifest·USDC SHA, 정상 최종 summary, 최종 시각의 완료 cycle을 확인한다. 최종 summary의 `actual_self_collision_audit_passed=true`와 실제 저장 상태에 대한 완료 audit도 필요하다. audit의 model SHA와 입력 `states.npz` SHA를 함께 확인하므로, audit이 진행 중이거나 다른 상태 파일을 검사한 경우에는 인계하지 않는다. 새 plant window에서 초기 검사만 하고 실패한 경우에는 실제 이동·수확을 시작하지 않은 진단 시도로 기록한다. 이미 실제 이동이나 수확을 진행한 뒤 실패하면 그 측정 이력을 보존하고 캠페인을 중단한다.

최종 `route_checkpoint`도 실제 마지막 cycle의 `completed_cycle`·물리 유효성·완료 timestamp가 명시되고 최종 summary와 audit가 통과했을 때만 인계할 수 있다. 일반 idle, 중단 snapshot 또는 marker가 없는 snapshot은 완료 행동으로 바꾸지 않는다.

현재 줄기를 완료한 것으로 표시하고 중복 재실행을 제외할 때는 수확 범위에 포함되는 모든 열매의 정확한 native joint를 확인한다. Plant22에서는 Truss01·02의 20개가 대상이다. 한 송이의 10개만 분리되었거나, 다른 송이를 포함한 분리 이벤트가 20개 있다는 이유로 전체 완료로 판단하지 않는다. 실제 최종 summary가 20개 전체를 포함하고, 각 이름·바디·joint의 native 분리 증거와 일치할 때만 전체 native 분리로 기록한다. 이전 checkpoint는 다음 window의 측정된 물리 시작 상태로 그대로 사용한다. 이는 이미 분리된 열매의 중복 시도를 피하는 기능이며 실제 수집·수확 성공 판정은 아니다.

`run_row_harvest.py --defer-exhausted-current-plant`는 별도의 운영상 보류 옵션이다. 정상 최종 checkpoint·완료 실제 자세 audit·전체 20개 범위가 확인되고, 아직 붙어 있는 각 열매의 checkpoint와 summary 시도 횟수가 원래 예산과 일치하면서 모두 소진된 경우에만 현재 줄기의 제안을 보류한다. 예를 들어 실제 19개가 분리되고 Fruit10이 6회 시도 후에도 붙어 있다면, 그 열매를 **붙어 있는 미확인 대상**으로 보존한 채 다른 줄기를 시험할 수 있다. `all_eligible_native_detached=false`는 그대로 유지한다. 시도 횟수·분리 증거·최종 측정 상태를 초기화하지 않으며, 보류한 줄기는 원본으로 다시 만들어 재시도하지 않는다. 아직 시도 예산이 남은 열매나 한 송이만 포함한 summary에는 전체 줄기 보류를 적용하지 않는다.

## 같은 줄기의 다음 송이

이전 실행 코드에서 한 cycle의 끝이 전체 시간 예산과 정확히 같으면, 다음 loop 경계에 진입하지 않아 완료 marker만 누락될 수 있다. `terminal_cycle_checkpoint.py`는 이 경우의 호환 도구다. 정상 최종 summary와 실제 모든 자세 audit, 정확한 마지막 완료 cycle·timestamp·최종 poses/q가 기존 raw `route_checkpoint`와 일치할 때만 같은 trial의 새 cycle checkpoint로 발행한다. 원래 raw checkpoint와 결과는 그대로 두며, 모든 측정 배열·속도·preload·drive 목표·native 이벤트를 그대로 복사한다. 새 physics나 수확 결과를 만들지 않는다. 일반적인 중단·미완료·이전 timestamp·audit 진행 중 상태에는 사용할 수 없다.


현재 원본 Plant22는 Truss02에만 승인된 elastic fruit-neck 가설이 적용되어 있다. Truss01을 같은 가설로 시험하려면 다음 도구로 새 assembly를 만든다.

```bash
/root/farmily_tomato/nvidia-sim/.asset-venv/bin/python \
  /root/farmily_tomato/nvidia-sim/harvest_s300/eligible_attachment_transfer.py \
  SOURCE_ASSEMBLY LATEST_VALID_TERMINAL_CHECKPOINT \
  NEW_ASSEMBLY RAIL_NEAR_INVENTORY_JSON
```

도구는 원본 Plant22의 아직 분리되지 않은, 수확 범위에 포함되는 Truss01 연결만 기존 3 N / 0.008 Nm elastic 설정으로 바꾼다. Truss00, 기존 Truss02 연결 설정, 충돌 형상, 질량, 마찰, actuator와 0.5 mm 기준은 보존한다. 이미 native에서 분리된 Truss02 연결은 분리 증거와 비활성 선언을 유지한다. 이는 명시적인 연결 가설 이전이며, 결합된 식물 반응이 이전과 같다고 가정하지 않는다.

실행에는 기존 checkpoint와 `--allow-resume-attachment-transfer`를 함께 전달한다. `--near-trusses-metadata`와 `--initial-truss-first`를 사용하면 기존 목표 송이의 남은 열매를 먼저 확인한 뒤, 소진되었을 때 같은 실제 식물의 다른 허용 송이를 선택한다. 다음 송이에는 그 송이의 측정된 열매 방향을 사용한다. 다른 송이의 이전 yaw를 가져오지 않는다.

USD property 보존 검사와 계획 통과는 native 수확 성공이 아니다. native 초기 상태·DOF 복원, 실제 접촉과 분리, 저장된 모든 실제 로봇 자세의 검사를 별도로 통과해야 한다.

## 물리 중단 뒤의 독립 새 실험

2026-10-11의 Truss01 연속 실행은 `prepare_joint_branch` 9.658333초에서 다른 줄기 Plant34의 잎과 충돌해 중단했다. Truss01 진입 전이며 새 native 분리는 0개다. 이 물리 이력 뒤에는 이전 9/10 checkpoint를 가져와 연속 성공으로 진행하지 않는다.

다른 준비 경로를 비교할 때는 `eligible_attachment_transfer.revise_fresh(assembly, output, inventory_path)`로 **독립적인 정상 초기 상태 실험**을 만든다. 도구는 기존 `apply_transfer`, compact 저장과 USD property/geometry 보존 검사를 재사용한다. 원본의 정상 로봇 관절 상태·모든 열매의 활성 연결을 확인하고, 승인된 Truss01 연결 가설만 적용한다. 초기 목표는 Truss01이고 대상 범위는 가까운 쪽 Truss01·02의 20개다. Truss00은 바꾸지 않는다.

`fresh_campaign`은 checkpoint 미사용·이전 native 분리 0개·이전 실행과 독립임을 명시한다. 측정 상태를 가져온 assembly나 비활성 과실 연결이 있는 source는 거부한다. 새 실험의 열매를 이전 9개 분리와 합산하지 않는다. 준비 경로는 robot/tool 자기충돌 검사에 더해 농장 장애물 검사를 통과한 후보만 실제 native로 실행한다.

## 레일 이동

새 window의 기존 route 인계는 실제 이전 상태에서 새 plant를 활성화한다. 이후 이동은 world-Y prismatic joint를 구동한다. 로봇 바디 transform을 순간 이동시키지 않는다.

같은 assembly에서 이미 이동한 checkpoint를 재개하면 계획기와 FCL의 초기 원점은 저장된 실제 base 위치를 사용한다. native 초기 FK 비교도 이 시작 위치에서 수행한다. 새로 요청한 도착 위치와 혼동하지 않는다.

같은 assembly의 checkpoint에서 추가 레일 이동은 다음 명시 옵션을 요구한다.

```text
--resume-native-checkpoint CHECKPOINT
--allow-resume-rail-travel
--rail-to-y STATION_WORLD_Y_M
```

rail phase는 저장된 실제 시작 관절 상태에서 연결한다. 이전 native drive 목표와 측정된 base 위치를 별도로 보존하며, 설정한 속도를 실제 smoothstep command가 넘지 않도록 시간을 계산한다. 이후 cycle은 도착 위치를 유지한다. 전체 경로와 실제 physics 검사는 그대로 적용한다.

## 제한된 잎 초기 안정화 실험

`--condition-foliage-seconds`는 최대 3초의 별도 idle 초기화 실험이다. 정확히 식별된 잎 관련 raw 충돌은 원자료로 남긴다. 이후 최소 1초를 기존 0.5 mm 기준으로 연속 검사하며, 새로운 native 분리·비정상 수치·로봇 또는 잎 이외의 충돌 예외가 있으면 canonical 시작 상태로 인정하지 않는다.

정상 strict suffix, native 최종 상태, controller 목표, 실제 모든 저장 자세 검사와 source/code/model SHA가 확인된 경우에만 `conditioned_start.json`을 만든다. 이 checkpoint는 `canonical_conditioned_start`이며 `completed_cycle=false`다. 수확하거나 완료 행동을 실행했다고 기록하지 않는다.

재개에는 `--allow-resume-conditioned-start`가 필요하다. 같은 assembly만 허용하며 연결·충돌 proxy 수정과 합치지 않는다. 일반 idle 결과나 이전 실패 결과에는 이 옵션을 붙여도 통과하지 않는다. `run_row_harvest.py`는 여전히 실제 완료 cycle을 요구하므로, canonical idle 상태는 먼저 기존 `run_motion.py`에서 실제 수확을 진행하고 정상 최종 완료 checkpoint를 만든 뒤 인계한다.

## 결과를 읽을 때

- 과거 native 분리 이벤트는 `imported_native_prior`, `native_new_event=false`로 보존한다. 재개 후 새 callback과 구분한다.
- 초기 warmup 중 분리는 수확 횟수에 포함하지 않는다.
- FCL self 경로 통과, 환경 초기 통과, 실제 접근 완료, 열매 분리는 각각 별도 결과다.
- 제한된 계획 검색에서 후보를 찾지 못한 경우는 미발견으로 남긴다. 절대적인 수확 불가능으로 바꾸지 않는다.
- 새 엔진에서 상태를 복원할 때 PhysX contact cache까지 연속 재현한 것은 아니다. 실제 바디·DOF·속도·drive 복원 오차와 첫 solved physics 결과를 확인한다.

관련 검사: `test_motion_continuation.py`, `test_conditioned_start_checkpoint.py`, `test_eligible_attachment_transfer.py`, `test_run_row_harvest.py`. CPU 회귀 통과만으로 native 수확이나 새 window 초기 물리 통과를 주장하지 않는다.
