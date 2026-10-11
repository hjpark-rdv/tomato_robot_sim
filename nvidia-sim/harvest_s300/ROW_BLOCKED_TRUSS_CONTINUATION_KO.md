# 일부 송이를 보류한 실제 레일 이동

`run_row_harvest.py --defer-blocked-other-trusses <proof.json>`는 별도 opt-in 운영 선택이다. 정상 시작에서 Truss02의 정확한 열매 10개 native joint가 분리되고, 최종 상태·actual self audit가 유효하면, 유한 탐색에서 준비 경로가 막힌 Truss01을 기록해 보류하고 다음 가까운 쪽 식물로 이동할 수 있다. Plant22 전체 20개 완료, 실물 수확 성공, 절대 수확 불가능을 뜻하지 않는다. 다른 송이의 시도 수를 만들어 넣거나 초기 상태를 바꾸지 않는다.

기존 전체 20개 완료 판정과 `--defer-exhausted-current-plant`의 실제 시도 예산 소진 판정은 그대로다. 기본 선택은 현재 식물을 우선하지만 명시적인 preferred order는 다른 식물을 먼저 제안할 수도 있다. 새 옵션은 이 선택에 **완료한 송이와 보류한 송이를 구분하는 증거**를 붙인다.

증거 JSON schema는 `s300-explicit-blocked-other-trusses-v1`이다. `parent`에는 exact source assembly, scene/manifest/materialized dependency, near inventory, 최종 checkpoint JSON/NPZ/metadata, 최종 native summary와 actual audit SHA를 넣는다. `completed_trusses`는 실제 native 분리 이벤트와 기록된 inventory로 확인한다. `deferred_trusses`는 나머지 eligible 송이를 빠짐없이 포함하고, 아직 달린 정확한 열매 이름·`reason=finite_planning_budget_blocked`·`reachability=unknown`·유한 planning blocker를 기록한다.

각 blocker는 기존 plan과 complete/blocked 환경 audit의 경로·SHA·source asset의 scene/manifest/USDC SHA를 가져야 한다. audit가 incomplete/inconclusive이거나 다른 송이의 검사면 거부한다. 과거 독립 fresh 검사라면 `state_relation=separate_historical_diagnostic`으로 표시한다. 그런 검사도 현재 변형된 최종 자세의 접근 불가능 증명으로 승격하지 않는다. 보수적인 tool proxy로 막힌 계획은 보수적인 유한 탐색 실패라는 제한을 유지한다.

수확 후 handoff는 기존 `route_checkpoint.switch_window`를 재사용한다. 로봇·carrier·controller target·열매의 실제 최종 상태를 보존한다. 보류된 부착 식물은 실제 변형 위치에서 static으로 바뀌는 기존 active-window 근사이고, native에서 이미 분리된 열매는 dynamic 위치·속도·원래 분리 근거를 유지한다. 새 식물의 정상 시작과 실제 레일 이동은 별도로 확인한다. 새 창의 독립 idle probe는 이동 실적이나 다음 부모 checkpoint가 아니다.

```bash
cd /root/farmily_tomato
./nvidia-sim/.asset-venv/bin/python nvidia-sim/harvest_s300/run_row_harvest.py \
  <ACTUAL_FINAL_SOURCE_ASSEMBLY> \
  /root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/rail_near_inventory.json \
  /root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/row_station_screen_v6_all31_union/report.json \
  <NEW_CAMPAIGN_OUTPUT> \
  --initial-native-checkpoint <ACTUAL_FINAL_COMPLETED_CHECKPOINT> \
  --defer-blocked-other-trusses <EXACT_PARENT_BOUND_DEFERRAL_PROOF.json> \
  --preferred-selection /root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/next_row_selection_after20_plant29_first.json \
  --motion-arguments <CURRENT_VERIFIED_MOTION_ARGUMENTS.json> \
  --preflight-new-windows --max-plants 1 \
  --benchmark-python /root/farmily_tomato/mujoco-benchmark/.venv/bin/python \
  --python /root/isaac-sim-6.1.0/python.sh --timeout 7200
```

기본은 plan-only다. parent 검증과 근거가 완성되면 같은 명령에 `--execute`를 붙여 기존 실행기를 사용한다. Plant29/Truss00의 Y=3.222384591770679m는 과거 120Hz self-clear 제안이며 현재 모델의 environment/physics 인증이 아니다. 실제 활성 모델에서 rail 및 preparation environment 검사와 정상 native 시작을 다시 확인한다. native tool cache를 쓴다면 새 asset/hash에 맞는 capture가 필요하며 다른 source cache를 복사해 사용하지 않는다.

잘못된 부모·완료 위조·일부 eligible 송이 누락·attached inventory 변경·inconclusive blocker·다른 target/source·stale plan/audit를 거부한다. 후속 실제 이동이 중단되면 실패 상태와 원인을 보존하고 이전 정상 checkpoint로 돌아가서 계속한 것처럼 집계하지 않는다. 관련 기존·신규 `test_run_row_harvest` 38개가 통과했다. 이 문서 작성 시 실제 최종 10/10 parent는 아직 실행·audit 중이며, 다음 Plant29 asset/native/레일 이동은 시작하지 않았다. `training_eligible=false`, `harvest_success=null`, collection 미검증을 유지한다.
