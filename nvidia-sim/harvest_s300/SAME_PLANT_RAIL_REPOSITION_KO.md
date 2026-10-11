# 같은 식물의 Truss01을 다른 레일 위치에서 다시 접근

Plant22/Truss01을 Y=1.6m에서 찾은 준비 경로가 막혔다고 해서 다른 레일 위치에서도 접근할 수 없다고 판정할 수 없다. 기존 `row_station_screen_v6_all31_union/report.json`에는 같은 송이의 두 120Hz 전체 SELF 통과 제안이 있다.

| 레일 Y | 기존 준비 branch | 검사 범위 |
| --- | --- | --- |
| 1.3920259074118901m | joint seed8 | 과거 authored 초기 상태의 전체 SELF |
| 1.5720259074118903m | joint seed6 | 과거 authored 초기 상태의 전체 SELF |

두 제안은 lift 재분배 −150mm, 준비 gap44mm, retract100mm, grasp depth65mm, height offset−10mm를 사용했다. 과거 송이 yaw는124.389796°였다. 현재 시험의 실제 height−1.5mm, 수확 후 변형된 송이 위치·yaw·로봇 q는 다를 수 있으므로 명령 배열이나 통과 판정을 그대로 재사용하지 않는다.

중요한 한계는 두 제안의 `prep_pose`와 `goal_pose`가 수치적으로 완전히 같다는 것이다. 동일한 실제 열매 위치와 송이 방향에서 레일 이동은 팔과 chassis의 위치·IK branch를 바꾼다. 요구한 world gripper pose 자체는 바꾸지 않는다. 따라서 Plant34와 팔의 준비 경로 충돌을 피할 가능성은 있지만, 특정 tool pose와 static foreign leaf의 겹침을 레일 이동만으로 제거했다고 할 수 없다. 실제 새 checkpoint의 staging/entry endpoint를 기존 SELF/ENV `check_state`로 먼저 확인하고, 통과 후보에 전체 경로 검사를 수행한다.

새 실행 파이프라인은 필요하지 않다. 기존 `eligible_attachment_transfer.revise(actual_source, actual_final_checkpoint, new_output, exact_inventory)`가 Truss01의 아직 부착된 열매에 기존3N/.008Nm elastic 가설을 명시적으로 적용한다. 기존 native 분리 이벤트와 broken joint 선언을 유지하고, Truss00·geometry·filter·material·mass·actuator는 바꾸지 않는다. 이는 Truss01의 물리 가설 변경이며, 이전 coupled dynamics가 동일하다고 주장하지 않는다.

기존 `assembly_plant22_truss01_after_attempt6_transfer`는 과거9/10 checkpoint에 묶였다. 새10/10 상태에 사용할 수 없다. 실제 최종 summary·state·actual audit가 모두 유효한 새 parent로 별도 출력에 다시 만들어야 한다. 이후 기존 runner의 다음 옵션을 사용한다.

```text
--resume-native-checkpoint <ACTUAL_VALID_FINAL_CHECKPOINT>
--allow-resume-attachment-transfer
--allow-resume-rail-travel
--rail-to-y 1.3920259074118901
--near-trusses-metadata <EXACT_NEAR_INVENTORY>
--initial-truss-first
--horizontal-ready-connector joint
--horizontal-ready-seed-index 8
--measured-ready-start
--verify-rail-environment-preflight
--verify-preparation-environment-preflight
```

`measured_resume_rail_origin`은 저장된 실제 base 위치와 이전 native rail target을 구분한다. `attach_resumed_rail_travel`은 실제 prismatic joint 명령을 만들며 q·plant pose를 초기화하거나 world transform으로 이동시키지 않는다. 약−208mm 이동은 speed0.08m/s에서 약3.90초다. 정확한 시작값과 duration은 최종 checkpoint에서 다시 계산한다. 초기 Truss02가 모두 분리되면 기존 near-scope selector가 남은 Truss01을 선택하고, 그 송이의 실제 XY PCA를 사용한다. Truss02 yaw를 Truss01에 복사하지 않는다.

새 model에 native tool cache를 사용하면 해당 transferred assembly의 SHA에 맞아야 한다. 원본 또는 과거 transfer cache를 다른 model에 넘겨서 인증한 것으로 해석하지 않는다. 레일 ENV 검사에도 attachment-transfer 검증 인자를 전달해야 한다. preparation ENV만 통과해도 전체 삽입의 environment 통과나 실제 도달을 뜻하지 않는다.

이 검토는 read-only였다. 기존 continuation/transfer 테스트24개가 통과했다. AST fixture가 새 last-fruit aim의 기본 필드를 포함하도록 수정했으며 실제 runner의 동작은 이 검토로 바꾸지 않았다. 실제 최종10/10 parent·새 transfer·레일 physics·Truss01 수확은 아직 미검증이다. 실패한 실행 뒤 과거 정상 checkpoint로 돌아가 계속 성공한 것처럼 집계하지 않는다.

자료: `/root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/same_plant_truss01_rail_reposition_review/report.json`, `pending_recipe.json`. `training_eligible=false`, `harvest_success=null`을 유지한다.
