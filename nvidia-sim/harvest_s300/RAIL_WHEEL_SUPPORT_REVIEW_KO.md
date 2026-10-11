# 레일 이동의 바퀴와 바닥 이음부 검사

Y=1.392025907m의 Truss01 제안은 coarse SELF를 통과했지만, backward rail 이동 중 `Part_0008`과 `CoverLap_1_2_2/Mesh`에서 ENV가 차단됐다. 이것을 전체 wheel convex envelope의 허위 충돌이라고 단정할 수 없다.

실제 source wheel CAD(`T_01_S300_8inch_Wheel_V2NR-1`, 105,080 vertices/84,942 triangles)와 해당 CoverLap의 실제 triangles를 FCL로 비교했다. 보고된 rail offset−91.018754mm에서 source surface끼리도 교차한다. 첫 source contact의 depth는 약0.424mm이고, whole-part wheel hull의 첫 contact는 약0.600mm다. 이것은 각 FCL contact의 값이며 전체 pair의 최대 penetration이나 native PhysX penetration 인증값이 아니다. wheel의 source bottom은 Z=3.000mm이고 CoverLap top은3.750mm다. 따라서 더 깊은 구간에서는 기존0.5mm native 기준을 넘을 수도 있다.

현재 rail ENV checker는 wheel/floor/cover에도 **교차0개**를 요구한다. 기존 USD filter만 사용하며 의도한 support contact를 별도로 표현하지 않는다. native runner는 robot 접촉의 기존0.5mm 관통 기준을 사용하고, wheel/ground에는 released-fruit permission을 적용하지 않는다. 두 조건은 서로 다른 검사다.

정확한 source4wheel과 정확한 horizontal GroundCover/CoverLap top support를 별도 opt-in 진단 class로 구분하는 것은 가능하다. raw 교차·pair·phase는 남기고 native collision response와0.5mm 중단은 유지해야 한다. chassis housing, arm/tool, 붙어 있는 과실, main stem/gutter/sidewall까지 허용해서는 안 된다. 이 진단을 전체 scene 무충돌 PASS로 표현하지 않는다. 이번 검토에서는 분류나 물리 코드를 변경하지 않았다.

관측된 native chassis piece를 기존 checker에 넣는 최소 방법은 기존 `native_collider_capture`/`native_tool_hulls`의 공통 검증을 재사용하는 것이다. `CookedLeafContactGeometry.cooked(path)`는 이미 rigid-owned chassis Mesh의 모든 native convex piece를 읽을 수 있다. 현재 exporter와 importer의 scope만 tool 경로로 제한되어 있다. tool 기본 동작을 유지하고, 기존 pinned passive CAD inventory와 manifest carrier lineage로 확인한 wheel4개만 별도 scope로 추가하면 된다. owner-local piece union을 그대로 `MotionSelfCollision`에 넣고, source/model/USDC/reader SHA와 전체 piece 수를 검증한다. 다른 model의 cache나 CAD 전체 hull로 축약한 결과를 사용하지 않는다.

CoverLap은 static `physics:approximation=none` triangle mesh다. 설치된 cooking API의 현재 사용 가능한 경로는 convex representation이며 기존 reader도 이 static triangle cooking을 인증하지 않는다. source triangles를 native cooked triangle 인증이라고 표현하지 않는다.

다음 접근 후보로는 기존 union47 중 현재 station1.6m보다 큰 Y의 가장 가까운 SELF 통과 제안인 **Plant35/Truss02, Y=1.777922835m, joint seed6 또는8**이 있다. actual root X=.678233m는 해당 gutter X=.775m보다 통로에 가까운 쪽이다. 이 +177.923mm 구간에서 source4wheel과8CoverLap 사이에는 연속 구간 전체의 triangle 교차가 없음을 별도로 확인했다. swept AABB 후보2개는 시작 distance37.557mm보다 작은6.098mm 이동 후 AABB가 분리되고, 나머지는 전 구간 AABB가 분리된다. 최소 separation 하한은31.460mm다. 이는 **CoverLap pair만의 authored-source 증거**이며, 전체 farm ENV·새 active plant 시작·actual rail 도달·수확 통과는 아직 아니다.

가능한 제한된 실제 시험은 유효한 정확한 최종 parent에서 +177.923mm 이동 후1초 hold 하나다.120Hz·충돌/힘/물성/0.5mm 기준·원래 native break inventory를 유지하고, 실제 qpos·qvel·drive targets와 모든 raw contact를 저장한다. backward208mm는 이와 다른 support-contact 진단이며 실패할 수 있다. 현재 CLI는 resume에 whole-truss를 요구하면서 whole-truss idle을 금지하므로 rail-only same-plant resume는 직접 실행할 수 없다. 별도 opt-in으로 기존 resume/rail sampler를 연결할 경우에도 prior native arm/lift/jaw target을 이동 중 유지해야 한다. 측정 q를 drive hold target으로 바꾸면 이미 관측한 gravity sag가 다시 발생한다. 이 제한은 기존 combined rail+harvest 실행에는 적용되지 않으며, 새 simulator나 planner가 필요하다는 뜻은 아니다.

자료: `/root/docker_share/mujoko_debugging_data/20261011_013045_row_harvest_recovery/rail_chassis_cover_source_review_v2/report.json` 및 `source_probe.py`. source4wheel/8CoverLap/5offset만 확인했고 새로운 native 실행, 자산 수정, 충돌 permission 변경은 없었다. 최초 scratch 검사에서 instance proxy를 누락한 결과는 별도 `invalid_analysis_coverage.json`으로 보존했으며 결론에 사용하지 않았다. `training_eligible=false`, `harvest_success=null`을 유지한다.
