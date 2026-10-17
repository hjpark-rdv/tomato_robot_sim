# Newton GPU 한쪽 베드 3곳 연속 이동·수확: native 충돌체 수정 후 완주

작성: 2026-10-18 KST. 같은 쪽 베드의 **Plant22 → Plant27 → Plant29 세 방문을 모두 완료**했다. 각 송이에서6개씩, 총18개 연결 해제(색상상 익음17개·미숙1개)를 기록했고 미방문 송이 조기 분리는0개였다. 이전 세 번째 방문을 막던 그리퍼 판의 단일 convex hull을 원본 PhysX 기본16조각으로 바로잡은 뒤 정상 초기 상태부터 전부 다시 실행한 결과다. 송이 전체 수확이나 포집 완료를 주장하지 않는다.

## 목적과 범위

사용자는 양쪽 베드를 번갈아 처리하지 않고, **같은 쪽 베드의 송이들을 레일을 따라 연속으로 접근·수확**하기를 요청했다. 이번 시험은 기존 온실 배경·로봇·그리퍼·수확 동작·계획기·Newton GPU 실행기를 연결하는 SIM-GT 소량 시험이다.

같은 거터 중심 X=-0.775m의 레일 쪽 줄기와 열매 중심을 선택했다. 반대쪽 X=+0.775m의 Plant41은 `opposite_bed`로 명시적으로 제외했다. 이 제외는 로봇 도달 불가능 판정이 아니다.

| 순서 | 대상 | 원본 송이 | 목표 레일 정차 Y | 열매 수 |
|---|---|---|---:|---:|
| 1 | Plant22 | Truss_02 | 1.580000m | 10 |
| 2 | Plant27 | Truss_00 | 2.652643m | 10 |
| 3 | Plant29 | Truss_00 | 3.222385m | 10 |

현재 코드는 각 창에서 기존 집단 선택기가 고른 집단에 기존 수확 동작을 실행한다. **송이에 남은 열매를 모두 분리했다는 의미가 아니며**, 선택된 집단·남은 열매·새 분리를 각각 저장한다. 리프트와 레일은 기존 유한 actuator 명령으로 이동한다. 이번 작업에서 새 수확 알고리즘, RL, RGB-D/mask 수집 또는 학습을 시작하지 않았다. 모든 자료는 `training_eligible=false`, `harvest_success=null`이다.

## 사용한 자산과 물리 표현

결과 루트:

```text
/root/docker_share/mujoko_debugging_data/20261018_010218_newton_same_bed_continuous
```

호스트/NAS의 실체는 `/mnt/nas_rdv_md3/mujoko_debugging_data/20261018_010218_newton_same_bed_continuous`이다. 기존 원본 자산은 `/root/docker_share/mujoko_debugging_data_bak/` 아래에 보존했고 덮어쓰지 않았다.

기존 `export_scene.py`, `merge_windows.py`, Newton 입력 준비기, coarse4와 과실 convex proxy를 재사용했다. 단일 타깃 133DOF 모델과 달리 이번 공통 MJB는 **3개 선택 식물의 줄기·송이·열매를 함께 가진 379DOF 모델**이다. 방문하지 않은 두 타깃도 이 MJB에서는 탄성·부착·분리 계산을 계속한다. 따라서 단일 133DOF와 속도를 직접 비교해 엔진 개선 수치로 해석하면 안 된다.

| 공통 물리 모델 | 값 |
|---|---:|
| 자유도 `nv` | 379 |
| 위치 좌표 `nq` | 409 |
| body | 357 |
| geom | 2,244 (part161 수정 전2,229) |
| 열매 free joint / CONNECT 부착 | 각각 30개 |
| 추가 주변 고정 식물 | 6그루 |
| 추가 주변 충돌체 | 1,150개 |

주변 고정 식물은 Plant01, 02, 09, 23, 26, 28이다. 대상별 가까운 두 식물의 합집합을 사용했다. 원본 collider를 가진 기존 변환 자산에서 capsule과 convex mesh를 가져와 고정 장애물로 추가했다. Plant01/02는 각각 289개, 나머지 네 식물은 각각 143개다. 원본에서 꺼진 잎 충돌 상태를 보존했으며, 잎을 강한 장애물로 만들지 않았다.

`route_fixed_neighbors_verification.json`은 각 collider의 source→합집합 world transform 차이가 0임을 확인한다. 이는 transform과 활성 상태의 재현성 검증이다. 원본 농장의 모든 triangle collider 또는 모든 주변 식물의 탄성 동작과 동등하다는 주장은 아니다. 전체 농장의 시각적 배경은 별도 display assembly에서 유지한다.

`native_piece_route/inputs_route/input_model.mjb`가 최종 세 구간의 공통 물리 입력이다. 각 구간은 이 파일을 새로 변환하지 않고 동일 SHA `55159876ddeb6250ffea7bccb102a2e31dbf8d41556a17140454279e5ded0cc9`의 MJB를 참조한다. 기존 열매 크기, 질량, CONNECT와 꼭지 스프링/분리 설정을 보존했다. source 열매 dimensions를30개 모두 대조했으며, 전체 body 표시 경로356/356과 초기 qpos/qvel/ctrl/preload의 유한성을 확인했다. 원래 공통 입력 `inputs_route/`와 그 결과는 별도 비교군으로 보존한다.

## 실제 상태를 이어가는 방식

`run_newton_row_harvest.py`는 새로운 physics loop가 아니라 기존 실행기의 연결부다.

1. `one_bed_selection.json`과 원본 배치 inventory SHA로 같은 쪽 베드와 대상 순서를 다시 확인한다.
2. 첫 Plant22는 기존 source plan을 재사용한다.
3. 구간이 정상 완료하면 실제 마지막 `qpos`, `qvel`, `eq_active`, `ctrl`, runtime damping, 물리 clock과 선택 가능한 `qacc_warmstart`를 다음 구간으로 넘긴다.
4. 같은 30개 attachment/body/robot 주소와 공통 MJB SHA, 고정 preload·dof length·actuator 계수를 확인한다.
5. 다음 집단과 계획은 실제 마지막 열매 pose, 실제 로봇 관절·jaw·레일 위치에서 기존 선택기와 `plan_adapter.py`로 만든다.
6. 분리 이력은 `cumulative_break_events.json`으로 누적하고 이미 분리된 열매를 원래 위치나 부착 상태로 되돌리지 않는다.

**연속되는 것은 실제 측정 상태와 이력이다.** 구간마다 Newton solver instance는 새로 생성되고 접촉·제약 캐시는 재구성한다. 한 번 생성한 solver의 모든 내부 캐시가 끊김 없이 이어지는 구현은 아니다. 이 재시작 경계를 `incoming_handoff.json`과 physics `summary.json`의 `continuation` 증거에 명시한다. 중단을 없었던 일로 바꾸어 연속 성공으로 집계하지 않는다. 정상 완료한 부모에서 다른 경로를 시험할 경우 별도 output의 대조 실험으로 보존한다.

Plant27의 기존 자산에는 실제 실행 `states.npz`가 없으므로, 초기 export에는 `source_states_kind=planning_only_no_recorded_states`를 명시했다. 이것은 기존 계획 자산의 출처이며 실제 수확 증거가 아니다. 새 결과의 `states.npz`와 분리 이벤트가 실제 실행 근거다. 원래 recorded state가 있어야 할 입력에서 파일 누락을 자동 무시하지 않는다.

## 접촉과 성공 해석

기본 감시는1mm이며 기존 `belt_strokes_2mm.json`의 정확한 belt↔현재 목표 과실/pedicel 및 명시된 닫기·열기 스트로크만2mm band를 사용한다. 마찰, 강성, margin/gap, actuator와 분리 임계값은 유지했다. 최종 시험에서는 그리퍼 판 한 부품의 collision 표현만 원본의 기본 PhysX convex decomposition으로 수정했다. 따라서 이전 단일 hull 결과와 물리 모델이 같다고 주장하지 않는다.

세 식물의 attachment가 모두 metadata에 존재하므로, 각 구간에 `active_harvest_attachment_names`를 정확한 native 이름 10개로 지정한다. 예를 들어 Plant22 구간에서는 Truss_02 열매와 그 송이 pedicel만 belt 완화 역할을 가지며, Plant27/29는 기본 1mm다. 빈 목록, 중복, 알 수 없는 이름은 거부한다. 이 필드를 생략한 기존 단일 타깃 동작은 그대로다.

첫 연속 시험에서 이미 분리된 열매가 거터에 닿아 중단되는 경우가 있었다. 기존 CPU의 `--released-gutter-diagnostic`를 GPU와 접촉 재판정에 연결했다. 이 옵션은 **이미 분리된 열매↔정확히 `gutter`로 분류된 world collider**의 충돌을 계속 계산·기록하면서 수확 진행 중단 조건에서만 분리한다. 붙어 있는 열매나 로봇의 거터 접촉, 이름에 gutter가 들어갈 뿐 inventory에 없는 물체는 계속 감시한다. 기존 분리 열매↔그리퍼/바닥/자기 송이 예외도 유지한다.

열매 분리는 native CONNECT가 실제 힘·토크 조건으로 꺼졌다는 뜻이다. 수집함 포집, 과실 손상, 실물 수확 성공 또는 송이 전체 완료와 동일하지 않다. 아직 접근하지 않은 식물이 저절로 분리된 경우는 현재 타깃의 수확 수에 더하지 않는다.

## 수치 수렴 문제와 33초 대조 시험

초기 전체 3식물 실행에서는 Plant22를 처리하는 중 방문 전 Plant27 Fruit01과 Plant29 Fruit02가 각각 약 31.235초와 44.226초에 분리됐다. 두 사건은 현재 타깃 수확 실적으로 제외했다.

저장 상태로 확인한 분리 직전 최대 변위는 약 0.367mm와 0.543mm였다. Plant27 Fruit01은 초기·분리 직전·직후의 CPU 기하 질의에서 과실 접촉이 없었다. Plant29 Fruit02는 자기 Rachis와 약 1.7µm 접촉이 있었고 로봇 접촉은 없었다. 이는 CPU에서 저장 GPU 상태의 기하를 조회한 결과이며, 같은 스텝 GPU 접촉력 원자료를 대신하는 것은 아니다.

원래 옵션과 같은 초기 상태·명령으로 33초 계측을 수행했다. 별도 wrapper가 원래 `post_step` 직전에 private GPU buffer에 CONNECT 행, type/id, force, D, aref, pos, sparse J, qacc와 힘 항을 기록했다. 원본 실행과 계측 실행은 전체 공통 prefix의 time/qpos/qvel/eq/robot_q가 완전히 같았고 4471스텝의 3.544N 분리가 재현됐다. 계측 버퍼를 제외한 물리·명령 순서는 유지했다.

| 4471스텝, P27 Fruit01 | 원래 실제 GPU tolerance 약 1e-6 | 실제 GPU tolerance 약 1e-8 |
|---|---:|---:|
| 저장 CONNECT 힘 norm | 3.544029474N | 0.074131712N |
| `-D*(J*qacc-aref)` 재계산 | 3.544029713N | 0.074131712N |
| equality ID / 행 | 정확한 ID10 / CONNECT 3행 | 동일한 정확한 소속 |
| 병진 `Jᵀf`와 public `qfrc_constraint` | 일치 | 일치 |
| 병진 `Ma-smooth-public` Y 성분 | 약 3.53949N | 약 2.25e-9N |
| 실제 조기 분리 | 발생 | 발생하지 않음 |

따라서 이번 스파이크를 단순한 force 읽기·equality 행 매핑 오류라고 해석하면 안 된다. raw force, 현재 qacc로 재계산한 force, `Jᵀf`와 public constraint force는 서로 맞았다. 반면 거의 정지한 과실의 반력 균형이 부정확했으며, 더 엄격한 실제 GPU 수렴 설정에서 정상화됐다. `Ma-smooth-public`에는 DISCRETE integrator의 effective-force 항 해석이 필요하므로 이를 모든 DOF의 최종 물리 잔차로 단정하지 않는다.

| 같은 최초 33초 구간 | 원래 설정 | 실제 GPU 1e-8 |
|---|---:|---:|
| 방문 전 송이의 분리 | 1개 | 0개 |
| P27 Fruit01 최대 힘 | 3.544N | 0.07525N |
| P29 Fruit02 최대 힘 | 1.00683N | 0.11191N |
| 방문 전 두 송이 총 20개 열매의 최대 힘 | 최대 3.544N | 모두 0.112N 미만 |
| 현재 Plant22 새 분리 | 01/02/03/05/08, 5개 | 01/02/03/05, 4개 |
| 접촉 정책 / capacity 검사 | PASS / PASS | PASS / PASS |

수렴 설정을 바꾼 후 분리 수가 줄었다. 기존 Fruit08의 force 스파이크에 의한 분리가 사라졌으므로, 분리 개수가 많다는 이유로 원래 설정을 더 좋은 수확으로 평가하면 안 된다. 두 계측 실행의 RTF는 계측 부하가 있는 값이며 production 성능으로 사용하지 않는다.

이 대조 자체는 최초 33초만 검증했다. 아래 전체 재실행에서 Plant22의 57초와 Plant27의 72초 구간을 포함해 미방문 송이의 조기 분리가 0개임을 추가로 확인했다. 다른 장면·솔버·GUI 설정까지 일반화한 검증은 아니다.

## Production 옵션 수정과 최종 재검증

설치된 MuJoCo Warp `put_model`은 tolerance를 최소 1e-6으로 올린다. 기존 runner는 source 값만 설정해서 더 작은 명시값도 GPU에서 바뀌고 있었다. `--solver-tolerance`를 명시할 때만 업로드 후 GPU `opt.tolerance`에 실제 반영하도록 수정했다. `None`을 생략 기본값으로 두어 기존 호출은 기존 GPU 설정을 보존한다. 이번 연속 시험만 `1e-8`을 명시적으로 전달했다. 실제 float32 값은 약 `9.999999939e-9`이며 모든 구간의 solver 설정이 일치했다. 이는 검증한 모델의 수렴 설정 개선이며 모든 GPU 모델에 대한 보장이나 분리 임계값 변경이 아니다.

### 최종 결과: native16 충돌체를 적용한 정상 시작

주 결과는 **`native_piece_route/sequence_native/`**다. 첫 구간은 물리 t=0·부착30개 활성·`continuation=None`에서 시작했고, 다른 MJB의 과거 부모 상태는 가져오지 않았다. 이후에는 이 새 MJB를 사용한 정상 완료 부모의 실제 상태만 이어받았다. 과거 결과나 추가 대조의 분리 수와 합산하지 않는다.

| 구간 | 실제 레일 Y 시작→끝 | 동작 | 현재 송이 새 분리(익음/미숙) | 미방문 분리 | 정책 | 평균 RTF / 최저 단계 RTF |
|---|---|---|---|---|---|---|
| Plant22 / Truss02 | 1.600→1.580m | 두 번 닫기·열기와 후퇴 완료 | 6 (5/1) | 0 | PASS | 1.071× / 0.269× |
| Plant27 / Truss00 | 1.580→2.653m | 두 번 닫기·열기와 후퇴 완료 | 6 (6/0) | 0 | PASS | 1.148× / 0.310× |
| Plant29 / Truss00 | 2.653→3.222m | 두 번 닫기·열기와 후퇴 완료 | 6 (6/0) | 0 | PASS | 1.087× / 0.322× |

총 **18개 연결 해제 = 색상상 익음17개 + 미숙1개**다. 미숙 열매는 Plant22 Fruit09다. 색상 기준 수확 대상21개 중17개가 분리됐고, 남은 부착12개는 색상상 익음4개와 미숙8개다. 익은 집단을 목표로 잡아도 인접 미숙 과실을 함께 분리할 수 있어 선택성은 미완료다. 각 송이에서4개씩 남았다. 최소 집단 크기3개 정책으로 송이당 한 집단을 방문했으며, 열매를 하나씩 전부 처리하는 시험이 아니다. 익음/미숙은 기존 authored RGB의 red hue±20°·saturation≥0.4·value≥0.15 기준인 SIM-GT proxy이며 생물학적 숙도나 카메라 검출 검증이 아니다.

세 구간 모두 중단 없이 finite state/qacc/capacity·접촉 정책 검사를 통과했다. 두 인계 경계의 qpos/qvel/eq/poses/time 차이는0이며, ctrl/damping/warmstart는 restore certificate로 확인했다. 다음 위치 명령도 실측 관절에서 시작한다. 각 구간의 분리 이벤트6개는 실제 equality1→0 변화와 정확히 일치했고, 재활성화0·시간 단조 증가를 확인했다.

| 구간 | other 최대 침투 | belt↔pedicel 최대 침투 | 접근 첫 개방폭 | 첫 닫기 / 재닫기 실제 최소 폭 |
|---|---:|---:|---:|---:|
| Plant22 | 0.865mm | 1.052mm | 95.995mm | 32.024 / 32.428mm |
| Plant27 | 0.851mm | 1.252mm | 95.995mm | 35.997 / 34.681mm |
| Plant29 | 0.835mm | 1.399mm | 95.993mm | 33.576 / 34.775mm |

벨트↔현재 목표 꼭지의1mm 초과는 기존 정확한 쌍·단계의2mm 예외 안에 있다. 따라서 운영 정책 PASS와 `raw_all_active_1mm_reference_passed=false`는 함께 기록한다. 명목 닫기 폭28mm와 실제 접촉 중 폭32~36mm를 구분한다. 접근 전 약96mm 사전 개방과 레일 정차는 저장 상태에서 확인했지만, executor의 phase 진입 실측 feedback gate를 새로 구현한 것은 아니다. 명시적으로 켠 분리 열매↔거터 진단 예외는 최종 세 방문에서 해당 접촉 발생이 모두0이었다.

| 구간 | 물리 시간 / loop wall | 레일 이동 RTF | 리프트 RTF | 접근 RTF | 닫기 RTF | 벨트 전체 RTF | 열기 RTF | 후퇴 RTF |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Plant22 | 57.004 / 53.240초 | 4.920× | 3.320× | 1.688× | 0.613× | 0.576× | 0.634× | 0.917× |
| Plant27 | 72.129 / 62.831초 | 2.605× | 2.682× | 0.893× | 0.520× | 0.501× | 0.500× | 0.877× |
| Plant29 | 62.684 / 57.672초 | 2.828× | 2.967× | 0.877× | 0.526× | 0.504× | 0.503× | 0.896× |

전체 물리 **191.816895초 / loop173.742993초 = 평균1.104027배속**이다. startup/JIT/solver setup155.801618초를 합하면329.544611초이며 계획·export·영상 생성은 별도다. 특히 `belt_finish`2sim초는 각각7.423/6.444/6.203wall초로0.269/0.310/0.322RTF이므로 **접촉 구간 실시간 목표는 아직 미달**이다. 이동 단계가 빠른 전체 평균으로 속도 문제가 끝났다고 해석하지 않는다. 위 묶음 구간은 일부 중첩되므로 각 행의 시간 합으로 전체를 다시 계산하지 않는다.

방문 전 attachment 최대 힘은 Plant22 구간0.111913N, Plant27 구간0.102573N으로 조기 분리는 없었다. 전체 접촉력 최대20.125/16.378/14.755N은 pair 식별 witness가 없어 목표 과실이나 비목표 물체의 힘으로 귀속하지 않는다. 주변6그루의 collider 포함·transform은 검증했으나, 실제로 어떤 주변 식물과 접촉했는지는 pair contact witness가 없어 미검증이다. `collision_certified=false`이며 모든 경로의 FCL 환경 사전 검사 통과로 보고하지 않는다.

실제 저장 pose로 만든 최종 세 방문의 전체 동작 영상은 결과 루트의 `native_video_list.json`에 있다. 뒤의 두 송이는 전체 동작 시점에서 팔·잎이 수확 부위를 가려, 같은 실제 상태의 접근 직전부터 후퇴까지 위쪽 시점과 표시용 cutaway 영상도 추가했다. cutaway는 기존 옵션으로 배경 식물과 활성 식물의 잎만 화면에서 숨기며, 물리 모델·저장 상태는 바꾸지 않는다. 이 옵션이 추가 두 타깃의 alias에도 적용되도록 표시 범위를 연결했고 숨긴 USD 경로를 `*_render.json`에 기록한다. 과거 비교와 추가 시점까지 포함한 목록은 `all_videos.json`과 `index.html`에 수록한다. 모두1배속이며 영상fps는 물리220/60Hz와 별개다. 명목 계획 영상을 실제 실행으로 제시하지 않았다. 새379DOF/native 모델의 GUI·headless 전체 물리 동등성 비교는 수행하지 않았다.

### 보존한 이전 기준: 단일 hull의17개 분리와 세 번째 중단

이전 `sequence_precise/`도 같은1e-8 옵션을 실제 GPU에 적용했지만, part161을 단일 hull로 표현한 **다른 collision model**이다.

| 구간 | 동작 | 현재 송이 새 분리(익음/미숙) | 미방문 분리 | 정책 | 평균 RTF / 최저 단계 RTF |
|---|---|---|---|---|---|
| Plant22 / Truss02 | 두 번 닫기·열기와 후퇴 완료 | 6 (5/1) | 0 | PASS | 1.078× / 0.260× |
| Plant27 / Truss00 | 두 번 닫기·열기와 후퇴 완료 | 6 (6/0) | 0 | PASS | 1.177× / 0.321× |
| Plant29 / Truss00 | 첫 닫기 중 중단 | 5 (5/0) | 0 | FAIL | 1.405× / 0.500× |

이전 총17개는 익음16+미숙1이며 최종 새18개에 더하지 않는다. 마지막 중단은 global164.350525s, 구간35.217529s, tick3798, `close`다. **Tool_L의 L/part_161 ↔ P29_Truss_00_Pedicel_distal_06_0**가 other class1.071557mm로 기본1mm를 넘었다. 삭제된 Fruit06의 꼭지 가지는 원본에 남은 형상이다. 배치 종료까지2step/9.094ms 추가 기록됐으며 정상 후퇴 완료가 아니다.

이전 물리164.360초 / loop139.203초 = 평균1.181RTF, setup154.988초다. 이전은 마지막 접촉 동작을 끝내지 않았고 새 모델은 다른 collision 표현으로 완주했으므로1.181→1.104 숫자를 엔진 속도 저하/향상의 증거로 사용하지 않는다. 이전 Plant22/27 벨트 묶음은0.573/0.511RTF였다.

### 추가 경로 대조

`sequence_shallow10/`은 원래 기본 tolerance 부모에서 depth35→25mm를 한 번 시험했고, 다른 꼭지와 Tool_R 접촉으로 중단했다. `sequence_precise_shallow10/`은 수정한 tolerance의 정상 완료 Plant27 상태에서 같은 깊이 변경을 한 번 시험한 **별도 대조**다. 원래 Plant29의 중단 이력은 보존했고 재시도 분리를 이전 기준17개나 최종 새 결과18개에 더하지 않는다. 추가 미세 조정을 반복하거나 중단 기준을 올리지 않았다. 세부 쌍과 시점은 `third_target_path_comparison.json` 및 최종 증거에 보존한다.

정밀 대조는 같은03/04/05 집단·aim·yaw·완료 부모 상태에서 depth만35→25mm로 바꿨다. 시작 qpos/qvel/eq/poses/time, 부모 SHA, MJB와 solver 옵션이 모두 같았다. 하지만 **같은 L/part_161↔남은 Pedicel_distal_06_0** 쌍으로34.817383초에 중단했고 other class 최대1.157543mm였다. 분리는 같은 익은 열매5개였으며, 단순히10mm 덜 들어가는 변경은 이 막힘을 해결하지 못했다.

해당 jaw 부품은 원본 USD에서 `convexDecomposition` 설정이지만 수정 전 이식 collider는 `convex_hull_per_source_mesh`였다. 따라서 이전 차단을 정확한 CAD 실물 형상에서도 수확 불가능하다는 뜻으로 해석하지 않았다. 원본 판 형상과 collision proxy의 덮임을 구분해 아래의 최소 수정으로 이어갔다.

추가 CPU 기하 검사로 해당 두 자세의 **convex hull 거짓 양성**을 확인했다. part161은1,028vertex/2,068triangle의 연결된 watertight 비볼록 메시이며, 단일 hull 부피가 원본의1.333배다. finite capsule-axis–triangle 거리, 축-삼각형 교차와 폐곡면 winding을 함께 검사했다. 원본 삼각형 표면과 capsule의 여유는 depth35mm에서 **+0.925911mm**, depth25mm에서 **+0.845696mm**였으나 native single-hull signed distance는 각각 **−1.072069mm/−1.157531mm**였다. 이는 두 저장 자세의 정확한 쌍에 대한 증거이며 전체 경로 인증은 아니다. 관련 결과는 `part161_triangle_vs_hull_diagnostic.json`, `part161_connected_components.json`에 있다. 단순 연결성분 분리로 해결할 수 없어 아래와 같이 원본 PhysX의 기본 cooked convex pieces를 재사용했다.

### 단일 hull을 원본 native16조각으로 수정

`newton_native_tool_pieces.py`는 위 한 부품의 정확한 native geom `s300_g0620`만 교체한다. 원본 USD `/World/ActuatedTool/L/part_161`의 `convexDecomposition`을 별도 fixture에서 SDK 기본 설정으로 cooking했고, 생성된16개를 선택 삭제 없이 전부 가져왔다. 이전 live runtime 캐시를 그대로 추출했다고 주장하지 않는다. native JSON은 source MJB·USD·collider·원본 삼각형·좌표 변환을 hash로 묶고, 이미 Tool_L body-local인 vertices를 identity geom transform으로 가져와 이중 변환을 막았다.

새 MJB는 geom2229→2244, mesh1254→1269이며 nv379/nq409/body357/attachment30은 유지한다. body/joint/equality/actuator, 질량·관성·마찰·contact params·runtime damping, 비선택 geom의 위치와 mesh 좌표/face, 초기 상태와 명령의 invariant **71,813개 PASS**를 기록했다. `discardvisual=true`도 유지했다. 원본 선택 mesh 파일은 source SHA와 함께 보존했고 사용되지 않는 원본 mesh가 새 MJB에서 제거되는 것은 명시한 layout 변화다. 따라서 새 MJB SHA를 가진 정상 초기 상태에서 다시 시작했으며 과거 부모 state의 SHA 검사를 우회하지 않았다.

| 원래 저장된 자세의 기하 대조 | 원본 triangle/capsule 여유 | 기존 single hull signed distance | native16 union signed distance |
|---|---:|---:|---:|
| depth35mm 중단 자세 | +0.925911mm | −1.072069mm | −0.278725mm |
| depth25mm 대조 자세 | +0.845696mm | −1.157531mm | −0.209167mm |

**작은 거짓 overlap은 남아 있어 strict nonintersection 음성 대조는 FAIL**이다. 다만 두 경우 모두 기존1mm runtime 중단 기준 아래로 줄었다. 실제 원본 plate 재료 안쪽0.5mm에 중심을 둔 양성 대조는 native union−1.324815mm로 차단했고,1mm 감시 기준을 높이지 않았다. 이 fixture는 해당 저장 자세에 대한 CPU 기하 증거이며 전체 경로·원본 CAD와의 완전 동등성은 아니다.

원본 vertex1028개 중1006, unique edge midpoint3102개 중3058, triangle centroid2068개 중2045가 native union 안/1nm 수치 경계에 들어왔다. 밖에 남은 표본의 실제 최근접 최대 거리는 각각0.612252/0.634583/0.434871mm다. 별도로 저장한 halfspace violation은 거리 하한이며 Euclidean 거리와 구분했다. 이 표본 측정은 **원본 전체 표면을 보수적으로 완전히 덮는다는 증명이 아니다.** 수치 재현성 tolerance를 biological/geometric universal HARD FAIL로 사용하지 않았다. 기본 native approximation의 과대·과소 덮임을 함께 공개한다.

근거는 `native_part161_default/native_pieces.json`, `native_piece_route/inputs_route/native_tool_pieces_proof.json`, `part161_native_piece_control_results.json`, `part161_native_source_sample_coverage.json`이다. 원본 OBJ SHA는 `1cc61d24215bec251fb687bd2b305c0644ee864870a1ef22f8b839331d2b4c65`, native JSON SHA는 `c24e0a06e332046f15788f2b4a423407edaea9241632ed13f295a06dd1fc056c`다.

### 검증과 범위

기존 관련 native 회귀·USD 베드 선택·row 시험의 **고유75개**와 native piece importer의 의미 있는 hollow-part/불변성/입력 거부 시험3개를 더해 **고유78개 PASS**다. 첫 pytest 호출은 해당 환경에 pytest가 없어 실패했고 기존 unittest 방식으로 수행했다. native3개도 첫 경로 설정 오류 후 올바른 cwd/PYTHONPATH로 통과했으며 `tests_native_pieces_final.log`를 보존했다. source와 입력 해시, 코드 snapshot과 실제 호출 인자는 결과 폴더에 보존한다.

현재 브랜치는 `codex/s300-newton-independent-tests-20261017`, 작업 시작 HEAD는 `c80f93ad1f6755934238397f6e950fed64553265`다. 이 HEAD가 이번 미커밋 구현을 포함한다는 뜻은 아니다. 기존 사용자 변경과 원본 결과를 보존했다.

전체 베드로 확대하기 전 남은 사항은 최소 집단이 없는 타깃을 건너뛰는 lineage, 실측 phase guard와 환경 사전 검사 연결, 접촉 구간 속도, 미숙 과실 선택성, native 근사의 표면 coverage 및 실제 포집 검증이다. 이번379DOF/native/tolerance 설정의 GUI·headless 새 비교나 학습은 수행하지 않았다.

## 재현 명령

현재 준비된 같은 쪽 세 타깃 입력으로 새 캠페인을 실행한다. 기존 캠페인 폴더는 덮어쓰지 않으며 `--campaign`은 새 단일 폴더 이름이어야 한다.

```bash
cd /root/farmily_tomato
./nvidia-sim/.newton-venv/bin/python \
  nvidia-sim/harvest_s300/run_newton_row_harvest.py \
  --prepared-root /root/docker_share/mujoko_debugging_data/20261018_010218_newton_same_bed_continuous/native_piece_route \
  --campaign "one_bed_native_tol1e8_$(date +%Y%m%d_%H%M%S)" \
  --solver-tolerance 1e-8 \
  --released-gutter-diagnostic
```

이 명령은 기존 `.asset-venv`로 다음 집단의 IK/경유점을 계획하고, `.newton-venv`의 기존 GPU 실행기로 physics를 실행한다. 기본 220Hz/자유 이동 60Hz, 접촉 capacity256·constraint capacity512를 사용한다. 한 번에 하나의 GPU 작업만 실행하도록 기존 `/tmp/s300_solver_compare_gpu.lock`을 사용한다.

거터 진단 예외의 기본값은 꺼짐이다. 위 명령은 저장된 진단 실험과 같은 범위를 재현하도록 `--released-gutter-diagnostic`를 명시한다. 옵션을 생략하거나 `--no-released-gutter-diagnostic`를 주면 이 예외를 적용하지 않는다. 이미 활성화해 완료한 부모 구간을 이어 실행할 때도 같은 옵션을 명시해야 한다.

원본 자산·입력 해시·같은 쪽 선택만 점검하려면 위 명령 끝에 `--validate-only`를 추가한다. 이 모드는 계획·physics·새 캠페인 출력을 만들지 않는다. 대용량 MJB와 입력 해시를 읽는 시간은 발생한다.

중단한 경로를 무조건 이어가는 옵션은 없다. `--start-index`와 `--parent`를 사용할 때도 바로 이전의 정상 완료 구간과 마지막 실제 상태를 검증해야 한다. 중단된 구간의 앞선 checkpoint로 돌아가 연속 성공을 만들지 않는다.

## 보존 증거

결과 루트 아래:

- `one_bed_selection.json`: 같은 베드 3타깃 및 반대쪽 Plant41 제외 사유, 원본 inventory SHA.
- `merged_model/`, `inputs_full/`, `inputs_coarse4/`, `inputs_fast/`, `inputs_route/`: 파생 단계별 공통 물리 모델과 hash.
- `display_assembly/`: 전체 농장 배경, 원본 목표와 추가 두 타깃의 표시용 alias·skin.
- `route_fixed_neighbors_selection.json`, `route_fixed_neighbors_verification.json`: 주변 고정 6그루의 선정·충돌체·transform 검증.
- `input_identity_review.json`: 30개 free/CONNECT/크기/초기 유한성/body 매핑 및 첫 타깃만 적용되는 belt scope.
- `sequence/`, `sequence_verified/`: 개발 중 원본 결과와 실패 증거. 최종 성공으로 재분류하지 않는다.
- `sequence_gutter_diagnostic/`: 기본 tolerance 전체 동작에서 발견한 방문 전 분리와 상태 인계 증거.
- `premature_future_releases_geometry.json`, `premature_future_releases_metrics.json`: 방문 전 분리의 저장 기하·힘·반복 수 조사.
- `force_probe_original/`, `force_probe_tolerance_1e8/`: 같은 최초 33초의 실제 계측 실행과 private buffer.
- `force_probe_original_analysis.json/.csv`, `force_probe_tolerance_1e8_analysis.json/.csv`, `force_probe_tolerance_comparison_33s.json`: raw/fresh/Jacobian/힘 균형과 타깃별 분리 비교.
- `native_part161_default/native_pieces.json`: 원본 SDK 기본 native16조각과 source/hash/좌표 출처.
- `native_piece_route/inputs_route/`: 최종 공통 새 MJB/XML/metadata 및 `native_tool_pieces_proof.json`.
- `native_piece_route/sequence_native/`: 새 정상 초기 상태에서 완료한 세 방문의 계획·실제 상태·분리·인계·영상.
- `report_native_metrics.json`, `NATIVE_SEQUENCE_ANALYSIS_KO.md`, `native_independent_review.json`: 최종18개·상태 인계·성능·실측 jaw/equality의 저장 자료 분석.
- `sequence_precise/`, `sequence_shallow10/`, `sequence_precise_shallow10/`, `third_target_path_comparison.json`: 이전17개 기준, 세 번째 차단 및 깊이 대조. 최종 결과와 합산하지 않는다.
- `part161_triangle_vs_hull_diagnostic.json`, `part161_native_piece_control_results.json`, `part161_native_source_sample_coverage.json`, `PART161_NATIVE_CPU_CONTROLS_KO.md`: 원본 surface/hull/native의 양성·음성 대조와 근사 한계.
- `native_video_list.json`, `all_videos.json`: 최종 실제 physics 영상 및 보존한 과거 비교 영상의 전체 절대경로.
- `test_suite_current_target_scope_and_continuation.log/.json`, `tests_native_pieces_final.log`: 초기60개와 최종 추가 시험의 원래 로그. 고유 회귀78개 PASS.

코드·입력·출력 SHA와 최종 테스트·영상 증거는 결과 루트의 `final_evidence.json` 및 작은 저장소 검증 요약으로 연결한다. 대형 MJB/영상/상태는 NAS에 보존하며 이 문서의 commit만으로 대형 자산이 Git에 포함됐다고 해석하지 않는다.
