# Farmily Tomato 작업 인수인계

## 최신 수정: 송이 50% 축소와 고리–주줄기 관통 판정

상세 변경·재현 명령·검증 한계: [HALF_SCALE_CONTACT_FIX.md](mujoco-benchmark/HALF_SCALE_CONTACT_FIX.md).

- 새 GLB 생성/수집 CLI는 `--truss-scale 0.5` 기본. 주줄기·잎·로봇은 그대로, 송이 전체 가지·꼭지·열매의 길이/반경 50%, 질량 1/8. 강성·감쇠·armature 유지로 동역학 동등성은 주장하지 않음. 원래 크기는 `--truss-scale 1`.
- 기존 크기 기록의 resume에는 `--truss-scale 1` 명시. 다른 크기를 같은 수집에 섞지 않도록 검사.
- `20260923_random_glb_full_smoke/scene_0001/targets/Tomato_05` 후보00005의 기존 성공은 잘못된 유효성 판정. 고리–기존 주줄기 관통3.860mm를 GLB 전용 검사에서 누락. 새 실행은 invalid_physics로 처리하고 학습에서 제외. 기하 중심 진입 조건은 변경하지 않음.
- 구버전 GLB 자료도 학습 로더에서 저장된 고리 관통 지표로 제외. 이전 학습 smoke 가중치/성능은 이 오류의 영향을 받으므로 유효 결과로 사용하지 말 것.
- 근거/독립 재실행: `/root/docker_share/mujoko_debugging_data/20260923_center_entry_audit/`. 기존 수치/상태는 보존하고 해당 기존 HTML에 정정 안내만 추가.
- 절반 크기3장면·기존 형상 비교·2후보 물리: `/root/docker_share/mujoko_debugging_data/20260923_half_scale_glb/comparison.html`. 정지3장면 통과, 로봇2후보 miss. 수확 성공 검증이 아님.

## 최신: 랜덤 GLB 장면 → 전체 열매 10후보·9 RGB-D → 장면 분할 학습 → 새 장면 물리 검증

2026-09-23, 브랜치 `mjlab-performance`. 기존 장면 생성기·후보 수집기·D435 촬영·camera action v2·학습기를 연결했다. **이번 작업의 커밋 여부는 `git status`로 확인한다.** 랜덤화는 GLB 송이 종류, 원래 주줄기·잎에서의 송이 부착 위치, GLB Y각도이며 GLB 안 개별 열매의 배열은 바꾸지 않는다. 9-view 출력은 원본 RGB·Depth를 저장하고 별도 열매 mask 및 Depth 유효성 PNG는 저장하지 않는다.

작은 전체 연결 시험: seed5의 3장면×송이1개×열매10개×후보10개 = 300회 시도. 계획 실패110, 물리 미진입160, 중심 진입18, 물리 오류12. RGB·Depth 각270장. 장면0/1/2를 train/validation/test로 분리했고 3모델×3seed 학습에서 검증 BCE로 ResNet18 RGB-D seed2를 선택했다. 테스트 장면은 발견된 진입이0개이며 계획 가능한 후보가 없는 열매3개를 포함한다. 실패 열매가 실제로 불가능할 수 있지만, 10후보 미발견을 그 증명으로 해석하지 않는다.

별도 seed24 새 장면에서 열매10개의 후보를 결과 없이 계획해 모델 추천을 확정하고, 추천10개를 새 MuJoCo 물리로 독립 실행했다. 중심 진입1(Tomato_10), 미진입9, 물리 오류0. 저장 상태10개 재생 해시 확인. 꼭지 걸림 수확 및 RGB-D 우위·일반화는 미입증. 수집 `/root/docker_share/mujoko_debugging_data/20260923_random_glb_full_smoke/index.html`, 학습 `/root/docker_share/mujoko_debugging_data/20260923_random_glb_training_smoke/index.html`, 새 장면 물리 `/root/docker_share/mujoko_debugging_data/20260923_random_glb_fresh_physics_smoke/index.html`. 명령·한계: [RANDOM_GLB_SCENES.md](mujoco-benchmark/RANDOM_GLB_SCENES.md), 보존 수치: `mujoco-benchmark/validation/random_glb_collection_pipeline.json`.

## 최신: 주줄기 송이 위치·Y각도 랜덤 물리 장면 생성

`mujoco-benchmark/scripts/generate_random_glb_scenes.py`는 seed로 GLB 종류·주줄기 segment 안 위치·Y축 각도를 재현 가능하게 뽑고, 기존 주줄기·잎을 유지한 여러 송이 물리 장면을 생성한다. 각 장면 초기 2초 겹침·정지 이동 검사를 저장하며 실패 장면도 표시한다. 2장면×2송이 샘플: `/root/docker_share/mujoko_debugging_data/20260923_random_glb_physics_final/index.html`. 실행 명령과 범위: [RANDOM_GLB_SCENES.md](mujoco-benchmark/RANDOM_GLB_SCENES.md).

2장면 모두 초기 검사 통과. 두 번째 송이를 대상으로 로봇 1경로 실제 실행: 최대 겹침0.347mm, 중심 미진입, 열매 이동52.51mm. 다중 송이의 이름 공간도 `RobotEngine`의 접촉 검사에 포함하도록 수정했다. 한 경로의 정상 실행을 전체 랜덤 장면의 수집 승인으로 해석하지 않는다.

## 최신: 전체 GLB 로봇 수집 소량 검사 완료

2026-09-23, 최신5종/각1개 전체 로봇 경로, Tomato_05, Y90°/segment11, 240Hz. white는2번 위치·강성 수정 모델, 첫seed0 준비자세IK실패 후seed1실행. 총 계획6회/실제물리5회. 결과: green/red/white miss(물리 기준통과), rotated90 partial_center_entry(물리 기준통과), cyan invalid_physics(최대겹침0.514mm로0.5mm기준초과, 학습제외). 경고/수치불안정0.

최대목표밀림 cyan219.00/green156.96/red81.02/rotated90 83.86/white90.58mm. 현재center_entry_only_v2는밀림을실패로분류하지않으므로 유효4개 또는 rotated90부분진입을 안전수확 성공으로 해석하지 않는다. 각1경로라 전체각도/부착위치 성공률이나대량수집승인아님. cyan 접촉중겹침은미해결로보존.

5개 초기RGB-D/목표mask640×480 저장, 각가상카메라로action_camera재변환 및복원검사통과. 후보states/trace해시·수치유한성·관측모델해시일치검사통과, 회귀12테스트통과. 관측의실제관절도달성은미검증.

결과 `/root/docker_share/mujoko_debugging_data/20260923_024914_all_glb_robot_smoke/index.html`. 보존요약 `mujoco-benchmark/validation/all_glb_robot_smoke.json`.


## 최신: white 2번 간격 및 탄성 안정성 수정

2번을 중심가지에서 멀어지는 방향으로3mm 이동(원본GLB 좌표 offset은 build.json), 꽃받침/말단 꼭지를 함께 이동하고 proximal 시작점은 유지해 변형했다. 원본GLB 보존, 나머지 body 위치와 모든 질량 유지 검사 통과. 6번 제거 및 연결부 필터 유지. 초기0.5mm 초과 겹침0개.

간격 수정만으로 힘 시험 미복원이 해결되지 않았다. 고정 중력 preload를 쓰는 초기 평형의 (중력 토크 미분+스프링) 대칭 행렬 최소 고유값이 -0.409로 국소 불안정 방향이 확인됐다. white 전용 `--rachis-stiffness-scale 2` 적용: Rachis k2→4, d0.15→0.2121. 최소 고유값+0.203. 물성 조정이며 실측 보정/기존과 동역학 동등성 아님. 다른GLB/기본값은1 유지.

240Hz 0.2N(2.5~3초) 이후12초까지 검사: 5번 X/Y/Z 최대51.89/19.43/26.00mm→잔류2.47/0.287/0.198mm. 2번 X 최대29.89mm→잔류1.47mm. 네 시험 모두 접촉 관통0, 경고0; 별도 구형 물체 접촉 반응26.24mm. 6초 시점 X잔류18.65mm로 빠른복원을 주장하지 않는다. 초기 정지2초 이동 수치오차 수준. 회귀12개 통과. 로봇 수집은 이 수정 모델로 미실행, 모든각도/부착위치 안정성은 미검증.

최신 white: `/root/docker_share/mujoko_debugging_data/20260923_white_fruit02_stable/index.html`. 이전모음의 white 실패기록은 보존한다. 요약 `mujoco-benchmark/validation/white_fruit02_fix.json`.


## 최신: 전체5 GLB 6번 제거 완료

cyan/green/red/white/rotated90 모두 `--remove-fruit 6` 적용, 열매10개 유지. 기존 주줄기·잎 및 Y90° 배치/segment11 유지. 각 모델 240Hz 정지·0.2N 힘·접촉·복원·reset 검사 완료, 회귀12테스트 통과. 초기0.5mm 초과 겹침은 cyan/green/red/rotated90에서0개. white는 Fruit_02와 Rachis_04 충돌체 약1.001mm 겹침이 별도로 남아 사용 보류. 또한 힘 시험 무부하 대비 최대/종료 변위가 모두626.09mm로 복원 검증 실패했으며, 이 큰 이동의 원인을 해당 초기 겹침으로 단정하지 않는다. 다른 알맹이를 임의 삭제하거나 해당 충돌을 숨기지 않았다. 초기2초 최대 바디 이동 cyan2.20/green2.30/red2.35/white3.18/rotated90약0mm. 로봇 후보 수집은 제거 모델들로 재실행하지 않아 전체 수집 물리 유효성은 미검증.

모음 `/root/docker_share/mujoko_debugging_data/20260923_all_glb_without_fruit06/index.html`. 요약 `mujoco-benchmark/validation/all_glb_without_fruit06.json`. 개별 하위폴더의 model.mjb/reference.json 사용. 원본GLB 보존.


## 최신: cyan 6번 알맹이 제거

`build_glb_physics.py --remove-fruit 6`로 Fruit_06 body/충돌/질량(60.45g)/꽃받침 시각/타깃을 제거한다. 꼭지 가지는 남기고 원본 GLB는 보존한다. 나머지 열매 번호 유지, 총10개. 기존9·10번 연결부 필터 적용. cyan Y90°, segment11에서 초기0.5mm 초과 겹침0개, 정지2초 최대 이동2.20mm. 0.2N 힘 시험 최대25.19mm, 잔류4.85mm. 접촉/복원/reset 및4테스트 통과. 로봇 후보 수집은 이 제거 모델로 재실행하지 않았으므로 수집 성공률/전체 물리 유효성 미검증.

결과 `/root/docker_share/mujoko_debugging_data/20260923_glb_without_fruit06/index.html`.


## 최신: 9·10번 정상 연결부 충돌 수정

GLB 첫 꼭지 가지와 바로 옆 중심가지 분절의 접합부 충돌 두 쌍 수정. cyan 정지2초 최대 이동51.69→8.66mm, 힘·접촉·복원 및4테스트 통과. 6번 열매 겹침4.93mm는 남아 학습 사용 미승인. 상세 [GLB_PHYSICS.md](mujoco-benchmark/GLB_PHYSICS.md).

## 최신: GLB 탄성·충돌 이식 및 로봇 수집 시험

기존 주줄기·잎과 GLB Y축 배치를 유지하여 241자유도 물리 모델로 이식했다. 힘/접촉에 의한 휘어짐·복원 및 후보3개 실제 실행 완료. 초기 내부 겹침4.93mm로 모두 invalid_physics이며 학습 사용 금지. 기존과 동일한 동역학이나 수확 성공을 의미하지 않는다. [구현·검증·실행 명령](mujoco-benchmark/GLB_PHYSICS.md).

최종 갱신: 2026-09-23 KST (실험 폴더 이름은 기존 기록의 표기를 그대로 유지함).
이 파일은 대화 기록 없이 다른 계정/새 세션에서 작업을 이어가기 위한 시작점이다.
아래 상태는 작성 시점 기준이므로, 재개할 때 `git status`를 먼저 확인한다.

## 최신: 단일 GLB 관측 수집 파일럿 — 로봇 실행 전 제외

- `collect_glb_pilot.py`: cyan송이1개/Y90°/기존 주줄기/목표Fruit_05 장면 생성, RGB·metric depth·유효깊이 마스크·목표 마스크·카메라 행렬 저장 및 초기 convex 겹침 검사 수행.
- 최종 출력 `/root/docker_share/mujoko_debugging_data/20260923_020731_glb_single_collection_pilot/index.html`. RGB640×480, 목표1765픽셀, 유효깊이42121픽셀. 배열/마스크/카메라 직교성 및 라벨 제외 검사 통과.
- 수집 전 검사에서 새 송이 충돌 꺼짐/탄성 없음/로봇 없음/장면별 계획·목표 참조 미연결을 검출. 상태rejected_before_rollout, 로봇 실행0회/라벨0개/training_eligible=false. 가상 카메라 관측이며 기존 D435/action14 학습 데이터로 바로 사용할 수 없다.
- 실제 경로 수집을 하려면 GLB 물리 이식과 장면별 로봇·계획·평가 참조 연결이 선행되어야 한다. 기존 데이터셋은 수정하지 않았다.

## 최신: GLB 부착 커밋 및 초기 겹침 검사

- 사용자 확인한 GLB5종/Y축0~180°/무작위 위치 부착은 `67483c7`로 커밋했다. GLB 원본5개 포함. push는 하지 않았다.
- `audit_glb_collisions.py`: 접촉 마스크를 우회한 명시적 convex 형상 거리 검사. 다중송이 장면1072형상에서 AABB 선별 후4592쌍 검사, 약1.49초. 임계값0.5mm.
- 서로 다른 송이 겹침0쌍. 털 제외 시 송이–주줄기7쌍/최대3.042mm, 송이–잎·잘린가지1쌍/최대3.201mm. 내부1292쌍은 정상 접합/꽃받침/convex 과대근사 포함이므로 실패 수로 해석하지 않는다.
- 원본 메시의 정확한 삼각형 관통 또는 동적 물리 검증은 아니다. 털 포함/제외 수치를 분리하며 원본/접촉 마스크는 변경하지 않았다. 새 송이 탄성·충돌은 여전히 미적용.
- 결과 `/root/docker_share/mujoko_debugging_data/20260923_multi_glb_collision_audit_classified/index.html`, 보존 요약 `mujoco-benchmark/validation/glb_initial_overlap.json`. 비활성 접촉 마스크 상태의 겹침/접함/분리 거리3조건 검사 통과.

## 최신: GLB 5종을 동일 주줄기에 무작위 부착

- 사용자 확정 규칙: GLB Y축0~180°만 회전. Y-up→Z-up 변환 C=Rx90 후 C·Ry(theta), 추가 X/Z 회전 없음. 기존 주줄기·잎 보존, GLB 원본 크기1 유지.
- `assemble_glb_plant.py --output NEW_DIR --seed 23 --view`: 5파일을 각각1번 사용, 주줄기 호 길이 구간별 무작위 위치에 동시 부착. `--axes` 선택 가능. seed 재현 가능, 출처해시/각도/부착점은 scene.json 저장.
- 결과 `/root/docker_share/mujoko_debugging_data/20260923_multi_glb_y_random/index.html`. red20.5°, cyan153.6°, green39.2°, white84.7°, rotated90 62.8°. 컴파일 후5송이 변환/Y축 위쪽 및 기존 주줄기302형상 위치·몸체 질량/관성 보존 검사 통과.
- 정지 시각 미리보기다. 송이 간·잎 간 겹침 필터와 송이 충돌/탄성은 미적용이며 학습용 물리 장면이 아니다.

## 최신 수정: GLB Y-up → MuJoCo Z-up 좌표계 변환

- `attach_glb_preview.py`는 좌표계 변환 C=Rx(+90°) 뒤 원본 GLB Y회전 적용: R=C·Ry(theta). GLB +X→월드 +X, +Y→월드 +Z, +Z→월드 -Y(추가회전0도). 이 좌표계 변환은 송이의 추가 X회전 조작과 구분한다.
- 기존 축 변환 없는 부착 샘플은 과거 결과다. 새 cyan0도 부착: `/root/docker_share/mujoko_debugging_data/20260923_glb_y_up_attached/axes.png`. GUI 열림, 주줄기·잎 보존. -90/-30/0/30/90도 모두 GLB Y가 월드 위쪽인지 검사.
- 새 송이는 시각 전용이며 탄성/충돌 연결 미완료. `view_glb_truss.py`의 이전 단독 뷰어는 아직 숫자 좌표 그대로 표시하므로 위쪽 축 수정은 부착 뷰어에 적용된 상태다.

## 현재 사용자 확정: 새 GLB, Y축 회전만

- 원본 송이5개: `nvidia-sim/env_usd/tomato_rotate_glb/*.glb`. 사용자 조건은 **GLB의 Y축만 회전하고 X/Z축 회전을 추가하지 않는 것**. 아래 과거 방향 추정/반구 제한을 새 기준으로 적용하지 않는다.
- `mujoco-benchmark/scripts/view_glb_truss.py FILE.glb`: 원본 노드 변환·스케일·좌표/원점 유지한 송이 단독 미리보기. `[`, `]`로 Y축±5도, `0`으로 원본. 마우스는 카메라만 회전. 원점 기준 공통 GLB XYZ 표시.
- 5파일 각154메시 읽기 및 Y회전시 Y좌표/원점거리 보존 검사 통과. cyan0도 화면 실행 확인. 시각 확인 단계이며 새 GLB의 탄성/충돌/주줄기 연결은 아직 구현하지 않았다. 원본 GLB는 수정하지 않았다.

## 현재 기준: 사용자 지정 단계로 롤백

사용자 요청으로 `20260923_tipward_surface_attachment` 단계의 생성 동작으로 복원했다. 짧은 시작 가지 기준 회전과 당시 표면 부착 계산을 사용한다. 이후 캡슐 축 순서 수정, 엄격한 접합 거부/재샘플링, 긴 중심가지/열매 중심 각도 변경은 현재 코드에서 되돌렸다. 아래 후속 변경 기록은 과거 이력이며 현재 동작이 아니다. 기존 주줄기·잎과 모든 결과 폴더는 보존한다. 이 단계의 알려진 초기 겹침도 복원되므로 수집 검증 완료로 해석하지 않는다.

기준 결과: `/root/docker_share/mujoko_debugging_data/20260923_tipward_surface_attachment/index.html`.

## 최신 수정: 부착점→열매 평균 중심 기준(v5)

- 사용자 최종 정정: 주줄기 표면 부착점에서 각 열매 중심 좌표의 평균을 향하는 방향으로 각도를 계산한다. 국소 주줄기 위쪽0°, 0~90도 허용. 아래 v3/v4는 과거 기준이다.
- 새9개 샘플: `/root/docker_share/mujoko_debugging_data/20260923_fruit_center_angle_samples/index.html`. 컴파일된 열매 body 좌표에서 각도 독립 재계산 통과. 부착 불량2개 제외 기록 보존.
- `--fruit-center-angle` 사용. 기존 주줄기·잎 보존, 접합/지지 가지 비관통 검사 유지. 전체 장면 물리/학습 수집 준비 완료는 아니다.

## 최신 수정: 긴 중심가지 기준 샘플

- 각도 기준을 짧은 시작 가지에서 **송이 Rachis 중심선 시작→끝 방향**으로 변경(v4). 국소 주줄기 위쪽0°, 0~90도 허용. 기존32도 등 v3 수치와 다른 정의다.
- `assemble_plant_scene.py --rachis-angle 60`으로 명시 가능. 생략시15~85도 seed 샘플링. 기존 주줄기·잎/송이 내부 형상 유지, 표면 접합·주줄기 관통 검사는 별도 유지.
- 새9개 샘플: `/root/docker_share/mujoko_debugging_data/20260923_rachis_angle_samples_checked/index.html`. 전체 장면 물리/로봇 수집 검증은 미완료.

## 최신 수정: 송이 허용 방향과 표면 접합

- 사용자 확정: 주줄기 위쪽 끝 방향0°, ±90° 허용. 국소 주줄기 끝 방향과 시작 가지의 각도가90° 이내인지 검사하며 뿌리 방향 배치는 거부한다.
- 기존 주줄기·잎 보존. 원본 캡슐 fromto 순서로 축 방향을 복구하고, 송이 시작 캡슐 끝을 주줄기 표면에 접합한다. 접합 간격/지지 가지 전체의 주줄기 관통을 컴파일된 형상으로 검사하고 실패하면 생성 거부.
- 새 미리보기 `/root/docker_share/mujoko_debugging_data/20260923_tipward_attachment_final/index.html`. 전체 장면 충돌/로봇 수집 준비 완료를 의미하지 않는다.

## 최신 수정: 기존 주줄기와 잎 보존

- 조립기가 기존 `plant_mujoco_optimized.xml`의 주줄기 트리를 복사하고 기존 송이만 교체하도록 수정했다. 주줄기16몸체와 잎·잘린 가지 포함302형상을 유지하며 주줄기를 재생성하지 않는다.
- 기존 월드 배치·메시·색상·질량·관성·관절·충돌 정책을 보존한다. 새 송이의 위치·종류·회전만 바꾸며 새 하중에 따른 물리 응답까지 동일하다는 뜻은 아니다.
- 전체 식물/송이 확대 미리보기: `/root/docker_share/mujoko_debugging_data/20260923_preserved_stem_leaves/index.html`. 새 송이 물리와 수집기 연결은 여전히 미완료.

## 최신 추가: USD 송이 부착 장면 생성 미리보기

- `assemble_plant_scene.py` / `view_plant_scene.py`: 원본 송이 선택, 주줄기 호 길이 위치·국소 회전 변경, seed 재현, MuJoCo XML/JSON/미리보기 HTML 저장.
- 온실 USD 54식물/162부착 사례 확인. 162종 형상이라는 뜻은 아니다. L_00의3개 송이 배치로6장면 생성·렌더 완료.
- 2초 물리 발산은 없지만 초기 충돌 겹침1.03~2.82mm로6개 모두 겹침 기준 실패. **형태 검토용, 학습 수집 준비 미완료**. 기존 모델/데이터는 보존.
- 로봇·리프트·30후보 수집기 연결, 새 장면 분할 및 Depth+mask 학습은 아직 구현하지 않았다.
- 상세 범위/명령/검사: [PLANT_ASSEMBLY.md](mujoco-benchmark/PLANT_ASSEMBLY.md).
- 결과 `/root/docker_share/mujoko_debugging_data/20260923_attachment_preview_convex/index.html`.

## 최신 상태: RGB-D 진입각 학습·물리 검증 완료

- 브랜치 `mjlab-performance`. 수집/경로 변경 커밋 `3f636c2`, 다중 토마토 학습·검증 커밋 **`ed13ca9`**.
- 현재 방향은 고정 경로 후보를 RGB-D로 평가하는 지도학습이다. 아래 RL 기록과 구분한다.
- 데이터10대상×30경로/710관측, Tomato_05는 없음. 중심 정렬, 하부2mm, 방위각±90° Sobol, 전진45°상승→후퇴45°상승. 최종 상승32.5mm. 정상 물리에서 중심진입이면 열매 밀림과 무관하게 부분 성공.
- 8대상 학습(01,02,03,04,07,08,10,11),09검증,06테스트. DINOv2 RGB-D seed2 선택.06의64관측 모두 candidate_00001(−81.09°) 추천.
- 고유 추천1경로를 새 물리로1회 실행하여 중심진입 성공 확인, 열매 최대변위30.78mm. 64회 독립 물리 성공이나 꼭지 수확 성공이 아니다.
- 영상 없는 기준 모델도 top1성공이므로 RGB-D 우위 미입증. 확률 calibration도 불량(Brier약0.315); sigmoid1.0을 실제 성공확률100%로 해석하지 않는다.
- 결과: `/root/docker_share/mujoko_debugging_data/20260922_234403_multi_tomato_training/index.html`.
- 물리/재생: 같은 폴더 `physics/index.html`. 실행 명령·학습 분할·해석은 [learning/MULTI_TOMATO.md](mujoco-benchmark/learning/MULTI_TOMATO.md).
- 다음 판단: 재생으로 실제 진입 확인, 다른 토마토 holdout 비교, 영상 없는 모델 대비 이점 평가. 현재 모델은 저장된 관측 특징과 기존 후보를 사용하며 새 실물 이미지 입력/자유 경로 생성 기능은 아니다.

## 이전 단계: Tomato_06 RL (당시 상태 기록)


- 브랜치 `mjlab-performance`, 구현 커밋 `a085f47`. 최신 작업 요약은 [RL 작업 인수인계](mujoco-benchmark/RL_HANDOFF.md), 세부 설계는 [TOMATO06_RL.md](mujoco-benchmark/TOMATO06_RL.md).
- 팔6관절+리프트를 자유 제어하는 mjlab/MJWarp + RSL-RL PPO 구현. 고정 장면 GT 사용, 카메라 정책/RL 수확 성공은 아직 아니다.
- 256환경 10분 추가학습 완료: 170업데이트/139만 전이. 평가 최근접 거리56.16→43.17mm, 성공은 전후0. 더 가까이 접근한 뒤 머무르는 상태.
- 근접 보상이 정지에도 지급되는 문제가 유력한 원인. **보상 progress_v2 수정 완료, 새 학습은 미실행**이며 물리적 진입 가능성도 추가 확인 필요.
- 1,000회 장기 실행은 중단했고, 10분 실행은 완료했다. 아래 과거 기록의 실행 중/PID 표기를 현재 상태로 해석하지 말 것.
- 다음 작업: 저장 평가 재생 확인 → 보상과 탐색 개선 → 동일 기준의 10분 비교. 무조건 장시간 학습부터 늘리지 않는다.

## 이전 카메라 데이터 수집 단계 기록 (이후 완료, 아래 내용은 당시 상태)

1. Tomato_05의 1,000개 경로를 카메라 좌표 action v2로 저장하고 재실험 완료: 510.83초. 기존 결과와 일치.
2. 71개 RGB-D 관측과 71,000개 관측–후보 쌍의 좌표 변환 검증 완료. 목표는 부분 중심 진입과 최대 이동량이며, 꼭지 걸림 성공 판정은 아직 없다.
3. ResNet18 / DINOv2의 고정 영상 특징 + 작은 평가기, 영상 없는 기준 모델을 각각 3회 학습 완료. 검증 손실 기준 선택은 영상 없는 모델이다. AP는 실제 성공 확률이 아니다.
4. 모델 추천 21회(서로 다른 경로 3개)를 새 물리 실행으로 확인하고 2배속 영상 저장. 영상 모델은 각각 7/7 중심 진입 및 최대 이동 20mm 이하. 동일 장면 반복이며 실물 일반화 검증은 아니다.
5. 05를 제외한 10개 토마토의 수집 실행기 준비 완료. 각 대상 1후보 + 1시점 촬영 소량 검증 통과. 이후 본 수집을 진행하다 사용자 중단: 01~04 완료, 06은 정상 저장931개/남음69개. 아래 최신 재개 기능으로 이어갈 수 있다.

중단된 수집을 이어가는 명령:

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/collect_tomatoes.py \
  --resume --output /root/docker_share/mujoko_debugging_data/20260922_134834_multi_tomato_collection \
  --candidates 1000 --workers 48 --planning-workers 16 --postprocess-workers 8
```

- 상세 수집 명령과 카메라 의미: [MULTI_TOMATO_COLLECTION.md](mujoco-benchmark/MULTI_TOMATO_COLLECTION.md)
- 학습 비교와 재실행 방법: [learning/README.md](mujoco-benchmark/learning/README.md)
- 센서 저장 형식: [D435_RGBD.md](mujoco-benchmark/D435_RGBD.md), [OBSERVATION_DATASET.md](mujoco-benchmark/OBSERVATION_DATASET.md)
- 결과 데이터·모델 가중치·영상은 `/root/docker_share/mujoko_debugging_data/`에 있다. Git에는 코드와 문서를 저장한다. 계정/머신 이동 시 이 디렉터리와 기존 MuJoCo 모델·계획 자산·Python 환경도 별도 보존해야 한다.
- 카메라는 대상별 가상 시점이며 실제 로봇 관측 자세의 도달성을 검증하지 않았다. 다음 학습에서는 토마토 ID 단위로도 학습/평가를 분리해야 한다.

## 이전 측정: Isaac 로봇 팔 비용 비교

- `mujoco-benchmark/scripts/robot_cost_comparison.py` 추가. 후보49 첫10초, full/CPU240Hz/PGS64/4,
  파단 비활성. 실제 관절 모터 명령을 재생하고 측정한 고리 경로로 단독 조건과 비교했다.
- 실행 시간: 고리 headless9.87초 / 팔 headless10.78초 / 고리 GUI18.62초 /
  팔 물리+외형 숨김20.41초 / 팔 외형 표시21.24초. 준비/검사/저장 제외, 각1회 측정.
- 팔 물리/제어 약+0.91초, 팔 외형 약+0.83초. 이번 조건에서는 팔보다 전체 화면 갱신 부담이 큼.
- 팔 세 조건의 상태 완전 일치. 고리 경로 차이0.00015mm 미만. kinematic/dynamic 구동 차이로
  식물 반응 차이 최대1.22mm는 있음. 모두 접촉/관통·부착 검사 통과. 고리걸기 성공률 시험 아님.
- 결과 `mujoco-benchmark/outputs/20260922_044331_robot_cost/index.html`, CSV/JSON/스크린샷.
  요약 `mujoco-benchmark/validation/robot_cost.json`, 해석 `RESULTS.md`.
- benchmark_isaac.py에 --robot-trace/--robot-visuals/--camera-scale 추가. MuJoCo 로봇 이식 아님.
  기존 GUI는 --gui --trajectory fixture_entry_lift --hz240 사용; 조명은 stage open 전에 추가.
- timestamp 경로 입력 수정, 초기 고리 USD notice를 모든 모드에서 flush 후 reset하도록 수정.
  실패한 초기 실험은 최종 표에서 제외. 자동 테스트7개 통과. 기존 Isaac 원본 코드는 유지.

## 최신 추가: 원본 식물의 MuJoCo 비교 (2026-09-22)

- 별도 `mujoco-benchmark/`에 이식/benchmark/viewer/RGB-D/영상 생성기 추가. 기존 Isaac 코드와 USD 유지.
- 기준 `8e0d820`의 **full** 모델. 90강체/300 DOF/식물376+고리35 collision 유지.
  팔은 제외하고 world 고리 기록을 직접 재생. 양쪽 파단 비활성화한 Phase 1이다.
- CPU 원본120Hz 100개 순차 실행: Isaac 전체565.27초, MuJoCo539.83초. 병렬100환경이 아니다.
  부착 오차0.5mm 초과는 각각5/100,22/100. 엔진 변경만으로 큰 속도 이득 없음.
- MuJoCo 원본60Hz는6/6 폭주.240/480Hz는6개 fixture 검사 통과, 각각 RTF1.07/0.54.
- 별도 최적화는 잎/잘린 가지 충돌286개 비활성, 열매free/weld 병합(파단 불가).
  120Hz10개 RTF3.46, 검사 통과.60Hz6개 중2개 약1mm 관통으로 정상 채택 불가.
- GPU RGB-D10초5.63초(RTF1.78), 실제 DISPLAY=:0 viewer10초5.73초. scoped NVIDIA EGL 설정 사용.
- 결과 `mujoco-benchmark/outputs/20260922_002000_original_comparison/index.html`: 영상16개·RGB-D·CSV/JSON.
- 결론/사용법/구조 차이: `mujoco-benchmark/RESULTS.md`, `README.md`, `CONVERSION.md`.
  테스트6개, 입력 동일성/초기 상태/반복성 확인. raw 출력/대형 모델/venv는gitignore,
  작은 보존 요약은 `mujoco-benchmark/validation/engine_comparison.json`.
- 아직 전체 로봇/자동 detach/병렬 MuJoCo/RL/탐색 미구현. 원본120Hz 부착 오차를 먼저 해결해야 한다.
- 이 추가 작업의 커밋 여부는 `git status`로 확인한다.

## 최신 확인: 초경량 GPU 1·16·32·64환경 비교

- 같은 후보49 저장 경로를 모든 복제에 재생했다. 서로 다른 후보 탐색이나 계획 성능 시험은 아니다.
- 각 환경의 첫 10초 처리: 1개 22.26초 / 16개 62.96초 / 32개 106.88초 / 64개 200.27초.
  GPU 1환경 대비 합산 처리량 5.66 / 6.66 / 7.11배. 32→64의 추가 이득은 약 7%다.
- 전체 접촉 실행도 완료. 모든 복제는 GPU 1환경과 상태·접촉 결과가 일치하지만,
  동일한 0.521mm 관통으로 invalid_physics 처리됐다. 0.5mm 기준 유지. 정상 데이터로 사용 금지.
- reset/파단 API 통과, 환경 간 접촉 혼선 및 네이티브 오류 없음. 128환경은 미측정.
- 64개 전체 재생 376.16초 중 물리 56.87초, 쓰기·물리·읽기 외 비용 288.08초.
  CPU 약 110%, 전체 GPU 메모리 최대 3534MiB. 다음 병목은 환경별 판정·상태 처리다.
- 추가 도구 `ultralight_scale_benchmark.py`; 관련 자동 테스트 4개 통과.
  로그는 한글. 보고서 `runs/20260921_231100_ultralight_gpu_scale/benchmark.md` 및 CSV/JSON.
  보존 요약 `nvidia-sim/rl/validation/ultralight_gpu_scale.json`; 상세 `ULTRALIGHT_PLANT.md`.
- 모델/물리 설정은 이번 비교에서 변경하지 않았다. 사용자 세션은 유지했다.

## 이전 수정: 초경량 모델과 CPU 단일 환경 실시간 미리보기

- 이전 light 작업은 `a270554`로 커밋했다. 그 뒤 초경량 작업의 커밋 여부는 `git status`로 확인한다.
- `--plant-resolution ultralight` 추가. full/light와 원본 USD 유지. 기본값 full/GPU 유지.
- 식물 79강체/234자유도(full), 79/165(light) → **33/66(ultralight)**.
  줄기 캡슐 68개는 원래 위치·반경대로 유지하고 여러 조각을 한 강체에 묶는다.
  전체 활성 식물 충돌체 90개, 총질량 유지. 목표 꼭지 3분절, 열매 11개와 파단 유지.
  원거리 말단 힌지 병합으로 물리 응답과 충돌 제외 범위는 달라진다.
- `--physics-device cpu --num-envs 1` 추가. 실제 CPU PhysX/CCD, GPU 렌더링이다.
  120Hz/PGS64 유지, 매 스텝 0.5mm 관통 검사 유지. CPU/GPU 데이터·영상 비교를 분리한다.
- CPU 초경량 10초 처리 7.54초(계획·판정·저장 포함), full CPU 12.54초.
  화면을 켠 10초 고정 구간도 9.24초에 처리했다.
  초경량 화면 표시 후보49는 16.48초 분량을 13.71초에 실행. 초기 로딩/촬영 별도.
- CPU 정지·휘어짐/복원·파단·reset·4방향 선삽입 고리 유지 검사 모두 통과.
  실제 후보49는 excessive_displacement이며 수확 성공이 아니다. 관통 검사 통과.
- GPU 초경량 4환경×10초는 46.41초. 이전 light의 66.08초보다 개선됐지만 실시간은 아니다.
  GPU 60Hz/낮은 반복 수의 후보49는 관통 기준 초과. 그 설정을 기본값으로 채택하지 않았다.
- 상세 실행법/범위: `nvidia-sim/rl/ULTRALIGHT_PLANT.md`.
  실험: `nvidia-sim/rl/runs/20260921_ultralight/`, 요약: `validation/ultralight_plant.json`.

## 이전 수정: 경량 모델의 주줄기 잎·잘린 가지 충돌 제외

- `full`은 그대로 기본값이다. `light`는 이제 `--main-appendage-collisions ignore`가 기본이다.
  `light --main-appendage-collisions keep`으로 이전 경량 모델을 비교할 수 있다.
- 주줄기 부속 잎·잘린 가지 286개 convex 충돌체를 생성하지 않는다. 원본 시각 메시와
  주줄기 추종은 유지한다. 두꺼운 주줄기, TRUSS 송이 가지·꼭지, 열매 충돌은 유지한다.
- 송이는 이미 원거리 관절을 63 → 50개로 줄였다. 목표 주변의 충돌 형상은 유지한다.
- 정책을 config/backend/CSV/JSON/summary/대표 영상에 전달하고 정책이 다르면 resume 및
  영상 비교를 거부한다. 정책이 없는 이전 실행은 keep으로 해석한다.
- 실제 활성 충돌체 개수는 plant_model.json의 collision_inventory로 확인한다.
- 실제 집계 376 → 90개(줄기 68, 열매 11, 열매 꼭지 11). 4환경/10초 비교:
  full 초기화/실행 17.76/76.03초, light keep 17.67/68.55초, light ignore 9.85/66.08초.
  이번 잎 충돌 제외의 추가 실행 개선은 약 3.6%로, 주된 효과는 초기화와 충돌체 수 감소다.
- 관절·줄기 형상/질량·남은 충돌 필터는 light keep과 동일하고 초기 depth가 정확히 같다.
  관련 자동 테스트 39개 통과. 실행 폴더 `nvidia-sim/rl/runs/20260921_main_appendages_comparison`.
- ignore로 5·49 접촉 실행 완료: 과도 변위 2개, 관통 제외 0개(최대 0.222/0.317mm),
  잎/잘린 가지 접촉 이벤트 0개. 정지/휘어짐·복원/파손/리셋/네 방향 고리 fixture 유지 통과.
  검증 요약 `validation/main_appendage_light.json`. 수확 성공을 의미하지 않는다.
- 자세한 실행법/범위: [LIGHT_PLANT.md](nvidia-sim/rl/LIGHT_PLANT.md).

## 이전 수정: 원본을 유지하는 선택형 경량 식물 (잎 충돌 keep)

- 원본 USD와 기본 모델은 유지한다. GPU dataset의 `--plant-resolution full`이 기본값,
  `--plant-resolution light`로만 별도 경량 구조를 사용한다. 자동 출력 폴더는 `_light`로 구분.
- 충돌체·질량·열매 11개와 파손 연결은 유지. 목표 70mm 이내/목표 꼭지는 원래 관절 유지,
  먼 구간은 관절 4개당 약 1개만 휘게 하고 나머지는 부모에 고정 연결한다.
- Tomato_05 자유도 234 → 165. 주줄기의 탄성 연결 15 → 5개, 잎 충돌체 286개는 유지.
- 동일 4환경/10초 비교: 실행 75.43 → 66.86초(11.4% 단축), 물리 45.76 → 38.23초.
  64/256환경 처리량은 재검증하지 않았다. 초기화는 각각 18.06/17.77초.
- full의 4후보 처음 600스텝이 기존 원본 trace와 정확히 일치했다.
- light 0·1·5·49 완료: miss 1개/과도 변위 3개/관통 오류 0개. 5번은 원본의 부분 진입과
  달리 조기에 과도 변위로 중단되어 모델 간 판정 차이가 있다. 최종 후보는 원본 재검증 필요.
- 실제 모델·관절 목록을 plant_model.json에 저장하고 JSON/CSV/summary/영상에도 종류를 전달한다.
  서로 다른 모델로 resume/영상 비교를 통과시키지 않는다. 관련 자동 테스트 37개 통과.
- 경량 mechanics: 정지/휘어짐·복원/파손/리셋/고리 유지 통과. 0.2N 최대 변위 24.67mm,
  잔류 1.51mm, 연결부 최대 간격 0.022mm. 네 방향 고리 fixture 유지 통과.
  로봇의 진입 성공을 의미하지 않는다. 검증 요약 `validation/light_plant.json`.
- 자세한 범위/실행법: [LIGHT_PLANT.md](nvidia-sim/rl/LIGHT_PLANT.md).
  검증 실행: `nvidia-sim/rl/runs/20260921_plant_model_comparison`,
  경량 접촉 결과: `nvidia-sim/rl/runs/20260921_light_plant_4env`.

## 이전 수정: 단일 환경 GUI 속도 개선

- 동일 49번 GUI 실행 구간 150.34초 → 103.95초(약 31% 단축).
- 단일 환경은 원본 하우스를 표시하며, 다중 환경은 표시용 격자를 유지한다.
- 화면용 USD 동기화를 렌더 시점으로 제한하고 식물 skin 계산을 캐시/일괄 적용한다.
- 120Hz 물리/60Hz 제어/관통 검사 유지. 계획 명령, 제어 trace 995개,
  물리 접촉 trace 1,990개, 접촉 이벤트 306개 및 audit이 원본과 정확히 일치했다.
- 화면 시간 960Hz 고정값 수정. 화면 비용 기반 갱신율 조정 및 display_timings 추가.
- 실행법/검증 범위: [SINGLE_VIEW_PERFORMANCE.md](nvidia-sim/rl/SINGLE_VIEW_PERFORMANCE.md).
  대규모 환경 처리량 개선으로 일반화하지 않는다.

## 이전 수정: candidate_00049 접촉 관통 대응

- GPU dataset 기본값은 `contact120`: 120Hz 물리 / 60Hz 제어 / PGS 64/4 /
  armature 5e-4. 이전 practical60과 동일한 형상/물성/로봇 명령에서 timestep만 절반.
- 49번 동일 명령의 매 물리 스텝 겹침: 60Hz 최대 1.793mm → 120Hz 0.196mm.
  수정 후 줄기가 밀려 목표 변위 20mm 중단. 수확 성공으로 바꾼 것이 아니다.
- contact offset/CCD만 조정, 드라이브 강성 감소, velocity iteration 증가만으로는
  관통이 남았다. TGS 60Hz는 초기 파손. 120Hz 32/4도 0.5mm 선별 기준 미달.
- 새 dataset은 실제 USD/native pose의 고리 arc/rail 대 식물 capsule/sphere 겹침을
  매 스텝 검사한다. 0.5mm 초과면 `invalid_physics`로 중단/별도 저장/대표 영상 제외.
  mesh와 스텝 사이 연속 swept 검사까지 보장하는 것은 아니다.
- `physical_audit.json`, `tool_contact_trace.json`, CSV 물리 유효성 필드를 저장한다.
  오류를 숨기거나 고리/식물 pose를 인위적으로 보정하지 않는다.
- 문서: [CONTACT_PENETRATION_FIX.md](nvidia-sim/rl/CONTACT_PENETRATION_FIX.md).
  진단 원본: `nvidia-sim/rl/runs/20260921_contact_fix/`.
  기존 원본 데이터/영상은 변경하지 않고 새 결과와 구분한다. 실행 원본과 영상은
  git 제외 대상인 `runs/`에 로컬 보관하고 코드·문서·검증 요약을 커밋한다.
- 4환경에서 0·1·5·49 완료: 관통 선별 기준 초과 0개, 과도 변위 2개/miss 2개.
  5번 중심 진입 부분 성공 0.833초 유지, 완전 걸림 0. 모든 관절 명령은 원본과 동일.
  정지/탄성·복원/파손/리셋 및 미리 끼운 고리 네 방향 접촉 유지 fixture 통과.
- 49번 단독 측정: 60Hz guard 45.17초 / sim16.183초,
  120Hz 84.65초 / sim16.583초. sim 시간당 비용 약 1.83배, 초기화/렌더 제외.
  전체 64환경 처리량은 아직 재측정하지 않았다.
- 수정 전후 2배속 영상:
  `nvidia-sim/rl/runs/20260921_200605_candidate49_contact_fix_2x/index.html`.
  검증 요약 `validation/contact_penetration_fix.json`, 관련 자동 테스트 37개 통과.

## 이전 진단: candidate_00049 물리 관통 확인

- 사용자 영상 지적으로 저장된 native pose/접촉을 재검사했다. 49번 고리 segment_22가
  TRUSS_Rachis_08을 삽입 중 16.6~16.9초에 관통했다. 최대 겹침 1.793mm.
- 재실행 비교 통과와 물리적 유효성은 다르다. 이전의 49번 "정상 비목표 접촉 사례"
  해석을 정정하고 영상 모음에서 물리 오류로 분리했다. 대표 2개/진단 2개로 변경.
- 원본 CSV/판정은 보존하고 candidate_00049/physical_audit.json을 추가했다.
  당시에는 자동 제외 필터가 없었다. 후속 수정으로 새 dataset에 제외 검사를 추가했다.
- 근거: `nvidia-sim/rl/runs/20260921_candidate49_collision_audit/README.md`,
  `validation/candidate49_penetration.json`. 이 변경의 영상 분류 테스트 6개 통과.
- 다른 영상의 rachis 겹침 선별: 05 최대 0.266mm, 01 없음, 00 최대 0.701mm.
  전체 64개/모든 충돌체 검증은 아니다. 이 최초 진단 때는 물리 설정을 변경하지 않았다.

## 최신 후속 작업: 64환경 실측과 대표 영상 자동화

- 64환경/64개 서로 다른 staged6d 후보, CPU 경로 16 workers, PhysX CPU threads 8.
  practical60 물리/로봇 속도/중단 기준 유지. 실제 전체 실행 **814.70초 (13분 35초)**.
- rollout 740.32초: physics 481.42초, 실행/판정 192.93초, 명령 쓰기 38.61초.
  CPU 경로 작업 전체 10.59초. 명령 FK를 미리 계산하고 동일 tick 조회를 재사용했다.
- 이전 4환경/8후보 대비 후보당 관측 처리량 2.64배. 풀 크기/조기 중단 비율이 달라
  순수 코드 최적화 효과로 해석하면 안 된다. 64/64는 마지막 빈 슬롯 손실이 크다
  (active env-step fraction 0.414). 대량 실행은 continuous 재충전을 사용한다.
- CPU 초기화 최대 2801%, 경로 계산 시 최대 약 1892%, rollout 중앙값 117%.
  GPU 메모리 최대 6191MiB, 샘플 GPU 사용률 중앙값 25%. CPU 직렬 판정과
  physics 동기화 병목이 남아 있다. 전체 자원을 최대 활용하는 최적 설정을 찾았다는 뜻은 아니다.
- 54개 과도 변위, 8개 miss, 2개 비목표 접촉, 완전 걸림 0.
  candidate_00005만 안전 기준 내 중심 진입 1.22초 달성, 최종 miss.
- 이전 8개와 새 관절 명령 파일은 전부 수치적으로 동일. 최종 범주/부분 진입 여부도 일치.
  GPU 환경 수 차이로 과실 최대 변위 수치는 최대 0.294mm 차이가 있었다.
- 원본: `nvidia-sim/rl/runs/20260921_175044_staged6d_64env_videos/`.
  `performance_report.json`, `resource_samples.json`, `dataset/summary.json` 참고.
- `--representative-videos`로 완료 후 대표 사례 선정→저장 명령 재실행→원본 trace 비교→
  두 외부 시점 2배속 MP4/HTML 생성. 부분 성공을 완전 걸림으로 바꾸지 않는다.
  자세한 설명: [DATASET_REPRESENTATIVE_VIDEOS.md](nvidia-sim/rl/DATASET_REPRESENTATIVE_VIDEOS.md).
- 관련 자동 테스트 35개 통과. `representative_videos_2x/index.html` 영상 생성 완료:
  00005 부분 진입, 00001 과도 변위, 00049 비목표 접촉은 원본 비교 통과.
  00000은 최종 miss는 같지만 중간 주줄기 변위가 최대 15.29mm 달라 별도 진단 영상으로 분리.
  두 시점 2배속, 1280x612/30fps, 네 MP4 전체 디코드와 프레임 수 검사 통과.
  영상 후처리 추가 시간은 843.39초(약 14분). `validation/trajectory_64env.json`에 검증 요약.
  재실행이 원본 병렬 실행과 항상 같지는 않으므로 모든 원본 영상이 필요하면 본 실행에서
  물리 pose를 저장하는 방식으로 확장해야 한다. 현재 코드는 차이를 숨기지 않고 표시한다.

## 이전 작업: 떨어진 위치부터 접근하는 6D 소규모 시험

- 기준 커밋 `973806b` 이후 작업. [TRAJECTORY_PILOT.md](nvidia-sim/rl/TRAJECTORY_PILOT.md) 참고.
- `--trajectory-mode staged6d` 선택 시 170mm 떨어진 준비 위치→진입→수평 삽입→상승→1초 유지.
  기존 legacy 기본 동작은 그대로다. 새 모드는 접촉만으로 상승을 조기 종료하지 않는다.
- 실제 로봇 4환경·8개 Sobol 후보: 한 번의 완료 실행 269.06초, rollout 234.25초.
  6개 과도 변위, 2개 끝까지 실행 후 miss. candidate_00005는 GT 중심 진입 1.87초를
  달성했지만 꼭지 걸림은 아님. 이 새 후보의 영상 검증은 아직 하지 않았다.
- 동일 4환경/실패 비율 단순 예상: 64개 약 32분, 128개 약 63분, 2,048개 약 16시간 40분.
  영상 제외, 8개 표본의 제한이 있다. 새 경로의 16/32/64환경 처리량은 미측정이다.
- 완료 원본은 `runs/20260921_170634_staged6d_timing_pilot/env4_n8_final/`.
  앞선 두 폴더는 대화 중단으로 종료된 불완전 실행이다. 마지막 그래프 변수명 오류는
  물리 8개가 모두 저장된 뒤 발생했고 오프라인 후처리로 복구했다. 원래 오류 기록 보존.
- 관련 테스트 30개 통과. 새 source fingerprint 및 두 parameter schema의 plot 지원 추가.

## 이전 작업: 실용 60Hz

사용자가 960Hz와 완전히 같은 결과보다 밀림·걸림·휘어짐·복원을 유지하는 실용적
처리량을 우선하도록 기준을 변경했다. 따라서 아래의 이전 "960Hz 유지" 작업 지침은
후속 작업에서 대체됐다. 원래 성능/물리 이력은 삭제하지 않고 보존한다.

- 기준 커밋은 `bc84d41`; 실용 preset/검사/문서와 관찰 영상 도구는 후속 커밋에 포함했다.
  한글 커밋 제목은 `시뮬레이션: 실용 60Hz GPU 물리 설정과 접촉 관찰 영상 도구 추가`다.
- GPU dataset 기본은 `--physics-preset practical60`: 60Hz, armature 5e-4, PGS 64/4 iterations.
- `reference960`으로 원래 960Hz/1e-5 모델 선택 가능. `practical120`은 120Hz/3e-4 중간 비교용.
- 60Hz/5e-4에서 정지·실제 과실 하중/복원·원래 임계값 파손·리셋·네 방향 CAD 고리
  접촉 유지 검사가 통과했다. 고리 fixture는 미리 삽입된 별도 kinematic 도구이며 로봇 진입 성공은 아니다.
- 저장 로봇 경로 4환경 재생: 17.50초에 sim 4.4초 실행. 과거 960Hz는 168.69초/sim 4.333초였다.
  물리 모델이 다르므로 정밀 동등성 speedup이라고 부르지 않는다.
- 16환경 같은 경로 재생도 27.71초/sim 4.4초로 통과했고 native 접촉 라우팅/reset 오류가 없었다.
- 실제 dataset 4환경/서로 다른 4후보/8초 제한: RGB-D·계획·reset 통과. 결과 1개 과도 변위,
  3개 incomplete로 전체 수확 성공 검사는 아니다. 관련 자동 테스트 60개 통과.
- 30Hz/1e-3 잔류 변위가 커 제외. 60Hz/1e-3/32 iterations는 연결 간격 0.555mm로 0.5mm 기준 초과해 제외.
- 60Hz/3e-4 초기 검사는 idle을 과실만 검사했던 문제가 있었다. 주줄기도 포함하도록 수정했고
  최종 채택 모델은 5e-4다. 초기 `behavior60`의 passed 값을 그대로 최종 판정에 사용하지 않는다.
- 코드: `gpu_physics_presets.py`, `gpu_behavior_probe.py`, `gpu_low_hz_benchmark.py`, GPU runner/sim 변경.
- 새 검증 원본: `nvidia-sim/rl/runs/20260921_low_hz_practical/` (ignored).
- 최신 문서/실행법/레퍼런스: [GPU_PRACTICAL_PHYSICS.md](nvidia-sim/rl/GPU_PRACTICAL_PHYSICS.md).
- 60Hz 동작 관찰 영상: [GPU_PHYSICS_VIDEOS.md](nvidia-sim/rl/GPU_PHYSICS_VIDEOS.md).
  `runs/20260921_162920_practical60_contact_videos_2x/`에 실제 로봇 밀림, 하중/복원,
  미리 끼운 고리 유지의 두 시점 영상과 원본 pose/contact를 저장했다. 2배속 기본,
  고리 1배속 추가. 촬영 코드는 후속 커밋에 포함했고, 영상은 ignored runs에만 있다.
- 요약: [gpu_practical_physics.json](nvidia-sim/rl/validation/gpu_practical_physics.json).
- 다음에는 실제 로봇의 성공 진입/상승 경로, 타겟 1~11, 빠른 접촉과 더 많은 환경 수를 검증한다.
  아직 실측 보정이나 모든 상황의 비관통 보증은 없다. RL/DR을 추가하지 않았다.

## 1. 현재 결론과 바로 이어갈 일

- 기존 작업은 **`1305651`**까지 커밋했다. 브랜치는 **`nvidia-sim`**이다.
- 그 이후 **저주파 탄성 안정성 진단·설정 옵션·검증 자료를 이 문서와 함께 후속 커밋에 포함**했다.
  해당 커밋 제목은 `feat(sim): add elastic stability diagnostics and handoff`이며 해시는 `git log`로 확인한다.
- CPU 경로 계산 병렬화는 검증했고 기본 최대 8개 worker로 동작한다.
- GPU 물리 및 여러 환경의 동시 실행은 구현되어 있다. CPU 물리 worker pool과 혼동하지 않는다.
- 현재 핵심 미해결 문제는 **물리 결과를 유지하면서 960Hz를 낮추는 것**이다.
- 보조 관성(armature)을 키워 저주파의 정지 흔들림은 줄였지만, 접촉 변형과 파손 응답이 달라졌다.
  **60/240/480Hz를 기존 물리의 동등한 고속 대체 설정으로 채택하지 않았다.**
- 기본값은 **960Hz, armature 1e-5 kg·m², 로봇 제어 60Hz**를 유지했다.
- 이 파일을 작성할 때 관련 dataset/probe/planning worker는 실행 중이지 않았다.

다음 담당자는 [저주파 실험 상세](nvidia-sim/rl/GPU_ELASTIC_STABILITY.md)와
[수치 기록](nvidia-sim/rl/validation/gpu_elastic_stability.json)을 먼저 읽고,
후속 커밋의 코드를 검토한 뒤 탄성 관절·초기 하중·파손 연결부의 시간 간격 민감도를 조사하면 된다.
이미 실패한 설정을 다시 기본값으로 채택하거나, 같은 실패 라벨만 보고 물리 동등성을 선언하지 않는다.

## 2. 사용자 목표와 제약

사용자는 고리형 그리퍼로 방울토마토 아래에서 접근하고, 안으로 진입한 뒤 상승하여
꼭지 윗부분을 걸고 필요하면 당기는 동작을 연구 중이다. 주줄기를 밀면 송이와 과실도
움직여 고리걸기가 실패하는 상황까지 시뮬레이션되어야 한다.

현재 단계는 **고정 장면의 접근 pose 후보 → 실제 physics 실행 → 결과 데이터셋 생성**이다.
현재 성능/물리 진단 타겟은 `Tomato_05`이며, 장기적으로 Tomato_01~11 및 다양한 실험을 다룬다.

- 이번 데이터 생성 단계에서는 RL/IL, pose inference, segmentation 학습, domain randomization을 하지 않는다.
- 후보 생성과 사후 판정에는 simulator GT를 쓸 수 있다. 실제 perception이 구현되었다고 설명하지 않는다.
- RGB-D는 **진입 전** 촬영한다. 후보 실행 뒤 성공/실패 라벨을 붙인다.
  성공한 순간의 사진만 저장하는 방식이 아니다.
- 목표를 바라보는 관찰 자세, 원본 영상과 타겟 주변 문맥을 포함하는 crop을 사용한다.
  꼭지나 고리가 항상 보인다고 가정하지 않는다. 가림 상태도 실제로 존재한다.
- 장기 sim-to-real에서는 카메라와 joint 정보를 우선 고려하고, 힘 센서는 제외하고자 한다.
- 사용자가 제공한 house/식물/로봇 장면을 기반으로 한다. 무관한 단순 환경으로 바꾸지 않는다.
  멀티환경 표시/실행 최적화의 세부 범위는 기존 문서와 코드를 확인한다.
- 식물 위치 예: `--spawn-stem -0.75 0.55 0.32`.
  초기 자세는 ROS2 GUI의 left pick ready 계열, 리프트는 타겟 송이보다 0.4m 낮게 시작하는 요구가 있었다.
  실제 GPU runner의 cfg 설정을 기준으로 확인한다.
- 영상/실험 출력은 datetime으로 시작하고 간단한 설명이 붙은 폴더에 저장한다. 영상은 주로 2배속 요청이었다.
- 목표 처리량은 약 1,000개 후보/1시간이지만 **달성 검증은 안 됐다**.
- 최근 사용자는 큰 실험의 로딩을 피하려고 **1환경과 4환경으로 먼저 비교**하도록 요청했다.
  성능 검증 때 곧바로 256/1,000환경을 띄우지 않는다.

`--num-envs 32 --candidates 128`은 총 128개 pose를 최대 32개씩 실행한다.
32×128개의 후보를 생성하는 것이 아니다. `continuous`는 빈 슬롯에 다음 후보를 넣는 방식이다.

## 3. 실행 환경과 Git 상태

| 항목 | 값 |
|---|---|
| 저장소 | `/root/farmily_tomato` |
| 브랜치 | `nvidia-sim` |
| 인수인계 포함 후속 커밋 | `feat(sim): add elastic stability diagnostics and handoff` (해시는 `git log` 확인) |
| 성능 개선 기준 커밋 | `1305651 perf(sim): precompute candidate paths on CPU workers and benchmark timesteps` |
| 이전 관련 커밋 | `ec0c9db perf(sim): batch GPU commands and continuously schedule candidates` |
| 이전 관련 커밋 | `8995518 feat(sim): scale GPU candidate experiments and add live grid viewer` |
| 이전 관련 커밋 | `d14589d feat(sim): add parallel dataset workers, GPU probes and result dashboard` |
| 런타임 | Isaac Sim 5.1 + Isaac Lab |
| Python | `/root/isaaclab_env/bin/python` |
| GPU | RTX 3090 Ti, 약 24GB |
| CPU | Threadripper 3960X, 24코어/48스레드 |
| 물리 모니터 | `DISPLAY=:0` 사용 가능 |

`run_gpu_candidate_dataset.sh`는 Python 경로를 `FARMILY_ISAAC_PYTHON`으로 재지정할 수 있다.
기본은 위 Python이며 `PYTHONHOME`, `VIRTUAL_ENV`를 해제하고 `PYTHONPATH`를 설정한다.

후속 커밋에 포함한 파일(아래 상태 문자는 커밋 전 기록이며, 이 문서도 함께 포함):

```text
 M nvidia-sim/rl/GPU_TIMESTEP_AND_PLANNING.md
 M nvidia-sim/rl/elastic_plant.py
 M nvidia-sim/rl/gpu_dataset_runner.py
 M nvidia-sim/rl/gpu_dataset_sim.py
 M nvidia-sim/rl/gpu_probe.py
 M nvidia-sim/rl/gpu_probe_worker.py
 M nvidia-sim/rl/test_gpu_dataset_runner.py
 M nvidia-sim/rl/test_gpu_probe.py
?? nvidia-sim/rl/GPU_ELASTIC_STABILITY.md
?? nvidia-sim/rl/validation/gpu_elastic_stability.json
```

**계정 전환 시 보존:** 같은 워크스페이스를 사용하면 이 문서와 코드를 읽어 이어갈 수 있다.
코드와 요약 자료는 로컬 Git에 커밋했으며 push는 하지 않았다. 새 머신에서 원격 저장소만
clone하면 이 로컬 커밋이 포함되지 않을 수 있다. 또한 `nvidia-sim/rl/runs/`는 Git ignore
대상이므로, 다른 머신으로 옮길 경우 로컬 커밋과 필요한 실험 원본 폴더를 별도로 보존해야 한다.

## 4. 현재 실행 구조

| 파일 | 역할 |
|---|---|
| `nvidia-sim/run_gpu_candidate_dataset.sh` | 현재 GPU 데이터셋 실행 진입점 |
| `nvidia-sim/rl/gpu_dataset_runner.py` | CLI, config, resume 일치 검사, subprocess 관리 |
| `nvidia-sim/rl/gpu_dataset_sim.py` | 실제 장면, GPU 설정, 관측/검증, 후보 실행 스케줄 |
| `nvidia-sim/rl/dataset_scene.py` | 복제 환경/슬롯과 물리 실행 |
| `nvidia-sim/rl/dataset_motion.py` | 기존 후보 경로 계산 및 실행 |
| `nvidia-sim/rl/gpu_planning.py` | 실장면 모델 export 및 CPU 경로 사전 계산 서비스 |
| `nvidia-sim/rl/gpu_planning_worker.py` | Isaac을 띄우지 않는 CPU worker 진입점 |
| `nvidia-sim/rl/elastic_plant.py` | 탄성 식물 articulation, preload, 시각 모델 연결 |
| `nvidia-sim/rl/gpu_probe_worker.py` | 저장 명령을 재생하는 물리/정지/하중 분리 진단 |
| `nvidia-sim/rl/gpu_probe.py` | probe 실행/비교, 유효하지 않은 speedup 제외 |
| `nvidia-sim/rl/gpu_timestep_benchmark.py` | 1/4환경의 물리 Hz 비교 |
| `nvidia-sim/rl/gpu_planning_benchmark.py` | 동일 모델의 순차/병렬 CPU 경로 비교 |

현재 GPU dataset은 GPU dynamics + GPU broadphase + CPU tensor readback/native collider reports를 사용한다.
**완전히 CUDA tensor로만 처리하는 파이프라인이 아니다.** 접촉 object 식별을 위해 readback을 유지한다.
현재 GPU 설정은 PGS, position iteration 64, velocity iteration 4,
`gpu_max_num_partitions=1`, GPU CCD off다. 이전 CPU/TGS와 동일하다고 보증하지 않는다.
probe의 기본 solver/partition은 dataset과 다를 수 있으므로 재현 명령에서는 명시한다.

CPU planning은 실제 USD에서 추출한 FCL 도형 44개/자기충돌 검사 쌍 228개와 FK/IK 정보를 전달해
동일한 `dataset_motion.plan()`을 호출한다. IK·joint limit·자기충돌 검사를 생략하지 않는다.
최대 8개 worker로 다음 후보를 미리 준비하고 물리와 겹쳐 실행한다.
`--planning-workers 0`은 원래 순차 방식이다.

`DATASET READY`는 경로 계산 완료, `DATASET PHYSICS`는 물리 실행이다.
예전 `DATASET START`가 순차로 느리게 증가하던 큰 원인은 경로 계산이었다.
`planning_s`는 메인 프로세스의 대기 시간이며, worker 계산 시간의 합과 다르다.

타겟/성공 판정은 기존 object identity와 접촉 판정을 유지한다. probe의 `identity()`에서는
목표 distal `PedicelCollider` 및 proximal 마지막 collider, ring segment 08~23을 기록한다.
이 문서만 보고 성공 조건을 새로 추정하지 말고 `dataset_motion.py` 및 실제 target prim 정보를 확인한다.

## 5. 커밋된 성능 개선 결과

상세: [GPU_TIMESTEP_AND_PLANNING.md](nvidia-sim/rl/GPU_TIMESTEP_AND_PLANNING.md),
[검증 JSON](nvidia-sim/rl/validation/gpu_timestep_planning.json).

64개의 서로 다른 전체 후보 경로를 계산한 결과:

| 방식 | 경로 준비 시간 | 순차 대비 |
|---|---:|---:|
| 순차 | 86.14초 | 1.00배 |
| 4 workers | 23.86초 | 3.61배 |
| 8 workers | 13.57초 | 6.35배 |

명령 배열, waypoint, pose와 preflight 결과가 정확히 같았다. **전체 물리 실험이 6.35배 빨라진다는 뜻은 아니다.**
4환경/8후보/8제어 스텝의 통합 비교도 명령·trace·reset이 같았으며 경로 대기는 12.61→3.24초였다.
이 짧은 검사에는 접촉이 없었고 결과는 모두 `incomplete`다.

원래 armature 1e-5로 Hz만 낮추면:

- 60/120/240Hz는 접촉 전에도 식물이 밀려 조기 종료했다.
- 480/720Hz는 같은 접촉 구간을 진행했지만 960Hz 대비 1mm 회귀 기준을 초과했다.
- 높은 물리 스텝 수의 비용은 여전히 큰 병목이다. CPU 사용률만으로 GPU 물리 비용을 판단하지 않는다.

## 6. 후속 저주파 안정화 작업과 결과

추가한 코드:

- dataset CLI `--elastic-joint-armature` (기존값 1e-5, 유효 범위 0~0.01, 유한값 검사).
- 실제 PhysX DOF armature 적용 확인, config/backend/후보 metadata 저장, 설정이 다른 resume 거부.
- probe의 `--no-gravity`, `--no-preload`, `--preload-mode`, `--joint-armature`, `--pulse-force`.
- 정지 중 식물/과실 이동 및 joint 속도 기록. probe 예외 시 비정상 종료 코드 반환.
- 비교 시 다른 armature/preload 모델 또는 실패한 native API 검사는 유효한 speedup으로 인정하지 않음.
- production preload/reset은 원래 방식 그대로다. 실패한 drive/velocity 방식은 probe 안의 진단용 monkeypatch만 남겼다.

기존 모델은 articulation에 가는 줄기와 과실 연결 anchor를 구성하고,
초기 Jacobian으로 중력 보상 preload를 계산하여 고정 effort로 넣는다.
이번 진단에서는 geometry, 분할 수, 질량, 강성/감쇠, 마찰, 파손 임계값을 바꾸지 않았다.
단, **추가 armature 자체는 동역학을 바꾸는 추가 관성**이다.

240Hz에서 로봇 정지 2초 시험:

- 원래 모델의 최대 식물 바디 이동 약 58.5mm.
- 중력+preload 제거 시 약 0.072mm. 충돌만 제거 시 약 32.4mm.
- preload를 position/velocity drive로 옮기면 불안정/파손. 채택하지 않았다.
- armature 1e-4: 약 0.43mm. armature 1e-3: 약 0.10mm.
- 60Hz / armature 1e-3: 약 0.50mm. 정지 결과만으로 접촉 정확도를 보증하지 않는다.

저장된 `candidate_00003` 명령의 접촉 재생 시간(초기화 제외):

| Hz / armature | 1환경 wall | 4환경 wall | 실행 sim 시간 |
|---|---:|---:|---:|
| 960 / 1e-5 기존 | 156.25초 | 168.69초 | 4.333초 |
| 480 / 1e-4 | 79.54초 | 88.31초 | 4.350초 |
| 240 / 1e-4 | 40.83초 | 47.90초 | 4.383초 |
| 60 / 1e-3 | 11.74초 | 17.32초 | 4.350초 |

모두 비목표 잎을 접촉한 뒤 `excessive_displacement`로 종료했다.
성공 고리걸기/전체 60초 모션 처리량 검증이 아니다. 4환경은 같은 명령의 복제다.
물리 모델과 종료 시점이 다르므로 위 비율을 동등한 물리의 가속률로 발표하지 않는다.

0.2N에 해당하는 초기 Jacobian 고정 토크를 0.5~1.5초 가한 뒤 제거한 시험:
최대 목표 변위는 기존 약 37.64mm, 240Hz/1e-4 약 21.39mm, 60Hz/1e-3 약 16.56mm였다.
복원은 일어나지만 원래 변형 응답과 다르다. 변형 중인 지점에 일정 world force를 주는 시험은 아니다.

같은 armature끼리 시간 간격을 비교해도:

- 1e-4, 960 vs 240Hz: 고리 간격 지표 최대 차이 3.83mm, 종료 3제어 스텝 차이.
- 1e-4, 960 vs 480Hz: 고리 간격 지표 최대 차이 1.96mm, 종료 1제어 스텝 차이.
- 1e-3, 960Hz: **첫 제어 스텝 0.0167초에 로봇 접촉 없이 비목표 `HarvestJoint_05_4` 파손**.
  native API 종합 검사도 false. 이것은 안정한 비교 기준이 아니다.

회귀 허용치는 위치 지표 1mm / 로봇 joint 0.005rad / 종료 1제어 스텝으로 두었다.
이 기준은 실물 정확도나 tunneling 부재를 보증하는 값이 아니다.
960Hz도 실물 정답으로 검증된 모델은 아니다.

## 7. 실험 자료 위치와 검증 범위

모두 저장소 기준 `nvidia-sim/rl/` 아래다.

| 위치 | 내용 |
|---|---|
| `validation/gpu_timestep_planning.json` | 커밋된 CPU 병렬화/Hz 비교 요약 |
| `validation/gpu_elastic_stability.json` | 후속 안정성·접촉·복원·통합 시험 요약 |
| `runs/20260921_143244_gpu_timestep/` | 원래 모델 1/4환경 × 60~960Hz 시험 |
| `runs/20260921_144300_planning_validation/` | CPU 64경로 benchmark 및 순차/병렬 통합 비교 |
| `runs/20260921_150500_elastic_diagnosis/` | 중력/preload/contact/drive/armature 분리 진단 |
| `runs/20260921_151000_elastic_validation/` | 접촉 재생, pulse, 같은 관성의 Hz 비교, 데이터셋 통합 |
| `runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003/` | 저장된 재생 fixture |

fixture 명령 SHA256:
`cd300bcd280159339580dcba375ed759459f458eb8e84d03da730701aa5d2ad2`.

마지막 validation 폴더의 주요 하위 폴더:

- `motion_240_{1,4}`, `motion_60_{1,4}`, `motion_480_{1,4}`: 접촉 재생.
- `motion_240_stronger_{1,4}`: 240Hz / armature 1e-3 추가 진단.
- `reference_240_model_960`, `reference_60_model_960`: 각각 같은 armature의 960Hz 기준 시도.
- `pulse_960`, `pulse_240`, `pulse_60`, `pulse_240_stronger`: 하중/복원 시험.
- `*_harness_error`: 첫 pulse 구현 오류 때문에 실패한 자료. 유효 결과에서 제외했다.
  실제 joint effort 접근을 수정한 뒤 위 pulse 폴더로 재실행했다.
- `dataset_240`, `dataset_60`: 각 4환경/8개의 서로 다른 후보/16제어 스텝 통합 검사.
  초기 상태·독립 reset·camera·armature·planning 연동 통과. **각 8개 `incomplete`, dataset_complete=false**.
- `run_*.py`, `summarize.py`: 당시 실험 orchestration/요약 스크립트. `runs/` 내부라 Git 미추적이다.
  과거 폴더 완료 조건을 기다리는 스크립트도 있으므로 그대로 재실행하지 말고 명령을 읽어 새 폴더에 실행한다.

기록된 마지막 자동 검증은 관련 테스트 **55개 통과** 뒤 비교 도구에 API 실패 제외 테스트를
추가하고 해당 파일 **6개 통과**를 확인한 상태다. 새 테스트를 포함한 전체 묶음을 한 번에
다시 실행한 것은 아니다. `git diff --check`도 통과했다.

## 8. 재개 및 재현 명령

먼저 상태와 문서를 확인한다. 이 문서 작성 요청에서는 추가 시뮬레이션을 돌리지 않았다.

```bash
cd /root/farmily_tomato
git status --short
git log -4 --oneline
```

현재 기본 물리로 작은 dataset 실행(전체 후보 동작이므로 짧게 끝난다고 보장하지 않음):

```bash
./nvidia-sim/run_gpu_candidate_dataset.sh \
  --num-envs 4 --candidates 8 --schedule continuous \
  --physics-hz 960 --elastic-joint-armature 0.00001 --planning-workers 8
```

짧은 연결 검사만 하려면 `--max-control-steps 16`을 추가한다.
이때 중단된 후보는 `incomplete`이며 성공/실패 학습 데이터로 쓰지 않는다.
GUI 1환경 확인은 위 명령을 `--num-envs 1 --candidates 1 --gui`로 바꾸고 `DISPLAY=:0`을 사용한다.
새 계정에서 재현하기 전에 `--help`로 현재 옵션을 확인한다.

동일 fixture의 240Hz / armature 1e-4 진단 재현(**동등성 검증에 실패한 실험 설정**):

```bash
task_probe_dir="nvidia-sim/rl/runs/$(date +%Y%m%d_%H%M%S)_elastic_240_probe"
/root/isaaclab_env/bin/python nvidia-sim/rl/gpu_probe_worker.py \
  --headless --mode gpu --solver pgs --batched-io --native-replication \
  --gpu-partitions 1 --num-envs 1 --physics-hz 240 --joint-armature 0.0001 \
  --fixture nvidia-sim/rl/runs/20260920_160125_tomato_05_candidate_dataset/results/candidate_00003 \
  --output "$task_probe_dir"
```

같은 설정으로 4환경을 검사하려면 `--num-envs 4`와 새로운 output 폴더를 사용한다.
동일 GPU에서 성능 시험을 동시에 실행하지 않는다. 초기화 시간과 motion wall time을 분리해 비교한다.

관련 테스트 재실행:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=nvidia-sim/rl \
/root/isaaclab_env/bin/python -m pytest -q \
  nvidia-sim/rl/test_gpu_planning.py \
  nvidia-sim/rl/test_gpu_step_profile.py \
  nvidia-sim/rl/test_gpu_throughput_benchmark.py \
  nvidia-sim/rl/test_gpu_replication.py \
  nvidia-sim/rl/test_gpu_scale_validation.py \
  nvidia-sim/rl/test_gpu_prim_lookup.py \
  nvidia-sim/rl/test_gpu_initialization.py \
  nvidia-sim/rl/test_gpu_batch_views.py \
  nvidia-sim/rl/test_gpu_dataset_runner.py \
  nvidia-sim/rl/test_gpu_validation.py \
  nvidia-sim/rl/test_gpu_probe.py \
  nvidia-sim/rl/test_dataset_pool.py
```

## 9. 다음에 검토할 작업 (미실시)

1. 후속 커밋 diff와 실험 기록을 검토하고 보존한다. 원래 preload/reset을 실험 실패 버전으로 되돌리지 않는다.
2. 보조 관성 1e-3/960Hz에서 접촉 전 파손되는 이유를 파손 연결부의 하중/초기화부터 분리 진단한다.
   원인이 엔진 버그라고 확정되지는 않았다.
3. 주줄기·꼭지·과실의 결합 및 하중 전달을 대상으로 작은 정지/토크/접촉 시험을 만든다.
   주파수를 바꿔도 변형·파손 응답이 수렴하는지 검사한다.
4. 실측 식물 변형 자료가 확보되면 모델의 응답을 보정한다. armature를 키우는 것만으로 sim-to-real을 보증하지 않는다.
5. 유효한 저주파 설정을 확보한 뒤 서로 다른 후보, 성공 trajectory, 타겟 1~11로 검증을 넓힌다.
6. 그 다음 대규모 환경 수에서 초기화/경로/physics/read-write/판정 시간을 다시 측정한다.

사용자가 요구한 충돌/탄성/파손 실패 사례를 없애거나 파손 임계값을 올려서 속도 시험을 통과시키지 않는다.
물리 정확도를 바꾸는 설정과 결과를 유지하는 코드 최적화는 기록에서 구분한다.

## 10. 이전 작업 문서 찾아보기

- [전체 시뮬레이터 README](nvidia-sim/README.md)
- [GPU 데이터셋](nvidia-sim/rl/GPU_CANDIDATE_DATASET.md)
- [GPU throughput](nvidia-sim/rl/GPU_THROUGHPUT.md)
- [GPU 병목 조사](nvidia-sim/rl/GPU_BOTTLENECK.md)
- [GPU 최적화](nvidia-sim/rl/GPU_OPTIMIZATION.md)
- [GPU probe](nvidia-sim/rl/GPU_PROBE.md)
- [후보 데이터셋/관측](nvidia-sim/rl/CANDIDATE_DATASET.md)
- [이전 CPU 다중 프로세스 pool](nvidia-sim/rl/CANDIDATE_DATASET_POOL.md)
- [탄성 식물](nvidia-sim/rl/ELASTIC.md)
- [고리걸기](nvidia-sim/rl/HOOKING.md)
- [pose 탐색](nvidia-sim/rl/POSE_SEARCH.md)
- [RGB 관찰/밀림 감지](nvidia-sim/rl/RGB_GUARD.md)

옛 문서의 CPU/TGS/worker pool 설정을 현재 GPU runner의 설정으로 착각하지 않는다.
충돌하는 수치는 최신 코드, 이번 검증 JSON, 마지막 두 성능/안정성 문서를 우선 확인한다.

## 새 대화에 붙일 문장

> `/root/farmily_tomato/HANDOFF.md`를 읽고 현재 Git diff와 연결된 최신 검증 자료를 확인한 뒤 이어서 작업해줘.
> 성능 개선 기준 커밋은 `1305651`이고, 저주파 안정화 진단과 이 문서는 후속 커밋에 포함했어.
> GPU 멀티환경과 CPU 경로 병렬화는 구현되어 있지만, 물리 결과를 유지하는 저주파 설정은 아직 검증하지 못했어.
> 그 이후 실용적인 물리 동작을 우선하는 기준으로 바뀌었고, 최신 기본값은 practical60이야.
> 이 문서 맨 위의 최신 후속 작업과 GPU_PRACTICAL_PHYSICS.md를 먼저 확인해줘.

## 2026-09-22 로봇 포함 MuJoCo CPU 기준선

- 현재 사용자 작업 브랜치 `mujoco-sim`에서 로봇 USD 관절/질량/관성/충돌 형상을 MuJoCo에 추가.
- `mujoco-benchmark/scripts/build_robot_model.py`, `robot_engine.py`, `robot_viewer.py`, `benchmark_robot_pool.py`.
- 기준 모델은 기존 최적화 식물, 로봇 7축, 전체 241 DOF. 고리는 동적 로봇 팔 끝에 고정. 하우스 없음.
- CPU 1/2/4/8/16/24/32/48/64 프로세스, 동일 49번 16.5초 기록을 각각 3회 실행.
- 48개 평균 6.443초/배치, 처리량 122.97 sim-s/s. 64개는 비슷한 처리량. 최대 검증64, 권장48.
- 모든 반복 발산0. 전체 영상/기하 검사 `outputs/20260922_robot_validation_final`, 순기구학 위치 오차 3.3e-8m, 9테스트 통과.
- 원본 시간/자원/코드 해시는 `outputs/20260922_robot_cpu_scaling{,64}`; 커밋용 요약은 `validation/robot_cpu_scaling.json`.
- 사용자는 CPU 기준선 커밋 후 새로운 브랜치에서 mjlab/MJWarp를 구축해 같은 조건의 GPU 성능 비교를 요청했다.

## mjlab GPU 비교 완료

- CPU 기준선 `38bac4d` 이후 `mjlab-performance` 브랜치 생성. 별도 `.mjlab-venv` 설치, 기존 `.venv` 보존.
- mjlab 1.6.0 / MuJoCo·MJWarp 3.11 / Warp 1.17 / Torch 2.7 cu128. `setup_mjlab.sh`, lock 파일 포함.
- 동일 모델 1/8/32/128/256/512/1024환경, 각 16.5초 × 3회. CUDA 제어·물리·평가 시간과 자원 시계열 저장.
- 가장 효율적인 GPU 설정 256: 평균 32.030초, 131.88 sim-s/s. 1024는 평균 179.960초, 93.89 sim-s/s.
- CPU 3.13/48프로세스 122.97 sim-s/s; 버전 맞춘 CPU3.11/24프로세스 75.62 sim-s/s.
- 모든 반복 비정상/overflow 0. GPU world0와 CPU 전체 스텝의 위치 오차는 validation/mjlab_robot_equivalence.json.
- 상세/한계/명령: mujoco-benchmark/MJLAB_PERFORMANCE.md. 비교 화면은 outputs/20260922_robot_backend_comparison/index.html.
- 파단은 비활성화, 동일49번 반복 성능 시험. 다양한 후보 탐색이나 RL 학습/수확 성공 평가기는 아님. mjlab Simulation 계층을 사용했으며 ManagerBasedRlEnv는 아직 추가하지 않음.

## 2026-09-22 CPU/GPU 운영 권장 정리

- GPU 구성 및 비교 구현 커밋: `115560b` (`mjlab-performance`). CPU 기준선: `38bac4d`.
- 대량 탐색 후속 구현 기본값은 GPU 256환경 권장. 기존 CPU 3.13/48 대비 처리량 약 7% 우세이며, 소수 경로 디버깅은 CPU 권장.
- 아직 동일 경로 재생 벤치마크이므로 다양한 후보 생성/저장까지 포함한 완료 시간은 미검증.
- autoreset 호환 변경은 별도 모델의 disable → enable 변경이며, MJWarp의 미지원 기능을 구현한 것이 아님. 원본과 오류 처리 의미가 같다고 설명하지 않는다.
- 상세 근거와 다음 단계: [MJLAB_PERFORMANCE.md](mujoco-benchmark/MJLAB_PERFORMANCE.md)의 운영 권장 절.

## MuJoCo 다양한 경로 생성 연결

- `mujoco-benchmark/scripts/candidate_experiment.py`: 기존 Isaac Sobol staged6d/IK/FCL 계획 → MuJoCo CPU 작업자별 서로 다른 경로 실행.
- 매 스텝 상태 저장, HTML/CSV, `replay_candidate.py`로 원본 상태 재생.
- 단순 벤치마크와 별도 실행기. 상세 명령/판정 한계: [CANDIDATE_SEARCH.md](mujoco-benchmark/CANDIDATE_SEARCH.md).
- 현재 부분 중심 진입/과도 변위/물리 이상 진단이며 꼭지 걸림 성공 평가기는 아직 아님.

- 경로 탐색 기본 출력 위치 변경: `/root/docker_share/mujoko_debugging_data/YYYYMMDD_HHMMSS_candidate_search/`. `candidate_experiment.py --output`으로 개별 지정 가능.

## D435 RGB-D 촬영

- `mujoco-benchmark/scripts/d435_capture.py`: 기존 ROS/Isaac URDF 광학 프레임 체인과69.4° FOV 사용, 저장된 후보 상태에서RGB/native depth/color-aligned depth와 행렬 저장.
- 사용법과 실물 센서 모사 한계: `mujoco-benchmark/D435_RGBD.md`.
- 후보87 preapproach/entry 샘플은 `/root/docker_share/mujoko_debugging_data/20260922_111838_candidate_search/candidates/candidate_00087/rgbd/`.

## Tomato_05 시점 변경 관측 준비 완료

- `prepare_observations.py`로 초기 상태 고정, 가상 D435 이동·회전, 자동 타깃 중앙정렬 없이71개RGB-D/GT목표표시/3배·6배크롭 촬영.
- 최종 데이터: `/root/docker_share/mujoko_debugging_data/20260922_122302_tomato05_observations/` (`index.html` 갤러리). 약192MiB.
- 기존 `20260922_111838_candidate_search`의1,000개경로/결과와 연결, per-view camera좌표라벨 제공. 당시 관측 생성 단계에는 새 물리시험0회. 이후 재실험·학습은 아래 기록 참조.
- `validate_observations.py`의71,000관측-후보pair좌표검증 및RGB/Depth/mask/crop검증통과. 대상gt마스크ID보간을피하기위해촬영MSAA비활성.
- 가상카메라 변환은 실제관절만으로복구불가. `camera.json`의유효tool_from_optical/world_from_optical 사용.
- 실제로봇도달성/새토마토일반화는검증하지않음. 상세: `mujoco-benchmark/OBSERVATION_DATASET.md`.

## 카메라 기준 action v2 및 1,000개 재실험 (2026-09-22)

- `camera_action.py`: 카메라 광학축 기준 진입 방향/고리 quaternion/거리/상승 방향의14차원 입력. 절대 시작 위치는 제외, 정확한 재연용 토마토 중심 상대 경유점은 별도 보존. 중력 방향도 관측별 저장.
- `candidate_experiment.py` → `action_frame.json`, `plan.json`/`result.json:action_camera`. 기존6개변수/월드경로 보존. 보고서 방위각은 명시적으로 월드 기준이라고 표시.
- 새 실험 `/root/docker_share/mujoko_debugging_data/20260922_124028_camera_action_v2_1000/`: CPU48/계획16,120Hz,seed0,1,000개 완료, 총510.83초. 부분중심진입36/과도변위908/miss56. 중심진입 자체는200개(과도변위 포함). 오류0. 이전111838실험과모든분류/최대변위동일.
- 기존71개관측은 `upgrade_observation_actions.py --run NEW_RUN`으로 새결과에연결. `actions_camera_v2.npz:features`를학습에사용하고 `candidate_ids`로`actions.json`과join. 원래초기카메라의단일action_camera를다른관측에그대로사용하면안됨.
- 사진과물리장면은보존. 모델해시/초기qpos동일확인. 71,000쌍좌표검증최대위치오차3.33e-16m. 카메라병진에대한불변성/회전에대한월드복원검증통과. v1원본메타는schema_v1_backup보존.
- 이 저장 형식 변경 이후 학습 완료(아래 참조). 꼭지걸림성공평가기는미구현. 동일Tomato05의시점변경만있으므로영상없는기준모델과비교필요.

## Tomato_05 카메라 기준 후보 학습 예비 비교 (2026-09-22)

- `mujoco-benchmark/learning/train.py`: 고정 DINOv2/ResNet18의 local/context RGB 특징 + metric Depth/validity/K + 카메라 action14/중력3 → 부분 중심 진입 확률/최대 이동량. 영상 없는 대조군 포함 3종×3seed 실제 GPU 학습, 41.21초.
- 결과 HTML/가중치/CSV/분할/코드 사본: `/root/docker_share/mujoko_debugging_data/20260922_130308_camera_action_training/`.
- 후보 train699/val149/test152와 관측 train50/val7/test7/잘림검사7을 별도 분리. 새 시점+새 후보가 주 평가. 모델/정규화/체크포인트 선택은 train/val만 사용.
- 테스트 AP: 영상 없음.837 / ResNet.849 / DINO.855. 이동 MAE 2.575/2.535/2.618mm. 검증 손실은 영상 없는 모델이 최소라 공식 선택도 영상 없음. 영상 모델의 확실한 우위 또는 sim-to-real 성공을 주장하지 않음.
- `predict.py`로 저장 관측의 후보 순위 재계산 가능. 세 모델 checkpoint 재로드/분할 비중복 확인 완료. 새 실물 이미지 배포는 미구현. 추가 physics 검증은 다음 절 참조.
- 모델 조사 근거와 명령: `mujoco-benchmark/learning/README.md`. DINOv3는 공식 가중치 접근 신청 필요해 미학습. 단일 장면 반복 관측의 한계/영상 없는 기준과의 비교를 계속 유지할 것.

## 학습 추천의 실제 물리 검증

- `learning/test_physics.py` / `render_physics.py` 추가. 평가용7시점×3모델(seed0)의1순위추천21회를독립reset후MuJoCo120Hz새실행. 중복제외3경로.
- `/root/docker_share/mujoko_debugging_data/20260922_model_recommendation_physics/index.html` 결과/상태/대표3개2배속영상. 물리41.52초(렌더제외).
- ResNet/DINO각진입7/7·20mm이하7/7. 영상없는모델진입7/7·20mm이하4/7. 초기상태동일/원본결과일치/불안정0확인.
- 기존카탈로그경로를추천해관절명령재실행. 새trajectory생성·새장면일반화·꼭지걸림성공실험은아님.

## Tomato01~11 다중 대상 수집 (05 제외)

- `candidate_experiment.py --target Tomato_XX`와 `RobotEngine(target=...)`로계획기하/과실진입판정/변위측정대상을선택. 기본05보존.
- `prepare_observations.py`는manifest대상으로GT과실마스크/라벨/action좌표자동연결. 가상카메라기준위치는05대비대상중심차만큼이동하고원래장착카메라pose와이동량을명시. 로봇실제관측자세검증은아님.
- `collect_tomatoes.py --targets 1,2,3,4,6,7,8,9,10,11 --candidates 1000 --workers 48 --planning-workers 16`으로대상순차물리시험→RGBD저장. 전체10,000개본수집은사용자가실행.
- 명령/출력/공간/카메라한계: `mujoco-benchmark/MULTI_TOMATO_COLLECTION.md`.
- 다중대상시상태재연카메라/HTML대상표시도연결. 기존데이터변경없음.

## 후처리 최적화와 재개 기능 (최신 추가)

- `camera_action.py`: 고정 기하 일괄 준비 / 카메라 변환·검증 벡터화. 관측별 압축·검증은 기본8 CPU 스레드. `prepare_observations.py`의 v1 중복 검증 제거, 최종v2 모든검사유지.
- 71,000쌍 후처리 131.81초→1.41초. 71장 촬영 포함65.61초(장면 초기화 제외). 기존RGB/Depth/mask각71파일완전일치, action오차1e-12이내. 물리 약8분은변경없음.
- `collect_tomatoes.py --resume --output 기존폴더` + `resume_candidates.py` 추가. 개별결과/상태/명령해시검사후누락만실행. 기존계획/완성관측재사용, Ctrl+C시단계프로세스그룹정리.
- 실제중단수집 `20260922_134834_multi_tomato_collection`: 01~04완료,06은정상931개/남음69개검사확인. 사용자대신본수집을다시시작하지않음. 재개명령은 MULTI_TOMATO_COLLECTION.md 최신절.
- 성능검증복사본 `20260922_144727_postprocess_benchmark/` 보존. 중단복구/원래물리결과일치/손상데이터검출확인.


## 2026-09-22 진입 탐색 범위 변경

새 staged6d 후보는 진입각 −45°~+45°, 하부 여유 0~10mm를 Sobol 샘플링한다. MuJoCo와 Isaac의 공통 생성기 `nvidia-sim/rl/trajectory_search.py`에 적용했다. 여유는 초기 열매 충돌 구의 아랫면과 고리 와이어 윗면 기준이다. 0mm는 기하학상 여유가 없는 조건이며 실제 접촉은 물리 엔진의 접촉 설정에도 영향을 받는다. Sobol은 끝값 조합을 반드시 포함하지 않는다. 기존 데이터/저장된 후보/재생은 변경하지 않았고, resume은 저장된 후보를 계속 사용한다. 같은 seed와 candidate_id라도 새 실행은 이전 범위와 다른 경로이므로 실행 폴더를 구분한다. 실제 새 범위 물리 실험은 아직 수행하지 않았다.

## 2026-09-22 Tomato_06 자유 동작 RL 파일럿

- `mujoco-benchmark/scripts/train_tomato_rl.py`: mjlab/MJWarp + RSL-RL PPO, 고정 Tomato_06 GT 장면에서 팔6관절+리프트7액션 학습. 회전 고정/직선 trajectory 제약 없음.
- 상세 실행·판정·제약: [TOMATO06_RL.md](mujoco-benchmark/TOMATO06_RL.md). 기존 데이터 수집/모델은 유지하고 RL 전용 모델에 자기충돌을 추가했다.
- 학습 완료 후 결정적 독립 평가 상태를 저장하며 `replay_tomato_rl.py`로 재생 가능. 짧은 검증은 학습 루프/물리 안정성 확인이며 수확 성공을 주장하지 않는다.

- 최종 16환경/20업데이트 검증: 학습28.88초, 준비·평가 포함45.93초(커널 캐시 사용). 10,240 전이, 학습32에피소드 및 독립평가 모두 부분 진입 성공0. 성공 정책이 아니라 실행 루프 확인 완료. 가중치 업데이트/reset/체크포인트 로드/재생 검증 통과. `mujoco-benchmark/validation/tomato06_rl_smoke.json`에 결과 보존.

## 2026-09-22 GPU 환경 확대 및 추가 학습 실행

기존 사용자 실행 `20260922_184128_tomato06_rl`의 200회 체크포인트에서 추가 학습을 시작했다. 짧은 동일 조건 비교(3초 에피소드, 32스텝, 4업데이트): 64환경 1,150전이/초, 256환경 2,352전이/초. 256환경에서 관측한 GPU 사용률 95%, 메모리4,414MiB(평균/최댓값 아님). 기존16환경 357전이/초는 30초 에피소드의 장기 측정이라 엄밀히 동일 조건 비교는 아니다.

- 실행 중: `/root/docker_share/mujoko_debugging_data/20260922_185257_tomato06_rl_gpu256`
- PID 286916 (종료/상태 확인 전 프로세스 명령을 다시 확인할 것)
- 환경256, 추가1,000업데이트, 32스텝, 에피소드30초. 총8,192,000전이. 기존 물리/보상 유지.
- 로그: 같은 경로 뒤 `.log`; 실행 인자/PID는 `.launcher.json`. 25업데이트마다 체크포인트, 완료 후 summary/평가 재생 데이터 저장.
- progress.jsonl에 처리량·누적 학습시간·잔여 예상분 추가. 약1시간 예상이나 접촉 복잡도에 따라 달라짐. 이 기록 작성 시 본 학습은 진행 중이며 성공 여부 미확인.
- 측정 보존: `mujoco-benchmark/validation/tomato06_rl_scaling.json`.

## 10분 학습 비교로 전환

사용자 요청으로 기존 1,000회 추가학습(PID286916)은 중단. 해당 실행의 model_00025.pt를 보존했으며 이후 미저장 업데이트는 보존되지 않았다. 비교의 출발점은 원래 사용자 200회 체크포인트다.

`--train-seconds 600 --compare-before --iterations 100000`으로 학습 구간만 600초 제한, 진행 중인 업데이트 종료까지 몇 초 초과 가능. 준비/기준평가/최종평가는 별도. 실제 완료 업데이트와 전이 수 기록, 마지막 체크포인트 저장. 2초 제한 검증에서 2.76초/2업데이트 후 자동 종료·저장·전후 비교 통과.

현재 실행: /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min

PID 291847; 로그: /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min.log

완료 후 summary.json의 comparison에 전후 성공수, 평균 최근접 거리, 평균 최대 열매 이동량 저장. baseline_world0.npz와 evaluation_world0.npz 저장. 실제 향상 여부는 완료 후 확인해야 한다.

## 10분 추가 학습 완료 및 진단

`20260922_185721_tomato06_rl_10min` 완료: 학습600.43초, 전체735.75초, 170업데이트/1,392,640전이. 학습2,306에피소드 성공0, 결정적 평가 전후 성공0. 평균 최근접 중심 거리56.16→43.17mm, 평균 최대 열매 이동량1.13→1.13mm. 결과 요약은 `mujoco-benchmark/validation/tomato06_rl_10min.json`에 보존.

평가 궤적은 약5초에 접근한 뒤30초까지 비슷한 위치에 머문다. 현재 근접 항은 정지 상태에서도 시간 비용 차감 후 약+0.024/제어스텝(다른 감점 전)을 지급한다. 성공 경험이 없는 정책에서 머무르기 보상과 밀림 실패 페널티가 정체를 유도했을 가능성이 크다. 성공 조건은 고리 평면2mm 이내/0.2초 유지이며 좁다. 다음 작업 후보는 정지 근접 보상 제거와 실제 정렬/삽입 진전 보상 설계, 물리적 진입 가능성 점검이다. **보상 수정은 아직 하지 않았다.**

이전 문단의 '실행 중'은 과거 기록이다. 1,000회 실행은 사용자 요청으로 중단했고 10분 비교 실행은 정상 완료했다. 학습 모델/대형 상태 기록은 docker_share에 유지하며 Git에는 코드·문서·작은 검증 요약만 포함한다.

## 최신 변경: 보상 progress_v2 (학습 실행 안 함)

사용자 요청으로 보상만 변경. 정지 근접 보상과 반복 내부 체류 보상을 제거했다. 기존 목표점 거리 감소20배 + 고리 좌표계의 포획 조건 위반 거리 감소20배로 진전을 보상한다. 포획 오차는 반원 내부/고리 평면/와이어 여유의 위반량이며 특정 세계 방향이나 경로를 고정하지 않는다. 최초 안전 부분 진입은 에피소드당1회 +2, 기존0.2초 유지 성공 +10. 기존 밀림/접촉/급격한 액션/시간 감점과 실패 -5 유지. 후퇴는 signed progress 감점이지만 물리적으로 금지하지 않는다.

물리·관측·행동·종료·성공 기준·학습기는 변경하지 않았다. manifest의 reward_version=progress_v2로 구분한다. 정지/진전/후퇴/왕복/최초 진입/포획 오차 산술 검증 통과. 수정 보상의 실제 학습 개선 여부는 아직 미검증. 학습은 사용자가 직접 실행한다.

2시간 추가 학습(이전10분 모델에서 시작):

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.mjlab-venv/bin/python -u \
  mujoco-benchmark/scripts/train_tomato_rl.py \
  --num-envs 256 --iterations 100000 \
  --steps-per-env 32 --episode-seconds 30 \
  --train-seconds 7200 --compare-before \
  --checkpoint /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min/model_00170.pt
```

학습 구간2시간이며 업데이트 경계까지 수 초 초과 가능, 준비·전후 평가는 별도다. 새 datetime 폴더로 저장. 기존 보상과 점수 크기가 달라졌으므로 보상 합보다는 독립 평가 성공/거리/변위를 비교한다.

## 최신 데이터 생성 조건: 각도만 탐색

새 staged6d/Sobol 후보는 1차원 scrambled Sobol로 진입 방위각 −45°~+45°만 바꾼다. GT 중심에 인식 오차 없음. 좌우 오프셋0mm, 하부 여유2mm 고정. 삽입42.5mm/수직 상승32.5mm는 이전 범위 중간값으로 고정, pre-hook170mm/roll0/elevation0 유지. 이 조건은 중심선을 향하는 계획이며 접촉/변형/제어 오차까지 제거하거나 실제 진입 성공100%를 보장하지 않는다. 삽입 깊이도 모든 후보에서 동일하고 열매 중심 도달을 자동 보장하지 않는다.

공통 trajectory_search 생성기와 새 실행 metadata에 sampling=scrambled_sobol_azimuth_only 기록. 파일/CLI trajectory_mode=staged6d 명칭은 호환성을 위해 유지. 기존 저장 후보/재생/계속 실행(resume)은 이전 경로를 유지하므로 새 조건 수집은 새 실행 폴더로 시작해야 한다. 이전 seed/candidate 번호와 새 경로는 다르다. RL 제어에는 적용하지 않는다.

생성/기하 검사5개 통과(1,000후보에서 각도만 다름, 좌우 중심 정렬, 와이어 여유2mm 확인). 물리 실행/데이터 수집은 실행하지 않았다.

## 두 구간 45° 상승 적용

새 후보에 `lift_profile=diagonal_45_return`을 저장한다. 진입 방향 기준 전진16.25mm+상승16.25mm 후, 후퇴16.25mm+상승16.25mm. 최종 위치는 이전 수직32.5mm 상승과 동일하며 고리 회전은 바꾸지 않는다. 경유점명은 `rise_mid`, 마지막은 `rise`; 실제 제어 단계는 두 구간 모두 rise로 처리하여 동일 상승 속도/접촉 판정을 적용한다. 이동 거리는 기존 상승의 √2배이므로 동일 속도에서는 상승 시간이 길어진다.

기존 저장 후보에 profile이 없으면 기존 상승을 유지한다. 새 방식은 waypoint5개를 카메라 좌표에도 모두 저장한다. 기존14개 compact features의 lift 방향/거리는 최종 순변위이며, 이14개만으로 직선 상승과 대각 상승을 구분할 수 없다. 두 방식을 학습용으로 섞으려면 phase_names/전체 waypoint 또는 별도 profile 입력을 사용해야 한다. 서로 다른 phase 구조를 한 batch로 섞으면 명시적으로 거부한다.

기하 검사6개 통과, 기존4점/새5점의 카메라 좌표 변환·복원 검증 통과.

## 중심 진입 단독 성공 기준

새 MuJoCo 데이터 수집의 classification_rule=center_entry_only_v2: 물리 오류는 invalid_physics, 정상 실행에서 중심 진입 이력이 있으면 열매 최대 밀림과 무관하게 partial_center_entry, 없으면 miss. target_center_max_displacement_m과 target_displacement_exceeded(20mm 초과)는 참고 지표로 별도 저장. 결과 JSON/CSV와 HTML의 판정 설명/밀림 집계를 갱신했다.

기존 데이터는 자동 덮어쓰지 않았다. 과거 manifest에 rule이 없으면 displacement_first_v1로 이어 실행하여 혼합 판정을 방지한다. 기존 ±90°/30개 데이터를 읽기 전용으로 재분류 검증하면 중심 진입 성공3개, 미진입27개다. RL 보상/종료 조건은 이번 변경 대상이 아니다.

## 다중 토마토 RGB-D 진입각 학습 완료

데이터10대상×30경로/710관측(05없음). 8대상 학습,09검증,06테스트로 분리. 진입 BCE만 학습하며 밀림 감점 제거. DINOv2 seed2 선택,06의64시점 모두−81.09° 후보00001 추천. 고유 경로1개 새 물리 실행에서 중심진입 성공(밀림30.78mm). action_only도동일 top1성공으로 영상 우위 미입증. 전체 후보 확률 calibration 불량(Brier약0.315). 상세/명령: [learning/MULTI_TOMATO.md](mujoco-benchmark/learning/MULTI_TOMATO.md). 결과 `/root/docker_share/mujoko_debugging_data/20260922_234403_multi_tomato_training/index.html`. 초기 요청 커밋3f636c2 이후 학습기 확장 작업.
