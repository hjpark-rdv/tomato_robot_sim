# GLB 송이 물리 이식

2026-09-23: 탄성·충돌·로봇 수집 연결 완료. **초기 내부 겹침으로 정상 학습 수집은 미완료**.

기존 주줄기와 잎의 302개 geom 초기 위치/회전 보존 검사 통과. GLB Y축 0~180° 배치와 Y-up→Z-up 변환 유지. cyan Y90°, 주줄기 segment11로 검증했다. 초기 배치 제한과 물리 중 탄성 관절 회전은 별개다.

Peduncle5/Rachis14/각 열매 proximal3+distal1의 탄성 연결, 원본 대응 질량·강성·감쇠·armature를 이식했다. 로봇 포함 241자유도. 새 크기·형상으로 관성이 달라지므로 기존과 동일한 물리 응답은 아니다. 충돌은 가지 capsule/열매 convex hull이며 털과 꽃받침은 시각 형상이다. 원본 optimized처럼 파단/분리는 비활성이다. 시각 가지를 분절별로 나누어 휘어질 때 이음새가 보일 수 있다.

## 실제 검증

- 240Hz/6초, 0.2N을 2.5~3초에 가한 뒤 동일 무부하 궤적과 비교: 최대24.07mm, 종료 잔류1.56mm(93.5% 감소). 기존 모델 비교 최대16.35mm.
- 구형 물체 접촉 시험: 접촉556건, 무부하 대비 최대21.58mm 이동. NaN/물리 경고 없음, reset 정확 일치.
- 초기 cyan 내부 겹침 최대4.93mm. 무부하 초기2초 식물 바디 최대 이동51.69mm. 초기 안정성을 통과한 모델은 아니다.
- 나머지4 GLB도 컴파일 완료하였으나 초기 최대 겹침2.25~5.47mm. capsule/convex 근사와 원본 형상의 영향을 추가 분리해야 한다.
- 로봇 후보3개 실제 계획/전체 실행 완료: 모두 invalid_physics, training_eligible=false. 후보0의 중심 진입 기록을 성공으로 인정하지 않는다.
- 새 모델 RGB/Depth/GT mask 640×480 촬영, 목표1852픽셀. 배경 depth는 NaN이며 valid mask 사용. 가상 관측 시점의 로봇 도달성은 미검증.
- 기존 로봇/장면 회귀 테스트10개 통과.

## 실행

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/build_glb_physics.py --output /root/docker_share/mujoko_debugging_data/NEW_glb_physics
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/candidate_experiment.py \
  --model /root/docker_share/mujoko_debugging_data/NEW_glb_physics/model.mjb \
  --reference /root/docker_share/mujoko_debugging_data/NEW_glb_physics/reference.json \
  --candidates 3 --workers 1 --planning-workers 1 --hz 240 --target Tomato_05
```

출력은 새 폴더를 사용한다. `--glb`, `--y-deg`, `--segment`로 원본/각도/부착 위치 지정. 수집의 `--model`은 MJB 파일이다. 계획기는 기존 로봇 FK/자기충돌 자산을 사용하고 목표 기하는 새 reference에서 받는다.

결과: `/root/docker_share/mujoko_debugging_data/20260923_glb_physics_pilot/index.html`.
수집: `/root/docker_share/mujoko_debugging_data/20260923_glb_physics_collection_pilot/index.html`.
검증 요약: `validation/glb_physics_migration.json`.


## 2026-09-23 9·10번 분기부 충돌 수정（최신）

cyan의 9·10번 꼭지 가지 시작점이 Rachis 분할 경계에 걸쳐 있어, 물리 부모 이외의 바로 옆 capsule과 충돌했다. 시작점과 중심가지 capsule의 거리/반경으로 접합부를 판별하여 첫 proximal body와 해당 인접 Rachis body 두 쌍만 제외했다. 열매·다른 가지 충돌, 형상, 질량, 강성, 감쇠, Y회전은 유지했다. body pair 제외이므로 이후 그 두 body끼리의 접촉도 제외되는 범위 제한이 있다.

cyan Y90°에서 수정 전후 2초 정지 영상 확인: 식물 바디 최대 이동51.69→8.66mm. 240Hz/6초 힘 시험 최대23.98mm→잔류1.28mm, 접촉 시험21.78mm, 경고0/reset 일치. 회귀 및 접합부 테스트4개 통과. 6번 열매의 초기 겹침4.93mm는 유지되므로 전체 학습 수집 승인 아님. 다른 GLB의 동적 재검증은 이번 수정에서 미실시.

결과: `/root/docker_share/mujoko_debugging_data/20260923_glb_junction_fix/index.html`. 보존 요약: `validation/glb_junction_fix.json`.


## 최신: cyan 6번 알맹이 제거

`build_glb_physics.py --remove-fruit 6`로 Fruit_06 body/충돌/질량(60.45g)/꽃받침 시각/타깃을 제거한다. 꼭지 가지는 남기고 원본 GLB는 보존한다. 나머지 열매 번호 유지, 총10개. 기존9·10번 연결부 필터 적용. cyan Y90°, segment11에서 초기0.5mm 초과 겹침0개, 정지2초 최대 이동2.20mm. 0.2N 힘 시험 최대25.19mm, 잔류4.85mm. 접촉/복원/reset 및4테스트 통과. 로봇 후보 수집은 이 제거 모델로 재실행하지 않았으므로 수집 성공률/전체 물리 유효성 미검증.

결과 `/root/docker_share/mujoko_debugging_data/20260923_glb_without_fruit06/index.html`.


## 최신: 전체5 GLB 6번 제거 완료

cyan/green/red/white/rotated90 모두 `--remove-fruit 6` 적용, 열매10개 유지. 기존 주줄기·잎 및 Y90° 배치/segment11 유지. 각 모델 240Hz 정지·0.2N 힘·접촉·복원·reset 검사 완료, 회귀12테스트 통과. 초기0.5mm 초과 겹침은 cyan/green/red/rotated90에서0개. white는 Fruit_02와 Rachis_04 충돌체 약1.001mm 겹침이 별도로 남아 사용 보류. 또한 힘 시험 무부하 대비 최대/종료 변위가 모두626.09mm로 복원 검증 실패했으며, 이 큰 이동의 원인을 해당 초기 겹침으로 단정하지 않는다. 다른 알맹이를 임의 삭제하거나 해당 충돌을 숨기지 않았다. 초기2초 최대 바디 이동 cyan2.20/green2.30/red2.35/white3.18/rotated90약0mm. 로봇 후보 수집은 제거 모델들로 재실행하지 않아 전체 수집 물리 유효성은 미검증.

모음 `/root/docker_share/mujoko_debugging_data/20260923_all_glb_without_fruit06/index.html`. 요약 `mujoco-benchmark/validation/all_glb_without_fruit06.json`. 개별 하위폴더의 model.mjb/reference.json 사용. 원본GLB 보존.


## 최신: white 2번 간격 및 탄성 안정성 수정

2번을 중심가지에서 멀어지는 방향으로3mm 이동(원본GLB 좌표 offset은 build.json), 꽃받침/말단 꼭지를 함께 이동하고 proximal 시작점은 유지해 변형했다. 원본GLB 보존, 나머지 body 위치와 모든 질량 유지 검사 통과. 6번 제거 및 연결부 필터 유지. 초기0.5mm 초과 겹침0개.

간격 수정만으로 힘 시험 미복원이 해결되지 않았다. 고정 중력 preload를 쓰는 초기 평형의 (중력 토크 미분+스프링) 대칭 행렬 최소 고유값이 -0.409로 국소 불안정 방향이 확인됐다. white 전용 `--rachis-stiffness-scale 2` 적용: Rachis k2→4, d0.15→0.2121. 최소 고유값+0.203. 물성 조정이며 실측 보정/기존과 동역학 동등성 아님. 다른GLB/기본값은1 유지.

240Hz 0.2N(2.5~3초) 이후12초까지 검사: 5번 X/Y/Z 최대51.89/19.43/26.00mm→잔류2.47/0.287/0.198mm. 2번 X 최대29.89mm→잔류1.47mm. 네 시험 모두 접촉 관통0, 경고0; 별도 구형 물체 접촉 반응26.24mm. 6초 시점 X잔류18.65mm로 빠른복원을 주장하지 않는다. 초기 정지2초 이동 수치오차 수준. 회귀12개 통과. 로봇 수집은 이 수정 모델로 미실행, 모든각도/부착위치 안정성은 미검증.

최신 white: `/root/docker_share/mujoko_debugging_data/20260923_white_fruit02_stable/index.html`. 이전모음의 white 실패기록은 보존한다. 요약 `mujoco-benchmark/validation/white_fruit02_fix.json`.

재생성 명령(새 출력폴더 사용):
```bash
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/build_glb_physics.py --glb /root/farmily_tomato/nvidia-sim/env_usd/tomato_rotate_glb/tomato_master_v10_cluster_curve_white.glb --remove-fruit 6 --fruit-offset 2 -0.0005066500582317888 -0.0008218853535071469 0.002840389090280442 --rachis-stiffness-scale 2 --output /root/docker_share/mujoko_debugging_data/NEW_white_fixed
```


## 최신: 전체 GLB 로봇 수집 소량 검사 완료

2026-09-23, 최신5종/각1개 전체 로봇 경로, Tomato_05, Y90°/segment11, 240Hz. white는2번 위치·강성 수정 모델, 첫seed0 준비자세IK실패 후seed1실행. 총 계획6회/실제물리5회. 결과: green/red/white miss(물리 기준통과), rotated90 partial_center_entry(물리 기준통과), cyan invalid_physics(최대겹침0.514mm로0.5mm기준초과, 학습제외). 경고/수치불안정0.

최대목표밀림 cyan219.00/green156.96/red81.02/rotated90 83.86/white90.58mm. 현재center_entry_only_v2는밀림을실패로분류하지않으므로 유효4개 또는 rotated90부분진입을 안전수확 성공으로 해석하지 않는다. 각1경로라 전체각도/부착위치 성공률이나대량수집승인아님. cyan 접촉중겹침은미해결로보존.

5개 초기RGB-D/목표mask640×480 저장, 각가상카메라로action_camera재변환 및복원검사통과. 후보states/trace해시·수치유한성·관측모델해시일치검사통과, 회귀12테스트통과. 관측의실제관절도달성은미검증.

결과 `/root/docker_share/mujoko_debugging_data/20260923_024914_all_glb_robot_smoke/index.html`. 보존요약 `mujoco-benchmark/validation/all_glb_robot_smoke.json`.
