# Start recovery 실제 서버 결과 — 2026-09-28

**초기 충돌은 해결했고, D1의 11개 설계를 모두 비교했다. 수확 동작의 성공은 확보하지 못했다.**
두 타깃의 세 자세 비교에서 접근 경로 18개가 검사에 통과했지만, 전체 조작 경로 통과는 0이다.
따라서 문서의 실행 조건에 따라 manipulation physics는 **0/8회**다. 막힌 경로를 강제 실행하지 않았다.
최종 분류는 `CASE2_LOCAL_MANIPULATION`이며 **계획/검사 근거**다. 실제 staging 도달 성공을 뜻하지 않는다.

- 브랜치: `codex/start-recovery-20260928`
- 실행 시작 코드: `30facc9` (기능 코드 `91acd47`, 기존 서버 결과 `ecc6bea` 포함)
- 별도 서버 호환 수정: `3c11d414208becc4fefff9b5e9517059f82f0c98`
- 새 결과: `/root/docker_share/mujoko_debugging_data/20260928_164835_start_recovery_d1`
- 호스트에서 열기: `/home/rdv/docker_share/mujoko_debugging_data/20260928_164835_start_recovery_d1/index.html`
- [최종 기계 판독 요약](validation/start_recovery_server/server_summary.json)
- [Git에 보존한 영상·이미지 대시보드](validation/start_recovery_server/index.html)
- `training_eligible=false`, `hook_success=null`, `impossible=null` 유지.

## 1. 실제 실행 및 예산

요청한 두 문서를 읽고 `run_start_recovery_pipeline.py`의 one-command를 실제 NAS 자산으로 실행했다.
canonical q를 찾은 뒤 별도 사용자 확인 없이 D1 → suffix → 전체 검사 → 실행 대상 선별까지 진행했다.

| 항목 | 결과 |
|---|---|
| 시작 q 검사 | 28개, 정적 유효 12개 |
| 상위 시작 q의 2초 hold | 6/6 PASS, 240 Hz |
| D1 고유 설계 질의 | 11개, 모두 평가 |
| adapter 호출 | 15회: 최초 11회 + IK 진입 전 호환 오류 4건만 동일 입력 재실행 |
| 접근 경로 확보 | 6질의 / 서로 다른 2타깃 / 총 18개 branch |
| 전체 경로 PASS | 0 |
| manipulation physics | 0회, 상한 8회 |
| 별도 영상용 canonical hold | 1회 × 2초, 실제 qpos 저장 |
| 원본 one-command wall time | 647.64초, 호환 수정 후 4건 재실행·렌더 별도 |

실패한 원본 출력도 보존했다. 최초 환경 연결 실패 `20260928_164744_start_recovery_d1`는
FCL import 오류로 계획 전 종료했으며 IK/경로 실패에 집계하지 않았다.

## 2. 해결한 초기 충돌

covered 모델 SHA256:
`6eaf5178c99be858a89a3c6f1d3dc39d578c9d5196a5a4e2be537d7f7543d724`

모델: `/mnt/nas_rdv_md3/covered_d1_20260928/covered_scene/model.mjb` (약 838 MiB).
기존 `Hook/g410`과 `neighbor_truss_collision_fruit_p19_t02_g052`의 **2.2344 mm 겹침**이 있었다.
다른 관절은 유지하고 **wrist2, q[5]만 −7.2°** 변경했다.

```text
original: [0.5036082864, 3.2516102791, 0.1146665663, -1.6495294571,
           -0.1797474325, -1.3975170851, 0.6881734133]
canonical:[0.5036082864, 3.2516102791, 0.1146665663, -1.6495294571,
           -0.1797474325, -1.5231807947, 0.6881734133]
```

선택 q에서 정적 검사 최소 거리는 **13.653 mm**, closest pair는 g410 ↔ 다른 이웃 과실 g062다.
2초 hold에서 로봇 drift 0, warning 0, 식물 최대 이동 2.968 mm.
원래부터 있던 active-plant Rachis04 ↔ Tomato02의 0.228 mm 겹침은 그대로 기록했다.
기존 0.5 mm 물리 기준을 바꾸지 않았다. collider/mask/material/margin/actuator/로봇 base도 변경하지 않았다.

이는 **유효한 q로 reset하는 시작 구성**이다. 기존 관통 자세에서 빠져나오는 동작을 실행한 것은 아니다.

## 3. D1 단계별 결과

| 질의 | 장면/타깃 | 설계 | 접근 branch 통과 | 전체 조작 경로의 병목 |
|---|---|---|---:|---|
| q00 | 0000 / 01 | recorded | 0 | 8 seed 중 goal collision 5, IK 미발견 3 |
| q01 | 0003 / 01 | recorded | 0 | goal collision 7, IK 미발견 1 |
| q02 | 0004 / 01 | recorded | 0 | goal collision 7, IK 미발견 1 |
| q03 | 0005 / 01 | recorded | 0 | goal collision 5, IK 미발견 3 |
| q04 | 0004 / 08 | recorded | 3 | seat에서 비목표 꼭지11에 차단 2 branch, seat 자기충돌 1 |
| q05 | 0004 / 08 | neutral | 3 | insert에서 비목표 Tomato10에 차단 3 |
| q06 | 0004 / 08 | neutral_pitch15 | 3 | insert에서 비목표 Tomato10에 차단 3 |
| q07 | 0000 / 08 | recorded | 3 | insert에서 비목표 Tomato10에 차단 2, 전체 검사 미완료 1 |
| q08 | 0000 / 08 | neutral | 3 | insert에서 비목표 Tomato10에 차단 2, 전체 검사 미완료 1 |
| q09 | 0000 / 08 | neutral_pitch15 | 3 | insert에서 비목표 Tomato10에 차단 2, 전체 검사 미완료 1 |
| q10 | 0000 / 01 | side_control | 0 | goal collision 5, IK 미발견 3 |

q00–03와 q10은 모두 새 initial q가 유효하다. IK가 수렴한 해도 주줄기/다른 과실/로봇 자기충돌 때문에
goal로 채택되지 않았다. 이들을 순수한 IK 수치 수렴 실패나 목표 pose의 절대적 불가능으로 합치지 않는다.
대표 q00은 robot_geom_19 ↔ g22(STEM_MainStem_05), 약 0.980 mm 겹침이다.

접근이 확보된 18 branch의 suffix 분류:

- 환경 접촉 차단 **14**: insert의 Tomato10 **12**, seat의 proximal pedicel11 **2**.
- 자기충돌 **1**: seat에서 link2 ↔ lift platform.
- 전체 검사 미완료 **3**: q07–09 branch02, `LimitReached: overall_planning_deadline`.
  이 세 개는 충돌 미발견이나 경로 통과가 아니다. 120초 기존 질의 한도를 늘리지 않았다.

접근 경로는 direct checked **15**, OMPL RRTConnect **3**이다. 18개 모두 별도의
approach-only audit가 `complete=true, passed=true`였다. 새 canonical q에서 새로 계획했고,
기존 prefix command를 복사하지 않았다. 명령 배열 SHA와 첫 q 검증은
[new_prefix_hash_comparison.json](validation/start_recovery_server/new_prefix_hash_comparison.json)에 보존했다.

q09의 첫 충돌은 q08과 같다. pitch 변경으로 유리해질 수 있는 후속 구간 이전에 차단되므로,
이 결과만으로 pitch 효과가 없다고 일반화하지 않는다.

### 모델 범위의 중요한 제한

**문서의 현재 one-command 구현은 dense covered MJB를 시작 q 검증에만 사용한다.**
`run_new_start_d1.prepare()`는 기존 campaign의 scene별 source MJB를 materialize한다.
이번 D1은 4개의 기존 장면(각 두 송이 모델)의 타깃/설계를 비교한 것이며,
106개 주변 송이를 포함한 dense covered MJB에서 11질의를 수행한 결과가 아니다.

설계를 바꾸지 말라는 요청에 따라 이를 임의로 대체하지 않았다. dense 모델의 좌표·타깃 매핑이 다른
장면에 단순 MJB 교체를 하면 기존 설계 비교도 깨진다. 각 source/selected/planning SHA는 요약 JSON과
원본 provenance에 남겼다. **dense 온실 전체에 대한 D1 접근성·장애물 안전성은 미검증**이다.

## 4. 필요한 서버 호환 수정

시작 q를 바꾸면 ring 위치도 변한다. 기존 `native_geometry()`는 현재 ring→target 방향으로
historical neutral/neutral_pitch15의 heading을 다시 만들었고, 고정해야 할 준비 pose가 바뀌었다.
따라서 네 질의가 IK 이전에 `Local design changed approach; not a matched comparison`으로 종료했다.

`3c11d41`은 historical q를 별도 MjData에 넣어 **설계 heading 계산에만** 사용한다.
실제 초기상태·장애물 검사·IK·prefix는 canonical q 그대로다.
기존 preparation-geometry 일치 검사는 유지했다. 기존 모드에는 reference override가 없다.

수정 파일:

- `scripts/run_motion_family_search.py`
- `scripts/plan_approach_connection.py`
- `tests/test_start_recovery.py`

실제 MuJoCo tiny model과 기존 후보 생성기를 결합한 회귀시험으로 준비 pose 유지,
live reset 불변, 잘못된 reference q 거부를 검증했다.
네 오류만 기존 runner의 `run_plans`로 새 출력에 재실행하고, 11개 결과를 합쳐
기존 `run_physics`로 대상 선별했다. 새 경로 생성기나 실행기를 만들지 않았다.

`.recovery-venv`에 빠져 있던 `python-fcl==0.7.0.11`을 설치했다.
MuJoCo 3.13.0 / Torch 2.7.0+cu128 / OMPL 2.0.1. 기존 다른 Python 환경은 수정하지 않았다.
영상용 보조 스크립트의 GL 초기화 실패는 기록을 보존하고 EGL renderer로 기존 저장 상태를 재생했다.

## 5. 영상·검증·재현

- [초기 충돌 전후 실제 qpos MP4](validation/start_recovery_server/start_pose_comparison/idle_ab.mp4):
  왼쪽 과거 invalid-start 물리 기록, 오른쪽 이번 canonical hold. 동일 covered MJB.
- [새 canonical hold MP4](validation/start_recovery_server/canonical_hold_video/canonical_hold.mp4):
  이번에 실제 480 physics step을 저장, 2배속. 접근/수확 영상이 아니다.
- [준비 pose 충돌](validation/start_recovery_server/diagnostic_images/q00_nominal_collision.png),
  [seat 충돌](validation/start_recovery_server/diagnostic_images/q04_nominal_collision.png):
  명목 q의 기하 이미지이며 실제 실행 영상이 아니다.

기존 시작 복구/환경/계획기 회귀 **121 passed**, 영향받는 추가 테스트 **49 passed**.
MP4 두 개 전체 decode 정상(각 21/11 frames). 명령 hash, 모든 source_unchanged,
18개 접근 audit 완료 및 canonical 첫 q 일치 검증 통과.

재현은 `START_RECOVERY_CODEX_KO.md`의 한 명령을 그대로 사용하되 반드시 새 `--output`을 지정한다.
이번 명령과 호환 오류 재개 스크립트, 테스트 로그, 환경 버전은 증거 압축파일에 포함했다.
`server_evidence.tar.gz`는 새 결과 폴더의 일반 파일을 포함하며 MJB/pkl symlink 자산은 제외했다.
원래 NAS 자산은 경로/SHA로 식별한다. archive만으로 대형 모델을 복구할 수 있다는 뜻은 아니다.

## 6. 다음 판단

이번에 확보한 것은 **충돌 없는 공통 시작 자세 + 두 타깃의 검증된 접근 계획**이다.
시작 충돌 때문에 모든 단계를 못 보던 문제에서, 준비 pose 문제와 local insert/seat 문제를 구분할 수 있게 됐다.

두 타깃에서는 접근 연결을 더 많이 반복하기보다 동일 staging에서 local 경로를 비교할 근거가 생겼다.
다만 실제 staging 도달 물리와 dense 모델의 D1 연결은 아직 확인하지 않았다.
나머지 타깃은 준비 pose의 충돌 없는 IK 해가 먼저다. 모든 타깃을 동일한 RL 문제로 묶을 근거는 없다.
이번에는 RL/새 설계/threshold 완화/추가 표본 탐색을 시작하지 않았다.
