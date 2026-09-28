# 주변 장애물 충돌 커버리지 검증 기록

기준 조상: `e5850e3621557b54dfbcc4e3450c246d93a58a0f`  
기능/테스트 HEAD: `9ac6dcf5c7f2c178ac6726b14800f29bc097f346`  
브랜치: `codex/obstacle-coverage-20260928`

## 구현 범위

- 기존 visual-only 주변 송이를 fixed collision obstacle로 변환하는 `add_neighbor_truss_obstacles.py` 추가.
- obstacle class: non-target fruit / Rachis / pedicel / peduncle.
- 기존 main-stem obstacle와 gutter collision은 별도 기존 단계 그대로 사용.
- target fruit / target pedicel / own-truss Rachis의 기존 exact contact allowlist는 변경하지 않음.
- 잎은 hard obstacle로 임의 승격하지 않고 visual-only 유지.
- 일반 랜덤 물리 장면의 `placements`를 주변 visual layout으로 잘못 해석해 중복 collider를 만들지 않도록 explicit `--layout` 또는 neighbor-layout metadata를 요구.
- environment preflight가 visual neighboring truss group 수보다 modeled collision truss group 수가 적은 dense scene을 fail-closed로 거절.
- 새 obstacle geom을 fruit/Rachis/pedicel/peduncle class로 별도 진단 집계.

## CI

GitHub Actions run: `36372889068`

| 환경 | 결과 |
|---|---:|
| Python 3.11 / MuJoCo 3.13.0 | 86 passed |
| Python 3.11 / MuJoCo 3.14.0 | 86 passed |

테스트는 다음을 포함한다.

- source collider에서 fruit mesh와 rachis/pedicel/peduncle capsule을 추출하고 visual-only geom을 제외하는지.
- fixed obstacle의 contact mask와 semantic geom name이 유지되는지.
- truss/stem/gutter coverage audit가 하나라도 빠지면 fail-closed인지.
- active physical scene의 일반 `placements`를 background layout으로 오인하지 않는지.
- environment preflight가 visual neighboring truss만 있고 collision group이 부족한 scene을 거절하는지.
- 기존 environment preflight와 exact search-contact-policy 회귀.

첫 CI 실행은 새 helper가 `greenhouse_visual.py`의 `nums`만 사용하면서 불필요하게 USD `pxr`를 import해 실패했다. helper의 숫자 직렬화를 독립시킨 뒤 후속 CI가 통과했다.

## 아직 검증하지 않은 것

- 서버/NAS의 실제 dense greenhouse MJB에 새 collider를 적용한 통합 build.
- 106개 수준 주변 송이까지 추가했을 때 planning/physics 처리량과 메모리.
- 새 collider를 포함한 initial penetration/idle screen.
- 실제 RB5 경로가 background truss obstacle에 의해 기대한 위치에서 차단되는지.
- 실제 qpos 물리 영상.

따라서 이 커밋은 **장애물 계약과 fail-closed 장면 생성/검사 기능을 구현한 것**이며, dense greenhouse의 성능/수확 성공을 입증한 것은 아니다. 서버에서는 먼저 한 dense scene에 적용해 collider count, 초기 겹침, planner blocker class, 처리량을 확인한 뒤 확장한다.
