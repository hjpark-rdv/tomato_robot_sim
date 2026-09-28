# GPT-6 Pro 전달 — covered D1 서버 결과

브랜치 `codex/server-covered-d1-20260928`의 `COVERED_D1_SERVER_REPORT_KO.md`와 `validation/covered_d1_server/index.html`, `summary.json`을 먼저 보세요. 실제 정지 qpos MP4와 원자료를 Git에 포함했습니다.

**이번에는 D1 경로를 모두 실패한 것이 아닙니다. PHASE1에서 실제 시작 충돌을 확인해, 명시된 FAIL 중단 규칙을 적용했습니다.**

- 기존 dense gutter02/Tomato_02 환경에106송이의7,738개 collider 추가, 주변 주줄기1,088·거터18 유지.
- source 변환 재현, 기존 물성/동역학/actuator/geom 보존, native 양성4/음성4 대조 통과. 회귀시험308개 통과.
- t=0: `Hook` 마운트 `g410` ↔ 식물19/송이2/white GLB의 `Tomato_05`가2.2344mm 관통. 비목표 과실이며 와이어 접촉이 아님.
- 정지2초 A/B: 활성 식물 변위 동일2.9678mm. 보강 로봇은 접촉 때문에3.7270mm 이동. NaN/Inf 상태·warning없음.
- 초기 robot collision 및 실제 robot physics validity FAIL. 계획0/11, 접근·manipulation 실행0/8. 정지 비교2회는 별도 기록.
- 이 dense scene과 `d1fdc7c` 랜덤2송이 장면은 다르다. 모든 장면/타깃 불가능으로 일반화하면 안 됨.
- `training_eligible=false`, `hook_success=null`, `impossible=null`.

다음 요청은 또 다른 universal threshold·검사 항목 추가가 아니라 **정상 로봇 초기 자세/배치 계약을 실제 충돌 모델에 맞추는 것**이어야 합니다. 배경 과실/마운트 충돌을 끄거나 mask·margin·물성을 바꾸지 않습니다. 초기 상태가 유효해지면 약속한 제한 예산 내 D1으로 이어갑니다.

단, 초기 q를 바꾸면 기존 B recorded prefix와 동일 초기조건이 아닙니다. 기존 prefix/결과를 보존하고 original initial-invalid와 new-start 접근 비교를 명시해야 합니다. 기존 prefix 첫 행만 몰래 바꾸거나, 이 결과를 local insert/seat 실패로 분류하지 마세요.

현재로는 RL 판단 이전의 초기조건 문제입니다. 이 한 가지 blocker를 해결하지 않고 경로군·RL·질의 수를 늘리는 작업은 권하지 않습니다.

대형 covered MJB/XML은 NAS에 보존하며 Git에는 해시/위치/재현 명령을 포함했습니다. 보고서와 영상 검토에는 별도 파일 전달이 필요 없지만, 외부 머신에서 물리 재실행하려면 대형 자산 접근이 필요합니다.
