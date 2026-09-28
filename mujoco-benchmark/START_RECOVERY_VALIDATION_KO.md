# Start recovery 구현 검증

기준 조상: `ecc6beafcc50e1c67116c470a1e6e2a44646edbc`  
코드 검증 HEAD: `91acd47e6ad43792d7ade417027cf3d370ea9b45`  
브랜치: `codex/start-recovery-20260928`

GitHub Actions:

- run: `36391161295`
- MuJoCo 3.13.0: **120 passed, skip 0**
- MuJoCo 3.14.0: **120 passed, skip 0**
- compileall: PASS

검증 범위:

- deterministic expanding start-q candidate generation
- bounded search exhaustion keeps `impossible=null`
- native environment overlap rejection
- canonical start selection requires hold-valid candidate
- new-start snapshot rewrites initial q and planner ring position
- historical candidate trace is not copied
- old recorded prefix reuse is disabled for new-start D1
- D1 result reclassification
- one-command pipeline CASE0/1/2/3/4 evidence classification
- existing approach connector, D1 recovery and environment-preflight regressions

이 CI는 실제 838 MiB covered MJB/NAS 서버 실행을 대신하지 않는다. 실제 dense model에서 canonical q가 존재하는지, D1 경로가 생기는지, physics가 실행되는지는 서버에서 `START_RECOVERY_CODEX_KO.md`의 한 명령으로 검증해야 한다.
