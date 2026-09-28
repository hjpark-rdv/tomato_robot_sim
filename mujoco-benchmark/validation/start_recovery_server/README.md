# Start recovery 서버 증거

[서버 보고서](../../START_RECOVERY_SERVER_REPORT_KO.md), [대시보드](index.html).

`server_evidence.tar.gz`에는 모든 질의의 connection result, IK record, prefix command,
환경 audit, source provenance, canonical hold qpos, 실제 MP4, 명목 충돌 이미지,
실행 스크립트와 로그가 들어 있다. 실패한 원본 및 4건의 호환 수정 후 결과를 모두 보존했다.

추출: `tar -xzf server_evidence.tar.gz -C 새_빈_폴더`.
대형 MJB/pkl symlink는 제외했다. 외부 경로는 `artifact_manifest.json:external_symlinks`,
모델 SHA는 `server_summary.json:models` 및 각 provenance를 확인한다.
NAS가 없는 곳에서도 JSON/영상/로그는 확인 가능하지만 물리 재실행은 외부 모델이 필요하다.

최종 결과는 `compat_recovery/effective_plan_results.json`이다.
원본 `d1/new_start_plan_results.json`의 4건 execution_error를 최종 실패로 세지 않는다.
18개 접근 branch는 계획/검사 결과이며 실제 도달 물리가 아니다.
실제 영상은 hold만이고 왼쪽 과거 기록과 오른쪽 새 기록의 출처를 명시했다.
