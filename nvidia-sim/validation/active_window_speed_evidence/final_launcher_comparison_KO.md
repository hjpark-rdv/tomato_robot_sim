# 최종 one-command 재실행 독립 비교

상태: **complete**, 완료비교 3/3. 원본220Hz3정차점과최종launcher새실제physics의배열을직접비교했다. 추가physics/GPU실행은0회다.

|대상|물리모델SHA동일|명령동일|저장배열byte동일|실제완주|새분리|새loopRTF|setup(s)|계획준비(s)|모델전환검증(s)|전체구간orchestration(s)|
|---|---|---|---|---|---:|---:|---:|---:|---:|---:|
|22|True|True|True|True|6|2.294|23.644|10.509|42.283|107.159|
|27|True|True|True|True|5|2.575|70.714|46.805|88.103|239.811|
|29|True|True|True|True|6|2.348|70.841|46.128|88.132|237.932|

완료구간물리191.816895초,loop79.563791초,setup165.198407초. 계획준비103.442027초,모델축소/전환검증218.517783초,합산orchestration584.902343초. loop와전체준비대기포함속도를분리했다.

최초 `validate_prepared`와 구간 타이머 이전의 준비는 위 합산에 포함되지 않는다. 별도 `/proc` 시작 시각과 Linux `CLOCK_BOOTTIME` 관측에 따르면 runner의 종료 확인 시점까지 **616.834624초 이하**였다. 이는 종료 후 관측 지연이 포함된 상한이며 정확한 총 실행 시간이 아니다. 정확한 구간 합계 584.902343초와의 차이 31.932281초도 준비·초기 검증·종료 관측 지연을 포함한 상한 잔차이며 전부 로딩 시간으로 귀속하지 않는다. 셸이 Python runner를 시작하기 전의 시간은 별도다. 과거 캠페인은 같은 단계별 타이머가 없어 전체 orchestration 속도비를 추측하지 않는다.

기존실제상태영상재사용자격: **True**. 완료3개모두model/command/rawstate/접촉지표/분리/정책이정확히동일할때만True이며화면USD자산도같아야한다.

JSON: [/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/final_launcher_comparison.json](/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/final_launcher_comparison.json)

감사 스크립트: [/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/audit_scripts/final_launcher_compare.py](/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed/audit_scripts/final_launcher_compare.py). 실행 시점 원본을 보존했으며, 외부 프로세스 시각 관측은 별도 JSON 증거로 덧붙였다.
