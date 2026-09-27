# 목표 과실 접촉 허용 후 서버 재시험 결과

2026-09-28. 기준 `804e40cec63bae8272e7d692a29673dc3451e1c6` (기능 `aa22bf1`).
작업 브랜치 `codex/server-target-fruit-contact-20260928`. 원본 작업 브랜치와 사용자 미추적 bundle 보존.

## 결론

**목표 과실 접촉만 허용하면 마지막 5개 기존 경로 모두 전체 명목 검사를 통과한다. 그러나 실제 정상 시작 실행 5회는 목표 과실 힘 합 1N 초과로 중단됐고, 동일 기하 감속 1회는 변위 5mm 초과로 중단됐다. 정상 접근 → 꼭지 안착·유지는 아직 확보하지 못했다.**

이번 결과는 종전의 “목표 과실 접촉 자체가 금지 충돌”이라는 병목을 제거한 결과다. 추가 금지 장애물은 명목 전체 검사에서 검출되지 않았다. 실제로 끝까지 통과한다는 뜻은 아니다. 모든 실행이 `seat` 도중 중단되어 나머지 경로의 실제 접촉·유지는 미검증이다.

1N/5mm는 사용자 지시에 따른 첫 관찰용 조기 중단값이다. 이를 손상 한계, 수확 불가능 또는 영구 실패로 해석하지 않는다. 한계를 자동 상향하지 않았다. 물리 총 **6회 = 기존5 + 감속1**, 예산7회 이내에서 종료했다. 별도 baseline 재실행, 신규 기하 경로, aperture 비교군, RL, 데이터 수집은 수행하지 않았다.

## 전체 결과

힘은 **접촉별 3차원 힘 크기 합**의 최대다. L은 live 이전 solve, P는 현재 상태 private-forward. 두 시점의 힘을 더하지 않는다. 변위는 처음 상태 대비 전체 실행 중 최댓값이다. 과실 침투는 목표 과실/로봇 와이어 접촉 기록에서 검출한 최댓값이다.

| candidate | 전체 검사 완료 / 남은 금지 충돌 | 실제 실행 | 중단 원인 / 시점 | 목표 과실 힘 L/P (N) | 최대 과실 변위 (mm) | 과실 침투 (mm) | 비허용 접촉력 (N) | 꼭지 안착 / 유지 | 실제 상태 영상 |
|---|---|---|---|---|---|---|---|---|---|
| seating_00_under | 완료·통과 / 미검출 | 실행 | 힘 / 16.1125s | 0.863 / 1.074 | 2.968 | 0 | 0 | 0 / 미도달 | [MP4](videos/seating_00_under.mp4) |
| seating_03_under | 완료·통과 / 미검출 | 실행 | 힘 / 16.4292s | 1.025 / 0.831 | 2.968 | 0 | 0 | 0 / 미도달 | 원자료 저장 |
| seating_04_under | 완료·통과 / 미검출 | 실행 | 힘 / 16.2208s | 0 / 1.061 | 2.968 | 0 | 0 | 0 / 미도달 | 원자료 저장 |
| seating_05_under | 완료·통과 / 미검출 | 실행 | 힘 / 16.1417s | 0.970 / 1.012 | 4.567 | 0 | 0 | 0 / 미도달 | [MP4](videos/seating_05_under.mp4) |
| seating_06_under | 완료·통과 / 미검출 | 실행 | 힘 / 16.2708s | 0.972 / 1.083 | 3.948 | 0 | 0 | 0 / 미도달 | 원자료 저장 |
| seating_00_under · 18배 감속 | 완료·통과 / 미검출 | 실행 | 변위 / 28.3000s | 0.604 / 0.604 | 5.015 | 0 | 0 | 0 / 미도달 | [MP4](videos/slow_seating_00_under.mp4) |

표의 “0 / 미도달”은 기하 안착 표본0, hold/verify 구간 미도달이다. 유지 실패 시험 또는 유지 통과로 집계하지 않는다. 목표 꼭지 접촉력도 전 실행에서0이었다. `training_eligible=false`, `hook_success=null`을 유지했다.

전체 모델의 모든 활성 접촉을 포함한 침투 감시에서는 첫 physics step(0.0041667s)의 최대 **0.228mm**가 각 실행의 최댓값이었다. 표의 목표 과실 침투0과 구분한다. 기존 **0.5mm 물리 기준 이내**이며, 중단 이전 물리 유효성은 모두 통과했다. 이 검사는 활성 충돌체의 이산 시점 검사이며 시각 메시 전체나 스텝 사이의 비관통 보증이 아니다.

## 감속 비교: 힘은 감소했지만 변위 제한 도달

`seating_00_under`의 `seat`를 포함하는 기존 joint 명령 구간을 18배로 재시간화했다. IK/경유점/기하를 새로 생성하지 않았다. 원본 모든 command/phase 꼭짓점을 정확히 보존했고, 추가 명령은 원래 선형 joint 구간 위에만 있다. 검사 결과 최대 joint 경로 차이0이다. `replay_assets`는 원본 읽기용 연결, 새 trace/plan/hash는 별도 `slow_run`에 저장했다.

- 명목 전체 길이: 20.6000s → 55.7333s. 실제 감속 실행은 28.3000s에서 중단.
- 기존 명목 seat TCP 최대: 34.950mm/s. 감속 명목 최대: **1.942mm/s**.
- 감속 실제 TCP 최대: **2.397mm/s**. RING 중심의 매 240Hz poststep 위치 차분이며, 명목2mm/s를 실제 속도 상한으로 주장하지 않는다.
- 감속 중 목표 과실 힘 최대0.604N, 변위5.015mm. 힘 제한은 피했지만 변위 제한에 도달했다.
- 감속 중단점은 원래 명령 경로의 시간 매개변수 약16.1639s에 해당한다. 기존 중단점16.1125s보다 약0.0514s 분량의 원래 경로를 더 진행했을 뿐 안착 목표에 도달하지 못했다.
- 감속의 양의 목표 과실 접촉력은 private 기준26.4542~28.3000s 사이에서 검출됐다. 377개 양의 힘 표본의 **연속 인접 양성 구간 시간 합은1.3042s**이다. 힘이0인 틈은 제외하며 이 수치를 꼭지 유지 시간으로 해석하지 않는다.

속도를 낮추면 이 경로의 순간 하중을 줄일 수 있다는 제한된 증거다. 같은 기하를 더 진행하려면 과실을 계속 밀게 되는지, 방향/회전 시점을 바꿔 접촉을 짧게 끝낼 수 있는지는 후속 검토 대상이다. 현재 자료로 정적 불가능이나 실물 안전을 결론 내리지 않는다.

## 매핑과 접촉 정책

실제 원본 run root:
`/root/docker_share/mujoko_debugging_data/20260927_hook_seating_server/robot_under`

manifest/replay_assets/candidates를 갖춘 실제 실행 폴더를 사용했다. 입력 폴더만으로 모델이 있다고 가정하지 않았다. 기존 `diagnostic_policy.json`을 base-policy로 사용했다.

- 과실: `glb_col_Tomato_02` 하나.
- 실제 CAD 반원 캡슐 검사로 확인한 와이어32개: `g376`~`g407`. 숫자 범위로 허용 목록을 만든 것이 아니다.
- 목표 꼭지: `glb_col_Attachment_01`, `glb_col_TRUSS_Pedicel_proximal_02_02`.
- 뒤쪽 와이어16개: `g384`~`g399`.
- 과실/와이어는 `insert/seat/hold/verify`만 허용. 목표 꼭지/뒤쪽 와이어는 기존 `seat/hold/verify`만 허용. phase 경계는 양쪽 조건을 적용했다.
- 고리 마운트, 팔, Rachis, 다른 과실·꼭지, 주줄기, 거터로 허용 범위를 확대하지 않았다.
- RING 중심과 Hook 원점을 별도 기록했다. [실제 좌표 매핑](mapping_verified.json).

5개 audit 모두 `complete=true`, `status=sampled_clear`, 마지막 시점까지 검사했다. 샘플 수는 각각2871/2900/2899/2873/2882다. 실행 전 같은 전체 검사를 다시 수행했고 감속 경로도 전체 검사를 통과한 뒤 실행했다. `complete=false` 결과로 실행한 사례는 없다.

검사는 명목 로봇과 초기 환경의 이산 검사다. 동적 변형/추종 오차는 실제 실행 기록으로 분리했다. 시각 전용 식물에는 없는 충돌을 만들어 검사한 것이 아니며, 기존 로봇 자기충돌 FCL 검사와 환경 검사의 범위를 혼동하지 않는다.

## 힘·시간·추종 기록과 해석

- 실제 물리240Hz의 매 step에서 기하, 힘, 변위, 침투, 추종 오차 기록. 6실행 모두 timestamp 누락 간격0.
- live force는 `poststep_time - dt`의 solve 결과, private-forward는 복사한 MjData에서 poststep 시점으로 재계산한 값이다. live simulation에 forward를 추가하지 않았다.
- 모든 접촉의 force6·활성 제약·거리·정확한 geom 쌍은 `contact_events_240hz.json.gz`에 보존했다. 원자료에서 작은 힘이나 양의 간격 접촉을 삭제하지 않았다.
- 특히04번은 live에는 아직 힘이 없지만 private-forward에서1.061N을 검출한 즉시 중단했다. 시점 차이이며 두 값을 평균하거나 상쇄하지 않는다.
- 목표 과실 접촉력이 있으면서 침투0인 경우도 있다. 기존 margin/gap과 활성 제약 때문에 양의 간격에서도 힘이 발생할 수 있다. 이번에 margin/gap을 변경하지 않았다.
- 리프트 최대 추종 오차: 원래00/03/05/06 및 감속3.665mm, 04는5.568mm. 회전 관절 최대0.00880~0.00997rad. 전체 구간 최댓값이며 실제 추종이 명목 경로와 완전히 같다고 주장하지 않는다.
- 과거 baseline의 `non_target_force_N` 최대3.397N에는 **목표 과실 접촉도 포함**됐다. 이를 모두 다른 과실/줄기에 대한 힘으로 해석하면 안 된다. 과거 단일 법선력 최대와 이번 여러 접촉의 힘 크기 합은 다른 지표다.
- 각 실행의 `legacy_evidence`와 `authorized_contact_evidence`를 함께 보존했다. 접촉 허용 변경이 꼭지 포획 성공으로 이어지도록 라벨을 변경하지 않았다.

## 구현·테스트·보존

기존 `target_fruit_contact_trial.py`는 서버 자산에서 그대로 실행 가능했다. 연결 실패나 물리 설정 우회가 필요하지 않았다. 이번 수정은 감속 시험용 동일 joint 경로 재시간화 도구, 매 physics step TCP 속도 기록, 접촉 합/시간을 구분한 읽기 전용 요약, 실제 힘 합을 표시하는 영상 자막이다.

- `retime_seating_trace.py`: 새 run에만 저장, 원본 명령 꼭짓점/구간 보존.
- `summarize_target_contact_trial.py`: live/private와 양의 힘 연속 구간을 분리, 누락 간격을 접촉 유지로 채우지 않음.
- 최종 관련 테스트 **188 passed**, 별도 Torch/FCL 기존 계획 회귀 **5 passed**. 총193개, skip 없음.
- Python3.11.14 / MuJoCo3.13.0 / NumPy2.4.6 / SciPy1.17.1. [테스트](tests_final.log), [계획 회귀](tests_planner.log).
- source model/manifest/reference/initial_trace/plan/trace/base-policy SHA256 전부 원본과 일치. [검증](verification.json).
- 명령·scope·코드 해시, 두 판정, 전체 검사, 240Hz 접촉/표본, 실제 상태 NPZ, 영상3개를 Git에 보존. 대형 원본 MJB/메시는 서버에 보존하며 새로 Git에 넣지 않았다. 보고서/영상 검토에는 별도 파일 전달이 필요 없지만 타 머신에서 실제 재실행하려면 원본 대형 자산이 필요하다.
- 영상3개 FFmpeg 전체 디코드·프레임 수·SHA256 확인. 두 기존 경로와 감속 경로 모두 실제 저장 qpos를 재생하며 재시뮬레이션하지 않는다. 왼쪽은 명목 비교, **오른쪽이 실제 물리**. 제목에 중단 원인과 시점 표시. [영상 검사](video_validation.json).
- 후처리 매핑 추출 중 NumPy의 1원소 배열 int 변환 오류를 scalar 인덱스로 수정했다. 실제 실행/물리 결과에는 영향이 없고 최종 검증은 정상 완료했다.

## 자료와 재현

[전체 수치 JSON](results.json) · [증거 목록](README.md)

호스트에서 대시보드:

```text
file:///home/rdv/docker_share/mujoko_debugging_data/20260928_target_fruit_contact_server/index.html
```

원본 5개 중 하나를 재실행할 때는 출력 이름을 새로 지정한다. 아래 명령은 **추가 물리 실행**이므로 이번 완료된 예산에 더 실행하지 않았다.

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/target_fruit_contact_trial.py \
  /root/docker_share/mujoko_debugging_data/20260927_hook_seating_server/robot_under \
  --candidate seating_00_under \
  --base-policy /root/docker_share/mujoko_debugging_data/20260927_hook_seating_server/diagnostic_policy.json \
  --output /root/docker_share/mujoko_debugging_data/NEW_UNIQUE_TRIAL \
  --execute --max-target-force-n 1.0 --max-target-displacement-m 0.005
```

SIM-GT 오프라인 진단이며 자동 수확, 실물 안전, 손상 없음, 정상 접근 후 기계적 걸림·유지 모두 미입증이다. 이번 증거는 기존 학습 데이터와 혼합하지 않는다.
