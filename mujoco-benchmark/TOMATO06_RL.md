# Tomato_06 자유 동작 PPO 파일럿

## 실행

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.mjlab-venv/bin/python -u mujoco-benchmark/scripts/train_tomato_rl.py \
  --num-envs 16 --iterations 200 --steps-per-env 32 --episode-seconds 30
```

짧은 점검은 `--iterations 20 --episode-seconds 15`. CPU 수집기의 `--workers`와 달리 `--num-envs`는 **하나의 MJWarp GPU 시뮬레이터 안의 환경 수**다. `iterations`는 PPO 업데이트 횟수이며 경로 후보 수가 아니다. 위 기본 명령은 102,400개 환경 제어 스텝을 수집한다. 전체 실행 후 독립 평가를 추가한다.

출력: `/root/docker_share/mujoko_debugging_data/YYYYMMDD_HHMMSS_tomato06_rl/`

- `manifest.json`: 관절 이름, 속도 상한, 버전, 모델 해시, 보상/알고리즘 설정, 초기 오차
- `rl_model.xml`, `rl_model.mjb`: 실제 이번 실행의 별도 모델
- `progress.jsonl`: 업데이트별 rollout/학습 시간, 손실, 보상, 종료 원인 누적
- `model_XXXXX.pt`: 25회마다 및 마지막 체크포인트
- `summary.json`: 준비/학습/평가 시간, 처리량, 평가 결과
- `evaluation_world0.npz`: 학습과 별도로 초기화한 결정적 평가 첫 환경의 실제 qpos 기록(20Hz, 종료 직전 상태 포함)

재생(영상 인코딩이 아니라 뷰어):

```bash
DISPLAY=:0 ./mujoco-benchmark/.mjlab-venv/bin/python \
  mujoco-benchmark/scripts/replay_tomato_rl.py /root/docker_share/mujoko_debugging_data/실행폴더
```

`--checkpoint /.../model_00200.pt`로 가중치·옵티마이저를 불러와 새 폴더에 추가 학습한다. 환경 상태는 동일 초기 상태로 초기화하며 난수 상태를 복원하는 비트 단위 중단 재개는 아니다. 추가 학습의 iteration 번호는 0부터다.

## 범위 및 물리

- mjlab Simulation/MJWarp + 설치된 RSL-RL PPO. RGB-D 입력, RL 이전 경로 후보, IK, 경로 유형 지정, 배치/물성 무작위화는 사용하지 않는다.
- 원래 common-ready 로봇 자세에서 시작. 식물의 qpos/qvel/외력 preload, 로봇 명령, 에피소드 통계 모두 reset한다. 고리나 열매를 순간이동하여 진입시키지 않는다.
- 액션 7개: 팔 6관절과 리프트의 속도 의도를 적분한 position drive 목표. 고리 회전 자유. 실제 모델 관절 한계 적용, 보수적 명령 속도 상한 팔 0.25rad/s, 리프트 0.04m/s. 실제 하드웨어 측정값이 아니다. 막힌 상태에서 명령 누적을 제한한다.
- 120Hz 물리 / 20Hz 정책. 기존 질량·탄성·식물 충돌·마찰·제어 게인 유지. 파단 비활성.
- 기존 경로 재생 모델의 로봇 자기충돌 비활성 조건은 자유 관절 탐색에 부적합하여 **RL 모델만** 비인접 로봇 링크 간 충돌 활성화. 인접/동일 강체 링크 제외. MJWarp MULTICCD 제약으로 로봇 충돌체 margin/gap=0, 식물은 기존 0.5mm 유지(기본 결합의 max로 로봇–식물 유효 margin 유지). 원본 모델 파일은 수정하지 않는다.
- GT 관측: 관절 위치/속도/명령 오차, 고리 좌표계의 목표 중심, 고리 회전행렬, 목표 이동량, 모든 열매 최대 이동량, 이전 액션, 남은 시간 정보. 고정 장면 동작 탐색용이며 sim-to-real/영상 정책이 아니다.

## 판정과 보상 한계

- 기존 `center_region`과 같은 고리 내부 기하 조건: rear 반원 쪽에 중심이 있고, 열매 구 단면이 와이어와 겹치지 않으며 고리 평면에서 2mm 이내.
- 0.2초 유지하고 **전체 열매의 에피소드 최대 이동량 <=20mm**, 로봇 관련 관통 <=2mm이면 부분 진입 성공. 꼭지 걸림/수확 성공과 다르다.
- 접근 거리 감소 shaping + 내부 진입/유지 보상. 전체 열매 밀림, 비목표 접촉, 액션 급변 감점. 최대 이동량 또는 관통 한계 초과 시 종료. 시간 제한은 유한 길이 문제의 terminal로 처리한다.
- 물리 substep마다 변위/접촉 진단, 비정상 상태/auto-reset/접촉 버퍼 overflow는 실행 오류로 중단한다. 접촉 표시는 기하 접촉 정보이며 힘 센서 측정치가 아니다.
- 평가 환경들은 동일 초기 상태/결정적 정책이므로 서로 독립적인 일반화 표본이 아니다. 실제 성공 판단은 저장 재생으로 추가 확인해야 한다.
- 멀리서 시작하는 7관절 탐색은 어려울 수 있다. 짧은 검증 통과는 진입 정책 학습 성공을 뜻하지 않는다.

## 참고

- https://github.com/mujocolab/mjlab
- https://github.com/leggedrobotics/rsl_rl

## 짧은 검증 결과 (2026-09-22)

최종 RL 전용 모델, 16환경 × 32스텝 × 20업데이트 = 10,240 전이. 준비 11.36초(커널 캐시 있음), 학습 28.88초, 평가 5.69초, 총 45.93초. 학습 처리량 354.6 전이/초. 최초 자기충돌 GPU 커널 컴파일은 별도로 수 분 소요되었으므로 새 캐시에서는 준비가 길어진다.

학습 32에피소드: 과도한 열매 변위 13, 시간 제한 19, 부분 진입 성공 0. 독립 평가 역시 성공 0/16으로, 학습 파이프라인 검증만 완료했으며 성공 정책은 아직 아니다. NaN/폭주/접촉 용량 초과 없이 완료했다. PPO 가중치 갱신, 선택적 reset, 체크포인트 로드 후 추가 업데이트, 저장 평가 재생 검증 통과. 기존 중심 진입 판정과 임의 위치 1,000개가 일치했다.

실행 결과: `/root/docker_share/mujoko_debugging_data/20260922_182738_tomato06_rl/`. 요약 보존: `validation/tomato06_rl_smoke.json`. 같은 처리량이라면 200업데이트 학습은 약 5분 수준이며 접촉 복잡도에 따라 달라진다.

## 2026-09-22 GPU 환경 확대 및 추가 학습 실행

기존 사용자 실행 `20260922_184128_tomato06_rl`의 200회 체크포인트에서 추가 학습을 시작했다. 짧은 동일 조건 비교(3초 에피소드, 32스텝, 4업데이트): 64환경 1,150전이/초, 256환경 2,352전이/초. 256환경에서 관측한 GPU 사용률 95%, 메모리4,414MiB(평균/최댓값 아님). 기존16환경 357전이/초는 30초 에피소드의 장기 측정이라 엄밀히 동일 조건 비교는 아니다.

- 중단된 실행: `/root/docker_share/mujoko_debugging_data/20260922_185257_tomato06_rl_gpu256`
- PID 286916 (종료/상태 확인 전 프로세스 명령을 다시 확인할 것)
- 환경256, 추가1,000업데이트, 32스텝, 에피소드30초. 총8,192,000전이. 기존 물리/보상 유지.
- 로그: 같은 경로 뒤 `.log`; 실행 인자/PID는 `.launcher.json`. 25업데이트마다 체크포인트, 완료 후 summary/평가 재생 데이터 저장.
- progress.jsonl에 처리량·누적 학습시간·잔여 예상분 추가. 약1시간 예상이나 접촉 복잡도에 따라 달라짐. 이 기록 작성 시 본 학습은 진행 중이며 성공 여부 미확인.
- 측정 보존: `mujoco-benchmark/validation/tomato06_rl_scaling.json`.

## 10분 학습 비교로 전환

사용자 요청으로 기존 1,000회 추가학습(PID286916)은 중단. 해당 실행의 model_00025.pt를 보존했으며 이후 미저장 업데이트는 보존되지 않았다. 비교의 출발점은 원래 사용자 200회 체크포인트다.

`--train-seconds 600 --compare-before --iterations 100000`으로 학습 구간만 600초 제한, 진행 중인 업데이트 종료까지 몇 초 초과 가능. 준비/기준평가/최종평가는 별도. 실제 완료 업데이트와 전이 수 기록, 마지막 체크포인트 저장. 2초 제한 검증에서 2.76초/2업데이트 후 자동 종료·저장·전후 비교 통과.

완료된 실행: /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min

PID 291847; 로그: /root/docker_share/mujoko_debugging_data/20260922_185721_tomato06_rl_10min.log

완료 후 summary.json의 comparison에 전후 성공수, 평균 최근접 거리, 평균 최대 열매 이동량 저장. baseline_world0.npz와 evaluation_world0.npz 저장. 완료 결과와 해석은 아래 절을 참고한다.

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
