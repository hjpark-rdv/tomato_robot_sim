# 송이 50% 축소와 물리 판정 정정

작성: 2026-09-23. 브랜치: `mjlab-performance`.

## 변경 내용

기존 주줄기·잎·로봇을 보존하고, GLB 송이의 가지·꼭지·열매 전체를 부착점 기준으로 균일하게 축소한다. 시각 형상과 충돌 형상 모두 적용한다.

| 항목 | 새 설정 |
|---|---|
| 송이 길이·반경 | 기존의 0.5배 |
| 송이 질량 | 기존의 0.125배 (밀도 유지) |
| 강성·감쇠·armature | 기존 설정 유지 |
| 회전 | 기존 GLB Y축 0–180° |
| 주줄기·잎·로봇 | 기존 모델 유지 |

`build_glb_physics.py`, `generate_random_glb_scenes.py`, `collect_random_glb_scenes.py`의 CLI 기본값은 `--truss-scale 0.5`다. 라이브러리 `build()`와 `generate()`의 생략 기본값은 호출 호환성을 위해 1이다. 규모가 달라지면 변형 응답도 달라지므로 기존 동역학과 같다고 해석하지 않는다.

## 잘못된 성공 표시의 원인

대상은 `20260923_random_glb_full_smoke/scene_0001/targets/Tomato_05/physics/candidates/candidate_00005`다.

- 기존 결과는 `partial_center_entry`였다.
- 저장 상태의 약 7.633초에 고리와 `STEM_MainStem_03` 사이 최대 관통 3.860mm를 확인했다.
- 기하 중심 진입 조건은 저장 프레임 약 46.033초부터 만족했다. 열매 최대 변위는 약 119mm였다.
- GLB 전용 관통 검사가 기존 주줄기와 고리의 접촉을 놓쳐 물리적으로 유효하다고 판정했다.
- 같은 명령을 독립 재실행한 결과, 수정된 검사에서는 `invalid_physics`다. 정상 진입 성공이나 단순 `miss`로 취급하지 않는다.

새 GLB 실행은 GLB 접촉뿐 아니라 **고리 전체 접촉**의 관통도 0.5mm 기준으로 검사한다. `physics_guard_version=glb_and_hook_v2`를 기록한다. 기하 중심 진입 조건과 변위를 참고 지표로 저장하는 정책은 변경하지 않았다. 새 후보 결과에는 기하 진입 최초/최종 시각과 해당 스텝 수도 저장한다.

## 기존 데이터와 학습

기존 물리 상태·결과 JSON은 보존했다. 해당 HTML에는 정정 안내와 새 실행 링크를 추가했다. 전체 기존 smoke의 저장 지표를 조사하면 성공 1개와 miss 3개가 추가 제외 대상이다. 목록은 아래 검증 JSON에 있다.

랜덤 장면 학습 로더는 기존 GLB 자료의 `max_hook_contact_penetration_m`도 확인해 해당 후보를 제외한다. 이전 smoke 학습 가중치와 성능 수치는 오류 영향을 받았으므로 유효한 학습 결과로 사용하지 않는다. 이번 작업에서 재학습은 하지 않았다.

크기가 다른 장면은 같은 수집 폴더에 이어 붙일 수 없다. 기존 100% 크기 수집을 resume/extend하려면 기존 인자와 함께 `--truss-scale 1`을 명시한다. 저장된 scale이 없는 과거 기록은 1로 해석한다.

## 검증 결과

| 검사 | 결과 |
|---|---|
| 같은 배치의 50% 송이 3장면 | 생성 완료, 2종 GLB 포함 |
| 2초 정지 검사, 240Hz | 3장면 통과 |
| 최대 정지 관통 | 0 / 0.174 / 0mm |
| 최대 정지 이동 | 1.897 / 2.778 / 2.050mm |
| 송이 위치의 정확한 0.5배 변환 오차 | 최대 3.62e-13m 미만 |
| 비송이 바디 초기 위치 차이 | 0m |
| 송이 질량 1/8 검사 | 오차 0 |
| 기존 주줄기 geom 위치·회전·크기 | 동일 |
| scene_0001 / Tomato_05 로봇 2후보 | 둘 다 miss, 물리 유효성 통과 |
| candidate_00000 저장 상태 재생 검사 | 1,534프레임 / 51.1초 확인 |
| 관련 자동 테스트 | 26개 통과 |

정지 검사와 2후보 실행만 확인했다. 모든 GLB·배치·접촉에 대한 검증이나 수확 성공을 뜻하지 않는다. 축소 모델의 탄성 응답 실측 보정은 미실시다.

## 실행 예시

새 50% 장면 3개 생성:

```bash
cd /root/farmily_tomato
./mujoco-benchmark/.venv/bin/python \
  mujoco-benchmark/scripts/generate_random_glb_scenes.py \
  --seed 5 --scenes 3 --trusses 1 --truss-scale 0.5
```

작은 수집 실행: 장면 1개, 모든 열매별 후보 10개와 RGB-D 9시점. 기본 datetime 출력 폴더로 저장한다. 아래 명령은 다음 실험용이며 이번 검증에서 실행한 전체 수집 명령은 아니다.

```bash
./mujoco-benchmark/.venv/bin/python \
  mujoco-benchmark/scripts/collect_random_glb_scenes.py \
  --seed 5 --scenes 1 --trusses 1 --truss-scale 0.5 \
  --candidates 10 --workers 4 --planning-workers 4 --hz 240
```

장면 1개는 연결 확인용이며 train/validation/test 분할 학습용이 아니다. 수집·학습 절차는 [RANDOM_GLB_SCENES.md](RANDOM_GLB_SCENES.md)를 참조한다.

## 보존 자료

- 검증 요약: [half_scale_and_contact_guard.json](validation/half_scale_and_contact_guard.json)
- 크기 비교: `/root/docker_share/mujoko_debugging_data/20260923_half_scale_glb/comparison.html`
- 3장면: 같은 폴더 `index.html`
- 축소 모델 로봇 시험: 같은 폴더 `robot_smoke/index.html`
- 문제 후보 재실행: `/root/docker_share/mujoko_debugging_data/20260923_center_entry_audit/reexecuted/index.html`
- 기존 자료 관통 재검토: 같은 audit 폴더 `legacy_audit.json`

Git에는 코드·문서·작은 검증 요약을 보존한다. 모델·상태·렌더 결과는 위 외부 데이터 폴더에 보존한다.
