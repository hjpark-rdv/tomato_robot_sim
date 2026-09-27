# 환경 사전검사 결과 재검토 — 근거, 수정, 다음 단계
작성: 2026-09-27

## 1. 검토 범위와 결론

이번 검토는 사용자가 제공한 `ENVIRONMENT_PREFLIGHT_PRO_FEEDBACK.zip`의 보고서·JSON·재현 스크립트·이미지와 실제 GitHub 소스를 대조했다. 새로운 온실 물리 실행이나 서버 SSH 접속을 수행한 것은 아니다.

- 원자료 검증 코드: `38cdb6a`.
- 재검토 전 feature HEAD: `95e48fb9af977b715b0b088411a0103188363db5`.
- 이번 코드 수정·테스트 커밋: `1f5bc1ff610fd8bb0b1897fcba8d6b5c51351708`.
- 작업 브랜치: `codex/environment-preflight-20260927`.
- ZIP SHA256: `ab46728410e0fd06858b5d630e21278911ac1967e6b5de9dad6e52466b13da03`.

**결론:** 명목 경로의 위험을 찾아 보류하는 구조는 유지할 가치가 있다. 그러나 전체 고리걸기 성능이 검증된 것은 아니다. 측정 시점과 접촉 의미를 정리한 뒤, 소수 타깃의 실제 고리 포획·유지 경로를 개선하는 데 연구 중심을 돌려야 한다. 전체 온실의 완벽한 물리 모델이나 모든 진단 도구 완성을 고리걸기 연구의 선행조건으로 만들지는 않는다.

## 2. ZIP이 실제로 뒷받침하는 범위

`SERVER_REVIEW_KO.md`, `results.json`의 7개 사례는 모두 첫 위반에서 차단됐다. 전체 길이를 통과한 경로는 없고, 보고서의 `clear_prefix`는 첫 1초만 시험한 것이다. 7개 중 일부는 같은 명령을 다른 장면에서 재사용한 것이므로 독립적인 7가지 성공·실패 분포라고 해석하지 않는다.

| 사례 | 최초 명목 거리 mm | 저장된 동일 쌍의 전 구간 최소 mm | 의미 |
|---|---:|---:|---|
| original02 | -0.1781 | +0.6244 | 부분 중심 진입 기록과 명목 검사 불일치 |
| visual02 | -0.0260 | +0.8690 | 위와 같음 |
| gutter02 | -0.0260 | +0.8690 | 위와 같음; 첫 위반 대상은 거터가 아니라 Rachis |
| gutter09 | -0.4116 | +0.0601 | 명목 거터 교차, 저장된 선택 쌍에서는 겹침 미관찰 |
| stem01 | -0.3220 | -1.1831 | 동일 쌍의 이후 겹침도 저장 상태에서 관찰 |
| stem02 | -0.4099 | -0.4049 | 동일 쌍의 이후 겹침 관찰 |
| stem09 | -0.1827 | -0.0219 | 동일 쌍의 이후 겹침 관찰 |

위 값은 `results.json`의 `first_violation`과 `comparison`에서 읽었다. 최초 음수 거리는 최대 침투량이 아니다. 선택된 한 쌍의 최소 거리와 모든 주줄기 접촉의 최대 침투량도 서로 다르다. 특히 stem02/09의 전체 주줄기 최대 침투는 각각 약 1.492/1.621mm이며, 표의 선택 쌍 값과 혼합하면 안 된다.

저장 상태는 최대 30fps이며 최초 위반 시점과 가까운 저장 시점의 차이는 최대 약 16.7ms다. 이 차이를 제거하지 않은 관절 비교를 순수 추종 오차라고 단정할 수 없다. 작은 음수 거리의 정확한 크기는 이미지 픽셀로 검증할 수 없다.

`fresh_success_diagnostic.json`의 gutter02는 240Hz, 41.85초, 10,044스텝 재생에서 특정 쌍의 최소 거리 +0.866435mm, 물리 유효성 기준 통과를 기록한다. 그러나 새 중심 진입 재분류·꼭지걸림 판정·힘 기록은 없다. 이것을 새 수확 성공이나 전 물체 무접촉으로 해석하지 않는다.

보고서에는 실행 보류 연결 검증이 서술돼 있지만, ZIP 안에는 언급된 `gate_results.json`과 gate 원출력 폴더가 없다. 원본 MJB·reference·trace·states도 포함되지 않았다. 따라서 보고된 서버 결과를 검토한 것과 해당 실험을 독립 재실행한 것은 구분한다.

## 3. 소스로 새로 확인한 사항

### 3.1 Rachis_05는 별도로 생성한 송이 중심축 분절이다

`build_glb_physics.py`는 다음을 따로 생성한다.

- `Rachis` → `TRUSS_Rachis_00...13`.
- 각 열매의 `Pedicel_proximal_XX`.
- 각 열매의 `Pedicel_distal_XX` → `Attachment_XX` 계열의 별도 바디.
- Tomato_02의 distal anchor는 생성 규칙상 `Attachment_01`.

따라서 코드 정의상 `glb_col_TRUSS_Rachis_05`는 Tomato_05 또는 Tomato_02의 꼭지를 뜻하지 않는다. 서버의 실제 MJB가 이 생성물인지 해시와 reference로 마지막 대조는 필요하지만, 의미 자체를 처음부터 추측하거나 사람이 새로 라벨링할 필요는 없다. 이 분절을 자동으로 허용 접촉에 넣지 않는다.

근거:
https://github.com/hjpark-rdv/tomato_robot_sim/blob/da04a0dbb16a2b6c661beb8f5ffff1bac355689b/mujoco-benchmark/scripts/build_glb_physics.py

### 3.2 양의 거리와 접촉력은 동시에 존재할 수 있다

MuJoCo 3.13.0의 공식 문서는 접촉 쌍의 margin을 힘 활성화 영역, margin+gap을 접촉 탐지 영역으로 설명한다. 따라서 오래된 버전의 공식을 기억해서 적용하거나, `contact.dist > 0`만 보고 힘이 없다고 판단하면 안 된다.

이번 native fixture는 명시적 pair margin=0.5mm, gap=0.5mm에서:
- 거리 +0.25mm: 활성 제약 및 양의 법선 힘.
- 거리 +0.75mm: 비활성 contact 기록 및 0인 접촉 힘.
두 경우를 MuJoCo 3.13.0과 3.14.0에서 각각 확인했다. 이 숫자는 작은 테스트 모델 값이며 온실 관측값이 아니다.

프로젝트 `convert_model.py`의 기본 geom에는 margin=0.5mm, gap=0.5mm가 들어간다. 이 설정이 현재 MJB에 어떻게 상속·혼합됐는지는 실제 geom/pair, contact.includemargin, efc_address, force를 확인해야 한다. 동일 우선순위 geom의 합산 margin이 적용된다면 양의 간격에서도 힘이 발생하는 설명이 가능하지만, 이를 gutter02의 확정 원인으로 선언하지 않는다.

근거:
https://github.com/google-deepmind/mujoco/blob/3.13.0/doc/computation/index.rst
https://github.com/hjpark-rdv/tomato_robot_sim/blob/da04a0dbb16a2b6c661beb8f5ffff1bac355689b/mujoco-benchmark/scripts/convert_model.py

### 3.3 현재 진단 콜백은 거리와 접촉의 계산 시점이 다를 수 있다

현재 RobotEngine은 `mj_step → mj_kinematics → on_step` 순서다. implicitfast에서는 스텝 내 동역학 계산 후 상태·시간이 적분되고, mj_kinematics는 새 위치를 갱신하지만 접촉/제약 계산 전체를 다시 수행하지 않는다. 따라서 callback의 geomDistance는 새 상태, 기존 contact/efc_force는 직전 solve 상태일 수 있다.

이번 native fixture에서도 이 순서에서 거리 0.0101m와 기존 contact.dist 0.0100m가 동시에 존재함을 확인했다. private MjData 전체 복사 후 mj_forward하면 같은 상태의 거리와 contact가 일치했고, 원래 데이터 및 다음 스텝 결과가 변하지 않음을 확인했다. 테스트에는 사용자 콜백/엔진 플러그인이 없다.

이는 “기존 모든 로그가 무효”라는 뜻이 아니다. 기하 거리와 접촉력을 같은 시점이라고 비교하거나 접촉력을 원인으로 지목하기 전에 시점을 맞춰야 한다는 뜻이다. private forward의 힘은 복사된 상태에서 재계산한 값이며 이전 실제 solve의 힘과 구분해야 한다.

근거:
https://mujoco.readthedocs.io/en/stable/programming/simulation.html
https://github.com/hjpark-rdv/tomato_robot_sim/blob/da04a0dbb16a2b6c661beb8f5ffff1bac355689b/mujoco-benchmark/scripts/robot_engine.py

## 4. 앞서 추가한 코드의 실제 결함과 이번 수정

기존 코드에서 재현한 문제:
1. `--all-violations`가 예산 초과로 중단돼도 `full_path_collected=true`.
2. 충돌을 이미 찾고 이후 중단됐는데 최종 status가 inconclusive로 남음.
3. 상세 목록이 64개에서 잘리지만 생략 여부와 실제 관찰한 고유 쌍 수가 없음.
4. 64개 뒤의 거터 충돌은 class hit 수에는 남아도 해당 쌍/자세 증거를 목록에서 찾지 못할 수 있음.

수정:
- 검사를 요청한 범위와 실제 완료 여부를 분리.
- 발견한 충돌은 blocked로 유지하고, 별도로 `complete=false`와 `scan_stop_reason` 기록.
- 관찰 고유 쌍/단계 수, 상세 저장 수, 생략 수, 잘림 표시.
- 상세 목록 제한과 무관하게 환경 종류별 최초/최악 위반 기록 보존.
- 저장된 쌍의 최초 거리와 관찰된 최소 거리·시각을 구분.
- namespaced GLB 이름도 진단 분류에 포함. 접촉 허용 정책에는 영향 없음.

기본 gate는 첫 위반에서 즉시 중단하는 기존 동작을 유지한다. clearance, sampling, 물리 모델, actuator, 기존 성공 라벨은 변경하지 않았다. 새 메타데이터는 기존 정책 스냅샷 해시를 바꾸지 않는다.

주의: `complete=true`와 `details_truncated=true`는 함께 가능하다. 전체 경로 검사는 완료했지만 상세 JSON 목록은 제한된 상태다. 종류별 요약과 생략 수를 함께 봐야 한다. sample_hits는 물리 접촉 횟수가 아니라 검사된 물체 쌍×샘플의 위반 수다.

gutter02는 기존 첫 위반 시점까지 이미 1,354,087/2,000,000 distance query를 사용했다. 전체 41.85초 진단은 기본 예산을 초과할 가능성이 있다. 아직 새 전체 진단을 서버에서 실행하지 않았으므로 실제 완료/비용은 미측정이다. 완료가 필요하면 진단용 복사 설정의 계산 예산을 명시적으로 늘릴 수 있지만, clearance나 샘플 간격을 완화해서 통과시키지는 않는다.

## 5. 이번에 실제 수행한 검증

코드 커밋: `1f5bc1ff610fd8bb0b1897fcba8d6b5c51351708`.

- 작성 컨테이너: 새 analytic 테스트 11개 통과, MuJoCo 미설치로 native 3개 skip.
- GitHub Actions Python 3.11 / MuJoCo 3.13.0: 전체 62개 통과, 0.90초, skip 없음.
- GitHub Actions Python 3.11 / MuJoCo 3.14.0: 전체 62개 통과, 0.73초, skip 없음.
- 위 시간은 테스트 함수 실행 시간이며 설치·체크아웃·온실 시뮬레이션 처리시간이 아니다.
- 기존 전체 48개 회귀 테스트도 포함한다. 사용자 온실 모델·새 GUI·수확 성공률은 이 검증에 포함되지 않는다.

실행 로그:
https://github.com/hjpark-rdv/tomato_robot_sim/actions/runs/36320778153

## 6. 다음 연구 방향: 진단을 짧게 마무리하고 고리걸기로 돌아간다

권고 순서는 다음과 같다.

1. 7개 경로의 전체 진단에서 완료 여부·잘림을 확인하고 후속 금지 위반을 파악한다.
2. gutter02와 stem01 같은 2개 대표 사례에서 동일 시점 거리/접촉/힘을 확인한다.
3. 이미 접근 가능한 소수 타깃에서 중심 진입과 꼭지 포획·유지를 분리해 평가한다. 기존 Isaac retained_geometry/retained_hook 로직의 재사용 가능성을 먼저 확인한다.
4. 기존 경로 생성기 주변에서 실제 지원하는 위치·기울기·상승 방향부터 제한적으로 탐색한다. 성공 판정이나 물리를 느슨하게 바꾸지 않는다.
5. 좁은 성공 각도 하나보다 작은 위치·각도 교란에서도 유지되는 걸림을 우선한다. 초기에는 SIM GT를 교사와 판정에만 사용하며 이것을 sensor-only 성공이라고 표현하지 않는다.
6. 운영 장면의 누락된 주변 충돌체는 반드시 공개한다. 다만 106개 송이 전체 탄성화가 끝나야 고리걸기 연구를 시작할 수 있는 것은 아니다. 별도 국소 연구 장면과 원래 전체 장면의 결과를 구분하고, 전체 장면 성공을 주장할 때는 경로 관련 장애물의 충돌 표현과 재검증이 필요하다.

“데이터를 더 모으면 된다”, “RL reward를 계속 바꾸면 된다”, “검사기만 통과시키면 된다”는 어느 것도 이번 결과만으로 지지되지 않는다. 우선 목표는 검증 가능한 실제 걸림 동작을 소수 장면에서 확보하는 것이다.

## 7. 남아 있는 미확인 사항

현재 MJB의 정확한 혼합 margin/gap, 같은 시점 접촉력, 전체 경로 통과 후 물리 결과, 실제 고리 포획/유지 결과, 센서 입력만으로 선택한 동작의 실물 성능은 아직 미검증이다. 원격 GitHub 기본 브랜치를 이번 작업에서 바꾸지 않았다. ZIP에 적힌 서버 로컬 mjlab-performance의 fast-forward와는 별개다.
