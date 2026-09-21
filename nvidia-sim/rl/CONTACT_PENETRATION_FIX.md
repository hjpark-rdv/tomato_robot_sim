# 49번 고리·가는 줄기 관통 수정 (2026-09-21)

## 변경

GPU dataset 기본 preset을 `contact120`으로 변경했다. 120Hz 물리 / 60Hz 제어 /
PGS 64 position·4 velocity iterations / plant armature 5e-4를 사용한다.
기존 practical60에서 시간 간격만 절반으로 줄였다. `practical120`의 관성 3e-4와는 다르다.
CAD·collider 크기, 줄기 강성·감쇠·질량·마찰, 파손 임계값, 로봇 관절 명령은 유지한다.
실물 물성 보정이나 모든 경로의 정확성을 보장하는 설정은 아니다.

```bash
./nvidia-sim/run_gpu_candidate_dataset.sh \
  --physics-preset contact120 --trajectory-mode staged6d \
  --num-envs 4 --candidates 64 --candidate-indices 0,1,5,49 \
  --planning-workers 4 --schedule continuous --representative-videos
```

기존 데이터에 이어 쓰지 말고 새 폴더로 실행한다. 설정/소스 fingerprint가 다른
resume는 거부한다. practical60은 비교용으로 명시해서 실행할 수 있다.

## 원인과 비교

기존 49번은 고리 segment_22와 TRUSS_Rachis_08 사이의 접촉이 보고되면서도
최대 1.793mm 겹친 뒤 반대쪽으로 나왔다. 충돌체 누락이 아니라 현재 모델과
solver·시간 간격 조합에서 접촉 제약이 충분히 유지되지 않은 현상이다.
60Hz 정지/단순 걸림 시험 통과만으로 이 복잡한 로봇 접근도 유효하다고 볼 수 없었다.

동일한 저장 관절 명령을 native GPU physics에서 재생하고 매 물리 스텝마다
현재 USD capsule/sphere의 표면 간격을 측정했다. 허용 겹침은 사전에 0.5mm로 정했다.
이는 수치 오류 선별 기준이며 재료의 실제 압축량을 뜻하지 않는다.

| 비교 설정 | 최대 겹침 | 판단 |
|---|---:|---|
| 기존 60Hz 64/4 | 1.793mm | 관통 재현 |
| 60Hz + speculative CCD + contact offset 1mm | 1.591mm | 다른 pedicel 관통, 미채택 |
| 60Hz TGS | 해당 없음 | 시작 1스텝에 연결부 파손, 미채택 |
| 60Hz 128/16 + offset 2mm | 0.430mm | 통과하지만 계산·접촉 설정 변화가 큼 |
| 60Hz arm drive stiffness 200 | 1.897mm | 관통, 미채택 |
| 60Hz 64/16 | 1.517mm | 관통, 미채택 |
| 120Hz 32/4 | 0.584mm | 선별 기준 초과, 미채택 |
| **120Hz 64/4** | **0.196mm** | 과도 변위 중단까지 관통 선별 통과 |

실패 설정은 진단용 1,150 제어 스텝 제한이 포함될 수 있다. 이를 완전 시험이나
일반적인 miss로 해석하지 않는다. 초기 비교는 여러 프로세스를 병행했으므로
wall time으로 설정별 처리 속도를 비교하지 않는다.

## 자동 제외 장치

`tool_contact_audit.py`는 현재 USD 형상과 native body pose를 사용한다.
고리 32개 arc·2개 rail과 식물 capsule/sphere를 매 물리 스텝 검사한다.
0.5mm 넘는 겹침이 발생하면 `invalid_physics`로 중단하고 다음 값을 저장한다.

- `physics_valid: false`, `exclude_from_valid_trajectory_analysis: true`
- `physical_audit.json`: 가장 깊은 겹침의 시점, 고리/식물 collider 경로
- `tool_contact_trace.json`: 물리 스텝별 최소 간격
- CSV의 `physics_valid`, `max_tool_penetration_mm`

관통 후보는 성공/부분 성공 대표 영상과 유효 물리 시험의 성공률 분모에서 제외한다.
`execution_complete`는 전체 후보 처리 완료, `dataset_complete`는 incomplete/invalid가
없는 경우에만 true다. 오류 행도 삭제하지 않아 재검토할 수 있다.
검사 이력은 매 candidate마다 초기화한다. 이 장치는 물체 위치를 투영하거나
숨은 반발력을 더하지 않는다. 관통 방지용 물리 설정과 별개의 데이터 품질 검사다.

범위 제한: mesh 잎/그리퍼 본체의 정확한 충돌 검증 및 스텝 사이 연속 swept 검사는
포함하지 않는다. 저장 영상은 15Hz 표본이지만 판정 검사는 매 물리 스텝이다.
native replay 일치와 물리적 유효성은 별개다.

## 참고 자료

- [NVIDIA 107.3 Collision Behavior Guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/collision_guide.html): contact offset, speculative CCD와 시간 간격 조정.
- [NVIDIA Articulation Stability Guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/dev_guide/guides/articulation_stability_guide.html): 구동/접촉 제약, 시간 간격과 반복 횟수의 영향.

최신 109 문서의 SolveArticulationContactLast 기능은 설치된 107.3 schema에서
확인되지 않아 적용하지 않았다. 지원되지 않는 USD 속성으로 해결했다고 주장하지 않는다.

진단 원본: `runs/20260921_contact_fix/`.

## 실제 기능 검사와 영상

- [49번 수정 전후 영상](runs/20260921_200605_candidate49_contact_fix_2x/index.html):
  두 관찰 각도, 2배속. 수정 후 1280×612 / 30fps / 279프레임, 전체 디코드 확인.
  삽입 중 변위 한계에 도달하므로 상승 완료나 수확 성공으로 표시하지 않는다.
- 120Hz 기능 fixture: 정지 안정, 0.2N 하중의 휘어짐/복원, 6N 과부하의 native
  파손, 리셋 복원 통과. 최대 과실 변위 18.10mm → 하중 제거 4초 후 잔류 0.327mm.
  접합부 최대 틈 0.030mm. 미리 끼운 고리의 네 roll 방향 모두 1초 접촉 유지 통과.
  이 fixture 통과는 로봇의 접근 성공을 뜻하지 않는다.
- 순차 단독 실행 시간(초기화/렌더 제외, pose 저장·매 스텝 검사 포함):
  기존 60Hz guard 실행 45.17초 / sim 16.183초,
  수정 120Hz 84.65초 / sim 16.583초.
  sim 시간으로 보정하면 약 1.83배 비용. 종료 조건/시점이 달라 전체 동일 길이
  시험의 정확한 배속은 아니며, 64환경 처리량으로 외삽하지 않는다.
- 원래 60Hz+guard는 16.183초에 0.526mm 겹침을 검출해 `invalid_physics`로
  중단했다. 단순히 정상 miss로 저장하지 않는 것도 실제 GPU 실행으로 확인했다.

기계 판독 요약: `validation/contact_penetration_fix.json`.

4환경에서 후보 0·1·5·49를 동시에 끝까지 실행했다. 새 경로 계산의 관절 명령은
기존 저장 명령과 모두 정확히 일치했다. reset/clone/RGB-D 검사와 결과/CSV 저장 통과.
별도 기능 시험과 병행한 실행이므로 이 실행의 433초를 독립 처리량 벤치마크로 쓰지 않는다.

| 후보 | 결과 | 최대 겹침 | 중심 진입 부분 성공 |
|---|---|---:|---|
| 0 | miss | 0.247mm | 없음 |
| 1 | excessive_displacement | 0.296mm | 없음 |
| 5 | miss | 0.348mm | 0.833초 |
| 49 | excessive_displacement | 0.196mm | 없음 |

4개 모두 실행 완료, `invalid_physics` 0, 완전 걸림 0이다. 전체 64개 재검사는 아니다.
추가 4환경 초기화 검사에서 한 clone만 순회해 추출한 124개 형상/로컬 끝점이
기존 전체-stage 추출과 정확히 같은 것도 확인했다. 자동 테스트 37개 통과.
