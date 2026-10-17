# Newton GPU 한쪽 베드 연속 수확 속도 개선

2026-10-18. 결과 원자료는 `/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed`에 보존한다. 아래 수치는 실제 완료 실행이며, 조기 중단한 실행의 빠른 구간을 완료 성능에 섞지 않는다.

## 검증된 결과

같은 쪽 베드 Plant22 → Plant27 → Plant29를 이동·접근·두 번 닫기/열기·후퇴까지 완료했다. 반대편 Plant41은 제외했다. 기존 집단 선택기·계획기·Newton GPU 실행기와 실제 저장 상태의 영상 생성기를 재사용했다.

| 대상 | 기존 3식물 탄성: 전체 RTF | 현재 타깃만 탄성: 전체 RTF | 기존 벨트 구간 RTF | 현재 벨트 구간 RTF | 현재 가장 느린 유지 구간 RTF | 새 열매 분리 |
|---|---:|---:|---:|---:|---:|---:|
| Plant22 | 1.071 | 2.298 | 0.576 | 1.310 | 0.859 | 6 |
| Plant27 | 1.148 | 2.582 | 0.501 | 1.212 | 0.836 | 5 |
| Plant29 | 1.087 | 2.346 | 0.504 | 1.191 | 0.814 | 6 |

현재3구간은 합계191.816895초의 물리를79.461427초에 계산했다(2.413962RTF). 기존3식물 탄성 실행 대비 루프 속도비는2.1865이지만, 아래의 물리 표현 변경을 포함한 비교다. 동일 모델의 솔버 단독 가속 수치가 아니다.

분리는 총17개(색상상 익음16·미숙1)이며 잔여13개다. 기존3식물 탄성 대조군은18개였다. 이번 실행에서 P27 Fruit02가 추가로 남았다. 미숙 과실 선택성, 전체 송이 수확, 포집과 실물 손상 한계가 검증됐다는 뜻은 아니다. `training_eligible=false`, `harvest_success=null`을 유지한다.

**아직 모든 접촉 세부 구간이 실시간 이상은 아니다.** `belt_finish`는0.814~0.859RTF이며, 벨트 전체 평균과 구분해야 한다. 기존3구간 실행의 실제 setup 합395.443초도 루프와 별개다. 준비·로딩을 포함한474.904초를2.414배속이라고 부르지 않는다. 이후 시작 비용 개선은 별도 A/B로 기록하며 이 과거 측정값을 소급 수정하지 않는다.

## 물리 표현과 상태 연결

기존379DOF 공통 모델은 세 식물을 동시에 탄성 계산했다. 새 opt-in `--active-window`는 현재 타깃 식물만 탄성으로 두고 다른 식물은 마지막 측정 자세의 고정 장애물로 남긴다. **주변 충돌을 꺼서 통과시키는 방식이 아니다.** body357·geom2244·부착30개와 주변6그루의1150충돌체를 유지한다. 전체 농장 USD 화면도 유지한다.

분리된 열매는 타깃이 바뀌어도 계속 free body다. 따라서 자유도는133 →169 →199로 증가한다. 이전6개/11개 분리 열매가 다음 구간에서도 실제 이동한다. 고정 식물의 부착은 `eq_active=true`를 유지하고 힘·토크는0이다. MuJoCo CPU는 생략하는 static-static CONNECT 행을 MuJoCo Warp가 생성하던 차이를 정확한 고정 attachment ID에 한해 제거했다. 그 전의 비정상 force 실행 `active133_baseline`은 유효한 성능 결과에서 제외했다.

`newton_window_continuation.py`는 기존 same-MJB resume 검증을 먼저 통과한 실제 종료 상태를 공통 은행에 올리고, 새 모델의 joint 이름·주소로 내려준다. qpos/qvel/eq/ctrl/damping/preload/warmstart/time과 누적 분리 이력을 검증한다. 원본 SHA 검사를 우회하거나 열매를 초기 부착 상태로 복원하지 않는다. 두 경계의 실제 물리 시각 차이는0이며 CPU 재구성 pose 오차는2.22e-16m, GPU float32 표시 pose 경계 오차는0.477/0.954µm였다.

구간마다 모델/solver와 접촉 캐시는 다시 생성한다. 완전히 단일 solver를 중단 없이 사용하는 구현은 아니다. 고정 식물의 저장 속도는 적분하지 않고 보존하므로, 재방문 때 과거 속도를 그대로 되살리는 문제는 후속 검증 대상이다. 이번 경로는 편도 방문만 검증했다.

## 지킨 접촉 조건

기본 감시1mm, 정확한 현재 타깃 belt↔fruit/pedicel은 기존 `close / cycle_reclose / belt_finish / cycle_open / cycle_open_final`에서2mm다. 완전히 열린 hold와 접근·이동은 이 예외 밖이며 기타 쌍은 belt_finish에서도1mm다. 마찰·강성·margin/gap·actuator와 분리3N/0.008Nm 조건은 바꾸지 않았다. 잎의 기존 비충돌 설정과 분리 과실 예외를 유지한다. 분리 과실↔정확한 gutter의 충돌은 계속 계산하되 기존 `--released-gutter-diagnostic` 중단 예외를 명시한다.

기존379DOF의 원본 MJB SHA는 `55159876ddeb6250ffea7bccb102a2e31dbf8d41556a17140454279e5ded0cc9`다. 현재 타깃을 고정/해제하면서 새 MJB SHA가 생기는 것을 정상적인 모델 변경으로 기록한다. Plant22의 원래 명령은 같지만 이후 두 계획은 각각의 실제 측정 상태에서 다시 계산하므로 이전379DOF 경로와 명령 SHA가 다르다.

## 추가 시험과 채택 기준

- 같은379DOF 첫 타깃에서 Hessian 용량128→256: 완료 RTF 약1.05→1.56. 그러나 두 번째 타깃은 다른 판·과실 접촉으로 중단해 이 설정만으로 전체 경로를 해결했다고 하지 않는다.
- active-window와128/256 두 용량 사용: 현재 검증된3구간 완주 설정이다. 초과 시 원래 솔버 fallback도 남긴다.
- 180Hz: 첫 타깃은7개 분리·최저1.035RTF지만 두 번째는 L/part147↔P27 Fruit08로 중단했다. 원본 삼각형도 교차하므로 허위 hull 충돌로 제거하지 않았다. 160/200Hz도 중단해 기본220Hz를 유지한다.
- block curvature, line-search block64, 다른 broadphase, AABB, 작은 contact/constraint capacity, solver tolerance, cuSOLVER64를 개별 비교했다. 완료·중단과 물리 결과를 각각 보존한다.
- solve-local Jacobian cache: 배열/접촉/분리 결과는 기준과 완전히 같고 rebuild77% 감소했지만 실제 시간은28.004초로 기준27.938초보다 짧지 않았다. 기본 채택하지 않는다.
- 정확히 같은 Hessian 분해 결과 cache:18.82% 재사용, 배열 결과 완전 동일. 실제28.276초로 더 느려 기본 채택하지 않는다.
- 169차원 synthetic Hessian의 실제 CUDA 동기화 마이크로벤치(수확 접촉 행렬 시험과 구분): cuSOLVER32가0.107ms로 최속. N=1 batched0.286ms, cuSOLVER64 0.702ms, Warp block32/64는 각각0.231/0.216ms였다. 모든 수치 대조는 통과했지만 빠른 대안이 아니어서 실제 수확 기본값을 바꾸지 않는다. Warp32/64는 block 크기이며 둘 다FP32다.
- R/part239의 single hull이 원본 빈 공간을 메운 사례는 원본 PhysX 기본16조각으로 별도 변환했다. LS32는완료7개지만 속도 이득은 없고, LS64는 다른 R/part100의 실제 표면 교차로 중단했다. 원본 결과와 새 모델을 구분하며 현재 검증된3구간 모델을 몰래 교체하지 않는다.

전체 개별 결과는 `experiment_metrics.json`, `INDEPENDENT_THREE_STATION_AUDIT.json`, `active169_cache_exact_comparison.json`, `factor169_compare.json`과 각각의 run/summary·입력·코드 해시에 있다.

- CCD block64→32도 실제 수확 배열·접촉·분리 결과가 정확히 같았다. clean 실행 RTF2.582→2.596, 최저0.836→0.841로 단회 약0.6% 차이여서 기본64를 유지한다. 프로파일의 벨트 평균에서는 충돌1.666ms(그중 convex1.225ms), solve3.955ms가 남았다.

- 마지막 `belt_finish` 2초만220→160/180Hz로 낮추는 별도 방법도 실제 Plant27에서 시험했다. 외부 명령·시간·phase와 구간 끝은 그대로 유지했다. 160Hz는 유지 구간 약1.07RTF지만 fixed/part_86↔P27_Rachis_13이1.001555mm에 도달해 중단했고,180Hz도 기타 쌍1.055245mm로 중단했다. 두 결과를 완료 처리량에 섞지 않으며 `--belt-hold-hz`는 기본값None인 미채택 진단 옵션이다. 최초 위반 시점과 CUDA graph 배치 종료의 중단 시점을 원자료에서 구분한다.

## 화면과 실제 물리 결과

Isaac 표시에서 동일 pose의 불필요한 USD 쓰기·스키닝을 생략하고, 내부/외부 프레임 제한의 중복 대기를 제거했다. 전체 농장과 실제 변형 표현은 유지한다. 안정 구간 표시가6.76→14.83FPS로 개선됐다. pose/skin CPU 평균70.76→27.38ms, app.update67.22→17.05ms였다. 화면을 더 자주 갱신하며 GPU/CPU를 함께 쓰므로 기존GUI27.38초→수정GUI30.05초(약9.8% 증가)였다. 수정판의 별도 headless24.65초와 비교하면 GUI30.05초는 약21.9% 증가이며, 해당 쌍의 전체 RTF는2.313/1.897이다. 이는 별도 표시 A/B이며 위의 최고 처리량 측정과 섞지 않는다.

첫 Plant22 구간에서 동일 초기상태로 두 번 새 적분한 전체8,221저장 상태·힘·접촉·명령·solver iterations·분리 이벤트는 GUI/headless가 정확히 일치했다. 두 실행은 같은 GPU session을 reset해 재사용했다. 기존 GUI와 수정 GUI의 저장 상태도 같았다. 자동 카메라 이동 중에도 검증했고 final pose/time ACK가 일치했다. 표시331전송 중326반영·5개 중간 표시 생략은 latest-frame 표시 정책이며 물리 스텝 누락이 아니다. 로컬 표시 timestamp 측정이며 사용자의 VNC 네트워크 FPS 자체를 측정한 것은 아니다. 세 정차점 전체의 GUI 동등성이나 별도 cold-start 프로세스 비교까지 완료했다는 뜻도 아니다.

## 준비 비용

파일별 독립8개 reader, immutable 자산 디렉터리 공유, SHA로 검증하는 local MJB cache, 호출 안에서만 동일 실파일 해시·읽기전용 모델 재사용을 적용했다. 변경된 inode/크기/mtime/ctime/alias는 거부하며 호출 간 검증을 생략하지 않는다.

같은169DOF bank의 준비 CPU A/B는55.699→49.399초(11.3% 단축), rchar4.407→3.561GB였다. 생성된 XML/MJB/metadata/proof/commands는 바이트 동일하고 초기/은행 배열도 모두 같다. warm filesystem 단회 측정이며 cold start 평균은 아니다.

MJB721.45MiB 중 BVH가451.28MB다. 모든1269mesh가 참조돼 단순 미사용 mesh 삭제는 불가능하다. convex support 저장 표현을 줄이는 별도 후보는 CPU compile 메모리상 약210MiB까지 줄었으나 대칭 중심 양성 대조의 접촉이 사라졌다. COM/PCA 재정렬에 따른 GJK 퇴화 차이를 확인해 채택하지 않고 GPU 수확 시험도 하지 않았다. 전체 재컴파일 graph 정점 대조의35.47mm 차이는 공면 중간점 제거인지 형상 차이인지 미확정이다. 이를 실제 형상 오차라고 단정하지 않는다. 기본721MiB 모델을 유지한다.

## 실행

기존 검증된 전체 자산을 사용하는 한쪽 베드 실행:

```bash
cd /root/farmily_tomato
./nvidia-sim/run_s300_newton.sh row-realtime
```

새 timestamp 캠페인으로 생성하며 기존 결과를 덮어쓰지 않는다. 이 명령은 기존 `run_newton_row_harvest.py`를 호출한다. `--validate-only`는 실제 physics 없이 자산과 배치를 검사한다. 다른 준비된 경로는 `row-realtime /절대경로/route`로 명시한다. GUI는 이 row 명령에 자동으로 붙지 않는다.

단일 완료 구간을 같은 입력에서 새 GPU physics로 확인하려면 `gpu-realtime INPUTS --gui`를 사용한다. 후속 구간은 반드시 대응 `--window-transition`도 전달해야 하며, 저장 상태 영상과 새 물리 실행을 구분한다.

## 추적성과 한계

작업 브랜치는 `codex/s300-newton-active-window-20261018`, 시작 HEAD는 `c80f93ad1f6755934238397f6e950fed64553265`다. 원래 미커밋 상태는 결과 루트의 `starting_tracked_changes.patch`/status/head에 보존했다. gripper-design 등 사용자 별도 변경은 이 작업 커밋에서 제외한다. 큰 MJB·배열·영상은 NAS, 코드·작은 검증 요약·이 문서는 Git으로 보존한다.

한쪽 베드3방문의 유효한 물리 처리량은 개선됐지만, 모든 접촉 세부 구간≥1RTF·무대기 구간 전환·여러 GPU 환경·전체 익은 열매 완료를 모두 달성했다는 결론은 아니다. 원자료에 따라 남은 병목과 다음 대안을 분리한다.

## 최종 명령 한 번으로 재검증

모든 수정 후 `row-realtime .../route --campaign final_launcher_validation`을 정상 초기 상태부터 새로 실행했다. 세 방문 모두 완료했고, 기존 검증 경로와 각 MJB·명령·모든 저장 상태/힘/접촉 배열·분리 이벤트가 정확히 같았다. 새 부모→자식 인계 certificate도 모두 통과했다. 그래서 아래4개 실제 영상은 새 실행과도 동일한 상태를 보여주며, 새 physics를 영상 재생으로 대신 검증한 것이 아니다.

최종 재실행의 물리 루프는191.816895초/79.563791초=**2.410857RTF**, 각6/5/6개 분리였다. 실행기 setup 합은165.198407초로 앞선395.442841초보다 감소했다. 다만 NAS/파일 캐시가 warm인 후속 실행이고 별도 과거 측정이어서 순수 코드 개선비율이라고 단정하지 않는다.

**전체 사용자 대기는 여전히 크다.** 계획 준비103.442027초 + 모델 축소/이관검증218.517783초 + physics 프로세스 전체262.932513초 = 세 구간584.902343초(약9분45초)다. 이 합에도 루프 밖 최초 자산 검증·셸 시작은 별도다. 단계별 총경과에 대한 RTF는0.327947이며,2.41배속은 물리 루프만의 수치다. source process 시작 tick부터 종료 관측까지616.835초는 관측 지연을 포함한 상한값으로 별도 보존한다. 물리의 약3분12초를 시작부터 종료까지 실시간으로 처리하는 목표는 아직 달성하지 못했다.

근거: `final_launcher_comparison.json`, `final_launcher_comparison_KO.md`, 각 `orchestration_timings.json`. 기존 영상 재사용 자격은 배열 비교를 통한 상태 동일성으로 확인했으며 동일 USD 표시 조건을 전제로 한다.

## 시험과 다음 병목

최신 수정까지 test ID를 중복 제거한 native/asset 회귀는395PASS,25SKIP다. 25개는 CUDA opt-in21개·별도 farm snapshot1개·pxr와MuJoCo 동시 환경3개이며 통과로 세지 않는다. 표시 캐시 관련15개는 별도 환경에서PASS했다. 마지막 수정군66개를 다시 실행해 전부PASS했고 실행 중 코드 변경이 없었다. 실제 GPU 전체 실행·GUI 대조는 위 원자료로 별도 검증했다.

이번 결과에서 기본 채택한 것은 현재 식물만 탄성화하는 상태 연결, 정확한 고정 attachment 처리, Hessian128/256 용량 선택, 자산 검증/읽기 재사용, 화면 갱신 최적화다. CCD block/수학 연산 cache/일괄 Hz 인하/hold Hz 인하/충돌 자산 압축은 개선 부족 또는 물리 대조 실패로 채택하지 않았다. 실패한 후보를 숨기지 않고 전체 표에 남겼다.

다음 우선순위는 (1)구간마다 NAS 검증·재컴파일·solver를 새로 만드는 대기를 줄이는 상주 실행기와 상태 전환, (2)벨트 유지 단계의 실제 solver/convex 처리시간 감소다. 표시 FPS나 전체 평균을 올리는 것만으로 마지막 접촉 구간 문제를 해결했다고 하지 않는다. 추가 물리 표현 변경은 이번 결과와 따로 대조해야 한다.

## 실제 저장 상태 영상

아래4개는 검증된220/60Hz 세 방문의 실제 상태를 재생한1배속 영상이다. 렌더러는 physics를 다시 실행하지 않았다. 앞3개는 전체 농장 화면, 마지막은 같은Plant27 상태의 접촉 확인용 표시 단면(잎 외형 숨김)이다.960×720/6FPS이며 자막을 작게 표시했다. 낮은 영상FPS는 물리Hz와 다르다.

- [/mnt/nas_rdv_md3/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_baseline/step_00_plant22/physics/active_window_plant22_actual.mp4](/mnt/nas_rdv_md3/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_baseline/step_00_plant22/physics/active_window_plant22_actual.mp4)

- [/mnt/nas_rdv_md3/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_01_plant27/physics/active_window_plant27_actual.mp4](/mnt/nas_rdv_md3/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_01_plant27/physics/active_window_plant27_actual.mp4)

- [/mnt/nas_rdv_md3/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_02_plant29/physics/active_window_plant29_actual.mp4](/mnt/nas_rdv_md3/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_02_plant29/physics/active_window_plant29_actual.mp4)

- [/mnt/nas_rdv_md3/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_01_plant27/physics/active_window_plant27_actual_contact.mp4](/mnt/nas_rdv_md3/mujoko_debugging_data/20261018_031544_newton_active_window_speed/route/active_window_followup/step_01_plant27/physics/active_window_plant27_actual_contact.mp4)
