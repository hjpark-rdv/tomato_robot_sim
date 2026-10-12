# S300 MuJoCo 표시 품질 선택

2026-10-12 KST. 물리 기준 `3e37040`, 브랜치 `codex/s300-mujoco-belt-smoke-20261012`.

전체 농장·S300·리프트·RB5·벨트 그리퍼의 **저장된 실제 물리 상태**를
`fast / balanced / ultra`로 표시한다. 기존 `view_smoke.py`와 실행 셸을 확장했다.
품질 옵션은 재생·영상 저장에 적용한다. `test`의 실제 물리 실행 설정은 그대로다.

## 실행

```bash
cd /root/farmily_tomato

# 최대 표시 품질. replay의 기본값도 ultra다.
./mujoco-benchmark/run_s300_mujoco.sh replay --quality ultra

# 원본 표면과 PBR을 유지하면서 화면 비용을 낮춤.
./mujoco-benchmark/run_s300_mujoco.sh replay --quality balanced

# 기존 native OpenGL 표시.
./mujoco-benchmark/run_s300_mujoco.sh replay --quality fast
```

| 옵션 | 렌더러 | 기본 GUI | 기본 영상 | 내부 가로·세로 배율 |
|---|---|---|---|---|
| fast | 기존 classic OpenGL | native viewer | 960×720 | 1× |
| balanced | 공식 Filament PBR | 1280×720 | 1920×1080 | 1× |
| ultra | 공식 Filament PBR | 1920×1080 | 3840×2160 | 2× |

`ultra`의 4K 영상은 실제 7680×4320으로 렌더링한 다음 Lanczos로 축소한다.
GUI에서도 내부 3840×2160 → 1920×1080으로 표시한다. AO 최고 품질,
bent normals, SSCT, PCSS 그림자와 8192 shadow map을 선택한다.
`balanced`는 AO high, PCF 2048 그림자를 사용한다. 두 PBR 옵션 모두 +1EV 표시 노출과
PBR Neutral tone mapping을 사용한다.
공식 RGB RenderTarget은 MSAA를 꺼 두므로 MSAA/TAA 적용을 주장하지 않는다.

PBR GUI: 왼쪽 드래그 회전, 오른쪽 드래그 이동, 휠 확대/축소,
Space 일시정지, Escape 종료. 기록 timestamp를 1배속으로 표시한다.
렌더링이 느리면 표시 프레임이 줄며, 새로운 물리 계산은 수행하지 않는다.

다른 결과 폴더·구간·해상도도 기존 옵션으로 지정한다.

```bash
./mujoco-benchmark/run_s300_mujoco.sh replay /path/to/result \
  --quality balanced --start 52 --end 68 --width 1600 --height 900

./mujoco-benchmark/run_s300_mujoco.sh render /path/to/result \
  --quality ultra --camera aisle --start 52 --end 58 \
  --output /path/to/new_actual_ultra.mp4
```

직접 `view_smoke.py`를 호출하면 호환성을 위해 기본값 `fast`다.
셸은 기본값 `ultra`다. `S300_MUJOCO_QUALITY`로 셸 기본값을 지정할 수 있다.
`S300_MUJOCO_RENDER_DISPLAY` 기본값은 `:0`이며 해당 서버 VNC 화면을 사용한다.
기존 파일과 같은 출력 이름이면 덮어쓰지 않고 오류를 낸다.

## 복원한 외형과 표시 오류 수정

- 원본 USD의 5,082개 형상에 대응하는 base color·roughness·metallic을 복원했다.
  원본의 바인딩·값·해시와 누락 채널의 기본값을 표시 자산 메타데이터에 보존한다.
- 98개 원본 줄기 스킨, 실제 정점 86,624개와 삼각형 129,584개를 복원했다.
  측정된 바디 자세와 기존 skin의 bone/weight/rest 데이터를 사용한다.
- 원본에서 보이지 않던 충돌 프록시 186개는 표시에서 숨긴다.
  Filament가 material alpha로 geom alpha를 덮어쓰는 동작 때문에 투명 geom에
  불투명 material을 바인딩하면 충돌 hull이 흰 얼룩처럼 나타났다.
  해당 geom의 표시용 material binding을 제거했으며 충돌 mask는 그대로다.
- 원본 USD의 linear Rec.709 색상과 Filament 입력의 sRGB 인코딩 차이를 처리한다.
  원래 색상과 변환 결과를 함께 보존한다. 원본 파일의 색상을 임의로 바꾸지 않는다.
- 기본 Filament 환경광의 따뜻한 색조 대신 중립 환경광을 표시 자산에 추가했다.
  실물 온실의 광량을 계측한 조명은 아니다. balanced/ultra는 같은 조명을 사용한다.
- 실제 턱 간격으로 기존 CAD 크랭크·링크 계산을 호출해 15개 시각 부품의 자세를 복원한다.
  qpos/qvel이나 물리 충돌체의 자세를 수정하는 방법이 아니다.

MuJoCo 3.15 Filament의 indexed skin 처리에는 vertex/index count가 다른 경우
native 종료되는 문제가 있었다. 사용하지 않는 정점을 추가해 이를 회피한다.
원래 shared vertex, face, smooth normal, weight를 유지하며 삼각형 수는 늘리지 않는다.
스킨 padding 정점은 302,128개, 총 표시 정점은 388,752개다.
원본 물리 collision mesh·capsule은 바꾸지 않는다.
Filament의 PBR shader는 텍스처 연결이 없어도 UV0를 요구한다. 기존 UV는 유지하고,
UV가 없는 표시 mesh/skin에만 0 좌표를 추가했다. 원본의 현재 재질은 모두
연결된 texture가 0개라 이 좌표가 표면 색상을 바꾸지 않는다. 경고를 숨기는 방법이 아니다.
물리 파라미터 해시 v2에서는 표시 전용 `mesh_texcoordnum`도 제외한다.
이 때문에 v1의 해시 문자열과 다르지만 원본 MJB bytes가 바뀐 것은 아니다.
동일 v2 함수로 original/display의 물리 배열·옵션 일치를 확인한다.

또한 MuJoCo 3.15의 색보정 setter는 첫 호출에 이전 옵션을 적용했다.
같은 공식 Scene을 두 번 구성해 +1EV가 실제 적용되도록 했다. GPU 양성 대조에서
첫 구성의 EV0/EV1 평균은 같았고, 두 번째 구성부터 밝기가 증가했다.
원본 코드나 설치한 MuJoCo 라이브러리를 패치하지 않는다.
Filament용 `harvest` 카메라는 중립 환경광을 사용한다. classic용 가상 headlight는
PBR 조명 설정에서는 유효하지 않으므로 적용했다고 기록하지 않는다.

## 환경·캐시·물리 보존

PBR용 Python 3.12 환경은 별도 `.mujoco315-render-venv`다.
기존 native 물리 Python 3.11 `.mujoco315-venv`를 교체하지 않는다.
두 환경 모두 MuJoCo 3.15.0이며 설치 목록은 `requirements-render-lock.txt`에 고정했다.

```bash
./mujoco-benchmark/setup_s300_render.sh
```

첫 표시 캐시는 기존 USD 자산 Python으로 원본 재질과 skin을 추출한다.
그 뒤 일반 재생에는 USD Python이 필요 없다. 원본 MJB를 변경하지 않고 별도
source SHA·MuJoCo 버전·품질 설정에 묶인 표시 MJB를 저장한다.
캐시가 달라지거나 source hash가 맞지 않으면 기존 캐시를 유효하다고 취급하지 않는다.
원본 MJB/NPZ의 해시, physical parameter hash, 바디·관절·geom 순서를 확인한다.

첫 빌드 비용, 로딩 비용, GPU 표시 FPS와 기존 물리 RTF는 다른 측정량이다.
최대 품질의 GPU 렌더 비용은 커지지만 화면 없는 `test`에는 적용되지 않는다.
기존 물리 RTF 0.780을 품질 옵션의 GUI FPS나 새 물리 성능으로 바꿔 해석하지 않는다.

## 검증 결과

검증 수치와 실제 이미지·영상·로그는 다음에 보존한다.

- 공유 결과: `/root/docker_share/mujoko_debugging_data/20261012_s300_mujoco_quality/index.html`
- 전체 증거: 같은 폴더 `quality_validation.json`
- 작은 Git 요약: `validation/s300_mujoco_render_quality_20261012.json`

최종 측정은 RTX 3090 Ti / 드라이버 570.211.01 / 서버 DISPLAY=:0에서 각 1회 수행했다.

| 측정 | 결과 |
|---|---|
| balanced GUI | 1280×720, 81프레임 / 10.27초, **7.89 FPS** |
| ultra GUI | 데스크톱에 맞춘 실제 1850×1016, 37프레임 / 10.32초, **3.59 FPS** |
| fast GUI | native `viewer.sync()` 113회 / 5.02초, 22.53회/s; 실제 표시 FPS는 별도 계측 안 함 |
| balanced PNG | 1920×1080, 렌더 1.48초 |
| ultra PNG | 3840×2160 / 내부 7680×4320, 렌더 3.79초 |
| 표시 캐시 | V6 첫 추출·컴파일 19.35초, 재사용 설정 약 4.3초; 원본 MJB/NPZ 로딩은 별도 |
| 테스트 | native 66 + USD·기존 CAD 28 + 실제 GPU·카메라 7 = **101 PASS** |
| 최종 PBR 로그 | missing-required-attributes 경고 0 |
| 원본 skin 위치 재현 | 실제 원본 정점에서 최대 0.000592mm |

PNG 시간은 렌더·축소·자막·파일 저장을 포함하며 렌더러 생성은 제외한다.
GUI 수치는 첫 프레임 비용과 종료 비용을 포함한 짧은 서버 측정이다.
`ultra`는 기본 GUI1920×1080을 요청했으나 현재 데스크톱의 창 영역에 맞춰 줄었다.
위 FPS는 표시 비용이지 새로운 물리 RTF가 아니다. 셋 모두 기록 시간은 1배속이다.
최대 품질에서는 화면이 매끄럽다고 주장하지 않는다. native viewer의 sync 횟수도
실제 화면 표시 FPS와 같은 값으로 집계하지 않는다.

## 범위와 한계

이 변경은 표시 기능이며 수확 성공 판정을 바꾸지 않았다.
실제 저장 상태를 렌더링하고, 움직임을 다시 시뮬레이션해 영상에 대체하지 않는다.
원본 USD의 현재 대상에는 연결된 texture/normal map 채널이 없으므로 새 텍스처를
만들거나 원본에 없는 표면 디테일을 추가하지 않는다. 재질이 없던 CAD 부품의
기본 PBR 값도 실제 도장·금속의 계측값이 아니다.
RTX 경로 추적과 같은 렌더러·광학 결과를 보장하지 않는다.

공식 근거:
[MuJoCo Filament](https://mujoco.readthedocs.io/en/latest/programming/visualization.html#filament-rendering),
[USD 색상 공간](https://openusd.org/release/user_guides/color_user_guide.html).
