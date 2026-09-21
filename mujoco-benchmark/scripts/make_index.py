"""Portable HTML report and RGB-D previews alongside the saved benchmark."""
import argparse,html,json,shutil
from pathlib import Path
import numpy as np
from PIL import Image

def build(root,mujoco_rgbd=None,isaac_rgbd=None,viewer=None):
    root=Path(root);comparison=json.loads((root/'comparison.json').read_text())
    for engine,source in [('mujoco',mujoco_rgbd),('isaac',isaac_rgbd)]:
        if not source:continue
        source=Path(source);dest=root/f'rgbd_{engine}';dest.mkdir(exist_ok=True)
        for name in ('rgb.png','depth_m.npy'):
            shutil.copy2(source/name,dest/name)
        if engine=='mujoco':report=json.loads((source/'rgbd_report.json').read_text())
        else:
            report=json.loads((source/'summary.json').read_text())['results'][0]
            report['camera']=json.loads((source/'rgbd_camera.json').read_text())
        report['source_directory']=str(source.resolve());(dest/'report.json').write_text(json.dumps(report,indent=2))
        z=np.load(dest/'depth_m.npy');valid=np.isfinite(z)&(z>0)&(z<5)
        gray=(255*(1-np.clip(np.nan_to_num(z,nan=5,posinf=5)/.75,0,1))).astype(np.uint8);rgb=np.repeat(gray[:,:,None],3,axis=2);rgb[~valid]=[160,0,160];Image.fromarray(rgb).save(dest/'depth_preview.png')
    if viewer:shutil.copy2(Path(viewer)/'viewer_report.json',root/'viewer_report.json')
    lines=['<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Isaac / MuJoCo 식물 물리 비교</title>',
        '<style>body{background:#15202b;color:#e8edf3;font:16px/1.65 sans-serif;max-width:1320px;margin:30px auto;padding:0 24px}a{color:#8dcfff}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border:1px solid #435162;padding:7px;text-align:right}th:first-child,td:first-child{text-align:left}video{width:100%;background:black}article{background:#202d3b;padding:18px;margin:20px 0;border-radius:8px}.images{display:flex;flex-wrap:wrap}.images img{width:48%;object-fit:contain}small{color:#b9c9d7}code{background:#314156;padding:2px 5px}</style>',
        '<h1>원본 식물 + 고리: Isaac / MuJoCo 비교</h1>',
        '<p>같은 고리 궤적을 직접 재생한 CPU 물리 시험입니다. 로봇 암·IK·탐색은 제외했고, 양쪽 모두 파단을 껐습니다. 물리 단계의 속도와 수치적 접촉/부착 오차를 비교하며 수확 성공률을 측정하지 않습니다.</p>',
        '<p><a href="comparison.md">상세 비교표</a> · <a href="results.csv">전체 경로 CSV</a> · <a href="comparison.json">동일 입력/초기 상태/반복성 검사 JSON</a> · <a href="../../README.md">실행 안내</a></p>',
        '<h2>물리 측정</h2><p>RTF = 시뮬레이션 시간 ÷ 실행 시간. 실행에는 궤적 입력·상태/접촉 기록을 포함합니다. 전체 시간은 추가 관통 검사와 저장까지 포함하며 로딩·영상 생성은 제외합니다. 60Hz 원본은 모두 조기 종료했으므로 처리량 비교에 사용할 수 없습니다.</p>',
        '<div style="overflow:auto"><table><tr><th>실행</th><th>경로</th><th>sim 초</th><th>실행 초</th><th>RTF</th><th>전체 초</th><th>관통</th><th>부착 오차</th><th>불안정</th></tr>']
    for r in comparison['runs']:
        lines.append('<tr>'+''.join(f'<td>{v}</td>' for v in [html.escape(r['run']),r['count'],f"{r['sim_s']:.1f}",f"{r['replay_wall_s']:.1f}",f"{r['rtf']:.2f}",f"{r['total_wall_s']:.1f}",r['penetration_trials'],r['attachment_invalid_trials'],r['unstable_trials']])+'</tr>')
    lines+=['</table></div><p>관통/부착 오차 기준은 각각 0.5mm입니다. capsule/sphere 검사이며 모든 mesh에 대한 안전 인증은 아닙니다. Optimized는 잎/잘린 가지 충돌을 끄고 열매 weld를 합쳤으므로 원본과 같은 물리 조건이 아닙니다.</p>',
        '<h2>대표 영상</h2><p>실행 당시의 모든 강체 상태를 재생했습니다. 왼쪽은 식물 전체, 오른쪽은 접촉 확인을 위해 잎/잘린 가지 mesh만 숨긴 화면입니다. 실제 물리에서 충돌을 끈 것이 아닙니다(Optimized의 별도 변경은 영상에 표시). 노란 점은 Tomato_05 중심 GT입니다.</p>',
        '<p><code>contact_no_detected_penetration</code>: 접촉이 보고됐고 검사에서 관통을 검출하지 않음. <code>invalid_attachment</code>: 부착부 오차 초과. <code>invalid_penetration</code>: 관통 기준 초과. <code>unstable</code>: 수치 폭주. 어느 항목도 고리걸기 성공을 뜻하지 않습니다.</p>']
    videos=sorted(root.glob('*/videos*/*.mp4'))
    for file in videos:
        relative=file.relative_to(root);label=' / '.join(relative.parts[:-1])+' / '+file.stem
        lines.append(f'<article><h3>{html.escape(label)}</h3><video controls preload="none" src="{relative.as_posix()}"></video></article>')
    cameras={}
    for engine in ('mujoco','isaac'):
        path=root/f'rgbd_{engine}'
        if not (path/'report.json').exists():continue
        r=json.loads((path/'report.json').read_text());cameras[engine]=r
        lines.append(f'<h2>{engine}: RGB-D</h2><p>10초 구간: {r["wall_s"]:.2f}초, RTF {r["rtf"]:.2f}. 640×480, 15Hz. 사진은 진입 전 첫 구간입니다. 왼쪽 RGB, 오른쪽 optical-Z depth 미리보기(0m 흰색 → 0.75m 검정, 배경/무효값 자주색). 렌더러의 조명과 재질은 다릅니다.</p><div class="images"><img src="rgbd_{engine}/rgb.png" alt="{engine} RGB"><img src="rgbd_{engine}/depth_preview.png" alt="{engine} depth"></div><p><a href="rgbd_{engine}/depth_m.npy">깊이 원본(m)</a> · <a href="rgbd_{engine}/report.json">카메라/속도/검증 JSON</a></p>')
    if (root/'viewer_report.json').exists():
        r=json.loads((root/'viewer_report.json').read_text());lines.append(f'<h2>직접 화면 표시</h2><p>DISPLAY=:0 MuJoCo viewer: {r["simulated_s"]:.1f}초를 {r["wall_s"]:.2f}초에 처리했습니다(RTF {r["rtf"]:.2f}). GUI는 비동기 렌더링하므로 화면에 표시된 프레임 수와 sync 호출 수는 다를 수 있습니다. <a href="viewer_report.json">측정 JSON</a></p>')
    lines+=['<h2>해석 범위</h2><p>mocap과 PhysX kinematic target은 접촉 속도 처리 방식이 다릅니다. 구면 관절과 XYZ hinge, weld, 마찰 모델 역시 완전히 동일하지 않습니다. 원본 120Hz 100개와 240/480Hz 6개 fixture의 결과를 구분해서 보아야 합니다. 이 자료만으로 실물 성능이나 대규모 병렬 처리량을 보장하지 않습니다.</p></html>']
    (root/'render_comparison.json').write_text(json.dumps(cameras,indent=2));(root/'index.html').write_text('\n'.join(lines));print('[비교 화면 저장]',root/'index.html','영상',len(videos),'개')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--mujoco-rgbd',type=Path);p.add_argument('--isaac-rgbd',type=Path);p.add_argument('--viewer',type=Path);a=p.parse_args();build(a.root,a.mujoco_rgbd,a.isaac_rgbd,a.viewer)
