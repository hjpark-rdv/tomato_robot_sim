from pathlib import Path
import json,shutil,hashlib,subprocess,html
O=Path(__file__).parent;repo=Path('/root/farmily_tomato');E=repo/'mujoco-benchmark/validation/contact_limit_sweep_20260928'
E.mkdir(parents=True,exist_ok=True)
data=json.loads((O/'results.json').read_text())
for r in data['rows']:
 source=O/r['name'];dest=E/r['name'];dest.mkdir(exist_ok=True)
 for f in source.iterdir():
  if f.is_file():shutil.copy2(f,dest/f.name)
for f in O.iterdir():
 if f.suffix in ('.py','.json','.log','.png','.svg') and not f.name.startswith('render_samples_'):shutil.copy2(f,E/f.name)
V=E/'videos';V.mkdir(exist_ok=True)
for f in (O/'videos').iterdir():
 if f.suffix in ('.mp4','.json') or '_frame' in f.name:shutil.copy2(f,V/f.name)
body='''<html lang="ko"><meta charset="utf-8"><title>힘·변위 중단값 비교</title><style>body{font:16px sans-serif;background:#13202c;color:#eee;margin:25px}a{color:#9ee3ff}td,th{padding:8px;border:1px solid #607080}table{border-collapse:collapse}video,img{width:min(100%,960px)}p{max-width:1100px;line-height:1.6}</style><h1>힘·변위 중단값 비교 — 정상 시작 12회</h1><p>같은 저장 경로와 물리, 다른 중단값. 목표 과실/와이어 접촉만 허용. 비허용 접촉0.01N과 침투0.5mm 중단 기준 유지. 실물 안전/손상 기준 또는 수확 불가능 판정 아님. <a href="REPORT_KO.md">보고서</a> · <a href="results.json">원자료 요약</a></p><table><tr><th>실험</th><th>힘 N / 변위 mm 제한</th><th>완료·중단 / 시간</th><th>과실 힘 L/P N</th><th>최대 변위 mm</th><th>비허용 힘 N</th><th>안착 표본 / 유지</th></tr>'''
for r in data['rows']:
 body+=f'<tr><td>{r["name"]}</td><td>{r["force"]} / {r["displacement_m"]*1000:g}</td><td>{",".join(r["stop"]) or "completed"} / {r["simulated_s"]:.3f}s</td><td>{r["force_live_N"]:.3f} / {r["force_private_N"]:.3f}</td><td>{r["target_displacement_mm"]:.3f}</td><td>{r["forbidden_force_N"]:.3f}</td><td>{r["seated_samples"]} / {r["authorized"]["hold"]["seated_with_force"]}</td></tr>'
body+='</table><p>L/P는 live 이전 solve / private-forward 현재 상태의 접촉별 힘 크기 합이다. 양쪽을 합산하지 않는다. 영상 왼쪽은 명목 명령, 오른쪽은 실제 저장 상태다. 제목에 실제 중단 원인과 시점 표시.</p><img src="force_displacement.png">'
for p in sorted((O/'videos').glob('*.mp4')):body+=f'<h2>{p.stem}</h2><video controls preload="metadata" src="videos/{p.name}"></video>'
body+='</html>'
(O/'index.html').write_text(body);(E/'index.html').write_text(body)
validation=[]
for f in (O/'videos').glob('*.mp4'):
 subprocess.run(['ffmpeg','-v','error','-i',str(f),'-f','null','-'],check=True,stdout=subprocess.DEVNULL)
 info=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-show_entries','stream=nb_read_frames,width,height','-of','json',str(f)]))
 validation.append(dict(file=f.name,sha256=hashlib.sha256(f.read_bytes()).hexdigest(),full_decode=True,info=info))
for root in [O,E]:(root/'video_validation.json').write_text(json.dumps(validation,indent=2)+'\n')
print('packaged',E)
