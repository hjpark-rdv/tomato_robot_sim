from pathlib import Path
import json,gzip,shutil,hashlib,html
O=Path(__file__).parent
repo=Path('/root/farmily_tomato')
E=repo/'mujoco-benchmark/validation/target_fruit_contact_server_20260928'
E.mkdir(parents=True,exist_ok=True)
R=Path('/root/docker_share/mujoko_debugging_data/20260927_hook_seating_server/robot_under')
ids=['seating_00_under','seating_03_under','seating_04_under','seating_05_under','seating_06_under','slow_seating_00_under']
rows=[]
for cid in ids:
 s=json.loads((O/f'summary_{cid}.json').read_text());f=s['forces'];t=s['trial']
 row=dict(candidate=cid,audit_complete=s['audit']['complete'],audit_status=s['audit']['status'],remaining_collision=s['audit']['first_violation'],
  physics_executed=t['physics_executed'],completed=t['completed'],stop=t['stop_reasons'],time_s=t['simulated_s'],
  fruit_force_live_N=f['contact_categories_live']['target_fruit_touch']['max_force_sum_N'],fruit_force_private_N=f['contact_categories_private']['target_fruit_touch']['max_force_sum_N'],
  max_displacement_mm=s['target_displacement_m']['max_value']*1000,fruit_penetration_mm=s['target_fruit_penetration_m']*1000,
  all_contact_penetration_mm=s['guard_penetration_m']['max_value']*1000,
  forbidden_force_N=max(f[k]['forbidden_contact']['max_force_sum_N'] for k in f),
  seated_samples=s['geometric_seated_samples'],hold=t['authorized_contact_evidence']['hold'],verify=t['authorized_contact_evidence']['verify'],
  training_eligible=False,hook_success=None,video=f'videos/{cid}.mp4' if (O/f'videos/{cid}.mp4').exists() else None)
 rows.append(row)
 for prefix in (['audit_','trial_'] if not cid.startswith('slow_') else ['trial_']):
  source=O/f'{prefix}{cid}';dest=E/source.name;dest.mkdir(exist_ok=True)
  for file in source.iterdir():
   if file.is_file():shutil.copy2(file,dest/file.name)
 shutil.copy2(O/f'summary_{cid}.json',E/f'summary_{cid}.json')
 inp=E/'inputs'/cid;inp.mkdir(parents=True,exist_ok=True)
 source=(O/'slow_run' if cid.startswith('slow_') else R)/'candidates'/cid.removeprefix('slow_')
 shutil.copy2(source/'plan.json',inp/'plan.json')
 (inp/'trace.json.gz').write_bytes(gzip.compress((source/'trace.json').read_bytes(),mtime=0))
for name in ['tests_initial.log','tests_final.log','tests_planner.log','verification.json','verification.log','mapping_verified.json','video_validation.json','render_trials.py','verify_evidence.py','package_results.py']:
 if (O/name).exists():shutil.copy2(O/name,E/name)
shutil.copy2(O/'slow_run/retiming.json',E/'retiming.json')
shutil.copy2(R/'manifest.json',E/'source_manifest.json')
shutil.copy2(R.parent/'diagnostic_policy.json',E/'base_policy.json')
V=E/'videos';V.mkdir(exist_ok=True)
for f in (O/'videos').iterdir():
 if f.suffix in ('.mp4','.json') or 'frame' in f.name:shutil.copy2(f,V/f.name)
summary=dict(base_commit='804e40cec63bae8272e7d692a29673dc3451e1c6',branch='codex/server-target-fruit-contact-20260928',
 normal_start_physics_executions=6,original_executions=5,retimed_executions=1,maximum_budget=7,rows=rows,training_eligible=False,hook_success=None)
for p in (O/'results.json',E/'results.json'):p.write_text(json.dumps(summary,indent=2)+'\n')
header='<html lang="ko"><meta charset="utf-8"><title>목표 과실 접촉 재시험</title><style>body{font:16px sans-serif;background:#15202b;color:#eee;margin:28px}a{color:#9ed7ff}td,th{padding:9px;border:1px solid #607080}table{border-collapse:collapse}video{width:min(100%,960px)}p{max-width:1100px;line-height:1.6}</style>'
body=header+'<h1>목표 과실 접촉 허용 후 실제 SIM 재시험</h1><p>기존 5경로 전체 검사 통과. 실제 5회는 목표 힘 1N 초과, 동일 기하 감속 1회는 변위 5mm 초과로 중단. 꼭지 안착·유지 미확보. 다른 물체 접촉 허용 및 물성 변경 없음. 정상 시작 총6회/예산7회. 수확 불가능·실물 손상 판정 아님.</p><p><a href="REPORT_KO.md">결과 보고서</a> · <a href="results.json">전체 수치</a></p><p>힘 L/P: 이전 solve의 live / 현재 상태의 private-forward. 각각 접촉별 3D 힘 크기 합의 최대이며 서로 더하지 않음. 침투는 목표 과실/와이어에서 검출된 값. 전체 접촉 최대 침투는 초기 0.228mm(기존 0.5mm 기준 이내). 기록된 비허용 로봇·환경 접촉력 및 목표 꼭지 접촉력은 모두0. 미도달 hold/verify를 통과로 처리하지 않음.</p><table><tr><th>candidate</th><th>전체 검사</th><th>남은 충돌</th><th>실행/중단</th><th>힘 L/P N</th><th>최대 변위 mm</th><th>과실 침투 mm</th><th>꼭지 안착/유지</th><th>영상</th></tr>'
for r in rows:
 cid=r['candidate'];link=f'<a href="{r["video"]}">MP4</a>' if r['video'] else '—'
 body+=f'<tr><td>{cid}</td><td>완료·통과</td><td>미검출</td><td>실제 {r["time_s"]:.3f}s / {r["stop"][0]}</td><td>{r["fruit_force_live_N"]:.3f} / {r["fruit_force_private_N"]:.3f}</td><td>{r["max_displacement_mm"]:.3f}</td><td>{r["fruit_penetration_mm"]:.3f}</td><td>안착0 / 유지 미도달</td><td>{link}</td></tr>'
body+='</table><p>아래 영상은 왼쪽에 명령+초기 식물, 오른쪽에 실제 저장된 SIM 상태를 보여줌. 물리 성공 영상이 아니며 중단시점/원인 표시. 같은 카메라. 노랑: 목표 꼭지, 청록: 뒤쪽 와이어, 분홍: Rachis (표시용 색만 변경). 힘 자막은 private-forward 합, live 값은 JSON에 별도 보존.</p>'
for r in rows:
 if r['video']:body+=f'<h2>{r["candidate"]}</h2><video controls preload="metadata" src="{r["video"]}"></video>'
body+='</html>'
(O/'index.html').write_text(body);(E/'index.html').write_text(body)
print('packaged',E)
