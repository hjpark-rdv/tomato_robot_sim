"""Build human review pages from saved, complete campaign summaries only."""
from pathlib import Path
from collections import Counter
import json,html
root=Path(__file__).resolve().parent
repo=Path('/root/farmily_tomato')
load=lambda p:json.loads(p.read_text())
cohorts=[('smoke_v2','연결 smoke'),('pilot','원래 5종 파일럿'),('side_repair','측면 준비점/입구 수정 비교')]
lines=['# 다중 경로군 서버 파일럿 결과 (2026-09-28)','']
allrows={name:load(root/(name+'_summary.json')) for name,_ in cohorts}
new_physics=sum(sum(r.get('physics_executed',False) for r in allrows[n]['candidates']) for n in ('pilot','side_repair'))
if new_physics==0:
 lines+=['**여러 장면의 계획·전체 검사·집계까지 연결했지만, 새 경로군의 실제 실행은 확보하지 못했다.** 허용되지 않은 장애물과의 경로 충돌로 차단됐다. 기존 두 경로의 실제 완주로 실행기와 접촉 정책은 별도로 검증했다. 아래 표의 물리 실행0은 미실행이며 물리 실패 횟수가 아니다.','']
else:lines += [f'새 파일럿과 측면 비교에서 실제 물리 {new_physics}회를 실행했다. 단계별 결과는 아래 표와 원자료를 참조한다. 기하 후보를 수확 성공으로 승격하지 않는다.','']
lines += ['## 경로군별 결과','', '| 실험 | 경로군 | 제안 | IK+로봇검사 | 전체검사 완료/통과 | 물리 | 완주 | 기하 후보 | 접촉 유지 | 분류 |','|---|---|---:|---:|---:|---:|---:|---:|---:|---|']
for name,title in cohorts:
 for family,t in allrows[name]['families'].items():
  outcomes=', '.join(k.split(':',1)[1]+':'+str(v) for k,v in t.items() if k.startswith('outcome:'))
  lines.append(f"| {title} | {family} | {t['proposed']} | {t.get('ik_and_robot_check_pass',0)} | {t.get('full_audit_complete',0)}/{t.get('full_audit_pass',0)} | {t.get('executed',0)} | {t.get('completed',0)} | {t.get('geometric_any',0)} | {t.get('retention_evidence',0)} | {outcomes} |")
lines+=['','계획 열은 IK와 기존 로봇 자기충돌·속도 검사 모두 통과한 수다. 검사 미완료는 통과로 세지 않는다. 기하 후보는 실제 실행에 대해서만 집계하며, 정지/기하 판정 fixture나 명목 계획은 포함하지 않는다.','', '## 차단 단계와 타깃별 차이','']
for name,title in cohorts:
 b=load(root/(name+'_blockers.json'))
 lines += ['',f'### {title}','','| 경로군 | 최초 차단 단계(개수) | 최초 장애물 종류(개수) |','|---|---|---|']
 for family,p in b['first_blocked_phase'].items():
  c=b['first_obstacle_category'][family]
  lines.append('| '+family+' | '+', '.join(f'{k}:{v}' for k,v in p.items())+' | '+', '.join(f'{k}:{v}' for k,v in c.items())+' |')
lines+=['','| 장면/타깃 | 원래 분류 | 측면 수정 분류 | 원래/수정 물리 |','|---|---|---|---:|']
case_results={n:load(root/n/'results.json')['reports'] for n in ('pilot','side_repair')}
repair={(r['case']['scene_id'],r['case']['target']):r for r in case_results['side_repair']}
compare=[]
for old in case_results['pilot']:
 key=old['case']['scene_id'],old['case']['target'];new=repair.get(key);nrows=new['records'] if new else []
 fmt=lambda rs:', '.join(f'{k}:{v}' for k,v in Counter(r['outcome'] for r in rs).items())
 lines.append(f"| {' / '.join(key)} | {fmt(old['records'])} | {fmt(nrows)} | {old['physics_budget_charged']}/{new['physics_budget_charged'] if new else 0} |")
 assert old['physics_budget_charged']+(new['physics_budget_charged'] if new else 0)<=12
 if new:
  assert len(old['records'])+len(nrows)<=24
  a=load(root/'pilot/cases'/key[0]/key[1]/'run/candidates.json')
  b=load(root/'side_repair/cases'/key[0]/key[1]/'run/candidates.json')
  a=[x for x in a if x['family']=='side_mouth']
  oldlookup={x['candidate_id']:x for x in old['records']};newlookup={x['candidate_id']:x for x in nrows}
  for x,y in zip(a,b):
   xp=dict(x['search_parameters']);yp=dict(y['search_parameters']);yp.pop('robot_facing_mouth',None)
   assert xp==yp
   ro=oldlookup[x['candidate_id']];rn=newlookup[y['candidate_id']]
   compare.append(dict(scene=key[0],target=key[1],parameters_equal_except_optin=True,old_id=x['candidate_id'],new_id=y['candidate_id'],old_outcome=ro['outcome'],new_outcome=rn['outcome'],old_first=ro.get('first_violation'),new_first=rn.get('first_violation'),old_waypoints=x['pose_waypoints'],new_waypoints=y['pose_waypoints']))
(root/'side_repair_comparison.json').write_text(json.dumps(compare,indent=2)+'\n')
lines+=['','## 실제 경유점과 FK 다양성','','| 실험 | FK 확인 후보 | 서로 다른 관절명령/FK | 최대 위치 오차 mm | 최대 회전 오차 rad |','|---|---:|---:|---:|---:|']
for name,title in cohorts:
 f=load(root/(('smoke' if name=='smoke_v2' else name)+'_fk.json'))
 lines.append(f"| {title} | {f['checked']} | {f['unique_command_hashes']}/{f['unique_fk_pose_hashes']} | {max((r['max_position_error_m'] for r in f['candidates']),default=0)*1000:.6f} | {max((r['max_rotation_error_rad'] for r in f['candidates']),default=0):.8g} |")
 if not f['endpoint_tolerance_pass']:lines.append('**FK 허용 오차 초과 있음. 원자료 확인 필요.**')
lines+=['','해시는 이름이 아닌 저장 명령 배열과 실제 FK 위치·회전 배열에서 계산했다. 각 후보의 방위각/고도/roll/pitch/우회폭과 모든 endpoint 배열을 FK JSON에 저장했다. 측면 비교는 동일 파라미터 쌍마다 실제 경유점이 어떻게 바뀌었는지도 보존한다. 이것은 명목 FK 검사이며 물리 추종 성공 증거는 아니다.','', '## CPU 실행 시간과 메모리','','| 실험 | wall 초 | 최대 표본 프로세스 트리 RSS GiB |','|---|---:|---:|']
for n,_ in cohorts:
 p=root/(n+'.resources.json')
 if p.exists():
  d=load(p);lines.append(f"| {n} | {d.get('wall_seconds',d.get('wall_s',0)):.2f} | {d.get('peak_sampled_tree_rss_bytes',0)/1024**3:.3f} |")
lines+=['','시간에는 계획·모델 읽기·전체 검사·집계가 포함되며 물리 처리량 수치가 아니다. smoke에서 자원을 확인한 뒤 파일럿 CPU case worker8/planning worker2로 실행했다. GPU 물리 병렬은 추가하지 않았다. 캐시 적용 시점과 NAS 읽기 차이가 있으므로 smoke/파일럿 시간을 순수 speedup으로 비교하지 않는다.','']
lines += [(root/'report_context.md').read_text()]
text='\n'.join(lines).rstrip()+'\n'
(root/'REPORT_KO.md').write_text(text)
(repo/'mujoco-benchmark/MULTIFAMILY_SEARCH_SERVER_RESULTS_KO.md').write_text(text)
# Simple gallery uses only actual saved assets, explicit kind displayed beside every video.
items=[]
for folder in ('smoke_videos','pilot_supplement_videos'):
 for item in load(root/folder/'manifest.json'):
  v=Path(item['video']);items.append(dict(path=str(v.relative_to(root)),kind=item['kind'],name=item['name']))
for n in ('pilot','side_repair'):
 for p in (root/n).rglob('*.mp4'):items.append(dict(path=str(p.relative_to(root)),kind='recorded_physics',name=n+' / '+p.stem))
(root/'video_manifest.json').write_text(json.dumps(items,indent=2)+'\n')
parts=['<!doctype html><meta charset="utf-8"><title>다중 경로군 서버 결과</title><style>body{font-family:sans-serif;max-width:1250px;margin:32px auto;background:#141922;color:#e5e8ed}a{color:#8bc9ff}video{width:100%;max-width:960px}pre{white-space:pre-wrap;font-size:14px;line-height:1.6}article{padding:18px;margin:20px 0;background:#202939}strong{color:#ffc96e}</style><h1>다중 경로군 서버 파일럿</h1><p><a href="REPORT_KO.md">결과 보고서 MD</a> · <a href="pilot/index.html">원래 파일럿</a> · <a href="side_repair/index.html">측면 수정 비교</a> · <a href="smoke_v2/index.html">Smoke</a></p><p><strong>실제 실행과 계획 영상은 다릅니다. NOMINAL ONLY는 충돌 검사에서 차단되어 물리 실행하지 않은 경로입니다.</strong></p>']
for i in items:
 parts.append('<article><h2>'+html.escape(i['name'])+'</h2><p><strong>'+html.escape(i['kind'])+'</strong></p><video controls preload="metadata" src="'+html.escape(i['path'])+'"></video></article>')
parts += ['<h2>검증 보고서</h2><pre>'+html.escape(text)+'</pre>']
(root/'index.html').write_text('\n'.join(parts))
print('Report and gallery written:',len(items),'videos')
