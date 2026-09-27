"""Validate and summarize a finished finite sweep without any physics execution."""
from pathlib import Path
import json,hashlib,gzip,numpy as np
O=Path(__file__).parent
BASE=Path('/root/docker_share/mujoko_debugging_data/20260928_target_fruit_contact_server')
m=json.loads((O/'sweep_manifest.json').read_text());results=[];checks=[]
assert len(m['jobs'])==12
for j in m['jobs']:
 d=O/j['name'];s=json.loads((d/'summary.json').read_text());t=s['trial'];f=s['forces']
 assert s['audit']['complete'] and s['audit']['passed'] and t['physics_executed']
 assert t['hook_success'] is None and t['training_eligible'] is False and t['source_unchanged']
 assert t['limits']['max_target_force_N']==j['force']
 assert t['limits']['max_target_displacement_m']==j['displacement_m']
 assert s['sample_timing']['missing_gap_count']==0
 assert t['limits']['max_forbidden_force_N']==.01
 for path,digest in json.loads((d/'contact_scope.json').read_text())['source_sha256'].items():
  assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest
 baseline=BASE/('trial_slow_'+j['candidate'] if j['speed']=='slow18' else 'trial_'+j['candidate'])
 oldfile=np.load(baseline/'trial_states.npz');newfile=np.load(d/'trial_states.npz')
 old={k:oldfile[k] for k in oldfile.files};new={k:newfile[k] for k in newfile.files}
 pairs=[]
 for i,v in enumerate(old['times_s']):
  k=int(np.argmin(abs(new['times_s']-v)))
  if abs(new['times_s'][k]-v)<1e-9:pairs.append((i,k))
 err=max(float(np.max(abs(old['qpos'][i]-new['qpos'][k]))) for i,k in pairs)
 assert err<1e-12
 checks.append(dict(name=j['name'],baseline=str(baseline),matching_stored_timestamps=len(pairs),max_qpos_prefix_error=err))
 row=dict(**j,complete=t['completed'],stop=t.get('stop_reasons',[]),simulated_s=t['simulated_s'],last_phase=s['last_phase'],
  force_live_N=f['contact_categories_live']['target_fruit_touch']['max_force_sum_N'],force_private_N=f['contact_categories_private']['target_fruit_touch']['max_force_sum_N'],
  pedicel_force_N=max(f[k]['target_pedicel_contact']['max_force_sum_N'] for k in f),
  forbidden_force_N=max(f[k]['forbidden_contact']['max_force_sum_N'] for k in f),
  target_displacement_mm=s['target_displacement_m']['max_value']*1000,fruit_penetration_mm=s['target_fruit_penetration_m']*1000,
  global_penetration_mm=s['guard_penetration_m']['max_value']*1000,seated_samples=s['geometric_seated_samples'],
  seated_with_force_samples=s['seated_with_pedicel_force_samples'],authorized=t['authorized_contact_evidence'],legacy=t['legacy_evidence'],
  forbidden_pair_peaks=[p for p in s['pair_peaks'] if p['role']=='forbidden_contact' and p['max_single_normal_N']>0],
  seat_max_tcp_speed_m_s=s.get('seat_actual_tcp_speed_m_s'),max_lift_error_mm=s['prismatic_error_m']['max_value']*1000,
  max_revolute_error_rad=s['revolute_error_rad']['max_value'],physics_valid_until_stop=t['authorized_contact_evidence']['physics_valid'])
 raw=json.loads(gzip.decompress((d/'authorized_contact_samples.json.gz').read_bytes()))
 row['phase_geometry']={}
 for phase in ['seat','hold','verify']:
  part=[v for v in raw if v['phase']==phase]
  gaps=[min(g['minimum_rear_surface_gap_m'] for g in v['geometry']) for v in part if v['geometry']]
  positive=sum(b['time_s']-a['time_s'] for a,b in zip(part,part[1:]) if a['seated'] and b['seated'] and b['time_s']-a['time_s']<=1.5/240)
  row['phase_geometry'][phase]=dict(samples=len(part),geometric_positive_samples=sum(v['seated'] for v in part),
    all_geometric=bool(part) and all(v['seated'] for v in part),positive_adjacent_elapsed_s=positive,
    minimum_rear_gap_mm=min(gaps)*1000 if gaps else None,maximum_nearest_rear_gap_mm=max(gaps)*1000 if gaps else None)
 results.append(row)
(O/'results.json').write_text(json.dumps(dict(executions=len(results),rows=results,training_eligible=False,hook_success=None),indent=2)+'\n')
(O/'verification.json').write_text(json.dumps(dict(all_audits_complete_clear=True,all_source_hashes_unchanged=True,all_samples_continuous=True,threshold_only_prefix_comparison=checks),indent=2)+'\n')
for r in results:
 print(r['name'],r['stop'],round(r['simulated_s'],4),'F',round(max(r['force_live_N'],r['force_private_N']),4),'D',round(r['target_displacement_mm'],3),'other',round(r['forbidden_force_N'],4),'seat',r['seated_samples'])
