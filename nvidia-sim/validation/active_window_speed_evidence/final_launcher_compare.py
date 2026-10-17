"""Saved-array exact comparison and measured orchestration audit; no physics/GPU."""
import argparse,json,pathlib,time,hashlib,datetime
import numpy as np
R=pathlib.Path('/root/docker_share/mujoko_debugging_data/20261018_031544_newton_active_window_speed');CAMPAIGN=R/'route/final_launcher_validation'
OLD=[R/'route/active_window_baseline/step_00_plant22',R/'route/active_window_followup/step_01_plant27',R/'route/active_window_followup/step_02_plant29']
NEW=[CAMPAIGN/f'step_{i:02d}_plant{p}' for i,p in enumerate([22,27,29])]
DIGESTS={}
def read(p):return json.loads(pathlib.Path(p).read_text())
def digest(p):
 p=pathlib.Path(p).resolve();st=p.stat();key=(str(p),st.st_size,st.st_mtime_ns,st.st_ctime_ns)
 if key not in DIGESTS:
  h=hashlib.sha256()
  with p.open('rb') as f:
   for block in iter(lambda:f.read(2**20),b''):h.update(block)
  DIGESTS[key]=h.hexdigest()
 return DIGESTS[key]
def arr(a,b):
 layout=a.shape==b.shape and a.dtype==b.dtype;same=bool(layout and a.tobytes()==b.tobytes());d=dict(shape_a=list(a.shape),shape_b=list(b.shape),dtype_a=str(a.dtype),dtype_b=str(b.dtype),byte_identical=same)
 if layout and a.dtype.kind in 'biufc':d['max_abs_difference']=float(np.max(np.abs(a.astype(np.longdouble)-b.astype(np.longdouble)))) if a.size else 0.
 if not same and layout:
  ix=np.argwhere(a!=b)
  if len(ix):
   at=tuple(ix[0]);d['first_difference_index']=list(at);d['first_a']=a[at].item();d['first_b']=b[at].item()
 return d

def archives(old,new):
 names=sorted({p.name for p in old.glob('*.npz')}|{p.name for p in new.glob('*.npz')}|{p.name for p in old.glob('*.npy')}|{p.name for p in new.glob('*.npy')})
 checks={}
 for n in names:
  if not (old/n).exists() or not (new/n).exists():checks[n]=dict(present_both=False,all_arrays_byte_identical=False);continue
  if n.endswith('.npz'):
   with np.load(old/n,allow_pickle=False) as a,np.load(new/n,allow_pickle=False) as b:
    fields={k:arr(a[k],b[k]) for k in sorted(set(a.files)&set(b.files))};keys=set(a.files)==set(b.files)
   row=dict(present_both=True,keys_identical=keys,all_arrays_byte_identical=keys and all(x['byte_identical'] for x in fields.values()),fields=fields)
  else:
   result=arr(np.load(old/n,allow_pickle=False),np.load(new/n,allow_pickle=False));row=dict(present_both=True,all_arrays_byte_identical=result['byte_identical'],fields={'array':result})
  row.update(source_file_sha256=digest(old/n),fresh_file_sha256=digest(new/n));checks[n]=row
 return checks

def first_state_difference(old,new):
 with np.load(old/'states.npz',allow_pickle=False) as a,np.load(new/'states.npz',allow_pickle=False) as b:
  earliest=[]
  for key in ['qpos','qvel','eq_active','poses','robot_q','phase','command','time']:
   if a[key].shape!=b[key].shape:earliest.append(dict(field=key,shape_changed=True));continue
   indices=np.argwhere(a[key]!=b[key])
   if len(indices):
    idx=indices[0];record=int(idx[0]);at=tuple(idx)
    earliest.append(dict(field=key,record_index=record,index=idx.tolist(),source_solver_step=int(a['solver_step'][record]),fresh_solver_step=int(b['solver_step'][record]),source_time_s=float(a['time'][record]),fresh_time_s=float(b['time'][record]),phase_source=str(a['phase'][record]),phase_fresh=str(b['phase'][record]),source_value=a[key][at].item(),fresh_value=b[key][at].item()))
 return sorted(earliest,key=lambda x:x.get('record_index',-1))

def compare(index,oldsegment,newsegment):
 old,new=oldsegment/'physics',newsegment/'physics';a,b=read(old/'summary.json'),read(new/'summary.json');checks=archives(old,new)
 metadata_keys=['status','stop','source_model_mjb_sha256','source_model_counts','executed_commands_sha256','commands_sha256','physics_hz','free_hz','actual_steps','requested_steps','timestep_counts','effective_gpu_options','contact_policy','initial_physics_clock_s','final_physics_clock_s','inherited_release_event_count','cumulative_release_event_count','finite_recorded_state','qacc_finite','solver_capacity_valid','continuation_model_identity','released_gutter_contacts']
 metadata={k:dict(equal=a.get(k)==b.get(k),source=a.get(k),fresh=b.get(k)) for k in metadata_keys}
 jsonnames=['break_events.json','cumulative_break_events.json','newton_break_events_all_worlds.json','contact_policy.json','contact_policy_input.json'];jsons={n:dict(exact=read(old/n)==read(new/n),source_sha256=digest(old/n),fresh_sha256=digest(new/n)) for n in jsonnames if (old/n).exists() and (new/n).exists()}
 ai,bi=pathlib.Path(a['input']),pathlib.Path(b['input']);initial={}
 for name in ['initial_state.npz','commands.npz','canonical_full_state.npz']:
  with np.load(ai/name,allow_pickle=False) as av,np.load(bi/name,allow_pickle=False) as bv:
   fields={k:arr(av[k],bv[k]) for k in sorted(set(av.files)&set(bv.files))};keys=set(av.files)==set(bv.files)
  initial[name]=dict(keys_identical=keys,all_arrays_byte_identical=keys and all(v['byte_identical'] for v in fields.values()),fields=fields)
 planned={name:dict(equal=read(oldsegment/name)==read(newsegment/name),source_sha256=digest(oldsegment/name),fresh_sha256=digest(newsegment/name)) for name in ['plan.json','selected_group.json'] if (oldsegment/name).exists() and (newsegment/name).exists()}
 cert=None
 if index:
  c=read(bi/'window_transition.json');parent=pathlib.Path(c['parent_run']);expected=NEW[index-1]/'physics'
  cert=dict(path=str(bi/'window_transition.json'),sha256=digest(bi/'window_transition.json'),actual_previous_fresh_parent=parent.resolve()==expected.resolve(),parent_summary_hash_valid=digest(parent/'summary.json')==c['parent_summary_sha256'],parent_final_hash_valid=digest(parent/'final_states.npz')==c['parent_final_states_sha256'],child_hashes_valid={n:digest(bi/n)==h for n,h in c['child_files'].items() if n!='input_model.mjb'},child_mjb_sha_bound_to_fresh_summary=c['child_files']['input_model.mjb']==b['source_model_mjb_sha256'],canonical_common_mjb_sha256=c['canonical_mjb_sha256'],initial_clock_s=c['initial_clock_s'],inherited_release_count=len(c['inherited_release_events']),gpu_encoding=c['backend_state_encoding'])
 timings=read(newsegment/'orchestration_timings.json');physics_total=timings['physics_process_including_lock_wait_setup_and_save_s'];setup=b['setup_wall_s'];loop=b['loop_wall_s'];process_residual=physics_total-setup-loop
 physical_model_same=a['source_model_mjb_sha256']==b['source_model_mjb_sha256'];commands_same=a['executed_commands_sha256']==b['executed_commands_sha256'];rawsame=all(v['all_arrays_byte_identical'] for v in checks.values());release_same=all(v['exact'] for v in jsons.values());metasame=all(v['equal'] for v in metadata.values());qualified=bool(b['status']=='completed' and b['stop'] is None and b['finite_recorded_state'] and b['qacc_finite'] and b['solver_capacity_valid'] and b['contact_policy']['policy_passed'])
 return dict(index=index,plant_id=[22,27,29][index],source_run=str(old),fresh_run=str(new),completed_physics_qualified=qualified,physical_model_sha_same=physical_model_same,executed_command_sha_same=commands_same,all_saved_archives_byte_identical=rawsame,all_release_and_policy_json_exact=release_same,metadata_equal=metasame,source_metadata_differences=[k for k,v in metadata.items() if not v['equal']],first_recorded_differences=[] if rawsame else first_state_difference(old,new),initial_and_command_archives=initial,plan_checks=planned,transition=cert,metadata=metadata,archives=checks,json_records=jsons,measured_performance=dict(source_loop_wall_s=a['loop_wall_s'],fresh_loop_wall_s=loop,source_loop_rtf=a['realtime_factor'],fresh_loop_rtf=b['realtime_factor'],source_setup_s=a['setup_wall_s'],fresh_setup_s=setup,simulated_segment_s=b['segment_simulated_s'],new_detachments=len(b['breaks']),orchestration=timings,physics_process_residual_beyond_setup_and_loop_s=process_residual,residual_scope='Combined wrapper/import/lock wait/save/teardown; cannot separate or attribute without instrumentation.'),source_summary_sha256=digest(old/'summary.json'),fresh_summary_sha256=digest(new/'summary.json'),existing_actual_video_state_reuse_eligible=bool(qualified and physical_model_same and commands_same and rawsame and release_same and metasame),video_reuse_scope='Only if complete actual time/pose/eq and commands match exact; old visual source must also be unchanged. Metadata source paths/certificate hashes may change without physical difference; their exact lineage is checked separately.')

def main(watch=False):
 rows={};out=R/'final_launcher_comparison.json';start=time.monotonic()
 while True:
  for i,(old,new) in enumerate(zip(OLD,NEW)):
   if i in rows or not (new/'orchestration_timings.json').exists():continue
   if not (new/'physics/summary.json').exists():continue
   rows[i]=compare(i,old,new);print('compared',i,rows[i]['all_saved_archives_byte_identical'],rows[i]['executed_command_sha_same'],flush=True)
  campaign=read(CAMPAIGN/'campaign.json') if (CAMPAIGN/'campaign.json').exists() else None;errors=[str(p) for p in CAMPAIGN.glob('step_*/orchestration_error.json')];finished=len(rows)==3 or bool(errors) or bool(campaign and campaign['rows'] and campaign['rows'][-1]['status']!='completed')
  entries=[rows[i] for i in sorted(rows)];sim=sum(x['measured_performance']['simulated_segment_s'] for x in entries);loop=sum(x['measured_performance']['fresh_loop_wall_s'] for x in entries);setup=sum(x['measured_performance']['fresh_setup_s'] for x in entries);knownorch=sum(x['measured_performance']['orchestration']['total_segment_orchestration_s'] for x in entries)
  invocation=read(CAMPAIGN/'invocation.json') if (CAMPAIGN/'invocation.json').exists() else None
  result=dict(schema='s300-final-one-command-launcher-independent-exact-comparison-v1',generated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='complete' if len(rows)==3 else 'stopped_or_error' if finished else 'in_progress',campaign=str(CAMPAIGN),campaign_rows=campaign,invocation_options=invocation['options'] if invocation else None,source_reference='Original qualified220Hz3station sequence, not180Hz or belt-hold variants.',segments=entries,pending_indices=[i for i in range(3) if i not in rows],orchestration_errors=errors,all_three_completed_and_qualified=len(rows)==3 and all(x['completed_physics_qualified'] for x in entries),all_three_raw_archives_exact=len(rows)==3 and all(x['all_saved_archives_byte_identical'] for x in entries),all_three_commands_exact=len(rows)==3 and all(x['executed_command_sha_same'] for x in entries),all_three_physical_model_hashes_same=len(rows)==3 and all(x['physical_model_sha_same'] for x in entries),old_video_state_reuse_qualified=len(rows)==3 and all(x['existing_actual_video_state_reuse_eligible'] for x in entries),measured_time_subtotals=dict(completed_segments=len(rows),simulated_s=sim,loop_wall_s=loop,loop_rtf=sim/loop if loop else None,runtime_setup_wall_s=setup,setup_plus_loop_s=setup+loop,setup_plus_loop_rtf=sim/(setup+loop) if setup+loop else None,plan_and_command_preparation_s=sum(x['measured_performance']['orchestration']['plan_and_command_preparation_s'] for x in entries),model_reduction_and_transition_validation_s=sum(x['measured_performance']['orchestration'].get('model_reduction_and_transition_validation_s',0) for x in entries),physics_process_with_lock_setup_save_s=sum(x['measured_performance']['orchestration']['physics_process_including_lock_wait_setup_and_save_s'] for x in entries),sum_segment_orchestration_s=knownorch,orchestration_rtf=sim/knownorch if knownorch else None,initial_validate_prepared_and_python_start_wall_s=None,total_shell_launch_elapsed_s=None,scope='Actual measured segment timers; the initial source validation before invocation.json and shell/Python startup are not timed here. Sum segment orchestration includes measured planning, verification, reduction, transitions and full GPU subprocess incl lock wait/setup/save; not a complete shell-launch stopwatch.'),analysis_physics_steps=0,analysis_gpu_execution=False,limitations=['Single serialized cold/new-process campaign under current cache and load conditions; no repeated-run distribution or cold-filesystem mean.','Baseline lacks per-stage orchestration timing, so no inferred full-campaign speed ratio is reported.','A loop RTF gain must not hide planning/startup/model-switch waits.','Exact saved metrics do not include every contact Jacobian/individual contact point; all available raw state/force/policy arrays are compared.','Body/pose equality is limited to this model and command configuration.','Three different active-window MJBs remain measured-state continuation, not one persistent solver or379DOF dynamics equivalence.'])
  out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
  md=['# 최종 one-command 재실행 독립 비교','',f"상태: **{result['status']}**, 완료비교 {len(rows)}/3. 원본220Hz3정차점과최종launcher새실제physics의배열을직접비교했다. 추가physics/GPU실행은0회다.",'','|대상|물리모델SHA동일|명령동일|저장배열byte동일|실제완주|새분리|새loopRTF|setup(s)|계획준비(s)|모델전환검증(s)|전체구간orchestration(s)|','|---|---|---|---|---|---:|---:|---:|---:|---:|---:|']
  for x in entries:
   p=x['measured_performance'];t=p['orchestration'];md.append(f"|{x['plant_id']}|{x['physical_model_sha_same']}|{x['executed_command_sha_same']}|{x['all_saved_archives_byte_identical']}|{x['completed_physics_qualified']}|{p['new_detachments']}|{p['fresh_loop_rtf']:.3f}|{p['fresh_setup_s']:.3f}|{t['plan_and_command_preparation_s']:.3f}|{t.get('model_reduction_and_transition_validation_s',0):.3f}|{t['total_segment_orchestration_s']:.3f}|")
  totals=result['measured_time_subtotals'];md+=['',f"완료구간물리{sim:.6f}초,loop{loop:.6f}초,setup{setup:.6f}초. 계획준비{totals['plan_and_command_preparation_s']:.6f}초,모델축소/전환검증{totals['model_reduction_and_transition_validation_s']:.6f}초,합산orchestration{knownorch:.6f}초. loop와전체준비대기포함속도를분리했다.",'','최초validate_prepared/셸·Python시작은이타이머범위밖이므로전체실행시간을0또는파일mtime추정으로채우지않았다. root의외부stopwatch가있으면별도추가해야한다. 과거캠페인은해당단계별타이머가없어전체orchestration속도비를추측하지않는다.','',f"기존실제상태영상재사용자격: **{result['old_video_state_reuse_qualified']}**. 완료3개모두model/command/rawstate/접촉지표/분리/정책이정확히동일할때만True이며화면USD자산도같아야한다.",'',f"JSON: [{out}]({out})"]
  (R/'final_launcher_comparison_KO.md').write_text('\n'.join(md)+'\n')
  if finished or not watch:break
  if time.monotonic()-start>1800:print('watch time bound reached;partial evidence preserved',flush=True);break
  time.sleep(30)
 print('saved',out,result['status'],flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--watch',action='store_true');main(p.parse_args().watch)
