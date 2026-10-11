"""Bounded eighteen staging IK endpoints; known unsafe direct prefixes retained.

Only complete self/environment-clear full-open endpoints may become connection
search goals. This is not a native execution or proof of an OMPL connection.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from pxr import Usd
from compact_motion_assets import file_sha256,verify_compact_dependencies
from motion_kinematics import ToolKinematics,plan_motion
from motion_self_collision import MotionSelfCollision,check_plan
import rail_environment_preflight as environment
from screen_fresh_truss_preparation import require_fresh_campaign,write_new


def probe(assembly,base_inputs,output):
    assembly=Path(assembly).resolve();base_inputs=Path(base_inputs).resolve();output=Path(output).resolve()
    manifest=json.loads((assembly/'motion_manifest.json').read_text());marker,start=require_fresh_campaign(manifest,'Truss_01')
    context=json.loads(base_inputs.read_text())
    if context['source_assembly']!=str(assembly) or context['checkpoint_imported'] is not False or context['fresh_campaign']!=marker:
        raise ValueError('Existing source-matching fresh screen input required')
    dep=verify_compact_dependencies(assembly,manifest)
    paths=[assembly/n for n in ('scene.usda','motion_manifest.json','skin.npz',dep['relative_path'])]
    before={str(p):file_sha256(p)for p in paths}
    if before!=context['source_sha256_before']:raise ValueError('Fresh source changed since initial proposals')
    stage=Usd.Stage.Open(str(assembly/dep['relative_path']));kin=ToolKinematics(stage)
    selfcheck=MotionSelfCollision(stage,kin,clearance_m=.002)
    env=environment.RailEnvironmentCheck(stage,manifest,selfcheck,allowed_truss_key='Truss_01',scope='preparation',max_distance_queries=2000000)
    output.mkdir(parents=True,exist_ok=False)
    code={n:file_sha256(Path(__file__).with_name(n))for n in (
        Path(__file__).name,'motion_kinematics.py','motion_self_collision.py','rail_environment_preflight.py')}
    (output/'environment_executed.py').write_bytes(environment.SOURCE_BYTES)
    opts=dict(context['parameters']);opts.update(horizontal_ready_connector='joint',horizontal_ready_normalize_turns=True,
        preparation_height_first=False,approach_distance_m=.25)
    reference=np.asarray(manifest.get('grasp_reference',manifest['target'])['center'],float)
    rows=[];memo={};started=time.perf_counter()
    for lift in (-.135,-.170):
        for seed in range(9):
            name='seed'+str(seed)+'_retract250_lift'+str(round(-lift*1000));folder=output/name;folder.mkdir()
            row=dict(name=name,approach_distance_m=.25,ready_lift_offset_m=lift,seed_index=seed,
                normalize_turns=True,status='not_evaluated',native_runs=0)
            tick=time.perf_counter()
            try:
                parameters=dict(opts,horizontal_ready_seed_index=seed,ready_lift_offset_m=lift)
                phases,plan=plan_motion(kin,reference,start=start,**parameters)
                plan['truss_axis']=context['axis'];plan['fresh_campaign']=marker;write_new(folder/'plan.json',plan)
                row['ik_complete']=True;qs=np.asarray(plan['prep_q'],float)
                key=hashlib.sha256(qs.astype('<f8',copy=False).tobytes()+np.asarray([.096,0.],dtype='<f8').tobytes()).hexdigest()
                if key in memo:
                    endpoint_self,endpoint_env,parent=memo[key];row['exact_endpoint_reused_from']=parent
                else:
                    endpoint_self=selfcheck.check(qs,.096,rail_position_m=0.,max_hits=64)
                    endpoint_env=env.check_state(qs,.096,rail_position_m=0.,phase='staging_full_open96')
                    memo[key]=(endpoint_self,endpoint_env,name)
                row.update(staging_q=qs.tolist(),staging_pose=plan['prep_pose'],grasp_pose=plan['goal_pose'],
                    exact_endpoint_key=key,full_open_gap_m=.096,endpoint_self_clear=not endpoint_self,
                    endpoint_self_hits=endpoint_self,endpoint_environment=endpoint_env)
                row['endpoint_clear_for_connection_search']=bool(not endpoint_self and endpoint_env['complete'] and endpoint_env['hard_clear'])
                if row['endpoint_clear_for_connection_search']:
                    begin=next(i for i,p in enumerate(plan['phases'])if p['name']=='prepare_open')
                    end=next(i for i,p in enumerate(plan['phases'])if p['name']=='withdraw')
                    suffix=dict(plan,phases=plan['phases'][begin:end+1])
                    write_new(folder/'open_manipulation_suffix.json',suffix)
                    suffixself=check_plan(selfcheck,suffix,hz=120)
                    write_new(folder/'open_manipulation_suffix_self_120hz.json',suffixself)
                    row['suffix_self_120hz']=dict(passed=suffixself['passed'],complete=suffixself['complete'],samples=suffixself['samples'],
                        first_blocked=next((r for r in suffixself['phases']if not r['clear']),None),
                        scope='Existing staging44→96 opening, approach, belt cycle, release and withdrawal; unsafe direct preparation/return excluded for OMPL replacement')
                    row['endpoint_and_manipulation_self_clear']=bool(suffixself['passed'] and suffixself['complete'])
                row['status']='clear_endpoint_connection_unproven'if row['endpoint_clear_for_connection_search']else'blocked_endpoint'
                row.update(complete_preparation_path_certified=False,native_execution_eligible=False,
                    manipulation_environment_certified=False)
            except(ValueError,RuntimeError)as error:row.update(status='ik_or_query_error',error=type(error).__name__+': '+str(error))
            row['elapsed_s']=time.perf_counter()-tick;rows.append(row);write_new(folder/'result.json',row)
            print(json.dumps({key:row.get(key)for key in ('name','status','endpoint_and_manipulation_self_clear','elapsed_s')}),flush=True)
    after={str(p):file_sha256(p)for p in paths}
    if before!=after:raise ValueError('Fresh source mutated during endpoint diagnostics')
    aftercode={n:file_sha256(Path(__file__).with_name(n))for n in code}
    report=dict(schema='s300-bounded-fresh-staging-branches-v1',source_assembly=str(assembly),source_sha256=before,
        source_unchanged=True,source_code_sha256=code,source_code_sha256_after=aftercode,source_code_unchanged=code==aftercode,
        environment_executed_sha256=environment.SOURCE_SHA256,baseline_inputs=str(base_inputs),baseline_inputs_sha256=file_sha256(base_inputs),
        fresh_campaign=marker,authored_start_q=start.tolist(),proposal_budget=18,proposals=rows,elapsed_s=time.perf_counter()-started,
        unique_exact_endpoint_queries=len(memo),endpoint_clear_count=sum(r.get('endpoint_clear_for_connection_search',False)for r in rows),
        endpoint_and_manipulation_self_clear_count=sum(r.get('endpoint_and_manipulation_self_clear',False)for r in rows),
        native_runs=0,training_eligible=False,harvest_success=None,finite_search_failure_is_impossible=False,
        physics_or_geometry_changed=False,goal_and_entry_orientation_unchanged=True,
        scope='Goal selection only; full120Hz collision-aware connection and native validation remain required')
    write_new(output/'summary.json',report);return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('assembly',type=Path)
    parser.add_argument('base_inputs',type=Path);parser.add_argument('output',type=Path)
    a=parser.parse_args();r=probe(a.assembly,a.base_inputs,a.output)
    print(json.dumps({k:r[k]for k in('endpoint_clear_count','endpoint_and_manipulation_self_clear_count','elapsed_s')},indent=2))
