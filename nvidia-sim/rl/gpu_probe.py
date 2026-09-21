"""Run a small reproducible CPU/GPU comparison, never production data generation."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np

HERE=Path(__file__).resolve().parent


def compare(reference,trial):
    """Never report early failure or different physical work as a speedup."""
    a=json.loads((reference/'report.json').read_text())
    b=json.loads((trial/'report.json').read_text())
    result=dict(reference=str(reference),trial=str(trial),comparable=False,speedup=None)
    physical_keys={'joint_armature':1e-5,'preload_mode':'effort'}
    result['same_physical_model']=all(a.get('arguments',{}).get(k,default)==b.get('arguments',{}).get(k,default)
                                      for k,default in physical_keys.items())
    if not a.get('complete') or not b.get('complete'):
        return dict(result,reason='experiment did not finish')
    if any(r.get('backend',{}).get('suppress_readback') for r in (a,b)):
        return dict(result,reason='native collider contact reports unavailable with suppressed CPU readback; labels are not comparable')
    if any(r.get('api_checks_passed') is False for r in (a,b)):
        return dict(result,reason='native contact/reset/break checks failed; speedup withheld')
    if a['num_envs']!=b['num_envs'] or a['fixture']['commands_sha256']!=b['fixture']['commands_sha256']:
        return dict(result,reason='different environment count or commanded trajectory')
    initial_a=np.load(reference/'initial_state.npz');initial_b=np.load(trial/'initial_state.npz')
    result['initial_state_max_abs_errors']={k:float(np.max(abs(initial_a[k]-initial_b[k]))) for k in initial_a}
    checks=[]
    for i in range(a['num_envs']):
        ta=json.loads((reference/f'trace_{i}.json').read_text());tb=json.loads((trial/f'trace_{i}.json').read_text())
        n=min(len(ta),len(tb))
        errors={k:float(np.max(abs(np.asarray([r[k] for r in ta[:n]])-np.asarray([r[k] for r in tb[:n]])))) if n else None
            for k in ('joints','target_displacement_m','main_stem_displacement_m','gap_m')}
        ma=a['motion']['outcomes'][i];mb=b['motion']['outcomes'][i]
        fields=('abort_reason','target_broken','other_broken','retained_hook','inserted','first_contact_object')
        same={k:ma[k]==mb[k] for k in fields}
        same['result']=ma['classification']['result']==mb['classification']['result']
        close=(n>0 and abs(len(ta)-len(tb))<=1 and all(same.values()) and
            errors['joints']<=.005 and max(errors[k] for k in errors if k!='joints')<=.001)
        checks.append(dict(env=i,reference_control_steps=len(ta),trial_control_steps=len(tb),
            same_outcome=same,common_prefix_max_abs_errors=errors,passed=bool(close)))
    result['trajectory_checks']=checks
    result['screening_tolerances']=dict(position_m=.001,joint_rad=.005,control_step_count=1,
        scope='small regression screen, not physical calibration or a no-tunnelling certificate')
    result['comparable']=bool(result['same_physical_model'] and all(c['passed'] for c in checks) and max(result['initial_state_max_abs_errors'].values())<=1e-6)
    if result['comparable']:
        result['speedup']=a['motion']['wall_s']/b['motion']['wall_s']
    else:result['reason']='different physical model, initial state, physical trajectory or outcome; speedup withheld'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture',type=Path,required=True,help='Existing candidate directory with candidate.json and planned_commands.npy')
    parser.add_argument('--run-dir',type=Path)
    parser.add_argument('--num-envs',type=int,choices=[1,4,8,16,32,64],default=1)
    parser.add_argument('--timeout-seconds',type=float,default=600)
    parser.add_argument('--diagnose-external-forces',action='store_true')
    parser.add_argument('--suite',choices=['motion','isolation'],default='motion',help='Isolation holds robot still and varies one diagnostic condition')
    parser.add_argument('--gpu-solver',choices=['tgs','pgs'],default='tgs')
    parser.add_argument('--gpu-position-iterations',type=int,default=64)
    parser.add_argument('--batched-io',action='store_true',help='GPU dynamics with batched efforts/state and native collider readback')
    parser.add_argument('--cuda-tensors',action='store_true')
    parser.add_argument('--gpu-partitions',type=int,default=1,choices=[1,2,4,8,16,32],help='1 avoids branched-plant multi-partition divergence; use 8 to reproduce the old baseline')
    parser.add_argument('--fabric',action='store_true')
    parser.add_argument('--stationary-steps',type=int,default=240)
    args=parser.parse_args()
    if args.batched_io and args.cuda_tensors:parser.error("batched-io preserves native CPU contact readback; omit cuda-tensors")
    if args.timeout_seconds<=0 or args.stationary_steps<=0:parser.error('timeout and stationary-steps must be positive')
    if not 1<=args.gpu_position_iterations<=255:parser.error('gpu-position-iterations must be 1..255')
    root=(args.run_dir or HERE/'runs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_gpu_physics_probe')).resolve()
    if root.exists():parser.error('Choose a new run directory')
    fixture=args.fixture.resolve()
    row=json.loads((fixture/'candidate.json').read_text())
    if row['target_id']!='Tomato_05':parser.error('This minimal probe currently uses Tomato_05 only')
    for name in ('candidate.json','planned_commands.npy'):
        if not (fixture/name).is_file():parser.error('Missing fixture '+name)
    (root/'fixture').mkdir(parents=True)
    for name in ('candidate.json','planned_commands.npy'):shutil.copy2(fixture/name,root/'fixture'/name)
    sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')}
    (root/'source_sha256.json').write_text(json.dumps(sources,indent=2)+'\n')
    (root/'experiment.json').write_text(json.dumps(dict(fixture_source=str(fixture),num_envs=args.num_envs,
        primary_modes=['cpu','cpu-no-ccd','gpu'],physics_hz=960,control_hz=60,solver_position_iterations=64,
        gpu_method='GPU dynamics and GPU broadphase, CPU tensor readback; not a batched CUDA port',
        source_geometry_randomization=False,suite=args.suite,gpu_solver=args.gpu_solver,
        gpu_position_iterations=args.gpu_position_iterations,gpu_max_num_partitions=args.gpu_partitions,cuda_tensors=args.cuda_tensors,fabric=args.fabric),indent=2)+'\n')
    variants=[('cpu','cpu',[]),('cpu-no-ccd','cpu-no-ccd',[]),
        ('gpu','gpu',['--solver',args.gpu_solver])]
    if args.diagnose_external_forces:
        variants.append(('gpu-forces-once','gpu',['--external-forces-once','--solver',args.gpu_solver]))
    if args.suite=='isolation':
        variants=[('cpu-stationary','cpu',[]),('gpu-stationary','gpu',[]),
            ('gpu-cold-joints','gpu',['--preserve-joints']),
            ('gpu-velocity-zero','gpu',['--velocity-iterations','0']),
            ('gpu-no-contacts','gpu',['--no-contacts']),
            ('gpu-zero-load','gpu',['--zero-load']),
            ('gpu-zero-load-no-contacts','gpu',['--zero-load','--no-contacts']),
            ('gpu-pgs','gpu',['--solver','pgs'])]
    summary=dict(run_dir=str(root),experiments={},comparisons={},production_gpu_validated=False)
    print('[GPU PROBE OUTPUT]',root,flush=True)
    for name,mode,extra in variants:
        folder=root/name;folder.mkdir()
        command=[sys.executable,'-u',str(HERE/'gpu_probe_worker.py'),'--output',str(folder),
            '--fixture',str(root/'fixture'),'--mode',mode,
            '--num-envs',str(args.num_envs),'--headless',*extra]
        if args.suite=='isolation':command+=['--stationary-steps',str(args.stationary_steps)]
        if args.batched_io and mode=='gpu':command.append('--batched-io')
        if args.cuda_tensors and mode=='gpu':command.append('--cuda-tensors')
        if mode=='gpu':command+=['--position-iterations',str(args.gpu_position_iterations),'--gpu-partitions',str(args.gpu_partitions)]
        if args.fabric:command.append('--fabric')
        write_path=folder/'command.json'
        write_path.write_text(json.dumps(command,indent=2)+'\n')
        began=time.monotonic();timed_out=False
        with (folder/'run.log').open('w') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
            try:
                while process.poll() is None:
                    if time.monotonic()-began>args.timeout_seconds:
                        timed_out=True;process.kill();process.wait();break
                    try:process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        report_path=folder/'report.json'
                        try:phase=json.loads(report_path.read_text()).get('phase')
                        except (FileNotFoundError,json.JSONDecodeError):phase='launching Isaac'
                        print('[GPU PROBE PROGRESS]',name,phase,round(time.monotonic()-began,1),'wall s',flush=True)
            except KeyboardInterrupt:
                process.kill();process.wait();raise
        report=json.loads((folder/'report.json').read_text()) if (folder/'report.json').exists() else {}
        summary['experiments'][name]=dict(exit_code=process.returncode,timed_out=timed_out,
            wall_s=time.monotonic()-began,complete=report.get('complete',False))
        if args.suite=='motion' and name!='cpu' and (root/'cpu/report.json').exists() and report:
            summary['comparisons'][name]=compare(root/'cpu',folder)
        if args.suite=='isolation':
            summary['experiments'][name]['stationary']=report.get('stationary')
        (root/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print('[GPU PROBE SUMMARY]',json.dumps(summary),flush=True)
    if any(v['timed_out'] or v['exit_code'] or not v['complete'] for v in summary['experiments'].values()):
        raise SystemExit('Probe execution incomplete; inspect summary.json and worker logs')


if __name__=='__main__':main()
