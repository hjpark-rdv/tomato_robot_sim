"""Experimental GPU PhysX dataset runner: one simulator, batched environments, native collider reports."""
import argparse
import json
import os
from pathlib import Path
import signal
import queue
import threading
import time
import subprocess
import sys
from datetime import datetime
from dataset_design import BOUNDS, candidates
from gpu_physics_errors import invalid_physics_message

HERE = Path(__file__).resolve().parent


def worker_lines(process, root):
    """Drain stdout without getting stuck forever in failed Kit shutdown."""
    lines=queue.Queue()
    def drain():
        for line in process.stdout:lines.put(line)
        lines.put(None)
    threading.Thread(target=drain,daemon=True).start()
    error_since=None
    last_notice=time.monotonic()
    while True:
        try:line=lines.get(timeout=1)
        except queue.Empty:line=''
        if line:
            # Observe the child's already-formatted log in the parent. A Carb
            # Python logger callback can contend with native USD worker locks.
            if invalid_physics_message(line,line):
                (root/'execution_error.json').write_text(json.dumps(dict(type='NativePhysicsError',message=line.strip(),dataset_valid=False),indent=2)+'\n')
                yield line
                try:os.killpg(process.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                raise RuntimeError('Native PhysX error invalidated this run; see execution_error.json')
            yield line
        if process.poll() is not None and lines.empty():return
        now=time.monotonic()
        if (root/'execution_error.json').exists():
            if error_since is None:error_since=now
            elif now-error_since>20:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
                raise RuntimeError('GPU worker failed and stalled during shutdown; see execution_error.json')
        if now-last_notice>30:
            if not (root/'backend.json').exists():
                print('[DATASET INIT] GPU scene initialization is still running; see run.log',flush=True)
            last_notice=now


def run_worker(command,root):
    with (root/'run.log').open('a') as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                   start_new_session=True)
        def stopped(signum, frame):
            raise KeyboardInterrupt
        previous={sig:signal.signal(sig,stopped) for sig in (signal.SIGTERM,signal.SIGHUP)}
        interrupted=False
        try:
            for line in worker_lines(process,root):
                log.write(line); log.flush()
                if any(word in line for word in ('[DATASET', '[ERROR', 'Traceback', 'Error:', 'RuntimeError', 'ValueError')):
                    print(line, end='', flush=True)
            status = process.wait()
        except KeyboardInterrupt:
            interrupted=True
            print('[DATASET STOP] Stop signal received; keeping completed candidate files.',flush=True)
        finally:
            previous[signal.SIGINT]=signal.getsignal(signal.SIGINT)
            for sig in previous:signal.signal(sig,signal.SIG_IGN)
            if process.poll() is None:
                try:os.killpg(process.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    try:os.killpg(process.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                    process.wait()
            for sig,handler in previous.items():signal.signal(sig,handler)
        if interrupted:raise SystemExit(130)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='Tomato_05', choices=[f'Tomato_{i:02d}' for i in range(1,12)])
    parser.add_argument('--num-envs', type=int, default=4, help='Environments in ONE GPU PhysX process; experimental PGS solver')
    parser.add_argument('--candidates', type=int, default=100)
    parser.add_argument('--candidate-indices',default='',help='Optional comma-separated Sobol candidate indices, e.g. 0,3,27; no resampling')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--prepare-only', action='store_true', help='Write configuration and Sobol candidates without launching Isaac')
    parser.add_argument('--validate-only', action='store_true', help='Check clones, independent reset, contact routing and RGB-D; no candidate rollouts')
    parser.add_argument('--camera-profile', choices=['current','left','right'], default='current', help='Existing D435 extrinsics or previously proposed tool mounts')
    parser.add_argument('--camera-distance', type=float, default=.35, help='Observation distance in metres; hardware depth validity must be calibrated')
    parser.add_argument('--depth-min', type=float, default=.10)
    parser.add_argument('--depth-max', type=float, default=3.)
    parser.add_argument('--crop-extent', type=float, default=.20, help='Context width/height in metres at target depth')
    parser.add_argument('--goal', choices=['rise','pull'], default='rise')
    parser.add_argument('--rise-speed', type=float, default=.002, help='Rise speed limit in m/s (changes physical motion)')
    parser.add_argument('--pull-speed', type=float, default=.004, help='Pull speed limit in m/s (changes physical motion)')
    parser.add_argument('--profile', action='store_true', help='Save CPU profile for throughput diagnosis')
    parser.add_argument('--benchmark-candidates', type=int, default=0, help='Execute only this many pending full candidates, estimate total runtime; resume without this flag')
    parser.add_argument('--physics-threads', type=int, default=4, help='CPU PhysX worker threads; benchmark before scaling environments')
    parser.add_argument('--physics-hz',type=int,choices=[60,120,240,480,720,960],default=960,help='Experimental physics rate; robot control remains 60 Hz; default preserves validated baseline')
    parser.add_argument('--elastic-joint-armature',type=float,default=1e-5,help='Experimental numerical plant joint inertia in kg m^2; changes physical response, not an equivalent speedup; original 1e-5')
    parser.add_argument('--planning-workers',type=int,default=8,help='CPU-only path workers; 0 uses original synchronous planning; capped at pending candidate count')
    parser.add_argument('--torch-threads', type=int, default=1, help='Small CPU tensors usually benefit from one Torch thread')
    parser.add_argument('--physics-sync', choices=['optimized','legacy'], default='optimized', help='Retain implicit drive targets between control updates; legacy is for equivalence checks')
    parser.add_argument('--max-control-steps', type=int, default=0, help='Debug only; truncated candidates are labelled incomplete, never failure training data')
    parser.add_argument('--schedule',choices=['batch','continuous'],default='batch',help='Continuous refills completed slots after independent reset; batch retains the original barrier')
    parser.add_argument('--command-uploads',choices=['batched','legacy'],default='batched',help='Batch joint targets across clones; legacy is for equivalence checks')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--view-grid', action='store_true', help='Show live physics states in a read-only 4x4 display grid (enables GUI)')
    parser.add_argument('--view-fps', type=float, default=5., help='Maximum display refreshes per wall-clock second; physics timestep unchanged')
    parser.add_argument('--keep-open', action='store_true', help='Keep the live display open after completion until the window is closed')
    parser.add_argument('--rebuild', action='store_true')
    args = parser.parse_args()
    if args.view_grid:args.gui=True
    if not 1 <= args.view_fps <= 30:parser.error('view-fps must be 1..30')
    if args.keep_open and not args.view_grid:parser.error('keep-open requires view-grid')
    if not 1 <= args.num_envs <= 1024 or not 1 <= args.candidates <= 100000:
        parser.error('num-envs must be 1..1024, candidates 1..100000')
    if not 0 < args.depth_min < args.depth_max or not .15 <= args.camera_distance <= 1. or not .05 <= args.crop_extent <= .6:
        parser.error('Invalid camera depth/distance/crop settings')
    if args.max_control_steps < 0:
        parser.error('max-control-steps must be nonnegative')
    if not 0<=args.planning_workers<=48:parser.error('planning-workers must be 0..48')
    if not 0<=args.elastic_joint_armature<=.01:parser.error('elastic-joint-armature must be finite and in 0..0.01')
    if not 0 < args.rise_speed <= .02 or not 0 < args.pull_speed <= .02:
        parser.error('rise-speed and pull-speed must be > 0 and <= .02 m/s')
    if not 2 <= args.physics_threads <= 48 or not 1 <= args.torch_threads <= 48:
        parser.error('physics-threads must be 2..48; torch-threads must be 1..48')
    if args.benchmark_candidates < 0:
        parser.error('benchmark-candidates must be nonnegative')
    try:
        indices=[int(v) for v in args.candidate_indices.split(',')] if args.candidate_indices else None
    except ValueError:
        parser.error('candidate-indices must be comma-separated integers')
    if indices is not None and (len(set(indices))!=len(indices) or any(i<0 or i>=args.candidates for i in indices)):
        parser.error('candidate-indices must be unique and within the candidate pool')
    root = (args.run_dir or HERE/'runs'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.target.lower()+'_gpu_candidate_dataset')).resolve()
    root.mkdir(parents=True, exist_ok=True)
    config = vars(args).copy(); config.pop('run_dir'); config.pop('resume'); config.pop('prepare_only'); config.pop('rebuild')
    config.pop('benchmark_candidates');config.pop('profile')
    config.update(schema_version=1, sampling='scrambled_sobol_plus_nominal', bounds=BOUNDS,
                  domain_randomization=False, learning=False, physics_device='gpu', tensor_device='cpu', solver='PGS',
                  experimental=True, cpu_tgs_equivalence_guaranteed=False, gpu_max_num_partitions=1,native_physics_replication=True,
                  parallelism='single GPU PhysX scene, batched articulation I/O, native CPU collider readback',
                  observation_mode='isolated_single_environment' if args.num_envs>128 and not args.gui else 'in_process')
    path = root/'config.json'
    if path.exists():
        if not args.resume:
            parser.error('Output exists; use --resume with the same configuration or a new directory')
        if json.loads(path.read_text()) != json.loads(json.dumps(config)):
            parser.error('Resume configuration differs; choose a new output directory')
    else:
        path.write_text(json.dumps(config, indent=2)+'\n')
        proposals=candidates(args.candidates,args.seed)
        if indices is not None:proposals=[proposals[i] for i in indices]
        (root/'candidates.json').write_text(json.dumps(proposals,indent=2)+'\n')
    print('[DATASET]',root,flush=True)
    if args.prepare_only:
        print('Prepared candidates only. Launch the same command with --resume and without --prepare-only.'); return
    command = [sys.executable, '-u', str(HERE/'gpu_dataset_sim.py'), '--run-dir', str(root),'--app-threads','8']
    isolated=config['observation_mode']=='isolated_single_environment'
    if isolated and not (root/'observation_complete.json').exists():
        (root/'execution_error.json').unlink(missing_ok=True)
        observation_status=run_worker(command+['--headless','--enable_cameras','--observation-only'],root)
        if observation_status or (root/'execution_error.json').exists() or not (root/'observation_complete.json').exists():
            raise SystemExit('Observation phase failed; see run.log')
    if not isolated:command+=['--enable_cameras']
    (root/'execution_error.json').unlink(missing_ok=True)
    if not args.gui: command += ['--headless']
    if args.rebuild: command += ['--rebuild']
    if args.profile: command += ['--profile']
    if args.benchmark_candidates: command += ['--benchmark-candidates',str(args.benchmark_candidates)]
    status=run_worker(command,root)
    completion = root/('validation_complete.json' if args.validate_only else 'summary.json')
    if status or (root/'execution_error.json').exists() or not completion.exists():
        raise SystemExit(f'Isaac process exited {status}; inspect {root / "run.log"}')
    if not args.validate_only and not json.loads(completion.read_text()).get('execution_complete'):
        raise SystemExit(f'Isaac stopped before this invocation finished; inspect {root / "run.log"}')


if __name__ == '__main__':
    main()
