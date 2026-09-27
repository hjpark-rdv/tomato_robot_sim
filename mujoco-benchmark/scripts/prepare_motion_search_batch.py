"""Create a bounded random-scene campaign with the EXISTING generator/snapshotter.

No old motion rollout is run. Invalid idle scenes stay in the manifest; selected
fruit snapshots retain the old physical model, controller and camera metadata.
The generated cases can be passed directly to run_motion_family_search.py.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import sys

from run_motion_family_search import read, write, run_command, file_hash, SCRIPTS
from motion_family_search import SCHEMA


def prepare(a):
    output=a.output.resolve()
    if output.exists():raise ValueError('Use a new preparation output')
    if not 1<=a.scenes<=64 or not 1<=a.trusses<=4 or not 0<=a.targets_per_scene<=40:
        raise ValueError('Invalid explicit scene/target budget')
    if not a.planning_model.is_file():raise ValueError('Existing exported planning_model.pkl required')
    output.mkdir(parents=True)
    command=[a.native_python,str(SCRIPTS/'generate_random_glb_scenes.py'),'--output',str(output/'scenes'),
             '--source-dir',str(a.source_dir.resolve()),'--seed',str(a.seed),'--scenes',str(a.scenes),
             '--trusses',str(a.trusses),'--segment-min',str(a.segment_min),'--segment-max',str(a.segment_max),
             '--angle-min',str(a.angle_min),'--angle-max',str(a.angle_max),'--truss-scale',str(a.truss_scale),
             '--no-render']
    if a.gutter_collisions:command.append('--gutter-collisions')
    code=run_command(command,output/'generation.log',a.timeout_s)
    if code:raise RuntimeError(f'Existing scene generator failed ({code}); see generation.log')
    manifest=read(output/'scenes/manifest.json')
    cases,failures,unselected=[],[],[]
    for record in manifest['records']:
        scene=record['scene'];path=output/'scenes'/scene
        if not record['validation']['scene_screen_passed']:
            failures.append(dict(scene=scene,reason='initial_scene_screen_failed',validation=record['validation']))
            continue
        specs=read(path/'reference.json')['fruit_specs']
        chosen=specs
        if a.targets_per_scene and len(specs)>a.targets_per_scene:
            # Evenly spread deterministic indices, not selection on successful motions.
            indices={i*len(specs)//a.targets_per_scene for i in range(a.targets_per_scene)}
            chosen=[s for i,s in enumerate(specs) if i in indices]
            unselected.extend(dict(scene=scene,target=s['name'],reason='declared_target_budget') for i,s in enumerate(specs) if i not in indices)
        for spec in chosen:
            target=spec['name'];run=output/scene/'targets'/target/'physics'
            command=[a.native_python,str(SCRIPTS/'candidate_experiment.py'),'--scene-only','--target',target,
                     '--model',str(path/'model.mjb'),'--reference',str(path/'reference.json'),
                     '--planning-model',str(a.planning_model.resolve()),'--output',str(run),
                     '--hz',str(a.hz),'--candidates','1','--workers','1','--planning-workers','1','--link-model']
            code=run_command(command,output/f'{scene}_{target}_snapshot.log',a.timeout_s)
            if code:
                failures.append(dict(scene=scene,target=target,reason='snapshot_error',returncode=code));continue
            cases.append(dict(scene_id=scene,target=target,source_run=str(run),split='pilot',
                              collision_coverage='Existing generator collider coverage only; visual-only objects untested'))
            write(output/'cases.partial.json',dict(schema=SCHEMA,cases=cases,source_scene_failures=failures))
    result=dict(schema=SCHEMA,cases=cases,source_scene_failures=failures,unselected_targets=unselected,
                requested_scenes=a.scenes,generated_scenes=len(manifest['records']),
                generator_manifest_sha256=file_hash(output/'scenes/manifest.json'),
                scope='Pilot generation, no physics motion labels; scene geometry may vary, physical settings retained',
                rgbd_collection='Not performed here. Reuse existing RGB-D collector after rollout results exist; no mask/label format changes.')
    write(output/'cases.json',result)
    return output/'cases.json'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source-dir',type=Path,required=True)
    p.add_argument('--planning-model',type=Path,required=True)
    p.add_argument('--native-python',default=sys.executable)
    p.add_argument('--scenes',type=int,default=3)
    p.add_argument('--trusses',type=int,default=1)
    p.add_argument('--targets-per-scene',type=int,default=0,help='0=all; otherwise fixed evenly-spaced selection')
    p.add_argument('--seed',type=int,default=20260928)
    p.add_argument('--segment-min',type=int,default=6)
    p.add_argument('--segment-max',type=int,default=10)
    p.add_argument('--angle-min',type=float,default=0.)
    p.add_argument('--angle-max',type=float,default=180.)
    p.add_argument('--truss-scale',type=float,default=.5)
    p.add_argument('--hz',type=int,default=240)
    p.add_argument('--gutter-collisions',action='store_true')
    p.add_argument('--timeout-s',type=float,default=1800.)
    a=p.parse_args()
    if a.hz<1 or a.timeout_s<=0:p.error('Hz and timeout must be positive')
    print(prepare(a))


if __name__=='__main__':main()
