"""Small paired low-tilt candidate batch; no physics and no Sobol explosion.

The existing motion family geometry is reused. Changing a tilt is a separate
matched batch, not a joint change of azimuth/elevation/roll/pitch/goal selectors.
"""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path


def configs(azimuths=(-45.,0.,45.), tilt_axis='none', tilt_deg=0.):
    import math
    if tilt_axis not in ('none','roll','pitch','elevation'):
        raise ValueError('Unknown ablation axis')
    if not math.isfinite(tilt_deg) or abs(tilt_deg)>20 or (tilt_axis=='none' and tilt_deg!=0):
        raise ValueError('Use an explicit single-axis tilt of at most 20 degrees')
    if not azimuths or len(azimuths)>8 or len(set(azimuths))!=len(azimuths):
        raise ValueError('Choose 1..8 distinct azimuths')
    result=[]
    for a in azimuths:
        if isinstance(a,bool) or not math.isfinite(a) or not -110<=a<=110:
            raise ValueError('Azimuth outside existing search range')
        p=dict(azimuth_deg=float(a),elevation_deg=0.,roll_deg=0.,pitch_deg=0.,
            pre_distance_m=.16,under_clearance_m=.012,lateral_m=0.,sweep_m=.045,
            target_fraction=.5,seat_gap_m=.0007,verify_m=.003,entry_twist_deg=0.,
            target_selector=.25,wire_selector=.5)
        if tilt_axis!='none':p[tilt_axis+'_deg']=float(tilt_deg)
        result.append(p)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source_run',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--families',nargs='+',default=['under_center','side_mouth'])
    p.add_argument('--azimuths',nargs='+',type=float,default=[-45.,0.,45.])
    p.add_argument('--tilt-axis',choices=['none','roll','pitch','elevation'],default='none')
    p.add_argument('--tilt-deg',type=float,default=0.)
    a=p.parse_args()
    from motion_family_search import FAMILIES,make_candidate
    from run_motion_family_search import REQUIRED,file_hash,read,write,prepare_snapshot,native_geometry
    if not a.families or not set(a.families)<=set(FAMILIES) or len(set(a.families))!=len(a.families):
        p.error('Choose distinct existing families')
    values=configs(a.azimuths,a.tilt_axis,a.tilt_deg)
    source=a.source_run.resolve();out=a.output.resolve()
    if out.exists() or source==out or source in out.parents:p.error('Use new output outside source')
    hashes={name:file_hash(source/name) for name in REQUIRED}
    if read(source/'manifest.json').get('model_sha256')!=hashes['replay_assets/model.mjb']:
        raise ValueError('Source model hash mismatch')
    out.mkdir(parents=True)
    robot=out/'robot';manifest=prepare_snapshot(dict(source_run=str(source)),robot)
    geometry,policy=native_geometry(robot)
    candidates=[];rejections=[]
    for i,v in enumerate(values):
        for f in a.families:
            cid=f'access_{i:02d}_{f}'
            par=copy.deepcopy(v)
            if f=='side_mouth':par['robot_facing_mouth']=True
            try:
                row=make_candidate(f,par,**geometry,candidate_id=cid)
                row['access_ablation']=dict(azimuth_group=i,tilt_axis=a.tilt_axis,tilt_deg=a.tilt_deg)
                candidates.append(row)
            except ValueError as exc:
                rejections.append(dict(candidate_id=cid,reason=str(exc)))
    write(robot/'candidates.json',candidates)
    manifest.update(count=len(candidates),plan_only=True,experiment_schema='access_ablation_v1')
    write(robot/'manifest.json',manifest);write(out/'base_policy.json',policy)
    write(out/'preparation.json',dict(candidates=len(candidates),rejections=rejections,
        source_run=str(source),source_sha256=hashes,
        source_unchanged=all(file_hash(source/name)==h for name,h in hashes.items()),
        sampling='paired_low_tilt_not_independent_random_samples',
        physics_executed=False,training_eligible=False,hook_success=None,impossible=None))
    print(robot)

if __name__=='__main__':main()
