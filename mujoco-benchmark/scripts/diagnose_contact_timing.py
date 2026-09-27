"""Read-only model diagnostic: align geometry and recomputed forces on private MjData.

Does not alter RobotEngine, solver settings, policy or production classification.
A/B/C states are counterfactual geometry; only D carries recomputed dynamics.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from robot_engine import RobotEngine
from suite import RING


def load_engine(run, candidate="predicted_00000"):
    run = Path(run)
    manifest = json.loads((run / 'manifest.json').read_text())
    assets = run / 'replay_assets'
    engine = RobotEngine(assets / 'model.mjb', assets / 'initial_trace.json', manifest['hz'],
                         reference=assets / 'reference.json', target=manifest['target'])
    folder = run / 'candidates' / candidate
    rows = json.loads((folder / 'trace.json').read_text())
    engine.commands = np.array([r['command'] for r in rows])
    engine.ts = np.arange(len(rows)) / 60
    return engine, rows, json.loads((folder / 'plan.json').read_text())


def force_rows(model, data, pair=None, robot_ids=None):
    """Return all detected contacts, including inactive contacts and positive gaps."""
    output = []
    for i, c in enumerate(data.contact):
        ids = (int(c.geom1), int(c.geom2))
        if pair is not None and set(ids) != set(pair):
            continue
        if robot_ids is not None and not ((ids[0] in robot_ids) != (ids[1] in robot_ids)):
            continue
        force = np.zeros(6)
        mj.mj_contactForce(model, data, i, force)
        output.append(dict(geoms=[model.geom(g).name for g in ids], dist_m=float(c.dist),
                           includemargin_m=float(c.includemargin), efc_address=int(c.efc_address),
                           active=bool(c.efc_address >= 0), normal_force_N=float(force[0]),
                           tangential_force_N=float(np.linalg.norm(force[1:3])), force6=force.tolist()))
    return output


def summarize_contacts(records):
    out = dict(detected_contacts=len(records), active_contacts=0, positive_normal_force_contacts=0,
               positive_distance_force_contacts=0, max_normal_force_N=0., examples=[])
    for r in records:
        out['active_contacts'] += int(r['active'])
        out['positive_normal_force_contacts'] += int(r['normal_force_N'] > 0)
        out['positive_distance_force_contacts'] += int(r['dist_m'] > 0 and r['normal_force_N'] > 0)
        out['max_normal_force_N'] = max(out['max_normal_force_N'], r['normal_force_N'])
    out['examples'] = sorted(records, key=lambda r: r['normal_force_N'], reverse=True)[:8]
    return out


def run_diagnostic(run, pair_names, output, sample_hz=60):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    e, trace, plan = load_engine(run)
    m = e.model
    if m.nplugin:
        raise ValueError('Private forward has not been validated with engine plugins')
    callbacks = {name: getattr(mj, name)() is not None for name in dir(mj) if name.startswith('get_mjcb_')}
    if any(callbacks.values()):
        raise ValueError('Private forward requires no user callbacks: ' + str(callbacks))
    pair = [m.geom(n).id for n in pair_names]
    robot_bodies = set()
    for b in range(1, m.nbody):
        if (m.body(b).name or '').startswith('Robot_') or m.body(b).name == 'Hook' or int(m.body_parentid[b]) in robot_bodies:
            robot_bodies.add(b)
    robot_geoms = {g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in robot_bodies}
    initial = e.data.qpos.copy()
    joint_types = np.array([m.jnt_type[np.flatnonzero(m.jnt_qposadr == q)[0]] for q in e.qids])
    prismatic = joint_types == mj.mjtJoint.mjJNT_SLIDE
    states = [mj.MjData(m) for _ in range(4)]
    for state in states:
        mj.mj_resetData(m, state)
    baseline_hash = hashlib.sha256()
    def baseline_step(model, data):
        baseline_hash.update(data.qpos.tobytes());baseline_hash.update(data.qvel.tobytes())
    baseline = e.rollout(seconds=plan['seconds'], record=False, on_step=baseline_step)
    after_hash = hashlib.sha256()
    rows, saved_times, saved_qpos = [], [], []
    selected_live, all_live, selected_private, all_private = [], [], [], []
    stride = max(1, round(1 / m.opt.timestep / sample_hz))
    count = [0]
    selected_live_min = [.1]
    segment = np.zeros(6)
    def measure(model, data):
        count[0] += 1
        after_hash.update(data.qpos.tobytes());after_hash.update(data.qvel.tobytes())
        # Forces from the previous solve, before the integration that produced data.time.
        live = force_rows(model, data, pair=pair)
        for r in live:r.update(solve_time_s=float(data.time-model.opt.timestep),poststep_time_s=float(data.time))
        selected_live.extend(live)
        other = force_rows(model, data, robot_ids=robot_geoms)
        for r in other:r['solve_time_s']=float(data.time-model.opt.timestep)
        all_live.extend(other)
        selected_live_min[0] = min(selected_live_min[0],float(mj.mj_geomDistance(model,data,*pair,.1,segment)))
        if count[0] % 8 == 0:
            saved_times.append(float(data.time));saved_qpos.append(data.qpos.copy())
        if count[0] % stride:
            return
        A, B, C, D = states
        command = e.command(data.time)
        A.qpos[:] = initial;A.qpos[e.qids] = command
        B.qpos[:] = initial;B.qpos[e.qids] = data.qpos[e.qids]
        C.qpos[:] = data.qpos;C.qpos[e.qids] = command
        for state in (A, B, C):mj.mj_kinematics(model, state)
        mj.mj_copyData(D, model, data)
        mj.mj_forward(model, D)  # Recomputed force at this state; not previous solve output.
        distances = [float(mj.mj_geomDistance(model, state, *pair, .1, segment)) for state in states]
        contacts = force_rows(model, D, pair=pair)
        for r in contacts:r['recomputed_time_s']=float(data.time)
        selected_private.extend(contacts)
        private_other = force_rows(model,D,robot_ids=robot_geoms)
        for r in private_other:r['recomputed_time_s']=float(data.time)
        all_private.extend(private_other)
        error = data.qpos[e.qids]-command
        ra=A.xmat[e.hook].reshape(3,3);rd=D.xmat[e.hook].reshape(3,3)
        pa=A.xpos[e.hook]+ra@RING;pd=D.xpos[e.hook]+rd@RING
        phase=trace[min(int(round(data.time*60)),len(trace)-1)]['phase']
        rows.append(dict(time_s=float(data.time),phase=phase,ABCD_distance_m=distances,
                         prismatic_error_m=float(np.max(abs(error[prismatic]))),
                         revolute_error_rad=float(np.max(abs(error[~prismatic]))),
                         ring_position_error_m=float(np.linalg.norm(pa-pd)),
                         ring_rotation_error_rad=float(Rotation.from_matrix(ra.T@rd).magnitude()),
                         live_pair_contacts_previous_solve=live,recomputed_pair_contacts=contacts))
    result=e.rollout(seconds=plan['seconds'],record=False,on_step=measure)
    assert baseline_hash.hexdigest()==after_hash.hexdigest(), 'Diagnostic changed live trajectory'
    np.savez_compressed(output/'replay_states.npz',times_s=saved_times,qpos=saved_qpos)
    (output/'synchronized_samples.json').write_text(json.dumps(rows))
    (output/'selected_contact_records.json').write_text(json.dumps(dict(live=selected_live,private_forward=selected_private)))
    config=[]
    for g in pair:
        config.append(dict(name=m.geom(g).name,body=m.body(int(m.geom_bodyid[g])).name,
                           margin_m=float(m.geom_margin[g]),gap_m=float(m.geom_gap[g]),priority=int(m.geom_priority[g]),
                           solref=m.geom_solref[g].tolist(),solimp=m.geom_solimp[g].tolist(),
                           contype=int(m.geom_contype[g]),conaffinity=int(m.geom_conaffinity[g])))
    explicit=[]
    for i in range(m.npair):
        if set([int(m.pair_geom1[i]),int(m.pair_geom2[i])])==set(pair):
            explicit.append(dict(index=i,margin_m=float(m.pair_margin[i]),gap_m=float(m.pair_gap[i])))
    summary=dict(source_run=str(run),pair=pair_names,version=mj.__version__,integrator=str(mj.mjtIntegrator(int(m.opt.integrator))),
                 timestep_s=float(m.opt.timestep),sample_hz=sample_hz,callbacks=callbacks,plugins=m.nplugin,
                 config=config,explicit_pairs=explicit,baseline=baseline,diagnostic=result,
                 baseline_qpos_qvel_sha256=baseline_hash.hexdigest(),diagnostic_qpos_qvel_sha256=after_hash.hexdigest(),
                 all_step_selected_pair_min_geometry_m=selected_live_min[0],
                 minimum_ABCD_distance_m=np.min([r['ABCD_distance_m'] for r in rows],axis=0).tolist(),
                 selected_live=summarize_contacts(selected_live),all_robot_environment_live=summarize_contacts(all_live),
                 selected_private=summarize_contacts(selected_private),all_robot_environment_private=summarize_contacts(all_private),
                 scope='A/B/C counterfactual geometry; D private forward at poststep state. Live force belongs to previous solve. Not a passed preflight run or hook-success label.')
    for key in ['prismatic_error_m','revolute_error_rad','ring_position_error_m','ring_rotation_error_rad']:
        summary['max_'+key]=max(r[key] for r in rows)
    (output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(output,'unchanged=True','ABCD min',summary['minimum_ABCD_distance_m'],flush=True)
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--pair',nargs=2,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run_diagnostic(a.run,a.pair,a.output)
