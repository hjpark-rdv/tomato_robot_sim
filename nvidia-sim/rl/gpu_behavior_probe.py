"""Qualitative mechanics checks in the original plant, not harvest success labels.

An isolated CAD ring fixture starts already surrounding the target stem. Its
kinematic pull tests contact retention, not robot reachability or insertion.
No joints, masses, collision filters or break thresholds are weakened here.
"""
import time
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
from omni.physx import get_physx_simulation_interface
from assets import capsule
from geometry import arc_points, RING_CENTER, RING_RADIUS, WIRE_RADIUS
from pose_candidates import rear_capsule_geometry
from dataset_design import write_json


def run_behavior(world, output, app, video=None):
    if len(world.slots)!=1:
        raise ValueError('Isolated contact fixture requires one environment; use motion replay for clone validation')
    slot=world.slots[0];dt=slot.physics_dt
    report=dict(scope='mechanics fixture only; no robot approach/IK or harvest success claim',
                tolerances=dict(idle_drift_m=.003,attachment_gap_m=.0005,
                                recovery_fraction=.25,ring_penetration_m=.001),checks={})
    trace=[];events=[];phase='';tick=0;began=time.monotonic()
    base=slot.elastic.fruit_centers().copy()
    def step(label, force=None):
        nonlocal tick,phase
        phase=label
        if not app.is_running():raise RuntimeError('Simulation closed')
        slot.contact_diagnostic_step=tick
        if force is not None:
            slot.fruit.root_physx_view.apply_forces(torch.tensor([force],dtype=torch.float32),
                                                  torch.tensor([0],dtype=torch.int32),True)
        world.step();tick+=1
        centers=slot.elastic.fruit_centers()
        poses=slot.elastic.poses()
        row=dict(phase=label,time_s=tick*dt,target_displacement_m=float(np.linalg.norm(centers[slot.target_index]-base[slot.target_index])),
                 target_position_xyz=centers[slot.target_index].tolist(),
                 main_displacement_m=float(np.linalg.norm(poses[slot.elastic.chains['STEM_MainStem'],:3]-np.asarray(slot.elastic.rest)[slot.elastic.chains['STEM_MainStem'],:3],axis=1).max()),
                 target_broken=bool(slot._broken[0]),other_broken=bool(slot._other_broken),
                 finite=bool(np.isfinite(centers).all() and np.isfinite(poses).all()))
        if tick%max(1,round(.1/dt))==0:
            row['attachment_gap_m']=slot.elastic.metrics()['max_attachment_gap_m']
        trace.append(row)
        if video: video.sample(row)
        if not row['finite']:raise RuntimeError('Nonfinite plant state')
        return row
    def run_phase(label,seconds,force=None):
        rows=[]
        for _ in range(round(seconds/dt)):
            rows.append(step(label,force))
            if rows[-1]['target_broken'] or rows[-1]['other_broken']:break
        print('[BEHAVIOR]',label,rows[-1],flush=True)
        return rows
    def save():
        report['wall_s']=time.monotonic()-began
        write_json(output/'behavior.json',report);write_json(output/'behavior_trace.json',trace)
        write_json(output/'behavior_contacts.json',events)
    if video:
        video.begin('elastic_recovery', f'{round(1/dt)} Hz | elastic bending and recovery',
            'Applied force test: 0.2 N on fruit, then release. Robot stays at pick-ready.')
    idle=run_phase('idle',5.)
    report['checks']['idle']=dict(passed=len(idle)==round(5./dt) and max(max(r['target_displacement_m'],r['main_displacement_m']) for r in idle)<.003,
                                 last=idle[-1])
    load=run_phase('fruit_load',1.,[.2,0.,0.])
    recovery=run_phase('fruit_recovery',4.)
    peak=max(r['target_displacement_m'] for r in load+recovery)
    gap=max(r.get('attachment_gap_m',0.) for r in idle+load+recovery)
    report['checks']['bending_recovery']=dict(passed=.001<peak<.1 and recovery[-1]['target_displacement_m']<max(.001,.25*peak)
        and not any(r['target_broken'] or r['other_broken'] for r in load+recovery) and gap<.0005,
        peak_m=peak,residual_m=recovery[-1]['target_displacement_m'],max_attachment_gap_m=gap)
    if video: video.finish('bending / recovery: '+str(report['checks']['bending_recovery']['passed']))
    world.reset_all()
    overload=run_phase('overload',1.,[6.,0.,0.])
    report['checks']['native_break']=dict(passed=bool(slot._broken[0]) and not slot._other_broken,
        last=overload[-1],events=[c for c in slot.contact_diagnostics if c['event']=='break'],force_N=6.)
    world.reset_all()
    reset=run_phase('reset_recovery',2.)
    report['checks']['reset']=dict(passed=len(reset)==round(2./dt) and reset[-1]['target_displacement_m']<.003,last=reset[-1])
    save()

    # A separate kinematic copy of the CAD half-ring. Robot stays at pick-ready.
    # The native contact subscriber is independent of the robot label callback.
    path=slot.root+'/MechanicsRing'
    xf=UsdGeom.Xform.Define(slot.stage,path);move=xf.AddTranslateOp();move.Set(Gf.Vec3d(0,0,10))
    xf.AddOrientOp().Set(Gf.Quatf(1.))
    body=UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim());body.CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(.1)
    PhysxSchema.PhysxContactReportAPI.Apply(xf.GetPrim()).CreateThresholdAttr(0.)
    points=np.asarray(arc_points())-np.asarray(RING_CENTER)
    for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
        capsule(slot.stage,path+f'/segment_{i:02d}',a,b,WIRE_RADIUS)
    for i,a in enumerate((points[0],points[-1])):
        capsule(slot.stage,path+f'/rail_{i:02d}',a,[.02840287,a[1],a[2]],WIRE_RADIUS)
    get_physx_simulation_interface().flush_changes()
    view=world.sim.physics_sim_view.create_rigid_body_view(path)
    ids=torch.tensor([0],dtype=torch.int32)
    def pose(p,r,teleport=False):
        data=torch.as_tensor(np.r_[p,r.as_quat()][None],dtype=torch.float32)
        (view.set_transforms if teleport else view.set_kinematic_targets)(data,ids)
    def contacts(headers,data):
        for h in headers:
            a,b=[str(PhysicsSchemaTools.intToSdfPath(p)) for p in (h.collider0,h.collider1)]
            if not (a.startswith(path+'/') or b.startswith(path+'/')):continue
            d=data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]
            if d:
                events.append(dict(phase=phase,tick=tick,a=a,b=b,force_N=float(sum(np.linalg.norm(tuple(p.impulse)) for p in d)/dt)))
    subscription=get_physx_simulation_interface().subscribe_contact_report_events(contacts)
    index=slot.elastic.chains['TRUSS_Pedicel_proximal_'+slot.target_spec['name'][-2:]][-1]
    target_path=slot.elastic.paths[index]+'/StemCollider'
    def target_capsule():
        p=slot.elastic.poses()[index];r=Rotation.from_quat(p[[4,5,6,3]])
        a,b,radius=slot.elastic.rod_shapes[index]
        return p[:3]+r.apply(a),p[:3]+r.apply(b),radius
    report['ring_fixture']=dict(target=target_path,robot_controlled=False,starts_preinserted=True,
        geometry='original 32 half-ring capsules and two rails; no proximal assembly',trials=[])
    try:
        for angle in (0.,90.,180.,270.):
            pose([0,0,10],Rotation.identity(),True);world.reset_all()
            a,b,radius=target_capsule();mid=(a+b)/2;y=(b-a)/np.linalg.norm(b-a)
            z=np.array([0.,0.,1.]);z-=y*np.dot(z,y);z/=np.linalg.norm(z);x=np.cross(y,z)
            r=Rotation.from_matrix(np.column_stack([x,y,z]))*Rotation.from_euler('y',angle,degrees=True)
            x=r.apply([1.,0.,0.]);start=mid+x*(RING_RADIUS-WIRE_RADIUS-radius-.003)
            pose(start,r,True);world.sim.forward();samples=[];event_start=len(events)
            if video:
                video.begin(f'ring_hold_{int(angle):03d}', f'{round(1/dt)} Hz | hook retention | fixture roll {angle:g} deg',
                    'Preinserted kinematic half-ring + rails; NOT robot insertion or harvest success.', (path, view))
            for label,seconds,d0,d1 in [('ring_approach',.8,0.,.008),('ring_hold',1.,.008,.008),
                                        ('ring_release',1.,.008,-.004),('ring_recover',2.,-.004,-.004)]:
                for u in np.linspace(0.,1.,round(seconds/dt)+1)[1:]:
                    p=start+x*(d0+(d1-d0)*u);pose(p,r)
                    row=step(label);gap,seated=rear_capsule_geometry([target_capsule()],p,r)
                    if video:
                        video.trace[-1].update(gap_m=gap, seated=seated,
                            intended_contact=any(e['tick']==tick-1 and target_path in (e['a'],e['b']) for e in events[event_start:]))
                    samples.append(dict(phase=label,gap_m=gap,seated=seated,**{k:row[k] for k in ['target_displacement_m','target_broken','other_broken']}))
                    if row['target_broken'] or row['other_broken']:break
                if samples[-1]['target_broken'] or samples[-1]['other_broken']:break
            held=[s for s in samples if s['phase']=='ring_hold'];ev=events[event_start:]
            target_hits=[e for e in ev if target_path in (e['a'],e['b'])]
            hold_hits=[e for e in target_hits if e['phase']=='ring_hold']
            result=dict(angle_deg=angle,target_contact_count=len(target_hits),hold_target_contact_count=len(hold_hits),
                minimum_gap_m=min(s['gap_m'] for s in samples),hold_seated_fraction=float(np.mean([s['seated'] for s in held])) if held else 0.,
                peak_target_displacement_m=max(s['target_displacement_m'] for s in samples),
                broken=any(s['target_broken'] or s['other_broken'] for s in samples))
            result['retention_passed']=len(held)==round(1./dt) and result['hold_seated_fraction']>.8 and len(hold_hits)>=3 and result['minimum_gap_m']>=-.001 and not result['broken']
            if video: video.finish('fixture retention: '+str(result['retention_passed']), retained_hook=result['retention_passed'])
            report['ring_fixture']['trials'].append(result)
            write_json(output/f'ring_{int(angle)}.json',samples);save()
            print('[BEHAVIOR RING]',result,flush=True)
        report['checks']['ring_retention']=dict(passed=any(r['retention_passed'] for r in report['ring_fixture']['trials']),
            scope='preinserted ring fixture; not an approach solution or guarantee for all orientations')
    finally:
        subscription=None
        pose([0,0,10],Rotation.identity(),True)
        required={'idle','bending_recovery','native_break','reset','ring_retention'}
        report['passed']=required.issubset(report['checks']) and all(c['passed'] for c in report['checks'].values())
        save()
    return report
