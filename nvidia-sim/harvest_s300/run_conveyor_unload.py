"""Newton GPU contact test for the concept conveyor bin, separate from the arm."""
from __future__ import annotations
import argparse,json,math,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
import warp as wp
from mujoco_warp._src.types import vec6,OverflowType
from preview_conveyor_bin import conveyor_bin_spec
from newton_mjcf import create_newton_mjcf
from newton_inputs import sha256


@wp.func
def belt_speed(t:float,enabled:int):
    return 0.25*wp.clamp((t-2.2)/.5,0.0,1.0)*float(enabled)


@wp.kernel
def actuate(clock:wp.array[float],ctrl:wp.array2d[float],radius:float,enabled:int):
    i=wp.tid();t=clock[0]
    if i==0:
        u=wp.clamp(t-1.0,0.0,1.0);s=u*u*(3.0-2.0*u)
        ctrl[0,i]=-1.919862177*s
    else:ctrl[0,i]=belt_speed(t,enabled)/radius


@wp.kernel
def set_surface(clock:wp.array[float],surface:wp.array2d[vec6],geom:int,enabled:int):
    surface[0,geom]=vec6(0.0,-belt_speed(clock[0],enabled),0.0,0.0,0.0,0.0)


def fixture(path,spec):
    root=ET.Element('mujoco',model='tomato_conveyor_unload')
    ET.SubElement(root,'compiler',angle='radian',autolimits='true')
    ET.SubElement(root,'option',timestep='.001',gravity='0 0 -9.81',integrator='implicitfast',
        solver='Newton',iterations='30',ls_iterations='30',jacobian='sparse',cone='elliptic')
    default=ET.SubElement(root,'default')
    ET.SubElement(default,'geom',friction='.8 .008 .012',condim='6',solref='.004 1',solimp='.95 .99 .001')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'geom',name='ground',type='plane',size='4 4 .1',friction='.8 .008 .001')
    for p in spec['parts']:
        if p.get('gate')or p.get('grip_only'):continue
        ET.SubElement(world,'geom',name=p['name'],type='box',pos=' '.join(map(str,p['position'])),
            size=' '.join(str(x/2)for x in p['dimensions']))
    door=ET.SubElement(world,'body',name='discharge_door',pos=' '.join(map(str,spec['gate_hinge_m'])))
    ET.SubElement(door,'joint',name='door_hinge',type='hinge',axis='1 0 0',range='-2.05 0',damping='.15')
    for p in spec['gate_parts']:
        ET.SubElement(door,'geom',name=p['name'],type='box',pos=' '.join(map(str,p['position'])),
            size=' '.join(str(x/2)for x in p['dimensions']),density='400')
    actuators=ET.SubElement(root,'actuator')
    ET.SubElement(actuators,'position',joint='door_hinge',kp='50',kv='6',forcerange='-8 8')
    for i,y in enumerate(spec['roller_y_m']):
        body=ET.SubElement(world,'body',name=f'roller_{i}',pos=f'0 {y} {spec["roller_z_m"]}')
        ET.SubElement(body,'joint',name=f'roller_joint_{i}',type='hinge',axis='1 0 0',limited='false',damping='.001')
        ET.SubElement(body,'geom',type='cylinder',size=f'{spec["roller_radius_m"]} {spec["roller_length_m"]/2}',
            euler=f'0 {math.pi/2} 0',mass='.20',contype='0',conaffinity='0')
        ET.SubElement(actuators,'velocity',joint=f'roller_joint_{i}',kv='.08',forcerange='-1.2 1.2')
    fruit=[]
    for layer in range(2):
        for x in np.linspace(-.20,.20,6):
            for y in np.linspace(-.365,.535,10):
                name=f'tomato_{len(fruit):03d}';fruit.append(name)
                body=ET.SubElement(world,'body',name=name,pos=f'{x} {y} {spec["belt_top_m"]+.020+layer*.037}')
                ET.SubElement(body,'freejoint',name=name+'_free')
                ET.SubElement(body,'geom',type='sphere',size='.018',mass='.020',
                    friction='.8 .008 .012',condim='6')
    ET.ElementTree(root).write(path,encoding='unicode')
    return fruit


def run(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    spec=conveyor_bin_spec();scene=output/'unload.xml';fruit=fixture(scene,spec)
    source=mj.MjModel.from_xml_path(str(scene));source.flg_surfacevel=True
    engine=create_newton_mjcf(scene,source_model=source,device='cuda:0',
        nconmax=4096,njmax=8192,njmax_nnz=131072,native_observables_only=True,native_control_callback=True)
    assert not engine.solver.use_mujoco_cpu
    model=engine.solver.mjw_model;data=engine.solver.mjw_data
    belt_id=source.geom('belt').id;fruit_ids=np.array([source.body(n).id for n in fruit])
    bin_geom_ids=np.array([g for g in range(source.ngeom)
        if source.geom_bodyid[g]in (0,source.body('discharge_door').id)and g!=source.geom('ground').id])
    def control(m,d):wp.launch(actuate,source.nu,inputs=[d.time,d.ctrl,spec['roller_radius_m'],1],device='cuda:0')
    model.callback.control=control
    from newton_gpu_session import NewtonGPUSessionCache
    initial=dict(qpos=source.qpos0.copy(),qvel=np.zeros(source.nv),eq_active=np.zeros(source.neq,dtype=bool),ctrl=np.zeros(source.nu))
    cache=NewtonGPUSessionCache()
    signature=dict(scene_sha256=sha256(scene),backend='cuda',dt=.001)
    # Register the fresh engine so both controlled trials reuse one upload/JIT.
    engine,_=cache.acquire(signature,lambda:engine,initial)
    trials={};dt=.001;batch=40
    for enabled in (0,1):
        engine,_=cache.acquire(signature,lambda:engine,initial)
        model=engine.solver.mjw_model;data=engine.solver.mjw_data
        def control(m,d,enabled=enabled):
            wp.launch(actuate,source.nu,inputs=[d.time,d.ctrl,spec['roller_radius_m'],enabled],device='cuda:0')
        model.callback.control=control
        # The prescribed belt velocity is installed before contact assembly.
        started=time.perf_counter()
        with wp.ScopedCapture(device='cuda:0')as cap:
            for _ in range(batch):
                wp.launch(set_surface,1,inputs=[data.time,model.geom_surfacevel,belt_id,enabled],device='cuda:0')
                engine.step(dt)
        wp.synchronize_device('cuda:0');compile_s=time.perf_counter()-started
        rows=[];times=[];gates=[];rollers=[];contacts=[];max_pen=0.;max_bin_pen=0.;flags=0;wall=time.perf_counter()
        # Stop when every sphere has crossed the outlet. Continued settling
        # of an external heap is outside this bin-discharge test.
        steps=4000 if not enabled else 12000
        all_crossed=np.zeros(len(fruit),bool)
        for i in range(steps//batch):
            wp.capture_launch(cap.graph)
            q=data.qpos.numpy()[0];v=data.qvel.numpy()[0]
            if not np.isfinite(np.r_[q,v]).all():raise ValueError('Nonfinite unloading physics state')
            raw=int(np.bitwise_or.reduce(data.overflow.numpy().ravel(),initial=0));flags|=raw
            capacity_mask=int(OverflowType.ALL&~(OverflowType.ITERATIONS|OverflowType.LS_ITERATIONS))
            if raw&capacity_mask:raise ValueError('Unloading contact/constraint buffer overflow')
            t=float(data.time.numpy()[0]);n=int(data.nacon.numpy()[0])
            if n:
                distances=data.contact.dist.numpy()[:n];pairs=data.contact.geom.numpy()[:n]
                max_pen=max(max_pen,float(-distances.min(initial=0.)))
                in_bin=np.isin(pairs,bin_geom_ids).any(axis=1)
                max_bin_pen=max(max_bin_pen,float(-distances[in_bin].min(initial=0.)))
            positions=data.xpos.numpy()[0,fruit_ids]
            rows.append(positions);times.append(t);gates.append(q[0]);rollers.append(q[1:32]);contacts.append(n)
            all_crossed|=(positions[:,1]<-.46)&(positions[:,2]>spec['belt_top_m']-.035)
            if enabled and all_crossed.all():break
            if i%100==99:print(f'Belt {enabled}: {t:.2f}sim seconds',flush=True)
        wp.synchronize_device('cuda:0');elapsed=time.perf_counter()-wall
        rows=np.asarray(rows)
        crossing=(rows[:,:,1]<-.46)&(rows[:,:,2]>spec['belt_top_m']-.035)
        crossed=crossing.any(axis=0)
        # A fruit which has already left may bounce back under the chassis;
        # only crossing the floor BEFORE leaving is an unintended leak.
        was_discharged=np.maximum.accumulate(crossing,axis=0)
        fell_inside=((rows[:,:,2]<spec['belt_top_m']-.05)&(rows[:,:,1]>-.42)&
            (np.abs(rows[:,:,0])<.25)&~was_discharged).any(axis=0)
        result=dict(belt_enabled=bool(enabled),fruit_count=len(fruit),discharged=int(crossed.sum()),
            unintended_underfloor_leaks=int(fell_inside.sum()),maximum_bin_contact_penetration_m=max_bin_pen,
            simulated_seconds=times[-1],wall_seconds=elapsed,rtf=times[-1]/elapsed,
            jit_and_capture_seconds=compile_s,maximum_penetration_m=max_pen,maximum_contacts=max(contacts),
            finite=True,buffer_overflow=False,physics_backend='Newton → MuJoCo Warp CUDA')
        result['iteration_warning_flags']=flags
        result['valid']=bool(not fell_inside.any() and max_bin_pen<.001 and (not enabled or crossed.all()))
        trials['belt_on'if enabled else'belt_off']=result
        (output/('belt_on_report.json'if enabled else'belt_off_report.json')).write_text(json.dumps(result,indent=2)+'\n')
        if enabled:
            np.savez_compressed(output/'unload_states.npz',time=np.array(times),tomato_positions=rows,
                gate_angle=np.array(gates),roller_angles=np.array(rollers),
                discharged_count=was_discharged.sum(axis=1),
                belt_distance=.25*np.clip(np.array(times)-2.2,0.,.5)**2+.25*np.maximum(0.,np.array(times)-2.7))
        print(json.dumps(result),flush=True)
    evidence=dict(schema='s300-conveyor-unload-gpu-v1',engine=engine.evidence,layout=spec,
        source_xml_sha256=sha256(scene),trials=trials,fruit_radius_m=.018,fruit_mass_kg=.020,
        belt_speed_m_s=.25,gate_command_deg=110.,physics_dt_s=dt,
        state_file='unload_states.npz',state_sha256=sha256(output/'unload_states.npz'),
        scope='Separate bin-only GPU rigid-sphere discharge fixture; not full robot dynamics, fruit damage or truss mechanics')
    (output/'unload_report.json').write_text(json.dumps(evidence,indent=2)+'\n')
    if not all(r['valid']for r in trials.values()):raise ValueError('Unloading fixture failed physical validity checks; inspect report')
    return evidence


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True)
    run(p.parse_args().output)
