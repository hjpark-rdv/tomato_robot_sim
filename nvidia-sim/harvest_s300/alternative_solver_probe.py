"""Bounded Newton solver qualification on the real S300 plant collider assets.

This is a unit fixture, not a new harvest runner. It keeps every source plant
body, inertia, joint and active collider, converts CONNECT attachments through
Newton's official MJCF loop-BALL importer, and applies the recorded plant
preload. It does not claim source-equivalent contact forces or release rules.
A failed idle/attachment test must stop promotion to whole-harvest execution.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.metadata
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import newton
import warp as wp


@wp.kernel
def _external_force(tick: wp.array[int], dt: float, start: float, end: float,
                    body: int, force: wp.vec3, applied: wp.array[wp.spatial_vector]):
    t = float(tick[0]) * dt
    if t >= start and t < end:
        applied[body] = wp.spatial_vector(force, wp.vec3(0.0))


@wp.kernel
def _record(tick: wp.array[int], q: wp.array[wp.transform],
            qd: wp.array[wp.spatial_vector], poses: wp.array2d[wp.transform],
            velocities: wp.array2d[wp.spatial_vector]):
    b = wp.tid()
    poses[tick[0], b] = q[b]
    velocities[tick[0], b] = qd[b]


@wp.kernel
def _tick(tick: wp.array[int]):
    tick[0] += 1


@wp.kernel
def _contact_log(tick: wp.array[int],q: wp.array[wp.transform],shape_body: wp.array[int],
        count: wp.array[int],s0: wp.array[int],s1: wp.array[int],
        p0: wp.array[wp.vec3],p1: wp.array[wp.vec3],normal: wp.array[wp.vec3],
        margin0: wp.array[float],margin1: wp.array[float],probe: int,
        count_log: wp.array2d[int],min_separation: wp.array[float]):
    c=wp.tid()
    if c==0:
        count_log[tick[0],0]=count[0]
    if c<count[0] and c<s0.shape[0] and s0[c]>=0 and s1[c]>=0:
        a=shape_body[s0[c]]
        b=shape_body[s1[c]]
        xa=p0[c]
        xb=p1[c]
        if a>=0:
            xa=wp.transform_point(q[a],xa)
        if b>=0:
            xb=wp.transform_point(q[b],xb)
        signed=wp.dot(normal[c],xb-xa)-margin0[c]-margin1[c]
        wp.atomic_min(min_separation,tick[0],signed)
        if probe>=0 and (a==probe or b==probe):
            wp.atomic_add(count_log,tick[0],1,1)


def _digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        while data := f.read(1 << 20):
            h.update(data)
    return h.hexdigest()


def fold_constant_preload(stiffness, target, preload, q_indices, lower=None, upper=None):
    """Represent the same scalar spring+constant force through an implicit drive."""
    stiffness=np.asarray(stiffness)
    preload=np.asarray(preload)
    q_indices=np.asarray(q_indices)
    result=np.asarray(target).copy()
    remaining=preload.copy()
    spring=stiffness>0
    if np.any(q_indices[spring]<0):
        raise ValueError('A spring drive must map to an actual scalar joint coordinate')
    old=result[q_indices[spring]].copy()
    desired=old+preload[spring]/stiffness[spring]
    if lower is not None and upper is not None:
        desired=np.clip(desired,np.asarray(lower)[spring],np.asarray(upper)[spring])
    result[q_indices[spring]]=desired
    remaining[spring]=preload[spring]-stiffness[spring]*(desired-old)
    return result,remaining


def validated_source_inertia(principal, quaternions_wxyz):
    """Preserve small, valid native tensors instead of applying an absolute floor."""
    from scipy.spatial.transform import Rotation
    principal=np.asarray(principal,dtype=np.float64)
    if (not np.isfinite(principal).all() or np.any(principal<=0) or
            np.any(2*np.max(principal,axis=1)>np.sum(principal,axis=1)*(1+1.e-7))):
        raise ValueError('Source principal inertia must be positive and satisfy triangle inequalities before exact restoration')
    rotation=Rotation.from_quat(np.asarray(quaternions_wxyz)[:,[1,2,3,0]]).as_matrix()
    tensor=(rotation*principal[:,None,:])@np.transpose(rotation,(0,2,1))
    return tensor,np.linalg.inv(tensor)


def extract_plant(source: Path, output: Path, *, probe_center=None) -> dict:
    """Remove robot/background only for a declared plant unit qualification."""
    root = ET.parse(source).getroot()
    world = root.find('worldbody')
    assert world is not None
    probe = None
    if probe_center is not None:
        original = next(b for b in world.iter('body') if b.get('name') == 'Tool_L_belt')
        probe = copy.deepcopy(original)
        probe.set('name', 'Qualification_probe')
        probe.set('pos', ' '.join(map(str, probe_center)))
        probe.set('quat', '1 0 0 0')
        # The real inner-belt box is isolated as a sliding contact-test specimen.
        # It is explicitly not the complete gripper or a harvesting benchmark.
        for child in list(probe):
            if child.tag != 'inertial' and not (child.tag == 'geom' and child.get('type') == 'box'):
                probe.remove(child)
        ET.SubElement(probe, 'joint', name='Qualification_probe_slide', type='slide', axis='1 0 0')
    for child in list(world):
        if child.tag != 'body' or not child.get('name', '').startswith(('Stem_', 'Truss_')):
            world.remove(child)
    if probe is not None:
        world.append(probe)
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'geom' and child.get('contype', '1') == '0' and child.get('conaffinity', '1') == '0':
                parent.remove(child)
    names = {b.get('name') for b in world.iter('body')}
    for section in ['actuator', 'sensor', 'keyframe', 'visual']:
        for child in root.findall(section):
            root.remove(child)
    contact = root.find('contact')
    if contact is not None:
        for c in list(contact):
            if c.tag == 'exclude' and (c.get('body1') not in names or c.get('body2') not in names):
                contact.remove(c)
    meshes = {g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
    asset = root.find('asset')
    if asset is not None:
        for a in list(asset):
            if a.tag != 'mesh' or a.get('name') not in meshes:
                asset.remove(a)
    ET.ElementTree(root).write(output, encoding='unicode')
    return dict(body_count=len(names), active_geom_count=len(list(world.iter('geom'))),
                mesh_assets=len(meshes),
                meaning=('Actual complete target plant plus isolated real inner-belt box on a dynamic slide; '
                         if probe is not None else 'Actual complete target plant only; ')
                        + 'robot/background omitted for unit qualification. Not full-scene throughput.')


def build_probe(args):
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    inputs = Path(args.inputs).resolve()
    report = dict(schema='s300_alternative_solver_qualification_v1', solver=args.solver,
                  vbd_mode=args.vbd_mode if args.solver=='vbd' else None,
                  iterations=args.iterations, dt=args.dt, requested_seconds=args.seconds,
                  source_inputs=str(inputs), source_model_sha256=_digest(inputs / 'input_model.mjb'),
                  source_scene_sha256=_digest(inputs / 'input_scene.xml'),
                  code_sha256=_digest(__file__),
                  versions={name:importlib.metadata.version(name) for name in ['newton','warp-lang','mujoco']},
                  training_eligible=False, harvest_success=None,
                  fixture_only=True, status='preparing',
                  omitted_runtime_features=['belt operation is outside the idle/push fixture',
                    'fruit passive angular spring adapter is not yet ported',
                    'source reaction-force-triggered attachment release is not yet ported'],
                  comparison_limits=['CONNECT changes to Newton BALL-loop constraint with solver-specific compliance',
                    'source armature/friction/effort limits not all supported by XPBD/VBD',
                    'source preload retained; cross-solver accuracy is unvalidated'])
    def save():
        (out / 'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    save()
    try:
        t = time.perf_counter()
        scene = out / 'plant_fixture.xml'
        source = mujoco.MjModel.from_binary_path(str(inputs / 'input_model.mjb'))
        initial = np.load(inputs / 'initial_state.npz')
        source_plant_dofs=np.asarray([d for d in range(source.nv) if
            source.body(int(source.jnt_bodyid[source.dof_jntid[d]])).name.startswith(('Stem_','Truss_','Leaf_'))])
        report['source_joint_feature_audit']=dict(
            native_plant_dof_count=len(source_plant_dofs),
            nonzero_armature_dof_count=int(np.sum(source.dof_armature[source_plant_dofs]!=0)),
            max_armature=float(np.max(source.dof_armature[source_plant_dofs])),
            nonzero_frictionloss_dof_count=int(np.sum(source.dof_frictionloss[source_plant_dofs]!=0)),
            generic_solver_armature_supported=False,
            full_robot_actuator_effort_limit_portability='not established by isolated plant fixture')
        probe_center = None
        if args.coupled:
            target = initial['body_positions'][source.body(args.push_body).id].copy()
            # A 120mm-wide belt specimen must clear neighboring fruits too.
            # Put it to the left of every active plant AABB that overlaps its YZ
            # footprint; target-only radius clearance would be an invalid start.
            check=mujoco.MjData(source);check.qpos[:]=initial['qpos']
            mujoco.mj_forward(source,check)
            obstacle_min_x=[]
            for g in range(source.ngeom):
                if not (source.geom_contype[g] or source.geom_conaffinity[g]):continue
                if not source.body(int(source.geom_bodyid[g])).name.startswith(('Stem_','Truss_')):continue
                rotation=check.geom_xmat[g].reshape(3,3)
                local_aabb=source.geom_aabb[g].reshape(2,3)
                center=check.geom_xpos[g]+rotation@local_aabb[0]
                half=np.abs(rotation)@local_aabb[1]
                if abs(center[1]-target[1])<=half[1]+.06000000285 and abs(center[2]-target[2])<=half[2]+.02600000054:
                    obstacle_min_x.append(float(center[0]-half[0]))
            if not obstacle_min_x:raise ValueError('No source plant AABB in probe footprint')
            box_center=target.copy();box_center[0]=min(obstacle_min_x)-.00249999948-.005
            probe_center=box_center-np.array([-.0195000004023,.0750000025146,.035000000149])
            probe_center[1]+=args.probe_offset_y
            report['probe_geometry']=dict(source_body='Tool_L_belt',source_geom='s300_g0814',
                preserved_mass_kg=float(source.body_mass[source.body('Tool_L_belt').id]),
                world_body_origin=probe_center.tolist(),positive_initial_clearance_m=.005,
                clearance_scope='All source active plant AABBs overlapping belt YZ footprint',
                diagnostic_probe_offset_y_m=args.probe_offset_y,
                scope='Single actual inner-belt box with recorded belt-body inertia on dynamic slide; not whole gripper')
        report['fixture'] = extract_plant(inputs / 'input_scene.xml', scene,probe_center=probe_center)
        report['fixture_sha256'] = _digest(scene)
        builder = newton.ModelBuilder()
        builder.balance_inertia=False
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
        builder.add_mjcf(str(scene), parse_visuals=False, parse_meshes=True, parse_sites=True,
                         parse_mujoco_options=False, collapse_fixed_joints=args.collapse_fixed,
                         convert_mjc_equality_constraints=True,
                         convert_3d_hinge_to_ball_joints=False, ctrl_direct=True,
                         enable_self_collisions=True)
        if args.solver == 'vbd':
            builder.color()
        model = builder.finalize(device='cuda:0')
        body_names = [s.rsplit('/', 1)[-1] for s in model.body_label]
        native_ids = np.asarray([source.body(name if name!='Qualification_probe' else 'Tool_L_belt').id for name in body_names])
        finalizer_inertia=model.body_inertia.numpy().copy()
        source_principal=source.body_inertia[native_ids]
        validated_native_inertia,validated_native_inverse=validated_source_inertia(source_principal,source.body_iquat[native_ids])
        if args.preserve_source_inertia:
            exact_inertia=validated_native_inertia
            if args.collapse_fixed:
                exact_inertia=np.asarray(builder.body_inertia,dtype=np.float64).reshape(-1,3,3)
                if np.min(np.linalg.eigvalsh(exact_inertia))<=0:
                    raise ValueError('Rigidly aggregated source tensors must remain positive before restoration')
            exact_inverse=np.linalg.inv(exact_inertia) if args.collapse_fixed else validated_native_inverse
            model.body_inertia.assign(exact_inertia.astype(np.float32))
            model.body_inv_inertia.assign(exact_inverse.astype(np.float32))
            report['source_inertia_adapter']=dict(
                source_min_principal_kg_m2=float(np.min(source_principal)),
                positive_and_triangle_valid=True,
                finalizer_changed_body_count=int(np.sum(np.any(finalizer_inertia!=exact_inertia.astype(np.float32),axis=(1,2)))),
                max_finalizer_error_kg_m2=float(np.max(np.abs(finalizer_inertia-exact_inertia))),
                restored_source_tensors=True,
                meaning=('Restore official fixed-group aggregate of source tensors and inverse after finalizer inflation; no mass/inertia weakening.'
                    if args.collapse_fixed else 'Restore native positive physical inertia and inverse after generic finalizer absolute-floor inflation; no source mass or inertia weakening.'))
        report['fixed_group_collapse']=dict(enabled=args.collapse_fixed,
            meaning='Official importer merges already-fixed source bodies with compound inertia/colliders; source MJB is unchanged.')
        native_child_ids = native_ids[model.joint_child.numpy()]
        q = model.joint_q.numpy()
        qd = model.joint_qd.numpy()
        preload = np.zeros(model.joint_dof_count, np.float32)
        native_ke = np.zeros(model.joint_dof_count, np.float32)
        native_kd = np.zeros(model.joint_dof_count, np.float32)
        drive_q_indices = np.full(model.joint_dof_count, -1, np.int32)
        spring_q = model.joint_q.numpy().copy()
        q_start = model.joint_q_start.numpy()
        qd_start = model.joint_qd_start.numpy()
        loop_ids = []
        for j, label in enumerate(model.joint_label):
            if label.rsplit('/', 1)[-1].endswith('_attachment'):
                loop_ids.append(j)
                continue
            b = native_child_ids[j]
            count = int(source.body_jntnum[b])
            if not count:
                continue
            native_joint = int(source.body_jntadr[b])
            first_q = int(source.jnt_qposadr[native_joint])
            first_v = int(source.jnt_dofadr[native_joint])
            n_q = q_start[j + 1] - q_start[j]
            n_v = qd_start[j + 1] - qd_start[j]
            values = initial['qpos'][first_q:first_q+n_q].copy()
            if source.jnt_type[native_joint] == mujoco.mjtJoint.mjJNT_FREE:
                values[3:] = values[[4, 5, 6, 3]]
            q[q_start[j]:q_start[j+1]] = values
            qd[qd_start[j]:qd_start[j+1]] = initial['qvel'][first_v:first_v+n_v]
            preload[qd_start[j]:qd_start[j+1]] = initial['plant_preload'][first_v:first_v+n_v]
            if source.jnt_type[native_joint] != mujoco.mjtJoint.mjJNT_FREE:
                assert n_q==n_v==count, f'Unexpected scalar source joint group {label}'
                native_ke[qd_start[j]:qd_start[j+1]]=source.jnt_stiffness[native_joint:native_joint+count]
                native_kd[qd_start[j]:qd_start[j+1]]=initial['runtime_dof_damping'][first_v:first_v+n_v]
                drive_q_indices[qd_start[j]:qd_start[j+1]]=np.arange(q_start[j],q_start[j+1])
                spring_q[q_start[j]:q_start[j+1]]=source.qpos_spring[first_q:first_q+n_q]
        assert len(loop_ids) == 10, f'Expected ten actual fruit attachments, got {len(loop_ids)}'
        model.joint_q.assign(q)
        model.joint_qd.assign(qd)
        newton.eval_fk(model,model.joint_q,model.joint_qd,model)
        state_0, state_1 = model.state(), model.state()
        body_q = np.c_[initial['body_positions'][native_ids], initial['body_quaternions_wxyz'][native_ids][:, [1,2,3,0]]].astype(np.float32)
        if args.coupled:
            probe_body=body_names.index('Qualification_probe')
            body_q[probe_body]=[*probe_center,0.,0.,0.,1.]
        generated_q=state_0.body_q.numpy()
        report['initial_transform_consistency']=dict(
            max_body_translation_error_m=float(np.max(np.linalg.norm(generated_q[:,:3]-body_q[:,:3],axis=1))),
            max_body_mass_error_kg=float(np.max(np.abs(model.body_mass.numpy()-source.body_mass[native_ids]))))
        original_inertia=np.asarray(builder.body_inertia,dtype=np.float32).reshape(-1,3,3)
        inertia_error=model.body_inertia.numpy()-original_inertia
        report['initial_transform_consistency'].update(
            automatic_inertia_balancing=False,
            max_imported_inertia_correction_kg_m2=float(np.max(np.abs(inertia_error))),
            changed_inertia_body_count=int(np.sum(np.any(inertia_error!=0,axis=(1,2)))))
        state_0.body_q.assign(body_q)
        state_1.body_q.assign(body_q)
        # Body velocities start at recorded initial values through FK, then retain
        # exact recorded poses (the initial plant is at zero elastic deflection).
        newton.eval_fk(model, model.joint_q, model.joint_qd, state_0)
        state_0.body_q.assign(body_q)
        state_1.assign(state_0)
        from scipy.spatial.transform import Rotation
        initial_anchor_gaps=[]
        for j in loop_ids:
            bp=int(model.joint_parent.numpy()[j]);bc=int(model.joint_child.numpy()[j])
            xp=model.joint_X_p.numpy()[j,:3];xc=model.joint_X_c.numpy()[j,:3]
            pa=body_q[bp,:3]+Rotation.from_quat(body_q[bp,3:]).apply(xp)
            ca=body_q[bc,:3]+Rotation.from_quat(body_q[bc,3:]).apply(xc)
            initial_anchor_gaps.append(float(np.linalg.norm(pa-ca)))
        report['initial_transform_consistency']['max_attachment_anchor_gap_m']=max(initial_anchor_gaps)
        control = model.control()
        # The MJCF importer currently stores passive damping separately and
        # does not turn MJCF hinge stiffness into generic drives. XPBD/VBD use
        # generic target_ke/target_kd. Bridge the unchanged physical coefficients
        # before evaluating either solver; raw importer-only trials are invalid.
        imported_ke=model.joint_target_ke.numpy()
        imported_kd=model.joint_target_kd.numpy()
        model.joint_target_ke.assign(native_ke)
        model.joint_target_kd.assign(native_kd)
        control.joint_target_q.assign(spring_q)
        report['passive_material_adapter']=dict(
            imported_nonzero_stiffness_dofs=int(np.sum(imported_ke!=0)),
            source_nonzero_stiffness_dofs=int(np.sum(native_ke!=0)),
            imported_nonzero_drive_damping_dofs=int(np.sum(imported_kd!=0)),
            source_nonzero_damping_dofs=int(np.sum(native_kd!=0)),
            meaning='Preserve exact source scalar joint stiffness/damping through supported generic drives; no gain tuning.')
        preload_runtime=preload.copy()
        if args.preload_mode=='implicit-drive':
            stiffness=model.joint_target_ke.numpy()
            target=control.joint_target_q.numpy()
            spring=stiffness>0
            target,preload_runtime=fold_constant_preload(stiffness,target,preload,drive_q_indices,
                model.joint_limit_lower.numpy(),model.joint_limit_upper.numpy())
            control.joint_target_q.assign(target)
            report['preload_adapter']=dict(mode=args.preload_mode,
                implicit_dof_count=int(np.sum(spring & (preload!=0))),
                explicit_remaining_nonzero_dof_count=int(np.sum(preload_runtime!=0)),
                meaning='Constant preload is algebraically folded into the same spring; targets respect unchanged source limits and any clamped remainder stays explicit. No mass/stiffness reduction.')
        else:
            report['preload_adapter']=dict(mode=args.preload_mode)
        control.joint_f.assign(preload_runtime)
        solver_cls = newton.solvers.SolverXPBD if args.solver == 'xpbd' else newton.solvers.SolverVBD
        solver_options={'rigid_compliant_alm':args.vbd_mode=='compliant-alm'} if args.solver=='vbd' else {}
        if args.coupled:
            from newton.solvers.experimental.coupled import SolverCoupled,SolverCoupledADMM
            probe_joints=[j for j,c in enumerate(model.joint_child.numpy()) if c==probe_body]
            plant_bodies=[b for b in range(model.body_count) if b!=probe_body]
            plant_joints=[j for j in range(model.joint_count) if j not in probe_joints]
            solver=SolverCoupledADMM(model=model,entries=[
                SolverCoupled.Entry(name='mjc',solver=lambda v:newton.solvers.SolverMuJoCo(v,
                    use_mujoco_contacts=False,nconmax=128,njmax=256),bodies=[probe_body],joints=probe_joints),
                SolverCoupled.Entry(name=args.solver,solver=lambda v:solver_cls(v,iterations=args.iterations,**solver_options),
                    bodies=plant_bodies,joints=plant_joints)],
                coupling=SolverCoupledADMM.Config(iterations=args.coupling_iterations,
                    rho=200.,gamma=.001,baumgarte=.5,contact_pairs=[
                        SolverCoupledADMM.ContactPair(source='mjc',destination=args.solver)]))
            report['coupling']=dict(kind='official SolverCoupledADMM',iterations=args.coupling_iterations,
                robot_specimen_solver='MuJoCo Warp GPU',plant_solver=args.solver,
                bidirectional_contact_required=True)
        else:
            solver = solver_cls(model, iterations=args.iterations,**solver_options)
        collision = newton.CollisionPipeline(model)
        contacts = collision.contacts()
        if args.coupled:
            solver.prepare_contacts(contacts)
        steps = max(2, round(args.seconds / args.dt))
        tick = wp.zeros(1, dtype=int)
        poses = wp.zeros((steps, model.body_count), dtype=wp.transform)
        velocities = wp.zeros((steps, model.body_count), dtype=wp.spatial_vector)
        contact_counts=wp.zeros((steps,2),dtype=int)
        contact_separation=wp.full(steps,1.e6,dtype=float)
        fruit = body_names.index(args.push_body)
        forced_body=probe_body if args.coupled else fruit
        push_start=args.push_start if args.push_start is not None else args.seconds*.25
        push_end=args.push_end if args.push_end is not None else args.seconds*.5
        report.update(status='building_graph', initialization_wall_s=time.perf_counter()-t,
                      model_body_count=model.body_count, model_joint_count=model.joint_count,
                      model_shape_count=model.shape_count, model_dof_count=model.joint_dof_count,
                      loop_joint_count=len(loop_ids), loop_joint_labels=[model.joint_label[j] for j in loop_ids],
                      collider_shape_count=int(np.sum((model.shape_flags.numpy() & int(newton.ShapeFlags.COLLIDE_SHAPES)) != 0)),
                      push_body=body_names[forced_body], push_force_n=[args.push_force,0,0],
                      push_interval_s=[push_start,push_end],
                      source_gravity=source.opt.gravity.tolist(), imported_gravity=model.gravity.numpy().tolist())
        save()
        def step():
            nonlocal state_0, state_1
            state_0.clear_forces()
            wp.launch(_external_force, dim=1, inputs=[tick,args.dt,push_start,push_end,
                      forced_body,wp.vec3(args.push_force,0,0),state_0.body_f])
            collision.collide(state_0, contacts)
            wp.launch(_contact_log,dim=contacts.rigid_contact_shape0.shape[0],inputs=[tick,state_0.body_q,
                model.shape_body,contacts.rigid_contact_count,contacts.rigid_contact_shape0,
                contacts.rigid_contact_shape1,contacts.rigid_contact_point0,contacts.rigid_contact_point1,
                contacts.rigid_contact_normal,contacts.rigid_contact_margin0,contacts.rigid_contact_margin1,
                probe_body if args.coupled else -1,contact_counts,contact_separation])
            solver.step(state_0,state_1,control,contacts,args.dt)
            wp.launch(_record,dim=model.body_count,inputs=[tick,state_1.body_q,state_1.body_qd,poses,velocities])
            wp.launch(_tick,dim=1,inputs=[tick])
            state_0,state_1=state_1,state_0
        # Capture an even number of steps so graph buffer identities remain stable.
        block= min(20, steps//2*2)
        t=time.perf_counter()
        with wp.ScopedCapture() as cap:
            for _ in range(block): step()
        wp.synchronize()
        report['compile_capture_wall_s']=time.perf_counter()-t
        tick.zero_()
        state_0.body_q.assign(body_q); state_1.body_q.assign(body_q)
        newton.eval_fk(model,model.joint_q,model.joint_qd,state_0)
        state_0.body_q.assign(body_q); state_1.assign(state_0)
        wp.synchronize()
        t=time.perf_counter()
        for _ in range(steps//block): wp.capture_launch(cap.graph)
        for _ in range(steps%block): step()
        wp.synchronize()
        elapsed=time.perf_counter()-t
        p=poses.numpy(); v=velocities.numpy()
        cc=contact_counts.numpy();cs=contact_separation.numpy()
        finite=bool(np.isfinite(p).all() and np.isfinite(v).all())
        displacement=np.linalg.norm(p[:,:,:3]-body_q[None,:,:3],axis=-1)
        idle_steps=max(1,round(push_start/args.dt))
        max_idle=float(np.max(displacement[:idle_steps])) if finite and push_start>0 else None
        jp=model.joint_parent.numpy(); jc=model.joint_child.numpy()
        xp=model.joint_X_p.numpy(); xc=model.joint_X_c.numpy()
        gaps=[]
        if finite:
            from scipy.spatial.transform import Rotation
            for j in loop_ids:
                parent=p[:,jp[j],:3]+Rotation.from_quat(p[:,jp[j],3:]).apply(np.tile(xp[j,:3],(steps,1)))
                child=p[:,jc[j],:3]+Rotation.from_quat(p[:,jc[j],3:]).apply(np.tile(xc[j,:3],(steps,1)))
                gaps.append(np.linalg.norm(parent-child,axis=-1))
            gap=np.array(gaps).T
        else:
            gap=np.full((steps,len(loop_ids)),np.nan)
        report.update(status='finished', physics_steps=steps, physics_seconds=steps*args.dt,
                      physics_wall_s=elapsed,fixture_RTF=steps*args.dt/elapsed,finite=finite,
                      max_idle_body_displacement_m=max_idle,
                      max_attachment_gap_m=float(np.max(gap)) if finite else None,
                      max_idle_attachment_gap_m=float(np.max(gap[:idle_steps])) if finite and push_start>0 else None,
                      push_body_max_displacement_m=float(np.max(displacement[:,fruit])),
                      push_body_final_displacement_m=float(displacement[-1,fruit]),
                      max_generated_contact_count=int(np.max(cc[:,0])),
                      steps_with_probe_contact=int(np.sum(cc[:,1]>0)),
                      max_pre_step_contact_penetration_m=float(max(0.,-np.min(cs))),
                      contact_distance_timing='Collision generation on state_in before solver integration; body_q saved after integration.',
                      full_harvest_eligible=False,
                      qualification='reject_nonfinite' if not finite else
                        'reject_attachment_gap' if np.max(gap)>.001 else
                        'reject_active_penetration' if -np.min(cs)>.001 else
                        'fixture_valid_response_comparison_pending_runtime_features')
        bad=np.flatnonzero(~np.isfinite(p).all((1,2)) | ~np.isfinite(v).all((1,2)))
        report['first_nonfinite_step']=int(bad[0]) if len(bad) else None
        report['first_nonfinite_time_s']=float((bad[0]+1)*args.dt) if len(bad) else None
        if args.coupled and finite:
            report['probe_final_displacement_x_m']=float(p[-1,probe_body,0]-body_q[probe_body,0])
            report['probe_final_velocity_x_m_s']=float(v[-1,probe_body,0])
        np.savez_compressed(out/'actual_states.npz',time=np.arange(1,steps+1)*args.dt,
                            body_names=np.array(body_names),body_q=p,body_qd=v,initial_body_q=body_q,
                            attachment_gap_m=gap,loop_joint_labels=np.array([model.joint_label[j] for j in loop_ids]),
                            contact_counts=cc,pre_step_min_contact_separation_m=cs)
        save()
    except Exception as exc:
        report.update(status='error',error_type=type(exc).__name__,error=str(exc))
        save()
        raise
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--solver',choices=['xpbd','vbd'],required=True)
    parser.add_argument('--iterations',type=int,default=16)
    parser.add_argument('--dt',type=float,default=.0025)
    parser.add_argument('--seconds',type=float,default=1.)
    parser.add_argument('--push-body',default='Truss_00_Fruit_05')
    parser.add_argument('--push-force',type=float,default=.05)
    parser.add_argument('--coupled',action='store_true',help='Official two-way MuJoCo↔plant ADMM specimen fixture')
    parser.add_argument('--coupling-iterations',type=int,default=2)
    parser.add_argument('--preload-mode',choices=['explicit','implicit-drive'],default='implicit-drive')
    parser.add_argument('--allow-finalizer-inertia',dest='preserve_source_inertia',action='store_false',default=True)
    parser.add_argument('--collapse-fixed',action='store_true')
    parser.add_argument('--push-start',type=float)
    parser.add_argument('--push-end',type=float)
    parser.add_argument('--probe-offset-y',type=float,default=0.)
    parser.add_argument('--vbd-mode',choices=['legacy','compliant-alm'],default='legacy')
    args=parser.parse_args()
    print(json.dumps(build_probe(args),indent=2,ensure_ascii=False))


if __name__=='__main__': main()
