"""Opt-in plant coordinate experiments; preserve original inputs and collision geometry.

Leaf locking is a changed dynamic model, not an exact execution optimization.
Coarse rod variants fit positive spring compliance against the original small-
deflection Jacobians before any harvest outcome is observed. Native CPU forward
and Jacobian calls are offline model preparation, never GPU throughput evidence.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.optimize import lsq_linear

from newton_inputs import ADAPTER, DOF_NAMES, sha256
from run_smoke import Attachments


def _is_leaf(name):
    return bool(re.match(r"^(?:.*_)?Leaf_", name))


def _plant_scope(name):
    """Identify source-window/truss chains without mixing matching suffixes."""
    stem = re.fullmatch(r"(.*?)Stem_MainStem_(\d+)", name)
    if stem:
        return stem.group(1), "stem", int(stem.group(2))
    truss = re.fullmatch(r"(.*?)(Truss_\d+)_(.+)", name)
    if not truss:
        return None
    prefix, truss_key, suffix = truss.groups()
    if "Pedicel_" in suffix:
        match = re.search(r"_(\d+)_\d+$", name)
        if not match:
            raise ValueError("Unknown pedicel segment identity: " + name)
        return prefix, truss_key + ":pedicel_" + match.group(1), None
    if "Peduncle_" in suffix:
        return prefix, truss_key + ":peduncle", int(suffix.rsplit("_", 1)[1])
    if "Rachis_" in suffix:
        return prefix, truss_key + ":rachis", None
    return None


def selected_bodies(model, mode):
    """Select whole segments independently in every named plant/truss chain."""
    if mode not in ('leaves_fixed', 'coarse2', 'coarse4'):
        raise ValueError('Unknown coordinate variant: '+mode)
    removed = set()
    chains = {}
    for b in range(1, model.nbody):
        name = model.body(b).name
        if _is_leaf(name):
            if np.any(((model.geom_contype | model.geom_conaffinity) != 0)
                      & (model.geom_bodyid == b)):
                raise ValueError('Leaf locking requires noncolliding leaves: ' + name)
            removed.add(name)
        elif mode.startswith('coarse') and model.body_jntnum[b]:
            scope = _plant_scope(name)
            if scope is not None:
                chains.setdefault(scope[:2], []).append(name)
    stride = int(mode.removeprefix('coarse')) if mode.startswith('coarse') else 1
    for chain in chains.values():
        removed.update(name for index, name in enumerate(chain) if index % stride)
    return removed


def sentinel_bodies(model, metadata):
    """Use each window's final stem/peduncle and every physical fruit anchor."""
    stems, peduncles = {}, {}
    for b in range(1, model.nbody):
        name = model.body(b).name
        scope = _plant_scope(name)
        if scope is None:
            continue
        prefix, chain, index = scope
        destination = stems if chain == "stem" else peduncles if chain.endswith(":peduncle") else None
        if destination is not None:
            key = prefix if destination is stems else (prefix, chain)
            if key not in destination or index > destination[key][0]:
                destination[key] = (index, name)
    names = [item[1] for item in stems.values()] + [item[1] for item in peduncles.values()]
    names.extend(spec["anchor"] for spec in metadata["attachments"])
    if not stems or not peduncles:
        raise ValueError("Coordinate compliance fit requires declared stem and peduncle endpoints")
    for name in names:
        if model.body(name).id <= 0:
            raise ValueError("Compliance sentinel does not exist: " + name)
    return names


def joint_mapping(old, new):
    qp, qv = [], []
    for j in range(new.njnt):
        source = old.joint(new.joint(j).name).id
        typ = int(new.jnt_type[j])
        nq = 7 if typ == mj.mjtJoint.mjJNT_FREE else 4 if typ == mj.mjtJoint.mjJNT_BALL else 1
        nv = 6 if typ == mj.mjtJoint.mjJNT_FREE else 3 if typ == mj.mjtJoint.mjJNT_BALL else 1
        qp.extend(range(int(old.jnt_qposadr[source]), int(old.jnt_qposadr[source])+nq))
        qv.extend(range(int(old.jnt_dofadr[source]), int(old.jnt_dofadr[source])+nv))
    return np.array(qp), np.array(qv)


def compliance(model, data, names):
    jac = []
    for name in names:
        jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        b = model.body(name).id
        mj.mj_jac(model, data, jp, jr, data.xpos[b], b)
        jac.extend(jp)
    jac = np.asarray(jac)
    ids = np.flatnonzero((model.jnt_stiffness > 0) &
        np.array([not _is_leaf(model.body(int(b)).name) for b in model.jnt_bodyid]))
    columns = jac[:, model.jnt_dofadr[ids]]
    c = (columns / model.jnt_stiffness[ids]) @ columns.T
    return c, columns, ids


def fit_compliance(old, old_data, new, new_data, sentinels):
    reference, _, _ = compliance(old, old_data, sentinels)
    before, columns, ids = compliance(new, new_data, sentinels)
    scale = 1 / np.sqrt(np.maximum(np.diag(reference), 1e-12))
    weight = scale[:, None] * scale[None, :]
    # One positive factor per complete segment, shared by all three axes.
    bodies = np.unique(new.jnt_bodyid[ids])
    basis = []
    for b in bodies:
        use = new.jnt_bodyid[ids] == b
        block = columns[:, use]
        basis.append((block / new.jnt_stiffness[ids[use]]) @ block.T)
    mat = np.stack([(x*weight).ravel() for x in basis], axis=1)
    target = (reference * weight).ravel()
    result = lsq_linear(mat, target, bounds=(0.05, 20.), tol=1e-10, max_iter=500)
    factors = []
    for b, factor in zip(bodies, result.x):
        use = ids[new.jnt_bodyid[ids] == b]
        new.jnt_stiffness[use] /= factor
        new.dof_damping[new.jnt_dofadr[use]] /= factor
        factors.append(dict(body=new.body(int(b)).name, compliance_factor=float(factor)))
    after, _, _ = compliance(new, new_data, sentinels)
    norm = max(np.linalg.norm(target), 1e-12)
    return dict(method='bounded positive segment spring compliance least squares',
        fitted_before_harvest=True, success=bool(result.success),
        weighted_relative_error_before=float(np.linalg.norm((before-reference)*weight)/norm),
        weighted_relative_error_after=float(np.linalg.norm((after-reference)*weight)/norm),
        factors=factors, sentinels=sentinels, calibration_scope='linear static compliance only; dynamics must be tested')


def prepare(source, output, mode='leaves_fixed'):
    source, output = Path(source).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    old = mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
    initial = dict(np.load(source/'initial_state.npz', allow_pickle=False))
    meta = json.loads((source/'source_export.json').read_text())
    proof = json.loads((source/'inputs.json').read_text())
    for filename, record in proof['input_files'].items():
        if sha256(source/filename) != record['sha256']:
            raise ValueError('Prepared source hash mismatch: '+filename)
    remove = selected_bodies(old, mode)
    tree = ET.parse(source/'input_scene.xml')
    root = tree.getroot()
    root.find('compiler').set('fusestatic', 'false')
    for body in root.iter('body'):
        if body.get('name') in remove:
            for joint in list(body.findall('joint')):
                j = old.joint(joint.get('name')).id
                if abs(float(initial['qpos'][old.jnt_qposadr[j]])) > 1e-12:
                    raise ValueError('Cannot lock a nonzero coordinate without baking transform')
                body.remove(joint)
    tree.write(output/'input_scene.xml', encoding='unicode')
    new = mj.MjModel.from_xml_path(str(output/'input_scene.xml'))
    qmap, vmap = joint_mapping(old, new)
    new.opt.timestep = old.opt.timestep
    new.flg_surfacevel = True
    new.dof_damping[:] = initial['runtime_dof_damping'][vmap]
    new.dof_length[:] = initial['runtime_dof_length'][vmap]
    od, nd = mj.MjData(old), mj.MjData(new)
    od.qpos[:] = initial['qpos']; od.qvel[:] = initial['qvel']
    nd.qpos[:] = initial['qpos'][qmap]; nd.qvel[:] = initial['qvel'][vmap]
    nd.ctrl[:] = initial['ctrl']; nd.eq_active[:] = initial['eq_active']
    mj.mj_forward(old, od); mj.mj_forward(new, nd)
    sentinels = sentinel_bodies(old, meta)
    fit = fit_compliance(old, od, new, nd, sentinels) if mode.startswith('coarse') else None
    if fit:
        for joint in root.iter('joint'):
            if joint.get('name'):
                j = new.joint(joint.get('name')).id
                if new.jnt_stiffness[j] > 0:
                    joint.set('stiffness', str(new.jnt_stiffness[j]))
                    joint.set('damping', str(new.dof_damping[new.jnt_dofadr[j]]))
        tree.write(output/'input_scene.xml', encoding='unicode')
    # Every named body's inertial data and every active collider is retained.
    exact = {}
    for field in ('body_mass', 'body_inertia', 'body_pos', 'body_quat', 'geom_type',
                  'geom_bodyid', 'geom_pos', 'geom_quat', 'geom_size', 'geom_dataid',
                  'geom_contype', 'geom_conaffinity', 'geom_friction', 'geom_margin',
                  'geom_gap', 'geom_solref', 'geom_solimp', 'eq_data', 'eq_type',
                  'actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange', 'mesh_vert', 'mesh_face'):
        a,b = getattr(old,field),getattr(new,field)
        exact[field] = bool(np.array_equal(a,b))
    if not all(exact.values()):
        failed = [k for k,v in exact.items() if not v]
        raise ValueError('Unexpected model field change: '+repr(failed))
    positions = float(np.max(np.abs(nd.xpos-od.xpos)))
    orientations = float(np.max(np.abs(nd.xquat-od.xquat)))
    if positions > 1e-12 or orientations > 1e-12:
        raise ValueError('Initial world transform changed')
    attach = Attachments(new, meta['attachments'])
    mj.mj_forward(new,nd)
    qadr = np.array([new.joint(name).qposadr[0] for name in DOF_NAMES])
    vadr = np.array([new.joint(name).dofadr[0] for name in DOF_NAMES])
    plant = np.array([i for i in range(new.nv) if i not in vadr and
                     new.jnt_type[new.dof_jntid[i]] != mj.mjtJoint.mjJNT_FREE])
    preload = np.zeros(new.nv); preload[plant] = nd.qfrc_bias[plant]
    for fruit,parent in zip(attach.ids, attach.parents):
        jp,jr = np.zeros((3,new.nv)),np.zeros((3,new.nv))
        mj.mj_jac(new,nd,jp,jr,nd.xpos[fruit],int(parent))
        preload[plant] += (jp.T @ (-new.body_mass[fruit]*new.opt.gravity))[plant]
    original_c,_,_=compliance(old,od,sentinels)
    variant_c,_,_=compliance(new,nd,sentinels)
    active=np.flatnonzero((old.geom_contype | old.geom_conaffinity)!=0)
    newly_welded=[]
    for index,a in enumerate(active):
        for b in active[index+1:]:
            ba,bb=int(old.geom_bodyid[a]),int(old.geom_bodyid[b])
            if old.body_weldid[ba]!=old.body_weldid[bb] and new.body_weldid[ba]==new.body_weldid[bb]:
                newly_welded.append([int(a),int(b)])
    variant = dict(mode=mode, source_inputs=str(source), source_mjb_sha256=sha256(source/'input_model.mjb'),
        physical_model_changed=True, exact_trajectory_equivalence_claimed=False,
        removed_joint_bodies=sorted(remove), original_nv=old.nv, variant_nv=new.nv,
        retained_all_body_masses=True, retained_all_collision_geometry=True,
        newly_rigid_internal_geom_pairs=len(newly_welded),
        newly_rigid_internal_geom_pair_ids=newly_welded,
        collision_note='Coarsened segments form rigid groups: internal contacts within a new group are suppressed by the native weld filter; all exterior collision shapes remain.',
        exact_fields=exact, initial_position_max_error_m=positions,
        initial_orientation_max_error=orientations, compliance_fit=fit,
        static_compliance_relative_error=float(np.linalg.norm(variant_c-original_c)/np.linalg.norm(original_c)),
        limitations=['Locked noncolliding leaves retain mass and follow the stem; independent leaf bending is omitted.',
            'Changed coordinate representation requires separate dynamic/harvest qualification.'],
        training_eligible=False, harvest_success=None)
    meta['physics_model_variant']=variant
    mj.mj_saveModel(new,str(output/'input_model.mjb'))
    np.savez_compressed(output/'initial_state.npz',qpos=nd.qpos,qvel=nd.qvel,eq_active=nd.eq_active,ctrl=nd.ctrl,
        robot_qpos_addresses=qadr,robot_dof_addresses=vadr,actuator_ids=initial['actuator_ids'],
        plant_dofs=plant,plant_preload=preload,runtime_dof_damping=new.dof_damping,
        runtime_dof_length=new.dof_length,body_positions=nd.xpos,body_quaternions_wxyz=nd.xquat,
        body_names=initial['body_names'],actuator_kp=initial['actuator_kp'])
    for name in ('commands.npz','sampled_plan.json'):
        shutil.copy2(source/name,output/name)
    (output/'source_export.json').write_text(json.dumps(meta,indent=2)+'\n')
    proof['input_files']={name:dict(path=str(output/name),sha256=sha256(output/name)) for name in
        ('input_scene.xml','input_model.mjb','initial_state.npz','commands.npz','source_export.json')}
    proof['physics_model_variant']=variant
    proof['model_counts']=dict(nbody=new.nbody,nq=new.nq,nv=new.nv,ngeom=new.ngeom,nu=new.nu,neq=new.neq)
    proof['physical_steps']=0
    proof.setdefault('code_sha256', {})[str(Path(__file__).resolve())] = sha256(__file__)
    proof['variant_prepare_seconds']=time.perf_counter()-started
    (output/'inputs.json').write_text(json.dumps(proof,indent=2)+'\n')
    (output/'variant.json').write_text(json.dumps(variant,indent=2)+'\n')
    return variant


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('inputs',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--mode',choices=('leaves_fixed','coarse2','coarse4'),default='leaves_fixed')
    a=p.parse_args();print(json.dumps(prepare(a.inputs,a.output,a.mode),indent=2))
