"""Opt-in enclosing analytic fruit colliders; never substitutes a harvest result.

Fit a positive-definite affine unit-ball map to every compiled source hull
vertex, then outwardly rescale to certify enclosure. This changes geometry and
contact manifolds; mass, inertia, other colliders and existing control stay fixed.
"""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from newton_fruit_proxy import (collision_hull_vertex_ids, geom_body_vertices,
                                mesh_vertices, sphere_directions)
from newton_inputs import sha256


def _json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def enclosing_ellipsoid(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 4 or not np.isfinite(points).all():
        raise ValueError('Finite nondegenerate 3D source hull required')
    origin = points.mean(axis=0)
    radius = float(np.linalg.norm(points-origin, axis=1).max())
    if radius <= 0 or np.linalg.matrix_rank(points-origin) != 3:
        raise ValueError('Finite nondegenerate 3D source hull required')
    p = (points-origin)/radius
    values, vectors = np.linalg.eigh(p.T@p/len(p))
    if min(values) <= 0:
        raise ValueError('Positive covariance required')
    a = (vectors/np.sqrt(values))@vectors.T
    a /= float(np.linalg.norm(p@a, axis=1).max())*1.001
    def unpack(x):
        return np.array([[x[0],x[3],x[4]],[x[3],x[1],x[5]],[x[4],x[5],x[2]]]), x[6:]
    def objective(x):
        matrix,_=unpack(x)
        sign, determinant=np.linalg.slogdet(matrix)
        return -determinant if sign > 0 else 1e9
    def gradient(x):
        matrix,_=unpack(x);inv=np.linalg.inv(matrix)
        return np.r_[-np.diag(inv),-2*inv[0,1],-2*inv[0,2],-2*inv[1,2],np.zeros(3)]
    def constraints(x):
        matrix, offset=unpack(x);q=p@matrix+offset
        return 1.-np.sum(q*q,axis=1)
    def jacobian(x):
        matrix,offset=unpack(x);q=p@matrix+offset
        return -2*np.column_stack((q[:,0]*p[:,0],q[:,1]*p[:,1],q[:,2]*p[:,2],
            q[:,0]*p[:,1]+q[:,1]*p[:,0],q[:,0]*p[:,2]+q[:,2]*p[:,0],
            q[:,1]*p[:,2]+q[:,2]*p[:,1],q))
    start=np.r_[np.diag(a),a[0,1],a[0,2],a[1,2],np.zeros(3)]
    result=minimize(objective,start,jac=gradient,method='SLSQP',
        constraints=[dict(type='ineq',fun=constraints,jac=jacobian),
                     dict(type='ineq',fun=lambda x:np.linalg.eigvalsh(unpack(x)[0])-1e-6)],
        options=dict(maxiter=120,ftol=1e-10))
    a, offset=unpack(result.x)
    values,rotation=np.linalg.eigh(a)
    if min(values)<=0 or not np.isfinite(result.x).all():
        raise ValueError('Ellipsoid fit did not retain a finite positive-definite map')
    if np.linalg.det(rotation)<0:rotation[:,0]*=-1
    center=origin-radius*np.linalg.solve(a,offset)
    axes=radius/values
    maximum=float(np.linalg.norm(((points-center)@rotation)/axes,axis=1).max())
    axes *= max(1.,maximum)*(1+1e-6)
    level=float(np.max(np.sum((((points-center)@rotation)/axes)**2,axis=1)))
    if level>=1.:raise ValueError('Source enclosure verification failed')
    directions=sphere_directions(4096)
    gaps=[]
    for block in np.array_split(directions,16):
        ellipsoid_support=block@center+np.linalg.norm((block@rotation)*axes,axis=1)
        gaps.extend((ellipsoid_support-np.max(points@block.T,axis=0)).tolist())
    return center,axes,rotation,dict(method='SPD affine ellipsoid log-volume fit; SLSQP',
        optimizer_success=bool(result.success),optimizer_message=str(result.message),
        optimizer_iterations=int(result.nit),source_hull_vertices=len(points),
        center_m=center.tolist(),semiaxes_m=axes.tolist(),rotation_columns=rotation.tolist(),
        max_source_quadratic_level=level,source_enclosed=True,
        sampled_support_directions=4096,sampled_outward_support_gap_max_m=float(max(gaps)),
        sampled_outward_support_gap_mean_m=float(np.mean(gaps)),
        discrepancy_scope='4096 direction support gaps are diagnostic samples, not a certified global Hausdorff upper bound.',
        enclosure_proof='Ellipsoid is convex; every original convex-hull vertex has quadratic level below1, so the entire original convex hull is enclosed.')


def protected_fields(old,new,changed):
    checks={}
    def same(name,a,b):
        a,b=np.asarray(a),np.asarray(b);checks[name]=bool(a.shape==b.shape and np.array_equal(a,b))
    for name in ('nbody','njnt','nq','nv','nu','neq','nexclude','ngeom','npair','nsite','ntendon'):
        same(name,getattr(old,name),getattr(new,name))
    for name in dir(old):
        if name.startswith(('body_','jnt_','dof_','actuator_','eq_','site_','tendon_')) or name in ('qpos0','qpos_spring','exclude_signature'):
            if name in ('body_bvhadr','body_bvhnum'):continue
            value=getattr(old,name)
            if isinstance(value,np.ndarray):same(name,value,getattr(new,name))
    unaffected=np.array([i for i in range(old.ngeom) if i not in changed])
    intentional={'geom_type','geom_dataid','geom_pos','geom_quat','geom_size','geom_rbound','geom_aabb','geom_sameframe'}
    for name in dir(old):
        if name.startswith('geom_') and isinstance(getattr(old,name),np.ndarray):
            if name=='geom_dataid':continue
            ids=unaffected if name in intentional else slice(None)
            same(name,getattr(old,name)[ids],getattr(new,name)[ids])
    # MuJoCo drops unused fruit mesh assets and renumbers the remaining dataid
    # addresses. Compare every nonfruit asset by source name and content.
    removed={int(old.geom_dataid[g]) for g in changed}
    for mesh in range(old.nmesh):
        if mesh in removed:continue
        other=new.mesh(old.mesh(mesh).name).id
        for field in ('mesh_pos','mesh_quat','mesh_scale','mesh_extrema'):
            same(f'mesh:{old.mesh(mesh).name}.{field}',getattr(old,field)[mesh],getattr(new,field)[other])
        same(f'mesh:{old.mesh(mesh).name}.vertices',mesh_vertices(old,mesh),mesh_vertices(new,other))
        for field,adr,count in (('mesh_face','mesh_faceadr','mesh_facenum'),
            ('mesh_normal','mesh_normaladr','mesh_normalnum'),('mesh_facenormal','mesh_faceadr','mesh_facenum')):
            a,n=int(getattr(old,adr)[mesh]),int(getattr(old,count)[mesh]);b,m=int(getattr(new,adr)[other]),int(getattr(new,count)[other])
            same(f'mesh:{old.mesh(mesh).name}.{field}',getattr(old,field)[a:a+n],getattr(new,field)[b:b+m])
    for geom in unaffected:
        if old.geom_type[geom]==mj.mjtGeom.mjGEOM_MESH:
            same(f'geom:{old.geom(geom).name}.mesh_name',old.mesh(int(old.geom_dataid[geom])).name,new.mesh(int(new.geom_dataid[geom])).name)
        else:same(f'geom:{old.geom(geom).name}.dataid',old.geom_dataid[geom],new.geom_dataid[geom])
    for name in dir(old.opt):
        if name.startswith('_'):continue
        value=getattr(old.opt,name)
        if isinstance(value,(np.ndarray,int,float,bool,np.integer,np.floating)):same('option.'+name,value,getattr(new.opt,name))
    failed=[name for name,passed in checks.items() if not passed]
    if failed:raise ValueError('Protected fields changed: '+repr(failed))
    return dict(passed=True,checks=checks,intentional_fruit_fields=sorted(intentional))


def prepare(source,output):
    source,output=Path(source).resolve(strict=True),Path(output).resolve()
    if source==output or output.exists():raise FileExistsError('Use a new output folder')
    started=time.perf_counter();provenance=json.loads((source/'inputs.json').read_text())
    for name,evidence in provenance['input_files'].items():
        if sha256(source/name)!=evidence['sha256']:raise ValueError('Prepared source SHA mismatch: '+name)
    old=mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
    initial=dict(np.load(source/'initial_state.npz',allow_pickle=False))
    metadata=json.loads((source/'source_export.json').read_text());targets=set(metadata['fruits'])
    tree=ET.parse(source/'input_scene.xml');root=tree.getroot()
    if root.find('compiler').get('inertiafromgeom')!='false':raise ValueError('Explicit inertials required')
    elements={e.get('name'):e for e in root.iter('geom') if e.get('name')}
    selected=[g for g in range(old.ngeom) if old.body(int(old.geom_bodyid[g])).name in targets
              and (old.geom_contype[g]|old.geom_conaffinity[g])!=0]
    if {old.body(int(old.geom_bodyid[g])).name for g in selected}!=targets:raise ValueError('Missing fruit collider')
    if any(old.geom_type[g]!=mj.mjtGeom.mjGEOM_MESH for g in selected):raise ValueError('Source fruit mesh required')
    rows=[];source_points={}
    output.mkdir(parents=True,exist_ok=False)
    for geom in selected:
        points=geom_body_vertices(old,geom);source_points[geom]=points
        mesh=int(old.geom_dataid[geom]);ids=collision_hull_vertex_ids(old,mesh)
        center,axes,rotation,row=enclosing_ellipsoid(points[ids])
        element=elements[old.geom(geom).name];element.set('type','ellipsoid');element.attrib.pop('mesh',None)
        for attribute in ('fromto','euler','axisangle','xyaxes','zaxis'):element.attrib.pop(attribute,None)
        element.set('pos',' '.join(format(float(x),'.17g') for x in center))
        quaternion=Rotation.from_matrix(rotation).as_quat()[[3,0,1,2]]
        element.set('quat',' '.join(format(float(x),'.17g') for x in quaternion))
        element.set('size',' '.join(format(float(x),'.17g') for x in axes))
        rows.append(dict(geom=old.geom(geom).name,geom_id=geom,body=old.body(int(old.geom_bodyid[geom])).name,
                         original_mesh_vertices=len(points),**row))
    tree.write(output/'input_scene.xml',encoding='unicode')
    new=mj.MjModel.from_xml_path(str(output/'input_scene.xml'))
    new.dof_damping[:]=old.dof_damping;new.dof_length[:]=old.dof_length
    new.geom_surfacevel[:]=old.geom_surfacevel;new.flg_surfacevel=old.flg_surfacevel
    for name in dir(old.opt):
        if name.startswith('_'):continue
        value=getattr(old.opt,name)
        if isinstance(value,np.ndarray):getattr(new.opt,name)[:]=value
        elif isinstance(value,(int,float,bool,np.integer,np.floating)):setattr(new.opt,name,value)
    invariants=protected_fields(old,new,set(selected))
    for row in rows:
        geom=new.geom(row['geom']).id;matrix=np.zeros(9);mj.mju_quat2Mat(matrix,new.geom_quat[geom])
        level=float(np.max(np.sum((((source_points[geom]-new.geom_pos[geom])@matrix.reshape(3,3))/new.geom_size[geom])**2,axis=1)))
        if level>=1.:raise ValueError('Compiled ellipsoid does not enclose original mesh')
        row['compiled_max_source_quadratic_level']=level
    data=mj.MjData(new)
    for name in ('qpos','qvel','eq_active','ctrl'):getattr(data,name)[:]=initial[name]
    mj.mj_forward(new,data)
    reference=mj.MjData(old)
    for name in ('qpos','qvel','eq_active','ctrl'):getattr(reference,name)[:]=initial[name]
    mj.mj_forward(old,reference)
    position_error=float(np.max(np.abs(data.xpos-reference.xpos)))
    rotation_error=float(np.max(np.abs(data.xquat-reference.xquat)))
    if position_error>1e-12 or rotation_error>1e-12:raise ValueError('Initial body transform changed')
    # Every source fruit is attached at this canonical start. Therefore no
    # released-fruit contact exception applies; audit every active contact.
    if len(data.eq_active) and not np.all(data.eq_active):raise ValueError('Initial diagnostic requires all fruits attached')
    contacts=[dict(geom_ids=[int(c.geom1),int(c.geom2)],geoms=[new.geom(c.geom1).name,new.geom(c.geom2).name],
                   distance_m=float(c.dist),active_constraint=int(c.efc_address)>=0) for c in data.contact]
    penetration=max([max(0.,-c['distance_m']) for c in contacts if c['active_constraint']]+[0.])
    variant=dict(mode='enclosing_analytic_fruit_ellipsoid',source_inputs=str(source),
        source_mjb_sha256=sha256(source/'input_model.mjb'),
        upstream_physics_model_variant=metadata.get('physics_model_variant'),
        physical_model_changed=True,collision_approximation_changed=True,exact_trajectory_equivalence_claimed=False,
        preserves_original_convex_exterior=True,retained_all_body_masses=True,retained_all_nonfruit_collision_geometry=True,
        original_nv=old.nv,variant_nv=new.nv,fruits=rows,invariant_proof=invariants,
        original_mesh_assets=old.nmesh,variant_mesh_assets=new.nmesh,
        unused_source_fruit_mesh_asset_note='Compiler prunes unused fruit physics mesh assets; original source visual assets remain unchanged.',
        initial_body_position_max_error_m=position_error,initial_body_rotation_max_error=rotation_error,
        initial_contacts=contacts,initial_active_penetration_m=penetration,
        initial_active_1mm_passed=penetration<=.001,
        visual_geometry='Source Isaac visual farm and fruit meshes unchanged; only physics fruit collider types differ.',
        contact_manifold_note='MW supports ellipsoid analytic support mapping through generic convex CCD; smooth contacts may have fewer manifold points than mesh contacts.',
        training_eligible=False,harvest_success=None)
    mj.mj_saveModel(new,str(output/'input_model.mjb'))
    for name in ('initial_state.npz','commands.npz','sampled_plan.json'):shutil.copy2(source/name,output/name)
    metadata['physics_model_variant']=variant;_json(output/'source_export.json',metadata)
    _json(output/'analytic_fruit_proof.json',variant)
    proof=copy.deepcopy(provenance)
    proof['input_files']={name:dict(path=str(output/name),sha256=sha256(output/name)) for name in
        ('input_scene.xml','input_model.mjb','initial_state.npz','commands.npz','source_export.json','analytic_fruit_proof.json')}
    proof['physics_model_variant']=variant;proof['physical_steps']=0
    proof['variant_prepare_seconds']=time.perf_counter()-started
    proof.setdefault('code_sha256',{})[str(Path(__file__).resolve())]=sha256(__file__)
    _json(output/'inputs.json',proof)
    return variant


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('inputs',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();result=prepare(args.inputs,args.output)
    print(json.dumps(dict(output=str(args.output),initial_active_penetration_m=result['initial_active_penetration_m'],
                         initial_active_1mm_passed=result['initial_active_1mm_passed'],
                         sampled_max_outward_support_gap_m=max(x['sampled_outward_support_gap_max_m'] for x in result['fruits'])),indent=2))
