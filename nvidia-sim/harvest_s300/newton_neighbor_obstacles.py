"""Prepare an opt-in fixed-neighbor comparison for the existing Newton runner.

Uses the previously converted USD capsules/convex colliders, not a new plant
model or execution loop. Source assets and default collision policy stay intact.
Requires USD Python in addition to the existing Newton preparation environment.
"""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from newton_inputs import sha256
from newton_fruit_proxy import _write_obj


def relocated(value, old_root, new_root):
    if isinstance(value, str) and value.startswith(str(old_root) + '/'):
        candidate = new_root / value[len(str(old_root)) + 1:]
        if candidate.exists():
            return str(candidate)
    if isinstance(value, dict):
        return {k: relocated(v, old_root, new_root) for k, v in value.items()}
    if isinstance(value, list):
        return [relocated(v, old_root, new_root) for v in value]
    return value


def selected_colliders(stage, root):
    prim = stage.GetPrimAtPath(root)
    if not prim or not prim.IsActive():
        raise ValueError('Missing active neighbor root: ' + root)
    result = []
    for child in Usd.PrimRange(prim):
        collider = UsdPhysics.CollisionAPI(child)
        if not collider or collider.GetCollisionEnabledAttr().Get() is False:
            continue
        # The established foliage policy has already authored collision=false.
        if child.IsA(UsdGeom.Mesh):
            if UsdPhysics.MeshCollisionAPI(child).GetApproximationAttr().Get() != 'convexHull':
                raise ValueError('Cannot treat a triangle obstacle as convex: '+str(child.GetPath()))
        elif not child.IsA(UsdGeom.Capsule):
            raise ValueError('Unsupported source collider: '+str(child.GetPath()))
        result.append(child)
    if not result:
        raise ValueError('Neighbor has no enabled colliders: '+root)
    return result


def add_colliders(stage, roots, xml, output):
    if UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdGeom.GetStageUpAxis(stage) != 'Z':
        raise ValueError('Expected established metre/Z-up source')
    cache = UsdGeom.XformCache()
    inventory = []
    assets, world = xml.find('asset'), xml.find('worldbody')
    folder = output/'neighbor_meshes'; folder.mkdir()
    for root in roots:
        for prim in selected_colliders(stage, root):
            name = 'neighbor_obstacle_'+str(len(inventory)).zfill(4)
            transform = np.asarray(cache.GetLocalToWorldTransform(prim)).T
            if not np.isfinite(transform).all():
                raise ValueError('Nonfinite source transform')
            attrs = dict(name=name, contype='8', conaffinity='7', density='0', group='3')
            if prim.IsA(UsdGeom.Capsule):
                cap = UsdGeom.Capsule(prim)
                scale = np.linalg.norm(transform[:3,:3], axis=0)
                if not np.allclose(transform[:3,:3].T@transform[:3,:3], np.eye(3)*scale[0]**2,
                                   atol=1e-10, rtol=1e-10):
                    raise ValueError('Nonuniform/sheared capsule: '+str(prim.GetPath()))
                axis = np.eye(3)['XYZ'.index(cap.GetAxisAttr().Get())]
                local = np.array([-axis,axis])*float(cap.GetHeightAttr().Get())/2
                ends = local@transform[:3,:3].T + transform[:3,3]
                attrs.update(type='capsule', fromto=' '.join(map(str,ends.ravel())),
                             size=str(float(cap.GetRadiusAttr().Get())*scale[0]))
                approximation = 'source_capsule_world_transform'
            else:
                mesh = UsdGeom.Mesh(prim)
                if not np.all(np.asarray(mesh.GetFaceVertexCountsAttr().Get()) == 3):
                    raise ValueError('Expected source triangle mesh')
                vertices = np.asarray(mesh.GetPointsAttr().Get(),float)@transform[:3,:3].T+transform[:3,3]
                faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
                if (mesh.GetOrientationAttr().Get() == 'leftHanded') != (np.linalg.det(transform[:3,:3]) < 0):
                    faces = faces[:,::-1]
                file = folder/(name+'.obj'); _write_obj(file,vertices,faces)
                ET.SubElement(assets,'mesh',name=name,file=str(file))
                attrs.update(type='mesh',mesh=name)
                approximation = 'source_convexHull_vertices; native_PhysX_hull_cooking_not_reproduced'
            ET.SubElement(world,'geom',**attrs)
            inventory.append(dict(name=name,source_path=str(prim.GetPath()),root=root,
                source_type=prim.GetTypeName(),approximation=approximation,transform=transform.tolist()))
    return inventory


def verify_preserved(old, new):
    """Check original dynamics and geoms by identity, excluding packed layout."""
    checks = {}
    for prefix, count, obj, skip in [
        ('body_',old.nbody,mj.mjtObj.mjOBJ_BODY,{'body_geomadr','body_geomnum','body_bvhadr','body_bvhnum'}),
        ('jnt_',old.njnt,mj.mjtObj.mjOBJ_JOINT,set()),
        ('dof_',old.nv,None,set()),
        ('eq_',old.neq,mj.mjtObj.mjOBJ_EQUALITY,set()),
        ('actuator_',old.nu,mj.mjtObj.mjOBJ_ACTUATOR,set()),
        ('geom_',old.ngeom,mj.mjtObj.mjOBJ_GEOM,{'geom_dataid'})]:
        ids = np.arange(count) if obj is None else np.array([
            mj.mj_name2id(new,obj,mj.mj_id2name(old,obj,i)) if mj.mj_id2name(old,obj,i) else i
            for i in range(count)],dtype=np.int64)
        if np.any(ids<0): raise ValueError('Missing original '+prefix)
        for key in dir(old):
            a = getattr(old,key)
            if key.startswith(prefix) and key not in skip and isinstance(a,np.ndarray) and a.shape[0]==count:
                checks[key]=bool(np.array_equal(a,getattr(new,key)[ids]))
    for key in ('qpos0','qpos_spring'):
        checks[key]=bool(np.array_equal(getattr(old,key),getattr(new,key)))
    for key in dir(old.opt):
        if key.startswith('_'): continue
        a,b=getattr(old.opt,key),getattr(new.opt,key)
        if isinstance(a,(np.ndarray,int,float,bool)):
            checks['opt.'+key]=bool(np.array_equal(a,b))
    failed=[k for k,v in checks.items() if not v]
    if failed:raise ValueError('Original physical fields changed: '+repr(failed))
    return dict(passed=True,checks=checks)


def prepare(source, output, scene, roots, old_root, new_root):
    source=Path(source).resolve(strict=True);output=Path(output).resolve()
    if output.exists():raise FileExistsError(output)
    provenance=json.loads((source/'inputs.json').read_text())
    for name,evidence in provenance['input_files'].items():
        if evidence.get('sha256') and sha256(source/name)!=evidence['sha256']:
            raise ValueError('Source hash mismatch: '+name)
    output.mkdir(parents=True)
    for name in ('initial_state.npz','commands.npz','sampled_plan.json'):
        shutil.copy2(source/name,output/name)
    # Preserve required certificates referenced by the prepared-input manifest.
    replaced={'input_scene.xml','input_model.mjb','source_export.json'}
    for name in provenance['input_files']:
        if name not in replaced and not (output/name).exists():
            (output/name).parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source/name,output/name)
    metadata=relocated(json.loads((source/'source_export.json').read_text()),old_root,new_root)
    (output/'source_export.json').write_text(json.dumps(metadata,indent=2)+'\n')
    xml=ET.parse(source/'input_scene.xml')
    for node in xml.iter():
        for key,value in list(node.attrib.items()):
            node.set(key,relocated(value,old_root,new_root))
    old=mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
    inventory=[]
    if roots:
        stage=Usd.Stage.Open(str(scene))
        if not stage:raise ValueError('Source USD did not open')
        inventory=add_colliders(stage,roots,xml.getroot(),output)
    xml.write(output/'input_scene.xml',encoding='unicode')
    if roots:
        new=mj.MjModel.from_xml_path(str(output/'input_scene.xml'))
        # The signed MJB includes the existing attachment runtime damping and
        # prepared dt which the import-description XML does not serialize.
        new.dof_length[:]=old.dof_length
        new.dof_damping[:]=old.dof_damping
        new.opt.timestep=old.opt.timestep
        proof=verify_preserved(old,new)
        mj.mj_saveModel(new,str(output/'input_model.mjb'))
    else:
        (output/'input_model.mjb').symlink_to(source/'input_model.mjb')
        new=old;proof=dict(passed=True,original_model_bytes_unchanged=True)
    report=dict(schema='s300-fixed-neighbor-comparison-v1',source=str(source),source_mjb_sha256=sha256(source/'input_model.mjb'),
        source_usd=str(scene),source_usd_sha256=sha256(scene),neighbor_roots=roots,
        inventory=inventory,added_colliders=len(inventory),original_ngeom=old.ngeom,ngeom=new.ngeom,
        nv=new.nv,original_physical_fields=proof,training_eligible=False,harvest_success=None,
        scope='Existing converted neighbors held fixed; not all historical background triangle meshes restored',
        defaults_changed=False,initial_and_commands_unchanged=True)
    metadata['neighbor_obstacles']=dict(source_usd=str(scene),roots=roots,geoms=inventory)
    (output/'source_export.json').write_text(json.dumps(metadata,indent=2)+'\n')
    provenance=copy.deepcopy(provenance)
    for name in provenance['input_files']:
        if (output/name).is_file():
            provenance['input_files'][name].update(sha256=sha256(output/name),path=str(output/name))
    # The authoritative MJB and its XML reconstruction must both be auditable.
    # Include newly generated mesh dependencies, not only the source file list.
    for file in sorted((output/'neighbor_meshes').glob('*.obj')):
        name=str(file.relative_to(output))
        provenance['input_files'][name]=dict(sha256=sha256(file),path=str(file))
    provenance.update(neighbor_obstacle_comparison=report,model_counts=dict(provenance['model_counts'],ngeom=new.ngeom))
    (output/'inputs.json').write_text(json.dumps(provenance,indent=2)+'\n')
    (output/'neighbor_obstacles.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--roots',nargs='*',default=[])
    p.add_argument('--old-root',type=Path,default=Path('/root/docker_share/mujoko_debugging_data'))
    p.add_argument('--new-root',type=Path,default=Path('/root/docker_share/mujoko_debugging_data_bak'))
    a=p.parse_args();r=prepare(a.source,a.output,a.scene,a.roots,a.old_root,a.new_root)
    print(json.dumps({k:r[k] for k in ('added_colliders','nv','ngeom','scope')},indent=2))
