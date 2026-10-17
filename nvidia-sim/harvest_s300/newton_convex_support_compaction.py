"""Opt-in serialization compaction of the current compiled convex supports.

Every current MuJoCo collision-graph vertex is retained, without decimation,
scaling, outward inflation or changed contact parameters. Original triangle
interiors/BVH and discarded XML visuals are absent only in this new physical
variant; the external full USD display and source assets remain unchanged.
A vertex-set roundtrip bounds convex support error in every direction. This is
not a claim of identical ray meshes, contact manifolds, or GPU trajectories.

EXPERIMENTAL DIAGNOSTIC, NOT QUALIFIED FOR DEFAULT USE: a centered positive
CPU contact control loses a contact after mesh COM recentering, and the full
CAD build does not pass the strict graph-vertex roundtrip certificate. Normal
harvest execution keeps its existing model; this module is not its pipeline.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial import cKDTree

from newton_inputs import sha256
from newton_continuation import VerificationContext
from newton_active_window import _verify_scoped_manifest
from newton_prepared_assets import link_manifest_assets
from newton_fruit_proxy import (collision_hull_vertex_ids, mesh_vertices,
    geom_body_vertices, _physical_invariants, _write_obj)
from newton_native_tool_pieces import _faces

SCHEMA = 's300-compiled-convex-support-compaction-v1'
DEFAULT_COORDINATE_TOLERANCE_M = 1e-6


def _json(path, value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


def _copy_numeric_options(old,new):
    # Existing exporters preserve runtime damping and surface-velocity values
    # which are not all serialized by the original source XML.
    for field in ('dof_damping','dof_length','geom_surfacevel'):
        getattr(new,field)[:] = getattr(old,field)
    changed=np.flatnonzero(old.body_margin!=new.body_margin)
    for body in changed:
        if (old.body(body).name!=new.body(body).name or old.body_geomnum[body]!=0 or new.body_geomnum[body]!=0):
            raise ValueError('Body margin changed on nonempty or mismapped body: '+old.body(body).name)
        new.body_margin[body]=old.body_margin[body]
    new.flg_surfacevel = old.flg_surfacevel
    for name in dir(old.opt):
        if name.startswith('_'):continue
        value=getattr(old.opt,name)
        if isinstance(value,np.ndarray):getattr(new.opt,name)[:] = value
        elif isinstance(value,(int,float,bool,np.integer,np.floating)):setattr(new.opt,name,value)
    return [old.body(int(body)).name for body in changed]


def support_vertex_proof(source_vertices,compiled_vertices,tolerance_m):
    """A bidirectional vertex-set bound also bounds all unit-direction support.

    If P and Q cover one another within epsilon, then their convex hulls have
    Hausdorff distance <=epsilon, hence |h_P(u)-h_Q(u)|<=epsilon for all ||u||=1.
    This certificate is stronger than finite sampled support directions.
    """
    a,b=np.asarray(source_vertices,float),np.asarray(compiled_vertices,float)
    if (a.ndim!=2 or b.ndim!=2 or a.shape[1:]!=(3,) or b.shape[1:]!=(3,) or
            len(a)<4 or len(b)<4 or not np.isfinite(a).all() or not np.isfinite(b).all()):
        raise ValueError('Finite three-dimensional source and compact supports required')
    if not np.isfinite(tolerance_m) or tolerance_m<=0:raise ValueError('Positive finite coordinate tolerance required')
    ab=float(cKDTree(b).query(a)[0].max());ba=float(cKDTree(a).query(b)[0].max());bound=max(ab,ba)
    if bound>tolerance_m:raise ValueError(f'Compiled support coordinate roundtrip failed: {bound} > {tolerance_m}')
    return dict(source_support_vertices=len(a),compact_support_vertices=len(b),
        source_to_compiled_max_distance_m=ab,compiled_to_source_max_distance_m=ba,
        all_direction_support_error_bound_m=bound,declared_coordinate_tolerance_m=tolerance_m,
        proof='Bidirectional finite support-vertex coverage bounds convex-hull Hausdorff distance and every unit-direction support value; no finite-direction sampling surrogate.',passed=True)


def _geom_support(model,geom):
    mesh=int(model.geom_dataid[geom]);ids=collision_hull_vertex_ids(model,mesh)
    return geom_body_vertices(model,geom)[ids]


def prepare(source,output,*,coordinate_tolerance_m=DEFAULT_COORDINATE_TOLERANCE_M):
    """Create a separate signed prepared variant; no stepping or GPU execution."""
    source,output=Path(source).resolve(strict=True),Path(output).resolve()
    if source==output or output.exists():raise FileExistsError('Use a new compact physical output: '+str(output))
    if not np.isfinite(coordinate_tolerance_m) or coordinate_tolerance_m<=0:raise ValueError('Positive finite coordinate tolerance required')
    started=time.perf_counter();context=VerificationContext();context.file_sha256(source/'inputs.json')
    provenance=json.loads((source/'inputs.json').read_text());protected=_verify_scoped_manifest(source,provenance['input_files'],context)
    for name in ('input_scene.xml','input_model.mjb','initial_state.npz','commands.npz','source_export.json'):
        if name not in protected:raise ValueError('Missing signed source input: '+name)
    old=context.model(source/'input_model.mjb',expected_sha256=protected['input_model.mjb'])
    if old.nflex or old.nskin or np.any(old.geom_type==mj.mjtGeom.mjGEOM_SDF):
        raise ValueError('Exact support compaction supports rigid convex meshes only')
    tree=ET.parse(source/'input_scene.xml');root=tree.getroot();compiler=root.find('compiler');asset=root.find('asset');world=root.find('worldbody')
    if (compiler is None or compiler.get('inertiafromgeom')!='false' or compiler.get('fusestatic')!='false'
            or compiler.get('discardvisual')!='true' or asset is None or world is None):
        raise ValueError('Explicit inertials, fusestatic=false and existing discardvisual=true required')
    if list(world.iter('frame')):raise ValueError('Nested XML frame transforms require explicit support')
    active_names={old.geom(i).name for i in range(old.ngeom)}
    if None in active_names or len(active_names)!=old.ngeom:raise ValueError('Exact unique compiled geometry names required')
    removed_visual=[]
    for parent in world.iter():
        for geom in list(parent):
            if geom.tag!='geom' or geom.get('name') in active_names:continue
            if geom.get('contype')!='0' or geom.get('conaffinity')!='0':
                raise ValueError('Noncompiled XML geometry is not explicitly visual-only: '+str(geom.get('name')))
            removed_visual.append(geom.get('name'));parent.remove(geom)
    xml_geoms={g.get('name'):g for g in world.iter('geom')}
    if set(xml_geoms)!=active_names:raise ValueError('Compiled and retained XML geometry identity sets differ')
    mesh_geoms=np.flatnonzero(old.geom_type==mj.mjtGeom.mjGEOM_MESH).tolist()
    used_meshes={int(old.geom_dataid[g]) for g in mesh_geoms}
    if used_meshes!=set(range(old.nmesh)):
        raise ValueError('All source compiled meshes must have direct geom ownership')
    mesh_names={old.mesh(i).name for i in used_meshes};removed_assets=[]
    for elem in list(asset):
        if elem.tag=='mesh' and elem.get('name') not in mesh_names:
            removed_assets.append(elem.get('name'));asset.remove(elem)
    xml_assets={e.get('name'):e for e in asset.findall('mesh')}
    if set(xml_assets)!=mesh_names:raise ValueError('Exact compiled mesh asset identity required')
    output.mkdir(parents=True,exist_ok=False);meshdir=output/'compact_support_meshes';meshdir.mkdir()
    rows=[];source_support={}
    for mesh in range(old.nmesh):
        name=old.mesh(mesh).name;points=mesh_vertices(old,mesh);ids=collision_hull_vertex_ids(old,mesh)
        if (len(ids)<4 or len(set(ids.tolist()))!=len(ids) or np.min(ids)<0 or np.max(ids)>=len(points)):
            raise ValueError('Valid complete current convex graph required: '+name)
        support=points[ids];source_support[mesh]=support
        # Preserve the source graph/exhaustive dispatch class (<10 vertices).
        # Additional points, when needed, are unchanged original non-extreme
        # vertices inside the SAME hull, never synthetic padding or inflation.
        padded_ids=ids
        if len(points)>=10 and len(ids)<10:
            extras=[i for i in range(len(points)) if i not in set(ids.tolist())]
            padded_ids=np.r_[ids,extras[:10-len(ids)]]
        stored=points[padded_ids]
        obj=meshdir/f'mesh_{mesh:05d}.obj';_write_obj(obj,stored,_faces(stored))
        elem=xml_assets[name];elem.attrib.clear();elem.set('name',name);elem.set('file',str(obj));elem.set('scale','1 1 1');elem.set('maxhullvert','-1')
        rows.append(dict(mesh_id=mesh,name=name,source_vertices=len(points),retained_current_graph_vertices=len(ids),
            retained_source_vertex_ids=ids.tolist(),all_source_graph_vertices_retained=True,obj=str(obj),obj_sha256=sha256(obj)))
        rows[-1].update(stored_vertices=len(stored),extra_original_non_extreme_vertex_ids=padded_ids[len(ids):].tolist())
    for geom in mesh_geoms:
        elem=xml_geoms[old.geom(geom).name]
        for key in ('euler','axisangle','xyaxes','zaxis','fromto','size'):elem.attrib.pop(key,None)
        # Vertices are in the SOURCE COMPILED mesh frame. The SOURCE COMPILED
        # body-local geom transform compensates source mesh centering exactly.
        # MuJoCo may choose a new PCA frame, and that second transform is checked
        # in body coordinates below. Never reapply old mesh_pos/mesh_quat here.
        elem.set('pos',' '.join(format(float(x),'.17g') for x in old.geom_pos[geom]))
        elem.set('quat',' '.join(format(float(x),'.17g') for x in old.geom_quat[geom]))
    generated={'input_scene.xml','input_model.mjb','source_export.json','convex_support_compaction_proof.json'}
    # Preserve all source bytes at their original signed source. Only files
    # needed by the new prepared format are linked; original OBJ dependencies
    # are deliberately not rehashed at every later compact-run startup.
    shared={name:e for name,e in provenance['input_files'].items() if name not in generated and Path(name).suffix.lower() not in ('.obj','.stl','.msh')}
    link_manifest_assets(source,output,shared)
    if (source/'sampled_plan.json').is_file() and not (output/'sampled_plan.json').exists():
        (output/'sampled_plan.json').symlink_to((source/'sampled_plan.json').resolve())
    tree.write(output/'input_scene.xml',encoding='unicode');new=mj.MjModel.from_xml_path(str(output/'input_scene.xml'))
    restored_margin_names=_copy_numeric_options(old,new);invariants=_physical_invariants(old,new,set(mesh_geoms))
    if any(old.geom(g).name!=new.geom(g).name for g in range(old.ngeom)):
        raise ValueError('Geometry name/order changed')
    if any(old.mesh(i).name!=new.mesh(i).name for i in range(old.nmesh)):
        raise ValueError('Mesh name/order changed')
    proofs=[]
    for geom in mesh_geoms:
        proofs.append(dict(geom=old.geom(geom).name,mesh=old.mesh(int(old.geom_dataid[geom])).name,
            **support_vertex_proof(_geom_support(old,geom),_geom_support(new,geom),coordinate_tolerance_m)))
    # Identical source support coordinates may have different graph-local IDs
    # and triangulation after compilation. Preserve that distinction explicitly.
    with np.load(source/'initial_state.npz',allow_pickle=False) as z:initial={k:z[k].copy() for k in z.files}
    data=[]
    for model in (old,new):
        d=mj.MjData(model)
        for field in ('qpos','qvel','ctrl','eq_active'):getattr(d,field)[:] = initial[field]
        mj.mj_kinematics(model,d);data.append(d)
    if not np.array_equal(data[0].xpos,data[1].xpos) or not np.array_equal(data[0].xquat,data[1].xquat):
        raise ValueError('Initial body transforms changed')
    meta=json.loads((source/'source_export.json').read_text())
    variant=dict(schema=SCHEMA,mode='current_compiled_convex_support_all_vertices',source_inputs=str(source),
        source_mjb_sha256=protected['input_model.mjb'],physical_model_serialization_changed=True,
        convex_support_shape_changed_beyond_declared_roundoff=False,source_original_assets_preserved=True,
        full_usd_visuals_changed=False,geometry_names_and_count_preserved=True,
        original_ray_triangle_surface_preserved=False,exact_contact_manifold_or_trajectory_equivalence_claimed=False,
        qualified_for_default_use=False,known_centered_positive_contact_blocker=True,
        all_current_graph_vertices_retained=True,no_decimation=True,no_inflation=True,
        coordinate_tolerance_m=coordinate_tolerance_m,
        maximum_body_frame_support_error_bound_m=max((x['all_direction_support_error_bound_m'] for x in proofs),default=0.),
        maximum_mesh_geom_origin_shift_m=float(np.linalg.norm(new.geom_pos[mesh_geoms]-old.geom_pos[mesh_geoms],axis=1).max()) if mesh_geoms else 0.,
        source_counts=dict(nmesh=old.nmesh,nmeshvert=old.nmeshvert,nmeshface=old.nmeshface,nbvh=old.nbvh,nv=old.nv,ngeom=old.ngeom),
        variant_counts=dict(nmesh=new.nmesh,nmeshvert=new.nmeshvert,nmeshface=new.nmeshface,nbvh=new.nbvh,nv=new.nv,ngeom=new.ngeom),
        discarded_xml_visual_geom_names=removed_visual,discarded_xml_unused_mesh_names=removed_assets,
        body_mass_inertia_joint_eq_material_invariants=invariants,initial_body_transforms_bit_exact=True,
        original_compiled_empty_body_margin_cache_preserved=True,
        body_margin_restore_scope='Only identical named bodies with zero source AND compact geometry; nonempty body differences remain strict errors.',
        restored_empty_body_margin_names=restored_margin_names,
        initial_state_and_commands_byte_preserved=True,source_original_prepared_hashes=protected,
        upstream_physics_model_variant=meta.get('physics_model_variant'),
        no_parent_mjb_sha_bypass=True,physical_steps=0,gpu_execution=False,training_eligible=False,harvest_success=None,
        limitations=['Body-frame support is certified for every unit direction by bidirectional current-support vertex coverage.',
            'Recompiled hull graph IDs/order, triangle rays, BVH and contact manifold/tie-breaking may differ; fresh full physics is still required.',
            'Mesh PCA/COM recentering can change internal geom origins while preserving body-frame shape and explicit body inertials. A symmetric fully-contained sphere toy exposes a CPU distance degeneracy; no all-pose solver equivalence is claimed.',
            'All CAD/source OBJ bytes remain in the signed original source. Only current compiled collision support is stored in this opt-in physical model.',
            'The full external USD display is unchanged; this compact collision mesh must not replace rendered CAD/ray sensing.'])
    meta['physics_model_variant']=variant
    for collection in [meta.get('geometry',[]),meta.get('robot',{}).get('geometry',[])]:
        for row in collection:
            if row.get('name') in active_names and old.geom_type[old.geom(row['name']).id]==mj.mjtGeom.mjGEOM_MESH:
                row['compiled_support_compacted']=True
    mj.mj_saveModel(new,str(output/'input_model.mjb'));variant['variant_mjb_sha256']=sha256(output/'input_model.mjb')
    variant['source_mjb_bytes']=(source/'input_model.mjb').stat().st_size;variant['variant_mjb_bytes']=(output/'input_model.mjb').stat().st_size
    _json(output/'source_export.json',meta);_json(output/'convex_support_compaction_proof.json',dict(variant,meshes=rows,geom_support_proofs=proofs))
    _verify_scoped_manifest(source,provenance['input_files'],context)
    shared_hashes=_verify_scoped_manifest(output,shared,context)
    filenames=set(shared)|generated|{str(p.relative_to(output)) for p in meshdir.glob('*.obj')}
    if (output/'sampled_plan.json').exists():filenames.add('sampled_plan.json')
    report=copy.deepcopy(provenance);report['input_files']={n:dict(path=str(output/n),sha256=shared_hashes[n] if n in shared_hashes else sha256(output/n)) for n in sorted(filenames)}
    report['physics_model_variant']=variant;report['model_counts']=dict(nbody=new.nbody,nq=new.nq,nv=new.nv,ngeom=new.ngeom,nu=new.nu,neq=new.neq)
    report['source_prepared_files_sha256']=protected;report['source_prepared_files_unchanged']=True
    report['physical_steps']=0;report['variant_prepare_seconds']=time.perf_counter()-started
    report.setdefault('code_sha256',{})[str(Path(__file__).resolve())]=sha256(__file__)
    report['verification_scope']=context.evidence();_json(output/'inputs.json',report)
    return variant


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('inputs',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--coordinate-tolerance-m',type=float,default=DEFAULT_COORDINATE_TOLERANCE_M)
    a=p.parse_args();r=prepare(a.inputs,a.output,coordinate_tolerance_m=a.coordinate_tolerance_m)
    print(json.dumps({k:r[k] for k in ['source_counts','variant_counts','source_mjb_bytes','variant_mjb_bytes','maximum_body_frame_support_error_bound_m']},indent=2))
if __name__=='__main__':main()
