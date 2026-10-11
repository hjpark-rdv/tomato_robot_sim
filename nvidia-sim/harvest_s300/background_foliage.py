"""Opt-in passive neighboring leaves; source stems remain static.

Uses the existing compound-leaf identity and rigid/D6 spring authoring helpers.
Only a predeclared exact leaf scope is converted. Native cooking and startup
must be tested separately; this is an uncalibrated dynamics/proxy revision.
"""
import argparse
import copy
import json
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from compliant_foliage import canonical_leaf_group, world_vertices
from motion_assets import joint, rigid, sha, physx
from motion_stage_metrics import copy_stage_metrics, require_motion_units


ROOT = '/World/BackgroundCompliantFoliage'


def declared_bodies(stage, manifest):
    """Exact separate standalone inventory; never alias robot/active plant."""
    rows=manifest.get('background_compliant_foliage',{}).get('bodies',[])
    paths={}
    for row in rows:
        path=row.get('path');jid=row.get('joint')
        if (not isinstance(path,str) or not path.startswith(ROOT+'/Plant_') or
                not isinstance(jid,str) or not jid.startswith(ROOT+'/Plant_') or path in paths):
            raise ValueError('Exact unique standalone background leaf identity required')
        p=stage.GetPrimAtPath(path);j=UsdPhysics.Joint(stage.GetPrimAtPath(jid))
        if (not p or not p.HasAPI(UsdPhysics.RigidBodyAPI) or not j or
                list(j.GetBody0Rel().GetTargets()) or
                [str(x) for x in j.GetBody1Rel().GetTargets()]!=[path] or
                j.GetExcludeFromArticulationAttr().Get() is not True):
            raise ValueError('Declared passive leaf is not an exact standalone world-connected rigid body')
        paths[path]=row
    return paths


def collider_paths(stage):
    """Include physical colliders inside source instance proxies."""
    return {str(p.GetPath()) for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
            if p.HasAPI(UsdPhysics.CollisionAPI) and
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False}


def _zero_signs(prim):
    attr=prim.GetAttribute('xformOp:transform')
    if not attr or attr.Get() is None:return 0
    matrix=np.array(attr.Get(),float,copy=True)
    count=int(np.count_nonzero((matrix==0.)&np.signbit(matrix)))
    if count:
        matrix[matrix==0.]=0.;attr.Clear();attr.Set(Gf.Matrix4d(matrix.tolist()))
    return count


def _scope_groups(stage, scope):
    rows=scope.get('critical_neighbor_leaf_groups')
    if not isinstance(rows,list) or not rows:raise ValueError('Nonempty exact neighboring leaf scope required')
    selected=[];identities=set()
    for row in rows:
        stem,group=row.get('stem'),row.get('group')
        if (not isinstance(stem,str) or not stem.startswith('/World/Greenhouse/Plants/Plant_') or
                not stem.endswith('/Stem') or canonical_leaf_group(group or '')!=group):
            raise ValueError('Exact original background stem/leaf identity required')
        if (stem,group) in identities:raise ValueError('Duplicate background leaf scope')
        identities.add((stem,group));p=stage.GetPrimAtPath(stem)
        if not p or not p.IsActive():raise ValueError('Original background stem is absent')
        if any(x.HasAPI(UsdPhysics.RigidBodyAPI) for x in Usd.PrimRange(p,Usd.TraverseInstanceProxies())):
            raise ValueError('Background-only conversion cannot replace an existing active rigid stem')
        meshes=[str(x.GetPath()) for x in Usd.PrimRange(p,Usd.TraverseInstanceProxies())
                if x.IsA(UsdGeom.Mesh) and canonical_leaf_group(x.GetCustomDataByKey('sourceName') or '')==group]
        if not meshes:raise ValueError('Exact source compound leaf not found: '+stem+'/'+group)
        expected=row.get('exact_intersected_meshes',[])
        if not expected or not set(expected)<=set(meshes):raise ValueError('Scope/source intersection mesh identity mismatch')
        if any(path not in collider_paths(stage) for path in expected):raise ValueError('Expected source leaf collider is absent')
        selected.append(dict(stem=stem,group=group,meshes=meshes))
    return selected


def apply_scope(stage, scope, stiffness=.005, damping=.0008):
    """Preserve exact rendered triangles and explicit filters; add only leaves.

    Dynamic source triangle meshes require convex decomposition. Raw source
    vertices/triangles remain unchanged and the proxy revision is recorded.
    There is no collider removal, broad filtering, stem elasticity or preload.
    """
    if not all(np.isfinite(x) and x>0 for x in (stiffness,damping)):
        raise ValueError('Positive finite existing leaf spring values required')
    selected=_scope_groups(stage,scope);before=collider_paths(stage)
    relationships=[(str(p.GetPath()),list(p.GetRelationship('physics:filteredPairs').GetTargets()))
                   for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies())
                   if p.GetRelationship('physics:filteredPairs')]
    # Selected source stems are instances. Editing only those two stems makes
    # their exact composed meshes available for immutable reparenting.
    uninstanced=[]
    for stem in sorted({row['stem'] for row in selected}):
        prim=stage.GetPrimAtPath(stem)
        if prim.IsInstance():prim.SetInstanceable(False);uninstanced.append(stem)
    if collider_paths(stage)!=before:raise ValueError('Source uninstancing changed physical collider coverage')
    flat=stage.Flatten(addSourceFileComment=False);mapping={};rows=[];max_error=0.;zero_sign_count=0
    for item in selected:
        stem,group,meshes=item['stem'],item['group'],item['meshes']
        plant=Sdf.Path(stem).GetParentPath().name
        body=ROOT+'/'+plant+'/Leaf_'+group
        if stage.GetPrimAtPath(body):raise ValueError('Background leaf already converted: '+body)
        source_prim=stage.GetPrimAtPath(stem)
        main=[str(p.GetPath()) for p in Usd.PrimRange(source_prim)
              if p.IsA(UsdGeom.Mesh) and p.GetCustomDataByKey('sourceName')=='MainStem']
        if not main:raise ValueError('Exact source MainStem mesh required for leaf attachment')
        stem_points=np.concatenate([world_vertices(stage,path) for path in main])
        leaf_points=np.concatenate([world_vertices(stage,path) for path in meshes])
        distance,nearest=cKDTree(stem_points).query(leaf_points)
        leaf_index=int(distance.argmin());pivot=stem_points[int(nearest[leaf_index])]
        # Same explicit mass/inertia approximation as the existing converter.
        center=(leaf_points.max(0)+leaf_points.min(0))/2
        size=np.maximum(np.ptp(leaf_points,axis=0),.002)
        mass=max(.0002,.002*(max(size)/.2)**2)
        inertia=mass/12*(np.sum(size**2)-size**2)
        pose=np.eye(4);pose[:3,3]=center
        prim=rigid(stage,body,pose,mass,inertia)
        physx(prim,'PhysxRigidBodyAPI',**{
            'physxRigidBody:solverPositionIterationCount':(Sdf.ValueTypeNames.Int,64),
            'physxRigidBody:solverVelocityIterationCount':(Sdf.ValueTypeNames.Int,4)})
        jid=ROOT+'/'+plant+'/Joints/Leaf_'+group
        j=joint(stage,jid,None,body,np.eye(4),pose,pivot,k=stiffness,d=damping)
        j.CreateExcludeFromArticulationAttr(True)
        for axis in ('rotX','rotY','rotZ'):
            limit=UsdPhysics.LimitAPI(j.GetPrim(),axis);limit.CreateLowAttr(-85.);limit.CreateHighAttr(85.)
        copied=[];source_names=[];proxy_changes=[]
        for old in meshes:
            old_prim=stage.GetPrimAtPath(old);source_names.append(old_prim.GetCustomDataByKey('sourceName'))
            old_world=world_vertices(stage,old)
            old_tf=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(old_prim)).T
            # The same Mesh remains visible and owns its dynamic collision.
            # This name makes the existing cooked-leaf reader recognize it.
            new=body+'/Appendage_'+Sdf.Path(old).name
            if not Sdf.CopySpec(flat,old,stage.GetRootLayer(),new):raise ValueError('Source mesh copy failed')
            new_prim=stage.GetPrimAtPath(new)
            UsdGeom.Xformable(new_prim).MakeMatrixXform().Set(Gf.Matrix4d((np.linalg.inv(pose)@old_tf).T.tolist()))
            zero_sign_count+=_zero_signs(new_prim)
            if old in before:
                old_approximation=old_prim.GetAttribute('physics:approximation').Get()
                UsdPhysics.MeshCollisionAPI.Apply(new_prim).CreateApproximationAttr('convexDecomposition')
                proxy_changes.append(dict(source=old,current=new,old=old_approximation,new='convexDecomposition',
                    raw_vertices_and_triangles_unchanged=True,native_cooking_verified=False))
            stage.OverridePrim(old).SetActive(False);mapping[old]=new;copied.append(new)
            max_error=max(max_error,float(np.max(np.abs(world_vertices(stage,new)-old_world))))
        zero_sign_count+=_zero_signs(prim)
        rows.append(dict(name=plant+'_Leaf_'+group,path=body,joint=jid,meshes=copied,
            source_stem=stem,source_group=group,source_names=sorted(set(source_names)),
            source_meshes=meshes,source_main_stem_meshes=main,
            rest_pose_world=np.r_[center,[0.,0.,0.,1.]].tolist(),pivot=pivot.tolist(),
            pivot_method='Closest exact authored MainStem surface vertex to exact leaf surface vertices',
            source_leaf_to_stem_vertex_gap_m=float(distance[leaf_index]),mass=mass,inertia=inertia.tolist(),
            stiffness=stiffness,damping=damping,angular_limits_deg=[-85.,85.],
            world_anchored=True,main_articulation_member=False,collision_proxy_revision=proxy_changes))
    for old,targets in relationships:
        prim=stage.GetPrimAtPath(mapping.get(old,old))
        remapped=[mapping.get(str(target),str(target)) for target in targets]
        if prim and prim.IsActive() and (old in mapping or any(str(x) in mapping for x in targets)):
            if prim.IsInstanceProxy():raise ValueError('A referenced source filter requires additional source instancing treatment')
            prim.CreateRelationship('physics:filteredPairs').SetTargets(remapped)
    if collider_paths(stage)!={mapping.get(path,path) for path in before}:
        raise ValueError('Scoped background conversion changed collider coverage')
    # No new filter targets are introduced. Static/static implicit exclusions
    # become dynamic/static contacts; this is a deliberate revision, not an
    # invented permission to ignore other crops, trunk, robot or structures.
    after_rels={str(p.GetPath()):list(p.GetRelationship('physics:filteredPairs').GetTargets())
                for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies())
                if p.GetRelationship('physics:filteredPairs')}
    for old,targets in relationships:
        if after_rels.get(mapping.get(old,old))!=[Sdf.Path(mapping.get(str(t),str(t))) for t in targets]:
            raise ValueError('Source filtered-pair graph changed beyond exact leaf path remap')
    if max_error>1e-9:raise ValueError('Background leaf authored world geometry changed')
    return dict(schema='s300-background-compliant-foliage-v1',bodies=copy.deepcopy(rows),leaves=rows,
        path_mapping=mapping,selected_leaf_count=len(rows),new_rigid_body_count=len(rows),
        world_connected_d6_joint_count=len(rows),main_articulation_added_dofs=0,
        added_mass_kg=sum(row['mass'] for row in rows),existing_body_masses_changed=False,
        authored_world_geometry_max_error_m=max_error,collider_coverage_preserved=True,
        source_filter_graph_preserved_by_exact_path_remap=True,additional_filter_targets=[],
        uninstanced_source_stems=uninstanced,numeric_zero_sign_canonicalization_count=zero_sign_count,
        passive_leaves_gravity_preload=False,numerical_armature='Not applied to standalone D6 joints; explicit existing leaf mass/inertia used',
        source_scope=copy.deepcopy(scope),native_validation='pending',calibrated=False,
        dynamics_revision=f'{len(rows)} scoped static leaf groups become standalone passive spring bodies; neighboring stems/trusses remain static',
        collision_proxy_revision='Exact rendered Mesh points/triangles retained; static triangles use convexDecomposition when made dynamic',
        source_implicit_static_static_contacts='Now dynamic/static contacts; no new broad exclusion/filter is added',
        training_eligible=False,harvest_success=None)


def revise(assembly,output,scope_path):
    from compact_motion_assets import verify_compact_dependencies,property_inventory,compare_geometry
    assembly,output,scope_path=map(lambda p:Path(p).resolve(),(assembly,output,scope_path))
    manifest=json.loads((assembly/'motion_manifest.json').read_text());source=assembly/'scene.usda'
    if sha(source)!=manifest['scene_sha256']:raise ValueError('Source scene SHA mismatch')
    verify_compact_dependencies(assembly,manifest)
    if manifest.get('background_compliant_foliage'):raise ValueError('Use explicit accumulation support before adding a second background leaf scope')
    scope=json.loads(scope_path.read_text());stage=Usd.Stage.Open(Usd.Stage.Open(str(source)).Flatten())
    require_motion_units(stage);output.mkdir(parents=True,exist_ok=False)
    report=apply_scope(stage,scope);report.update(source_assembly=str(assembly),source_scene_sha256=sha(source),
        source_manifest_sha256=sha(assembly/'motion_manifest.json'),scope_path=str(scope_path),scope_sha256=sha(scope_path))
    before=property_inventory(stage);stage.Flatten(addSourceFileComment=False).Export(str(output/'composed.usdc'))
    wrapper=Usd.Stage.CreateNew(str(output/'scene.usda'));wrapper.GetRootLayer().subLayerPaths=['composed.usdc']
    copy_stage_metrics(stage,wrapper);wrapper.SetDefaultPrim(wrapper.GetPrimAtPath('/World'));wrapper.GetRootLayer().Save()
    if property_inventory(wrapper)!=before:raise ValueError('Background revision USDC changed effective authored properties')
    geometry=compare_geometry(stage,wrapper)
    (output/'background_geometry_materialization.json').write_text(json.dumps(geometry,indent=2)+'\n')
    manifest['background_compliant_foliage']=report
    manifest['compact_assets']=dict(schema='s300-lossless-materialized-usdc-v1',source_assembly=str(assembly),
        source_scene_sha256=sha(source),source_manifest_sha256=sha(assembly/'motion_manifest.json'),
        materialized_dependency=dict(relative_path='composed.usdc',sha256=sha(output/'composed.usdc'),
            format='usdc',materialized=True,external_usd_sublayers=[]),geometry_unchanged=True,
        native_validation='pending',note='Lossless relative to explicit standalone passive-foliage derived stage')
    if sha(source)!=manifest['scene_sha256']:raise ValueError('Source asset changed during background revision')
    manifest['scene_sha256']=sha(output/'scene.usda');manifest.update(training_eligible=False,harvest_success=None)
    (output/'motion_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    shutil.copy2(assembly/'skin.npz',output/'skin.npz')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('assembly',type=Path)
    p.add_argument('output',type=Path);p.add_argument('scope',type=Path);a=p.parse_args()
    print(json.dumps(revise(a.assembly,a.output,a.scope),indent=2))


if __name__=='__main__':main()
