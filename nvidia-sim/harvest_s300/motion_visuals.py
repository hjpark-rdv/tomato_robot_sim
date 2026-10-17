"""Display-only rod skin and crank linkage, driven by measured physics state.

``prepare_display_meshes`` must run before physics initialization.  Removing
unused collider schemas from explicitly disabled display meshes prevents point
updates from entering PhysX's mesh-change/cooking path.  It never removes an
enabled collider or changes a body, joint, material, or collision-filter value.
"""
import json
import time
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
from scipy.spatial.transform import Rotation as R
from motion_assets import REPO  # installs the existing rl helper directory
from motion_kinematics import transmission_transforms
from motion_pose_display import exact_changed_pose_rows


def prepare_display_meshes(stage):
    """Strip dormant collision APIs only from this adapter's visual meshes.

    The USD asset on disk is not saved or changed.  The caller records the
    returned paths and applies this before creating native physics views, for
    headless and GUI alike.  Repeated calls are harmless.  Disabled colliders
    elsewhere, including any background collision proxy, remain untouched.
    """
    selected=[]
    for root in ('/World/ElasticTarget', '/World/ElasticNeighbors', '/World/ActuatedTool/fixed/Transmission'):
        prim=stage.GetPrimAtPath(root)
        if not prim:
            continue
        for child in Usd.PrimRange(prim):
            path=str(child.GetPath())
            display_only=(path.startswith('/World/ElasticTarget/Skin/') or
                          (path.startswith('/World/ElasticNeighbors/') and '/Skin/' in path) or
                          '/Visual_' in path or
                          path.startswith('/World/ActuatedTool/fixed/Transmission/'))
            if not display_only or not child.IsA(UsdGeom.Mesh):
                continue
            collision=UsdPhysics.CollisionAPI(child)
            if not collision or collision.GetCollisionEnabledAttr().Get() is not False:
                continue
            if child.HasAPI(UsdPhysics.RigidBodyAPI) or child.HasAPI(UsdPhysics.MassAPI):
                raise ValueError('Display mesh unexpectedly owns a physical body: '+path)
            schemas=[x for x in child.GetAppliedSchemas()
                     if x in ('PhysicsCollisionAPI','PhysicsMeshCollisionAPI',
                              'PhysxCollisionAPI','PhysxMeshCollisionAPI',
                              'PhysxConvexDecompositionCollisionAPI') or
                     x.startswith('PhysxCookedDataAPI:')]
            selected.append((child,path,schemas))
    # Collect all USD queries first; coalesce edits into one change notice.
    with Usd.EditContext(stage,stage.GetSessionLayer()):
        with Sdf.ChangeBlock():
            for prim,_,schemas in selected:
                for schema in schemas:
                    prim.RemoveAppliedSchema(schema)
    return dict(mesh_count=len(selected),paths=[path for _,path,_ in selected],
                removed_schemas={path:schemas for _,path,schemas in selected},
                policy='explicit adapter display meshes with collisionEnabled=false only',
                persistent_asset_modified=False)


class DeformationVisuals:
    def __init__(self,stage,assembly,body_paths):
        self.stage=stage
        self.pose_count=len(body_paths)
        index={p:i for i,p in enumerate(body_paths)}
        meta=json.loads((assembly/'motion_manifest.json').read_text())
        self.bound=[]
        self.dependencies=[]
        sources=[(meta['plant'], 'skin.npz')]+[(row['plant'],row['skin_file'])
                    for row in meta.get('elastic_neighbors',[])]
        for plant,skin_file in sources:
            with np.load(assembly/skin_file) as skins:
                for i,path in enumerate(plant['skin_paths']):
                    mesh=UsdGeom.Mesh(stage.GetPrimAtPath(path))
                    ids=skins[f'{i}_ids']
                    offsets=skins[f'{i}_vertices'][:,None]-skins[f'{i}_rest'][ids]
                    body_ids=np.array([index[x] for x in skins[f'{i}_body_paths']])
                    self.bound.append((mesh.GetPointsAttr(),mesh.GetExtentAttr(),offsets,
                        ids,skins[f'{i}_weights'],body_ids))
                    self.dependencies.append(np.unique(body_ids[ids]))
        with Usd.EditContext(stage,stage.GetSessionLayer()):
            self.transmission={name:UsdGeom.Xformable(stage.GetPrimAtPath(
                '/World/ActuatedTool/fixed/Transmission/'+name)).MakeMatrixXform()
                for name in ('crank','linkL','linkR')}
        self.last_gap=None
        self._last_poses=None

    def invalidate(self):
        """Invalidate visual caches only; no physics state is touched."""
        self._last_poses=None
        self.last_gap=None

    def update(self,poses,gap):
        started=time.perf_counter()
        poses=np.asarray(poses)
        if poses.shape!=(self.pose_count,7) or not np.isfinite(poses).all() or not np.isfinite(gap):
            raise ValueError('Finite measured poses and gripper gap required')
        changed=exact_changed_pose_rows(poses,self._last_poses)
        # Rod skinning depends only on these measured bones. The jaw gap drives
        # the separate transmission below, never a rod vertex or its extent.
        dirty=[bool(np.any(changed[ids])) for ids in self.dependencies]
        rotations=R.from_quat(poses[:,3:7]).as_matrix() if any(dirty) else None
        writes=[];vertices=0
        skipped_vertices=0
        for selected,(points,extent,offsets,ids,weights,body_ids) in zip(dirty,self.bound):
            if not selected:
                skipped_vertices+=len(offsets)
                continue
            local_rotations=rotations[body_ids]
            transformed=np.einsum('nkij,nkj->nki',local_rotations[ids],offsets)
            vnow=((transformed+poses[body_ids,:3][ids])*weights[:,:,None]).sum(axis=1)
            vnow=vnow.astype(np.float32)
            writes.append((points,Vt.Vec3fArray.FromNumpy(vnow),extent,
                           Vt.Vec3fArray.FromNumpy(np.array([vnow.min(0),vnow.max(0)],np.float32))))
            vertices+=len(vnow)
        transforms=[]
        if self.last_gap is None or abs(gap-self.last_gap)>1e-9:
            transforms=[(self.transmission[name],Gf.Matrix4d(tf.T.tolist()))
                        for name,tf in transmission_transforms(gap).items()]
        calculated=time.perf_counter()
        # One notice per display frame instead of one notice per changed mesh.
        # No USD reads occur inside ChangeBlock, only writes to cached attributes.
        with Usd.EditContext(self.stage,self.stage.GetSessionLayer()):
            with Sdf.ChangeBlock():
                for points,value,extent,bounds in writes:
                    points.Set(value)
                    extent.Set(bounds)
                for op,tf in transforms:
                    op.Set(tf)
        self.last_gap=gap
        self._last_poses=poses.copy()
        finished=time.perf_counter()
        return dict(calculation_s=calculated-started,usd_write_s=finished-calculated,
                    meshes_updated=len(writes),vertices_updated=vertices,
                    meshes_skipped=len(self.bound)-len(writes),vertices_skipped=skipped_vertices,
                    transmission_updates=len(transforms))
