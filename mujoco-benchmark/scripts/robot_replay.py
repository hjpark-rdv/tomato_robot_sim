"""Native articulation replay for isolating robot cost; no IK or planner."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from suite import RING

class RobotReplay:
    def __init__(self,stage,reference,asset,trace_file,control_dt,visible):
        self.stage=stage;self.ref=reference;self.tool_path=reference['tool_path'];self.root=self.tool_path.split('/link6/')[0]
        rows=json.loads(Path(trace_file).read_text());self.ts=np.arange(len(rows))*control_dt
        self.commands=np.asarray([r['command'] for r in rows]);self.initial=np.asarray(rows[0]['joints'])
        self.names=reference['robot_fk']['joint_names'];self.visible=visible
        robot=UsdGeom.Xform.Define(stage,self.root);robot.GetPrim().GetReferences().AddReference(str(Path(asset).resolve()))
        UsdGeom.Xformable(robot).MakeMatrixXform().Set(Gf.Matrix4d(np.asarray(reference['robot_fk']['world']).T.tolist()))
        roots=[]
        for prim in Usd.PrimRange(robot.GetPrim()):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):roots.append(str(prim.GetPath()))
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
        if len(roots)!=1:raise RuntimeError(f'Expected one robot articulation, found {roots}')
        self.art_path=roots[0]
        node=stage.GetPrimAtPath(self.tool_path)
        while node and not node.HasAPI(UsdPhysics.RigidBodyAPI):node=node.GetParent()
        if not node:raise RuntimeError('No native rigid body owns the hook')
        self.tool_body_name=node.GetName();cache=UsdGeom.XformCache()
        self.local=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(self.tool_path))*cache.GetLocalToWorldTransform(node).GetInverse()).T
        UsdGeom.Imageable(stage.GetPrimAtPath(self.tool_path)).MakeInvisible()
        if not visible:UsdGeom.Imageable(robot.GetPrim()).MakeInvisible()
    def bind(self,view):
        self.art=view.create_articulation_view(self.art_path)
        if self.art.count!=1:raise RuntimeError('Robot native articulation unavailable')
        meta=self.art.get_metatype(0);self.order=[self.names.index(n) for n in meta.dof_names];self.tool_index=list(meta.link_names).index(self.tool_body_name)
        self.idx=np.array([0],np.int32);self.root_pose=self.art.get_root_transforms().copy()
        lift=np.array([n=='farmily_lift_height_joint' for n in meta.dof_names])[None]
        self.art.set_dof_stiffnesses(np.where(lift,50000.,800.).astype(np.float32),self.idx)
        self.art.set_dof_dampings(np.where(lift,2500.,40.).astype(np.float32),self.idx)
        self.art.set_dof_max_forces(np.where(lift,10000.,80.).astype(np.float32),self.idx)
        limits=self.art.get_dof_max_velocities().copy();limits[~lift]=1.;self.art.set_dof_max_velocities(limits,self.idx)
    def reset(self):
        q=self.initial[self.order][None].astype(np.float32)
        self.art.set_root_transforms(self.root_pose,self.idx);self.art.set_root_velocities(np.zeros((1,6),np.float32),self.idx)
        self.art.set_dof_positions(q,self.idx);self.art.set_dof_velocities(np.zeros_like(q),self.idx)
        self.art.set_dof_position_targets(q,self.idx);self.art.set_dof_velocity_targets(np.zeros_like(q),self.idx)
    def command(self,t):
        q=np.array([np.interp(t,self.ts,self.commands[:,i]) for i in self.order],np.float32)[None]
        self.art.set_dof_position_targets(q,self.idx)
    def get_transforms(self):
        p=self.art.get_link_transforms()[0,self.tool_index];r=R.from_quat(p[3:]);matrix=r.as_matrix()@self.local[:3,:3]
        return np.array([[*(p[:3]+r.apply(self.local[:3,3])),*R.from_matrix(matrix).as_quat()]],np.float32)
    def joint_positions(self):
        q=self.art.get_dof_positions()[0];out=np.empty_like(q);out[self.order]=q;return out
