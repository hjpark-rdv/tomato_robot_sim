"""Native PhysX elastic truss over the user's original plant meshes.

Spring-linked rigid rods approximate bending; this is not a calibrated continuum
plant model. Original fruit masses and break joints are retained. Visual skinning
reads simulated body transforms and never moves a physics body.
"""
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, Vt
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from assets import capsule
from elastic_geometry import (tube_centerline, resample_rod, point_segment_distances,
                              bind_terminal_frame, skin_points, weld_tube_end, vertex_normals)


class ElasticPlant:
    def __init__(self, env, static):
        self.env, self.stage = env, env.stage
        self.root = '/World/envs/env_0/ElasticPlant'
        self.bodies, self.rest, self.paths = [], [], []
        self.skin = []
        self.seams = []
        self.chains = {}
        self.endpoints = {}
        self.joints = []
        self.joint_specs = []
        self.spring_stiffness = {}
        self.rod_masses = []
        self.rod_shapes = []
        self.contact_records = []
        self.scale = env.cfg.elastic_stiffness_scale
        self.cache = UsdGeom.XformCache()
        self.static = static
        root_prim = UsdGeom.Xform.Define(self.stage,self.root).GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(root_prim)
        articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(root_prim)
        articulation_api.CreateEnabledSelfCollisionsAttr(True)
        articulation_api.CreateSolverPositionIterationCountAttr(64)
        articulation_api.CreateSolverVelocityIterationCountAttr(4)
        self.model = dict(model='spring-linked-original-truss-v2',
                          stiffness_scale=self.scale, rod_density_kg_m3=1000.,
                          minimum_rod_mass_kg=.0005,
                          peduncle_stiffness_Nm_rad=25.*self.scale,
                          rachis_stiffness_Nm_rad=2.*self.scale,
                          pedicel_stiffness_Nm_rad=.25*self.scale,
                          calibrated_to_real_plant=False)
        self.armature=1e-5
        self.model.update(joint_armature_kg_m2=self.armature,
                          peduncle_damping_Nms_rad=1.*np.sqrt(self.scale),
                          rachis_damping_Nms_rad=.15*np.sqrt(self.scale),
                          pedicel_damping_Nms_rad=.02*np.sqrt(self.scale),
                          physics_dt_s=env.cfg.sim.dt,
                          rest_shape='gravity-preloaded from authored geometry and original fruit masses')
        # Source meshes have ordered 14-sided rachis/peduncle and 12-sided pedicels.
        peduncle = self._tube('TRUSS_Truss_01_Peduncle', 14, 5, None, 25., 1.)
        # The peduncle grows out of the fixed main stem/stub: these connected
        # surfaces overlap in the original asset, so do not collide at the root.
        root_neighbors = [p.GetPath() for p in static.GetChildren()
                          if p.GetName() == 'STEM_MainStem' or p.GetName().startswith('STEM_Removed_Truss_')]
        UsdPhysics.FilteredPairsAPI.Apply(self.stage.GetPrimAtPath(self.paths[peduncle[0]])).CreateFilteredPairsRel().SetTargets(root_neighbors)
        rachis = self._tube('TRUSS_Rachis', 14, 14, peduncle[-1], 2., .15)
        for index, spec in enumerate(env.fruit_specs):
            name = 'TRUSS_Pedicel_proximal_' + spec['name'][-2:]
            points, _ = self._source_tube(name, 12)
            parent = min(rachis, key=lambda i: np.linalg.norm(self.rest[i][:3] - points[0]))
            chain = self._tube(name, 12, 3, parent, .25, .02)
            connected = [self.paths[i] for i in rachis if abs(i-parent)<=1]
            UsdPhysics.FilteredPairsAPI.Apply(self.stage.GetPrimAtPath(self.paths[chain[0]])).CreateFilteredPairsRel().SetTargets(connected)
            anchor_pose = np.asarray(spec['pose'])
            self._joint(f'anchor_{index:02d}', chain[-1], spec['anchor'], anchor_pose,
                        self.endpoints[name], .25, .02)
            # Only the fruit's own terminal stem capsule is excluded, because the
            # authored distal pedicel and proximal cap meet at the break section.
            targets = [self.paths[chain[-1]]]
            UsdPhysics.FilteredPairsAPI.Apply(self.stage.GetPrimAtPath(spec['path'])).CreateFilteredPairsRel().SetTargets(targets)
        # Terminal cut surfaces must follow the attachment frame, including its
        # rotation. The frame origin in the authored asset is slightly offset
        # from the actual cut surface; the elastic pivot above uses the surface.
        self.visual_rest = np.asarray(self.rest + [s['pose'] for s in env.fruit_specs])
        self.terminal_frames = {'TRUSS_Pedicel_proximal_'+s['name'][-2:]: len(self.paths)+i
                                for i,s in enumerate(env.fruit_specs)}
        self.rest_repairs = {}
        authored_gaps = {}
        for spec in env.fruit_specs:
            name = 'TRUSS_Pedicel_proximal_'+spec['name'][-2:]
            original, _ = self._world_points(static.GetChild(name))
            distal = self.stage.GetPrimAtPath(spec['path']+'/TRUSS_Pedicel_distal_'+spec['name'][-2:])
            distal_points, _ = self._world_points(distal)
            _, first = np.unique(np.round(distal_points,7),axis=0,return_index=True)
            target_ring = distal_points[np.sort(first)[:12]]
            repaired = weld_tube_end(original,target_ring)
            _, first = np.unique(np.round(original,7),axis=0,return_index=True)
            cap = original[np.sort(first)[-12:]]
            authored_gaps[spec['name']] = float(np.linalg.norm(cap[:,None]-target_ring[None],axis=2).min(axis=1).max())
            self.rest_repairs[name] = (original,repaired)
        self.model.update(seam_rest_repair='match proximal end ring to distal ring; taper over six source rings',
                          authored_seam_surface_gap_m=authored_gaps)
        # Transfer every original TRUSS mesh to its corresponding dynamic chain.
        for prim in static.GetChildren():
            name = prim.GetName()
            if not name.startswith('TRUSS_') or not prim.IsA(UsdGeom.Mesh):
                continue
            stem_name = name.removesuffix('_Trichomes')
            chain = self.chains.get(stem_name, rachis)
            self._bind_visual(prim, chain)
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(False)
        for spec in env.fruit_specs:
            name = 'TRUSS_Pedicel_proximal_' + spec['name'][-2:]
            skin = next(s for s in self.skin if s[0].GetPrim().GetName() == name)
            _, unique_indices = np.unique(np.round(skin[1], 7), axis=0, return_index=True)
            cap_ids = np.sort(unique_indices)[-12:]
            distal = self.stage.GetPrimAtPath(spec['path']+'/TRUSS_Pedicel_distal_'+spec['name'][-2:])
            distal_points, _ = self._world_points(distal)
            _, unique_indices = np.unique(np.round(distal_points,7), axis=0, return_index=True)
            distal_cap_ids = np.sort(unique_indices)[:12]
            self.seams.append((spec, skin, cap_ids, distal, distal_cap_ids))
        self.model.update(dynamic_rods=len(self.bodies), elastic_joints=len(self.joints),
                          original_visual_meshes=len(self.skin),
                          collision_model='source-radius capsules; trichomes visual only',
                          fruit_plant_contact=True)
        cfg = ArticulationCfg(prim_path=self.root,spawn=None,
                init_state=ArticulationCfg.InitialStateCfg(pos=tuple(self.rest[0][:3]),joint_pos={'.*':0.}),
                actuators={'springs':ImplicitActuatorCfg(joint_names_expr=['.*'],stiffness=None,damping=None,armature=self.armature)})
        self.articulation = Articulation(cfg)
        env.scene.articulations['elastic_plant'] = self.articulation
        self.body_ids = None
        print('[ELASTIC MODEL]', self.model, flush=True)

    def _world_points(self, prim):
        # Gf matrices use row vectors; vectorize instead of one Python call/vertex.
        points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        matrix = np.asarray(self.cache.GetLocalToWorldTransform(prim))
        return points @ matrix[:3, :3] + matrix[3, :3], matrix

    def _source_tube(self, name, sides):
        prim = self.static.GetChild(name)
        if not prim:
            raise ValueError('Missing original tube ' + name)
        vertices, _ = self._world_points(prim)
        return tube_centerline(vertices, sides)

    def _tube(self, name, sides, segments, parent, stiffness, damping):
        points, radii = self._source_tube(name, sides)
        nodes, radii = resample_rod(points, radii, segments)
        chain = []
        for j, (a, b) in enumerate(zip(nodes[:-1], nodes[1:])):
            radius = float(max(radii[j:j+2]))
            center = (a+b)/2
            path = self.root + f'/{name}_{j:02d}'
            x = UsdGeom.Xform.Define(self.stage, path)
            x.AddTranslateOp().Set(Gf.Vec3d(*center))
            x.AddOrientOp().Set(Gf.Quatf(1.))
            UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
            mass = max(.0005, 1000.*np.pi*radius**2*np.linalg.norm(b-a))
            UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(float(mass))
            phys = PhysxSchema.PhysxRigidBodyAPI.Apply(x.GetPrim())
            phys.CreateSolverPositionIterationCountAttr(64)
            phys.CreateSolverVelocityIterationCountAttr(16)
            phys.CreateEnableCCDAttr(True)
            phys.CreateSleepThresholdAttr(0.)
            PhysxSchema.PhysxContactReportAPI.Apply(x.GetPrim()).CreateThresholdAttr(0.)
            shape = capsule(self.stage, path+'/StemCollider', a-center, b-center, radius)
            shape.MakeInvisible()
            pose = np.r_[center, 1., 0., 0., 0.]
            self._joint(f'{name}_{j:02d}', parent, path, pose, a, stiffness, damping)
            index = len(self.bodies)
            self.bodies.append(path)
            self.rod_masses.append(mass)
            self.rod_shapes.append((a-center,b-center,radius))
            self.paths.append(path)
            self.rest.append(pose)
            chain.append(index)
            parent = index
        self.chains[name] = chain
        self.endpoints[name] = nodes[-1]
        return chain

    def _joint(self, name, parent, child_path, child_pose, point, stiffness, damping):
        joint_type = UsdPhysics.FixedJoint if parent is None else UsdPhysics.Joint
        joint = joint_type.Define(self.stage, self.root+'/Joints/'+name)
        joint.CreateBody1Rel().SetTargets([child_path])
        for slot, pose in [(0, np.r_[0.,0.,0.,1.,0.,0.,0.] if parent is None else self.rest[parent]),
                           (1, child_pose)]:
            r = Rotation.from_quat(pose[[4,5,6,3]])
            local = r.inv().apply(point - pose[:3])
            q = r.inv().as_quat()
            getattr(joint, f'CreateLocalPos{slot}Attr')().Set(Gf.Vec3f(*local))
            getattr(joint, f'CreateLocalRot{slot}Attr')().Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3])))
        if parent is not None:
            joint.CreateBody0Rel().SetTargets([self.paths[parent]])
        joint.CreateExcludeFromArticulationAttr(False)
        joint.CreateCollisionEnabledAttr(False)
        if parent is None:
            self.joints.append(str(joint.GetPath()))
            self.joint_specs.append((parent, child_path, np.array(joint.GetLocalPos0Attr().Get()),
                                     np.array(joint.GetLocalPos1Attr().Get())))
            return
        for axis in ('transX','transY','transZ'):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            limit.CreateLowAttr(1.)
            limit.CreateHighAttr(-1.)  # low > high locks translation
        for axis in ('rotX','rotY','rotZ'):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            limit.CreateLowAttr(-45.)
            limit.CreateHighAttr(45.)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
            drive.CreateTypeAttr('force')
            # USD angular drives use per-degree units; PhysX uses per-radian.
            drive.CreateStiffnessAttr(float(stiffness*self.scale*np.pi/180))
            drive.CreateDampingAttr(float(damping*np.sqrt(self.scale)*np.pi/180))
            drive.CreateTargetPositionAttr(0.)
            drive.CreateTargetVelocityAttr(0.)
            drive.CreateMaxForceAttr(100.)
        self.joints.append(str(joint.GetPath()))
        self.joint_specs.append((parent, child_path, np.array(joint.GetLocalPos0Attr().Get()),
                                 np.array(joint.GetLocalPos1Attr().Get())))
        self.spring_stiffness[name] = stiffness*self.scale

    def _bind_visual(self, prim, chain):
        points, matrix = self._world_points(prim)
        stem_name = prim.GetName().removesuffix('_Trichomes')
        repaired = stem_name in self.rest_repairs
        if repaired:
            original, corrected = self.rest_repairs[stem_name]
            if prim.GetName() == stem_name:
                points = corrected
            else:
                # Keep the source hairs attached to the locally repaired skin.
                nearest = cKDTree(original).query(points)[1]
                points = points + (corrected-original)[nearest]
        centers = np.array([self.rest[i][:3] for i in chain])
        distances = np.linalg.norm(points[:, None] - centers, axis=2)
        local_ids = np.argsort(distances, axis=1)[:, :2]
        ids = np.asarray(chain)[local_ids]
        weights = 1./np.maximum(np.take_along_axis(distances, local_ids, axis=1), .0002)**2
        weights /= weights.sum(axis=1, keepdims=True)
        if stem_name in self.terminal_frames:
            cap_ids = ()
            if prim.GetName() == stem_name:
                # Include every split-normal duplicate in the final 12-sided
                # ring, not just the unique representatives used by diagnostics.
                _, first, inverse_ids = np.unique(np.round(points,7), axis=0,
                                                  return_index=True, return_inverse=True)
                final_ring = np.argsort(first)[-12:]
                cap_ids = np.flatnonzero(np.isin(inverse_ids,final_ring))
            ids, weights = bind_terminal_frame(points, ids, weights, centers[-1],
                self.endpoints[stem_name], self.terminal_frames[stem_name], cap_ids)
        mesh=UsdGeom.Mesh(prim)
        normals=np.asarray(mesh.GetNormalsAttr().Get(),dtype=float)
        if mesh.GetNormalsInterpolation() != 'vertex' or normals.shape != points.shape:
            raise ValueError('Original truss must retain per-vertex normals: '+str(prim.GetPath()))
        if repaired:
            inverse = np.linalg.inv(matrix)
            local = points @ inverse[:3,:3]+inverse[3,:3]
            normals = vertex_normals(local,mesh.GetFaceVertexCountsAttr().Get(),
                                     mesh.GetFaceVertexIndicesAttr().Get())
        world_normals=normals @ np.linalg.inv(matrix[:3,:3]).T
        self.skin.append((mesh, points, np.linalg.inv(matrix), ids, weights, world_normals, matrix[:3,:3].T))

    def reset(self):
        q = torch.zeros_like(self.articulation.data.joint_pos)
        self.articulation.write_joint_state_to_sim(q,q)
        if self.body_ids is None:
            self.body_ids = [self.articulation.body_names.index(p.rsplit('/',1)[-1]) for p in self.paths]
            self.anchor_ids = {s['anchor']:self.articulation.body_names.index(s['anchor'].rsplit('/',1)[-1])
                               for s in self.env.fruit_specs}
            print('[ELASTIC DRIVES]',self.articulation.joint_names[:6],
                  'stiffness',self.articulation.data.joint_stiffness[0,:9].tolist(),
                  'limits',self.articulation.data.joint_effort_limits[0,:9].tolist(),flush=True)
            self.preload = self._gravity_preload()
        self.articulation.set_joint_position_target(self.preload)

    def _gravity_preload(self):
        """Infer stress-free angles so the supplied loaded pose balances gravity.

        This is a static preload, not a feedback force that cancels plant motion.
        It remains fixed during contact, loading, release and joint breakage.
        """
        children = {p:[] for p in self.paths}
        mass_points = {p:[(m,pose[:3])] for p,m,pose in zip(self.paths,self.rod_masses,self.rest)}
        for spec in self.env.fruit_specs:
            pose = np.asarray(spec['pose'])
            center = pose[:3]+Rotation.from_quat(pose[[4,5,6,3]]).apply(spec['center'])
            mass = UsdPhysics.MassAPI(self.stage.GetPrimAtPath(spec['path'])).GetMassAttr().Get()
            mass_points[spec['anchor']] = [(.0005,pose[:3]),(float(mass),center)]
            children[spec['anchor']] = []
        for parent,child,_,_ in self.joint_specs:
            if parent is not None:
                children[self.paths[parent]].append(child)
        def subtree(path):
            return mass_points[path] + [pair for child in children[path] for pair in subtree(child)]
        targets = {}
        for path,(parent,child,p0,_) in zip(self.joints,self.joint_specs):
            if parent is None:
                continue
            point = self.rest[parent][:3]+p0
            torque = sum((np.cross(center-point,[0.,0.,-9.81*mass]) for mass,center in subtree(child)),start=np.zeros(3))
            name = path.rsplit('/',1)[-1]
            targets[name] = -torque/self.spring_stiffness[name]
        q = torch.zeros_like(self.articulation.data.joint_pos)
        for i,name in enumerate(self.articulation.joint_names):
            stem,axis = name.rsplit(':',1)
            q[0,i] = float(targets[stem][int(axis)])
        self.model['maximum_preload_angle_deg'] = float(torch.rad2deg(q.abs()).max())
        return q

    def poses(self):
        return self.articulation.data.body_state_w[0,self.body_ids,:7].cpu().numpy()

    def _visual_transforms(self):
        ids = self.body_ids + [self.anchor_ids[s['anchor']] for s in self.env.fruit_specs]
        poses = self.articulation.data.body_state_w[0,ids,:7].cpu().numpy()
        current = Rotation.from_quat(poses[:,[4,5,6,3]]).as_matrix()
        original = Rotation.from_quat(self.visual_rest[:,[4,5,6,3]]).as_matrix()
        return poses, current @ original.transpose(0,2,1)

    def sync_visuals(self):
        poses, rotations = self._visual_transforms()
        for mesh, points, inverse, ids, weights, normals, normal_inverse in self.skin:
            world = skin_points(points,self.visual_rest[:,:3],rotations,poses[:,:3],ids,weights)
            local = world @ inverse[:3,:3] + inverse[3,:3]
            mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(local.astype(np.float32)))
            rotated=np.einsum('nkij,nj->nki',rotations[ids],normals)
            local_normals=(rotated*weights[:,:,None]).sum(axis=1) @ normal_inverse
            local_normals/=np.maximum(np.linalg.norm(local_normals,axis=1,keepdims=True),1e-12)
            mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(local_normals.astype(np.float32)))

    def visual_attachment_metrics(self):
        """Compare the actual skinned cut surface with the fruit-side rigid frame.

        Checking only the anchor origins cannot detect a visible skinning tear.
        A broken native joint is intentionally allowed to separate afterwards.
        """
        poses, rotations = self._visual_transforms()
        errors = []
        for (spec, skin, cap_ids, _, _), fruit in zip(self.seams, self.env.fruits):
            _, points, _, ids, weights, _, _ = skin
            points, ids, weights = points[cap_ids], ids[cap_ids], weights[cap_ids]
            proximal = skin_points(points,self.visual_rest[:,:3],rotations,poses[:,:3],ids,weights)
            original = np.asarray(spec['pose'])
            fruit_pose = fruit.data.root_state_w[0,:7].cpu().numpy()
            local = Rotation.from_quat(original[[4,5,6,3]]).inv().apply(points-original[:3])
            distal = Rotation.from_quat(fruit_pose[[4,5,6,3]]).apply(local)+fruit_pose[:3]
            errors.append(float(np.linalg.norm(proximal-distal,axis=1).max()))
        return dict(max_visual_attachment_error_m=max(errors),
                    target_visual_attachment_error_m=errors[self.env.target_index])

    def rendered_attachment_metrics(self):
        """Inspect both actual USD mesh boundaries after renderer synchronization."""
        cache = UsdGeom.XformCache()
        gaps, surface_gaps, pose_errors = [], [], []
        for (spec, skin, cap_ids, distal, distal_cap_ids), fruit in zip(self.seams,self.env.fruits):
            boundaries = []
            for prim, ids in [(skin[0].GetPrim(), cap_ids),(distal, distal_cap_ids)]:
                p = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),dtype=float)[ids]
                m = np.asarray(cache.GetLocalToWorldTransform(prim))
                boundaries.append(p @ m[:3,:3]+m[3,:3])
            gaps.append(float(np.linalg.norm(boundaries[0].mean(0)-boundaries[1].mean(0))))
            distances = np.linalg.norm(boundaries[0][:,None]-boundaries[1][None],axis=2)
            surface_gaps.append(float(max(distances.min(axis=0).max(),distances.min(axis=1).max())))
            usd_pose = np.array(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(spec['path'])).ExtractTranslation())
            native_pose = fruit.data.root_pos_w[0].cpu().numpy()
            pose_errors.append(float(np.linalg.norm(usd_pose-native_pose)))
            if spec is self.env.target_spec and pose_errors[-1]>.00005:
                ops=UsdGeom.Xformable(self.stage.GetPrimAtPath(spec['path'])).GetOrderedXformOps()
                print('[ELASTIC DISPLAY MISMATCH]',spec['name'],'USD',usd_pose.tolist(),
                      'PhysX',native_pose.tolist(),'ops',[(op.GetName(),str(op.Get())) for op in ops],flush=True)
        return dict(max_rendered_seam_center_gap_m=max(gaps),
                    target_rendered_seam_center_gap_m=gaps[self.env.target_index],
                    max_rendered_seam_surface_gap_m=max(surface_gaps),
                    target_rendered_seam_surface_gap_m=surface_gaps[self.env.target_index],
                    max_usd_fruit_pose_error_m=max(pose_errors))

    def metrics(self):
        poses = self.poses()
        body_poses = {p:q for p,q in zip(self.paths,poses)}
        body_poses.update({p:self.articulation.data.body_state_w[0,i,:7].cpu().numpy() for p,i in self.anchor_ids.items()})
        gaps=[]
        for parent,child,p0,p1 in self.joint_specs:
            a=np.r_[0.,0.,0.,1.,0.,0.,0.] if parent is None else poses[parent]
            b=body_poses[child]
            wa=a[:3]+Rotation.from_quat(a[[4,5,6,3]]).apply(p0)
            wb=b[:3]+Rotation.from_quat(b[[4,5,6,3]]).apply(p1)
            gaps.append(np.linalg.norm(wa-wb))
        ids = self.chains['TRUSS_Rachis']
        rotations = Rotation.from_quat(poses[ids][:,[4,5,6,3]])
        a = rotations.apply(np.array([self.rod_shapes[i][0] for i in ids]))+poses[ids,:3]
        b = rotations.apply(np.array([self.rod_shapes[i][1] for i in ids]))+poses[ids,:3]
        radii = np.array([self.rod_shapes[i][2] for i in ids])
        center = self.env._target_geometry()[0][0].cpu().numpy()
        gap = (point_segment_distances(center[None],a,b)[0]-radii-self.env.target_spec['radius']).min()
        attachment_gaps = [np.linalg.norm(body_poses[s['anchor']][:3]-f.data.root_pos_w[0].cpu().numpy())
                           for s,f in zip(self.env.fruit_specs,self.env.fruits)]
        return dict(max_rod_displacement_m=float(np.linalg.norm(poses[:,:3]-np.asarray(self.rest)[:,:3],axis=1).max()),
                    max_rod_rotation_deg=float(np.rad2deg(Rotation.from_quat(poses[:,[4,5,6,3]]).magnitude()).max()),
                    max_joint_gap_m=float(max(gaps)),worst_joint=self.joints[int(np.argmax(gaps))],
                    target_fruit_rachis_gap_m=float(gap),max_attachment_gap_m=float(max(attachment_gaps)),
                    **self.visual_attachment_metrics())
