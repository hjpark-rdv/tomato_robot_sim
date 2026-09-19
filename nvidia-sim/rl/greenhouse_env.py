"""Harvest in the supplied greenhouse and complete, referenced tomato cluster."""
import numpy as np
import torch
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
from omni.physx import get_physx_simulation_interface
from omni.physx.bindings._physx import SimulationEvent
from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
from isaaclab.utils.math import quat_apply, quat_apply_inverse
from scipy.spatial.transform import Rotation
from assets import SIM_DIR, capsule, collision, make_joint
from geometry import RING_RADIUS, WIRE_RADIUS
from harvest_env import HarvestEnv
from ready_pose import read_left_pick_ready, ARM_JOINT_NAMES, READY_STATE, SRDF_PATH, POLICY_SCHEMA

SCENE_SOURCE = SIM_DIR / 'scenes/farmily_greenhouse_robot.usd'
STEM_SOURCE = SIM_DIR / 'env_usd/tomato_stem_v8_with_rotated90_cluster_harvestable_FIXED.usd'


class GreenhouseHarvestEnv(HarvestEnv):
    def __init__(self, cfg, **kwargs):
        if cfg.scene.num_envs != 1 or cfg.curriculum != 'approach':
            raise ValueError('The original greenhouse currently supports one environment and curriculum approach')
        super().__init__(cfg, **kwargs)

    def _setup_scene(self):
        self.stage = self.scene.stage
        self.sim.carb_settings.set_bool('/physics/disableContactProcessing', False)
        root = '/World/envs/env_0'
        scene = Usd.Stage.Open(str(SCENE_SOURCE))
        cache = UsdGeom.XformCache()
        robot_tf = cache.GetLocalToWorldTransform(scene.GetPrimAtPath('/World/Robot'))
        rq = robot_tf.ExtractRotationQuat()
        self.cfg.robot.init_state.pos = tuple(robot_tf.ExtractTranslation())
        self.cfg.robot.init_state.rot = (rq.GetReal(), *rq.GetImaginary())
        # Reference each original scene child: asset transforms/materials remain composed from the source.
        for child in scene.GetDefaultPrim().GetChildren():
            if child.GetName() in ('Robot', 'PhysicsScene'):
                continue
            self.stage.DefinePrim(root + '/' + child.GetName()).GetReferences().AddReference(str(SCENE_SOURCE), child.GetPath())
        self.ready_joints = read_left_pick_ready()
        self.cfg.robot.init_state.joint_pos.update(self.ready_joints)
        self.robot = Articulation(self.cfg.robot)
        plant = UsdGeom.Xform.Define(self.stage, root + '/HarvestableStem')
        plant.GetPrim().GetReferences().AddReference(str(STEM_SOURCE))
        plant.AddTranslateOp().Set(Gf.Vec3d(*self.cfg.stem_position))
        plant.AddRotateZOp().Set(self.cfg.stem_yaw)
        plant.AddScaleOp().Set(Gf.Vec3d(self.cfg.stem_scale))
        self.stage.OverridePrim(str(plant.GetPath()) + '/PhysicsScene').SetActive(False)
        self.stage.OverridePrim(str(plant.GetPath()) + '/Joints').SetActive(False)
        cache = UsdGeom.XformCache()
        self.fruits = []
        self.anchors = []
        self.fruit_specs = []
        harvestables = plant.GetPrim().GetChild('Harvestables')
        for index, prim in enumerate(harvestables.GetChildren()):
            path = str(prim.GetPath())
            sphere = UsdGeom.Sphere(prim.GetChild('FruitCollider'))
            if not sphere:
                continue
            tf = cache.GetLocalToWorldTransform(prim)
            center_world = cache.GetLocalToWorldTransform(sphere.GetPrim()).ExtractTranslation()
            center_local = tf.GetInverse().Transform(center_world)
            radius_local = sphere.GetRadiusAttr().Get()
            direction = -np.array(center_local)
            distance = np.linalg.norm(direction)
            direction /= distance
            # A contact proxy spans the source fruit surface to its authored attachment.
            # Keep the original fruit/calyx visual meshes and spherical fruit collider.
            a = np.array(center_local) + direction * radius_local
            pedicel = capsule(self.stage, path + '/PedicelCollider', a, (0, 0, 0), 0.0015 / self.cfg.stem_scale)
            pedicel.MakeInvisible()
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateEnableCCDAttr(True)
            q = Gf.Transform(tf).GetRotation().GetQuat().GetNormalized()
            pose = [*tf.ExtractTranslation(), q.GetReal(), *q.GetImaginary()]
            anchor_path = root + f'/Attachment_{index:02d}'
            anchor = UsdGeom.Xform.Define(self.stage, anchor_path)
            anchor.AddTranslateOp().Set(Gf.Vec3d(*pose[:3]))
            anchor.AddOrientOp().Set(Gf.Quatf(pose[3], Gf.Vec3f(*pose[4:])))
            UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim()).CreateKinematicEnabledAttr(True)
            UsdPhysics.MassAPI.Apply(anchor.GetPrim()).CreateMassAttr(1.0)
            anchor_object = RigidObject(RigidObjectCfg(prim_path=anchor_path, spawn=None))
            self.anchors.append(anchor_object)
            self.scene.rigid_objects[f'anchor_{index}'] = anchor_object
            self.fruit_specs.append(dict(name=prim.GetName(), path=path, anchor=anchor_path, pose=pose,
                local_matrix=UsdGeom.Xformable(prim).GetLocalTransformation(),
                center=np.array(center_local)*self.cfg.stem_scale,
                axis=direction, radius=radius_local*self.cfg.stem_scale,
                neck=np.array(center_local)*self.cfg.stem_scale + direction*(radius_local*self.cfg.stem_scale + max(0, (distance-radius_local)*self.cfg.stem_scale)*0.5)))
            obj = RigidObject(RigidObjectCfg(prim_path=path, spawn=None))
            self.fruits.append(obj)
            self.scene.rigid_objects[prim.GetName()] = obj
        source = Usd.Stage.Open(str(STEM_SOURCE))
        for spec in self.fruit_specs:
            joint = UsdPhysics.FixedJoint(source.GetPrimAtPath('/World/Joints/TomatoJoint_' + spec['name'][-2:]))
            spec['force'] = self.cfg.break_force if self.cfg.override_break_force else joint.GetBreakForceAttr().Get()
            spec['torque'] = self.cfg.break_torque if self.cfg.override_break_torque else joint.GetBreakTorqueAttr().Get()
        # Source static plant meshes have no colliders. Use their actual triangles as static obstacles.
        static = plant.GetPrim().GetChild('StaticPlant')
        self.static_colliders = 0
        static_paths = []
        for prim in Usd.PrimRange(static):
            if prim.IsA(UsdGeom.Mesh):
                collision(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
                self.static_colliders += 1
                static_paths.append(prim.GetPath())
        for spec in self.fruit_specs:
            UsdPhysics.FilteredPairsAPI.Apply(self.stage.GetPrimAtPath(spec['path'])).CreateFilteredPairsRel().SetTargets(static_paths)
        names = [s['name'] for s in self.fruit_specs]
        if self.cfg.target_fruit not in names:
            raise ValueError(f'Unknown target {self.cfg.target_fruit}; available: {names}')
        self.target_index = names.index(self.cfg.target_fruit)
        self.target_spec = self.fruit_specs[self.target_index]
        self.fruit = self.fruits[self.target_index]
        self.scene.articulations['robot'] = self.robot
        self.scene.clone_environments(copy_from_source=True)
        self.scene.filter_collisions(global_prim_paths=[])
        self.cluster_joint_paths = []
        self._other_broken = False
        self._start_q = None
        self.provenance = dict(scene=str(SCENE_SOURCE), stem=str(STEM_SOURCE), target=self.cfg.target_fruit,
            stem_position=self.cfg.stem_position, stem_yaw=self.cfg.stem_yaw, stem_scale=self.cfg.stem_scale,
            fruit_count=len(self.fruits), static_plant_colliders=self.static_colliders,
            fruit_centers_world=[(np.array(s['pose'][:3])+Rotation.from_quat(s['pose'][4:]+[s['pose'][3]]).apply(s['center'])).tolist() for s in self.fruit_specs],
            break_force=self.target_spec['force'], break_torque=self.target_spec['torque'])

    def _target_geometry(self):
        spec = self.target_spec
        q = self.fruit.data.root_quat_w
        vector = lambda key: torch.tensor(spec[key], dtype=torch.float32, device=self.device).reshape(1, 3)
        center = self.fruit.data.root_pos_w + quat_apply(q, vector('center'))
        neck = self.fruit.data.root_pos_w + quat_apply(q, vector('neck'))
        axis = quat_apply(q, vector('axis'))
        return center, neck, axis

    def _geometry(self):
        pos, quat = self.ring_pose()
        center, neck_w, axis_w = self._target_geometry()
        local = quat_apply_inverse(quat, center-pos)
        neck = quat_apply_inverse(quat, neck_w-pos)
        axis = quat_apply_inverse(quat, axis_w)
        radius = self.target_spec['radius']
        clear = (torch.linalg.vector_norm(local[:, [0, 2]], dim=1)+radius < RING_RADIUS-WIRE_RADIUS+0.001) & (local[:, 0] < -0.003)
        # The ROS GUI uses a downward ring normal. Either normal describes the
        # same physical aperture; requiring +Y rejects the GUI's valid pose.
        inside = clear & (local[:, 1].abs() < radius+0.002) & (axis[:, 1].abs() > 0.7)
        lip = torch.tensor([-RING_RADIUS+WIRE_RADIUS+0.0015, 0, 0], device=self.device)
        insert = torch.tensor([-0.012, 0, 0], device=self.device)
        return local, neck, axis, inside, torch.linalg.vector_norm(neck-lip, dim=1), torch.linalg.vector_norm(local-insert, dim=1)

    def _on_event(self, event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            path = str(PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath']))
            if path in self.cluster_joint_paths:
                if hasattr(self, 'contact_diagnostics'):
                    self.contact_diagnostics.append(dict(event='break',path=path,step=getattr(self,'contact_diagnostic_step',None)))
                if path == self.joint_paths[0]:
                    self._broken[0] = True
                else:
                    self._other_broken = True

    def _on_contacts(self, headers, data):
        for header in headers:
            if not header.num_contact_data:
                continue
            a, b = [str(PhysicsSchemaTools.intToSdfPath(p)) for p in (header.collider0, header.collider1)]
            if '/Robot/' not in a and '/Robot/' not in b:
                continue
            if '/Robot/' in a and '/Robot/' in b:
                continue
            pair = a+' '+b
            impulse = sum(np.linalg.norm(tuple(data[j].impulse)) for j in range(header.contact_data_offset, header.contact_data_offset+header.num_contact_data))
            force = float(impulse/self.physics_dt)
            if hasattr(self, 'contact_diagnostics') and force > .01:
                self.contact_diagnostics.append(dict(event='contact',a=a,b=b,force=force,step=getattr(self,'contact_diagnostic_step',None),
                    positions=[list(data[j].position) for j in range(header.contact_data_offset,header.contact_data_offset+header.num_contact_data)]))
            self._peak_force[0] = max(self._peak_force[0], force)
            if '/RingCollision/' in pair and self.target_spec['path']+'/PedicelCollider' in pair and force > .005:
                self._stem_contact[0] = True
            elif force > .1:
                self._bad_contact[0] = True

    def _get_dones(self):
        done, timeout = super()._get_dones()
        if self._other_broken:
            self.success[:] = False
            self.invalid_break[:] = True
            done[:] = True
        return done, timeout

    def _control_pose(self):
        # Arm deltas are relative to the moving lift, so holding the arm steady
        # does not make IK cancel the independent lift action.
        pos, quat = self.ring_pose()
        return pos - self.robot.data.body_pos_w[:, self.platform_id], quat

    def _pre_physics_step(self, actions):
        super()._pre_physics_step(actions)
        limits = self.robot.data.soft_joint_pos_limits[:, self.lift_id]
        lift_target = self.targets[:, self.lift_id] + self.actions[:, 6]*self.cfg.lift_speed*self.step_dt
        self.targets[:, self.lift_id] = lift_target.clamp(limits[:, 0], limits[:, 1])

    def _get_observations(self):
        obs = super()._get_observations()['policy']
        lift = torch.stack((self.robot.data.joint_pos[:, self.lift_id],
                            self.robot.data.joint_vel[:, self.lift_id]), dim=1)
        return {'policy': torch.cat((obs, lift), dim=1)}

    def _calibrate_start(self):
        """Use the GUI's exact arm state and solve only the lift's initial height."""
        self.lift_id = self.robot.find_joints('farmily_lift_height_joint')[0][0]
        self.platform_id = self.robot.find_bodies('farmily_lift_platform')[0][0]
        q = self.robot.data.default_joint_pos.clone()
        for index, name in zip(self.arm_ids, ARM_JOINT_NAMES):
            q[:, index] = self.ready_joints[name]
        limits = self.robot.data.soft_joint_pos_limits
        if torch.any(q[:, self.arm_ids] < limits[:, self.arm_ids, 0]) or torch.any(q[:, self.arm_ids] > limits[:, self.arm_ids, 1]):
            raise ValueError('ROS PICK_READY exceeds the imported arm joint limits')
        q[:, self.lift_id] = 0.
        self.robot.write_joint_state_to_sim(q, torch.zeros_like(q))
        self.scene.update(self.physics_dt)
        # link0 is the mounting surface, 12.5 mm above the platform body origin.
        cache = UsdGeom.XformCache()
        mount = self.stage.GetPrimAtPath('/World/envs/env_0/Robot/farmily_lift_platform/link0')
        platform = self.stage.GetPrimAtPath('/World/envs/env_0/Robot/farmily_lift_platform')
        if not mount or not platform:
            raise ValueError('Imported robot must retain its lift platform and link0 mounting frame')
        relative = cache.GetLocalToWorldTransform(mount) * cache.GetLocalToWorldTransform(platform).GetInverse()
        self.mount_offset = torch.tensor([list(relative.ExtractTranslation())], device=self.device, dtype=torch.float32)
        platform_pos = self.robot.data.body_pos_w[:, self.platform_id]
        platform_quat = self.robot.data.body_quat_w[:, self.platform_id]
        mount_z = (platform_pos + quat_apply(platform_quat, self.mount_offset))[0, 2]
        reference_z = self.ring_pose()[0][0, 2] if self.cfg.lift_height_reference == 'ring' else mount_z
        cluster_z = float(np.mean(np.asarray(self.provenance['fruit_centers_world'])[:, 2]))
        desired_z = cluster_z - self.cfg.lift_start_below
        requested_q = desired_z - float(reference_z)
        low, high = limits[0, self.lift_id].tolist()
        if not low <= requested_q <= high:
            raise ValueError(f'Requested lift start {requested_q:.4f} m is outside [{low:.4f}, {high:.4f}]; cannot meet the specified height offset')
        q[:, self.lift_id] = requested_q
        self.provenance.update(policy_schema=POLICY_SCHEMA, ready_state=READY_STATE, ready_srdf=str(SRDF_PATH),
            ready_arm_joints=self.ready_joints, height_reference=self.cfg.lift_height_reference,
            cluster_center_height_m=cluster_z, initial_reference_height_m=desired_z,
            lift_start_below_m=self.cfg.lift_start_below, initial_lift_joint_m=requested_q,
            lift_joint_limits_m=[low, high], lift_speed_m_s=self.cfg.lift_speed,
            start_joint_positions=q[0].tolist(), action_size=7, observation_size=39)
        print(f'[GREENHOUSE] {READY_STATE}: cluster z={cluster_z:.4f} m; {self.cfg.lift_height_reference} z={desired_z:.4f} m; lift q={requested_q:.4f} m', flush=True)
        return q

    def _reset_idx(self, env_ids):
        if env_ids is not None and len(env_ids) == 0:
            return
        if self.episode_length_buf[0] > 0:
            record = dict(env=0, success=bool(self.success[0]), invalid_break=bool(self.invalid_break[0]), wrong_fruit_break=self._other_broken, steps=int(self.episode_length_buf[0]))
            self.last_episode = (self.last_episode+[record])[-100:]
            self.total_successes += int(record['success'])
            self.total_invalid_breaks += int(record['invalid_break'])
            self.total_episodes += 1
        for path in self.cluster_joint_paths:
            self.stage.RemovePrim(path)
        self.cluster_joint_paths = []
        get_physx_simulation_interface().flush_changes()
        # Skip the fixture's reset which relocates its procedural fruit.
        from isaaclab.envs import DirectRLEnv
        DirectRLEnv._reset_idx(self, env_ids)
        poses = []
        for fruit, spec in zip(self.fruits, self.fruit_specs):
            prim = self.stage.GetPrimAtPath(spec['path'])
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(spec['local_matrix'])
            UsdPhysics.RigidBodyAPI(prim).CreateVelocityAttr(Gf.Vec3f(0))
            UsdPhysics.RigidBodyAPI(prim).CreateAngularVelocityAttr(Gf.Vec3f(0))
            pose = torch.tensor([spec['pose']], dtype=torch.float32, device=self.device)
            poses.append(pose)
            fruit.write_root_pose_to_sim(pose)
            fruit.write_root_velocity_to_sim(torch.zeros(1, 6, device=self.device))
            fruit.set_external_force_and_torque(torch.zeros(1, 1, 3, device=self.device), torch.zeros(1, 1, 3, device=self.device))
        # Attachment frames never move. Preserve authored USD transforms and
        # synchronize only their native tensor poses.
        for anchor, pose in zip(self.anchors, poses):
            anchor.write_root_pose_to_sim(pose)
        get_physx_simulation_interface().flush_changes()
        self.sim.forward()
        self.scene.update(self.physics_dt)
        if self._start_q is None:
            self._start_q = self._calibrate_start()
        q = self._start_q.clone()
        # Robot start randomization is deliberately disabled until original-scene calibration is validated.
        self.robot.write_joint_state_to_sim(q, torch.zeros_like(q))
        self.targets[:] = q
        self.robot.set_joint_position_target(q)
        self.joint_generation[0] += 1
        for index, (fruit, spec, pose) in enumerate(zip(self.fruits, self.fruit_specs, poses)):
            path = f'/World/envs/env_0/HarvestJoint_{index:02d}_{self.joint_generation[0]}'
            make_joint(self.stage, path, spec['path'], spec['pose'][:3], spec['pose'][3:], (0, 0, 0),
                       spec['force'], spec['torque'], anchor_path=spec['anchor'])
            self.cluster_joint_paths.append(path)
            fruit.write_root_pose_to_sim(pose)
            fruit.write_root_velocity_to_sim(torch.zeros(1, 6, device=self.device))
        self.joint_paths[0] = self.cluster_joint_paths[self.target_index]
        get_physx_simulation_interface().flush_changes()
        for anchor, pose in zip(self.anchors, poses):
            anchor.write_root_pose_to_sim(pose)
        self._broken.fill(False)
        self._other_broken = False
        self._stem_contact.fill(False)
        self._bad_contact.fill(False)
        self._peak_force.fill(0)
        self.inserted[:] = self.cfg.curriculum in ('hook', 'inserted')
        self.hooked[:] = False
        self.hook_age[:] = 1000
        self.success[:] = False
        self.invalid_break[:] = False
        self.actions[:] = 0
        self.previous_actions[:] = 0
        self.ik.reset()
        self.sim.forward()
        self.scene.update(self.physics_dt)
        self.nominal_ring_pos, self.nominal_ring_quat = [x.clone() for x in self.ring_pose()]
        _, _, _, _, lip, insertion = self._geometry()
        self.previous_potential[:] = torch.where(self.inserted, 1-torch.tanh(lip/.04), 1-torch.tanh(insertion/self.cfg.insertion_reward_scale))
