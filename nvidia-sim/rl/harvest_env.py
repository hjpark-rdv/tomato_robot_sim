"""State-based ring harvesting with native PhysX break events and resettable joints."""

import math
import re

import numpy as np
import torch
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physx.bindings._physx import SimulationEvent

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import combine_frame_transforms, quat_apply, quat_apply_inverse, quat_mul, skew_symmetric_matrix

from assets import ROBOT_ASSET, capsule, collision, make_joint
from geometry import FRUIT_RADIUS, FRUIT_MASS, STEM_RADIUS, STEM_LENGTH, BREAK_FORCE, BREAK_TORQUE, RING_RADIUS, WIRE_RADIUS


@configclass
class HarvestEnvCfg(DirectRLEnvCfg):
    decimation = 4
    episode_length_s = 6.0
    action_space = 6
    observation_space = 36
    state_space = 0
    curriculum = "hook"  # hook, inserted, approach
    position_jitter = 0.001
    break_force = BREAK_FORCE
    break_torque = BREAK_TORQUE
    break_randomization = 0.1
    max_target_distance = 0.25
    insertion_reward_scale = 0.06
    translation_step = 0.0015
    rotation_step = 0.02
    sim = sim_utils.SimulationCfg(
        dt=1/240, render_interval=4, use_fabric=False, enable_scene_query_support=True,
        physx=sim_utils.PhysxCfg(enable_ccd=True, min_position_iteration_count=16, min_velocity_iteration_count=4),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4, restitution=0.0),
    )
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=3.0, replicate_physics=False)
    robot = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=str(ROBOT_ASSET),
                                 rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True)),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 0.8),
            joint_pos={"farmily_lift_height_joint": 0.25, "base": 0.0, "shoulder": -0.3,
                       "elbow": 1.2, "wrist1": -0.9, "wrist2": 1.57, "wrist3": 1.541930975},
        ),
        actuators={
            "arm": ImplicitActuatorCfg(joint_names_expr=["base", "shoulder", "elbow", "wrist[123]"],
                                       stiffness=800.0, damping=40.0, effort_limit_sim=80.0, velocity_limit_sim=1.0),
            "lift": ImplicitActuatorCfg(joint_names_expr=["farmily_lift_height_joint"],
                                        stiffness=50000.0, damping=2500.0, effort_limit_sim=10000.0),
        },
    )


class HarvestEnv(DirectRLEnv):
    cfg: HarvestEnvCfg

    def __init__(self, cfg, **kwargs):
        if cfg.sim.device != "cpu":
            raise ValueError("Native USD joint recreation currently requires CPU physics")
        if cfg.curriculum not in ("hook", "inserted", "approach"):
            raise ValueError("curriculum must be hook, inserted, or approach")
        super().__init__(cfg, **kwargs)
        self.arm_ids, _ = self.robot.find_joints(["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"], preserve_order=True)
        self.tool_id = self.robot.find_bodies("link6")[0][0]
        # Imported fixed frames remain in USD even though the bodies are merged.
        cache = UsdGeom.XformCache()
        link = self.stage.GetPrimAtPath("/World/envs/env_0/Robot/link6")
        ring = self.stage.GetPrimAtPath("/World/envs/env_0/Robot/link6/tcp/tomato_gripper/ring_center")
        tf = cache.GetLocalToWorldTransform(ring) * cache.GetLocalToWorldTransform(link).GetInverse()
        q = tf.ExtractRotationQuat()
        self.offset_pos = torch.tensor(tuple(tf.ExtractTranslation()), device=self.device, dtype=torch.float32).repeat(self.num_envs, 1)
        self.offset_quat = torch.tensor((q.GetReal(), *q.GetImaginary()), device=self.device, dtype=torch.float32).repeat(self.num_envs, 1)
        # Isaac Lab initializes default joint buffers without applying them until reset.
        initial_q = self.robot.data.default_joint_pos.clone()
        self.robot.write_joint_state_to_sim(initial_q, torch.zeros_like(initial_q))
        self.robot.set_joint_position_target(initial_q)
        self.scene.write_data_to_sim()
        self.sim.step(render=False)
        self.scene.update(self.physics_dt)
        self.nominal_ring_pos, self.nominal_ring_quat = [x.clone() for x in self.ring_pose()]
        self.ik = DifferentialIKController(DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"), self.num_envs, self.device)
        self.actions = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)
        self.previous_actions = torch.zeros_like(self.actions)
        self.targets = self.robot.data.default_joint_pos.clone()
        self._broken = np.zeros(self.num_envs, dtype=bool)
        self._stem_contact = np.zeros(self.num_envs, dtype=bool)
        self._bad_contact = np.zeros(self.num_envs, dtype=bool)
        self._peak_force = np.zeros(self.num_envs, dtype=np.float32)
        self.inserted = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.hooked = torch.zeros_like(self.inserted)
        self.hook_age = torch.full((self.num_envs,), 1000, dtype=torch.long, device=self.device)
        self.previous_potential = torch.zeros(self.num_envs, device=self.device)
        self.success = torch.zeros_like(self.inserted)
        self.invalid_break = torch.zeros_like(self.inserted)
        self.last_episode = []
        self.total_successes = 0
        self.total_invalid_breaks = 0
        self.total_episodes = 0
        self._contact_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contacts)
        self._event_sub = get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(self._on_event)
        self.joint_generation = [0] * self.num_envs
        self.joint_paths = [f"/World/envs/env_{i}/BreakJoint" for i in range(self.num_envs)]
        try:
            self.reset()
        except Exception:
            # In particular, an unreachable original-scene target must release PhysX before Kit closes.
            self.close()
            raise

    def _setup_scene(self):
        self.stage = self.scene.stage
        # This task uses collider-level native reports, including pedicel identity.
        self.sim.carb_settings.set_bool("/physics/disableContactProcessing", False)
        self.robot = Articulation(self.cfg.robot)
        root_path = "/World/envs/env_0/Tomato"
        root = UsdGeom.Xform.Define(self.stage, root_path)
        root.AddTranslateOp().Set(Gf.Vec3d(2, 0, 1))
        root.AddOrientOp().Set(Gf.Quatf(1.0))
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(root.GetPrim())
        mass.CreateMassAttr(FRUIT_MASS)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
        inertia = 0.4 * FRUIT_MASS * FRUIT_RADIUS**2
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
        rb.CreateEnableCCDAttr(True)
        rb.CreateSolverPositionIterationCountAttr(16)
        rb.CreateSolverVelocityIterationCountAttr(4)
        PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim()).CreateThresholdAttr(0.0)
        fruit = UsdGeom.Sphere.Define(self.stage, root_path + "/FruitCollider")
        fruit.CreateRadiusAttr(FRUIT_RADIUS)
        fruit.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.035, 0.015)])
        collision(fruit.GetPrim())
        capsule(self.stage, root_path + "/PedicelCollider", (0, 0, FRUIT_RADIUS),
                (0, 0, FRUIT_RADIUS + STEM_LENGTH), STEM_RADIUS)
        stem = UsdGeom.Xform.Define(self.stage, "/World/envs/env_0/Stem")
        stem.AddTranslateOp().Set(Gf.Vec3d(2, 0, 1 + FRUIT_RADIUS + STEM_LENGTH))
        stem.AddOrientOp().Set(Gf.Quatf(1.0))
        UsdPhysics.RigidBodyAPI.Apply(stem.GetPrim()).CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(stem.GetPrim()).CreateMassAttr(1.0)
        capsule(self.stage, str(stem.GetPath()) + "/StemCollider", (0, 0, 0.006), (0, 0, 0.05), 0.0025)
        # Attach only after cloning and assigning world poses in _reset_idx.
        self.scene.clone_environments(copy_from_source=True)
        self.scene.filter_collisions(global_prim_paths=[])
        self.fruit = RigidObject(RigidObjectCfg(prim_path="/World/envs/env_.*/Tomato", spawn=None))
        self.stem = RigidObject(RigidObjectCfg(prim_path="/World/envs/env_.*/Stem", spawn=None))
        self.scene.articulations["robot"] = self.robot
        self.scene.rigid_objects["fruit"] = self.fruit
        self.scene.rigid_objects["stem"] = self.stem
        light = sim_utils.DomeLightCfg(intensity=1800)
        light.func("/World/Light", light)

    @staticmethod
    def _env_index(path):
        match = re.search(r"/env_(\d+)/", path)
        return int(match.group(1)) if match else None

    def _on_event(self, event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            path = str(PhysicsSchemaTools.decodeSdfPath(*event.payload["jointPath"]))
            i = self._env_index(path)
            if i is not None and i < self.num_envs and path == self.joint_paths[i]:
                self._broken[i] = True

    def _on_contacts(self, headers, data):
        for header in headers:
            if not header.num_contact_data:
                continue
            a = str(PhysicsSchemaTools.intToSdfPath(header.collider0))
            b = str(PhysicsSchemaTools.intToSdfPath(header.collider1))
            i = self._env_index(a)
            if i is None or i >= self.num_envs:
                continue
            pair = a + " " + b
            if "/Robot/" not in pair or ("/Tomato/" not in pair and "/Stem/" not in pair):
                continue
            impulse = sum(np.linalg.norm(tuple(data[j].impulse)) for j in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data))
            force = float(impulse / self.physics_dt)
            self._peak_force[i] = max(self._peak_force[i], force)
            valid_pair = "/RingCollision/" in pair and "/PedicelCollider" in pair
            if valid_pair and force > 0.005:
                self._stem_contact[i] = True
            elif force > 0.1:
                self._bad_contact[i] = True

    def ring_pose(self):
        return combine_frame_transforms(self.robot.data.body_pos_w[:, self.tool_id],
                                        self.robot.data.body_quat_w[:, self.tool_id], self.offset_pos, self.offset_quat)

    def _control_pose(self):
        return self.ring_pose()

    def _pre_physics_step(self, actions):
        self.previous_actions.copy_(self.actions)
        self.actions = torch.nan_to_num(actions).clamp(-1, 1)
        self._stem_contact.fill(False)
        self._bad_contact.fill(False)
        self._peak_force.fill(0)
        pos, quat = self._control_pose()
        command = self.actions[:, :6].clone()
        command[:, :3] = quat_apply(quat, command[:, :3] * self.cfg.translation_step)
        command[:, 3:] = quat_apply(quat, command[:, 3:] * self.cfg.rotation_step)
        self.ik.set_command(command, pos, quat)

    def _apply_action(self):
        pos, quat = self._control_pose()
        jacobian = self.robot.root_physx_view.get_jacobians()[:, self.tool_id - 1, :, :].clone()
        offset_w = quat_apply(self.robot.data.body_quat_w[:, self.tool_id], self.offset_pos)
        jacobian[:, :3] -= torch.bmm(skew_symmetric_matrix(offset_w), jacobian[:, 3:])
        q = self.robot.data.joint_pos[:, self.arm_ids]
        desired = self.ik.compute(pos, quat, jacobian[:, :, self.arm_ids], q)
        # Bound each physics-step command and honor URDF limits.
        desired = q + (desired - q).clamp(-0.008, 0.008)
        limits = self.robot.data.soft_joint_pos_limits[:, self.arm_ids]
        self.targets[:, self.arm_ids] = desired.clamp(limits[:, :, 0], limits[:, :, 1])
        self.robot.set_joint_position_target(self.targets)

    def _geometry(self):
        pos, quat = self.ring_pose()
        fruit_pos = self.fruit.data.root_pos_w
        fruit_quat = self.fruit.data.root_quat_w
        local = quat_apply_inverse(quat, fruit_pos - pos)
        axis = torch.zeros_like(local)
        axis[:, 2] = 1
        stem_axis_w = quat_apply(fruit_quat, axis)
        stem_local = quat_apply_inverse(quat, stem_axis_w)
        neck = quat_apply_inverse(quat, fruit_pos + stem_axis_w * (FRUIT_RADIUS + 0.012) - pos)
        # Half-ring: fruit must lie in the negative-X semicircle with full-radius clearance.
        radial = torch.linalg.vector_norm(local[:, [0, 2]], dim=1)
        clear = (radial + FRUIT_RADIUS < RING_RADIUS - WIRE_RADIUS + 0.001) & (local[:, 0] < -0.003)
        inside = clear & (local[:, 1].abs() < FRUIT_RADIUS + 0.002) & (stem_local[:, 1] > 0.7)
        lip = torch.zeros_like(neck)
        lip[:, 0] = -RING_RADIUS + WIRE_RADIUS + STEM_RADIUS
        lip_distance = torch.linalg.vector_norm(neck - lip, dim=1)
        insertion_target = torch.zeros_like(local)
        insertion_target[:, 0] = -0.012
        insertion_distance = torch.linalg.vector_norm(local - insertion_target, dim=1)
        return local, neck, stem_local, inside, lip_distance, insertion_distance

    def _get_dones(self):
        local, neck, axis, inside, lip_distance, insertion_distance = self._geometry()
        self.just_inserted = inside & ~self.inserted
        self.inserted |= inside
        contact = torch.as_tensor(self._stem_contact.copy(), device=self.device)
        self.just_hooked = contact & self.inserted & ~self.hooked
        self.hooked |= contact & self.inserted
        self.hook_age += 1
        self.hook_age[contact & self.inserted] = 0
        broken = torch.as_tensor(self._broken.copy(), device=self.device)
        self.success = broken & self.inserted & self.hooked & (self.hook_age <= 2)
        self.invalid_break = broken & ~self.success
        self.potential = torch.where(self.inserted, 1 - torch.tanh(lip_distance / 0.04),
                                     1 - torch.tanh(insertion_distance / self.cfg.insertion_reward_scale))
        out_of_bounds = torch.linalg.vector_norm(local, dim=1) > self.cfg.max_target_distance
        invalid_state = ~torch.isfinite(self.robot.data.joint_pos).all(dim=1) | ~torch.isfinite(local).all(dim=1)
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        self.extras["log"] = {"success_count": float(self.total_successes), "invalid_break_count": float(self.total_invalid_breaks)}
        return broken | out_of_bounds | invalid_state, timeout

    def _get_rewards(self):
        force = torch.as_tensor(self._peak_force.copy(), device=self.device)
        bad = torch.as_tensor(self._bad_contact.copy(), device=self.device)
        reward = (5 * (self.potential - self.previous_potential)
                  + 2 * self.just_inserted.float() + 3 * self.just_hooked.float()
                  + 30 * self.success.float() - 15 * self.invalid_break.float()
                  - 0.2 * bad.float() - 0.03 * (force - 5).clamp(min=0, max=100)
                  - 0.005 * (self.actions - self.previous_actions).square().sum(dim=1) - 0.002)
        self.previous_potential.copy_(self.potential)
        return reward

    def _get_observations(self):
        local, neck, axis, _, _, _ = self._geometry()
        pos, quat = self.ring_pose()
        velocity = self.robot.data.body_vel_w[:, self.tool_id]
        obs = torch.cat((local, neck, axis,
                         self.robot.data.joint_pos[:, self.arm_ids], self.robot.data.joint_vel[:, self.arm_ids],
                         velocity, self.actions, self.inserted[:, None].float(), self.hooked[:, None].float(),
                         torch.as_tensor(self._peak_force.copy(), device=self.device)[:, None] / 10), dim=1)
        return {"policy": obs}

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        ids = env_ids.tolist()
        for i in ids:
            if self.episode_length_buf[i] > 0:
                record = {"env": i, "success": bool(self.success[i]), "invalid_break": bool(self.invalid_break[i]),
                          "steps": int(self.episode_length_buf[i])}
                self.last_episode.append(record)
                self.last_episode = self.last_episode[-100:]
                self.total_successes += int(record["success"])
                self.total_invalid_breaks += int(record["invalid_break"])
                self.total_episodes += 1
            self.stage.RemovePrim(self.joint_paths[i])
        # Native joints are destroyed before bodies move; new joints are created at matching frames.
        get_physx_simulation_interface().flush_changes()
        super()._reset_idx(env_ids)
        q = self.robot.data.default_joint_pos[env_ids].clone()
        self.robot.write_joint_state_to_sim(q, torch.zeros_like(q), env_ids=env_ids)
        self.targets[env_ids] = q
        self.robot.set_joint_position_target(self.targets[env_ids], env_ids=env_ids)
        pos = self.nominal_ring_pos[env_ids]
        quat = self.nominal_ring_quat[env_ids]
        offsets = {"hook": (-0.023, -0.024, 0), "inserted": (-0.012, -0.006, 0), "approach": (-0.012, -0.055, 0)}
        local = torch.tensor(offsets[self.cfg.curriculum], device=self.device).repeat(len(ids), 1)
        local += (torch.rand_like(local) * 2 - 1) * self.cfg.position_jitter
        fruit_pos = pos + quat_apply(quat, local)
        # Fruit local +Z points along the gripper's +Y (normal to the ring plane).
        align = torch.tensor([math.sqrt(0.5), -math.sqrt(0.5), 0, 0], device=self.device).repeat(len(ids), 1)
        fruit_quat = quat_mul(quat, align)
        pose = torch.cat((fruit_pos, fruit_quat), dim=1)
        self.fruit.write_root_pose_to_sim(pose, env_ids=env_ids)
        self.fruit.write_root_velocity_to_sim(torch.zeros(len(ids), 6, device=self.device), env_ids=env_ids)
        anchor_local = torch.zeros(len(ids), 3, device=self.device)
        anchor_local[:, 2] = FRUIT_RADIUS + STEM_LENGTH
        anchor_pos = fruit_pos + quat_apply(fruit_quat, anchor_local)
        stem_pose = torch.cat((anchor_pos, fruit_quat), dim=1)
        self.stem.write_root_pose_to_sim(stem_pose, env_ids=env_ids)
        for j, i in enumerate(ids):
            # Joint creation reads USD poses, whereas tensor writes update PhysX.
            # Synchronize both representations before attaching the new joint.
            fruit_prim = self.stage.GetPrimAtPath(f"/World/envs/env_{i}/Tomato")
            fq = fruit_quat[j].tolist()
            fruit_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*(fruit_pos[j] - self.scene.env_origins[i]).tolist()))
            fruit_prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(fq[0], Gf.Vec3f(*fq[1:])))
            UsdPhysics.RigidBodyAPI(fruit_prim).CreateVelocityAttr(Gf.Vec3f(0))
            UsdPhysics.RigidBodyAPI(fruit_prim).CreateAngularVelocityAttr(Gf.Vec3f(0))
            stem = self.stage.GetPrimAtPath(f"/World/envs/env_{i}/Stem")
            stem.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*(anchor_pos[j] - self.scene.env_origins[i]).tolist()))
            stem.GetAttribute("xformOp:orient").Set(Gf.Quatf(fq[0], Gf.Vec3f(*fq[1:])))
        get_physx_simulation_interface().flush_changes()
        for j, i in enumerate(ids):
            self.joint_generation[i] += 1
            self.joint_paths[i] = f"/World/envs/env_{i}/BreakJoint_{self.joint_generation[i]}"
            factor = 1 + self.cfg.break_randomization * float(torch.rand(()) * 2 - 1)
            make_joint(self.stage, self.joint_paths[i], f"/World/envs/env_{i}/Tomato",
                       anchor_pos[j].tolist(), fruit_quat[j].tolist(), anchor_local[j].tolist(),
                       self.cfg.break_force * factor, self.cfg.break_torque * factor,
                       anchor_path=f"/World/envs/env_{i}/Stem")
            stem = self.stage.GetPrimAtPath(f"/World/envs/env_{i}/Stem")
            stem.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*(anchor_pos[j] - self.scene.env_origins[i]).tolist()))
            fq = fruit_quat[j].tolist()
            stem.GetAttribute("xformOp:orient").Set(Gf.Quatf(fq[0], Gf.Vec3f(*fq[1:])))
            self._broken[i] = False
            self._stem_contact[i] = False
            self._bad_contact[i] = False
            self._peak_force[i] = 0
        get_physx_simulation_interface().flush_changes()
        self.fruit.write_root_pose_to_sim(pose, env_ids=env_ids)
        self.fruit.write_root_velocity_to_sim(torch.zeros(len(ids), 6, device=self.device), env_ids=env_ids)
        self.stem.write_root_pose_to_sim(stem_pose, env_ids=env_ids)
        self.inserted[env_ids] = self.cfg.curriculum in ("hook", "inserted")
        self.hooked[env_ids] = False
        self.hook_age[env_ids] = 1000
        self.success[env_ids] = False
        self.invalid_break[env_ids] = False
        self.actions[env_ids] = 0
        self.previous_actions[env_ids] = 0
        # Start shaping from the actual reset configuration, without a reset bonus.
        insertion_distance = torch.linalg.vector_norm(local - torch.tensor([-0.012, 0, 0], device=self.device), dim=1)
        neck_local = local + torch.tensor([0, FRUIT_RADIUS + 0.012, 0], device=self.device)
        lip_local = torch.tensor([-RING_RADIUS + WIRE_RADIUS + STEM_RADIUS, 0, 0], device=self.device)
        lip_distance = torch.linalg.vector_norm(neck_local - lip_local, dim=1)
        self.previous_potential[env_ids] = torch.where(self.inserted[env_ids],
            1 - torch.tanh(lip_distance / 0.04), 1 - torch.tanh(insertion_distance / self.cfg.insertion_reward_scale))
        self.ik.reset(env_ids)
        self.sim.forward()

    def close(self):
        self._contact_sub = None
        self._event_sub = None
        super().close()
