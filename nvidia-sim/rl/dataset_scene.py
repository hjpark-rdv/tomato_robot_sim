"""Isolated cloned greenhouse slots sharing one Isaac Lab scene/physics step.

Per-slot Isaac asset views deliberately preserve the original CPU break-joint
and elastic-plant implementations. This is not a process pool or GPU PhysX port.
"""
import copy
import re
import time
import numpy as np
import torch
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysicsSchemaTools
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physx.bindings._physx import SimulationEvent
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
from isaacsim.core.cloner import Cloner
from isaaclab.utils.math import combine_frame_transforms
from greenhouse_env import GreenhouseHarvestEnv
from assets import make_joint
from hook_motion import RobotKinematics


class Slot(GreenhouseHarvestEnv):
    @property
    def num_envs(self):
        return 1

    def close(self):
        # Only DatasetScene owns the shared SimulationContext.
        pass

    def __init__(self, cfg, sim, scene, index=0):
        # Do not construct another DirectRLEnv or SimulationContext.
        self.cfg, self.sim, self.scene, self.index = cfg, sim, scene, index
        self.root = f'/World/envs/env_{index}'
        self.stage = scene.stage
        self.contact_diagnostics = []
        self.contact_diagnostic_step = 0
        self._broken = np.zeros(1, bool)
        self._other_broken = False
        self.cluster_joint_paths = []
        self.joint_paths = ['']
        self.generation = 0
        self.start_q = None
        self.dataset_origin = np.zeros(3)
        self.command_dirty = True

    def initialize(self):
        self.arm_ids, _ = self.robot.find_joints(['base','shoulder','elbow','wrist1','wrist2','wrist3'], preserve_order=True)
        self.tool_id = self.robot.find_bodies('link6')[0][0]
        cache = UsdGeom.XformCache()
        link = self.stage.GetPrimAtPath(self.root+'/Robot/link6')
        ring = self.stage.GetPrimAtPath(self.root+'/Robot/link6/tcp/tomato_gripper/ring_center')
        tf = cache.GetLocalToWorldTransform(ring)*cache.GetLocalToWorldTransform(link).GetInverse()
        q = tf.ExtractRotationQuat()
        self.offset_pos = torch.tensor([list(tf.ExtractTranslation())], dtype=torch.float32)
        self.offset_quat = torch.tensor([[q.GetReal(),*q.GetImaginary()]], dtype=torch.float32)
        self.targets = self.robot.data.default_joint_pos.clone()
        self.reset_slot()
        self.pose_search_kin = RobotKinematics(self)

    def reset_slot(self):
        for path in self.cluster_joint_paths:
            self.stage.RemovePrim(path)
        self.cluster_joint_paths = []
        get_physx_simulation_interface().flush_changes()
        self.robot.reset(); self.elastic.articulation.reset()
        self.elastic.reset()
        for fruit, spec in zip(self.fruits,self.fruit_specs):
            fruit.reset()
            pose = torch.tensor([spec['pose']], dtype=torch.float32)
            fruit.write_root_pose_to_sim(pose)
            fruit.write_root_velocity_to_sim(torch.zeros(1,6))
            # reset() already clears the wrench. Setting a zero permanent wrench
            # activates 11 redundant force uploads per slot and physics step.
            if getattr(self.cfg,'dataset_physics_sync','legacy')=='legacy':
                fruit.set_external_force_and_torque(torch.zeros(1,1,3), torch.zeros(1,1,3))
        self.sim.forward(); self.scene.update(self.physics_dt)
        self.command_dirty = True
        if self.start_q is None:
            self.start_q = self._calibrate_start()
        self.robot.write_joint_state_to_sim(self.start_q, torch.zeros_like(self.start_q))
        self.targets[:] = self.start_q
        self.robot.set_joint_position_target(self.targets)
        self.robot.write_joint_stiffness_to_sim(self.robot.data.default_joint_stiffness)
        self.robot.write_joint_damping_to_sim(self.robot.data.default_joint_damping)
        self.sim.physics_sim_view.update_articulations_kinematic()
        get_physx_interface().update_transformations(False,True,True)
        self.generation += 1
        for i, spec in enumerate(self.fruit_specs):
            path = self.root+f'/HarvestJoint_{i:02d}_{self.generation}'
            make_joint(self.stage,path,spec['path'],spec['pose'][:3],spec['pose'][3:],(0,0,0),
                       spec['force'],spec['torque'],anchor_path=spec['anchor'])
            self.cluster_joint_paths.append(path)
        self.joint_paths[0] = self.cluster_joint_paths[self.target_index]
        get_physx_simulation_interface().flush_changes()
        self._broken.fill(False); self._other_broken = False
        self.contact_diagnostics = []; self.contact_diagnostic_step = 0
        self.sim.forward(); self.scene.update(self.physics_dt)

    def reset(self, **kwargs):
        self.reset_slot()


class DatasetScene:
    def __init__(self,cfg,num_envs):
        self.sim = SimulationContext(cfg.sim)
        self.scene = InteractiveScene(InteractiveSceneCfg(num_envs=1,env_spacing=20.,replicate_physics=False))
        cfg.robot.prim_path = '/World/envs/env_0/Robot'
        first = Slot(cfg,self.sim,self.scene)
        GreenhouseHarvestEnv._setup_scene(first)
        self.slots = [first]
        self.step_timings = dict(steps=0,write_s=0.,physics_s=0.,read_s=0.)
        self.routing_faults = []
        # Explicit collision groups isolate co-located clones. Keeping plant
        # coordinates identical avoids float32 errors at 20..140 m offsets in
        # these millimetre-scale, stiff articulated stems. Only env_0 is drawn.
        self.origins = np.zeros((num_envs,3),np.float32)
        paths = [f'/World/envs/env_{i}' for i in range(num_envs)]
        if num_envs > 1:
            self.scene.stage.RemovePrim('/World/collisions')
            Cloner.clone(self.scene.cloner,source_prim_path=paths[0],prim_paths=paths,
                                    positions=self.origins,replicate_physics=False,copy_from_source=True)
            self.scene.env_prim_paths = paths
            self.scene.cfg.num_envs = num_envs
            self.scene._default_env_origins = torch.tensor(self.origins)
            self.scene._ALL_INDICES = torch.arange(num_envs)
            self.scene.filter_collisions(global_prim_paths=[])
        for index in range(1,num_envs):
            UsdGeom.Imageable(self.scene.stage.GetPrimAtPath(paths[index])).MakeInvisible()
            cfg_i = copy.deepcopy(cfg)
            cfg_i.robot.prim_path = paths[index]+'/Robot'; cfg_i.robot.spawn = None
            cfg_i.robot.init_state.pos = tuple(np.asarray(cfg.robot.init_state.pos)+self.origins[index])
            slot = Slot(cfg_i,self.sim,self.scene,index)
            slot.dataset_origin = self.origins[index].copy()
            replace = lambda path: path.replace(paths[0],paths[index])
            slot.robot = Articulation(cfg_i.robot)
            self.scene.articulations[f'robot_{index}'] = slot.robot
            slot.ready_joints = first.ready_joints.copy()
            slot.fruit_specs = copy.deepcopy(first.fruit_specs); slot.fruits=[]; slot.anchors=[]
            for spec in slot.fruit_specs:
                spec['path'] = replace(spec['path']); spec['anchor'] = replace(spec['anchor'])
                spec['pose'][:3] = (np.asarray(spec['pose'][:3])+self.origins[index]).tolist()
                fruit = RigidObject(RigidObjectCfg(prim_path=spec['path'],spawn=None))
                slot.fruits.append(fruit); self.scene.rigid_objects[f"{index}_{spec['name']}"] = fruit
            slot.target_index = first.target_index; slot.target_spec = slot.fruit_specs[slot.target_index]
            slot.fruit = slot.fruits[slot.target_index]
            slot.provenance = copy.deepcopy(first.provenance)
            slot.provenance['fruit_centers_world'] = (np.asarray(first.provenance['fruit_centers_world'])+self.origins[index]).tolist()
            elastic = copy.copy(first.elastic); elastic.env=slot; elastic.root=replace(elastic.root)
            elastic.paths = [replace(p) for p in elastic.paths]
            elastic.rest = [np.r_[np.asarray(p[:3])+self.origins[index],p[3:]] for p in elastic.rest]
            elastic.body_ids = None; elastic.model=copy.deepcopy(elastic.model)
            elastic.joints = [replace(p) for p in elastic.joints]
            elastic.visual_rest = first.elastic.visual_rest.copy(); elastic.visual_rest[:,:3] += self.origins[index]
            elastic.articulation = Articulation(first.elastic.articulation.cfg.replace(prim_path=elastic.root,spawn=None))
            self.scene.articulations[f'elastic_{index}'] = elastic.articulation
            slot.elastic = elastic
            # Native world-fixed joints author world-space body0 anchors. Cloner
            # remaps body references but these numeric anchors need translation.
            for prim in Usd.PrimRange(self.scene.stage.GetPrimAtPath(paths[index])):
                if prim.IsA(UsdPhysics.Joint):
                    joint=UsdPhysics.Joint(prim)
                    for rel in (joint.GetBody0Rel(),joint.GetBody1Rel()):
                        if any(str(p).startswith(paths[0]+'/') for p in rel.GetTargets()):
                            raise RuntimeError('Cloned joint still references env_0: '+str(prim.GetPath()))
                    if not joint.GetBody0Rel().GetTargets():
                        p = np.array(joint.GetLocalPos0Attr().Get())+self.origins[index]
                        joint.GetLocalPos0Attr().Set(Gf.Vec3f(*p.tolist()))
            self.slots.append(slot)
        self._contact_sub = get_physx_simulation_interface().subscribe_contact_report_events(self.on_contacts)
        self._event_sub = get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(self.on_event)
        self.sim.reset(); self.scene.update(cfg.sim.dt)
        for slot in self.slots:
            slot.initialize()
        self.scene.write_data_to_sim()

    @staticmethod
    def index(path):
        match = re.search(r'/env_(\d+)/',path)
        return int(match[1]) if match else None

    def on_contacts(self,headers,data):
        for header in headers:
            if not header.num_contact_data: continue
            a,b = [str(PhysicsSchemaTools.intToSdfPath(p)) for p in (header.collider0,header.collider1)]
            ia,ib = self.index(a),self.index(b)
            if ia is not None and ib is not None and ia != ib:
                self.routing_faults.append(dict(a=a,b=b)); continue
            index = ia if ia is not None else ib
            if index is None or index >= len(self.slots) or '/Robot/' not in a+b: continue
            if '/Robot/' in a and '/Robot/' in b: continue
            slot=self.slots[index]
            points=data[header.contact_data_offset:header.contact_data_offset+header.num_contact_data]
            force=float(sum(np.linalg.norm(tuple(p.impulse)) for p in points)/slot.physics_dt)
            if force > .01:
                slot.contact_diagnostics.append(dict(event='contact',a=a,b=b,force=force,
                    step=slot.contact_diagnostic_step,positions=[list(p.position) for p in points]))

    def on_event(self,event):
        if event.type != int(SimulationEvent.JOINT_BREAK): return
        path=str(PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath']))
        index=self.index(path)
        if index is None or index >= len(self.slots): return
        slot=self.slots[index]
        if path not in slot.cluster_joint_paths: return
        slot.contact_diagnostics.append(dict(event='break',path=path,step=slot.contact_diagnostic_step))
        if path == slot.joint_paths[0]: slot._broken[0]=True
        else: slot._other_broken=True

    def step(self):
        before=time.monotonic()
        if getattr(self.slots[0].cfg,'dataset_physics_sync','legacy')=='legacy':
            self.scene.write_data_to_sim()
        else:
            # Implicit spring/position targets persist in PhysX. Efforts still
            # upload each substep so the gravity preload has identical lifetime.
            # This path is only valid for the scene's implicit actuators and no
            # body wrenches; fail closed if the model changes.
            from isaaclab.actuators import ImplicitActuator
            for slot in self.slots:
                for asset in (slot.robot,slot.elastic.articulation):
                    if not all(isinstance(a,ImplicitActuator) for a in asset.actuators.values()):
                        raise RuntimeError('Optimized sync requires implicit actuators')
                    if asset._instantaneous_wrench_composer.active or asset._permanent_wrench_composer.active:
                        raise RuntimeError('Optimized sync does not support external body wrenches')
                    if slot.command_dirty:
                        asset.write_data_to_sim()
                    else:
                        asset.root_physx_view.set_dof_actuation_forces(asset._joint_effort_target_sim,asset._ALL_INDICES)
                for fruit in slot.fruits:
                    if fruit._instantaneous_wrench_composer.active or fruit._permanent_wrench_composer.active:
                        raise RuntimeError('Optimized sync does not support fruit wrenches')
                slot.command_dirty=False
        written=time.monotonic()
        self.sim.step(render=False)
        simulated=time.monotonic()
        self.scene.update(self.sim.get_physics_dt())
        after=time.monotonic()
        self.step_timings['steps']+=1
        self.step_timings['write_s']+=written-before
        self.step_timings['physics_s']+=simulated-written
        self.step_timings['read_s']+=after-simulated
        if self.routing_faults:
            raise RuntimeError('Contact crossed environment boundaries: '+str(self.routing_faults[-1]))

    def reset_all(self):
        for slot in self.slots: slot.reset_slot()
        self.scene.write_data_to_sim()

    def close(self):
        self._contact_sub=None; self._event_sub=None
        self.sim.clear_all_callbacks()
        self.sim.clear_instance()
