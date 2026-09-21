"""GPU PhysX with native collider reports and batched articulation effort uploads.

CPU tensor readback is intentional: Isaac Sim 5.1's CUDA contact filters cannot
identify individual colliders of a rigid body. Physics still uses GPU dynamics
and GPU broadphase. Keeping native contact reports preserves the hook labels.
"""
import time
import numpy as np
import torch
from isaaclab.actuators import ImplicitActuator
from pxr import Sdf, UsdPhysics
from isaacsim.core.simulation_manager import SimulationManager
from omni.physx import get_physx_interface, get_physx_simulation_interface
from dataset_scene import DatasetScene
from gpu_batch_views import BatchReadCache, BatchSliceView
from gpu_prim_lookup import fast_explicit_asset_lookup
from gpu_replication import native_environment_replication
from gpu_initialization import initialize_identical_slots


class GpuDatasetScene(DatasetScene):
    @property
    def origins(self):
        return self._origins

    @origins.setter
    def origins(self, value):
        self._origins = value
        if self._diagnostic_grid_spacing:
            columns = int(np.ceil(np.sqrt(len(value))))
            self._origins[:, 0] = np.arange(len(value)) % columns * self._diagnostic_grid_spacing
            self._origins[:, 1] = np.arange(len(value)) // columns * self._diagnostic_grid_spacing

    def __init__(self, cfg, num_envs):
        self._diagnostic_grid_spacing = getattr(cfg, 'gpu_diagnostic_grid_spacing', 0.)
        if cfg.sim.device != 'cpu':
            raise ValueError('Native collider logging requires CPU tensor readback')
        cfg.probe_gpu_dynamics = True
        with fast_explicit_asset_lookup(), native_environment_replication(getattr(cfg,'gpu_native_replication',False)), initialize_identical_slots():
            super().__init__(cfg, num_envs)
        self.batch_groups=[]
        self.read_caches=[]
        for suffix,get_asset in [('Robot',lambda s:s.robot),('ElasticPlant',lambda s:s.elastic.articulation)]:
            by_path={get_asset(s).root_physx_view.prim_paths[0]:get_asset(s) for s in self.slots}
            view=self.sim.physics_sim_view.create_articulation_view(list(by_path))
            if set(view.prim_paths)!=set(by_path):
                raise RuntimeError('Batched articulation paths do not match scene')
            assets=[by_path[path] for path in view.prim_paths]
            for index,asset in enumerate(assets):
                meta=view.get_metatype(index)
                if list(meta.dof_names)!=list(asset.joint_names) or list(meta.link_names)!=list(asset.body_names):
                    raise RuntimeError('Batched view DOF/link order differs from local articulation adapter')
                if not all(isinstance(a,ImplicitActuator) for a in asset.actuators.values()):
                    raise RuntimeError('Batched path supports implicit actuators only')
            self.batch_groups.append((view,assets,torch.arange(len(assets),dtype=torch.int32)))
            cache=BatchReadCache(view)
            self.read_caches.append(cache)
            for index,asset in enumerate(assets):
                proxy=BatchSliceView(asset.root_physx_view,cache,index)
                asset._root_physx_view=proxy
                asset.data._root_physx_view=proxy
        self._efforts=None
        self.backend_description='GPU PhysX / GPU broadphase / CPU native collider readback / batched efforts'
        self.reset_all()

    def step(self):
        before=time.monotonic()
        dirty=False
        for slot in self.slots:
            for asset in (slot.robot,slot.elastic.articulation,*slot.fruits):
                if asset._instantaneous_wrench_composer.active or asset._permanent_wrench_composer.active:
                    raise RuntimeError('Batched step does not support external body wrenches')
            if slot.command_dirty:
                slot.robot.write_data_to_sim()
                slot.elastic.articulation.write_data_to_sim()
                slot.command_dirty=False
                dirty=True
        if dirty or self._efforts is None:
            self._efforts=[torch.cat([a._joint_effort_target_sim for a in assets]) for _,assets,_ in self.batch_groups]
        for (view,_,indices),efforts in zip(self.batch_groups,self._efforts):
            view.set_dof_actuation_forces(efforts,indices)
        written=time.monotonic()
        self.sim.step(render=False)
        simulated=time.monotonic()
        for cache in self.read_caches: cache.invalidate()
        self.scene.update(self.sim.get_physics_dt())
        after=time.monotonic()
        self.step_timings['steps']+=1
        self.step_timings['write_s']+=written-before
        self.step_timings['physics_s']+=simulated-written
        self.step_timings['read_s']+=after-simulated
        if self.routing_faults:
            raise RuntimeError('Contact crossed environment boundaries: '+str(self.routing_faults[-1]))

    def reset_all(self):
        # All candidates share a reset barrier. Flush/update the global scene
        # twice, not four times per environment (quadratic work at scale).
        if getattr(self.slots[0].cfg, 'gpu_legacy_reset', False):
            super().reset_all()
            self._efforts=None
            return
        reuse=getattr(self.slots[0].cfg, 'probe_preserve_joints', False)
        if not reuse:
            # These prims are only standalone break joints, never assets.
            # Isaac Lab otherwise sends every deletion to every asset's regex
            # callback (11*N joints x 13*N assets), exhausting re's cache at N=1000.
            for slot in self.slots:
                for path in slot.cluster_joint_paths:
                    if not path.startswith(slot.root+'/HarvestJoint_') or not slot.stage.GetPrimAtPath(path).IsA(UsdPhysics.Joint):
                        raise RuntimeError('Bulk reset attempted to delete a non-harvest joint')
            SimulationManager.enable_usd_notice_handler(False)
            try:
                with Sdf.ChangeBlock():
                    for slot in self.slots:
                        for path in slot.cluster_joint_paths:
                            slot.stage.RemovePrim(path)
                        slot.cluster_joint_paths=[]
                # PhysX's own USD listener remains enabled; constraints really
                # are removed. Only unrelated asset deletion callbacks pause.
                get_physx_simulation_interface().flush_changes()
            finally:
                SimulationManager.enable_usd_notice_handler(True)
        for cache in self.read_caches: cache.invalidate()
        for slot in self.slots:
            slot.robot.reset();slot.elastic.articulation.reset();slot.elastic.reset()
            for fruit,spec in zip(slot.fruits,slot.fruit_specs):
                fruit.reset()
                fruit.write_root_pose_to_sim(torch.tensor([spec['pose']],dtype=torch.float32,device=slot.device))
                fruit.write_root_velocity_to_sim(torch.zeros(1,6,device=slot.device))
        self.sim.forward();self.scene.update(self.sim.get_physics_dt())
        for slot in self.slots:
            slot.robot.write_joint_state_to_sim(slot.start_q,torch.zeros_like(slot.start_q))
            slot.targets[:]=slot.start_q
            slot.robot.set_joint_position_target(slot.targets)
            slot.robot.write_joint_stiffness_to_sim(slot.robot.data.default_joint_stiffness)
            slot.robot.write_joint_damping_to_sim(slot.robot.data.default_joint_damping)
            slot.command_dirty=True
        self.sim.physics_sim_view.update_articulations_kinematic()
        get_physx_interface().update_transformations(False,True,True)
        if not reuse:
            for slot in self.slots:slot.create_harvest_joints()
            get_physx_simulation_interface().flush_changes()
        for slot in self.slots:
            slot.joint_paths[0]=slot.cluster_joint_paths[slot.target_index]
            slot._broken.fill(False);slot._other_broken=False
            slot.contact_diagnostics=[];slot.contact_diagnostic_step=0
        self.sim.forward();self.scene.update(self.sim.get_physics_dt())
        self.scene.write_data_to_sim()
        self._efforts=None
