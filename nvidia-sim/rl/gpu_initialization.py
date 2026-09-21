"""Initialize identical GPU slots without N global scene refreshes per slot.

Only cached geometry/calibration is shared; every robot/plant/fruit still owns
its native state and break joints. GpuDatasetScene performs a bulk reset after
all adapters and batch views exist. Nonidentical origins/configurations use the
original initializer. No simulation is advanced during this operation.
"""
from contextlib import contextmanager
import copy
import numpy as np


def same_calibration(a,b):
    return (a.ready_joints==b.ready_joints
            and a.cfg.lift_start_below==b.cfg.lift_start_below
            and a.cfg.lift_height_reference==b.cfg.lift_height_reference
            and np.array_equal(a.dataset_origin,b.dataset_origin)
            and np.array_equal(a.cfg.robot.init_state.pos,b.cfg.robot.init_state.pos)
            and np.array_equal(a.cfg.robot.init_state.rot,b.cfg.robot.init_state.rot)
            and a.target_spec['name']==b.target_spec['name']
            and len(a.fruit_specs)==len(b.fruit_specs)
            and all(np.array_equal(x['pose'],y['pose']) for x,y in zip(a.fruit_specs,b.fruit_specs)))


@contextmanager
def initialize_identical_slots():
    from dataset_scene import Slot
    original=Slot.initialize
    first=None
    def initialize(slot):
        nonlocal first
        if first is None:
            original(slot);first=slot
            return
        if not same_calibration(first,slot):
            original(slot)
            return
        for asset,reference in ((slot.robot,first.robot),(slot.elastic.articulation,first.elastic.articulation)):
            if asset.joint_names!=reference.joint_names or asset.body_names!=reference.body_names:
                raise RuntimeError('Cannot reuse calibration: clone joint/link ordering differs')
        for name in ('arm_ids','tool_id','lift_id','platform_id'):
            setattr(slot,name,copy.deepcopy(getattr(first,name)))
        for name in ('offset_pos','offset_quat','mount_offset','start_q'):
            setattr(slot,name,getattr(first,name).clone())
        slot.targets=slot.start_q.clone()
        slot.provenance.update(copy.deepcopy(first.provenance))
        slot.elastic.body_ids=first.elastic.body_ids.copy()
        slot.elastic.anchor_ids={spec['anchor']:slot.elastic.articulation.body_names.index(spec['anchor'].rsplit('/',1)[-1]) for spec in slot.fruit_specs}
        slot.elastic.preload=first.elastic.preload.clone()
        slot.elastic.model.update(copy.deepcopy(first.elastic.model))
        slot.pose_search_kin=copy.copy(first.pose_search_kin)
        slot.pose_search_kin.env=slot
        slot.pose_search_kin.world=first.pose_search_kin.world.copy()
        slot.pose_search_kin.bounds=first.pose_search_kin.bounds.copy()
    Slot.initialize=initialize
    try:
        yield
    finally:
        Slot.initialize=original
