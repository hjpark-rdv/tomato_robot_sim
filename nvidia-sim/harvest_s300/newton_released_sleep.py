"""Opt-in native sleeping for free fruit trees, with ordinary contact wake-up.

No fruit, collider, gravity or contact is removed. Attached fruit is connected
to an always-awake plant by its original equality; native island wake rules
remain responsible for activation. Other robot/plant trees cannot sleep.
"""
from __future__ import annotations

import mujoco as mj
import numpy as np
import warp as wp


def configure_fruit_sleep(model, metadata):
    if model.opt.disableflags & int(mj.mjtDisableBit.mjDSBL_ISLAND):
        raise ValueError('Native sleeping requires islands')
    fruit_ids = [model.body(item['name']).id for item in metadata['attachments']]
    trees = sorted({int(model.body_treeid[b]) for b in fruit_ids})
    if any(tree < 0 for tree in trees):
        raise ValueError('Every fruit requires a dynamic free tree')
    for b, tree in zip(fruit_ids, [int(model.body_treeid[b]) for b in fruit_ids]):
        members = np.flatnonzero(model.body_treeid == tree)
        if len(members) != 1 or members[0] != b:
            raise ValueError('Fruit sleep must not put a branch or robot to sleep')
    before = model.tree_sleep_policy.copy()
    model.tree_sleep_policy[:] = int(mj.mjtSleepPolicy.mjSLEEP_AUTO_NEVER)
    model.tree_sleep_policy[trees] = int(mj.mjtSleepPolicy.mjSLEEP_AUTO_ALLOWED)
    model.opt.enableflags |= int(mj.mjtEnableBit.mjENBL_SLEEP)
    return dict(native_sleep_enabled=True, fruit_body_ids=fruit_ids, fruit_tree_ids=trees,
                policy_before=before.tolist(), policy_after=model.tree_sleep_policy.tolist(),
                sleep_velocity_tolerance=float(model.opt.sleep_tolerance),
                colliders_removed=0, source_mass_and_contacts_changed=False,
                wake_policy='Native collision/equality/force island wake-up; all non-fruit trees always awake')


@wp.kernel
def record_sleep(tick: wp.array[int], body_ids: wp.array[int],
                 eqids: wp.array[int], eq: wp.array2d[bool],
                 awake: wp.array2d[int], log: wp.array3d[int],
                 asleep_attached: wp.array[int]):
    w, i = wp.tid()
    state = awake[w, body_ids[i]]
    log[w, tick[0], i] = state
    if state == int(0) and eq[w, eqids[i]]:
        wp.atomic_add(asleep_attached, w, 1)


class FruitSleepAudit:
    def __init__(self, controller):
        self.c = controller
        self.log = wp.zeros((controller.worlds, controller.steps, len(controller.bodies_np)),
                            int, device=controller.device)
        self.asleep_attached = wp.zeros(controller.worlds, int, device=controller.device)

    def audit(self):
        c = self.c
        d = c.solver.mjw_data
        wp.launch(record_sleep, (c.worlds, len(c.bodies_np)),
                  inputs=[c.tick, c.bodies, c.eqids, d.eq_active, d.body_awake,
                          self.log, self.asleep_attached], device=c.device)

    def save(self, output, actual):
        states = self.log.numpy()[:, :actual]
        np.savez_compressed(output/'fruit_sleep_metrics.npz', body_awake=states,
                            attached_asleep_pair_steps=self.asleep_attached.numpy())
        return dict(body_awake_enum={name:int(value) for name,value in mj.mjtSleepState.__members__.items()},
                    asleep_fruit_pair_steps=int(np.sum(states == int(mj.mjtSleepState.mjS_ASLEEP))),
                    attached_asleep_pair_steps=self.asleep_attached.numpy().tolist())
