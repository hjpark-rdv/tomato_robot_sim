"""Opt-in stage-order compatibility for the existing MuJoCo Warp backend.

MuJoCo Warp 3.15 assembles CONNECT references during ``fwd_position``, before
the normal ``fwd_velocity`` refresh of ``cvel`` and ``cdof_dot``. Native MuJoCo
3.15 evaluates those references after that velocity refresh. The helper below
refreshes only the two derived velocity buffers at an existing callback point
after current kinematics/COM coordinates and before constraint construction.

No contact is filtered, moved, added or removed. No model, integration state,
force, threshold, solver option or installed-library source is changed. This
does not make float32 Warp and native float64 MuJoCo numerically identical.
"""

from __future__ import annotations

import mujoco_warp as mw


class _ConnectVelocityCacheRefresh:
    def __init__(self, previous):
        self.previous = previous

    def __call__(self, model, data):
        if self.previous is not None:
            self.previous(model, data)
        # collision_driver.collision calls this after fwd_kinematics (which
        # updates cdof) and before constraint.make_constraint. The callback is
        # also called when the current narrowphase produced zero contacts.
        mw.com_vel(model, data)


def install_connect_velocity_cache_refresh(model, data):
    """Refresh current CONNECT velocity caches before constraint assembly.

    Call once after native MuJoCo Warp model/data creation and before capture
    or stepping. Existing contact filters retain their behavior and ordering.
    The current installed backend always reaches its contactfilter callback
    when these guards hold; configurations bypassing that stage are rejected
    rather than silently running an ineffective fix.

    Cost with sleeping disabled: one additional ``mw.com_vel`` call per
    forward, comprising ``_comvel_root`` and ``_comvel_branch`` launches. There
    is no extra kinematics pass, collision query, forward solve or integration.
    With sleeping enabled, collision's optional second pass can call it twice.
    """
    if not model.opt.run_collision_detection:
        raise ValueError("CONNECT velocity refresh requires native collision detection")
    if model.opt.disableflags & (mw.DisableBit.CONSTRAINT | mw.DisableBit.CONTACT):
        raise ValueError("CONNECT velocity refresh callback is unreachable with constraints/contact disabled")
    if data.naconmax <= 0:
        raise ValueError("CONNECT velocity refresh requires positive native contact capacity")
    if model.eq_connect_adr.size <= 0:
        raise ValueError("CONNECT velocity refresh requires a model with CONNECT equalities")

    current = model.callback.contactfilter
    already_installed = isinstance(current, _ConnectVelocityCacheRefresh)
    if not already_installed:
        model.callback.contactfilter = _ConnectVelocityCacheRefresh(current)
    hook = model.callback.contactfilter
    return dict(
        enabled=True,
        mode="connect_current_velocity_at_contactfilter",
        backend_version=mw.__version__,
        callback="native MuJoCo Warp collision_driver.collision/contactfilter",
        refresh="mujoco_warp.com_vel",
        stage="after fwd_kinematics/com_pos; before constraint.make_constraint",
        refreshed_derived_arrays=["cvel", "cdof_dot"],
        contacts_modified=False,
        integration_or_model_fields_modified=False,
        previous_contactfilter_preserved=hook.previous is not None,
        additional_com_vel_calls_per_forward_without_sleeping=1,
        additional_kernel_launches_per_forward_without_sleeping=2,
        sleeping_note="Optional second collision pass invokes the refresh again",
    )
