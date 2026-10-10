"""Session-only controls for identifying native GPU synchronization costs.

These are explicitly named diagnostics. They do not authorize removing moving
conveyor behavior from a harvest motion or claiming engine equivalence.
"""
from __future__ import annotations

import hashlib

import numpy as np
from pxr import Usd


SURFACE_API = 'PhysxSurfaceVelocityAPI'
VELOCITY_ATTRIBUTES = ('physxSurfaceVelocity:surfaceVelocity',
                       'physxSurfaceVelocity:surfaceAngularVelocity')


def disable_zero_surface_velocity_for_diagnostic(stage, *, mode):
    """Remove only a zero conveyor API during idle/pulse, preserving assets.

    Numeric attributes, collision shapes, materials and joints are untouched.
    All authored velocity samples must be finite and exactly zero. The caller
    must never write nonzero belt commands during this diagnostic.
    """
    if mode not in {'idle', 'pulse'}:
        raise ValueError('Zero-conveyor diagnostic is restricted to idle/pulse modes')
    selected = []
    for prim in stage.Traverse():
        schema_op = prim.GetMetadata('apiSchemas')
        if not schema_op or SURFACE_API not in schema_op.GetAppliedItems():
            continue
        velocities = {}
        for name in VELOCITY_ATTRIBUTES:
            attr = prim.GetAttribute(name)
            values = [attr.Get()] if attr and attr.HasValue() else []
            if attr:
                values += [attr.Get(t) for t in attr.GetTimeSamples()]
            for value in values:
                array = np.asarray(value, dtype=float)
                if not np.isfinite(array).all() or np.any(array != 0.):
                    raise ValueError(f'Nonzero/nonfinite surface velocity at {prim.GetPath()}: {name}')
            velocities[name] = [np.asarray(value, dtype=float).tolist() for value in values]
        selected.append((prim, velocities))
    root_before = stage.GetRootLayer().ExportToString()
    session = stage.GetSessionLayer()
    with Usd.EditContext(stage, session):
        for prim, _ in selected:
            if not prim.RemoveAppliedSchema(SURFACE_API):
                raise RuntimeError(f'Cannot remove zero surface-velocity API: {prim.GetPath()}')
    if root_before != stage.GetRootLayer().ExportToString():
        raise RuntimeError('Zero-conveyor diagnostic modified the source layer')
    return dict(mode=mode, diagnostic='zero conveyor API removed',
                removed_api=SURFACE_API,
                bodies=[dict(path=str(prim.GetPath()), velocity_values=values) for prim, values in selected],
                zero_commands_required=True, source_assets_saved=False,
                source_layer_sha256=hashlib.sha256(root_before.encode()).hexdigest(),
                session_layer=session.identifier, collider_changes=0, physical_attribute_changes=0,
                accepted_engine_equivalence=False, training_eligible=False,
                reason='Separate native contact-modification overhead from solver cost during zero belt motion')
