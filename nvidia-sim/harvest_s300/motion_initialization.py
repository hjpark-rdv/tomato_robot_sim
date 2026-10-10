"""Opt-in gravity-neutral native warmup, with exact USD opinion restoration.

Use only around native initialization and calculation/application of a fixed
authored-pose preload. Restore before the first trial step or initial-state
capture. This neither resets physics nor continuously compensates gravity.
Native A/B validation is required before treating this as an accepted mode.
"""
from __future__ import annotations

import math

from pxr import Sdf, Usd, UsdPhysics


GRAVITY_ATTRIBUTES = ('physics:gravityMagnitude', 'physics:gravityDirection')


def _json_value(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else repr(value)
    if value is None:
        return None
    # Gf.Vec types expose the Python sequence protocol without __iter__ on
    # some native OpenUSD builds. iter(value) handles both implementations.
    try:
        return [float(v) for v in iter(value)]
    except TypeError:
        return value


class GravityNeutralWarmup:
    """Temporarily author zero scene gravity in the anonymous session layer.

    Example::

        warmup = GravityNeutralWarmup(stage, '/World/PhysicsScene')
        with warmup:
            sim.initialize_physics()
            # Create native views and calculate/apply fixed rest-pose preload.
        # Exact original gravity opinions are restored here, before trial step.

    Existing default/time-sample/metadata opinions on the two gravity attributes
    are preserved. Unrelated session edits made inside the context remain.
    ``restore`` is idempotent and is also called when the body raises an error.
    """
    def __init__(self, stage, scene_path='/World/PhysicsScene'):
        self.stage = stage
        self.scene_path = Sdf.Path(scene_path)
        self._scene = UsdPhysics.Scene(stage.GetPrimAtPath(self.scene_path))
        if not self._scene:
            raise ValueError(f'PhysicsScene is missing: {scene_path}')
        self._session = stage.GetSessionLayer()
        self._backup = Sdf.Layer.CreateAnonymous('gravity_warmup_original_opinions.usda')
        self._saved = {}
        self._new_prim_paths = []
        self._entered = False
        self._restored = False
        self.metadata = dict(mode='gravity_neutral_initialization_diagnostic',
                             scene_path=str(self.scene_path),
                             scope='native initialization and fixed preload calculation only',
                             gravity_restored_before_trial=False,
                             resets_joint_states=False,
                             changes_trial_gravity=False,
                             native_validation='pending', training_eligible=False)

    def __enter__(self):
        if self._entered:
            raise RuntimeError('Gravity-neutral warmup cannot be re-entered')
        self._entered = True
        prim = self._scene.GetPrim()
        self.metadata['source_resolved_gravity'] = {
            name: _json_value(prim.GetAttribute(name).Get()) for name in GRAVITY_ATTRIBUTES}
        # Only prim specs introduced by this helper can be removed afterwards.
        for path in self.scene_path.GetPrefixes():
            if path.IsPrimPath() and self._session.GetPrimAtPath(path) is None:
                self._new_prim_paths.append(path)
        Sdf.CreatePrimInLayer(self._backup, self.scene_path)
        for name in GRAVITY_ATTRIBUTES:
            path = self.scene_path.AppendProperty(name)
            self._saved[name] = self._session.GetPropertyAtPath(path) is not None
            if self._saved[name]:
                if not Sdf.CopySpec(self._session, path, self._backup, path):
                    raise RuntimeError(f'Could not preserve gravity opinion: {path}')
        with Usd.EditContext(self.stage, self._session):
            self._scene.CreateGravityMagnitudeAttr().Set(0.0)
        if self._scene.GetGravityMagnitudeAttr().Get() != 0.0:
            self.restore()
            raise RuntimeError('Zero warmup gravity did not resolve in the session layer')
        return self

    def restore(self):
        if not self._entered or self._restored:
            return
        prim = self._scene.GetPrim()
        with Usd.EditContext(self.stage, self._session):
            for name in GRAVITY_ATTRIBUTES:
                path = self.scene_path.AppendProperty(name)
                prim.RemoveProperty(name)
                if self._saved.get(name):
                    if not Sdf.CopySpec(self._backup, path, self._session, path):
                        raise RuntimeError(f'Could not restore gravity opinion: {path}')
        # Remove empty over specs left by temporary properties, without touching
        # native/session additions made during initialization.
        for path in reversed(self._new_prim_paths):
            spec = self._session.GetPrimAtPath(path)
            if spec is not None and not spec.properties and not spec.nameChildren and \
                    set(spec.ListInfoKeys()) <= {'specifier'}:
                edit = Sdf.BatchNamespaceEdit()
                edit.Add(path, Sdf.Path.emptyPath)
                if not self._session.Apply(edit):
                    raise RuntimeError(f'Could not remove temporary gravity over: {path}')
        for name in GRAVITY_ATTRIBUTES:
            path = self.scene_path.AppendProperty(name)
            present = self._session.GetPropertyAtPath(path) is not None
            if present != self._saved[name]:
                raise RuntimeError(f'Gravity opinion restoration mismatch: {path}')
            if present:
                verify = Sdf.Layer.CreateAnonymous('gravity_restore_verify.usda')
                Sdf.CreatePrimInLayer(verify, self.scene_path)
                Sdf.CopySpec(self._session, path, verify, path)
                # Comparing only this property excludes unrelated initialization
                # changes on the PhysicsScene or surrounding session prims.
                expected = Sdf.Layer.CreateAnonymous('gravity_restore_expected.usda')
                Sdf.CreatePrimInLayer(expected, self.scene_path)
                Sdf.CopySpec(self._backup, path, expected, path)
                if verify.ExportToString() != expected.ExportToString():
                    raise RuntimeError(f'Gravity opinion contents changed: {path}')
        resolved = {name: _json_value(prim.GetAttribute(name).Get()) for name in GRAVITY_ATTRIBUTES}
        if resolved != self.metadata['source_resolved_gravity']:
            raise RuntimeError('Resolved gravity changed during warmup restoration')
        self._restored = True
        self.metadata.update(gravity_restored_before_trial=True, restored_resolved_gravity=resolved)

    def __exit__(self, exc_type, exc, traceback):
        self.restore()
        return False
