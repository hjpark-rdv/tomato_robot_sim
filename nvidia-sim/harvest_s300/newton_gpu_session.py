"""Retain one native GPU model across identical S300 trials.

This owns no execution loop, controller, viewer or output files. The existing
runner acquires/reset the engine, installs a fresh controller, and performs its
ordinary capture and execution. A cache refuses different source/configuration
or initial-state identities instead of silently reusing incompatible assets.
"""
from __future__ import annotations

from collections.abc import Mapping
import dataclasses
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import mujoco_warp as mw
import numpy as np
import warp as wp


def _signature_digest(signature):
    if not isinstance(signature, Mapping) or not signature:
        raise ValueError("Session signature must be a nonempty mapping of source hashes and configuration")
    value = json.dumps(signature, sort_keys=True, separators=(",", ":"), allow_nan=False,
                       default=lambda x: str(x) if isinstance(x, Path) else _unsupported(x))
    return hashlib.sha256(value.encode()).hexdigest()


def _unsupported(value):
    raise TypeError(f"Unsupported session signature value: {type(value).__name__}")


def _initial_digest(initial):
    result = hashlib.sha256()
    for name, value in sorted(initial.items()):
        array = np.asarray(value)
        if array.dtype.hasobject:
            raise ValueError(f"Initial field {name} cannot contain Python objects")
        result.update(f"{name}:{array.dtype.str}:{array.shape}".encode())
        result.update(np.ascontiguousarray(array).tobytes())
    return result.hexdigest()


def _arrays(root, prefix=""):
    """Walk only known array containers, never models/devices/kernel objects."""
    for name, value in vars(root).items():
        path = f"{prefix}.{name}" if prefix else name
        if isinstance(value, wp.array):
            yield path, value
        elif dataclasses.is_dataclass(value) or isinstance(value, SimpleNamespace) or \
                type(value).__name__ == "AttributeNamespace":
            yield from _arrays(value, path)


class _InitialBuffers:
    def __init__(self, root):
        self.entries = [(name, array, wp.clone(array)) for name, array in _arrays(root)]
        self.bytes = sum(array.size * wp.types.type_size_in_bytes(array.dtype)
                         for _, array, _ in self.entries)

    def restore(self):
        for _, target, original in self.entries:
            wp.copy(target, original)


class NewtonGPUContinuationCache:
    """Reuse upload buffers after independently verified measured continuation.

    The existing reset cache remains strict and unchanged. Here reset is only
    scratch-buffer initialization: the caller restores verified q/v/clock,
    equality activity, damping and warmstart immediately before execution.
    Keep at most one engine; a changed active-window MJB starts a fresh cache.
    """
    allow_verified_continuation = True

    def __init__(self):
        self.cache=None;self.identity=None;self.initial=None;self.kinematics_graph=None
        self.deterministic_adapter=None
        self.metadata_digests={}

    def metadata_digest(self,file_digest,metadata,*,released_gutter_diagnostic):
        # The caller verified immutable input bytes and only overlays this
        # one runtime flag. Do not reserialize33MB of source geometry per hop.
        key=(file_digest,bool(released_gutter_diagnostic))
        if key not in self.metadata_digests:
            self.metadata_digests[key]=_signature_digest(metadata)
        return self.metadata_digests[key]

    def acquire(self,signature,factory,initial):
        if not signature.get('physical_model_sha256'):
            raise ValueError('Continuation cache requires exact physical model SHA')
        identity=_signature_digest(signature)
        if identity!=self.identity:
            self.close()
            # The executor restores the measured state and runs kinematics
            # afterwards. Computing it here on the scratch initial state is
            # redundant (and switches Warp's deterministic module variant).
            self.cache=NewtonGPUSessionCache(initialize_kinematics=False);self.identity=identity
            self.kinematics_graph=None
            self.initial={k:np.asarray(v).copy() for k,v in initial.items()}
        for name in ('plant_preload','runtime_dof_length','actuator_kp','robot_qpos_addresses','actuator_ids'):
            if name in initial and not np.array_equal(initial[name],self.initial[name]):
                raise ValueError('Continuation cache fixed adapter field changed: '+name)
        engine,evidence=self.cache.acquire(signature,factory,self.initial)
        from newton_frozen_constraints import restore_frozen_connect_launch_list
        # configure_frozen_connects still validates the entire source model,
        # metadata and exact GPU launch arrays. Identical static metadata does
        # not need a restore followed immediately by the same filtering.
        evidence['frozen_connect_launch_list_restored']=(False if signature.get('frozen_connect_metadata_sha256')
            else restore_frozen_connect_launch_list(engine.solver))
        evidence.update(verified_continuation=True,measured_state_restored_by_executor=True,
            persistent_solver_caches=False,scope='Model/GPU allocation reuse; scratch caches reset before exact measured state restoration')
        return engine,evidence

    def bounded_deterministic(self,solver,**options):
        from newton_gpu_deterministic import BoundedDeterministicExecution,validate_features
        validate_features(solver,allow_cg=options.get('allow_cg',False))
        if self.deterministic_adapter is None:
            self.deterministic_adapter=BoundedDeterministicExecution(solver,**options)
        elif self.deterministic_adapter.solver is not solver:
            raise ValueError('Deterministic adapter belongs to a different solver')
        return self.deterministic_adapter

    def close(self):
        if self.deterministic_adapter is not None:
            self.deterministic_adapter.close();self.deterministic_adapter=None

    def kinematics(self,engine):
        """Same native kernels, captured once, with no integrated time."""
        if self.cache is None or engine is not self.cache.engine:
            raise ValueError('Kinematics graph belongs to a different continuation model')
        device=engine.model.device
        if not device.is_cuda:
            mw.kinematics(engine.solver.mjw_model,engine.solver.mjw_data)
            return
        if self.kinematics_graph is None:
            with wp.ScopedCapture(device=device) as capture:
                mw.kinematics(engine.solver.mjw_model,engine.solver.mjw_data)
            self.kinematics_graph=capture.graph
        wp.capture_launch(self.kinematics_graph)


class NewtonGPUSessionCache:
    """Single-model in-process cache. Call ``acquire`` before every trial.

    ``signature`` must include source MJB/XML hashes plus all model/solver,
    device/world-count and optional tuning/compatibility settings. It must
    exclude presentation/output settings, allowing GUI/headless comparisons.
    The factory is invoked once and must return an engine before stepping,
    capture or trial-specific callback installation.
    """
    def __init__(self, *, initialize_kinematics=True):
        self.initialize_kinematics=initialize_kinematics
        self.engine = None
        self._signature = None
        self._initial = None
        self.acquisitions = 0

    def acquire(self, signature, factory, initial):
        started = time.perf_counter()
        identity = _signature_digest(signature)
        initial_identity = _initial_digest(initial)
        reused = self.engine is not None
        if reused and (identity != self._signature or initial_identity != self._initial):
            raise ValueError("GPU session source/configuration or initial state mismatch")
        load_s = 0.0
        if not reused:
            load_started = time.perf_counter()
            engine = factory()
            solver = engine.solver
            if solver.use_mujoco_cpu:
                raise ValueError("NewtonGPUSessionCache requires the native Warp backend")
            if int(getattr(solver, "_step", 0)):
                raise ValueError("Session factory must return an unstepped, uncaptured engine")
            self.engine = engine
            self._signature, self._initial = identity, initial_identity
            self._states = (engine.state_0, engine.state_1)
            # Public reset clears control/warm-start/history state. Restoring
            # the fresh-upload snapshots additionally removes derived-buffer
            # carry-over (e.g. cvel/cdof_dot), including fields not reset by the
            # public API. Model-sized geometry/meshes are never duplicated.
            with wp.ScopedDevice(engine.model.device):
                self._data_buffers = _InitialBuffers(solver.mjw_data)
                self._state_buffers = [_InitialBuffers(state) for state in self._states]
                self._control_buffers = _InitialBuffers(engine.control)
                self._damping = wp.clone(solver.mjw_model.dof_damping)
                self._surface = wp.clone(solver.mjw_model.geom_surfacevel)
            self._callbacks = dict(vars(solver.mjw_model.callback))
            load_s = time.perf_counter() - load_started
        engine, solver = self.engine, self.engine.solver
        model, data = solver.mjw_model, solver.mjw_data
        reset_started = time.perf_counter()
        with wp.ScopedDevice(engine.model.device):
            # Discard references to previous trial controller instances. Stable
            # model compatibility hooks are reinstalled by the existing runner.
            for name, value in self._callbacks.items():
                setattr(model.callback, name, value)
            mw.reset_data(model, data)
            self._data_buffers.restore()
            wp.copy(model.dof_damping, self._damping)
            wp.copy(model.geom_surfacevel, self._surface)
            engine.state_0, engine.state_1 = self._states
            for state in self._state_buffers:
                state.restore()
            self._control_buffers.restore()
            # Reset these explicitly even if the uploaded forward state had
            # populated them. They are never inherited from a previous trial.
            data.time.zero_()
            data.qacc_warmstart.zero_()
            data.qacc.zero_()
            for name in ("qpos", "qvel", "eq_active", "ctrl"):
                target = getattr(data, name)
                value = np.asarray(initial[name])
                if value.shape == target.shape[1:]:
                    value = np.broadcast_to(value, target.shape)
                if value.shape != target.shape:
                    raise ValueError(f"Initial {name} shape {value.shape} != {target.shape}")
                if not np.isfinite(value).all():
                    raise ValueError(f"Initial {name} contains nonfinite values")
                target.assign(np.ascontiguousarray(value))
            if self.initialize_kinematics:mw.kinematics(model, data)
            wp.synchronize_device(engine.model.device)
        solver._step = 0
        self.acquisitions += 1
        bytes_saved = (self._data_buffers.bytes + self._control_buffers.bytes +
                       sum(state.bytes for state in self._state_buffers) +
                       self._damping.size * wp.types.type_size_in_bytes(self._damping.dtype) +
                       self._surface.size * wp.types.type_size_in_bytes(self._surface.dtype))
        return engine, dict(schema="s300-newton-gpu-session-v1", reused=reused,
                            acquisition=self.acquisitions, signature_sha256=identity,
                            initial_state_sha256=initial_identity,
                            model_load_wall_s=load_s,
                            reset_wall_s=time.perf_counter() - reset_started,
                            acquisition_wall_s=time.perf_counter() - started,
                            initial_buffer_snapshot_bytes=int(bytes_saved),
                            reset_api="mujoco_warp.reset_data",
                            initial_derived_buffers_restored=True,
                            qacc_and_warmstart_cleared=True,
                            mutable_damping_and_surface_velocity_restored=True,
                            callbacks_restored_before_new_controller=True,
                            physical_time_advanced=False)
