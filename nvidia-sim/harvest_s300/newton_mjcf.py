"""Newton SolverMuJoCo adapter that retains a compiled MJCF physics model.

Newton's ordinary MJCF round trip currently changes or omits some native
fields. This adapter imports Newton's articulated state/control metadata, then
uses a narrowly overridden model-build hook to upload the original MjModel.
The integration loop remains ``newton.solvers.SolverMuJoCo.step``. Its native
data is authoritative (``update_data_interval=0``), suitable for a callback that
applies the existing torque controller at MuJoCo's normal control phase.

Newton metadata omits mesh shapes; the physics model retains every original
mesh, collider, actuator, equality and exclusion. Collision queries and contact
audits must therefore use ``solver.mjw_*`` / ``solver.mj_*`` rather than Newton's
metadata-only shape arrays. This distinction is recorded in ``evidence``.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path

import mujoco
import newton
import numpy as np
import warp as wp
from newton._src.solvers.mujoco.solver_mujoco import _mujoco_warp_deterministic_max_records


@wp.kernel(enable_backward=False)
def _copy_control(source: wp.array(dtype=wp.float32), destination: wp.array2d(dtype=wp.float32)):
    world, actuator = wp.tid()
    destination[world, actuator] = source[world * destination.shape[1] + actuator]


@wp.kernel(enable_backward=False)
def _copy_bodies(body_map: wp.array2d(dtype=wp.int32),
                 xpos: wp.array2d(dtype=wp.vec3), xquat: wp.array2d(dtype=wp.quat),
                 body_q: wp.array(dtype=wp.transform)):
    world, native_body = wp.tid()
    body = body_map[world, native_body]
    if body >= 0:
        # MJWarp stores quaternion array entries in MuJoCo wxyz component order.
        q = xquat[world, native_body]
        body_q[body] = wp.transform(xpos[world, native_body], wp.quat(q[1], q[2], q[3], q[0]))


def _names(model, kind, count):
    return [mujoco.mj_id2name(model, kind, i) for i in range(count)]


def _suffix(value):
    return str(value).rsplit("/", 1)[-1]


def _map_unique(labels, names, description):
    index = {}
    for i, label in enumerate(labels):
        name = _suffix(label)
        if name in index:
            raise ValueError(f"Ambiguous {description} name: {name}")
        index[name] = i
    missing = [name for name in names if name not in index]
    if missing:
        raise ValueError(f"Missing {description} mapping: {missing[:8]}")
    return np.asarray([index[name] for name in names], dtype=np.int32)


def model_array_digest(model):
    """Digest all exposed numeric arrays and numeric option fields, including visuals."""
    result = hashlib.sha256()
    for label, obj in (("model", model), ("option", model.opt)):
        for name in sorted(dir(obj)):
            if name.startswith("_"):
                continue
            value = getattr(obj, name)
            if isinstance(value, np.ndarray):
                result.update(f"{label}.{name}:{value.dtype}:{value.shape}".encode())
                result.update(value.tobytes())
            elif isinstance(value, (int, float, bool, np.integer, np.floating)):
                result.update(f"{label}.{name}:{value!r}".encode())
    return result.hexdigest()


@dataclass
class NewtonMJCF:
    model: object
    solver: object
    state_0: object
    state_1: object
    control: object
    evidence: dict

    def step(self, dt):
        """Explicit inherited Newton solver step, with no rendering or wall-clock dt."""
        self.solver.step(self.state_0, self.state_1, self.control, None, dt)
        self.state_0, self.state_1 = self.state_1, self.state_0


class SourceModelSolverMuJoCo(newton.solvers.SolverMuJoCo):
    """Preserve source MJCF while using Newton's actual SolverMuJoCo.step."""

    def __init__(self, model, source_model, *, nconmax=512, njmax=2048,
                 njmax_nnz=None, use_mujoco_cpu=False, native_observables_only=False,
                 native_control_callback=False,
                 deterministic=wp.DeterministicMode.NOT_GUARANTEED):
        self.native_observables_only = bool(native_observables_only)
        self.native_control_callback = bool(native_control_callback)
        if not isinstance(deterministic, wp.DeterministicMode):
            raise ValueError("deterministic must be a warp.DeterministicMode enum value")
        if self.native_control_callback and use_mujoco_cpu:
            raise ValueError("native_control_callback requires the MuJoCo Warp GPU backend")
        self.source_model = source_model
        self.source_array_digest = model_array_digest(source_model)
        if use_mujoco_cpu and model.world_count > 1:
            raise ValueError("The CPU diagnostic supports one world only")
        super().__init__(model, use_mujoco_cpu=use_mujoco_cpu,
                         use_mujoco_contacts=True, update_data_interval=0,
                         nconmax=nconmax, njmax=njmax, njmax_nnz=njmax_nnz,
                         skip_visual_only_geoms=False, include_sites=True,
                         deterministic=deterministic)
        if model_array_digest(self.mj_model) != self.source_array_digest:
            raise RuntimeError("Source physical model changed during Newton initialization")

    def _convert_to_mjc(self, model, state=None, **options):
        import mujoco_warp

        self.mj_model = self.source_model
        self.mj_data = mujoco.MjData(self.mj_model)
        mujoco.mj_forward(self.mj_model, self.mj_data)
        self._total_loop_joint_dofs = 0
        self._total_loop_joint_coords = 0
        self.has_connect_constraints = False
        self.has_jnt_connect_constraints = False
        world_count = max(1, model.world_count)
        bodies_per_world = model.body_count // world_count
        joints_per_world = model.joint_count // world_count
        body_names = _names(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, self.mj_model.nbody)
        body_ids = _map_unique(model.body_label[:bodies_per_world], body_names[1:], "body")
        template_body = np.concatenate(([-1], body_ids)).astype(np.int32)
        body_mapping = np.stack([np.where(template_body < 0, -1,
                                          template_body + w * bodies_per_world)
                                 for w in range(world_count)])
        self.mjc_body_to_newton = wp.array(body_mapping, dtype=wp.int32, device=model.device)
        inverse_body = {int(newton_body): native_body for native_body, newton_body in enumerate(template_body)
                        if newton_body >= 0}
        q_start = np.full(model.joint_count, -1, dtype=np.int32)
        qd_start = np.full(model.joint_count, -1, dtype=np.int32)
        child = model.joint_child.numpy()
        source_joint_to_newton = np.full((world_count, self.mj_model.njnt), -1, dtype=np.int32)
        source_dof_to_newton = np.full((world_count, self.mj_model.nv), -1, dtype=np.int32)
        q_start_newton = model.joint_q_start.numpy()
        qd_start_newton = model.joint_qd_start.numpy()
        for world in range(world_count):
            for local_joint in range(joints_per_world):
                joint = world * joints_per_world + local_joint
                native_body = inverse_body[int(child[joint] % bodies_per_world)]
                count = int(self.mj_model.body_jntnum[native_body])
                native_joint = int(self.mj_model.body_jntadr[native_body])
                if not count:
                    continue
                q_start[joint] = self.mj_model.jnt_qposadr[native_joint]
                qd_start[joint] = self.mj_model.jnt_dofadr[native_joint]
                q_size = int(q_start_newton[joint + 1] - q_start_newton[joint])
                qd_size = int(qd_start_newton[joint + 1] - qd_start_newton[joint])
                q_end = (int(self.mj_model.jnt_qposadr[native_joint + count])
                         if native_joint + count < self.mj_model.njnt else self.mj_model.nq)
                qd_end = (int(self.mj_model.jnt_dofadr[native_joint + count])
                          if native_joint + count < self.mj_model.njnt else self.mj_model.nv)
                if q_end - q_start[joint] != q_size or qd_end - qd_start[joint] != qd_size:
                    raise ValueError(f"Coordinate topology changed at body {body_names[native_body]}")
                source_joint_to_newton[world, native_joint:native_joint + count] = joint
                source_dof_to_newton[world, qd_start[joint]:qd_end] = np.arange(
                    qd_start_newton[joint], qd_start_newton[joint + 1])
        if np.any(source_joint_to_newton < 0) or np.any(source_dof_to_newton < 0):
            raise ValueError("Incomplete source joint or DOF mapping")
        self.mj_q_start = wp.array(q_start, dtype=wp.int32, device=model.device)
        self.mj_qd_start = wp.array(qd_start, dtype=wp.int32, device=model.device)
        self.mjc_jnt_to_newton_jnt = wp.array(source_joint_to_newton, dtype=wp.int32, device=model.device)
        self.mjc_dof_to_newton_dof = wp.array(source_dof_to_newton, dtype=wp.int32, device=model.device)
        self.mjc_jnt_to_newton_dof = wp.array(np.stack([
            source_dof_to_newton[w, self.mj_model.jnt_dofadr] for w in range(world_count)]),
            dtype=wp.int32, device=model.device)
        eq_labels = getattr(model.mujoco, "equality_constraint_label", [])
        eq_names = _names(self.mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, self.mj_model.neq)
        eq_per_world = model.mujoco.equality_constraint_count // world_count
        eq_map = (_map_unique(eq_labels[:eq_per_world], eq_names, "equality")
                  if eq_names else np.empty(0, dtype=np.int32))
        self.mjc_eq_to_newton_eq = wp.array(np.stack([
            eq_map + world * eq_per_world for world in range(world_count)]), dtype=wp.int32, device=model.device)
        self.mappings = {"body": body_mapping.tolist(),
                         "joint": source_joint_to_newton.tolist(),
                         "dof": source_dof_to_newton.tolist(),
                         "newton_joint_source_qpos_start": q_start.tolist(),
                         "newton_joint_source_dof_start": qd_start.tolist(),
                         "equality": self.mjc_eq_to_newton_eq.numpy().tolist(),
                         "actuator": list(range(self.mj_model.nu)),
                         "geom": list(range(self.mj_model.ngeom)),
                         "native_body_names": body_names, "native_equality_names": eq_names,
                         "native_actuator_names": _names(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, self.mj_model.nu)}
        if self.use_mujoco_cpu:
            # Inherited SolverMuJoCo's native CPU step and coordinate readback
            # need no device physics model or constraint workspace. Optional
            # GPU rigid-body observables are deliberately outside this path.
            self.mjw_model = None
            self.mjw_data = None
            self.nvmax = self.mj_model.nv
            self._cpu_native_body_ids = np.flatnonzero(template_body >= 0)
            self._cpu_newton_body_ids = template_body[self._cpu_native_body_ids]
            self.upload_compatibility = dict(not_applicable_cpu=True,
                unsupported_disableflags_removed=[], source_disableflags=int(self.mj_model.opt.disableflags),
                gpu_disableflags=None, invalid_state_guard_required=False,
                meaning="Native CPU execution retains all source flags and allocates no GPU physics workspace.")
            return
        with wp.ScopedDevice(model.device):
            # These two physical fields change independently after fruit release
            # or belt control. Each world needs its own device storage.
            original_disableflags = int(self.mj_model.opt.disableflags)
            unsupported_autoreset = original_disableflags & int(mujoco.mjtDisableBit.mjDSBL_AUTORESET)
            self.upload_compatibility = {
                "unsupported_disableflags_removed": ["mjDSBL_AUTORESET"] if unsupported_autoreset else [],
                "source_disableflags": original_disableflags,
                "gpu_disableflags": original_disableflags & ~unsupported_autoreset,
                "invalid_state_guard_required": bool(unsupported_autoreset),
                "meaning": "MuJoCo Warp lacks native disable-autoreset; caller must reject invalid physics."}
            try:
                self.mj_model.opt.disableflags = original_disableflags & ~unsupported_autoreset
                self.mjw_model = mujoco_warp.put_model(
                    self.mj_model, batch_sizes={"dof_damping": world_count, "geom_surfacevel": world_count})
            finally:
                self.mj_model.opt.disableflags = original_disableflags
            self.mjw_data = mujoco_warp.put_data(self.mj_model, self.mj_data, nworld=world_count,
                                                nconmax=options.get("nconmax"), njmax=options.get("njmax"),
                                                njmax_nnz=options.get("njmax_nnz"))
        self.nvmax = self.mjw_data.nvmax
        self._expand_option_fields(self.mjw_model, world_count)
        if not self.use_mujoco_cpu:
            self._configure_native_gpu_compilation()

    def _configure_native_gpu_compilation(self):
        # The source-model upload bypasses Newton's ordinary model conversion.
        # Reproduce its model-derived atomic-record bound before compiling any
        # deterministic generated kernels; zero is unsafe for dynamic loops.
        if self._deterministic != wp.DeterministicMode.NOT_GUARANTEED:
            self._deterministic_max_records = _mujoco_warp_deterministic_max_records(
                self.mj_model, self.mjw_data)
        self._set_mujoco_warp_module_options()
        self._prepare_generated_kernels()

    def _apply_mjc_control(self, model, state, control, mj_data):
        if self.native_control_callback:
            # This explicit opt-in asserts that the installed callback writes
            # every native actuator at MuJoCo's normal forward-control phase.
            # Newton control metadata is therefore neither read nor copied.
            self._validate_native_control_callback()
            return
        # Native controller callbacks may overwrite these ten finite commands at
        # the normal forward-control phase; no effort gains are synthesized.
        if control is None:
            return
        source = getattr(getattr(control, "mujoco", None), "ctrl", None)
        if source is None:
            raise RuntimeError("Original CTRL_DIRECT actuator namespace is missing")
        if self.use_mujoco_cpu:
            mj_data.ctrl[:] = source.numpy()
        else:
            wp.launch(_copy_control, dim=mj_data.ctrl.shape,
                      inputs=[source, mj_data.ctrl], device=model.device)

    def _update_mjc_data(self, *args, **kwargs):
        raise RuntimeError("SourceModelSolverMuJoCo requires authoritative native data / update_data_interval=0")

    def _validate_native_control_callback(self):
        if not self.native_control_callback:
            return
        if self.use_mujoco_cpu:
            raise RuntimeError("native_control_callback requires the MuJoCo Warp GPU backend")
        callback = getattr(getattr(self.mjw_model, "callback", None), "control", None)
        if not callable(callback):
            raise RuntimeError("native_control_callback requires an installed control callback that writes every actuator")
        if self.mjw_model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_ACTUATION):
            raise RuntimeError("native_control_callback cannot run while ACTUATION is disabled")

    def validate_observables(self, observables, contacts=None):
        # The inherited public step calls this before any physics. Reject an
        # unsupported metadata request before advancing time, not at readback.
        self._validate_native_control_callback()
        if self.native_observables_only:
            if self.update_data_interval != 0:
                raise RuntimeError("native_observables_only requires authoritative native data / update_data_interval=0")
            if observables is not None:
                raise RuntimeError("native_observables_only cannot populate Newton observables; read mapped native arrays")
        return super().validate_observables(observables, contacts)

    def _update_newton_state(self, model, state, mj_data, state_prev, observables=None):
        if self.native_observables_only:
            if self.update_data_interval != 0:
                raise RuntimeError("native_observables_only requires authoritative native data / update_data_interval=0")
            if observables is not None:
                raise RuntimeError("native_observables_only cannot populate Newton observables; read mapped native arrays")
        else:
            super()._update_newton_state(model, state, mj_data, state_prev, observables)
        if self.use_mujoco_cpu:
            mujoco.mj_kinematics(self.mj_model, self.mj_data)
            if self.native_observables_only:
                return
            poses = np.empty((model.body_count, 7), dtype=np.float32)
            poses[self._cpu_newton_body_ids, :3] = self.mj_data.xpos[self._cpu_native_body_ids]
            poses[self._cpu_newton_body_ids, 3:] = self.mj_data.xquat[self._cpu_native_body_ids][:, [1, 2, 3, 0]]
            state.body_q.assign(poses)
        else:
            import mujoco_warp

            mujoco_warp.kinematics(self.mjw_model, self.mjw_data)
            if self.native_observables_only:
                return
            wp.launch(_copy_bodies, dim=self.mjc_body_to_newton.shape,
                      inputs=[self.mjc_body_to_newton, self.mjw_data.xpos, self.mjw_data.xquat, state.body_q],
                      device=model.device)

    def notify_model_changed(self, flags):
        raise RuntimeError("Native model fields are authoritative; update explicit mapped native arrays instead")


def create_newton_mjcf(scene, *, device="cuda:0", world_count=1, nconmax=512,
                       njmax=2048, njmax_nnz=None, use_mujoco_cpu=False,
                       source_model=None, configure_source=None, native_observables_only=False,
                       native_control_callback=False,
                       deterministic=wp.DeterministicMode.NOT_GUARANTEED):
    """Build Newton metadata and an unmodified source-model SolverMuJoCo.

    ``native_observables_only`` skips coordinate/FK/body-copy readback into the
    Newton state buffers. In that opt-in mode those metadata states are stale;
    callers must read mapped native solver arrays and cannot request Newton
    observables. Native post-step kinematics and inherited solver integration
    remain unchanged.

    ``native_control_callback`` explicitly asserts that a native GPU control
    callback overwrites every actuator on every step. The caller must install
    that callback before stepping. In this mode Newton control metadata is not
    copied into native data; callback presence is checked before integration.
    """
    scene = Path(scene).resolve(strict=True)
    if not isinstance(world_count, int) or world_count <= 0:
        raise ValueError("world_count must be a positive integer")
    source = source_model if source_model is not None else mujoco.MjModel.from_xml_path(str(scene))
    if configure_source is not None:
        configure_source(source)
    template = newton.ModelBuilder()
    # Site-based CONNECT equalities require the source sites to be parsed even
    # though these reference points do not add physical collision geometry.
    template.add_mjcf(str(scene), parse_visuals=False, parse_meshes=False, parse_sites=True,
                      parse_mujoco_options=False, collapse_fixed_joints=False,
                      convert_mjc_equality_constraints=False, convert_3d_hinge_to_ball_joints=False,
                      ctrl_direct=True, enable_self_collisions=True)
    if world_count == 1:
        builder = template
    else:
        builder = newton.ModelBuilder()
        for world in range(world_count):
            builder.add_world(template, label_prefix=f"world_{world}/")
    with wp.ScopedDevice(device):
        model = builder.finalize(device=device)
        solver = SourceModelSolverMuJoCo(model, source, nconmax=nconmax, njmax=njmax,
                                         njmax_nnz=njmax_nnz, use_mujoco_cpu=use_mujoco_cpu,
                                         native_observables_only=native_observables_only,
                                         native_control_callback=native_control_callback,
                                         deterministic=deterministic)
        state_0, state_1 = model.state(), model.state()
        control = model.control()
        solver._update_newton_state(model, state_0,
                                    solver.mj_data if use_mujoco_cpu else solver.mjw_data,
                                    state_prev=state_0)
        state_1.assign(state_0)
    evidence = {"schema": "newton-source-mjcf-v1", "source_xml": str(scene),
                "source_xml_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
                "source_model_array_sha256": solver.source_array_digest,
                "engine": "newton.solvers.SolverMuJoCo.step", "adapter": type(solver).__name__,
                "backend": "native_mujoco_cpu" if use_mujoco_cpu else "mujoco_warp_cuda",
                "world_count": world_count, "native_data_authoritative": True,
                "native_observables_only": bool(native_observables_only),
                "native_control_callback": bool(native_control_callback),
                "native_control_callback_writes_every_actuator": bool(native_control_callback),
                "deterministic_mode": solver._deterministic.name,
                "deterministic_max_records": int(solver._deterministic_max_records),
                "newton_state_metadata_current": not bool(native_observables_only),
                "native_kinematics_refreshed_after_each_step": True,
                "source_fields_preserved": True, "metadata_mesh_shapes_omitted": True,
                "gpu_upload_compatibility": solver.upload_compatibility,
                "physics_meshes_and_colliders_preserved": True,
                "source_counts": {name: int(getattr(source, name)) for name in
                                  ("nbody", "njnt", "nq", "nv", "ngeom", "nu", "neq", "nexclude", "nmesh")},
                "newton_metadata_counts": {name: int(getattr(model, name)) for name in
                                           ("body_count", "joint_count", "joint_coord_count", "joint_dof_count", "shape_count")},
                "mappings": solver.mappings, "training_eligible": False}
    return NewtonMJCF(model, solver, state_0, state_1, control, evidence)
