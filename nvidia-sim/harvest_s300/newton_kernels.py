"""GPU adapters for the existing S300 belt and fruit-attachment mechanics.

These kernels run at actual physics frequency. They preserve the existing
MuJoCo adapter's spring/deadband/implicit damping, finite actuator targets,
native surface velocities and measured force/torque release thresholds.
"""
import numpy as np
import warp as wp
from mujoco_warp._src import math as mm
from mujoco_warp._src.support import contact_force_fn
from mujoco_warp._src.types import OverflowType, vec5, vec6


# Reaching a solver iteration budget is diagnostic. Capacity flags indicate
# incomplete constraints/contacts and therefore invalidate physics evidence.
CAPACITY_OVERFLOW_MASK = wp.constant(int(OverflowType.ALL & ~(
    OverflowType.ITERATIONS | OverflowType.LS_ITERATIONS)))


@wp.kernel
def audit_runtime_health(tick: wp.array[int], simulation_time: wp.array[float],
        qacc: wp.array2d[float], nacon: wp.array[int], ncollision: wp.array[int],
        nefc: wp.array[int], overflow: wp.array[int], naconmax: int, njmax: int,
        nacon_log: wp.array[int], ncollision_log: wp.array[int],
        nefc_log: wp.array2d[int], overflow_log: wp.array2d[int],
        qacc_abs_max: wp.array2d[float], time_log: wp.array2d[float],
        first_qacc_nonfinite: wp.array[int], first_capacity_overflow: wp.array[int]):
    w, dof = wp.tid()
    t = tick[0]
    if dof < qacc.shape[1]:
        acceleration = qacc[w, dof]
        if not wp.isfinite(acceleration):
            wp.atomic_min(first_qacc_nonfinite, w, t)
        else:
            wp.atomic_max(qacc_abs_max, w, t, wp.abs(acceleration))
    if dof == 0:
        time_log[w, t] = simulation_time[w]
        nefc_log[w, t] = nefc[w]
        overflow_log[w, t] = overflow[w]
        if not wp.isfinite(simulation_time[w]):
            wp.atomic_min(first_qacc_nonfinite, w, t)
        if (nacon[0] > naconmax or ncollision[0] > naconmax or nefc[w] > njmax
                or (overflow[w] & CAPACITY_OVERFLOW_MASK) != 0):
            wp.atomic_min(first_capacity_overflow, w, t)
        if w == 0:
            nacon_log[t] = nacon[0]
            ncollision_log[t] = ncollision[0]


@wp.kernel
def set_input(preload: wp.array[float], qfrc: wp.array2d[float],
              commands: wp.array2d[float], tick: wp.array[int],
              actuator: wp.array[int], dofs: wp.array[int], kp: wp.array[float],
              bias: wp.array2d[float], ctrl: wp.array2d[float]):
    w, dof = wp.tid()
    qfrc[w, dof] = preload[dof]
    for a in range(actuator.shape[0]):
        if dof == dofs[a]:
            ctrl[w, actuator[a]] = commands[tick[0], a] + bias[w, dof] / kp[a]


@wp.kernel
def attachment_torques(xquat: wp.array2d[wp.quat], cvel: wp.array2d[wp.spatial_vector],
        eq: wp.array2d[bool], eqids: wp.array[int], bodies: wp.array[int], parents: wp.array[int],
        rot0: wp.array[wp.quat], rot1: wp.array[wp.quat], stiffness: wp.array[wp.vec3],
        damping: wp.array[float], dead: wp.array[float],
        torque: wp.array2d[wp.vec3], implicit: wp.array2d[wp.vec3],
        magnitude: wp.array2d[float], attached: wp.array2d[bool]):
    w, i = wp.tid()
    active = eq[w, eqids[i]]
    attached[w, i] = active
    tau = wp.vec3(0.0)
    impl = wp.vec3(0.0)
    if active:
        q0 = mm.mul_quat(xquat[w, parents[i]], rot0[i])
        q1 = mm.mul_quat(xquat[w, bodies[i]], rot1[i])
        angle = mm.quat_to_vel(mm.mul_quat(mm.quat_inv(q0), q1))
        vel_f = wp.spatial_top(cvel[w, bodies[i]])
        vel_p = wp.spatial_top(cvel[w, parents[i]])
        speed = mm.rot_vec_quat(vel_f - vel_p, mm.quat_inv(q0))
        local = wp.vec3(0.0)
        for axis in range(3):
            displacement = wp.sign(angle[axis]) * wp.max(wp.abs(angle[axis]) - dead[i], 0.0)
            local[axis] = -stiffness[i][axis] * displacement - damping[i] * speed[axis]
        tau = mm.rot_vec_quat(local, q0)
        impl = damping[i] * vel_f
    torque[w, i] = tau
    implicit[w, i] = impl
    magnitude[w, i] = wp.length(tau)


@wp.kernel
def apply_torques(bodies: wp.array[int], parents: wp.array[int],
        torque: wp.array2d[wp.vec3], implicit: wp.array2d[wp.vec3],
        applied: wp.array2d[wp.spatial_vector]):
    # One writer per body, fixed summation order; GUI scheduling cannot change
    # the order of atomic torque additions on shared parent branches.
    w, body = wp.tid()
    tau = wp.vec3(0.0)
    for i in range(bodies.shape[0]):
        if body == bodies[i]:
            tau += torque[w, i] + implicit[w, i]
        if body == parents[i]:
            tau -= torque[w, i]
    applied[w, body] = wp.spatial_vector(wp.vec3(0.0), tau)


@wp.kernel
def belt_velocities(tick: wp.array[int], commands: wp.array2d[float],
        ids: wp.array[int], side: wp.array[int], local_y: wp.array[wp.vec3],
        surface: wp.array2d[vec6]):
    w, i = wp.tid()
    velocity = local_y[i] * commands[tick[0], side[i]]
    surface[w, ids[i]] = vec6(velocity[0], velocity[1], velocity[2], 0.0, 0.0, 0.0)


@wp.kernel
def release_attachments(tick: wp.array[int], dt: float,
        ids: wp.array[int], eqids: wp.array[int], damping_adr: wp.array[int],
        attached: wp.array2d[bool], torque: wp.array2d[float],
        force_limit: wp.array[float], torque_limit: wp.array[float],
        nefc: wp.array[int], types: wp.array2d[int], object_ids: wp.array2d[int],
        force: wp.array2d[float], eq: wp.array2d[bool], damping: wp.array2d[float],
        break_step: wp.array2d[int], force_log: wp.array3d[float], torque_log: wp.array3d[float]):
    w, i = wp.tid()
    norm2 = float(0.0)
    # Source constraints are CONNECTs: at most three rows per fruit equality.
    # All equality rows precede joint/contact rows in MuJoCo's ordering.
    for r in range(wp.min(nefc[w], 3 * eqids.shape[0])):
        if types[w, r] == 0 and object_ids[w, r] == eqids[i]:
            norm2 += force[w, r] * force[w, r]
    norm = wp.sqrt(norm2)
    force_log[w, tick[0], i] = norm
    torque_log[w, tick[0], i] = torque[w, i]
    if attached[w, i] and (norm > force_limit[i] or torque[w, i] > torque_limit[i]):
        eq[w, eqids[i]] = False
        break_step[w, i] = tick[0]
        for j in range(3):
            damping[w, damping_adr[i] + j] = 0.0


@wp.kernel
def audit_contacts(tick: wp.array[int], nacon: wp.array[int], njmax: int, cone: int,
        world: wp.array[int], geoms: wp.array[wp.vec2i], distance: wp.array[float],
        frame: wp.array[wp.mat33], friction: wp.array[vec5], dimensions: wp.array[int],
        addresses: wp.array2d[int], adhesion: wp.array[float], forces: wp.array2d[float],
        geom_body: wp.array[int], body_fruit: wp.array[int],
        allowed_release: wp.array2d[bool], attached: wp.array2d[bool],
        active_max: wp.array2d[float], released_max: wp.array2d[float], force_max: wp.array2d[float],
        first_event: wp.array[wp.uint64]):
    c = wp.tid()
    if c < nacon[0] and addresses[c, 0] >= 0:
        w = world[c]
        g = geoms[c]
        if g[0] >= 0 and g[1] >= 0:
            b0 = geom_body[g[0]]
            b1 = geom_body[g[1]]
            f0 = body_fruit[b0]
            f1 = body_fruit[b1]
            exception = bool(False)
            released = bool(False)
            if f0 >= 0 and not attached[w, f0]:
                released = True
                exception = allowed_release[f0, g[1]]
            if f1 >= 0 and not attached[w, f1]:
                released = True
                exception = exception or allowed_release[f1, g[0]]
            pen = wp.max(0.0, -distance[c])
            if released:
                wp.atomic_max(released_max, w, tick[0], pen)
            if not exception:
                wp.atomic_max(active_max, w, tick[0], pen)
                if pen > 0.001:
                    pair = wp.uint64(wp.min(g[0], g[1])) * wp.uint64(65536) + wp.uint64(wp.max(g[0], g[1]))
                    event = (wp.uint64(tick[0]) << wp.uint64(32)) | pair
                    wp.atomic_min(first_event, w, event)
            wrench = contact_force_fn(cone, frame, friction, dimensions, addresses, adhesion,
                                      forces, njmax, nacon, w, c, False)
            wp.atomic_max(force_max, w, tick[0], wp.length(wp.spatial_top(wrench)))


@wp.kernel
def advance_tick(tick: wp.array[int]):
    tick[0] += 1


@wp.kernel
def finish_step(tick: wp.array[int], niter: wp.array[int], log: wp.array2d[int]):
    t = tick[0]
    for w in range(niter.shape[0]):
        log[w, t] = niter[w]
    tick[0] = t + 1


@wp.kernel
def pack_runtime_status(nonfinite: wp.array[int], qacc_nonfinite: wp.array[int],
        capacity: wp.array[int], first_event: wp.array[wp.uint64],
        packed: wp.array2d[wp.uint64]):
    w = wp.tid()
    packed[w, 0] = wp.uint64(nonfinite[w])
    packed[w, 1] = wp.uint64(qacc_nonfinite[w])
    packed[w, 2] = wp.uint64(capacity[w])
    packed[w, 3] = first_event[w]


@wp.kernel
def save_trace(tick: wp.array[int], stride: int, q: wp.array2d[float], v: wp.array2d[float],
        eq: wp.array2d[bool], xpos: wp.array2d[wp.vec3], xquat: wp.array2d[wp.quat],
        q_log: wp.array2d[float], v_log: wp.array2d[float], eq_log: wp.array2d[bool],
        pose_log: wp.array2d[wp.transform], nonfinite: wp.array[int]):
    w, i = wp.tid()
    t = tick[0]
    if i < q.shape[1] and not wp.isfinite(q[w, i]):
        wp.atomic_min(nonfinite, w, t)
    if i < v.shape[1] and not wp.isfinite(v[w, i]):
        wp.atomic_min(nonfinite, w, t)
    if w == 0 and t % stride == 0:
        row = t // stride
        if i < q.shape[1]:
            q_log[row, i] = q[w, i]
        if i < v.shape[1]:
            v_log[row, i] = v[w, i]
        if i < eq.shape[1]:
            eq_log[row, i] = eq[w, i]
        if i < xpos.shape[1]:
            z = xquat[w, i]
            pose_log[row, i] = wp.transform(xpos[w, i], wp.quat(z[1], z[2], z[3], z[0]))


class HarvestGPUController:
    def __init__(self, solver, inputs, initial, metadata, command, belt, steps, worlds=1):
        import mujoco as mj
        from scipy.spatial.transform import Rotation as R
        m = solver.mj_model
        self.solver = solver
        self.device = solver.model.device
        self.worlds, self.steps = worlds, steps
        def arr(value, dtype=float):
            return wp.array(np.asarray(value), dtype=dtype, device=self.device)
        self.tick = wp.zeros(1, int, device=self.device)
        self.commands = arr(command)
        self.belts = arr(belt)
        self.preload = arr(initial['preload'])
        self.actuators = arr(initial['actuator_ids'], int)
        self.dofs = arr(initial['robot_dof_addresses'], int)
        self.kp = arr(m.actuator_gainprm[initial['actuator_ids'], 0])
        specs = metadata['attachments']
        self.bodies_np = np.array([m.body(s['name']).id for s in specs])
        self.parents_np = np.array([m.body(s['anchor']).id for s in specs])
        self.eq_np = np.array([m.equality(s['name'] + '_attachment').id for s in specs])
        if not np.all(m.eq_type[self.eq_np] == mj.mjtEq.mjEQ_CONNECT) or len(self.eq_np) != m.neq:
            raise ValueError('S300 release adapter requires exactly the existing fruit CONNECT constraints')
        self.bodies, self.parents, self.eqids = arr(self.bodies_np, int), arr(self.parents_np, int), arr(self.eq_np, int)
        self.rot0 = arr(np.array([s['local_rot0'] for s in specs])[:, [3, 0, 1, 2]], wp.quat)
        self.rot1 = arr(np.array([s['local_rot1'] for s in specs])[:, [3, 0, 1, 2]], wp.quat)
        self.stiffness = arr([s['stiffness'] for s in specs], wp.vec3)
        self.damping = arr([s['damping'] for s in specs])
        self.dead = arr(np.deg2rad([s['deadband_deg'] for s in specs]))
        self.force_limit = arr([s['force_n'] for s in specs])
        self.torque_limit = arr([s['torque_nm'] for s in specs])
        self.damping_adr = arr([m.jnt_dofadr[m.body_jntadr[b]] + 3 for b in self.bodies_np], int)
        n = len(specs)
        self.torque = wp.zeros((worlds, n), wp.vec3, device=self.device)
        self.implicit = wp.zeros_like(self.torque)
        self.magnitude = wp.zeros((worlds, n), float, device=self.device)
        self.attached = wp.zeros((worlds, n), bool, device=self.device)
        self.break_step = arr(np.full((worlds, n), -1, np.int32), int)
        self.force_log = wp.zeros((worlds, steps, n), float, device=self.device)
        self.torque_log = wp.zeros_like(self.force_log)
        ids, sides, local = [], [], []
        for i, name in enumerate(metadata['robot']['belt_body_names']):
            g = np.flatnonzero((m.geom_bodyid == m.body(name).id) & ((m.geom_contype != 0) | (m.geom_conaffinity != 0)))
            ids.extend(g); sides.extend([i] * len(g))
            local.extend(R.from_quat(m.geom_quat[g][:, [1, 2, 3, 0]]).inv().apply(np.tile([0, 1, 0], (len(g), 1))))
        self.beltids, self.beltside, self.local_y = arr(ids, int), arr(sides, int), arr(local, wp.vec3)
        self.geom_body = arr(m.geom_bodyid, int)
        fruit_map = np.full(m.nbody, -1, np.int32)
        fruit_map[self.bodies_np] = np.arange(n)
        self.body_fruit = arr(fruit_map, int)
        allowed = np.zeros((n, m.ngeom), bool)
        tools = set(metadata['tools']) | {'Robot_world', 'Robot_S300Carrier'}
        for i, spec in enumerate(specs):
            truss = spec['name'].split('_Fruit_', 1)[0]
            for g in range(m.ngeom):
                name = m.body(int(m.geom_bodyid[g])).name
                gn = m.geom(g).name or ''
                allowed[i, g] = name in tools or name.startswith(truss + '_') or (m.geom_bodyid[g] == 0 and ('ground' in gn.lower() or gn == 'CatchFloor'))
        self.allowed_release = arr(allowed, bool)
        self.active_max = wp.zeros((worlds, steps), float, device=self.device)
        self.released_max = wp.zeros_like(self.active_max)
        self.force_max = wp.zeros_like(self.active_max)
        self.first_event = arr(np.full(worlds, np.iinfo(np.uint64).max, np.uint64), wp.uint64)
        self.nacon_log = wp.zeros(steps, int, device=self.device)
        self.ncollision_log = wp.zeros(steps, int, device=self.device)
        self.nefc_log = wp.zeros((worlds, steps), int, device=self.device)
        self.overflow_log = wp.zeros_like(self.nefc_log)
        self.qacc_abs_max = wp.zeros((worlds, steps), float, device=self.device)
        self.time_log = wp.zeros_like(self.qacc_abs_max)
        self.solver_niter_log = wp.zeros((worlds, steps), int, device=self.device)
        self.first_qacc_nonfinite = arr(np.full(worlds, np.iinfo(np.int32).max, np.int32), int)
        self.first_capacity_overflow = wp.zeros_like(self.first_qacc_nonfinite)
        self.first_capacity_overflow.assign(np.full(worlds, np.iinfo(np.int32).max, np.int32))
        self.health_timestamp_scope = ('post-integration backend d.time; zero-based step log '
                                       'index before controller tick increment')
        self.capacity_overflow_mask = int(CAPACITY_OVERFLOW_MASK)

    def control(self, model, data):
        wp.launch(set_input, (self.worlds, self.solver.mj_model.nv), inputs=[self.preload, data.qfrc_applied,
            self.commands, self.tick, self.actuators, self.dofs, self.kp, data.qfrc_bias, data.ctrl], device=self.device)
        wp.launch(attachment_torques, self.torque.shape, inputs=[data.xquat, data.cvel, data.eq_active,
            self.eqids, self.bodies, self.parents, self.rot0, self.rot1, self.stiffness, self.damping, self.dead,
            self.torque, self.implicit, self.magnitude, self.attached], device=self.device)
        wp.launch(apply_torques, (self.worlds, self.solver.mj_model.nbody), inputs=[self.bodies,
            self.parents, self.torque, self.implicit, data.xfrc_applied], device=self.device)
    def pre_step(self):
        # MuJoCo Warp assembles the contact velocity constraints in
        # fwd_position, before callback.control. Set prescribed surface speeds
        # before that assembly so transitions use the current command tick.
        model = self.solver.mjw_model
        if self.beltids.shape[0]:
            wp.launch(belt_velocities, (model.geom_surfacevel.shape[0], self.beltids.shape[0]), inputs=[
                self.tick, self.belts, self.beltids, self.beltside, self.local_y, model.geom_surfacevel], device=self.device)

    def post_step(self):
        m, d = self.solver.mjw_model, self.solver.mjw_data
        wp.launch(audit_runtime_health, (self.worlds, max(1, self.solver.mj_model.nv)), inputs=[
            self.tick, d.time, d.qacc, d.nacon, d.ncollision, d.nefc, d.overflow,
            d.naconmax, d.njmax, self.nacon_log, self.ncollision_log,
            self.nefc_log, self.overflow_log, self.qacc_abs_max, self.time_log,
            self.first_qacc_nonfinite, self.first_capacity_overflow], device=self.device)
        wp.launch(release_attachments, self.torque.shape, inputs=[self.tick, 0., self.bodies, self.eqids,
            self.damping_adr, self.attached, self.magnitude, self.force_limit, self.torque_limit,
            d.nefc, d.efc.type, d.efc.id, d.efc.force, d.eq_active, m.dof_damping,
            self.break_step, self.force_log, self.torque_log], device=self.device)
        wp.launch(audit_contacts, d.naconmax, inputs=[self.tick, d.nacon, d.njmax, int(self.solver.mj_model.opt.cone),
            d.contact.worldid, d.contact.geom, d.contact.dist, d.contact.frame, d.contact.friction,
            d.contact.dim, d.contact.efc_address, d.contact.adhesion, d.efc.force,
            self.geom_body, self.body_fruit, self.allowed_release, self.attached,
            self.active_max, self.released_max, self.force_max, self.first_event], device=self.device)
        wp.launch(finish_step, 1, inputs=[self.tick, d.solver_niter,
            self.solver_niter_log], device=self.device)
