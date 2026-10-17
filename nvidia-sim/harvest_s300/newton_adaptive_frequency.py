"""Contact-driven scheduling for the existing fixed-command GPU executor.

The same continuous plan is pre-sampled on an integer master clock. Actual
tool/plant contacts select the next integration interval; no recorded motion
or future contact labels are used. The controller still logs every physics step.
"""
from __future__ import annotations

import math
import numpy as np
import warp as wp

from newton_inputs import command_timeline


@wp.kernel
def contact_activity(tick: wp.array[int], nacon: wp.array[int],
                     geoms: wp.array[wp.vec2i], distance: wp.array[float],
                     tool: wp.array[bool], plant: wp.array[bool], near: float,
                     flag: wp.array[int], log: wp.array[int]):
    c = wp.tid()
    if c < nacon[0]:
        g = geoms[c]
        if g[0] >= 0 and g[1] >= 0 and distance[c] <= near:
            if (tool[g[0]] and plant[g[1]]) or (tool[g[1]] and plant[g[0]]):
                wp.atomic_max(flag, 0, 1)
                wp.atomic_max(log, tick[0], 1)


class ContactActivityGPU:
    def __init__(self, controller, near_m=.002):
        self.c, self.near = controller, near_m
        m = controller.solver.mj_model
        names = [m.body(int(b)).name or '' for b in m.geom_bodyid]
        self.tool = wp.array([n.startswith(('Robot_', 'Tool_')) for n in names],
                             dtype=bool, device=controller.device)
        self.plant = wp.array([n.startswith(('Truss_', 'Stem_')) for n in names],
                              dtype=bool, device=controller.device)
        self.flag = wp.zeros(1, int, device=controller.device)
        self.log = wp.zeros(controller.steps, int, device=controller.device)

    def audit(self):
        d, c = self.c.solver.mjw_data, self.c
        self.flag.zero_()
        wp.launch(contact_activity, d.naconmax,
                  inputs=[c.tick,d.nacon,d.contact.geom,d.contact.dist,
                          self.tool,self.plant,self.near,self.flag,self.log],device=c.device)


class AdaptiveSchedule:
    def __init__(self, metadata, initial_command, high_hz, low_hz, *, seconds=None,
                 hold_seconds=.1):
        if (int(high_hz) != high_hz or int(low_hz) != low_hz
                or not 0 < low_hz < high_hz):
            raise ValueError('Adaptive rates must be positive integer low < high')
        self.high_hz, self.low_hz = int(high_hz), int(low_hz)
        self.master_hz = math.lcm(self.high_hz,self.low_hz)
        self.high_ticks = self.master_hz//self.high_hz
        self.low_ticks = self.master_hz//self.low_hz
        self.reference, self.description, _ = command_timeline(metadata,initial_command,
            self.master_hz,command_source='plan',seconds=seconds)
        self.target_tick = int(round(self.reference['time_edges'][-1]*self.master_hz))
        self.maximum_steps = int(math.ceil(self.target_tick/self.high_ticks))+2
        self.timeline = {}
        for key,value in self.reference.items():
            if key in ('time_edges','dt','time','source_sample_time_s','source_command_record_time_s',
                       'command','phase','authored_belt_speed','belt_surface_velocity_local_y_m_s'):
                size = self.maximum_steps+1 if key == 'time_edges' else self.maximum_steps
                self.timeline[key] = np.zeros((size,*value.shape[1:]),dtype=value.dtype)
            else:
                self.timeline[key] = value.copy()
        self.timeline['phase'] = np.full(self.maximum_steps,'adaptive_pending',dtype='<U64')
        self.timeline['time_edges'][:] = np.arange(self.maximum_steps+1)/self.high_hz
        self.timeline['dt'][:] = 1/self.high_hz
        self.timeline['dof_names'] = self.reference['dof_names'].copy()
        self.current_tick = 0
        self.hold_ticks = int(round(hold_seconds*self.master_hz))
        self.last_contact_tick = -self.hold_ticks-1
        self.decisions = []
        self.complete = False

    @property
    def graph_timesteps(self):
        return [round(1/self.high_hz,12),round(1/self.low_hz,12)]

    def choose_rate(self, active):
        if active:
            self.last_contact_tick = self.current_tick
        return self.high_hz if self.current_tick-self.last_contact_tick <= self.hold_ticks else self.low_hz

    def prepare(self, actual, maximum_batch, active):
        if self.current_tick >= self.target_tick:
            self.complete = True
            return 0
        rate = self.choose_rate(active)
        ticks = self.master_hz//rate
        # Observe every low-frequency step so a 10-step batch cannot hide
        # an entire short encounter. High-frequency steps use normal graphs.
        count = maximum_batch if rate == self.high_hz else 1
        start = self.current_tick
        first_phase = str(self.reference['phase'][min(start,len(self.reference['phase'])-1)])
        used = 0
        for i in range(count):
            if self.current_tick >= self.target_tick:
                break
            ref = min(self.current_tick,len(self.reference['phase'])-1)
            if i and str(self.reference['phase'][ref]) != first_phase:
                break
            row = actual+i
            for key in ('command','phase','authored_belt_speed','belt_surface_velocity_local_y_m_s'):
                self.timeline[key][row] = self.reference[key][ref]
            for key in ('time','source_sample_time_s','source_command_record_time_s'):
                self.timeline[key][row] = self.current_tick/self.master_hz
            self.timeline['time_edges'][row] = self.current_tick/self.master_hz
            self.current_tick += ticks
            self.timeline['time_edges'][row+1] = self.current_tick/self.master_hz
            self.timeline['dt'][row] = ticks/self.master_hz
            used += 1
        self.decisions.append(dict(first_step=actual,steps=used,start_tick=start,
                                  end_tick=self.current_tick,hz=rate,observed_contact=bool(active)))
        return used

    def upload(self, controller, actual, count):
        def copy_rows(destination, values):
            source = wp.array(np.ascontiguousarray(values),dtype=destination.dtype,device=controller.device)
            width = math.prod(destination.shape[1:]) if destination.ndim > 1 else 1
            wp.copy(destination.flatten(),source.flatten(),dest_offset=actual*width,count=values.size)
        copy_rows(controller.commands,self.timeline['command'][actual:actual+count])
        copy_rows(controller.belts,self.timeline['belt_surface_velocity_local_y_m_s'][actual:actual+count])
        if controller.contact_policy is not None:
            policy = controller.contact_policy
            from newton_contact_policy import OPEN_PHASES
            phase = np.array([1 if x in policy.config['relaxed_phases'] else 2 if x in OPEN_PHASES else 0
                              for x in self.timeline['phase'][actual:actual+count]],np.int32)
            copy_rows(policy.phase,phase)
            copy_rows(policy.start_time,self.timeline['time_edges'][actual:actual+count])

    def finish(self, actual):
        self.complete = self.current_tick >= self.target_tick
        out = {}
        for key,value in self.timeline.items():
            out[key] = value[:actual+1] if key == 'time_edges' else value.copy() if key == 'dof_names' else value[:actual]
        out['master_hz'] = np.asarray(self.master_hz,dtype=np.int64)
        out['time_edges_master_ticks'] = np.rint(out['time_edges']*self.master_hz).astype(np.int64)
        # Match the existing command format exactly. The float64 subtraction
        # is bookkeeping only; captured GPU intervals keep their rounded dt.
        out['dt'] = np.diff(out['time_edges'])
        return out

    def evidence(self):
        return dict(policy='Actual tool/plant contact from preceding physics step plus short hold; one-step low-frequency observation',
            high_hz=self.high_hz,low_hz=self.low_hz,master_hz=self.master_hz,
            nominal_requested_s=self.target_tick/self.master_hz,
            scheduled_end_s=self.current_tick/self.master_hz,complete=self.complete,
            host_feedback=True,command_upload='Only current batch, continuous original plan sampled on exact integer master clock',
            decisions=self.decisions)
