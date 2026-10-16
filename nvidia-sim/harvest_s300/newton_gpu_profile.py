"""CUDA event intervals inside the actual recorded physics graphs.

Unlike host timers around uncaptured calls, these include cuBLAS/cuSOLVER work
on the physics stream. Reused events report the last invocation in each graph
batch; nested categories overlap and must not be added together.
"""
from functools import wraps

import warp as wp
from mujoco_warp._src import constraint, collision_driver, forward, smooth, solver


@wp.func_native(snippet='uint64_t t; asm volatile("mov.u64 %0, %%globaltimer;" : "=l"(t)); return t;')
def _global_ns() -> wp.uint64:
    return wp.uint64(0)


@wp.kernel(enable_backward=False)
def _stamp(slot: int, ending: bool, starts: wp.array[wp.uint64],
           last: wp.array[wp.uint64], total: wp.array[wp.uint64], counts: wp.array[int]):
    now = _global_ns()
    if ending:
        delta = now - starts[slot]
        last[slot] = delta
        total[slot] += delta
        counts[slot] += 1
    else:
        starts[slot] = now


class GraphEventProfile:
    def __init__(self, model, device):
        self.model, self.device = model, device
        self.hooks, self.events, self.called = [], {}, set()
        targets = [(forward, n) for n in (
            'fwd_kinematics', 'fwd_position', 'fwd_velocity', 'fwd_actuation',
            'fwd_acceleration', 'discrete')]
        # CUDA conditional bodies prohibit event-record nodes. Measure the
        # complete solve outside its conditional loop, including CUDA libraries.
        targets += [(collision_driver, 'collision'), (constraint, 'make_constraint'),
                    (solver, 'solve'), (smooth, 'factor_solve_i')]
        self.inner_targets = [(solver, n) for n in ('_update_constraint', '_update_gradient',
                                                   '_cholesky_factorize_solve', '_linesearch')]
        self.inner_names = [n for _, n in self.inner_targets]
        self.starts, self.last, self.total = [wp.zeros(len(self.inner_targets), wp.uint64,
            device=device) for _ in range(3)]
        self.counts = wp.zeros(len(self.inner_targets), int, device=device)
        for module, name in targets:
            original = getattr(module, name)
            label = module.__name__.rsplit('.', 1)[-1] + '.' + name
            pair = (wp.Event(device, enable_timing=True), wp.Event(device, enable_timing=True))
            self.events[label] = pair
            setattr(module, name, self.wrap(original, pair, label))
            self.hooks.append((module, name, original))
        # A tiny device timestamp kernel is legal inside CUDA conditional
        # graphs; event-record nodes are not. %globaltimer is GPU nanoseconds,
        # not CPU submission time or SM cycles. Keep the instrumentation opt-in.
        for slot, (module, name) in enumerate(self.inner_targets):
            original = getattr(module, name)
            setattr(module, name, self.wrap_inner(original, slot))
            self.hooks.append((module, name, original))

    def wrap_inner(self, original, slot):
        @wraps(original)
        def measured(model, *args, **kwargs):
            if model is not self.model:
                return original(model, *args, **kwargs)
            arrays = [self.starts, self.last, self.total, self.counts]
            wp.launch(_stamp, 1, inputs=[slot, False, *arrays], device=self.device)
            result = original(model, *args, **kwargs)
            wp.launch(_stamp, 1, inputs=[slot, True, *arrays], device=self.device)
            return result
        return measured

    def wrap(self, original, pair, label):
        @wraps(original)
        def measured(model, *args, **kwargs):
            if model is not self.model:
                return original(model, *args, **kwargs)
            self.called.add(label)
            wp.record_event(pair[0])
            result = original(model, *args, **kwargs)
            wp.record_event(pair[1])
            return result
        return measured

    def read(self):
        wp.synchronize_device(self.device)
        result = {name: wp.get_event_elapsed_time(*pair, synchronize=False)
                  for name, pair in self.events.items() if name in self.called}
        for name, last, total, count in zip(self.inner_names, self.last.numpy(),
                                          self.total.numpy(), self.counts.numpy()):
            result['solver.'+name] = dict(last_ms=float(last)/1e6,
                cumulative_ms=float(total)/1e6, invocations=int(count))
        return result

    def close(self):
        wp.synchronize_device(self.device)
        for module, name, original in reversed(self.hooks):
            setattr(module, name, original)
        self.hooks.clear()
