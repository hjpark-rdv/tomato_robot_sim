"""Diagnostic wall/CPU timing around existing PhysX calls, without extra sync.

CPU time is not GPU kernel time. Process CPU time includes all CPU threads;
thread CPU time measures the calling Python/native thread. fetch_results wall
time can include GPU waiting, CPU work, readback and callbacks.
"""
import time


class NativeStepProfile:
    def __init__(self, context):
        self.context = context
        self.original = context._physx_sim_interface
        self.timings = {}

    def __getattr__(self, name):
        return getattr(self.original, name)

    def _call(self, name, *args, **kwargs):
        wall, process, thread = time.perf_counter(), time.process_time(), time.thread_time()
        try:
            return getattr(self.original, name)(*args, **kwargs)
        finally:
            elapsed = (time.perf_counter()-wall, time.process_time()-process, time.thread_time()-thread)
            row = self.timings.setdefault(name, dict(calls=0, wall_s=0., process_cpu_s=0., caller_cpu_s=0.))
            row['calls'] += 1
            for key, value in zip(('wall_s', 'process_cpu_s', 'caller_cpu_s'), elapsed):
                row[key] += value

    def simulate(self, *args, **kwargs):
        return self._call('simulate', *args, **kwargs)

    def fetch_results(self, *args, **kwargs):
        return self._call('fetch_results', *args, **kwargs)

    def start(self):
        self.context._physx_sim_interface = self

    def stop(self):
        self.context._physx_sim_interface = self.original

    def report(self):
        return dict(calls=self.timings,
                    scope='Existing native calls only; no extra GPU synchronization; CPU time is not GPU kernel duration')
