"""Diagnostic-only CUPTI trace of the first 32 motion physics steps.

Accepts gpu_probe_worker.py arguments; requires --profile-motion. Capturing a
trace adds overhead, so do not use this run for throughput ranking. GPU kernels
from native PhysX are captured through PyTorch's CUPTI/Kineto integration.
"""
import argparse
from pathlib import Path
import runpy

import gpu_step_profile

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument('--output', type=Path, required=True)
args, _ = parser.parse_known_args()


class CudaTraceProfile(gpu_step_profile.NativeStepProfile):
    def start(self):
        self.trace = None
        super().start()
        import torch
        self.trace = torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU,
                                                       torch.profiler.ProfilerActivity.CUDA])
        self.trace.__enter__()

    def _call(self, name, *a, **kw):
        if self.trace is not None:
            import torch
            with torch.profiler.record_function('physx.'+name):
                return super()._call(name, *a, **kw)
        return super()._call(name, *a, **kw)

    def finish_trace(self):
        if self.trace is not None:
            trace, self.trace = self.trace, None
            trace.__exit__(None, None, None)
            trace.export_chrome_trace(str(args.output/'cuda_trace.json'))

    def fetch_results(self, *a, **kw):
        result = super().fetch_results(*a, **kw)
        if self.timings['fetch_results']['calls'] == 32:
            self.finish_trace()
        return result

    def stop(self):
        self.finish_trace()
        super().stop()


if __name__ == '__main__':
    import sys
    if '--profile-motion' not in sys.argv:
        raise SystemExit('Use --profile-motion to activate the CUDA trace')
    gpu_step_profile.NativeStepProfile = CudaTraceProfile
    runpy.run_path(str(Path(__file__).with_name('gpu_probe_worker.py')), run_name='__main__')
