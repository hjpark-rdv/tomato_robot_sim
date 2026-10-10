"""Small wall-time ledger for the existing native motion runner."""
import json
import time
from collections import defaultdict
from contextlib import contextmanager


class MotionProfile:
    def __init__(self, output):
        self.output=output
        self.total=defaultdict(float)
        self.count=defaultdict(int)
        self.maximum=defaultdict(float)
        self.started=time.perf_counter()

    @contextmanager
    def section(self, name):
        start=time.perf_counter()
        try:
            yield
        finally:
            elapsed=time.perf_counter()-start
            self.total[name]+=elapsed
            self.count[name]+=1
            self.maximum[name]=max(self.maximum[name],elapsed)

    def checkpoint(self, tick, operation):
        # Only first frames and periodic progress checkpoints; no per-tick I/O.
        data=dict(tick=tick,operation=operation,wall_s=time.perf_counter()-self.started,
                  sections={name:dict(total_s=value,count=self.count[name],max_s=self.maximum[name])
                            for name,value in self.total.items()})
        self.output.write_text(json.dumps(data,indent=2)+'\n')
        return data
