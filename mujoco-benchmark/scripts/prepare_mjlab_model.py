"""Make a version-matched CPU/GPU model; record the sole compatibility change."""
import json,hashlib
from pathlib import Path
import mujoco as mj
HOME=Path(__file__).resolve().parents[1]
a=HOME/'models/robot_plant_optimized.xml';b=HOME/'models/robot_plant_mjlab.xml'
s=a.read_text();assert s.count('autoreset="disable"')==1
b.write_text(s.replace('autoreset="disable"','autoreset="enable"'))
m=mj.MjModel.from_xml_path(str(b));mj.mj_saveModel(m,str(b.with_suffix('.mjb')))
b.with_suffix('.json').write_text(json.dumps(dict(mujoco=mj.__version__,source_sha256=hashlib.sha256(a.read_bytes()).hexdigest(),xml_sha256=hashlib.sha256(b.read_bytes()).hexdigest(),changes=['MJWarp rejects mjDSBL_AUTORESET; remove that disable bit for BOTH matched CPU/GPU benchmarks. Valid finite-state trajectories are otherwise unchanged. Nonfinite/overflow results are rejected, never counted as fast valid runs.'],nq=m.nq,nv=m.nv,ngeom=m.ngeom),indent=2))
print('[mjlab 공통 모델]',b,flush=True)
