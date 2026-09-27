import sys,json,gzip,hashlib,subprocess
from pathlib import Path
import numpy as np
sys.path.insert(0,'/root/farmily_tomato/mujoco-benchmark/scripts')
from robot_engine import forward
from suite import RING
from diagnose_contact_timing import load_engine
from measure_seating_rollout import SeatingProbe
from target_fruit_contact_trial import scope_from_engine
from hook_retention_diagnostic import capsule_endpoints
O=Path(__file__).parent;R=Path('/root/docker_share/mujoko_debugging_data/20260927_hook_seating_server/robot_under')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checks={}
for file in O.glob('*/contact_scope.json'):
 for name,digest in json.loads(file.read_text())['source_sha256'].items():
  checks[name]=sha(Path(name))==digest
assert all(checks.values())
e,t,p=load_engine(R,'seating_00_under');probe=SeatingProbe(e,t);scope=scope_from_engine(e,probe.mapping)
position,rotation=probe.mapping.frame(e.data)
mapping=dict(scope=scope.__dict__,ring_world=position.tolist(),hook_body_world=e.data.xpos[e.hook].tolist(),
 ring_offset_in_hook_m=RING.tolist(),initial_fruit_world=e.data.xpos[e.fruit].tolist(),
 shapes=[dict(name=n,body=e.model.body(int(e.model.geom_bodyid[e.model.geom(n).id])).name,
       endpoints_world=[v.tolist() for v in capsule_endpoints(e.model,e.data,e.model.geom(n).id)[:2]],
       radius_m=float(e.model.geom(n).size[0])) for n in (*scope.wires,*scope.pedicels)])
(O/'mapping_verified.json').write_text(json.dumps(mapping,indent=2)+'\n')
old=json.loads((R/'candidates/seating_00_under/trace.json').read_text());slowroot=O/'slow_run'
new=json.loads((slowroot/'candidates/seating_00_under/trace.json').read_text());meta=json.loads((slowroot/'retiming.json').read_text())
assert [new[i] for i in meta['original_command_indices']]==old
errors=[]
for a,b,lo,hi in zip(old,old[1:],meta['original_command_indices'],meta['original_command_indices'][1:]):
 for j in range(lo,hi+1):
  f=(j-lo)/(hi-lo);errors.append(float(np.max(abs(np.array(new[j]['command'])-(np.array(a['command'])*(1-f)+np.array(b['command'])*f)))))
assert max(errors)==0
ref=json.loads((R/'replay_assets/reference.json').read_text())
def speed(rows):
 speeds=[]
 for a,b in zip(rows,rows[1:]):
  if 'seat' not in (a['phase'],b['phase']):continue
  ps=[]
  for f in np.linspace(0,1,5):
   m=forward(ref,np.array(a['command'])*(1-f)+np.array(b['command'])*f);ps.append(m[:3,3]+m[:3,:3]@RING)
  speeds.extend(np.linalg.norm(np.diff(ps,axis=0),axis=1)*240)
 return float(max(speeds))
orig_speed=speed(old);slow_speed=speed(new)
assert slow_speed<.002
result=dict(source_hashes_unchanged=checks,all_sources_unchanged=True,original_vertices_exact=True,
 piecewise_joint_path_max_error=0.,stretch_factor=18,source_duration_s=(len(old)-1)/60,
 slow_duration_s=(len(new)-1)/60,source_nominal_seat_max_tcp_speed_m_s=orig_speed,
 slow_nominal_seat_max_tcp_speed_m_s=slow_speed,speed_scope='finite differences at 240Hz along commanded FK RING centre; not actual dynamics')
# A/B stop points in the same original joint-polyline parameter.
stop=json.loads((O/'trial_slow_seating_00_under/target_contact_trial.json').read_text())['simulated_s']
result['slow_stop_equivalent_original_time_s']=float(np.interp(stop,np.array(meta['original_command_indices'])/60,np.arange(len(old))/60))
(O/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
videos=[]
for v in sorted((O/'videos').glob('*.mp4')):
 subprocess.run(['ffmpeg','-v','error','-i',str(v),'-f','null','-'],check=True,stdout=subprocess.DEVNULL)
 info=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-show_entries','stream=width,height,nb_read_frames,r_frame_rate','-of','json',str(v)]))
 videos.append(dict(path=v.name,sha256=sha(v),full_decode_passed=True,info=info))
(O/'video_validation.json').write_text(json.dumps(videos,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='source_hashes_unchanged'},indent=2))
