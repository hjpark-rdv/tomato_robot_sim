"""Matched idle/force and contact-response checks for the GLB migration pilot."""
import argparse,json,time
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import mujoco as mj
from robot_engine import RobotEngine,DEFAULT_MODEL

def run(model,ref,force=False,probe=False):
 e=RobotEngine(model,reference=ref,hz=240);m,d=e.model,e.data;target=e.fruit;plant=[m.body(x['name']).id for x in e.ref['bodies']];initial=d.xpos[plant].copy();q0=d.qpos.copy()
 initial_contacts=[dict(a=m.geom(c.geom1).name,b=m.geom(c.geom2).name,penetration_m=float(-c.dist)) for c in d.contact if c.dist<-.0005]
 # Warm-up is measured, not silently accepted as the authored initial geometry.
 maxwarm=0.;rows=[];peak_pen=0.;probecontacts=0
 for step in range(1440):
  t=step/240
  d.xfrc_applied[:]=0
  if force and 2.5<=t<3.:d.xfrc_applied[target,:3]=[.2,0,0]
  if probe:
   pid=m.body('ContactProbe').id;mid=m.body_mocapid[pid]
   if step==480:
    fruit=d.xpos[target].copy();radius=e.target_spec['radius']
   if step>=480:
    progress=np.clip((t-2.5)/.5,0,1) if t<3 else max(0,1-(t-3)/.5)
    d.mocap_pos[mid]=fruit+np.array([radius+.014-.025*progress,0,0])
   else:d.mocap_pos[mid]=[0,0,10]
  mj.mj_step(m,d);mj.mj_forward(m,d)
  if step<480:maxwarm=max(maxwarm,float(np.linalg.norm(d.xpos[plant]-initial,axis=1).max()))
  if probe:
   gid=m.geom('contact_probe').id;contacts=[c for c in d.contact if c.geom1==gid or c.geom2==gid]
   probecontacts+=len(contacts);peak_pen=max(peak_pen,max([-float(c.dist) for c in contacts],default=0))
  rows.append(d.xpos[target].copy())
  if not np.isfinite(d.qpos).all() or np.max(np.abs(d.qvel))>1e4 or np.any(d.warning.number):raise RuntimeError('Unstable mechanics test')
 final_q=d.qpos.copy();e.reset();np.testing.assert_array_equal(e.data.qpos,q0)
 return np.asarray(rows),dict(initial_contacts_over_0_5mm=initial_contacts,warmup_max_plant_motion_m=maxwarm,warning_counts=d.warning.number.tolist(),reset_exact=True,contact_probe_events=probecontacts,contact_probe_max_penetration_m=peak_pen,qpos_changed=bool(np.linalg.norm(final_q-q0)>1e-6))

def main():
 p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args();folder=a.folder;ref=folder/'reference.json';model=folder/'model.xml'
 idle,info=run(model,ref);forced,finfo=run(model,ref,force=True);delta=np.linalg.norm(forced-idle,axis=1);peak=float(delta.max());residual=float(delta[-1]);assert peak>1e-5
 tree=E.parse(model);body=E.SubElement(tree.getroot().find('worldbody'),'body',name='ContactProbe',mocap='true',pos='0 0 10');E.SubElement(body,'geom',name='contact_probe',type='sphere',size='.01',contype='4',conaffinity='1',rgba='0.2 0.6 1 1');tree.write(folder/'contact_probe.xml',encoding='unicode')
 pushed,pinfo=run(folder/'contact_probe.xml',ref,probe=True);assert pinfo['contact_probe_events']>0;push_delta=float(np.linalg.norm(pushed-idle,axis=1).max());assert push_delta>1e-5
 baseline,binfo=run(DEFAULT_MODEL,None);baseline_force,_=run(DEFAULT_MODEL,None,force=True);baseline_delta=np.linalg.norm(baseline_force-baseline,axis=1)
 result=dict(idle=info,force=finfo,force_peak_delta_vs_idle_m=peak,force_residual_delta_vs_idle_m=residual,recovered_fraction=1-residual/peak,contact=pinfo,contact_peak_delta_vs_idle_m=push_delta,baseline_force_peak_delta_m=float(baseline_delta.max()),baseline_force_residual_delta_m=float(baseline_delta[-1]),force_N=[.2,0,0],force_interval_s=[2.5,3],duration_s=6,hz=240,restoration_passed=bool(residual<=.25*peak),restoration_rule='Residual at 6s <=25% of peak matched-idle displacement',scope='Matched idle force/contact diagnostics; consult restoration_passed; not physical equivalence or collection acceptance',initial_geometry_valid=not info['initial_contacts_over_0_5mm'])
 np.savez_compressed(folder/'mechanics_traces.npz',times=np.arange(1,1441)/240,idle=idle,force=forced,contact=pushed,baseline_idle=baseline,baseline_force=baseline_force)
 (folder/'mechanics_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
