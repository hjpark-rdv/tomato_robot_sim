"""Human-readable tables from the extracted native/USD values."""
import csv,json
from pathlib import Path
import numpy as np
HOME=Path(__file__).resolve().parents[1]
def write(path,rows):
    with path.open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
def main():
    root=HOME/'assets/reference';d=json.loads((root/'reference.json').read_text())
    write(root/'body_parameters.csv',[dict(name=b['name'],path=b['path'],mass_kg=b['mass'],x_m=b['pose'][0],y_m=b['pose'][1],z_m=b['pose'][2],
        com_x_m=b['com_pose_xyzw'][0],com_y_m=b['com_pose_xyzw'][1],com_z_m=b['com_pose_xyzw'][2],
        Ixx_kgm2=b['inertia'][0],Iyy_kgm2=b['inertia'][4],Izz_kgm2=b['inertia'][8]) for b in d['bodies']])
    joints=[]
    for j in d['joints']:
        for a in j['axes'] or [dict(axis='fixed',k=None,d=None,limit_deg=[None,None],max_force=None)]:
            joints.append(dict(joint=j['path'],parent=j['parent'],child=j['child'],axis=a['axis'],stiffness_Nm_rad=a['k'],damping_Nms_rad=a['d'],lower_deg=a['limit_deg'][0],upper_deg=a['limit_deg'][1],drive_limit_Nm=a['max_force'],source_break_N=j['break_force'],source_break_Nm=j['break_torque'],phase1_break_enabled=False))
    write(root/'joint_parameters.csv',joints)
    write(root/'collider_parameters.csv',[dict(name=s['name'],path=s['path'],body=s['body'],type=s['type'],radius_m=s.get('radius'),
        axis_length_m=float(np.linalg.norm(np.diff(s['ends'],axis=0))) if 'ends' in s else None,mesh_vertices=len(s.get('vertices',[])),mesh_triangles=len(s.get('faces',[])),contact_offset_m=s.get('contact_offset'),rest_offset_m=s.get('rest_offset')) for s in d['shapes']])
    summary=dict(source_commit=d['source_commit'],source_sha256=d['source_sha256'],plant_model=d['plant_model'],gravity=d['gravity'],armature=d['armature'],friction=d['friction'],mass_kg=sum(b['mass'] for b in d['bodies']),bodies=len(d['bodies']),colliders=len(d['shapes']),visual_meshes=len(d['visuals']))
    (root/'reference_summary.json').write_text(json.dumps(summary,indent=2));print('[변환표 저장]',root)
if __name__=='__main__':main()
