"""Read-only initial convex geometry overlap screen, independent of contact masks.

MuJoCo treats each mesh as a convex hull: curved branches/leaves can produce
false positives. Same-truss contacts include authored branch/calyx connections.
This is not an exact triangle-mesh audit or dynamic physics validation.
"""
import argparse,collections,hashlib,html,json,struct,time
from pathlib import Path
import numpy as np
import mujoco as mj

def glb_labels(path):
    b=Path(path).read_bytes();n,_=struct.unpack_from('<II',b,12);doc=json.loads(b[20:20+n]);labels=[]
    def visit(i):
        node=doc['nodes'][i]
        if 'mesh' in node:
            for k,_ in enumerate(doc['meshes'][node['mesh']]['primitives']):labels.append(node.get('name',f'node_{i}')+f'/primitive_{k}')
        for child in node.get('children',[]):visit(child)
    for i in doc['scenes'][doc.get('scene',0)]['nodes']:visit(i)
    return labels

def audit(folder,threshold=.0005):
    started=time.perf_counter();xml=folder/'scene.xml';meta=json.loads((folder/'scene.json').read_text());m=mj.MjModel.from_xml_path(str(xml));d=mj.MjData(m);mj.mj_forward(m,d)
    labels={};owners={};bounds={};truss_ids=set()
    for truss in meta['trusses']:
        bid=m.body(truss['id']).id;gids=np.where(m.geom_bodyid==bid)[0];names=glb_labels(truss['source_glb'])
        if len(names)!=len(gids):raise ValueError('Source mesh/geom count mismatch')
        for g,label in zip(gids,names):labels[int(g)]=label;owners[int(g)]=truss['id'];truss_ids.add(int(g))
    for g in range(m.ngeom):
        if g not in owners:
            body=m.body(m.geom_bodyid[g]).name
            if not body.startswith('STEM_MainStem_'):continue
            owners[g]='main_stem' if m.geom_type[g]==mj.mjtGeom.mjGEOM_CAPSULE else 'stem_appendages'
            labels[g]=m.geom(g).name
        rot=d.geom_xmat[g].reshape(3,3);pos=d.geom_xpos[g]
        if m.geom_type[g]==mj.mjtGeom.mjGEOM_MESH:
            mid=m.geom_dataid[g];offset=m.mesh_vertadr[mid];count=m.mesh_vertnum[mid]
            v=m.mesh_vert[offset:offset+count].astype(float)@rot.T+pos;lo=v.min(0);hi=v.max(0)
        elif m.geom_type[g]==mj.mjtGeom.mjGEOM_CAPSULE:
            ends=np.array([pos-rot[:,2]*m.geom_size[g,1],pos+rot[:,2]*m.geom_size[g,1]])
            lo=ends.min(0)-m.geom_size[g,0];hi=ends.max(0)+m.geom_size[g,0]
        else:raise ValueError('Unsupported shape')
        bounds[g]=(lo,hi)
    ids=sorted(bounds);lo=np.array([bounds[g][0] for g in ids]);hi=np.array([bounds[g][1] for g in ids]);overlaps=[];tested=0;counts=collections.Counter()
    for index,g in enumerate(ids):
        candidates=np.where(np.all(hi[index]>=lo[index+1:],axis=1)&np.all(hi[index+1:]>=lo[index],axis=1))[0]+index+1
        for index2 in candidates:
            h=ids[index2]
            if g not in truss_ids and h not in truss_ids:continue
            if owners[g]==owners[h]:category='within_truss'
            elif 'main_stem' in (owners[g],owners[h]):category='truss_main_stem'
            elif 'stem_appendages' in (owners[g],owners[h]):category='truss_leaves_stubs'
            else:category='between_trusses'
            tested+=1;counts[category]+=1
            segment=np.zeros(6);distance=float(mj.mj_geomDistance(m,d,g,h,.001,segment))
            if distance < -threshold:
                overlaps.append(dict(category=category,includes_trichomes=('Trichomes' in labels[g] or 'Trichomes' in labels[h]),geom1=g,geom2=h,owner1=owners[g],owner2=owners[h],shape1=labels[g],shape2=labels[h],penetration_m=-distance,witness_points=segment.tolist()))
    overlaps.sort(key=lambda x:x['penetration_m'],reverse=True)
    groups={}
    for key in ['within_truss','truss_main_stem','truss_leaves_stubs','between_trusses']:
        selected=[x for x in overlaps if x['category']==key]
        structural=[x for x in selected if not x['includes_trichomes']]
        groups[key]=dict(non_hair_overlap_pairs=len(structural),non_hair_max_penetration_m=max([x['penetration_m'] for x in structural],default=0),candidate_pairs=counts[key],overlaps_over_threshold=len(selected),max_penetration_m=max([x['penetration_m'] for x in selected],default=0),examples=selected[:5])
    result=dict(scene=str(folder),scene_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),method='explicit_mj_geomDistance_convex_hulls_bypassing_contact_masks',threshold_m=threshold,geom_count=len(ids),truss_geom_count=len(truss_ids),tested_pairs=tested,groups=groups,overlaps=overlaps,elapsed_s=time.perf_counter()-started,physics_ready=False,limitations=['Mesh collision uses convex hull, not exact original triangle surfaces','Within-truss overlaps include intentional connections; not automatically failures','Main stem attachment may contain intentional junction overlap','Truss geometry has contact masks disabled and no elastic physics; this is a separate read-only geometry screen'])
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('scene',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);r=audit(args.scene);(args.output/'audit.json').write_text(json.dumps(r,indent=2))
    rows=''.join(f'<tr><td>{x["category"]}</td><td>{html.escape(x["owner1"]+" / "+x["shape1"])}</td><td>{html.escape(x["owner2"]+" / "+x["shape2"])}</td><td>{x["penetration_m"]*1000:.3f}</td></tr>' for x in r['overlaps'][:200])
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>GLB 초기 겹침 검사</title><style>body{font:15px sans-serif}td,th{padding:6px;border:1px solid #ddd}</style><h1>초기 convex 형상 겹침 검사</h1><p>0.5mm 초과. 접촉 마스크와 무관하게 검사. 곡선 가지/잎의 convex hull 오탐 및 정상 접합 포함. 실제 메시 관통 또는 물리 통과 판정과 다릅니다.</p><pre>'+html.escape(json.dumps({k:{a:b for a,b in v.items() if a!='examples'} for k,v in r['groups'].items()},indent=2))+'</pre><table><tr><th>범주</th><th>형상1</th><th>형상2</th><th>겹침 mm</th></tr>'+rows+'</table>',encoding='utf-8')
    print(json.dumps({k:{a:b for a,b in v.items() if a!='examples'} for k,v in r['groups'].items()},indent=2));print('elapsed_s',r['elapsed_s'])
if __name__=='__main__':main()
