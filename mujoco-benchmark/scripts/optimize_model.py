"""Optional Phase-1 model: leaf collisions off, unbreakable fruit welds eliminated."""
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R
from convert_model import HOME,nums

def main():
    source=HOME/'models/plant_original_equivalent.xml';ref=json.loads((HOME/'assets/reference/reference.json').read_text());tree=ET.parse(source)
    names={s['name'] for s in ref['shapes'] if '/Appendage_' in s['path']};changed=0
    for g in tree.getroot().iter('geom'):
        if g.get('name') in names:g.set('contype','0');g.set('conaffinity','0');changed+=1
    if changed!=286:raise ValueError('Original leaf/stub inventory changed')
    # Eliminate the six free DOFs and weld for each fruit. All masses, inertia,
    # shapes and elastic joints remain. Detachment is impossible in this model.
    world=tree.getroot().find('worldbody');bodies={b.get('name'):b for b in world.iter('body')}
    reference={b['path']:b for b in ref['bodies']}
    for spec in ref['fruit_specs']:
        child=bodies[spec['name']];parent=bodies[spec['anchor'].rsplit('/',1)[-1]]
        cp=np.asarray(reference[spec['path']]['pose']);pp=np.asarray(reference[spec['anchor']]['pose']);cr=R.from_quat(cp[[4,5,6,3]]);pr=R.from_quat(pp[[4,5,6,3]])
        world.remove(child);child.remove(child.find('freejoint'));child.set('pos',nums(pr.inv().apply(cp[:3]-pp[:3])));q=(pr.inv()*cr).as_quat();child.set('quat',nums(q[[3,0,1,2]]));parent.append(child)
    eq=tree.getroot().find('equality');tree.getroot().remove(eq)
    target=source.with_name('plant_mujoco_optimized.xml');tree.write(target,encoding='unicode')
    target.with_suffix('.json').write_text(json.dumps(dict(source=str(source),optimization='disable 286 leaf/stub collisions and eliminate 11 unbreakable fruit welds/free joints; original elastic joints, bodies, masses/inertias retained',active_plant_colliders=90,elastic_dofs=234,fruit_free_dofs=0,detachment_supported=False,comparable_to_original_physics=False),indent=2))
    print('[별도 최적화 모델]',target)
if __name__=='__main__':main()
