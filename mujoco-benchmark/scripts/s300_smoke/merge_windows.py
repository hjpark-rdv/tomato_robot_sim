"""Combine exported target plants into one persistent native robot/farm model."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as E


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def merge(base, additions, output):
    base=Path(base);output=Path(output)
    shutil.copytree(base,output,copy_function=os.link)
    tree=E.parse(base/'scene.xml');root=tree.getroot();meta=json.loads((base/'export.json').read_text())
    # Never overwrite a hardlinked base artifact.
    for filename in ('scene.xml','export.json','model.mjb','source_plan.json',
                     'compiled_scene.mjb','compiled_scene.json'):
        p=output/filename
        if p.exists():p.unlink()

    def snapshot_plan(source, index, expected=None):
        source=Path(source)
        source_hash=sha(source)
        if expected is not None and source_hash!=expected:
            raise ValueError('Exported source plan hash mismatch: '+str(source))
        destination=output/f'window_{index:02d}_source_plan.json'
        # Own the snapshot's inode. A later source-plan change must not alter
        # the merged model's bound input, including sources copied with links.
        if destination.exists():destination.unlink()
        shutil.copy2(source,destination)
        if sha(destination)!=source_hash:
            raise ValueError('Source plan changed while copying: '+str(source))
        return dict(source_plan=str(destination.resolve()),source_plan_sha256=source_hash,
                    source_plan_original=str(source.resolve()))

    manifest=json.loads(Path(meta['source_manifest']).read_text())
    windows=[dict(id=manifest['plant']['source_plant'].rsplit('_',1)[1],prefix='',
                  source_assembly=meta['source_assembly'],source_trial=meta['source_trial'],
                  bodies=meta['rods']+meta['fruits'],fruit_specs=meta['fruit_specs'],
                  source_manifest=meta['source_manifest'],
                  **snapshot_plan(base/'source_plan.json',0,meta.get('source_plan_sha256')))]
    for prefix,model in additions:
        model=Path(model);add=json.loads((model/'export.json').read_text());sub=E.parse(model/'scene.xml').getroot()
        bodies=set(add['rods']+add['fruits'])
        selected=[copy.deepcopy(n) for n in sub.find('worldbody').findall('body') if n.get('name') in bodies]
        mesh_names={g.get('mesh') for node in selected for g in node.iter('geom') if g.get('mesh')}
        elems=selected+[copy.deepcopy(n) for n in sub.find('asset') if n.get('name') in mesh_names]
        elems += [copy.deepcopy(n) for n in sub.find('contact')]
        elems += [copy.deepcopy(n) for n in sub.find('equality')]
        rename={n.get('name'):prefix+n.get('name') for elem in elems for n in elem.iter() if n.get('name')}
        for elem in elems:
            for n in elem.iter():
                for key,value in list(n.attrib.items()):
                    if key in ('name','mesh','body1','body2','site1','site2','joint','geom1','geom2') and value in rename:n.set(key,rename[value])
                if n.tag=='mesh':
                    old=n.get('file');new=prefix+old
                    os.link(model/'meshes'/old,output/'meshes'/new);n.set('file',new)
        for n in elems:
            parent='asset' if n.tag=='mesh' else 'contact' if n.tag=='exclude' else 'equality' if n.tag=='connect' else 'worldbody'
            root.find(parent).append(n)
        for item in add['attachments']:
            item=copy.deepcopy(item);item['name']=prefix+item['name'];item['anchor']=prefix+item['anchor'];meta['attachments'].append(item)
        for g in add['geometry']:
            g=copy.deepcopy(g);g['name']=prefix+g['name'];g['owner']=prefix+g['owner'];meta['geometry'].append(g)
        source=json.loads(Path(add['source_manifest']).read_text())
        plan_path=model/'source_plan.json'
        if not plan_path.exists():plan_path=Path(add['source_trial'])/'plan.json'
        windows.append(dict(id=source['plant']['source_plant'].rsplit('_',1)[1],prefix=prefix,
            source_assembly=add['source_assembly'],source_trial=add['source_trial'],
            bodies=[prefix+n for n in add['rods']+add['fruits']],fruit_specs=add['fruit_specs'],
            source_manifest=add['source_manifest'],
            **snapshot_plan(plan_path,len(windows),add.get('source_plan_sha256'))))
    meta['windows']=windows;meta['rods']=[b for w in windows for b in w['bodies'] if '_Fruit_' not in b]
    meta['fruits']=[s['name'] for s in meta['attachments']]
    meta['schema']='s300-persistent-multiplant-native-v1'
    E.indent(root);tree.write(output/'scene.xml',encoding='unicode');meta['model_sha256']=sha(output/'scene.xml')
    (output/'export.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(dict(model=str(output),windows=[w['id'] for w in windows],fruits=len(meta['fruits'])),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('base',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--add',nargs=2,action='append',default=[],metavar=('PREFIX','MODEL'))
    a=p.parse_args();merge(a.base,a.add,a.output)
