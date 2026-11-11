"""Offline spatial packing of the authored visual farm, without physics edits.

Runtime opens only the current/next packed section. Global instance identities,
compiled vertices, materials and dynamic body numbering are preserved. These
are camera/display assets, never replacement collision or continuation models.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from newton_inputs import sha256


class PackedScene:
    """Small read-only geometry interface shared by the existing two renderers."""
    def __init__(self, path, expected_sha=None):
        path=Path(path)
        if expected_sha is not None and sha256(path)!=expected_sha:
            raise ValueError('Packed scene SHA mismatch: '+str(path))
        with np.load(path,allow_pickle=False) as arrays:
            self.__dict__.update({k:arrays[k] for k in arrays.files})
        self.ngeom=len(self.geom_names);self.nbody=len(self.body_names)
        self._body_ids={str(n):i for i,n in enumerate(self.body_names)}

    def body(self, key):
        i=self._body_ids[key] if isinstance(key,str) else int(key)
        return SimpleNamespace(name=str(self.body_names[i]),id=i)

    def geom(self, i):
        return SimpleNamespace(name=str(self.geom_names[i]),id=int(i))


def pack_scene(model, selected, path):
    selected=np.asarray(selected,dtype=int)
    meshes=sorted(set(int(m) for m in model.geom_dataid[selected] if m>=0))
    mesh_map={old:new for new,old in enumerate(meshes)}
    data={name:np.asarray(getattr(model,name)[selected]).copy() for name in
          ('geom_type','geom_size','geom_bodyid','geom_pos','geom_quat','geom_rgba')}
    # Resolve materials once; retain authored RGBA, without textures/decimation
    # (the current Newton/Viser renderers also consume these same RGBA values).
    from newton_viser import geom_color
    data.update(geom_rgba=np.array([geom_color(model,i) for i in selected]),
        geom_matid=np.full(len(selected),-1,dtype=int),mat_rgba=np.empty((0,4)),
        geom_dataid=np.array([mesh_map.get(int(model.geom_dataid[i]),-1) for i in selected]),
        geom_names=np.array([model.geom(i).name for i in selected]),
        body_names=np.array([model.body(i).name for i in range(model.nbody)]))
    for kind in ('vert','face'):
        arrays=[getattr(model,'mesh_'+kind)[getattr(model,'mesh_'+kind+'adr')[m]:
                    getattr(model,'mesh_'+kind+'adr')[m]+getattr(model,'mesh_'+kind+'num')[m]] for m in meshes]
        lengths=np.array([len(a) for a in arrays],dtype=int)
        data['mesh_'+kind]=np.concatenate(arrays) if arrays else np.empty((0,3))
        data['mesh_'+kind+'num']=lengths
        data['mesh_'+kind+'adr']=np.r_[0,np.cumsum(lengths)[:-1]].astype(int) if meshes else np.array([],dtype=int)
    np.savez_compressed(path,**data)


def ray_radius(mount):
    """Conservative Euclidean range for optical-Z depth, including corner rays."""
    k=np.asarray(mount['K']);w,h=mount['resolution_wh']
    x=max(abs(-k[0,2]/k[0,0]),abs((w-k[0,2])/k[0,0]))
    y=max(abs(-k[1,2]/k[1,1]),abs((h-k[1,2])/k[1,1]))
    return float(mount['depth_range_m'][1]*np.sqrt(1+x*x+y*y))


def geom_y_bounds(model):
    """World-static surface bounds, including long meshes across a section edge."""
    from newton_viser import geom_mesh
    from scipy.spatial.transform import Rotation
    cache={};bounds=np.empty((model.ngeom,2))
    for i in range(model.ngeom):
        if model.geom_bodyid[i]!=0:bounds[i]=[-np.inf,np.inf];continue
        key=(int(model.geom_type[i]),int(model.geom_dataid[i]),tuple(model.geom_size[i]))
        if key not in cache:cache[key]=geom_mesh(model,i,local_transform=False).vertices
        q=model.geom_quat[i]
        y=cache[key]@Rotation.from_quat(q[[1,2,3,0]]).as_matrix()[1]+model.geom_pos[i,1]
        bounds[i]=[np.min(y),np.max(y)]
    return bounds


def prepare(assets,output,*,stride=2.,overlap=2.,camera_excursion=2.):
    """One-time export; deliberately not called on the moving simulation thread."""
    import mujoco as mj
    assets=Path(assets).resolve(strict=True);output=Path(output)
    if not all(np.isfinite(v) and v>0 for v in (stride,overlap,camera_excursion)):
        raise ValueError('Positive finite section dimensions required')
    config=json.loads((assets/'patrol_assets.json').read_text())
    if config.get('start_modes'):raise ValueError('Prepare each authored elbow profile separately')
    mount=config.get('mounted_rgbd') or json.loads((Path(config['plants'][0]['assembly'])/'mounted_rgbd.json').read_text())
    model=mj.MjModel.from_xml_path(str(assets/'model/scene.xml'))
    catalog=json.loads((assets/'sensor_catalog.json').read_text())
    output.mkdir(parents=True,exist_ok=False)
    bounds=geom_y_bounds(model)
    # Camera-relative range is independent of yaw, lift or elbow branch. The
    # additional body excursion is asserted against measured camera poses.
    radius=ray_radius(mount)
    source_hashes={name:sha256(assets/name) for name in
                   ('patrol_assets.json','model/scene.xml','sensor_catalog.json','inputs/input_model.mjb')}
    signatures={name:[(assets/name).stat().st_size,(assets/name).stat().st_mtime_ns,
                       (assets/name).stat().st_ino] for name in source_hashes}
    chunks=[]
    previous_selected=None;previous_payload=None
    plant=np.array([bool(catalog['objects'].get(str(catalog['geom_to_instance'].get(model.geom(i).name,0)),{}).get('plant'))
                    for i in range(model.ngeom)])
    for index,lo in enumerate(np.arange(config['start_station_m'],config['end_station_m'],stride)):
        hi=min(float(lo+stride),config['end_station_m'])
        valid=[float(lo-overlap),float(hi+overlap)]
        camera_y=[valid[0]-camera_excursion,valid[1]+camera_excursion]
        extent=[camera_y[0]-radius,camera_y[1]+radius]
        # Keep infrastructure and every dynamic body; crop static vegetation
        # only. Outside-range RGB background can change; in-range occluders
        # cannot be removed, regardless of whether the object has a GT label.
        selected=np.flatnonzero((model.geom_bodyid!=0)|~plant|((bounds[:,1]>=extent[0])&(bounds[:,0]<=extent[1])))
        if previous_selected is not None and np.array_equal(selected,previous_selected):
            file,labels=previous_payload
        else:
            file=f'section_{index:04d}.npz';pack_scene(model,selected,output/file)
            geom_lut={model.geom(i).name:catalog['geom_to_instance'][model.geom(i).name]
                      for i in selected if model.geom(i).name in catalog['geom_to_instance']}
            ids=set(geom_lut.values())-{0}
            # mesh_paths are export provenance, unused by recognition and huge.
            objects={str(i):{k:v for k,v in catalog['objects'][str(i)].items() if k!='mesh_paths'} for i in ids}
            labels=f'section_{index:04d}.json'
            (output/labels).write_text(json.dumps(dict(objects=objects,geom_to_instance=geom_lut))+'\n')
            previous_selected=selected;previous_payload=(file,labels)
        chunks.append(dict(index=index,core_station_m=[float(lo),hi],valid_station_m=valid,
            camera_y_m=camera_y,geometry_y_m=extent,geometry=file,geometry_sha256=sha256(output/file),
            catalog=labels,catalog_sha256=sha256(output/labels),ngeom=len(selected),
            nstatic=int(np.sum(model.geom_bodyid[selected]==0)),bytes=(output/file).stat().st_size+(output/labels).stat().st_size))
    manifest=dict(schema='s300-visual-stream-v1',assets=str(assets),source_hashes=source_hashes,source_signatures=signatures,
        mount=mount,stride_m=stride,overlap_m=overlap,camera_excursion_m=camera_excursion,
        ray_radius_m=radius,chunks=chunks,source_ngeom=model.ngeom,
        scope='Visual camera/display sections only; physics, prepared route and harvest ledger unchanged')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def stream_manifest(assets,mode='auto'):
    if mode=='full':return None
    path=Path(assets)/'scene_stream/manifest.json'
    if not path.exists():
        if mode=='stream':raise ValueError('Prepare scene_stream before using --scene-loading stream')
        return None
    manifest=json.loads(path.read_text())
    if manifest['schema']!='s300-visual-stream-v1':raise ValueError('Unsupported scene stream')
    # Verify configuration identity, not the whole visual XML/catalog at startup.
    # Each loaded geometry/catalog file has its own verified content hash.
    for name,expected in manifest['source_hashes'].items():
        source=Path(assets)/name;stat=source.stat()
        signature=[stat.st_size,stat.st_mtime_ns,stat.st_ino]
        if signature!=manifest.get('source_signatures',{}).get(name) and sha256(source)!=expected:
            raise ValueError('Prepared stream source changed: '+name)
    return path


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('assets',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--stride',type=float,default=2.);p.add_argument('--overlap',type=float,default=2.)
    p.add_argument('--camera-excursion',type=float,default=2.)
    a=p.parse_args();m=prepare(a.assets,a.output,stride=a.stride,overlap=a.overlap,camera_excursion=a.camera_excursion)
    print(json.dumps({k:v for k,v in m.items() if k not in ('mount','source_hashes')},indent=2))
