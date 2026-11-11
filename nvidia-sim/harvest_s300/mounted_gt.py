"""Visible-pixel SIM-GT labels/overlays from USD identity, never RGB inference.

Replicator supplies the nearest visible mesh per pixel, including instanced
background plants. Source names and authored colors classify those meshes.
No USD geometry, material, physics, or camera transform is modified here.
"""
import colorsys
import copy
import html
import json
import math
import os
import re
import shutil
import subprocess
import zlib
from pathlib import Path

import numpy as np
from recording_storage import save_array, save_arrays, save_shared_labels, array_path
from PIL import Image, ImageDraw, ImageFont

CLASSES = {'unknown':0, 'fruit':1, 'main_stem':2, 'rachis':3,
           'pedicel':4, 'peduncle':5, 'calyx':6, 'leaf':7,
           'gutter':8, 'rail':9, 'structure':10, 'robot':11, 'ground':12}
COLORS = {'unknown':(100,100,100), 'main_stem':(60,255,45),
          'rachis':(130,255,55), 'pedicel':(245,220,30),
          'peduncle':(175,245,35), 'calyx':(55,170,110),
          'leaf':(50,160,95), 'gutter':(55,195,255), 'rail':(245,215,90),
          'structure':(130,150,165), 'robot':(180,160,230), 'ground':(110,120,130)}
FRUIT_COLORS = {'red':(255,65,45), 'turning':(255,185,25),
                'green':(130,235,40), 'unknown':(190,190,190)}


def authored_color(rgb):
    if rgb is None or len(rgb)!=3 or not np.isfinite(rgb).all():
        return dict(color_stage='unknown',red_proxy=False,authored_rgb=rgb)
    h,s,v=colorsys.rgb_to_hsv(*rgb);h*=360
    # Same SIM-GT red-color proxy as prepare_harvest_group_policy.py.
    red=(h<=20 or h>=340) and s>=.4 and v>=.15
    stage='red' if red else ('turning' if 20<h<70 and s>=.4 else
                            'green' if 70<=h<=170 and s>=.2 else 'unknown')
    return dict(color_stage=stage,red_proxy=red,authored_rgb=rgb,hue_deg=h)


def classify(path, source, *, known_plant=False):
    if path.startswith(('/World/HarvestRobot/','/World/ActuatedTool/')):return 'robot'
    plant=(known_plant or '/Plants/' in path or '/ElasticTarget/' in path or '/ElasticNeighbors/' in path)
    if plant:
        if re.fullmatch(r'Fruit_\d+',source):return 'fruit'
        if 'Calyx' in source:return 'calyx'
        if 'Pedicel' in source:return 'pedicel'
        if 'Peduncle' in source:return 'peduncle'
        if 'Rachis' in source:return 'rachis'
        if 'Leaf' in source or 'Petiole' in source:return 'leaf'
        if 'MainStem' in source or 'Pruned_' in source or 'Removed_Truss' in source:return 'main_stem'
        return 'unknown'
    name=path.lower()
    if 'gutter' in name:return 'gutter'
    if 'rail' in name:return 'rail'
    if 'ground' in name or 'foundation' in name:return 'ground'
    if '/Greenhouse/House/' in path:return 'structure'
    return 'unknown'


def stable_id(key):
    return (zlib.crc32(key.encode()) & 0xffffffff) or 1


def build_catalog(stage, assembly):
    from pxr import Usd, UsdGeom
    manifest=json.loads((Path(assembly)/'motion_manifest.json').read_text()) if assembly else {}
    sources={};skin_truss={}
    plants=[(manifest['plant'],'skin.npz')] if 'plant' in manifest else []
    plants += [(n['plant'],n['skin_file']) for n in manifest.get('elastic_neighbors',[])]
    for plant,skinfile in plants:
        sources[plant['root']]=plant['source_plant']
        with np.load(Path(assembly)/skinfile) as skin:
            for i,path in enumerate(plant['skin_paths']):
                names=skin[f'{i}_body_paths'].tolist()
                trusses={m.group(1) for n in names for m in [re.search(r'/(Truss_\d+)_',n)] if m}
                if len(trusses)==1:skin_truss[path]=next(iter(trusses))
    # Native route switching reparents outgoing measured plant geometry. The
    # history explicitly binds that new root to its original plant identity;
    # a RouteFrozen-looking name alone does not establish a plant mapping.
    for outgoing in manifest.get('route_checkpoint',{}).get('outgoing_history',[]):
        root=outgoing.get('frozen_root');original=outgoing.get('source_plant')
        if (not isinstance(root,str) or not isinstance(original,str) or
                not re.fullmatch(r'/World/RouteFrozen/Plant_\d+',root) or
                not re.fullmatch(r'/World/Greenhouse/Plants/Plant_\d+',original) or
                root.rsplit('/',1)[-1]!=original.rsplit('/',1)[-1]):
            raise ValueError('Exact frozen-root/original-plant route identity required')
        if root in sources and sources[root]!=original:
            raise ValueError('Conflicting frozen plant source identity')
        sources[root]=original
    meshes={};objects={}
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Gprim):continue
        path=str(prim.GetPath());source=str(prim.GetCustomDataByKey('sourceName') or '')
        match=re.search(r'(/World/Greenhouse/Plants/Plant_\d+)(?:/|$)',path)
        plant=match.group(1) if match else None
        for root,original in sources.items():
            if path.startswith(root+'/'):plant=original;break
        kind=classify(path,source,known_plant=plant is not None)
        truss_match=re.search(r'/(Truss_\d+)(?:/|_)',path)
        truss=truss_match.group(1) if truss_match else skin_truss.get(path)
        truss_source=('authored_mesh_or_body_path' if truss_match else
                      'active_skin_body_mapping' if path in skin_truss else 'unknown')
        source_base=re.sub(r'_Trichomes$','',source)
        if kind=='main_stem':key=(plant or path)+'/MainStem'
        elif plant and kind!='unknown':key='/'.join(x for x in [plant,truss,source_base] if x)
        else:key=path
        identity=stable_id(key)
        if identity in objects and objects[identity]['key']!=key:raise ValueError('Stable object ID collision')
        color=prim.GetAttribute('primvars:displayColor').Get()
        rgb=list(map(float,color[0])) if color is not None and len(color)==1 else None
        row=dict(instance_id=identity,key=key,class_name=kind,class_id=CLASSES[kind],
                 plant=plant,truss=truss,source_name=source_base,truss_identity_source=truss_source)
        if kind=='fruit':row.update(authored_color(rgb))
        if identity not in objects:objects[identity]=dict(row,mesh_paths=[])
        objects[identity]['mesh_paths'].append(path)
        meshes[path]=identity
    return meshes,objects


def resolve_pixels(raw, labels, meshes, objects):
    raw=np.asarray(raw,dtype=np.uint32)
    values,inverse=np.unique(raw,return_inverse=True)
    instance_lut=np.zeros(len(values),np.uint32);semantic_lut=np.zeros(len(values),np.uint8)
    unresolved={}
    for i,render_id in enumerate(values):
        path=labels.get(str(int(render_id)),labels.get(int(render_id)))
        if path in ('BACKGROUND','UNLABELLED',None):
            if int(render_id)>1:unresolved[int(render_id)]=path
            continue
        if not isinstance(path,str):raise ValueError('Unexpected renderer identity: '+repr(path))
        identity=meshes.get(path)
        if identity is None:unresolved[int(render_id)]=path;continue
        instance_lut[i]=identity;semantic_lut[i]=objects[identity]['class_id']
    return instance_lut[inverse].reshape(raw.shape),semantic_lut[inverse].reshape(raw.shape),unresolved


def visible_objects(instance,depth,objects,K,world_from_optical):
    result=[]
    values,inverse,counts=np.unique(instance,return_inverse=True,return_counts=True)
    order=np.argsort(inverse.ravel(),kind='stable');offset=0
    for identity,count in zip(values,counts):
        pixels=order[offset:offset+count];offset+=count
        if not identity:continue
        row=objects[int(identity)];ys,xs=np.divmod(pixels,instance.shape[1])
        valid=np.isfinite(depth[ys,xs]);z=depth[ys[valid],xs[valid]]
        points=np.c_[(xs[valid]-K[0,2])*z/K[0,0],(ys[valid]-K[1,2])*z/K[1,1],z]
        # Visible surface median, NOT the full fruit's GT center or its size.
        center=np.median(points,axis=0) if len(points) else None
        world=world_from_optical[:3,:3]@center+world_from_optical[:3,3] if center is not None else None
        record=dict(instance_id=int(identity),key=row['key'],class_name=row['class_name'],
            plant=row['plant'],truss=row['truss'],color_stage=row.get('color_stage'),red_proxy=row.get('red_proxy'),
            visible_pixels=len(xs),bbox_xyxy=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)],
            visible_centroid_uv=[float(xs.mean()),float(ys.mean())],valid_depth_pixels=int(valid.sum()),
            median_depth_m=float(np.median(z)) if len(z) else None,
            visible_surface_median_optical_m=center.tolist() if center is not None else None,
            visible_surface_median_world_m=world.tolist() if world is not None else None)
        if row['class_name']=='rachis' and len(points)>=12:
            # Fit only the renderer-visible, positive-depth surface pixels.
            # This never reads authored skeletons, hidden mesh vertices, or
            # the complete fruit/branch centers used by the simulator.
            measured=points[(z>0)&np.isfinite(points).all(axis=1)]
            measured=(world_from_optical[:3,:3]@measured.T).T+world_from_optical[:3,3]
            measured=measured[np.isfinite(measured).all(axis=1)]
            if len(measured)>=12:
                centered=measured-measured.mean(axis=0)
                spatial_values,spatial_vectors=np.linalg.eigh(centered.T@centered/len(centered))
                spatial_noise=np.finfo(float).eps*max(1.,float(np.max(np.abs(measured)))**2)
                if float(spatial_values[-1])>spatial_noise:
                    long_axis=spatial_vectors[:,-1]
                    long_axis/=np.linalg.norm(long_axis)
                    record.update(visible_surface_long_axis_world=long_axis.tolist(),
                        visible_surface_long_axis_horizontal_fraction=float(np.clip(np.linalg.norm(long_axis[:2]),0.,1.)),
                        visible_surface_long_axis_variance_world_m2=np.maximum(spatial_values,0.).tolist(),
                        visible_surface_long_axis_method='unsigned world-XYZ PCA of the same visible depth-backprojected surface points; horizontal fraction is XY norm of unit longest axis')
                xy=measured[:,:2]-measured[:,:2].mean(axis=0)
                eigenvalues,eigenvectors=np.linalg.eigh(xy.T@xy/len(xy))
                major=float(eigenvalues[-1]);minor=max(0.,float(eigenvalues[0]))
                numerical_variance=np.finfo(float).eps*max(1.,float(np.max(np.abs(measured[:,:2])))**2)
                if major>numerical_variance:
                    axis=np.r_[eigenvectors[:,-1],0.]
                    axis/=np.linalg.norm(axis)
                    record.update(visible_surface_xy_axis_world=axis.tolist(),
                        visible_surface_xy_anisotropy=major/max(minor,np.finfo(float).eps*major),
                        visible_surface_xy_axis_points=len(measured),
                        visible_surface_xy_axis_method='unsigned world-XY PCA of visible depth-backprojected surface points; anisotropy is major/minor covariance eigenvalue ratio',
                        visible_surface_xy_axis_source='existing instance-mask pixels + aligned optical-Z depth + camera intrinsics/extrinsics; no hidden GT geometry')
        result.append(record)
    return result


def font(size):
    return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',size)


def overlays(rgb,depth,instance,semantic,objects,visible,timestamp):
    rgb=rgb.astype(np.uint8);recognition=rgb.astype(float).copy()
    values,inverse=np.unique(instance,return_inverse=True)
    colors=np.zeros((len(values),3));alphas=np.zeros(len(values))
    for i,identity in enumerate(values):
        if not identity:continue
        row=objects[int(identity)];kind=row['class_name']
        if kind not in ('fruit','main_stem','rachis','pedicel','peduncle','calyx','leaf'):continue
        color=FRUIT_COLORS[row['color_stage']] if kind=='fruit' else COLORS[kind]
        alpha=.23 if kind=='fruit' else .18 if kind in ('leaf','calyx') else .68
        colors[i]=color;alphas[i]=alpha
    alpha=alphas[inverse].reshape(*instance.shape,1)
    recognition=(1-alpha)*recognition+alpha*colors[inverse].reshape(*instance.shape,3)
    rec=Image.fromarray(recognition.astype(np.uint8));draw=ImageDraw.Draw(rec)
    for row in visible:
        if row['class_name']!='fruit' or row['visible_pixels']<6:continue
        x0,y0,x1,y1=row['bbox_xyxy'];color=FRUIT_COLORS[row['color_stage']]
        # Corners stay at the visible extent; do not fabricate hidden fruit.
        n=min(5,max(1,(x1-x0)//3),max(1,(y1-y0)//3))
        for x,dx in [(x0,n),(x1-1,-n)]:
            for y,dy in [(y0,n),(y1-1,-n)]:draw.line([(x,y+dy),(x,y),(x+dx,y)],fill=(235,235,235),width=1)
        u,v=row['visible_centroid_uv'];draw.rectangle((int(u)-1,int(v)-1,int(u)+1,int(v)+1),fill=color,outline=(15,15,15))
    valid=np.isfinite(depth)
    # Fixed *display* range across frames, raw sensor depth still .1..5m.
    # Nearby crop detail spans the palette; farther values saturate blue.
    t=np.clip((np.nan_to_num(depth,nan=2.)-.1)/1.9,0,1)
    anchors=np.array([[255,60,40],[250,210,30],[65,215,100],[35,180,235],[65,70,220]])
    scaled=t*4;i=np.minimum(scaled.astype(int),3);f=scaled-i
    heat=anchors[i]*(1-f[:,:,None])+anchors[i+1]*f[:,:,None]
    overlay=rgb.astype(float).copy();overlay[valid]=.30*overlay[valid]+.70*heat[valid]
    dep=Image.fromarray(overlay.astype(np.uint8));d=ImageDraw.Draw(dep)
    w,h=rec.size
    for x in range(w):
        u=x/(w-1)*4;j=min(int(u),3);v=u-j
        color=tuple((anchors[j]*(1-v)+anchors[j+1]*v).astype(int))
        d.line((x,h-9,x,h-1),fill=color)
    d.text((5,h-26),'0.1m near',font=font(11),fill='white',stroke_width=1,stroke_fill='black')
    d.text((w-75,h-26),'2m+ far',font=font(11),fill='white',stroke_width=1,stroke_fill='black')
    trip=Image.new('RGB',(w*3,h+30),(20,25,30));td=ImageDraw.Draw(trip)
    counts={k:sum(r['class_name']=='fruit' and r.get('color_stage')==k for r in visible) for k in FRUIT_COLORS}
    titles=[f'RGB | t={timestamp:.1f}s',f'SIM-GT visible | red {counts["red"]} / turning {counts["turning"]} / green {counts["green"]}', 'Depth overlay | display 0.1-2m / raw 0.1-5m']
    for n,(panel,title) in enumerate(zip([Image.fromarray(rgb),rec,dep],titles)):
        trip.paste(panel,(w*n,30));td.text((w*n+8,7),title,font=font(13),fill='white')
    return rec,dep,trip


class GroundTruthReadbackError(RuntimeError):
    """A retryable renderer payload failure, never an unknown background ID.

    The raw owned payload remains available for diagnostics.  ``details`` is
    deliberately small and JSON serializable so a bad texture cannot turn a
    terminal log into hundreds of thousands of numeric IDs.
    """
    def __init__(self,reason,details,*,snapshot=None):
        self.details=dict(details,reason=reason,retryable_readback=True)
        self.snapshot=snapshot
        self.raw=None if snapshot is None else snapshot.get('data')
        info={} if snapshot is None else snapshot.get('info',{})
        self.labels=info.get('idToLabels',{}) if isinstance(info,dict) else {}
        super().__init__('GT renderer readback: '+reason+
            '; missing IDs='+str(self.details.get('missing_label_count',0)))


def owned_gt_snapshot(data,expected_shape=None):
    """Own and validate one renderer label bundle without advancing physics."""
    if not isinstance(data,dict) or 'data' not in data:
        raise GroundTruthReadbackError('missing data bundle',dict(payload_type=type(data).__name__))
    # Replicator's default get_data may lend asynchronous annotator storage.
    # Also copy here so tests/callers cannot mutate an accepted snapshot later.
    raw=np.array(data['data'],copy=True)
    if raw.ndim==3 and raw.shape[-1]==1:raw=raw[:,:,0].copy()
    info=copy.deepcopy(data.get('info',{}));snapshot=dict(data=raw,info=info)
    details=dict(dtype=str(raw.dtype),shape=list(raw.shape),
                 expected_shape=None if expected_shape is None else list(expected_shape))
    if raw.dtype!=np.dtype(np.uint32):
        raise GroundTruthReadbackError('instance texture must be uint32',details,snapshot=snapshot)
    if raw.ndim!=2 or expected_shape is not None and raw.shape!=tuple(expected_shape):
        raise GroundTruthReadbackError('instance/depth shape mismatch',details,snapshot=snapshot)
    labels=info.get('idToLabels') if isinstance(info,dict) else None
    if not isinstance(labels,dict):
        raise GroundTruthReadbackError('missing renderer label map',details,snapshot=snapshot)
    values=np.unique(raw)
    missing=[int(identity) for identity in values if identity>1 and
             labels.get(str(int(identity)),labels.get(int(identity))) in (None,'BACKGROUND','UNLABELLED')]
    if missing:
        details.update(unique_instance_count=len(values),label_count=len(labels),
            missing_label_count=len(missing),missing_label_ids_sample=missing[:16])
        raise GroundTruthReadbackError('instance IDs are absent from this label bundle',details,snapshot=snapshot)
    return snapshot


class GroundTruthOverlay:
    def __init__(self,stage,product,output,assembly):
        import omni.replicator.core as rep
        self.meshes,self.objects=build_catalog(stage,assembly)
        self.annotator=rep.AnnotatorRegistry.get_annotator('instance_id_segmentation_fast',init_params={'colorize':False})
        self.annotator.attach(product);self.product=product
        self.catalog=dict(schema='s300-visible-sim-gt-v1',classes=CLASSES,objects=list(self.objects.values()),
            source='USD sourceName + authored displayColor; renderer visible instance IDs',
            visibility='Visible pixels only; no amodal masks; no RGB inference',
            maturity='Authored-color proxy, not biological ripeness or reachability',
            position='Depth-derived visible surface, not complete object center',
            depth_overlay_range_m=[.1,2.],depth_overlay_clipped_for_display_only=True,
            training_eligible=False,harvest_region_selected=False)
        (Path(output)/'gt_catalog.json').write_text(json.dumps(self.catalog,indent=2)+'\n')

    def read_snapshot(self,expected_shape=None):
        # The installed Replicator API explicitly requires ownership for data
        # that can outlive its asynchronous rendering buffers.
        data=self.annotator.get_data(device='cpu',do_array_copy=True)
        return owned_gt_snapshot(data,expected_shape)

    def save(self,directory,rgb,depth,K,pose,timestamp,*,snapshot=None,save_depth=False):
        data=(self.read_snapshot(expected_shape=depth.shape) if snapshot is None else
              owned_gt_snapshot(snapshot,depth.shape))
        raw=data['data'];labels=data['info']['idToLabels']
        if not hasattr(self,'_verified_label_hashes'):self._verified_label_hashes=set()
        # Persist the exact owned renderer evidence even if catalog resolution
        # fails. Such a mapping failure is strict, not a retry/background rule.
        evidence={'renderer_instance.npy':raw}
        if save_depth:evidence['depth.npy']=depth
        save_arrays(directory,evidence,labels=labels,
            label_pool=(directory.parent if directory.name.startswith('frame_') else directory)/'shared_labels',
            verified=self._verified_label_hashes)
        ids,classes,unresolved=resolve_pixels(raw,labels,self.meshes,self.objects)
        if unresolved:
            (directory/'gt_mapping_error.json').write_text(json.dumps(dict(
                reason='renderer identity absent from exact USD catalog',unresolved=unresolved,
                renderer_instance='renderer_instance.npy.gz',renderer_labels='renderer_labels.json',
                retryable_readback=False,training_eligible=False),indent=2)+'\n')
            raise RuntimeError('GT catalog mapping failed for '+str(len(unresolved))+
                               ' renderer IDs; see gt_mapping_error.json')
        visible=visible_objects(ids,depth,self.objects,np.asarray(K),pose)
        save_arrays(directory,{'instance.npy':ids,'semantic.npy':classes})
        (directory/'objects.json').write_text(json.dumps(dict(timestamp_s=timestamp,objects=visible,
            visible_only=True,harvest_region_selected=False),indent=2)+'\n')
        rec,dep,trip=overlays(rgb,depth,ids,classes,self.objects,visible,timestamp)
        rec.save(directory/'recognition.png');dep.save(directory/'depth_overlay.png');trip.save(directory/'analysis.png')
        return dict(semantic='semantic.npy.gz',instance='instance.npy.gz',array_encoding='gzip-npy',objects='objects.json',
                    recognition='recognition.png',depth_overlay='depth_overlay.png',analysis='analysis.png',
                    visible_objects=len(visible),visible_fruits=sum(x['class_name']=='fruit' for x in visible),
                    unresolved_renderer_ids=0)

    def close(self):self.annotator.detach(self.product)


def frame_durations(rows,fps):
    """Actual acquisition intervals in source order, plus one nominal tail.

    Extra cycle-boundary exposures make a fixed image-sequence frame rate
    incorrect. Never sort away malformed clocks or silently merge duplicates.
    """
    if isinstance(fps,bool) or not isinstance(fps,(int,float,np.number)) or not np.isfinite(fps) or fps<=0:
        raise ValueError('Positive finite nominal capture fps required')
    times=[]
    for row in rows:
        stamp=row.get('timestamp_s')
        if isinstance(stamp,bool) or not isinstance(stamp,(int,float,np.number)) or not np.isfinite(stamp) or stamp<0:
            raise ValueError('Finite nonnegative camera timestamp required')
        times.append(float(stamp))
    if any(after<=before for before,after in zip(times,times[1:])):
        raise ValueError('Camera timestamps must be strictly increasing without duplicates')
    return [after-before for before,after in zip(times,times[1:])]+([1./float(fps)] if times else [])


def write_review(output,fps):
    """Review already acquired frames only. No scene/physics execution."""
    output=Path(output)
    rows=[json.loads(line) for line in (output/'frames.jsonl').read_text().splitlines()]
    durations=frame_durations(rows,fps)
    if not rows:return {}
    video=output/'analysis.mp4'
    if video.exists():
        raise FileExistsError('Preserve the existing analysis.mp4 before regenerating its timestamp-correct review: '+str(video))
    images=[(output/Path(row['rgb']).parent/row['gt'].get('analysis','analysis.png')).resolve(strict=True) for row in rows]
    def quoted(path):
        text=str(path)
        if '\n' in text or '\r' in text:raise ValueError('Unsupported newline in captured frame path')
        return "'"+text.replace("'","'\\''")+"'"
    concat=output/'analysis.ffconcat'
    origin=float(rows[0]['timestamp_s'])
    # The concat demuxer's global clock is microseconds. Round cumulative
    # acquisition boundaries upward (at most1us each), rather than rounding
    # each interval and accumulating drift or displaying an exposure early.
    boundaries=[math.ceil((float(row['timestamp_s'])-origin)*1_000_000) for row in rows]
    boundaries.append(math.ceil((float(rows[-1]['timestamp_s'])-origin+durations[-1])*1_000_000))
    concat_durations=[(after-before)/1_000_000 for before,after in zip(boundaries,boundaries[1:])]
    lines=['ffconcat version 1.0']
    for path,duration in zip(images,concat_durations):
        # A high-resolution image time base avoids the default 25Hz demuxer
        # quantization. The display filter rounds each exposure UP to its next
        # 10Hz display tick, so a future observation is never shown early.
        lines += ['file '+quoted(path),'option framerate 1000000',f'duration {duration:.12f}']
    lines += ['file '+quoted(images[-1]),'option framerate 1000000']
    concat.write_text('\n'.join(lines)+'\n')
    error=None
    # ffconcat per-file options require a newer demuxer than Ubuntu's system
    # FFmpeg4.4. Reuse the encoder already bundled in the local vision env.
    # An explicit override permits other installations without downloading.
    bundled=sorted(Path('/root/isaaclab_env').glob('lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg*'))
    ffmpeg=os.environ.get('FARMILY_FFMPEG') or next(
        (str(path) for path in bundled if path.is_file() and os.access(path,os.X_OK)),None) or shutil.which('ffmpeg')
    if ffmpeg:
        try:
            subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-n','-threads','2',
                '-f','concat','-safe','0','-i',str(concat),
                '-vf','fps=fps=10:round=up','-c:v','libx264','-threads','2','-preset','fast','-crf','20',
                '-pix_fmt','yuv420p','-movflags','+faststart',str(video)],check=True,capture_output=True)
        except subprocess.CalledProcessError as exc:error=exc.stderr.decode(errors='replace')[-2000:]
    else:error='ffmpeg unavailable; original PNG frames remain available'
    items=[dict(time=r['timestamp_s'],phase=r['phase'],directory=Path(r['rgb']).parent.as_posix(),
                fruits=r['gt']['visible_fruits'],
                files=dict(analysis=r['gt'].get('analysis','analysis.png'),
                    recognition=r['gt'].get('recognition','recognition.png'),
                    depth_overlay=r['gt'].get('depth_overlay','depth_overlay.png'),rgb=Path(r['rgb']).name,
                    ids=array_path(output/Path(r['rgb']).parent/'instance.npy').name,
                    classes=array_path(output/Path(r['rgb']).parent/'semantic.npy').name,
                    depth=array_path(output/Path(r['rgb']).parent/'depth.npy').name),
                region=r['gt'].get('region'),selected_visible=r['gt'].get('visible_selected_count')) for r in rows]
    selected_frames=sum(bool(row['region']) for row in items)
    region_note=('핑크색 윤곽선은 기존 계획기가 채택한 수확 그룹 중 현재 카메라에 보이는 열매입니다. '
                 '그룹 선택이 기록된 프레임에만 표시하며, 가려지거나 보이지 않는 열매의 윤곽은 만들지 않습니다.'
                 if selected_frames else '이 기록에는 채택된 수확 영역이 없습니다.')
    video_html=('<video src="analysis.mp4" controls preload="metadata"></video><p><a href="analysis.mp4">'
                +html.escape(str(video))+'</a></p>') if video.exists() else '<p>MP4 unavailable; use the frame slider.</p>'
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>카메라 SIM-GT 분석</title><style>body{margin:24px auto;max-width:1480px;padding:0 20px;background:#101820;color:#e2edf2;font:16px/1.6 system-ui}a{color:#7cd1ff;overflow-wrap:anywhere}video,img{width:100%;height:auto}input{width:100%}h1{font-size:24px}.note{color:#b2c1ca}button{margin:5px;padding:8px 12px}pre{white-space:pre-wrap}</style>
<h1>RGB → 모델 GT 표시 → Depth overlay</h1><p>카메라에 보이는 픽셀만 모델의 객체 정보로 표시합니다. 영상 인식 모델을 실행하지 않습니다. 밝은 초록=주줄기/중심가지, 노랑=꼭지 가지, 옅은 초록=잎. 열매 색상은 모델에 지정된 빨강/주황·노랑/초록 분류입니다.</p>
VIDEO
<p id="caption"></p><input id="seek" type="range" min="0" value="0"><p><button id="analysis">3화면</button><button id="recognition">GT 표시만</button><button id="depth_overlay">깊이 표시만</button><button id="rgb">RGB만</button></p><img id="frame">
<p><a id="objects">보이는 객체·좌표 JSON</a> · <a id="ids">객체 ID NPY/GZ</a> · <a id="classes">종류 ID NPY/GZ</a> · <a id="depth">깊이(m) NPY/GZ</a> · <a id="camera">카메라 파라미터 JSON</a> · <a id="region">채택된 수확 영역 JSON</a> · <a href="gt_catalog.json">전체 ID 사전</a></p>
<p class="note">가려진 부분의 마스크는 채우지 않습니다. 박스는 보이는 픽셀의 외곽 범위입니다. 좌표는 깊이에서 계산한 보이는 표면의 중앙값이며 열매 전체 중심이 아닙니다. 픽셀 하나라도 보이는 열매는 원자료에 포함하고, 화면 박스는 6픽셀 이상만 표시합니다. 색상 구분은 실물 숙도·로봇 도달 가능성 판정이 아닙니다. REGION_NOTE</p>
<p class="note">깊이 시각화는 모든 프레임에서 0.1~2m 고정 색상 범위이며, 더 먼 값은 파랑으로 포화됩니다. 원본 깊이는 0.1~5m, float32 m이며 무효 값은 NaN입니다. 영상은 기록된 실제 촬영 시각 간격으로 1배속 재생합니다. 추가 촬영도 시간순으로 유지하며, 다음 촬영 전에는 직전 프레임을 유지합니다. 마지막 프레임은 명목 촬영 주기 1회만큼 유지합니다.</p>
<script>const rows=DATA;const $=id=>document.getElementById(id);let kind='analysis';function show(){const i=+$('seek').value,r=rows[i],base=r.directory+'/';$('caption').textContent=`${i+1}/${rows.length} · t=${r.time.toFixed(3)}s · ${r.phase} · 보이는 열매 ${r.fruits}개`+(r.region?` · 선택 영역 ${r.selected_visible}개`:'');$('frame').src=base+r.files[kind];for(const [id,file] of Object.entries({objects:'objects.json',ids:r.files.ids,classes:r.files.classes,depth:r.files.depth,camera:'camera.json'}))$(id).href=base+file;$('region').style.display=r.region?'':'none';if(r.region)$('region').href=base+r.region;}for(const k of ['analysis','recognition','depth_overlay','rgb'])$(k).onclick=()=>{kind=k;show()};$('seek').max=rows.length-1;$('seek').oninput=show;show();</script></html>'''
    (output/'index.html').write_text(page.replace('VIDEO',video_html).replace('REGION_NOTE',region_note).replace('DATA',json.dumps(items)))
    result=dict(index=str(output/'index.html'),video=str(video) if video.exists() else None,video_error=error,
                source='captured visible GT/RGB/depth, no new physics',frames=len(rows),capture_fps=fps,
                region_annotated_frames=selected_frames,
                playback_speed=1.,display_fps=10,source_timing='variable_acquisition_timestamps',
                original_timestamps_s=[float(row['timestamp_s']) for row in rows],
                source_frame_durations_s=durations,source_start_timestamp_s=float(rows[0]['timestamp_s']),
                source_end_timestamp_s=float(rows[-1]['timestamp_s'])+durations[-1],
                source_duration_s=sum(durations),concat_manifest=str(concat),
                concat_frame_durations_s=concat_durations,concat_timebase_hz=1_000_000,
                concat_timestamp_rounding='cumulative exposure boundaries rounded up; less than1us delay; no interval drift',
                encoder_executable=ffmpeg,
                display_resampling='hold previous captured frame; round new exposure up to next 10Hz display tick; no interpolation')
    (output/'review.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description='Build a review of existing mounted GT captures')
    parser.add_argument('output',type=Path);parser.add_argument('--fps',type=float,default=2.)
    args=parser.parse_args();print(json.dumps(write_review(args.output,args.fps),indent=2))
