"""Exact active-plant truss scope for rail-side harvesting.

The selection is a placement policy, not a reachability/success certificate.
It retains every eligible fruit's existing path and native break joint identity.
All target selection uses SIM-GT; visual perception is not implied.
"""
import re
import numpy as np


def truss_key(name):
    match=re.fullmatch(r'(Truss_\d+)_Fruit_\d+',str(name))
    if not match:raise ValueError('Unknown active fruit truss identity: '+str(name))
    return match.group(1)


def _active_fruits(manifest):
    fruits=manifest['plant']['fruits'];root=manifest['plant']['root']
    for key in ('name','path','joint'):
        if len({f[key] for f in fruits})!=len(fruits):raise ValueError('Duplicate active fruit '+key)
    for f in fruits:
        truss_key(f['name'])
        if not f['path'].startswith(root+'/') or f['path'].rsplit('/',1)[-1]!=f['name']:
            raise ValueError('Fruit path does not belong to the active plant: '+f['name'])
        center=np.asarray(f['center'],float)
        if center.shape!=(3,) or not np.isfinite(center).all():raise ValueError('Nonfinite active fruit center')
    return fruits


def _placement_rows(farm_metadata,plant_id):
    # Prefer the rail inventory measured on the actual USD, including the
    # active plant override. The old covered metadata remains a supported input.
    if isinstance(farm_metadata,list):
        candidates=[r for r in farm_metadata if r.get('plant_id')==plant_id]
    elif 'trusses' in farm_metadata:
        candidates=[r for r in farm_metadata['trusses'] if r.get('plant_id')==plant_id]
    else:
        plants=farm_metadata.get('neighbor_placements',farm_metadata.get('placements',[]))
        matches=[r for r in plants if r.get('id')==plant_id]
        if len(matches)!=1:raise ValueError('Source plant metadata is missing or ambiguous')
        p=matches[0]
        candidates=[dict(plant_id=plant_id,truss_index=i,root_position_world_m=p['position'],
                         gutter_center_x=p['gutter_center_x']) for i,_ in enumerate(p['trusses'])]
    if not candidates:raise ValueError('Source plant metadata is missing')
    indices=[r['truss_index'] for r in candidates]
    if len(set(indices))!=len(indices):raise ValueError('Duplicate source truss metadata')
    return {int(r['truss_index']):r for r in candidates}


def scoped_trusses(manifest,farm_metadata,*,rail_x_m=0.):
    """Select near-side stem trusses whose centroid also faces the rail aisle."""
    if not np.isfinite(rail_x_m):raise ValueError('Finite rail X is required')
    source=manifest['plant']['source_plant'];match=re.search(r'/Plant_(\d+)$',source)
    if not match:raise ValueError('Missing exact source plant identity')
    plant_id=int(match.group(1));placements=_placement_rows(farm_metadata,plant_id)
    fruits=_active_fruits(manifest);keys=sorted({truss_key(f['name']) for f in fruits});rows=[]
    for key in keys:
        index=int(key.split('_')[1])
        if index not in placements:raise ValueError('Active truss is missing from source metadata: '+key)
        p=placements[index];root=np.asarray(p.get('root_position_world_m',p.get('root_position')),float)
        gutter=float(p['gutter_center_x'])
        if root.shape!=(3,) or not np.isfinite(root).all() or not np.isfinite(gutter):
            raise ValueError('Nonfinite source plant/gutter placement')
        side=float(np.sign(rail_x_m-gutter))
        if side==0:raise ValueError('Rail and gutter X cannot coincide for near-side classification')
        selected=[f for f in fruits if truss_key(f['name'])==key]
        if 'fruit_count' in p and p['fruit_count']!=len(selected):raise ValueError('Source/active truss fruit count mismatch')
        centers=np.array([f['center'] for f in selected]);centroid=centers.mean(0)
        stem_near=bool((root[0]-gutter)*side>0)
        centroid_near=bool((centroid[0]-gutter)*side>0)
        if 'rail_near' in p and bool(p['rail_near'])!=stem_near:
            raise ValueError('Source inventory and root/gutter side calculation disagree')
        reason=('source_stem_opposite_gutter_side' if not stem_near else
                'fruit_centroid_behind_gutter' if not centroid_near else None)
        rows.append(dict(truss_key=key,truss_index=index,plant_id=plant_id,
                         source_plant=source,root_position_world_m=root.tolist(),gutter_center_x=gutter,
                         rail_x_m=float(rail_x_m),stem_rail_near=stem_near,
                         fruit_centroid_rail_near=centroid_near,eligible=reason is None,exclusion_reason=reason,
                         fruit_centroid_world_m=centroid.tolist(),fruit_count=len(selected),
                         fruit_names=[f['name'] for f in selected],
                         reachability='unknown_pending_finite_planning_and_physics'))
    return dict(schema='s300-active-plant-near-trusses-v1',source_plant=source,plant_id=plant_id,
                trusses=rows,eligible_truss_keys=[r['truss_key'] for r in rows if r['eligible']],
                eligible_fruit_count=sum(r['fruit_count'] for r in rows if r['eligible']),
                selection='Near-side source stem and fruit centroid on rail side of the gutter',
                training_eligible=False,harvest_success=None)


def eligible_truss_fruits(manifest,scope):
    """Return existing records only; preserve exact native body/joint mapping."""
    if scope['source_plant']!=manifest['plant']['source_plant']:raise ValueError('Target scope belongs to another plant')
    keys=set(scope['eligible_truss_keys']);fruits=[f for f in _active_fruits(manifest) if truss_key(f['name']) in keys]
    if len(fruits)!=scope['eligible_fruit_count']:raise ValueError('Target scope fruit count differs from active manifest')
    return fruits


def group_truss_fruits(selected_group,all_fruits):
    """Exact truss records for PCA/break filtering, without crossing truss names."""
    names=selected_group['fruits'];by_name={f['name']:f for f in all_fruits}
    if not names or len(set(names))!=len(names) or any(n not in by_name for n in names):
        raise ValueError('Selected group contains unknown or duplicate fruits')
    keys={truss_key(n) for n in names}
    if len(keys)!=1:raise ValueError('Selected group crosses truss boundaries')
    key=next(iter(keys));return [f for f in all_fruits if truss_key(f['name'])==key]


def select_scoped_group(fruits,breaks,body_paths,poses,attempts,max_attempts=2,order='lowest-first',
                        single_fruit_when_remaining=0,rejected_groups=()):
    """Reuse existing group selection independently inside each exact truss."""
    from truss_sequence import select_group,validate_max_fruit_attempts,validate_single_fruit_when_remaining,validated_rejected_groups
    max_attempts=validate_max_fruit_attempts(max_attempts)
    single_fruit_when_remaining=validate_single_fruit_when_remaining(single_fruit_when_remaining)
    rejected=validated_rejected_groups(fruits,rejected_groups)
    groups=[]
    for key in sorted({truss_key(f['name']) for f in fruits}):
        subset=[f for f in fruits if truss_key(f['name'])==key]
        group=select_group(subset,breaks,body_paths,poses,attempts,max_attempts=max_attempts,order=order,
                           single_fruit_when_remaining=single_fruit_when_remaining,
                           rejected_groups=[g for g in rejected if all(truss_key(name)==key for name in g)])
        if group is not None:
            group['truss_key']=key;groups.append(group)
    if not groups:return None
    sign=1 if order=='lowest-first' else -1
    return min(groups,key=lambda g:(attempts.get(g['fruits'][0],0),
        sign*g['measured_centers_world_m'][0][2],g['fruits'][0]))
