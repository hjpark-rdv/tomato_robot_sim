"""Bounded read-only geometry evidence after an invalid native warmup.

This never clears the failure or restores broken plant topology. Actual cooked
convex points and authored-only static triangle fallbacks remain distinct.
"""
import numpy as np


def _serializable(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {key:_serializable(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):return [_serializable(item) for item in value]
    return value


def capture_initialization_geometry(reader,events,poses,*,max_pairs=3):
    """Keep worst exact eligible pairs; failed native queries stay inconclusive."""
    if not 1<=max_pairs<=3:raise ValueError('Initialization geometry evidence budget must be1..3pairs')
    poses=np.asarray(poses,float)
    if poses.ndim!=2 or poses.shape[1]!=7 or not np.isfinite(poses).all():
        raise ValueError('Finite measured native body transforms required')
    pairs={}
    for index,event in enumerate(events):
        a,b=event['a'],event['b']
        if not reader.eligible(a,b):continue
        key=tuple(sorted((a,b)))
        if key not in pairs or event['separation_m']<pairs[key][1]['separation_m']:
            pairs[key]=(index,event)
    records=[]
    selected=sorted(pairs.values(),key=lambda item:item[1]['separation_m'])[:max_pairs]
    for index,event in selected:
        row=dict(raw_contact_index=index,raw_event=dict(event),colliders=[])
        for path in (event['a'],event['b']):
            shape=dict(path=path)
            try:
                data=reader.cooked(path)
                shape['representation']=_serializable(data)
                body=data.get('body')
                if body is not None:
                    shape['measured_owner_pose_world_xyzw']=poses[reader.body_ids[body]].tolist()
                shape['native_cooked_convex_verified']=data.get('kind')=='native_cooked_convex'
                shape['native_cooked_triangle_verified']=data.get('native_cooked_triangle_verified',False)
            except Exception as error:
                shape.update(inconclusive=True,error=type(error).__name__+': '+str(error))
            row['colliders'].append(shape)
        records.append(row)
    return dict(schema='s300-invalid-native-initialization-geometry-v1',records=records,
        eligible_pair_count=len(pairs),capture_budget=max_pairs,
        measurement_scope='Measured body pose AFTER native warmup, before trial; raw callback may describe an earlier pre-solve iteration',
        failure_overridden=False,physics_continued=False,training_eligible=False,harvest_success=None)
