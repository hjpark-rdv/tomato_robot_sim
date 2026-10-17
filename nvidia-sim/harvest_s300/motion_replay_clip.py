"""Select bounded replay samples from existing native timestamps, never rerun physics."""
import numpy as np


def plant_contact_caption(manifest, portable_metadata=None):
    """Prefer executed portable collision coverage over the older display USD."""
    if portable_metadata is not None and 'neighbor_obstacles' in portable_metadata:
        roots=portable_metadata['neighbor_obstacles']['roots']
        if roots:
            return f'FIXED NEIGHBOR PLANTS: {len(roots)}; OTHER BACKGROUND PLANTS NONCOLLIDING'
    if manifest.get('plant_interaction_variant',{}).get('mode')=='target_truss_and_own_stem_only':
        return 'NON-TARGET PLANT CONTACTS OFF'
    if manifest.get('elastic_neighbors'):
        return f'DYNAMIC PLANTS: {1+len(manifest["elastic_neighbors"])}'
    return ''


def select_replay_ticks(stamps, *, fps, speed, start_time=None, end_time=None):
    stamps=np.asarray(stamps,dtype=float)
    if (stamps.ndim!=1 or not len(stamps) or not np.isfinite(stamps).all() or
            len(stamps)>1 and np.any(np.diff(stamps)<=0)):
        raise ValueError('Replay requires finite, strictly increasing native timestamps')
    if not isinstance(fps,int) or fps<1 or not np.isfinite(speed) or speed<=0:
        raise ValueError('Replay fps and speed must be positive')
    start=float(stamps[0] if start_time is None else start_time)
    end=float(stamps[-1] if end_time is None else end_time)
    if not np.isfinite([start,end]).all() or start>end or start<stamps[0] or end>stamps[-1]:
        raise ValueError('Requested replay clip lies outside recorded timestamps or has reversed bounds')
    first=int(np.searchsorted(stamps,start,side='left'))
    last=int(np.searchsorted(stamps,end,side='right')-1)
    if first>last:raise ValueError('Requested clip contains no recorded native state')
    targets=np.arange(stamps[first],stamps[last]+1e-8,speed/fps)
    selection=np.minimum(np.searchsorted(stamps,targets),last)
    if not len(selection) or selection[-1]!=last:selection=np.r_[selection,last]
    if len(selection)<fps:selection=np.r_[selection,np.repeat(last,fps-len(selection))]
    return selection.astype(int)
