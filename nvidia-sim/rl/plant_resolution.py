"""Optional coarse skeleton and an explicit policy for main-stem appendages."""
import numpy as np

RESOLUTIONS = ('full', 'light')
# 70 mm around the target centre covers the 55 mm ring and nearby pedicels.
# Rod collision shapes outside this region remain present and contact-enabled too.
PROTECTED_RADIUS_M = .07


def appendage_policy(resolution, policy=None):
    if resolution not in RESOLUTIONS:raise ValueError('Unknown plant resolution: '+str(resolution))
    if policy is None:policy='ignore' if resolution=='light' else 'keep'
    if policy not in ('keep','ignore'):raise ValueError('Unknown appendage collision policy: '+str(policy))
    if resolution=='full' and policy!='keep':raise ValueError('full preserves original collisions; use light to ignore appendages')
    return policy


def collision_inventory(stage, root):
    """Inspect authored enabled plant colliders, excluding robot and greenhouse."""
    from collections import Counter
    from pxr import Usd, UsdPhysics
    counts=Counter();types=Counter()
    for branch in ('ElasticPlant','HarvestableStem'):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root+'/'+branch)):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:continue
            path=str(prim.GetPath());name=prim.GetName()
            if branch=='ElasticPlant':kind='main_appendage' if '/Appendage_' in path else 'elastic_rod'
            elif '/Harvestables/' in path:
                kind='fruit' if name=='FruitCollider' else 'fruit_pedicel' if name=='PedicelCollider' else 'other_harvestable'
            else:kind='other_static_plant'
            counts[kind]+=1;types[kind+':'+prim.GetTypeName()]+=1
    return dict(enabled_total=sum(counts.values()),by_category=dict(counts),by_shape_type=dict(types))


def joint_layout(resolution, name, nodes, target_center, target_name, world_root=False):
    """Keep near-target hinges; replace distant hinges by fixed connections.

    Remaining distant springs represent their serial group. Scaling both K and D
    by retained/original count preserves constant-moment small-angle compliance
    and the D/K ratio, not general bending modes or dynamic equivalence.
    """
    if resolution not in RESOLUTIONS:
        raise ValueError('Unknown plant resolution: '+str(resolution))
    nodes=np.asarray(nodes,dtype=float)
    count=len(nodes)-1
    available=np.ones(count,dtype=bool)
    if world_root:available[0]=False
    free=available.copy()
    protected=np.zeros(count,dtype=bool)
    scales=np.ones(count)
    if resolution=='light':
        a,b=nodes[:-1],nodes[1:]
        ab=b-a
        t=np.clip(np.sum((np.asarray(target_center)-a)*ab,axis=1)/np.maximum(np.sum(ab*ab,axis=1),1e-20),0,1)
        near=np.linalg.norm(a+t[:,None]*ab-target_center,axis=1)<=PROTECTED_RADIUS_M
        protected=near & available
        if name=='TRUSS_Pedicel_proximal_'+target_name[-2:]:protected=available.copy()
        free=protected.copy()
        # Each contiguous distant group retains its first hinge, then one in four.
        distant=np.flatnonzero(available & ~protected)
        groups=np.split(distant,np.flatnonzero(np.diff(distant)>1)+1)
        for group in groups:
            if not len(group):continue
            retained=group[::4]
            free[retained]=True
            scales[retained]=len(retained)/len(group)
    return dict(free=free,protected=protected,spring_scale=scales)
