"""Scope PhysX native replication/environment IDs to this GPU scene build.

GPU broadphase must reject co-located environments before allocating overlap
pairs. USD collision groups alone filter too late at large environment counts.
The original USD geometry and per-environment native contact paths are retained.
"""
from contextlib import contextmanager
import math


def environment_id_bits(num_envs):
    if not 1 <= num_envs <= 32768:
        raise ValueError('Native environment ID range exceeded')
    return max(4,math.ceil(math.log2(num_envs+1)))


@contextmanager
def native_environment_replication(enabled):
    if not enabled:
        yield
        return
    from isaacsim.core.cloner import Cloner
    from pxr import Sdf,UsdPhysics
    original=Cloner.clone
    def clone(cloner,*args,**kwargs):
        paths=kwargs.get('prim_paths',[])
        selected=kwargs.get('source_prim_path')=='/World/envs/env_0' and len(paths)>1
        if selected:
            kwargs.update(replicate_physics=True,enable_env_ids=True,
                          base_env_path='/World/envs',root_path='/World/envs/env_')
        result=original(cloner,*args,**kwargs)
        if selected:
            for prim in cloner._stage.Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    prim.CreateAttribute('physxScene:envIdInBoundsBitCount',Sdf.ValueTypeNames.Int).Set(environment_id_bits(len(paths)))
            print('[GPU REPLICATION]',len(paths),'environments, bounds bits',environment_id_bits(len(paths)),flush=True)
        return result
    Cloner.clone=clone
    try:
        yield
    finally:
        Cloner.clone=original
