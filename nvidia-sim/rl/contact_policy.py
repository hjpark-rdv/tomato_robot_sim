"""Contact-generation settings; collider geometry/rest offsets stay unchanged."""
POLICIES=('legacy','speculative','offset1','speculative_offset1','offset2')


def apply(stage, root, policy):
    if policy not in POLICIES:raise ValueError('Unknown contact policy: '+policy)
    if policy=='legacy':return dict(policy=policy,shapes=0,bodies=0)
    from pxr import Usd,UsdPhysics,PhysxSchema
    shapes=bodies=0
    offset={'offset1':.001,'speculative_offset1':.001,'offset2':.002}.get(policy)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and 'speculative' in policy:
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateEnableSpeculativeCCDAttr(True);bodies+=1
        if offset is not None and prim.HasAPI(UsdPhysics.CollisionAPI):
            PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateContactOffsetAttr(offset);shapes+=1
    return dict(policy=policy,shapes=shapes,bodies=bodies,contact_offset_m=offset)
