"""Diagnostic equivalent joint authoring variants; never raise break thresholds."""
from pxr import Sdf, UsdPhysics


def install_joint_variant(variant):
    import dataset_scene
    original=dataset_scene.make_joint

    def author(*args, **kwargs):
        joint=original(*args, **kwargs)
        prim=joint.GetPrim()
        with Sdf.ChangeBlock():
            if variant=='reverse-fixed':
                bodies=[joint.GetBody0Rel().GetTargets(),joint.GetBody1Rel().GetTargets()]
                positions=[joint.GetLocalPos0Attr().Get(),joint.GetLocalPos1Attr().Get()]
                rotations=[joint.GetLocalRot0Attr().Get(),joint.GetLocalRot1Attr().Get()]
                joint.GetBody0Rel().SetTargets(bodies[1]);joint.GetBody1Rel().SetTargets(bodies[0])
                joint.GetLocalPos0Attr().Set(positions[1]);joint.GetLocalPos1Attr().Set(positions[0])
                joint.GetLocalRot0Attr().Set(rotations[1]);joint.GetLocalRot1Attr().Set(rotations[0])
            elif variant=='locked-d6':
                prim.SetTypeName('PhysicsJoint')
                for axis in ('transX','transY','transZ','rotX','rotY','rotZ'):
                    limit=UsdPhysics.LimitAPI.Apply(prim,axis)
                    limit.CreateLowAttr(1.);limit.CreateHighAttr(-1.)
            else:
                raise ValueError(variant)
        return UsdPhysics.Joint(prim)
    dataset_scene.make_joint=author
