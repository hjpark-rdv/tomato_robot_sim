#!/usr/bin/env python3
"""Prepare a read-only USD import diagnostic for Isaac's official Newton stage.

This derivative strips display-only geometry and mirrors the existing portable
MuJoCo adapter's per-mesh convex-hull approximation. It is deliberately *not*
advertised as an unchanged PhysX stage or a qualified harvest implementation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def prepare(source: Path, output: Path) -> dict:
    from pxr import Usd, UsdGeom, UsdPhysics

    source = source.resolve(strict=True)
    before_hash = sha256(source)
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    stage_path = output / "native_physics_hulls.usda"
    stage = Usd.Stage.CreateNew(str(stage_path))
    stage.GetRootLayer().subLayerPaths = [str(source)]
    # Stage metadata is not inherited via every composition path uniformly.
    original = Usd.Stage.Open(str(source))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(original))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(original))
    disabled_visuals, hull_overrides, enabled_colliders = [], [], []
    bodies, joints, semantics = [], [], []
    for prim in list(stage.Traverse()):
        # Deactivating a visual Gprim also expires its material GeomSubsets.
        if not prim:
            continue
        path = str(prim.GetPath())
        is_body = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        if is_body:
            bodies.append(path)
        if prim.IsA(UsdPhysics.Joint):
            joints.append({"path": path, "type": prim.GetTypeName(),
                           "enabled": bool(UsdPhysics.Joint(prim).GetJointEnabledAttr().Get())})
        special = {attr.GetName(): str(attr.Get()) for attr in prim.GetAttributes()
                   if attr.HasAuthoredValueOpinion() and
                   ("surfaceVelocity" in attr.GetName() or
                    attr.GetName() in ("physics:breakForce", "physics:breakTorque"))}
        if special:
            semantics.append({"path": path, "attributes": special})
        is_collision = (prim.HasAPI(UsdPhysics.CollisionAPI) and
                        bool(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()))
        if is_collision:
            enabled_colliders.append(path)
            if prim.IsA(UsdGeom.Mesh):
                api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                old = api.GetApproximationAttr().Get()
                api.CreateApproximationAttr().Set("convexHull")
                if old != "convexHull":
                    hull_overrides.append({"path": path, "from": old, "to": "convexHull"})
        elif prim.IsA(UsdGeom.Gprim) and not is_body:
            # Never remove bodies or joints to make the diagnostic import pass.
            prim.SetActive(False)
            disabled_visuals.append(path)
    # Large instanced display forests may contain no traversable Gprims at the
    # instance root. Prune only top-level Xform subtrees proven to contain no
    # body, joint or enabled collider. Their original full-farm layer remains
    # available to the display process and is never edited here.
    protected = bodies + [x["path"] for x in joints] + enabled_colliders
    disabled_subtrees = []
    world = stage.GetPrimAtPath("/World")
    if world:
        for prim in world.GetChildren():
            path = str(prim.GetPath())
            if (prim.IsA(UsdGeom.Xform) and
                    not any(p == path or p.startswith(path + "/") for p in protected)):
                prim.SetActive(False)
                disabled_subtrees.append(path)
    stage.GetRootLayer().Save()
    report = {
        "schema": "s300-native-newton-source-derivative-v1",
        "source": str(source), "source_sha256_before": before_hash,
        "source_sha256_after": sha256(source), "stage": str(stage_path),
        "stage_sha256": sha256(stage_path),
        "display_geometry_deactivated": disabled_visuals,
        "display_subtrees_deactivated": disabled_subtrees,
        "mesh_hull_overrides": hull_overrides,
        "enabled_colliders_preserved": enabled_colliders,
        "source_bodies_preserved": bodies, "source_joints_preserved": joints,
        "semantics_requiring_adapter": semantics,
        "counts": {"source_bodies": len(bodies), "source_joints": len(joints),
                   "enabled_joints": sum(x["enabled"] for x in joints),
                   "enabled_colliders": len(enabled_colliders),
                   "display_geometries_deactivated": len(disabled_visuals),
                   "mesh_hull_overrides": len(hull_overrides)},
        "training_eligible": False, "harvest_success": None,
        "qualification": "import diagnostic only; no harvest controller, preload, belt or break adapter",
        "changes": [
            "Display-only non-body Gprims are inactive in this physics-only derivative; original full-farm stage is preserved.",
            "Enabled meshes use per-mesh convex hulls, matching the portable adapter's documented approximation, not the original PhysX convex decomposition.",
            "Bodies, enabled joints, materials, contact properties and enabled collider transforms are retained from the source USD.",
            "Native USD parser representation differs from portable MJB; no model equivalence or throughput claim is made.",
        ],
    }
    if report["source_sha256_before"] != report["source_sha256_after"]:
        raise RuntimeError("Source USD changed")
    (output / "preparation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args.source, args.output)
    print(json.dumps({"stage": result["stage"], "counts": result["counts"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
