"""Add fixed collision proxies for visual-only neighboring tomato trusses.

This does not change the active target plant, contact permissions, or plant
materials.  It converts the already-authored background truss placements in
scene.json into fixed MuJoCo obstacles so a planner/rollout cannot treat
visible neighboring fruit/rachis/pedicel/peduncle as empty space.

Leaves remain visual-only by design in this tool.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

def nums(values):
    return " ".join(format(float(v), ".10g") for v in np.asarray(values).ravel())


COLLISION_PREFIX = "neighbor_truss_collision_"
COLLISION_CONTYPE = 32
COLLISION_CONAFFINITY = 15
REQUIRED_KINDS = ("fruit", "rachis", "pedicel", "peduncle")


def obstacle_kind(body_name):
    """Map authored truss bodies to the user's non-target obstacle classes."""
    name = body_name or ""
    if "Tomato_" in name:
        return "fruit"
    if "Rachis" in name:
        return "rachis"
    if "Pedicel" in name or name.startswith("Attachment_"):
        return "pedicel"
    if "Peduncle" in name:
        return "peduncle"
    return None


def collision_template(model, data, origin):
    """Extract active truss collision geoms in the y_deg=0 attachment frame."""
    origin = np.asarray(origin, dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("Invalid attachment origin")
    rows = []
    for gid in range(model.ngeom):
        if not (int(model.geom_contype[gid]) or int(model.geom_conaffinity[gid])):
            continue
        body = model.body(int(model.geom_bodyid[gid])).name or ""
        kind = obstacle_kind(body)
        if kind is None:
            continue
        name = model.geom(gid).name or f"#geom_{gid}"
        gtype = int(model.geom_type[gid])
        xpos = np.asarray(data.geom_xpos[gid], dtype=float)
        xmat = np.asarray(data.geom_xmat[gid], dtype=float).reshape(3, 3)
        if gtype == int(mj.mjtGeom.mjGEOM_CAPSULE):
            radius = float(model.geom_size[gid, 0])
            half = float(model.geom_size[gid, 1])
            axis = xmat[:, 2] * half
            rows.append(dict(kind=kind, source_geom=name, source_body=body,
                             shape="capsule",
                             endpoints_m=np.asarray([xpos-axis-origin, xpos+axis-origin]).tolist(),
                             radius_m=radius))
        elif gtype == int(mj.mjtGeom.mjGEOM_MESH):
            mid = int(model.geom_dataid[gid])
            start = int(model.mesh_vertadr[mid]); count = int(model.mesh_vertnum[mid])
            fstart = int(model.mesh_faceadr[mid]); fcount = int(model.mesh_facenum[mid])
            verts = np.asarray(model.mesh_vert[start:start+count], dtype=float)
            faces = np.asarray(model.mesh_face[fstart:fstart+fcount], dtype=int)
            world = verts @ xmat.T + xpos - origin
            rows.append(dict(kind=kind, source_geom=name, source_body=body,
                             shape="mesh", vertices_m=world.tolist(), faces=faces.tolist()))
        else:
            raise ValueError(f"Unsupported neighboring truss collider {name}: type={gtype}")
    kinds = {r["kind"] for r in rows}
    missing = sorted(set(REQUIRED_KINDS) - kinds)
    if missing:
        raise ValueError(f"Truss collider template missing obstacle classes: {missing}")
    if not rows:
        raise ValueError("No neighboring truss collision geometry extracted")
    return rows


def _canonical_variant(placement):
    source = Path(placement["source_glb"]).resolve()
    payload = dict(source_glb=str(source),
                   source_sha256=placement.get("source_sha256"),
                   remove_fruits=sorted(int(x) for x in placement.get("remove_fruits", [])),
                   fruit_offsets={str(k): list(v) for k, v in sorted(
                       placement.get("fruit_offsets", {}).items(), key=lambda kv: int(kv[0]))},
                   rachis_stiffness_scale=float(placement.get("rachis_stiffness_scale", 1.0)),
                   truss_scale=float(placement.get("truss_scale", .5)))
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode()).hexdigest()[:16], payload


def _build_variant_template(placement, temp_root):
    from build_glb_physics import build as build_truss
    key, spec = _canonical_variant(placement)
    source = Path(spec["source_glb"])
    if not source.is_file():
        raise FileNotFoundError(f"Missing source GLB for obstacle template: {source}")
    if spec["source_sha256"]:
        actual = hashlib.sha256(source.read_bytes()).hexdigest()
        if actual != spec["source_sha256"]:
            raise ValueError(f"Source GLB hash mismatch: {source}")
    part = Path(temp_root) / key
    offsets = {int(k): v for k, v in spec["fruit_offsets"].items()}
    build_truss(source, part, y_deg=0., segment=4, stem_fraction=.5,
                remove_fruits=spec["remove_fruits"], fruit_offsets=offsets,
                rachis_stiffness_scale=spec["rachis_stiffness_scale"],
                truss_scale=spec["truss_scale"])
    model = mj.MjModel.from_binary_path(str(part / "model.mjb"))
    data = mj.MjData(model); mj.mj_forward(model, data)
    meta = json.loads((part / "build.json").read_text())
    rows = collision_template(model, data, meta["attachment_world_m"])
    return key, spec, rows


def _install_mesh_assets(asset, key, rows):
    result = []
    for index, row in enumerate(rows):
        entry = copy.deepcopy(row)
        if row["shape"] == "mesh":
            mesh_name = f"{COLLISION_PREFIX}mesh_{key}_{index:03d}"
            if asset.find(f"mesh[@name='{mesh_name}']") is None:
                ET.SubElement(asset, "mesh", name=mesh_name,
                              vertex=nums(np.asarray(row["vertices_m"], dtype=float)),
                              face=" ".join(map(str, np.asarray(row["faces"], dtype=int).ravel())))
            entry["mesh_asset"] = mesh_name
        result.append(entry)
    return result


def append_fixed_colliders(world, template_rows, attachment_world, rotation,
                           plant_id, truss_index):
    """Append fixed-world obstacle geoms and return their inventory."""
    attachment_world = np.asarray(attachment_world, dtype=float)
    if attachment_world.shape != (3,) or not np.isfinite(attachment_world).all():
        raise ValueError("Invalid neighboring truss attachment")
    if not isinstance(rotation, Rotation):
        raise TypeError("rotation must be scipy Rotation")
    quat = rotation.as_quat()[[3, 0, 1, 2]]
    inventory = []
    for index, row in enumerate(template_rows):
        kind = row["kind"]
        name = f"{COLLISION_PREFIX}{kind}_p{int(plant_id):02d}_t{int(truss_index):02d}_g{index:03d}"
        attrs = dict(name=name, contype=str(COLLISION_CONTYPE),
                     conaffinity=str(COLLISION_CONAFFINITY), density="0",
                     group="3", rgba=".2 .5 1 0", friction=".5 .005 .0001",
                     condim="3", margin="0", gap="0", solref=".004 1",
                     solimp=".99 .999 .001")
        if row["shape"] == "capsule":
            local = np.asarray(row["endpoints_m"], dtype=float)
            endpoints = rotation.apply(local) + attachment_world
            attrs.update(type="capsule", fromto=nums(endpoints),
                         size=str(float(row["radius_m"])))
        elif row["shape"] == "mesh":
            attrs.update(type="mesh", mesh=row["mesh_asset"],
                         pos=nums(attachment_world), quat=nums(quat))
        else:
            raise ValueError(f"Unknown obstacle shape {row['shape']}")
        ET.SubElement(world, "geom", **attrs)
        inventory.append(dict(name=name, kind=kind, plant_id=int(plant_id),
                              truss_index=int(truss_index),
                              source_geom=row["source_geom"],
                              source_body=row["source_body"],
                              shape=row["shape"]))
    return inventory


def _expected_trusses(metadata):
    expected = []
    for plant in metadata.get("placements", []):
        pid = int(plant["id"])
        for ti, truss in enumerate(plant.get("trusses", [])):
            expected.append((pid, ti, truss))
    return expected


def coverage_from_names(metadata, geom_names, require_neighbor_stems=False,
                        require_gutter=False):
    """Fail-closed coverage accounting from scene metadata and actual geom names."""
    names = set(geom_names)
    missing = []
    expected = _expected_trusses(metadata)
    for pid, ti, _ in expected:
        for kind in REQUIRED_KINDS:
            prefix = f"{COLLISION_PREFIX}{kind}_p{pid:02d}_t{ti:02d}_"
            if not any(n.startswith(prefix) for n in names):
                missing.append(dict(type="neighbor_truss", plant_id=pid,
                                    truss_index=ti, kind=kind))
    if require_neighbor_stems:
        for plant in metadata.get("placements", []):
            pid = int(plant["id"])
            for segment in range(16):
                name = f"neighbor_stem_collision_{pid:02d}_{segment:02d}"
                if name not in names:
                    missing.append(dict(type="neighbor_stem", plant_id=pid,
                                        segment=segment))
    gutter_count = sum(n.startswith("gutter_collision_") for n in names)
    if require_gutter and gutter_count != 18:
        missing.append(dict(type="gutter", expected=18, found=gutter_count))
    counts = {kind: sum(n.startswith(f"{COLLISION_PREFIX}{kind}_") for n in names)
              for kind in REQUIRED_KINDS}
    return dict(passed=not missing,
                expected_neighbor_trusses=len(expected),
                truss_collision_geom_counts=counts,
                neighbor_stem_required=bool(require_neighbor_stems),
                gutter_required=bool(require_gutter),
                gutter_colliders=gutter_count,
                leaves_collision=False,
                leaves_policy="visual_only_not_part_of_this_obstacle_contract",
                missing=missing)


def _neighbor_layout_metadata(scene, layout=None):
    scene = Path(scene).resolve()
    scene_meta = json.loads((scene / "scene.json").read_text())
    if layout is not None:
        layout_path = Path(layout).resolve()
        layout_meta = json.loads((layout_path / "scene.json").read_text())
    elif "neighbor_placements" in scene_meta:
        layout_path = scene
        layout_meta = dict(placements=scene_meta["neighbor_placements"],
                           neighbors=len(scene_meta["neighbor_placements"]),
                           physics_added=False)
    elif "neighbors" in scene_meta and scene_meta.get("physics_added") is False:
        layout_path = scene
        layout_meta = scene_meta
    else:
        raise ValueError("Neighbor layout is ambiguous; pass --layout pointing to preview_neighbor_plants scene")
    if not isinstance(layout_meta.get("placements"), list):
        raise ValueError("Neighbor layout has no placements list")
    return scene_meta, layout_meta, layout_path


def add_neighbor_truss_obstacles(scene, output, layout=None):
    """Add fixed colliders for every visual neighboring truss recorded by the layout."""
    scene = Path(scene).resolve(); output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Output exists: {output}")
    scene_meta, metadata, layout_path = _neighbor_layout_metadata(scene, layout)
    expected = _expected_trusses(metadata)
    if not expected:
        raise ValueError("scene.json contains no neighboring truss placements")
    tree = ET.parse(scene / "model.xml"); root = tree.getroot()
    asset = root.find("asset"); world = root.find("worldbody")
    if any((g.get("name") or "").startswith(COLLISION_PREFIX) for g in world.findall("geom")):
        raise ValueError("Neighbor truss obstacles already present")
    source_model = mj.MjModel.from_binary_path(str(scene / "model.mjb"))
    cache = {}; inventories = []; variant_specs = {}
    with tempfile.TemporaryDirectory(prefix="neighbor_truss_obstacles_") as temp:
        for pid, ti, placement in expected:
            key, spec = _canonical_variant(placement)
            if key not in cache:
                built_key, built_spec, rows = _build_variant_template(placement, temp)
                if built_key != key:
                    raise AssertionError("Variant identity changed while building")
                cache[key] = _install_mesh_assets(asset, key, rows)
                variant_specs[key] = built_spec
            plant = next(p for p in metadata["placements"] if int(p["id"]) == pid)
            yaw = float(plant["yaw_deg"])
            y_deg = float(placement["y_deg"])
            rotation = Rotation.from_euler("z", yaw, degrees=True) * Rotation.from_euler("z", y_deg, degrees=True)
            inventories.extend(append_fixed_colliders(
                world, cache[key], placement["attachment_world_m"], rotation, pid, ti))
    output.mkdir(parents=True)
    for node in root.iter():
        if node.get("file") and not Path(node.get("file")).is_absolute():
            node.set("file", str((scene / node.get("file")).resolve()))
    tree.write(output / "model.xml", encoding="unicode")
    new_model = mj.MjModel.from_xml_path(str(output / "model.xml"))
    mj.mj_saveModel(new_model, str(output / "model.mjb"))
    if (new_model.nq, new_model.nv, new_model.nbody, new_model.nu) != (
            source_model.nq, source_model.nv, source_model.nbody, source_model.nu):
        raise ValueError("Adding fixed obstacles changed robot/plant dynamics topology")
    for attr in ("body_mass", "body_inertia", "dof_damping", "jnt_stiffness"):
        np.testing.assert_array_equal(getattr(source_model, attr), getattr(new_model, attr))
    if (scene / "reference.json").is_file():
        shutil.copy2(scene / "reference.json", output / "reference.json")
    names = [new_model.geom(i).name or f"#geom_{i}" for i in range(new_model.ngeom)]
    coverage = coverage_from_names(metadata, names)
    if not coverage["passed"]:
        raise ValueError(f"Incomplete neighboring truss collision coverage: {coverage['missing'][:4]}")
    source_sha = hashlib.sha256((scene / "model.mjb").read_bytes()).hexdigest()
    output_sha = hashlib.sha256((output / "model.mjb").read_bytes()).hexdigest()
    record = copy.deepcopy(scene_meta)
    record.update(parent_scene=str(scene), neighbor_layout=str(layout_path),
                  neighbor_placements=copy.deepcopy(metadata["placements"]),
                  parent_model_sha256=source_sha,
                  model_sha256=output_sha,
                  background_truss_physics="fixed collision proxies; no background bending",
                  background_truss_collision_geoms=len(inventories),
                  background_truss_collision_kinds=list(REQUIRED_KINDS),
                  leaves_collision=False,
                  validation={"scene_screen_passed": False,
                              "note": "Collision coverage changed; rerun initial screen before planning/physics"})
    (output / "scene.json").write_text(json.dumps(record, indent=2))
    report = dict(schema="neighbor_truss_obstacles_v1", source_scene=str(scene),
                  neighbor_layout=str(layout_path),
                  source_model_sha256=source_sha, output_model_sha256=output_sha,
                  policy=dict(non_target_fruit="obstacle", non_target_rachis="obstacle",
                              non_target_pedicel="obstacle", non_target_peduncle="obstacle",
                              main_stem="handled_by_neighbor_stem_collision_stage",
                              gutter="handled_by_gutter_collision_stage",
                              leaves="visual_only_not_in_obstacle_contract"),
                  variants=variant_specs, inventory=inventories, coverage=coverage)
    (output / "truss_obstacles.json").write_text(json.dumps(report, indent=2))
    return report


def audit_scene(scene, require_neighbor_stems=False, require_gutter=False, layout=None):
    scene = Path(scene).resolve()
    _, metadata, _ = _neighbor_layout_metadata(scene, layout)
    model = mj.MjModel.from_binary_path(str(scene / "model.mjb"))
    names = [model.geom(i).name or f"#geom_{i}" for i in range(model.ngeom)]
    result = coverage_from_names(metadata, names, require_neighbor_stems, require_gutter)
    result.update(scene=str(scene),
                  model_sha256=hashlib.sha256((scene / "model.mjb").read_bytes()).hexdigest())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--layout", type=Path, help="preview_neighbor_plants scene providing background placements")
    parser.add_argument("--audit", action="store_true")
    parser.add_argument("--require-neighbor-stems", action="store_true")
    parser.add_argument("--require-gutter", action="store_true")
    args = parser.parse_args()
    if args.audit:
        result = audit_scene(args.scene, args.require_neighbor_stems, args.require_gutter, args.layout)
        print(json.dumps(result, indent=2))
        return 0 if result["passed"] else 2
    if args.output is None:
        parser.error("--output is required unless --audit is used")
    result = add_neighbor_truss_obstacles(args.scene, args.output, args.layout)
    print(json.dumps({k: v for k, v in result.items() if k != "inventory"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
