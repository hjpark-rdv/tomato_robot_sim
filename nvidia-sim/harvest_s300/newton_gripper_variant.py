"""Remove strictly buried CAD colliders while retaining the gripper exterior.

Every removed convex collider is contained in a retained convex collider on the
same rigid body, with identical contact parameters and a declared positive
clearance. Belts and other bodies are protected. This changes the collision
decomposition (and possibly deeply penetrating contact manifolds), never the
rigid-body exterior union, body dynamics, commands, or harvest executor.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial import ConvexHull

from newton_fruit_proxy import collision_hull_vertex_ids, geom_body_vertices, mesh_vertices
from newton_inputs import sha256


TOOL_BODIES = ("Tool_fixed", "Tool_L", "Tool_R")
CONTACT_FIELDS = ("geom_contype", "geom_conaffinity", "geom_condim", "geom_priority",
                  "geom_friction", "geom_solmix", "geom_solref", "geom_solimp",
                  "geom_margin", "geom_gap", "geom_surfacevel")
DEFAULT_CONTACT_GEOMS = ("s300_g0606", "s300_g0581", "s300_g0620")


def prepare_contact_proxy(source, output, *, geom_names=DEFAULT_CONTACT_GEOMS, max_outward_m=0.0001):
    """Reuse the established conservative mesh certificate on named outer parts.

    Belts and all unselected parts remain exact. The gripper opening defined by
    belt inner faces is unchanged, while these outer pieces may grow by the
    declared bound. This is independent from buried-collider removal.
    """
    from newton_fruit_proxy import prepare as prepare_convex_proxy
    model = mj.MjModel.from_binary_path(str(Path(source) / "input_model.mjb"))
    for name in geom_names:
        geom = model.geom(name).id
        body = model.body(int(model.geom_bodyid[geom])).name
        if body not in TOOL_BODIES:
            raise ValueError("Only named outer tool body meshes may be proxied; belts remain protected")
    return prepare_convex_proxy(source, output, geom_names=geom_names, role="gripper_outer_contact",
        max_outward_m=max_outward_m, budgets=(128,256,512,1024,2048,4096),
        compiled_hull_only=True, full_hull_fallback=True, support_precondition_power=.5)


def _json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def same_contact(model, first, second):
    return all(np.array_equal(getattr(model, field)[first], getattr(model, field)[second])
               for field in CONTACT_FIELDS)


def containment_audit(model, *, minimum_clearance_m=0.001, tool_bodies=TOOL_BODIES):
    """Finite vertices prove complete convex-shape containment in every pose.

    Both colliders share a rigid body. For each outer-hull normalized halfspace,
    every source inner vertex must lie at least minimum_clearance inside. This
    extends to the complete convex inner shape and all rigid-body transforms.
    Numerical acceptance never permits positive signed distances.
    """
    if not np.isfinite(minimum_clearance_m) or minimum_clearance_m <= 0:
        raise ValueError("A finite positive minimum clearance is required")
    if model.npair:
        raise ValueError("Explicit geom-pair remapping is not supported")
    if any("belt" in name.lower() for name in tool_bodies):
        raise ValueError("Belt body colliders must remain protected")
    selected = [g for g in range(model.ngeom)
                if model.body(int(model.geom_bodyid[g])).name in tool_bodies
                and model.geom_type[g] == mj.mjtGeom.mjGEOM_MESH
                and (model.geom_contype[g] | model.geom_conaffinity[g]) != 0]
    points, hulls, bounds = {}, {}, {}
    for geom in selected:
        mesh = int(model.geom_dataid[geom])
        # Audit the actual compiled collision graph. The render/mesh source can
        # contain vertices not retained by a limited-vertex collision hull.
        vertices = geom_body_vertices(model, geom)[collision_hull_vertex_ids(model, mesh)]
        hull = ConvexHull(vertices)
        points[geom], hulls[geom] = vertices[hull.vertices], hull
        bounds[geom] = (vertices.min(axis=0), vertices.max(axis=0))
    retained, removed = [], []
    # Descending volume ensures each removal witness is itself retained. Strict
    # positive clearance prevents equal-shape cycles and arbitrary tie removal.
    for inner in sorted(selected, key=lambda g: (-hulls[g].volume, model.geom(g).name)):
        witnesses = []
        for outer in retained:
            if model.geom_bodyid[inner] != model.geom_bodyid[outer] or not same_contact(model, inner, outer):
                continue
            if (np.any(bounds[inner][0] < bounds[outer][0] + minimum_clearance_m) or
                    np.any(bounds[inner][1] > bounds[outer][1] - minimum_clearance_m)):
                continue
            equations = hulls[outer].equations
            maximum = float(np.max(points[inner] @ equations[:, :3].T + equations[:, 3]))
            if maximum <= -minimum_clearance_m:
                witnesses.append((maximum, model.geom(outer).name, outer))
        if not witnesses:
            retained.append(inner)
            continue
        maximum, _, outer = min(witnesses)
        removed.append(dict(geom=model.geom(inner).name, source_geom_id=inner,
            body=model.body(int(model.geom_bodyid[inner])).name,
            retained_containing_geom=model.geom(outer).name,
            containing_source_geom_id=outer,
            source_collision_hull_vertices=len(points[inner]),
            source_mesh_vertices=int(model.mesh_vertnum[int(model.geom_dataid[inner])]),
            containing_hull_planes=len(hulls[outer].equations),
            max_signed_distance_m=maximum, certified_minimum_clearance_m=-maximum,
            identical_contact_parameters=True))
    return dict(schema="s300-gripper-buried-collider-proof-v1",
        minimum_clearance_m=minimum_clearance_m, selected_tool_bodies=list(tool_bodies),
        selected_source_mesh_colliders=len(selected), removed_colliders=len(removed),
        retained_tool_mesh_colliders=len(retained), removed=removed,
        exterior_union_unchanged=True, protected_belt_colliders=True,
        proof="Each removed complete convex hull lies strictly inside one retained hull on the same body with identical contact parameters; all removed vertices satisfy every normalized containing halfspace. Convexity and shared rigid transform extend containment to every point and pose.",
        caveat="Contact manifolds and force contributions can differ during deep interpenetration even though the occupied exterior union is unchanged.")


def verify_invariants(old, new, removed_names, *, keep_disabled_geom_ids=False):
    checks = {}
    def same(label, left, right):
        a, b = np.asarray(left), np.asarray(right)
        checks[label] = bool(a.shape == b.shape and np.array_equal(a, b))
    removed = set(removed_names)
    for field in ("nbody", "njnt", "nq", "nv", "nu", "neq", "nexclude", "npair", "nsite", "ntendon"):
        same(field, getattr(old, field), getattr(new, field))
    layout = set() if keep_disabled_geom_ids else {"body_geomadr", "body_geomnum", "body_bvhadr", "body_bvhnum"}
    for field in dir(old):
        if ((field.startswith(("body_", "jnt_", "dof_", "actuator_", "eq_", "site_", "tendon_"))
             and field not in layout) or field in ("qpos0", "qpos_spring", "exclude_signature")):
            value = getattr(old, field)
            if isinstance(value, np.ndarray): same(field, value, getattr(new, field))
    old_names = {old.geom(g).name: g for g in range(old.ngeom)}
    new_names = {new.geom(g).name: g for g in range(new.ngeom)}
    same("retained_geom_names", sorted(set(old_names) if keep_disabled_geom_ids else set(old_names) - removed), sorted(new_names))
    for name in sorted(new_names):
        a, b = old_names[name], new_names[name]
        for field in dir(old):
            if field.startswith("geom_") and field != "geom_dataid" and isinstance(getattr(old, field), np.ndarray):
                if keep_disabled_geom_ids and name in removed and field in ("geom_contype", "geom_conaffinity"):
                    same(f"geom:{name}.{field}.disabled", 0, getattr(new, field)[b])
                    continue
                same(f"geom:{name}.{field}", getattr(old, field)[a], getattr(new, field)[b])
        if old.geom_type[a] == mj.mjtGeom.mjGEOM_MESH:
            ma, mb = int(old.geom_dataid[a]), int(new.geom_dataid[b])
            same(f"geom:{name}.mesh_name", old.mesh(ma).name, new.mesh(mb).name)
            same(f"geom:{name}.mesh_vertices", mesh_vertices(old, ma), mesh_vertices(new, mb))
            for field in ("mesh_pos", "mesh_quat", "mesh_scale"):
                same(f"geom:{name}.{field}", getattr(old, field)[ma], getattr(new, field)[mb])
            for field, adr, count in (("mesh_face", "mesh_faceadr", "mesh_facenum"),
                    ("mesh_normal", "mesh_normaladr", "mesh_normalnum")):
                oa, on = int(getattr(old, adr)[ma]), int(getattr(old, count)[ma])
                na, nn = int(getattr(new, adr)[mb]), int(getattr(new, count)[mb])
                same(f"geom:{name}.{field}", getattr(old, field)[oa:oa+on], getattr(new, field)[na:na+nn])
            graphs = []
            for model, mesh in ((old, ma), (new, mb)):
                adr = int(model.mesh_graphadr[mesh])
                size = 2 + 3*int(model.mesh_graph[adr]) + 6*int(model.mesh_graph[adr+1]) if adr >= 0 else 0
                graphs.append(model.mesh_graph[adr:adr+size] if adr >= 0 else np.array([], dtype=int))
            same(f"geom:{name}.compiled_collision_graph", *graphs)
    for field in dir(old.opt):
        if field.startswith("_"): continue
        value = getattr(old.opt, field)
        if isinstance(value, (np.ndarray, int, float, bool, np.integer, np.floating)):
            same("option." + field, value, getattr(new.opt, field))
    for field in ("meanmass", "meaninertia"):
        same("stat." + field, getattr(old.stat, field), getattr(new.stat, field))
    failed = [name for name, passed in checks.items() if not passed]
    if failed: raise ValueError("Protected physical fields changed: " + repr(failed[:25]))
    return dict(passed=True, exact_numeric_comparison=True, checks=checks,
        removed_geom_names=sorted(removed), keep_disabled_geom_ids=keep_disabled_geom_ids,
        intentional_changes=("Strictly buried same-body collider contype/conaffinity disabled; native geom/mesh IDs, vertices and BVH storage retained." if keep_disabled_geom_ids else
            "Strictly buried same-body mesh colliders removed; compiler geometry/mesh storage addresses and body BVH are remapped."))


def prepare(source, output, *, minimum_clearance_m=0.001, keep_disabled_geom_ids=False):
    source, output = Path(source).resolve(strict=True), Path(output).resolve()
    if source == output or output.exists(): raise FileExistsError("Use a new output folder")
    started = time.perf_counter()
    provenance = json.loads((source / "inputs.json").read_text())
    for name, evidence in provenance["input_files"].items():
        if evidence.get("sha256") and sha256(source / name) != evidence["sha256"]:
            raise ValueError("Source SHA mismatch: " + name)
    source_files = {name: sha256(source / name) for name in
                    ("input_scene.xml", "input_model.mjb", "initial_state.npz", "commands.npz", "source_export.json")}
    old = mj.MjModel.from_binary_path(str(source / "input_model.mjb"))
    audit = containment_audit(old, minimum_clearance_m=minimum_clearance_m)
    if not audit["removed"]: raise ValueError("No strictly contained gripper colliders found")
    removed = {row["geom"] for row in audit["removed"]}
    tree = ET.parse(source / "input_scene.xml")
    compiler = tree.getroot().find("compiler")
    if compiler is None or compiler.get("inertiafromgeom") != "false":
        raise ValueError("Explicit inertials with inertiafromgeom=false required")
    for parent in tree.getroot().iter():
        for child in list(parent):
            if child.tag == "geom" and child.get("name") in removed:
                if keep_disabled_geom_ids:
                    child.set("contype", "0"); child.set("conaffinity", "0")
                else: parent.remove(child)
    output.mkdir(parents=True, exist_ok=False)
    tree.write(output / "input_scene.xml", encoding="unicode")
    if keep_disabled_geom_ids:
        # The established adapter uploads this authoritative MJB. Its XML
        # describes the same active collision policy but discardvisual may
        # prune masked geom storage during Newton's metadata-only import.
        new = mj.MjModel.from_binary_path(str(source / "input_model.mjb"))
        for name in removed:
            geom = new.geom(name).id
            new.geom_contype[geom] = 0
            new.geom_conaffinity[geom] = 0
    else:
        new = mj.MjModel.from_xml_path(str(output / "input_scene.xml"))
    new.dof_damping[:] = old.dof_damping
    new.dof_length[:] = old.dof_length
    for geom in range(new.ngeom):
        new.geom_surfacevel[geom] = old.geom_surfacevel[old.geom(new.geom(geom).name).id]
    new.flg_surfacevel = old.flg_surfacevel
    for field in dir(old.opt):
        if field.startswith("_"): continue
        value = getattr(old.opt, field)
        if isinstance(value, np.ndarray): getattr(new.opt, field)[:] = value
        elif isinstance(value, (int, float, bool, np.integer, np.floating)): setattr(new.opt, field, value)
    invariants = verify_invariants(old, new, removed, keep_disabled_geom_ids=keep_disabled_geom_ids)
    initial = dict(np.load(source / "initial_state.npz", allow_pickle=False))
    a, b = mj.MjData(old), mj.MjData(new)
    for data in (a, b):
        for field in ("qpos", "qvel", "ctrl", "eq_active"): getattr(data, field)[:] = initial[field]
    mj.mj_forward(old, a); mj.mj_forward(new, b)
    if not np.array_equal(a.xpos, b.xpos) or not np.array_equal(a.xquat, b.xquat):
        raise ValueError("Initial body transforms changed")
    metadata = json.loads((source / "source_export.json").read_text())
    variant = dict(mode="buried_gripper_convex_colliders", source_inputs=str(source),
        upstream_physics_model_variant=metadata.get("physics_model_variant"),
        collision_decomposition_changed=True, exterior_union_unchanged=True,
        physical_model_changed=True, exact_trajectory_equivalence_claimed=False,
        original_nv=old.nv, variant_nv=new.nv, source_ngeom=old.ngeom, variant_ngeom=new.ngeom,
        source_nmesh=old.nmesh, variant_nmesh=new.nmesh, retained_body_mass_inertia_joints=True,
        source_active_geoms=int(np.count_nonzero(old.geom_contype | old.geom_conaffinity)),
        variant_active_geoms=int(np.count_nonzero(new.geom_contype | new.geom_conaffinity)),
        native_disabled_geom_storage_preserved=keep_disabled_geom_ids,
        belt_surface_geometry_unchanged=True, initial_body_transforms_bit_exact=True,
        containment=audit, invariant_proof=invariants, training_eligible=False, harvest_success=None)
    metadata["physics_model_variant"] = variant
    for row in metadata.get("geometry", []):
        if row.get("name") in removed:
            row["collision"] = False
            row["variant_removal_reason"] = "Strictly buried inside retained same-body collider"
    mj.mj_saveModel(new, str(output / "input_model.mjb"))
    for name in ("initial_state.npz", "commands.npz", "sampled_plan.json"):
        if (source / name).is_file(): shutil.copy2(source / name, output / name)
    _json(output / "source_export.json", metadata)
    _json(output / "gripper_variant_proof.json", variant)
    report = copy.deepcopy(provenance)
    files = ("input_scene.xml", "input_model.mjb", "initial_state.npz", "commands.npz", "source_export.json", "gripper_variant_proof.json")
    report["input_files"] = {name: dict(path=str(output / name), sha256=sha256(output / name)) for name in files}
    report["physics_model_variant"] = variant
    report["model_counts"] = dict(nbody=new.nbody, nq=new.nq, nv=new.nv, ngeom=new.ngeom, nu=new.nu, neq=new.neq)
    report["physical_steps"] = 0
    report["variant_prepare_seconds"] = time.perf_counter() - started
    report["source_prepared_files_sha256"] = source_files
    report.setdefault("code_sha256", {})[str(Path(__file__).resolve())] = sha256(__file__)
    report["source_prepared_files_unchanged"] = all(sha256(source / name) == digest for name, digest in source_files.items())
    if not report["source_prepared_files_unchanged"]: raise ValueError("Source input changed")
    _json(output / "inputs.json", report)
    return variant


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--minimum-clearance-m", type=float, default=0.001)
    parser.add_argument("--keep-disabled-geom-ids", action="store_true",
                        help="Disable buried collisions in authoritative MJB without compacting native geom/mesh IDs")
    parser.add_argument("--outer-contact-proxy", action="store_true",
                        help="Independent named outer-tool convex proxy experiment; does not remove buried colliders")
    parser.add_argument("--max-outward-m", type=float, default=.0001)
    parser.add_argument("--geoms", nargs="+", default=DEFAULT_CONTACT_GEOMS)
    args = parser.parse_args()
    if args.outer_contact_proxy:
        if args.keep_disabled_geom_ids: parser.error("Outer proxy and buried-mask experiments must remain independent")
        result = prepare_contact_proxy(args.inputs, args.output, geom_names=args.geoms, max_outward_m=args.max_outward_m)
        summary = dict(output=str(args.output), source_collision_vertices=result["source_vertices"],
                       proxy_collision_vertices=result["proxy_vertices"],
                       max_outward_bound_m=max(row["compiled_outward_bound_m"] for row in result["colliders"]))
    else:
        result = prepare(args.inputs, args.output, minimum_clearance_m=args.minimum_clearance_m,
                         keep_disabled_geom_ids=args.keep_disabled_geom_ids)
        summary = dict(output=str(args.output), removed=result["containment"]["removed_colliders"],
                       source_geoms=result["source_ngeom"], variant_geoms=result["variant_ngeom"],
                       retained_nv=result["variant_nv"])
    print(json.dumps(summary, indent=2))
