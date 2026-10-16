"""Opt-in conservative convex fruit meshes with an explicit outward-error proof.

For an interior center c and selected source-hull vertices p, the proxy vertices
are c+s*(p-c). s is chosen from every source vertex and every inner-hull plane,
so the proxy contains the original. Each proxy vertex is within (s-1)*R of a
point in the original convex hull. Convexity extends this bound to the complete
proxy. Compilation roundoff/transform error is measured and added to the bound.

This changes collision approximation. It never changes mass, joint dynamics,
friction, contact margin, gap, control, initial state or the harvest executor.
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
from scipy.spatial import ConvexHull, QhullError, cKDTree

from newton_inputs import sha256


DEFAULT_BUDGETS = (128, 256, 512, 1024)


def _json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def sphere_directions(count: int) -> np.ndarray:
    if count < 10:
        raise ValueError("At least10 support directions required")
    remaining = count - 6
    index = np.arange(remaining, dtype=float)
    z = 1 - 2 * (index + 0.5) / remaining
    angle = index * (np.pi * (3 - np.sqrt(5)))
    radius = np.sqrt(np.maximum(0, 1 - z * z))
    return np.vstack((np.eye(3), -np.eye(3),
                      np.column_stack((radius * np.cos(angle), radius * np.sin(angle), z))))


def conservative_proxy(points, max_outward_m=0.0001, *, budgets=DEFAULT_BUDGETS,
                       source_hull_vertex_ids=None, containment_guard_m=1e-8,
                       compile_reserve_m=1e-7):
    """Return vertices/faces and a checkable finite-vertex convexity certificate."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 4 or not np.isfinite(points).all():
        raise ValueError("Finite3D source vertices required")
    if (not np.isfinite([max_outward_m, compile_reserve_m, containment_guard_m]).all() or
            not 0 < compile_reserve_m < max_outward_m or containment_guard_m <= 0):
        raise ValueError("Positive error limit, compile reserve and containment guard required")
    budgets = tuple(int(x) for x in budgets)
    if not budgets or any(x < 10 for x in budgets) or tuple(sorted(set(budgets))) != budgets:
        raise ValueError("Strictly increasing support budgets of at least10 required")
    try:
        if source_hull_vertex_ids is None:
            source_ids = ConvexHull(points).vertices
        else:
            source_ids = np.asarray(source_hull_vertex_ids, dtype=np.int64)
            if source_ids.ndim != 1 or len(source_ids) < 4 or np.any(source_ids < 0) or np.any(source_ids >= len(points)):
                raise ValueError("Invalid source collision hull vertex indices")
            source_ids = np.unique(source_ids)
        candidates = points[source_ids]
        center = candidates.mean(axis=0)
        radius = float(np.linalg.norm(points - center, axis=1).max())
        attempts = []
        for budget in budgets:
            directions = sphere_directions(budget)
            local = np.unique(np.argmax(candidates @ directions.T, axis=0))
            ids = source_ids[local]
            selected = points[ids]
            inner = ConvexHull(selected)
            normals = inner.equations[:, :3]
            offsets = -(normals @ center + inner.equations[:, 3])
            if np.min(offsets) <= 0:
                attempts.append(dict(support_budget=budget, rejected="center outside selected hull"))
                continue
            support = np.max((points - center) @ normals.T, axis=0)
            scale_required = float(max(1.0, np.max(support / offsets)))
            scale = scale_required + containment_guard_m / float(offsets.min())
            bound = (scale - 1.0) * radius
            entry = dict(support_budget=budget, selected_vertices=int(len(selected)),
                         scale_required=scale_required, scale_applied=scale,
                         analytic_outward_bound_m=float(bound))
            attempts.append(entry)
            if bound + compile_reserve_m > max_outward_m:
                continue
            proxy = center + scale * (selected - center)
            outer = ConvexHull(proxy)
            containment = float(np.max(points @ outer.equations[:, :3].T + outer.equations[:, 3]))
            if containment >= 0:
                raise ValueError("Conservative proxy failed strict source containment")
            # Qhull simplices are not necessarily oriented for OBJ volume use.
            faces = outer.simplices.copy()
            for face in faces:
                v = proxy[face]
                normal = np.cross(v[1] - v[0], v[2] - v[0])
                if np.dot(normal, v.mean(axis=0) - center) < 0:
                    face[1], face[2] = face[2], face[1]
            certificate = dict(source_vertex_count=len(points),
                source_collision_hull_vertices=len(source_ids), proxy_vertex_count=len(proxy),
                support_budget=budget, source_vertex_indices=ids.tolist(), center_m=center.tolist(),
                source_radius_m=radius, scale_required=scale_required, scale_applied=scale,
                analytic_outward_bound_m=float(bound), compile_reserve_m=compile_reserve_m,
                maximum_outward_error_m=max_outward_m,
                source_containment_max_signed_distance_m=containment,
                numerical_containment_guard_m=containment_guard_m, attempts=attempts,
                proof="Allsourcevertices satisfy outerhull halfspaces; proxyvertices lie within(s-1)*R of selectedsourcehullvertices; convexity extends the bound to the entireproxy.")
            return proxy, faces, certificate
    except QhullError as exc:
        raise ValueError("Source/proxy hull must be nondegenerate3D") from exc
    raise ValueError("No proxy satisfies outward bound within support budgets: " + json.dumps(attempts))


def collision_hull_vertex_ids(model, mesh):
    """Decode the documented MuJoCo mesh_graph vert_globalid record."""
    address = int(model.mesh_graphadr[mesh])
    if address < 0:
        raise ValueError("Source mesh has no compiled convex collision graph")
    count = int(model.mesh_graph[address])
    ids = model.mesh_graph[address + 2 + count:address + 2 + 2 * count].copy()
    if len(ids) != count:
        raise ValueError("Incomplete source convex collision graph")
    return ids


def mesh_vertices(model, mesh):
    start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
    return model.mesh_vert[start:start + count].astype(np.float64)


def geom_body_vertices(model, geom):
    if model.geom_type[geom] != mj.mjtGeom.mjGEOM_MESH:
        raise ValueError("Mesh collider required")
    matrix = np.zeros(9)
    mj.mju_quat2Mat(matrix, model.geom_quat[geom])
    return mesh_vertices(model, int(model.geom_dataid[geom])) @ matrix.reshape(3, 3).T + model.geom_pos[geom]


def verify_compiled_proxy(source_body_vertices, ideal_proxy_body_vertices, compiled_body_vertices,
                           analytic_bound_m, max_outward_m):
    actual = np.asarray(compiled_body_vertices, dtype=float)
    source = np.asarray(source_body_vertices, dtype=float)
    ideal = np.asarray(ideal_proxy_body_vertices, dtype=float)
    if not all(np.isfinite(x).all() for x in (source, ideal, actual)):
        raise ValueError("Nonfinite compiled geometry")
    hull = ConvexHull(actual)
    signed = float(np.max(source @ hull.equations[:, :3].T + hull.equations[:, 3]))
    displacement = float(cKDTree(ideal).query(actual)[0].max())
    bound = float(analytic_bound_m + displacement)
    if signed >= 0:
        raise ValueError("Compiled proxy no longer encloses every source vertex")
    if bound > max_outward_m:
        raise ValueError("Compiled proxy exceeds declared outward-error bound")
    return dict(compiled_source_containment_max_signed_distance_m=signed,
                compiled_vertex_to_ideal_max_distance_m=displacement,
                compiled_outward_bound_m=bound, source_enclosed=True,
                outward_bound_passed=True, compiled_proxy_vertices=len(actual))


def _physical_invariants(old, new, changed_geoms):
    checks = {}
    def same(name, a, b):
        a, b = np.asarray(a), np.asarray(b)
        checks[name] = bool(a.shape == b.shape and np.array_equal(a, b))
    for name in ("nbody", "njnt", "nq", "nv", "nu", "neq", "nexclude", "ngeom", "nmesh", "npair", "nsite", "ntendon"):
        same(name, getattr(old, name), getattr(new, name))
    for name in dir(old):
        if name.startswith(("body_", "jnt_", "dof_", "actuator_", "eq_", "site_", "tendon_")) or name in ("qpos0", "qpos_spring", "exclude_signature"):
            if name in ("body_bvhadr", "body_bvhnum"):
                continue
            value = getattr(old, name)
            if isinstance(value, np.ndarray):
                same(name, value, getattr(new, name))
    unchanged = np.array([g for g in range(old.ngeom) if g not in changed_geoms], dtype=int)
    geometry_fields = {"geom_pos", "geom_quat", "geom_size", "geom_rbound", "geom_aabb", "geom_sameframe"}
    for name in dir(old):
        if name.startswith("geom_"):
            value = getattr(old, name)
            if isinstance(value, np.ndarray):
                ids = unchanged if name in geometry_fields else slice(None)
                same(name, value[ids], getattr(new, name)[ids])
    changed_meshes = {int(old.geom_dataid[g]) for g in changed_geoms}
    for mesh in range(old.nmesh):
        if mesh in changed_meshes:
            continue
        for name in ("mesh_pos", "mesh_quat", "mesh_scale", "mesh_extrema"):
            same(f"mesh{mesh}.{name}", getattr(old, name)[mesh], getattr(new, name)[mesh])
        same(f"mesh{mesh}.vertices", mesh_vertices(old, mesh), mesh_vertices(new, mesh))
        for name, address, count in (("mesh_face", "mesh_faceadr", "mesh_facenum"),
                                     ("mesh_normal", "mesh_normaladr", "mesh_normalnum"),
                                     ("mesh_facenormal", "mesh_faceadr", "mesh_facenum")):
            a, n = int(getattr(old, address)[mesh]), int(getattr(old, count)[mesh])
            b, m = int(getattr(new, address)[mesh]), int(getattr(new, count)[mesh])
            same(f"mesh{mesh}.{name}", getattr(old, name)[a:a + n], getattr(new, name)[b:b + m])
        graph_rows = []
        for model in (old, new):
            a = int(model.mesh_graphadr[mesh])
            size = 2 + 3 * int(model.mesh_graph[a]) + 6 * int(model.mesh_graph[a + 1]) if a >= 0 else 0
            graph_rows.append(model.mesh_graph[a:a + size] if a >= 0 else np.array([], dtype=int))
        same(f"mesh{mesh}.collision_graph", *graph_rows)
    for name in dir(old.opt):
        if name.startswith("_"):
            continue
        value = getattr(old.opt, name)
        if isinstance(value, (np.ndarray, int, float, bool, np.integer, np.floating)):
            same("option." + name, value, getattr(new.opt, name))
    for name in ("meanmass", "meaninertia"):
        same("stat." + name, getattr(old.stat, name), getattr(new.stat, name))
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise ValueError("Unexpected physical field change: " + repr(failed))
    return dict(passed=True, checks=checks,
                intentional_changes="Only target-fruit mesh vertices, faces and derived geom frames/bounds are changed; body-frame enclosure is separately certified.")


def _write_obj(path, vertices, faces):
    with path.open("x") as stream:
        for v in vertices:
            stream.write("v " + " ".join(format(float(x), ".17g") for x in v) + "\n")
        for face in faces:
            stream.write("f " + " ".join(str(int(x) + 1) for x in face) + "\n")


def prepare(source, output, *, max_outward_m=0.0001, budgets=DEFAULT_BUDGETS):
    """Create the existing immutable prepared-input format, without physics steps."""
    source, output = Path(source).resolve(strict=True), Path(output).resolve()
    if source == output or output.exists():
        raise FileExistsError("Use a new output folder: " + str(output))
    started = time.perf_counter()
    provenance = json.loads((source / "inputs.json").read_text())
    for name, evidence in provenance["input_files"].items():
        if evidence.get("sha256") and sha256(source / name) != evidence["sha256"]:
            raise ValueError("Prepared source SHA mismatch: " + name)
    source_hashes = {name: sha256(source / name) for name in
                     ("input_scene.xml", "input_model.mjb", "initial_state.npz", "commands.npz", "source_export.json")}
    old = mj.MjModel.from_binary_path(str(source / "input_model.mjb"))
    initial = dict(np.load(source / "initial_state.npz", allow_pickle=False))
    metadata = json.loads((source / "source_export.json").read_text())
    targets = set(metadata.get("fruits", []))
    if not targets:
        raise ValueError("Existing fruit identity list required")
    tree = ET.parse(source / "input_scene.xml")
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None or compiler.get("inertiafromgeom") != "false":
        raise ValueError("Explicit source inertials with inertiafromgeom=false required")
    asset = root.find("asset")
    if asset is None:
        raise ValueError("Mesh asset element required")
    assets = {e.get("name"): e for e in asset.findall("mesh")}
    geoms = {e.get("name"): e for e in root.iter("geom") if e.get("name")}
    selected = [g for g in range(old.ngeom) if old.body(int(old.geom_bodyid[g])).name in targets
                and (old.geom_contype[g] | old.geom_conaffinity[g]) != 0]
    if {old.body(int(old.geom_bodyid[g])).name for g in selected} != targets:
        raise ValueError("Every target fruit requires an active collider")
    if any(old.geom_type[g] != mj.mjtGeom.mjGEOM_MESH for g in selected):
        raise ValueError("This opt-in experiment only replaces fruit mesh colliders")
    selected_meshes = [int(old.geom_dataid[g]) for g in selected]
    if len(set(selected_meshes)) != len(selected_meshes):
        raise ValueError("Each selected fruit collider must own a unique mesh")
    if any(sum(old.geom_dataid == mesh) != 1 for mesh in selected_meshes):
        raise ValueError("Shared target mesh would change another collider")
    output.mkdir(parents=True, exist_ok=False)
    meshes = output / "proxy_meshes"
    meshes.mkdir()
    rows, ideals, changed = [], {}, set(selected)
    for geom in selected:
        name, mesh = old.geom(geom).name, int(old.geom_dataid[geom])
        old_asset_name = old.mesh(mesh).name
        if name not in geoms or old_asset_name not in assets:
            raise ValueError("Missing named XML source geometry: " + name)
        points = mesh_vertices(old, mesh)
        proxy, faces, certificate = conservative_proxy(points, max_outward_m, budgets=budgets,
            source_hull_vertex_ids=collision_hull_vertex_ids(old, mesh))
        obj = meshes / (name + ".obj")
        _write_obj(obj, proxy, faces)
        old_element = assets[old_asset_name]
        new_name = old_asset_name + "_conservative_fruit_proxy"
        new_element = ET.Element("mesh", name=new_name, file=str(obj), scale="1 1 1", maxhullvert="-1")
        index = list(asset).index(old_element)
        asset.remove(old_element)
        asset.insert(index, new_element)
        element = geoms[name]
        element.set("mesh", new_name)
        element.set("pos", " ".join(format(float(x), ".17g") for x in old.geom_pos[geom]))
        for attribute in ("euler", "axisangle", "xyaxes", "zaxis"):
            element.attrib.pop(attribute, None)
        element.set("quat", " ".join(format(float(x), ".17g") for x in old.geom_quat[geom]))
        matrix = np.zeros(9)
        mj.mju_quat2Mat(matrix, old.geom_quat[geom])
        ideals[geom] = proxy @ matrix.reshape(3, 3).T + old.geom_pos[geom]
        rows.append(dict(geom=name, geom_id=int(geom), body=old.body(int(old.geom_bodyid[geom])).name,
                         source_mesh=old_asset_name, proxy_mesh=new_name, obj=str(obj),
                         obj_sha256=sha256(obj), **certificate))
    tree.write(output / "input_scene.xml", encoding="unicode")
    new = mj.MjModel.from_xml_path(str(output / "input_scene.xml"))
    # Preserve existing runtime-derived physical constants after recompilation.
    new.dof_damping[:] = old.dof_damping
    new.dof_length[:] = old.dof_length
    new.geom_surfacevel[:] = old.geom_surfacevel
    new.flg_surfacevel = old.flg_surfacevel
    for name in dir(old.opt):
        if name.startswith("_"):
            continue
        value = getattr(old.opt, name)
        if isinstance(value, np.ndarray):
            getattr(new.opt, name)[:] = value
        elif isinstance(value, (int, float, bool, np.integer, np.floating)):
            setattr(new.opt, name, value)
    invariants = _physical_invariants(old, new, changed)
    for row in rows:
        geom = old.geom(row["geom"]).id
        row.update(verify_compiled_proxy(geom_body_vertices(old, geom), ideals[geom],
            geom_body_vertices(new, new.geom(row["geom"]).id), row["analytic_outward_bound_m"], max_outward_m))
    od, nd = mj.MjData(old), mj.MjData(new)
    for data in (od, nd):
        data.qpos[:] = initial["qpos"]
        data.qvel[:] = initial["qvel"]
        data.eq_active[:] = initial["eq_active"]
        data.ctrl[:] = initial["ctrl"]
    mj.mj_forward(old, od)
    mj.mj_forward(new, nd)
    initial_position_error = float(np.max(np.abs(od.xpos - nd.xpos)))
    initial_rotation_error = float(np.max(np.abs(od.xquat - nd.xquat)))
    if initial_position_error > 1e-12 or initial_rotation_error > 1e-12:
        raise ValueError("Body initial transform changed")
    upstream = metadata.get("physics_model_variant")
    variant = dict(mode="conservative_fruit_convex_proxy", source_inputs=str(source),
        source_mjb_sha256=source_hashes["input_model.mjb"], upstream_physics_model_variant=upstream,
        physical_model_changed=True, collision_approximation_changed=True,
        exact_trajectory_equivalence_claimed=False, maximum_outward_error_m=max_outward_m,
        preserves_original_convex_exterior=True, retained_all_body_masses=True,
        retained_all_collision_geometry=False, retained_all_nonfruit_collision_geometry=True,
        original_nv=old.nv, variant_nv=new.nv, source_vertices=sum(r["source_vertex_count"] for r in rows),
        proxy_vertices=sum(r["compiled_proxy_vertices"] for r in rows), fruits=rows,
        invariant_proof=invariants, initial_body_position_max_error_m=initial_position_error,
        initial_body_rotation_max_error=initial_rotation_error,
        limitations=["Fruit exterior is conservatively enlarged by the certified bound; contact timing/manifold can change.",
                     "This is a changed collision representation requiring full physics/harvest qualification.",
                     "All other geometry, masses, inertias, joints, equality data, control and initial-state files are retained."],
        training_eligible=False, harvest_success=None)
    metadata["physics_model_variant"] = variant
    mj.mj_saveModel(new, str(output / "input_model.mjb"))
    for name in ("initial_state.npz", "commands.npz", "sampled_plan.json"):
        if (source / name).is_file():
            shutil.copy2(source / name, output / name)
    _json(output / "source_export.json", metadata)
    _json(output / "fruit_proxy_proof.json", variant)
    report = copy.deepcopy(provenance)
    filenames = ["input_scene.xml", "input_model.mjb", "initial_state.npz", "commands.npz", "source_export.json", "fruit_proxy_proof.json"]
    filenames.extend(str(p.relative_to(output)) for p in meshes.glob("*.obj"))
    report["input_files"] = {name: dict(path=str(output / name), sha256=sha256(output / name)) for name in filenames}
    report["source_asset_dependencies"] = [*report.get("source_asset_dependencies", []),
        *[dict(path=str(p), sha256=sha256(p)) for p in meshes.glob("*.obj")]]
    report["physics_model_variant"] = variant
    report["model_counts"] = dict(nbody=new.nbody, nq=new.nq, nv=new.nv, ngeom=new.ngeom, nu=new.nu, neq=new.neq)
    report["physical_steps"] = 0
    report["variant_prepare_seconds"] = time.perf_counter() - started
    report.setdefault("code_sha256", {})[str(Path(__file__).resolve())] = sha256(__file__)
    report["source_prepared_files_sha256"] = source_hashes
    report["source_prepared_files_unchanged"] = all(sha256(source / name) == digest for name, digest in source_hashes.items())
    if not report["source_prepared_files_unchanged"]:
        raise ValueError("Source prepared input changed")
    _json(output / "inputs.json", report)
    return variant


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-outward-m", "--max-error-m", dest="max_outward_m", type=float, default=0.0001)
    parser.add_argument("--budgets", type=int, nargs="+", default=DEFAULT_BUDGETS)
    args = parser.parse_args()
    result = prepare(args.inputs, args.output, max_outward_m=args.max_outward_m, budgets=args.budgets)
    print(json.dumps({"output": str(args.output), "source_vertices": result["source_vertices"],
                      "proxy_vertices": result["proxy_vertices"],
                      "max_outward_bound_m": max(r["compiled_outward_bound_m"] for r in result["fruits"])}, indent=2))
