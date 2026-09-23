"""Seeded, physical GLB attachments on the unchanged main stem and leaves."""
import argparse
import copy
import hashlib
import html
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import numpy as np

import render_backend  # Set up scoped EGL before importing MuJoCo.
import mujoco as mj
from PIL import Image

from build_glb_physics import ROOT, build
from robot_engine import RobotEngine

GLB_DIR = ROOT / "nvidia-sim/env_usd/tomato_rotate_glb"
WHITE_OFFSET = [-0.0005066500582317888, -0.0008218853535071469, 0.002840389090280442]


def profiles(source_dir):
    files = sorted(source_dir.glob("*.glb"))
    if not files:
        raise ValueError(f"No GLB files in {source_dir}")
    rows = []
    for path in files:
        white = path.name == "tomato_master_v10_cluster_curve_white.glb"
        rows.append(dict(path=path, remove_fruits=[6],
                         fruit_offsets={2: WHITE_OFFSET} if white else {},
                         rachis_stiffness_scale=2.0 if white else 1.0))
    return rows


def sample_placements(rng, variants, count, segment_min, segment_max, angle_min, angle_max):
    # Independent intervals spread several attachments across the chosen stem span.
    indices = rng.choice(len(variants), size=count, replace=count > len(variants))
    boundaries = np.linspace(segment_min, segment_max + 1, count + 1)
    rows = []
    for i, variant_index in enumerate(indices):
        low, high = boundaries[i:i + 2]
        position = float(rng.uniform(low, high))
        # Avoid the segment ends, where a root can jump across a body joint.
        segment = min(int(np.floor(position)), segment_max)
        fraction = float(np.clip(position - segment, .15, .85))
        variant = variants[int(variant_index)]
        rows.append(dict(source_glb=str(variant["path"].resolve()),
                         source_sha256=hashlib.sha256(variant["path"].read_bytes()).hexdigest(),
                         stem_segment=segment, stem_fraction=fraction,
                         y_deg=float(rng.uniform(angle_min, angle_max)),
                         remove_fruits=variant["remove_fruits"],
                         fruit_offsets=variant["fruit_offsets"],
                         rachis_stiffness_scale=variant["rachis_stiffness_scale"]))
    return rows


def combine(parts, placements, destination):
    tree = ET.parse(parts[0] / "model.xml")
    root = tree.getroot()
    asset = root.find("asset")
    contacts = root.find("contact")
    specs = json.loads((parts[0] / "reference.json").read_text())["fruit_specs"]
    for index, part in enumerate(parts[1:], 1):
        other = ET.parse(part / "model.xml").getroot()
        prefix = f"truss_{index:02d}__"
        parent_name = f"STEM_MainStem_{placements[index]['stem_segment']:02d}"
        parent = root.find(f".//body[@name='{parent_name}']")
        old_parent = other.find(f".//body[@name='{parent_name}']")
        branch = next((b for b in old_parent.findall("body")
                       if b.get("name") == "TRUSS_Truss_01_Peduncle_00"), None)
        if parent is None or branch is None:
            raise ValueError(f"GLB root absent from {parent_name}")
        branch = copy.deepcopy(branch)
        old_bodies = {b.get("name") for b in branch.iter("body")}
        meshes = {g.get("mesh") for g in branch.iter("geom") if g.get("mesh")}
        for mesh in other.find("asset").findall("mesh"):
            if mesh.get("name") in meshes:
                node = copy.deepcopy(mesh)
                node.set("name", prefix + node.get("name"))
                asset.append(node)
        for node in branch.iter():
            if node.tag in ("body", "joint", "geom") and node.get("name"):
                node.set("name", prefix + node.get("name"))
            if node.tag == "geom" and node.get("mesh"):
                node.set("mesh", prefix + node.get("mesh"))
        parent.append(branch)
        for exclusion in other.find("contact").findall("exclude"):
            a, b = exclusion.get("body1"), exclusion.get("body2")
            if a in old_bodies or b in old_bodies:
                node = copy.deepcopy(exclusion)
                node.set("body1", prefix + a if a in old_bodies else a)
                node.set("body2", prefix + b if b in old_bodies else b)
                contacts.append(node)
        for fruit in json.loads((part / "reference.json").read_text())["fruit_specs"]:
            entry = copy.deepcopy(fruit)
            entry["name"] = prefix + entry["name"]
            entry["anchor"] = "/GLB/" + prefix + Path(entry["anchor"]).name
            entry["path"] = "/GLB/" + entry["name"]
            specs.append(entry)
    tree.write(destination / "model.xml", encoding="unicode")
    model = mj.MjModel.from_xml_path(str(destination / "model.xml"))
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    mj.mj_saveModel(model, str(destination / "model.mjb"))
    ref = json.loads((parts[0] / "reference.json").read_text())
    ref.update(schema="random_glb_plant_v1", fruit_specs=specs,
               source_glbs=[p["source_glb"] for p in placements],
               bodies=[], shapes=[], visuals=[])
    for i in range(model.nbody):
        name = model.body(i).name
        if not (name.startswith(("STEM_", "TRUSS_", "Attachment_", "Tomato_", "truss_"))):
            continue
        ref["bodies"].append(dict(name=name, path="/GLB/" + name,
                                  pose=[*data.xpos[i].tolist(), *data.xquat[i].tolist()],
                                  mass=float(model.body_mass[i])))
    for i in range(model.ngeom):
        name = model.geom(i).name
        body = model.body(model.geom_bodyid[i]).name
        if body != "Hook" and not body.startswith(("STEM_", "TRUSS_", "Attachment_", "Tomato_", "truss_")):
            continue
        item = dict(name=name, path="/GLB/" + body + "/" + name,
                    body=ref["tool_path"] if body == "Hook" else "/GLB/" + body)
        (ref["shapes"] if model.geom_contype[i] or model.geom_conaffinity[i]
         else ref["visuals"]).append(item)
    (destination / "reference.json").write_text(json.dumps(ref, indent=2))
    return model, data


def inspect(model_path, reference_path, seconds=2., hz=240,
            collision_threshold=.0005, idle_motion_threshold=.005):
    engine = RobotEngine(model_path, reference=reference_path, hz=hz)
    model, data = engine.model, engine.data
    plant = np.array([model.body(b["name"]).id for b in engine.ref["bodies"]])
    start = data.xpos[plant].copy()
    max_motion = 0.
    max_penetration = 0.
    worst = None
    initial_penetration = 0.
    for step in range(round(seconds * hz) + 1):
        for contact in data.contact:
            a, b = model.geom(contact.geom1).name, model.geom(contact.geom2).name
            if not (a.startswith("glb_col_") or b.startswith("glb_col_")
                    or a.startswith("truss_") or b.startswith("truss_")):
                continue
            depth = max(0., -float(contact.dist))
            if step == 0:
                initial_penetration = max(initial_penetration, depth)
            if depth > max_penetration:
                max_penetration = depth
                worst = dict(a=a, b=b, penetration_m=depth, step=step)
        max_motion = max(max_motion, float(np.linalg.norm(data.xpos[plant] - start, axis=1).max()))
        if step < round(seconds * hz):
            mj.mj_step(model, data)
            mj.mj_forward(model, data)
    stable = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
                  and not np.any(data.warning.number))
    return dict(initial_penetration_m=initial_penetration,
                max_penetration_m=max_penetration, worst_contact=worst,
                max_idle_motion_m=max_motion, warnings=data.warning.number.tolist(),
                finite_and_warning_free=stable,
                collision_screen_passed=stable and max_penetration <= collision_threshold,
                idle_motion_screen_passed=stable and max_motion <= idle_motion_threshold,
                scene_screen_passed=(stable and max_penetration <= collision_threshold
                                     and max_motion <= idle_motion_threshold),
                thresholds_m=dict(collision=collision_threshold,
                                  idle_motion=idle_motion_threshold),
                scope=f"idle {seconds:g}s at {hz}Hz; no robot trajectory or force pulse")


def preview(model, data, destination, placements):
    try:
        model.vis.global_.offwidth = 1000
        model.vis.global_.offheight = 800
        camera = mj.MjvCamera()
        camera.lookat[:] = np.mean([p["attachment_world_m"] for p in placements], axis=0)
        camera.distance = 2.
        camera.azimuth = 105
        camera.elevation = -12
        renderer = mj.Renderer(model, height=800, width=1000)
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(destination / "preview.png")
        renderer.close()
        return "preview.png"
    except Exception as error:
        # A headless GPU display problem must not mislabel the physics result.
        return dict(render_error=str(error))


def generate(output, seed, scenes, trusses, source_dir=GLB_DIR,
             segment_min=4, segment_max=13, angle_min=0., angle_max=180.,
             idle_seconds=2., render=True, start_scene=0, truss_scale=1.):
    if scenes < 1 or trusses < 1:
        raise ValueError("scenes and trusses must be positive")
    if not 0 <= segment_min <= segment_max < 16:
        raise ValueError("Stem segment range must lie within [0, 15]")
    if not 0 <= angle_min <= angle_max <= 180:
        raise ValueError("GLB Y angle range must lie within [0, 180]")
    if idle_seconds < 0 or not np.isfinite(idle_seconds):
        raise ValueError("Invalid idle duration")
    if not np.isfinite(truss_scale) or truss_scale<=0:
        raise ValueError('Invalid truss scale')
    variants = profiles(source_dir)
    if start_scene:
        old=json.loads((output/'manifest.json').read_text())
        if (old['seed']!=seed or old['scenes']!=start_scene or old['trusses_per_scene']!=trusses
                or old['source_dir']!=str(source_dir.resolve())
                or old['segment_range']!=[segment_min,segment_max]
                or old['y_deg_range']!=[angle_min,angle_max]
                or old.get('truss_scale',1.)!=truss_scale):
            raise ValueError('Existing scene generator settings differ')
        records=old['records']
    else:
        output.mkdir(parents=True, exist_ok=False)
        records=[]
    for scene_id in range(start_scene,scenes):
        scene = output / f"scene_{scene_id:04d}"
        scene.mkdir()
        rng = np.random.default_rng(np.random.SeedSequence([seed, scene_id]))
        placements = sample_placements(rng, variants, trusses, segment_min,
                                       segment_max, angle_min, angle_max)
        parts = []
        for i, placement in enumerate(placements):
            part = scene / "parts" / f"truss_{i:02d}"
            build(Path(placement["source_glb"]), part, y_deg=placement["y_deg"],
                  segment=placement["stem_segment"],
                  stem_fraction=placement["stem_fraction"],
                  remove_fruits=placement["remove_fruits"],
                  fruit_offsets=placement["fruit_offsets"],
                  rachis_stiffness_scale=placement["rachis_stiffness_scale"],truss_scale=truss_scale)
            metadata = json.loads((part / "build.json").read_text())
            placement["truss_scale"] = truss_scale
            placement["attachment_world_m"] = metadata["attachment_world_m"]
            parts.append(part)
        if trusses == 1:
            for filename in ("model.xml", "model.mjb", "reference.json"):
                (scene / filename).write_bytes((parts[0] / filename).read_bytes())
        else:
            combine(parts, placements, scene)
        model = mj.MjModel.from_binary_path(str(scene / "model.mjb"))
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        check = inspect(scene / "model.mjb", scene / "reference.json",
                        seconds=idle_seconds)
        visual = preview(model, data, scene, placements) if render else None
        record = dict(scene=scene.name, seed=seed, scene_id=scene_id,
                      placements=placements, validation=check, preview=visual,
                      model_sha256=hashlib.sha256((scene / "model.mjb").read_bytes()).hexdigest(),
                      generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      physics_ready=False,
                      note="Idle screen only; robot motion and labels require a separate rollout")
        (scene / "scene.json").write_text(json.dumps(record, indent=2))
        shutil.rmtree(scene / "parts")
        records.append(record)
        print(scene.name, "trusses", trusses, "scene screen",
              check["scene_screen_passed"], "max depth mm",
              round(check["max_penetration_m"] * 1000, 3), flush=True)
    (output / "manifest.json").write_text(json.dumps(dict(seed=seed, scenes=scenes,
        trusses_per_scene=trusses, source_dir=str(source_dir.resolve()),
        segment_range=[segment_min, segment_max], y_deg_range=[angle_min, angle_max],
        idle_seconds=idle_seconds, truss_scale=truss_scale, records=records), indent=2))
    cards = []
    for record in records:
        path = record["scene"]
        rows = "".join(f"<li>{html.escape(Path(p['source_glb']).stem)}: "
                       f"segment {p['stem_segment']} + {p['stem_fraction']:.2f}, "
                       f"Y {p['y_deg']:.1f}°, scale {p.get('truss_scale',1):g}</li>" for p in record["placements"])
        image = f"<img src='{path}/preview.png'>" if record["preview"] == "preview.png" else ""
        cards.append(f"<section><h2>{path}</h2><p>초기 정지 검사 통과: "
                     f"{record['validation']['scene_screen_passed']}; 최대 겹침 "
                     f"{record['validation']['max_penetration_m']*1000:.3f}mm; "
                     f"최대 정지 이동 {record['validation']['max_idle_motion_m']*1000:.2f}mm</p>"
                     f"{image}<ul>{rows}</ul><a href='{path}/scene.json'>metadata</a></section>")
    (output / "index.html").write_text("<!doctype html><meta charset='utf-8'>"
        "<title>랜덤 GLB 송이 장면</title><style>body{font:18px sans-serif;"
        "max-width:1200px;margin:40px auto}img{max-width:100%}section{border-top:1px solid #888}"
        "</style><h1>랜덤 GLB 송이 부착</h1><p>기존 주줄기·잎 유지. GLB Y축 "
        "0–180° 회전. 장면마다 초기 2초 검사 결과를 표시한다. 로봇 수집 결과는 별도다.</p>"
        + "".join(cards))
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/root/docker_share/mujoko_debugging_data") /
                        (datetime.now().strftime("%Y%m%d_%H%M%S") + "_random_glb_scenes"))
    parser.add_argument("--source-dir", type=Path, default=GLB_DIR)
    parser.add_argument("--seed", type=int, default=23)
    parser.add_argument("--scenes", type=int, default=3)
    parser.add_argument("--trusses", type=int, default=1)
    parser.add_argument("--segment-min", type=int, default=4)
    parser.add_argument("--segment-max", type=int, default=13)
    parser.add_argument("--angle-min", type=float, default=0.)
    parser.add_argument("--angle-max", type=float, default=180.)
    parser.add_argument("--truss-scale", type=float, default=.5)
    parser.add_argument("--idle-seconds", type=float, default=2.)
    parser.add_argument("--no-render", action="store_true")
    args = parser.parse_args()
    generate(args.output, args.seed, args.scenes, args.trusses, args.source_dir,
             args.segment_min, args.segment_max, args.angle_min, args.angle_max,
             args.idle_seconds, not args.no_render, truss_scale=args.truss_scale)
    print(args.output / "index.html")


if __name__ == "__main__":
    main()
