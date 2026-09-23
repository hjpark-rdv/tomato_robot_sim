"""Reuse the existing candidate and RGB-D collectors for every fruit in seeded GLB scenes."""
import argparse
import datetime
import fcntl
import hashlib
import json
import sys
import time
from pathlib import Path

from collect_tomatoes import log_status, run_step
from generate_random_glb_scenes import generate


def scene_splits(count):
    if count < 3:
        return {f"scene_{i:04d}": "smoke" for i in range(count)}
    validation = max(1, round(count * .15))
    test = max(1, round(count * .15))
    if validation + test >= count:
        raise ValueError("At least one training scene is required")
    return {f"scene_{i:04d}": ("test" if i >= count - test else
                                  "validation" if i >= count - test - validation else "train")
            for i in range(count)}


def complete_target(folder, model_hash, target, count, views, plan_only):
    try:
        physics = folder / "physics"
        observations = folder / "observations"
        run = json.loads((physics / "manifest.json").read_text())
        results = json.loads((physics / "results.json").read_text())
        meta = json.loads((observations / "dataset.json").read_text())
        rows = [json.loads(line) for line in (observations / "observations.jsonl").read_text().splitlines()]
        return (run["model_sha256"] == model_hash and run["target"] == target
                and run["count"] == count and run.get("plan_only", False) == plan_only
                and len(results) == count and meta["target"] == target
                and meta["observations"] == views
                and len(rows) == views
                and all((observations / "observations" / row["observation_id"] / "rgb.png").is_file()
                        and (observations / "observations" / row["observation_id"] / "depth_aligned_to_color_m.npy").is_file()
                        for row in rows)
                and meta["source_results_sha256"] == hashlib.sha256((physics / "results.json").read_bytes()).hexdigest()
                and (meta["schema"] == "farmily_observation_v2" or meta["pose_label_actions"] == 0))
    except (OSError, KeyError, ValueError):
        return False


def mark_scene_views(observations):
    """The collection, never image-cell position, owns the split assignment."""
    from PIL import Image, ImageDraw
    from prepare_observations import gallery
    rows=[json.loads(line) for line in (observations/'observations.jsonl').read_text().splitlines()]
    meta=json.loads((observations/'dataset.json').read_text())
    if (all(row['split']=='scene' for row in rows)
            and meta.get('split_rule')=='collection scene_splits applies to all views'
            and (meta.get('pose_label_actions',0)>0 or all(row.get('action_poses_camera') is None for row in rows))
            and (meta.get('pose_label_actions',0)>0 or meta.get('validation',{}).get('mask_files_saved') is False)
            and all(not (observations/'observations'/row['observation_id']/filename).exists()
                    for row in rows for filename in ('target_visible_mask.png','target_isolated_mask_gt.png',
                                                     'aligned_valid.png','depth_valid.png'))):
        gallery(observations,rows)
        return
    for row in rows:
        row['split']='scene'
        if not meta.get('pose_label_actions',0):
            row['action_poses_camera']=None
        folder=observations/'observations'/row['observation_id']
        (folder/'observation.json').write_text(json.dumps(row,indent=2))
        Image.open(folder/'rgb.png').convert('RGB').save(folder/'annotated_preview.jpg',quality=85)
        for filename in ('target_visible_mask.png','target_isolated_mask_gt.png',
                         'aligned_valid.png','depth_valid.png'):
            (folder/filename).unlink(missing_ok=True)
    with (observations/'observations.jsonl').open('w') as stream:
        for row in rows:stream.write(json.dumps(row)+'\n')
    meta.update(split_rule='collection scene_splits applies to all views',
                split_caveat='Scene-held-out split is assigned by the collection; views are never independently split.',
                plant_randomization=True,target_mask_files_saved=False)
    (observations/'dataset.json').write_text(json.dumps(meta,indent=2))
    sheet=Image.new('RGB',(960,792),'#e9efea');draw=ImageDraw.Draw(sheet)
    for row in rows:
        cell=row['screen_cell']
        if cell not in range(9):continue
        image=Image.open(observations/'observations'/row['observation_id']/'rgb.png').resize((320,240))
        x,y=(cell%3)*320,(cell//3)*264;sheet.paste(image,(x,y))
        draw.text((x+8,y+244),row['observation_id']+' / cell '+str(cell),fill='#234e3e')
    sheet.save(observations/'coverage_preview.jpg',quality=90)
    from validate_observations import validate
    validation=validate(observations)
    if not meta.get('pose_label_actions'):
        meta['validation']=validation
        (observations/'dataset.json').write_text(json.dumps(meta,indent=2))
    gallery(observations,rows)


def collect(args):
    if min(args.scenes, args.trusses, args.candidates, args.workers,
           args.planning_workers, args.postprocess_workers, args.hz) < 1:
        raise ValueError("All counts and Hz must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=args.resume)
    lock = (output / "collection.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    config = dict(schema="random_glb_collection_v1", seed=args.seed,
                  scenes=args.scenes, trusses=args.trusses, candidates_per_target=args.candidates,
                  views_per_target=9, hz=args.hz, plan_only=args.plan_only,
                  scene_splits=scene_splits(args.scenes),
                  source_dir=str(args.source_dir.resolve()),
                  segment_range=[args.segment_min, args.segment_max],
                  angle_range=[args.angle_min, args.angle_max])
    manifest_path = output / "collection.json"
    scene_root = output / "scenes"
    extending=False
    generated_scenes=0
    if args.resume:
        previous = json.loads(manifest_path.read_text())
        generated_scenes=json.loads((scene_root / "manifest.json").read_text())["scenes"]
        extending=bool(args.extend and args.scenes>previous['scenes'])
        recovering_extension=(previous['scenes']==args.scenes and generated_scenes<args.scenes)
        checked={k:v for k,v in config.items() if not (extending and k in ('scenes','scene_splits'))}
        if any(previous.get(key) != value for key, value in checked.items()):
            raise ValueError("Resume configuration differs from saved collection")
        if generated_scenes<args.scenes and not (extending or recovering_extension):
            raise ValueError("Adding scenes requires --extend")
        manifest = previous
    else:
        manifest = dict(config, status="running", targets=[], failures=[])
        manifest_path.write_text(json.dumps(manifest, indent=2))
    if not args.resume or generated_scenes<args.scenes:
        generate(scene_root, args.seed, args.scenes, args.trusses, args.source_dir,
                 args.segment_min, args.segment_max, args.angle_min, args.angle_max,
                 args.idle_seconds, not args.no_render,
                 start_scene=generated_scenes if args.resume else 0)
    if args.resume and (extending or recovering_extension):
        manifest.update(scenes=args.scenes,scene_splits=config['scene_splits'])
        for row in manifest['targets']:
            row['scene_split']=config['scene_splits'][row['scene']]
        manifest['status']='running'
        manifest.pop('error',None)
        manifest_path.write_text(json.dumps(manifest,indent=2))
    records = json.loads((scene_root / "manifest.json").read_text())["records"]
    python = sys.executable
    started = time.perf_counter()
    try:
        for record in records:
            scene_id = record["scene"]
            if not record["validation"]["scene_screen_passed"]:
                manifest["failures"].append(dict(scene=scene_id, reason="initial_scene_screen_failed"))
                log_status(f"초기 물리 검사 실패, 장면 제외: {scene_id}")
                continue
            scene = scene_root / scene_id
            reference = json.loads((scene / "reference.json").read_text())
            model_hash = record["model_sha256"]
            for index, spec in enumerate(reference["fruit_specs"]):
                target = spec["name"]
                folder = output / scene_id / "targets" / target
                folder.mkdir(parents=True, exist_ok=args.resume)
                physics = folder / "physics"
                observations = folder / "observations"
                if args.resume and complete_target(folder, model_hash, target, args.candidates, 9, args.plan_only):
                    mark_scene_views(observations)
                    log_status(f"완료 대상 재사용: {scene_id}/{target}")
                    continue
                if args.resume and observations.exists():
                    observations.rename(folder / ("observations_interrupted_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
                if physics.exists():
                    run = json.loads((physics / "manifest.json").read_text())
                    if run["model_sha256"] != model_hash or run["target"] != target or run["count"] != args.candidates or run.get("plan_only", False) != args.plan_only:
                        raise ValueError("Existing physics run differs: " + str(physics))
                    if args.plan_only:
                        raise ValueError("Interrupted plan-only run requires a fresh output directory")
                    command = [python, str(Path(__file__).with_name("resume_candidates.py")), str(physics),
                               "--workers", str(args.workers), "--planning-workers", str(args.planning_workers)]
                else:
                    command = [python, str(Path(__file__).with_name("candidate_experiment.py")),
                               "--model", str(scene / "model.mjb"), "--reference", str(scene / "reference.json"),
                               "--target", target, "--candidates", str(args.candidates),
                               "--workers", str(args.workers), "--planning-workers", str(args.planning_workers),
                               "--hz", str(args.hz), "--seed", str(args.seed + index),
                               "--link-model", "--output", str(physics)]
                    if args.plan_only:
                        command.append("--plan-only")
                log_status(f"후보 {'계획' if args.plan_only else '물리'}: {scene_id}/{target}")
                with (folder / "physics.log").open("a" if args.resume else "w") as log:
                    run_step(command, log)
                capture = [python, str(Path(__file__).with_name("prepare_observations.py")), str(physics),
                           "--views", "9", "--seed", str(args.seed + index),
                           "--postprocess-workers", str(args.postprocess_workers), "--output", str(observations)]
                log_status(f"RGB-D 9장: {scene_id}/{target}")
                with (folder / "observations.log").open("w") as log:
                    run_step(capture, log)
                mark_scene_views(observations)
                results = json.loads((physics / "results.json").read_text())
                meta = json.loads((observations / "dataset.json").read_text())
                row = dict(scene=scene_id, scene_split=config["scene_splits"][scene_id], target=target,
                           physics=str(physics), observations=str(observations),
                           candidates=len(results), planned=sum(bool(r.get("preflight", {}).get("passed")) for r in results),
                           center_entries=sum(r["result"] == "partial_center_entry" for r in results),
                           result_counts={label:sum(r["result"] == label for r in results)
                                          for label in sorted({r["result"] for r in results})},
                           views=meta["observations"], model_sha256=model_hash,
                           search_outcome="entry_found" if any(r["result"] == "partial_center_entry" for r in results)
                           else "not_found_in_tested_candidates")
                manifest["targets"] = [r for r in manifest["targets"] if (r["scene"], r["target"]) != (scene_id, target)] + [row]
                manifest_path.write_text(json.dumps(manifest, indent=2))
        manifest["status"] = "complete"
    except BaseException as error:
        manifest["status"] = "interrupted_or_failed"
        manifest["error"] = repr(error)
        raise
    finally:
        manifest["wall_s"] = manifest.get("wall_s", 0) + time.perf_counter() - started
        manifest_path.write_text(json.dumps(manifest, indent=2))
        cards = "".join(f'<li>{r["scene"]} / {r["target"]}: '
                        f'<a href="{r["scene"]}/targets/{r["target"]}/physics/index.html">physics</a> · '
                        f'<a href="{r["scene"]}/targets/{r["target"]}/observations/index.html">RGB-D</a> '
                        f'({r["candidates"]} 후보, {r["views"]} 시점, {r["search_outcome"]})</li>'
                        for r in manifest["targets"])
        (output / "index.html").write_text('<meta charset="utf-8"><h1>랜덤 GLB 장면 데이터 수집</h1>'
                                            '<p>중심 진입 판정이며 꼭지 걸림 성공이 아닙니다.</p><ul>'
                                            + cards + '</ul><a href="collection.json">manifest</a>')
    return manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=Path("/root/docker_share/mujoko_debugging_data") /
                   (datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + "_random_glb_collection"))
    p.add_argument("--seed", type=int, default=23)
    p.add_argument("--scenes", type=int, default=3)
    p.add_argument("--trusses", type=int, default=1)
    p.add_argument("--candidates", type=int, default=10)
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--planning-workers", type=int, default=4)
    p.add_argument("--postprocess-workers", type=int, default=4)
    p.add_argument("--hz", type=int, default=240)
    p.add_argument("--source-dir", type=Path, default=Path(__file__).resolve().parents[2] / "nvidia-sim/env_usd/tomato_rotate_glb")
    p.add_argument("--segment-min", type=int, default=4)
    p.add_argument("--segment-max", type=int, default=13)
    p.add_argument("--angle-min", type=float, default=0)
    p.add_argument("--angle-max", type=float, default=180)
    p.add_argument("--idle-seconds", type=float, default=2)
    p.add_argument("--plan-only", action="store_true")
    p.add_argument("--no-render", action="store_true")
    p.add_argument("--resume", action="store_true")
    p.add_argument("--extend", action="store_true",help="with --resume, append more scenes using the saved seed")
    args = p.parse_args()
    if args.extend and not args.resume:p.error("--extend requires --resume")
    result = collect(args)
    log_status(f"완료: {args.output}/index.html · 상태 {result['status']}")


if __name__ == "__main__":
    main()
