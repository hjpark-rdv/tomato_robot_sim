"""GLB 5종 각 1송이 학습 데이터 수집 스크립트.

토마토 종류별(총 5송이) 1번씩, 알맹이 1개당 20회 후보 실험 + 20장 RGB-D 관측 수집.

제약 조건:
  - Y축 회전 0~20도 (GLB 대칭상 ±20도와 동등)
  - 부착 segment 6~10 (너무 높거나 낮은 위치 제외)
  - candidates=20, views=20
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import sys
import time
from pathlib import Path

from collect_tomatoes import log_status, run_step


# GLB 5종 파일명과 짧은 식별자 목록 (순서 고정)
GLB_VARIANTS = [
    ("tomato_master_v10_cluster_curve_cyan.glb",    "cyan"),
    ("tomato_master_v10_cluster_curve_green.glb",   "green"),
    ("tomato_master_v10_cluster_curve_red.glb",     "red"),
    ("tomato_master_v10_cluster_curve_white.glb",   "white"),
    ("tomato_master_v10_cluster_rotated90.glb",     "rotated90"),
]

# 수집 설정
ANGLE_MIN   = 0.0    # Y축 회전 최솟값(도) — GLB 대칭상 ±20도와 동등
ANGLE_MAX   = 20.0   # Y축 회전 최댓값(도)
SEGMENT_MIN = 6      # 부착 segment 하한 (너무 낮은 위치 제외)
SEGMENT_MAX = 10     # 부착 segment 상한 (너무 높은 위치 제외)
VIEWS_ARG   = 71     # prepare_observations 에 전달할 --views (9 또는 71만 지원)
VIEWS_LIMIT = 20     # 실제로 수집할 이미지 수 (--limit으로 앞에서 자름)


def complete_target(folder, model_hash, target, count, n_views):
    """이미 완료된 target인지 확인한다."""
    try:
        physics = folder / "physics"
        observations = folder / "observations"
        run = json.loads((physics / "manifest.json").read_text())
        results = json.loads((physics / "results.json").read_text())
        meta = json.loads((observations / "dataset.json").read_text())
        rows = [json.loads(line)
                for line in (observations / "observations.jsonl").read_text().splitlines()]
        return (run["model_sha256"] == model_hash
                and run["target"] == target
                and run["count"] == count
                and len(results) == count
                and meta["target"] == target
                and meta["observations"] == n_views
                and len(rows) == n_views
                and all(
                    (observations / "observations" / row["observation_id"] / "rgb.png").is_file()
                    and (observations / "observations" / row["observation_id"] / "depth_aligned_to_color_m.npy").is_file()
                    for row in rows
                )
                and meta["source_results_sha256"] == hashlib.sha256(
                    (physics / "results.json").read_bytes()).hexdigest()
                and (meta.get("schema") == "farmily_observation_v2"
                     or meta.get("pose_label_actions", 0) == 0))
    except (OSError, KeyError, ValueError):
        return False


def build_single_glb_scene(output_dir, glb_path, label, label_index, seed,
                            segment_min, segment_max,
                            angle_min, angle_max,
                            idle_seconds, truss_scale, render):
    """단일 GLB 파일로 장면 하나를 생성한다.

    generate_random_glb_scenes.generate()의 핵심 로직을 단일 GLB에 대해 실행.
    """
    import copy
    import hashlib as _hs
    import shutil
    import xml.etree.ElementTree as ET
    import numpy as np
    import mujoco as mj
    from PIL import Image
    from build_glb_physics import build
    from generate_random_glb_scenes import inspect, preview, WHITE_OFFSET
    from robot_engine import RobotEngine  # noqa: F401 (render_backend must be loaded first)

    scene_dir = output_dir / f"scene_{label}"
    scene_dir.mkdir(parents=True, exist_ok=False)

    rng = np.random.default_rng(np.random.SeedSequence([seed, label_index]))

    # GLB 프로파일 결정
    white = glb_path.name == "tomato_master_v10_cluster_curve_white.glb"
    fruit_offsets = {2: WHITE_OFFSET} if white else {}
    rachis_stiffness_scale = 2.0 if white else 1.0
    remove_fruits = [6]

    # segment와 Y 회전 샘플링 (구간 중간에서 1개)
    # fraction을 0.25~0.75로 좁혀 segment 경계 근처 불안정 배치 방지
    boundaries = np.linspace(segment_min, segment_max + 1, 2)
    low, high = boundaries[0], boundaries[1]
    position = float(rng.uniform(low, high))
    segment = min(int(np.floor(position)), segment_max)
    fraction = float(np.clip(position - segment, .25, .75))
    y_deg = float(rng.uniform(angle_min, angle_max))

    placement = dict(
        source_glb=str(glb_path.resolve()),
        source_sha256=_hs.sha256(glb_path.read_bytes()).hexdigest(),
        stem_segment=segment,
        stem_fraction=fraction,
        y_deg=y_deg,
        remove_fruits=remove_fruits,
        fruit_offsets=fruit_offsets,
        rachis_stiffness_scale=rachis_stiffness_scale,
        truss_scale=truss_scale,
    )

    log_status(f"  장면 생성: {label} | segment={segment}+{fraction:.2f} | Y={y_deg:.1f}°")

    part = scene_dir / "parts" / "truss_00"
    build(glb_path, part,
          y_deg=y_deg, segment=segment, stem_fraction=fraction,
          remove_fruits=remove_fruits, fruit_offsets=fruit_offsets,
          rachis_stiffness_scale=rachis_stiffness_scale,
          truss_scale=truss_scale)

    metadata = json.loads((part / "build.json").read_text())
    placement["truss_scale"] = truss_scale
    placement["attachment_world_m"] = metadata["attachment_world_m"]

    for filename in ("model.xml", "model.mjb", "reference.json"):
        (scene_dir / filename).write_bytes((part / filename).read_bytes())
    shutil.rmtree(scene_dir / "parts")

    model = mj.MjModel.from_binary_path(str(scene_dir / "model.mjb"))
    data = mj.MjData(model)
    mj.mj_forward(model, data)

    check = inspect(scene_dir / "model.mjb", scene_dir / "reference.json",
                    seconds=idle_seconds)
    visual = preview(model, data, scene_dir, [placement]) if render else None

    record = dict(
        scene=scene_dir.name, label=label, glb=glb_path.name, seed=seed,
        placements=[placement], validation=check, preview=visual,
        model_sha256=_hs.sha256((scene_dir / "model.mjb").read_bytes()).hexdigest(),
        physics_ready=False,
        note="학습 데이터 수집용 단일-GLB 장면",
    )
    (scene_dir / "scene.json").write_text(json.dumps(record, indent=2))

    print(f"  {label}: segment={segment}+{fraction:.2f} Y={y_deg:.1f}° "
          f"screen={check['scene_screen_passed']} "
          f"max_depth_mm={round(check['max_penetration_m']*1000,3)}",
          flush=True)
    return record


def collect(args):
    """5종 GLB 학습 데이터 수집 메인 로직."""
    if min(args.candidates, args.workers, args.planning_workers,
           args.postprocess_workers, args.hz) < 1:
        raise ValueError("All counts and Hz must be positive")

    source_dir = args.source_dir.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=args.resume)

    # 중복 실행 방지 잠금
    lock = (output / "collection.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

    n_views = args.views_limit if not args.smoke else 9

    manifest_path = output / "collection.json"
    if args.resume:
        manifest = json.loads(manifest_path.read_text())
    else:
        manifest = dict(
            schema="training_dataset_v1",
            seed=args.seed,
            angle_min=ANGLE_MIN, angle_max=ANGLE_MAX,
            segment_min=SEGMENT_MIN, segment_max=SEGMENT_MAX,
            candidates_per_target=args.candidates,
            views_per_target=n_views,
            truss_scale=args.truss_scale,
            hz=args.hz,
            status="running",
            glb_scenes=[],
            targets=[],
            failures=[],
        )
        manifest_path.write_text(json.dumps(manifest, indent=2))

    python = sys.executable
    started = time.perf_counter()

    # smoke 모드: cyan 1종, Tomato_05만, candidates=2
    variants = (GLB_VARIANTS[:1] if args.smoke else GLB_VARIANTS)
    smoke_target_only = args.smoke

    scenes_dir = output / "scenes"
    scenes_dir.mkdir(exist_ok=True)

    # ── 장면 생성 단계 ──────────────────────────────────────────────
    log_status("=== 장면 생성 단계 ===")
    existing_labels = {s["label"] for s in manifest.get("glb_scenes", [])}
    records = list(manifest.get("glb_scenes", []))

    MAX_RETRIES = 5  # 물리 검사 실패 시 최대 재시도 횟수 (seed offset 변경)
    for glb_idx, (glb_name, label) in enumerate(variants):
        if args.resume and label in existing_labels:
            log_status(f"  장면 재사용: {label}")
            continue
        glb_path = source_dir / glb_name
        if not glb_path.exists():
            raise FileNotFoundError(f"GLB 파일 없음: {glb_path}")

        record = None
        for attempt in range(MAX_RETRIES):
            attempt_seed = args.seed + attempt * 1000  # seed offset으로 다른 배치 시도
            scene_label = label if attempt == 0 else f"{label}_attempt{attempt}"
            # 이전 실패한 scene 디렉토리 정리
            failed_dir = scenes_dir / f"scene_{scene_label}"
            if failed_dir.exists() and attempt > 0:
                import shutil as _shutil
                _shutil.rmtree(failed_dir)
            try:
                record = build_single_glb_scene(
                    scenes_dir, glb_path, scene_label, glb_idx, attempt_seed,
                    SEGMENT_MIN, SEGMENT_MAX, ANGLE_MIN, ANGLE_MAX,
                    args.idle_seconds, args.truss_scale, not args.no_render,
                )
            except Exception as e:
                log_status(f"  장면 생성 오류 ({label} 시도 {attempt+1}/{MAX_RETRIES}): {e}")
                continue
            if record["validation"]["scene_screen_passed"]:
                # 성공: 디렉토리 이름을 label로 정규화
                if scene_label != label:
                    final_dir = scenes_dir / f"scene_{label}"
                    (scenes_dir / f"scene_{scene_label}").rename(final_dir)
                    record["scene"] = f"scene_{label}"
                    record["label"] = label
                log_status(f"  장면 물리 검사 통과: {label} (시도 {attempt+1})")
                break
            else:
                log_status(f"  물리 검사 실패, 재시도 ({label} {attempt+1}/{MAX_RETRIES}): "
                           f"idle_motion={record['validation']['max_idle_motion_m']*1000:.2f}mm")
                # 실패한 디렉토리 정리 후 재시도
                failed_dir = scenes_dir / f"scene_{scene_label}"
                if failed_dir.exists():
                    import shutil as _shutil
                    _shutil.rmtree(failed_dir)
                record = None
        if record is None:
            raise RuntimeError(f"GLB 장면 물리 검사 {MAX_RETRIES}회 모두 실패: {label}")

        records.append(record)
        manifest["glb_scenes"] = records
        manifest_path.write_text(json.dumps(manifest, indent=2))

    # ── 물리 + 관측 수집 단계 ─────────────────────────────────────
    log_status("=== 물리·관측 수집 단계 ===")
    try:
        for record in records:
            label = record["label"]
            scene_dir = scenes_dir / f"scene_{label}"
            reference = json.loads((scene_dir / "reference.json").read_text())
            model_hash = record["model_sha256"]

            if not record["validation"]["scene_screen_passed"]:
                manifest["failures"].append(dict(scene=label, reason="scene_screen_failed"))
                log_status(f"  초기 물리 검사 실패, 제외: {label}")
                manifest_path.write_text(json.dumps(manifest, indent=2))
                continue

            fruit_specs = reference["fruit_specs"]
            if smoke_target_only:
                # smoke: Tomato_05만 (또는 첫 번째 열매)
                fruit_specs = [s for s in fruit_specs if s["name"] == "Tomato_05"]
                if not fruit_specs:
                    fruit_specs = reference["fruit_specs"][:1]
                candidates_n = min(2, args.candidates)
            else:
                candidates_n = args.candidates

            for index, spec in enumerate(fruit_specs):
                target = spec["name"]
                folder = output / f"scene_{label}" / "targets" / target
                folder.mkdir(parents=True, exist_ok=True)
                physics = folder / "physics"
                observations = folder / "observations"

                if args.resume and complete_target(folder, model_hash, target,
                                                   candidates_n, n_views):
                    log_status(f"  완료 재사용: {label}/{target}")
                    continue
                if args.resume and observations.exists():
                    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    observations.rename(folder / f"observations_interrupted_{ts}")

                # 물리 후보 실험
                if physics.exists():
                    run_meta = json.loads((physics / "manifest.json").read_text())
                    if (run_meta["model_sha256"] != model_hash
                            or run_meta["target"] != target
                            or run_meta["count"] != candidates_n):
                        raise ValueError(f"기존 물리 실행 설정 불일치: {physics}")
                    command = [python,
                               str(Path(__file__).with_name("resume_candidates.py")),
                               str(physics),
                               "--workers", str(args.workers),
                               "--planning-workers", str(args.planning_workers)]
                else:
                    command = [python,
                               str(Path(__file__).with_name("candidate_experiment.py")),
                               "--model", str(scene_dir / "model.mjb"),
                               "--reference", str(scene_dir / "reference.json"),
                               "--target", target,
                               "--candidates", str(candidates_n),
                               "--workers", str(args.workers),
                               "--planning-workers", str(args.planning_workers),
                               "--hz", str(args.hz),
                               "--seed", str(args.seed + index),
                               "--link-model",
                               "--output", str(physics)]

                log_status(f"  물리 실험: {label}/{target} ({candidates_n}회)")
                with (folder / "physics.log").open("a" if args.resume else "w") as log:
                    run_step(command, log)

                # RGB-D 관측 수집
                # --views 71 + --limit 20 → 정확히 20장 수집
                views_arg = str(VIEWS_ARG) if not args.smoke else "9"
                capture = [python,
                           str(Path(__file__).with_name("prepare_observations.py")),
                           str(physics),
                           "--views", views_arg,
                           "--seed", str(args.seed + index),
                           "--postprocess-workers", str(args.postprocess_workers),
                           "--output", str(observations)]
                if not args.smoke:
                    capture.extend(["--limit", str(VIEWS_LIMIT)])

                log_status(f"  관측 촬영: {label}/{target} ({n_views}장)")
                with (folder / "observations.log").open("w") as log:
                    run_step(capture, log)

                results = json.loads((physics / "results.json").read_text())
                meta = json.loads((observations / "dataset.json").read_text())
                row = dict(
                    scene=label, target=target,
                    physics=str(physics), observations=str(observations),
                    candidates=len(results),
                    planned=sum(
                        bool(r.get("preflight", {}).get("passed")) for r in results),
                    center_entries=sum(
                        r["result"] == "partial_center_entry" for r in results),
                    result_counts={
                        lbl: sum(r["result"] == lbl for r in results)
                        for lbl in sorted({r["result"] for r in results})},
                    views=meta["observations"],
                    model_sha256=model_hash,
                    search_outcome=(
                        "entry_found"
                        if any(r["result"] == "partial_center_entry" for r in results)
                        else "not_found_in_tested_candidates"),
                )
                manifest["targets"] = (
                    [r for r in manifest["targets"]
                     if (r["scene"], r["target"]) != (label, target)]
                    + [row])
                manifest_path.write_text(json.dumps(manifest, indent=2))
                log_status(f"  완료: {label}/{target} "
                           f"| 진입={row['center_entries']}/{candidates_n} "
                           f"| 관측={row['views']}장")

        manifest["status"] = "complete"
    except BaseException as error:
        manifest["status"] = "interrupted_or_failed"
        manifest["error"] = repr(error)
        raise
    finally:
        manifest["wall_s"] = manifest.get("wall_s", 0) + time.perf_counter() - started
        manifest_path.write_text(json.dumps(manifest, indent=2))

        cards = "".join(
            f'<li>{r["scene"]} / {r["target"]}: '
            f'<a href="scene_{r["scene"]}/targets/{r["target"]}/physics/index.html">물리</a> · '
            f'<a href="scene_{r["scene"]}/targets/{r["target"]}/observations/index.html">RGB-D</a> '
            f'({r["candidates"]}후보, {r["views"]}시점, {r["search_outcome"]})</li>'
            for r in manifest["targets"])
        (output / "index.html").write_text(
            '<meta charset="utf-8">'
            '<h1>토마토 학습 데이터 수집 (GLB 5종)</h1>'
            f'<p>Y축 ±20° / segment {SEGMENT_MIN}~{SEGMENT_MAX} / '
            f'candidates={manifest["candidates_per_target"]} / '
            f'views={manifest["views_per_target"]}</p><ul>'
            + cards
            + f'</ul><a href="collection.json">manifest</a>')
    return manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path,
                   default=Path("/root/docker_share/mujoko_debugging_data") /
                   (datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + "_training_dataset"))
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--candidates", type=int, default=20,
                   help="열매당 물리 후보 실험 횟수")
    p.add_argument("--views-limit", type=int, default=VIEWS_LIMIT,
                   help="열매당 관측 이미지 수 (prepare_observations --limit)")
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--planning-workers", type=int, default=4)
    p.add_argument("--postprocess-workers", type=int, default=4)
    p.add_argument("--hz", type=int, default=240)
    p.add_argument("--truss-scale", type=float, default=0.5,
                   help="송이 크기 배율 (0.5=반 크기)")
    p.add_argument("--idle-seconds", type=float, default=2.0,
                   help="장면 초기 물리 검사 시간(초)")
    p.add_argument("--source-dir", type=Path,
                   default=Path(__file__).resolve().parents[2] /
                   "nvidia-sim/env_usd/tomato_rotate_glb",
                   help="GLB 파일 디렉토리")
    p.add_argument("--no-render", action="store_true",
                   help="장면 preview.png 렌더링 생략")
    p.add_argument("--resume", action="store_true",
                   help="완료된 대상은 건너뛰고 이어서 수집")
    p.add_argument("--smoke", action="store_true",
                   help="smoke 테스트: cyan 1종 · Tomato_05만 · candidates=2 · views=9")
    args = p.parse_args()

    log_status(f"=== 토마토 학습 데이터 수집 시작 ===")
    log_status(f"  GLB {len(GLB_VARIANTS)}종 × 열매 10개 × "
               f"후보 {args.candidates}회 × 관측 {args.views_limit}장")
    log_status(f"  Y축 {ANGLE_MIN}~{ANGLE_MAX}° (±20° 동등) | "
               f"segment {SEGMENT_MIN}~{SEGMENT_MAX}")
    log_status(f"  출력 디렉토리: {args.output}")
    if args.smoke:
        log_status("  [SMOKE 모드] cyan 1종 · Tomato_05 · candidates=2 · views=9")

    result = collect(args)
    total = sum(t.get("views", 0) for t in result.get("targets", []))
    log_status(f"=== 완료: 상태={result['status']} | 총 이미지={total}장 ===")
    log_status(f"  결과: {args.output}/index.html")


if __name__ == "__main__":
    main()
