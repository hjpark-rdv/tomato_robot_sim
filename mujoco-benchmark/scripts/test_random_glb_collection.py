import json
import numpy as np
import pytest

from collect_random_glb_scenes import complete_target, scene_splits
from prepare_observations import views


def test_scene_splits_keep_all_targets_and_views_together():
    assert set(scene_splits(2).values()) == {"smoke"}
    split = scene_splits(10)
    assert len(split) == 10
    assert set(split.values()) == {"train", "validation", "test"}
    assert all(name.startswith("scene_") for name in split)


def test_nine_view_mode_samples_each_image_cell_once():
    nominal = np.eye(4)
    nominal[:3, 3] = [0, 0, -.5]
    k = np.array([[462., 0, 320.], [0, 462., 240.], [0, 0, 1.]])
    first = views(nominal, np.zeros(3), k, seed=42, count=9)
    repeat = views(nominal, np.zeros(3), k, seed=42, count=9)
    assert [row["screen_cell"] for row in first] == list(range(9))
    assert [row["sobol_index"] for row in first] == [row["sobol_index"] for row in repeat]
    assert len(first) == 9
    with pytest.raises(ValueError):
        views(nominal, np.zeros(3), k, seed=42, count=10)


def test_completed_target_requires_matching_source_hash_and_nine_views(tmp_path):
    folder = tmp_path / "Tomato_01"
    (folder / "physics").mkdir(parents=True)
    (folder / "observations").mkdir()
    physics = folder / "physics"
    observations = folder / "observations"
    results = [{"candidate_id": "candidate_00000", "result": "miss"}]
    (physics / "results.json").write_text(json.dumps(results))
    import hashlib
    digest = hashlib.sha256((physics / "results.json").read_bytes()).hexdigest()
    (physics / "manifest.json").write_text(json.dumps(dict(model_sha256="abc", target="Tomato_01", count=1, plan_only=False)))
    (observations / "dataset.json").write_text(json.dumps(dict(schema="farmily_observation_v2",
        source_results_sha256=digest, target="Tomato_01", observations=9, pose_label_actions=1)))
    rows = []
    for index in range(9):
        name = f"view_{index:04d}"
        view = observations / "observations" / name
        view.mkdir(parents=True)
        (view / "rgb.png").write_bytes(b"rgb")
        (view / "depth_aligned_to_color_m.npy").write_bytes(b"depth")
        rows.append(dict(observation_id=name))
    (observations / "observations.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    assert complete_target(folder, "abc", "Tomato_01", 1, 9, False)
    (observations / "observations" / "view_0008" / "rgb.png").unlink()
    assert not complete_target(folder, "abc", "Tomato_01", 1, 9, False)
    (observations / "observations" / "view_0008" / "rgb.png").write_bytes(b"rgb")
    assert not complete_target(folder, "other", "Tomato_01", 1, 9, False)
    assert not complete_target(folder, "abc", "Tomato_01", 1, 71, False)
