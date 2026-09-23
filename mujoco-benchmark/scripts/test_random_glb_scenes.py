import numpy as np
import json

from generate_random_glb_scenes import GLB_DIR, profiles, sample_placements
from robot_engine import RobotEngine
from benchmark_mujoco import HOME


def test_seed_repeats_sources_positions_and_y_angles():
    variants = profiles(GLB_DIR)
    first = sample_placements(np.random.default_rng(np.random.SeedSequence([23, 4])),
                              variants, 3, 4, 13, 10, 170)
    repeat = sample_placements(np.random.default_rng(np.random.SeedSequence([23, 4])),
                               variants, 3, 4, 13, 10, 170)
    changed = sample_placements(np.random.default_rng(np.random.SeedSequence([23, 5])),
                                variants, 3, 4, 13, 10, 170)
    assert first == repeat
    assert first != changed
    assert len({p["source_glb"] for p in first}) == 3
    assert all(4 <= p["stem_segment"] <= 13 and .15 <= p["stem_fraction"] <= .85
               and 10 <= p["y_deg"] <= 170 for p in first)


def test_white_uses_its_mechanics_profile():
    variants = profiles(GLB_DIR)
    white = next(v for v in variants if v["path"].name.endswith("curve_white.glb"))
    assert white["remove_fruits"] == [6]
    assert 2 in white["fruit_offsets"]
    assert white["rachis_stiffness_scale"] == 2
    assert all(v["rachis_stiffness_scale"] == 1 for v in variants if v is not white)


def test_multi_scene_schema_retains_rollout_collision_metrics(tmp_path):
    reference = json.loads((HOME / "assets/reference/reference.json").read_text())
    reference["schema"] = "random_glb_plant_v1"
    path = tmp_path / "reference.json"
    path.write_text(json.dumps(reference))
    result = RobotEngine(reference=path).rollout(seconds=.05)
    assert result["initial_glb_penetration_m"] == 0
    assert result["max_glb_penetration_m"] == 0
    assert result["glb_physics_valid"] is True
