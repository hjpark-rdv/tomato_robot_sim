"""Regression tests for the explicit neighboring-truss obstacle contract."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from add_neighbor_truss_obstacles import (
    COLLISION_CONAFFINITY,
    COLLISION_CONTYPE,
    REQUIRED_KINDS,
    append_fixed_colliders,
    collision_template,
    coverage_from_names,
    obstacle_kind,
    _neighbor_layout_metadata,
)
from environment_preflight import classify_environment_geom


def tiny_model():
    xml = r"""
<mujoco>
  <asset>
    <mesh name="fruit_mesh"
          vertex="0 0 0  .01 0 0  0 .01 0  0 0 .01"
          face="0 1 2  0 1 3  0 2 3  1 2 3"/>
  </asset>
  <worldbody>
    <body name="TRUSS_Truss_01_Peduncle_00">
      <geom name="peduncle_col" type="capsule" fromto="0 0 0  0 0 .05"
            size=".002" contype="1" conaffinity="15"/>
      <geom name="peduncle_visual" type="capsule" fromto=".01 0 0  .01 0 .05"
            size=".002" contype="0" conaffinity="0"/>
    </body>
    <body name="TRUSS_Rachis_00">
      <geom name="rachis_col" type="capsule" fromto="0 0 0  .05 0 0"
            size=".002" contype="1" conaffinity="15"/>
    </body>
    <body name="TRUSS_Pedicel_proximal_01_00">
      <geom name="pedicel_col" type="capsule" fromto="0 0 0  0 .03 0"
            size=".001" contype="1" conaffinity="15"/>
    </body>
    <body name="Tomato_01">
      <geom name="fruit_col" type="mesh" mesh="fruit_mesh"
            contype="1" conaffinity="15"/>
    </body>
  </worldbody>
</mujoco>
"""
    model = mj.MjModel.from_xml_string(xml)
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    return model, data


def test_obstacle_kind_matches_user_contract():
    assert obstacle_kind("Tomato_03") == "fruit"
    assert obstacle_kind("TRUSS_Rachis_04") == "rachis"
    assert obstacle_kind("TRUSS_Pedicel_proximal_03_00") == "pedicel"
    assert obstacle_kind("Attachment_02") == "pedicel"
    assert obstacle_kind("TRUSS_Truss_01_Peduncle_00") == "peduncle"
    assert obstacle_kind("STEM_MainStem_04") is None


def test_collision_template_extracts_physical_truss_not_visual_meshes():
    model, data = tiny_model()
    rows = collision_template(model, data, np.zeros(3))
    assert {row["kind"] for row in rows} == set(REQUIRED_KINDS)
    assert {row["source_geom"] for row in rows} == {
        "peduncle_col", "rachis_col", "pedicel_col", "fruit_col"
    }
    assert all(row["source_geom"] != "peduncle_visual" for row in rows)
    fruit = next(row for row in rows if row["kind"] == "fruit")
    assert fruit["shape"] == "mesh" and len(fruit["vertices_m"]) == 4
    assert next(row for row in rows if row["kind"] == "rachis")["shape"] == "capsule"


def test_fixed_obstacles_are_active_world_geoms_and_keep_semantic_names():
    world = ET.Element("worldbody")
    rows = [
        dict(kind="rachis", source_geom="r", source_body="TRUSS_Rachis_00",
             shape="capsule", endpoints_m=[[0, 0, 0], [0, 0, .1]], radius_m=.002),
        dict(kind="fruit", source_geom="f", source_body="Tomato_01",
             shape="mesh", mesh_asset="fruit_proxy"),
    ]
    inventory = append_fixed_colliders(
        world, rows, [1., 2., 3.], Rotation.identity(), plant_id=4, truss_index=2)
    geoms = world.findall("geom")
    assert len(geoms) == 2 and len(inventory) == 2
    assert geoms[0].get("name").startswith("neighbor_truss_collision_rachis_p04_t02_")
    assert geoms[1].get("name").startswith("neighbor_truss_collision_fruit_p04_t02_")
    assert all(int(g.get("contype")) == COLLISION_CONTYPE for g in geoms)
    assert all(int(g.get("conaffinity")) == COLLISION_CONAFFINITY for g in geoms)
    assert geoms[0].get("fromto") is not None
    assert geoms[1].get("mesh") == "fruit_proxy"


def coverage_fixture():
    return dict(placements=[
        dict(id=2, trusses=[dict(source_glb="a.glb"), dict(source_glb="b.glb")]),
        dict(id=5, trusses=[dict(source_glb="c.glb")]),
    ])


def complete_names(metadata, stems=True, gutter=True):
    names = []
    for plant in metadata["placements"]:
        pid = plant["id"]
        for ti, _ in enumerate(plant["trusses"]):
            for kind in REQUIRED_KINDS:
                names.append(f"neighbor_truss_collision_{kind}_p{pid:02d}_t{ti:02d}_g000")
        if stems:
            names += [f"neighbor_stem_collision_{pid:02d}_{s:02d}" for s in range(16)]
    if gutter:
        names += [f"gutter_collision_{i:02d}" for i in range(18)]
    return names


def test_coverage_audit_is_fail_closed_for_missing_neighbor_obstacle():
    meta = coverage_fixture()
    names = complete_names(meta)
    result = coverage_from_names(meta, names, require_neighbor_stems=True, require_gutter=True)
    assert result["passed"] and result["expected_neighbor_trusses"] == 3
    assert result["leaves_collision"] is False

    missing = names.copy()
    missing.remove("neighbor_truss_collision_pedicel_p05_t00_g000")
    result = coverage_from_names(meta, missing, require_neighbor_stems=True, require_gutter=True)
    assert not result["passed"]
    assert dict(type="neighbor_truss", plant_id=5, truss_index=0, kind="pedicel") in result["missing"]

    missing_stem = names.copy()
    missing_stem.remove("neighbor_stem_collision_02_03")
    assert not coverage_from_names(meta, missing_stem, True, True)["passed"]

    assert not coverage_from_names(meta, names[:-1], True, True)["passed"]


def test_environment_diagnostics_identify_new_obstacle_classes():
    assert classify_environment_geom("neighbor_truss_collision_fruit_p00_t00_g000") == "neighbor_fruit"
    assert classify_environment_geom("neighbor_truss_collision_rachis_p00_t00_g000") == "neighbor_rachis"
    assert classify_environment_geom("neighbor_truss_collision_pedicel_p00_t00_g000") == "neighbor_pedicel"
    assert classify_environment_geom("neighbor_truss_collision_peduncle_p00_t00_g000") == "neighbor_peduncle"


def test_neighbor_layout_is_explicit_and_active_scene_placements_are_not_duplicated(tmp_path):
    active = tmp_path / "active"; active.mkdir()
    (active / "scene.json").write_text('{"placements":[{"stem_segment":4}]}')
    with pytest.raises(ValueError, match="Neighbor layout is ambiguous"):
        _neighbor_layout_metadata(active)

    layout = tmp_path / "layout"; layout.mkdir()
    payload = {"neighbors": 1, "physics_added": False,
               "placements": [{"id": 7, "yaw_deg": 0, "trusses": []}]}
    (layout / "scene.json").write_text(__import__("json").dumps(payload))
    scene_meta, layout_meta, layout_path = _neighbor_layout_metadata(active, layout)
    assert "placements" in scene_meta
    assert layout_meta["placements"][0]["id"] == 7
    assert layout_path == layout.resolve()
