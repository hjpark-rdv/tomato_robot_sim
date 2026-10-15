"""Offline regression tests for existing-command Newton input preparation."""
import json
import hashlib
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from newton_inputs import (DOF_NAMES, _snapshot_xml, _verified, command_timeline,
                           compare_physical_model, discard_noncolliding_visual_geoms,
                           excluded_body_evidence, sha256, write_display_evidence)


class NewtonInputTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.manifest = self.directory / "manifest.json"
        self.manifest.write_text(json.dumps(dict(rail=dict(initial_station_y_m=1.5))))
        self.meta = dict(source_trial=str(self.directory), source_manifest=str(self.manifest))
        self.initial = np.zeros(len(DOF_NAMES))

    def tearDown(self):
        self.temporary.cleanup()

    def recorded(self):
        # Row zero is an initial observation, not a command for the first step.
        commands = np.repeat(np.arange(4)[:, None], len(DOF_NAMES), axis=1)
        belts = np.repeat(np.arange(4)[:, None], 2, axis=1) / 10
        np.savez(self.directory / "states.npz", time=np.arange(4) / 60,
                 command=commands, belt_surface_velocity_local_y_m_s=belts,
                 phase=np.asarray(["initial", "a", "b", "c"]))
        return commands, belts

    def plan(self, duration=.105):
        source = dict(settle_seconds=0., phases=[dict(name="close", duration_s=duration,
            waypoint_q=[np.zeros(7).tolist(), np.ones(7).tolist()], end_gap_m=.028,
            belt_local_y_m_s=[.05, .05])])
        (self.directory / "plan.json").write_text(json.dumps(source))

    def test_recorded_commands_belong_to_interval_ending_at_row_time(self):
        commands, belts = self.recorded()
        rows, metadata, _ = command_timeline(self.meta, self.initial, 60, command_source="recorded")
        np.testing.assert_array_equal(rows["command"], commands[1:])
        np.testing.assert_array_equal(rows["authored_belt_speed"], belts[1:])
        np.testing.assert_array_equal(rows["phase"], ["a", "b", "c"])
        self.assertEqual(metadata["steps"], 3)
        self.assertEqual(rows["source_command_record_time_s"][0], 1 / 60)

    def test_recorded_upsampling_holds_each_original_interval(self):
        commands, _ = self.recorded()
        rows, _, _ = command_timeline(self.meta, self.initial, 120, command_source="recorded")
        np.testing.assert_array_equal(rows["command"], np.repeat(commands[1:], 2, axis=0))

    def test_plan_keeps_final_step_for_non_grid_duration(self):
        self.plan()
        rows, metadata, _ = command_timeline(self.meta, self.initial, 100)
        self.assertEqual(metadata["steps"], 11)
        self.assertAlmostEqual(metadata["simulated_s"], .11)
        self.assertLess(rows["source_sample_time_s"][-1], .105)
        np.testing.assert_array_equal(rows["belt_surface_velocity_local_y_m_s"],
                                      -rows["authored_belt_speed"])

    def test_frozen_controller_is_explicitly_not_advanced_physical_state(self):
        self.plan()
        rows, metadata, _ = command_timeline(self.meta, self.initial, 100,
                                             seconds=.03, freeze_phase="close")
        self.assertEqual(len(np.unique(rows["command"], axis=0)), 1)
        self.assertTrue(np.all(rows["phase"] == "close"))
        self.assertFalse(metadata["physical_state_advanced_to_source_start"])
        self.assertFalse(metadata["motion_executed"])
        with self.assertRaises(ValueError):
            command_timeline(self.meta, self.initial, 100, freeze_phase="missing")

    def test_variable_frequency_reuses_free_policy_and_preserves_contact_open_phases(self):
        from run_harvest import phase_frequency
        names = [("prepare_open", .05), ("approach", .02), ("cycle_open", .02),
                 ("cycle_open_hold", .01), ("cycle_reclose", .01),
                 ("cycle_open_final", .01), ("return_joint_branch", .05), ("hold", .05)]
        source = dict(settle_seconds=0., phases=[dict(name=name, duration_s=duration,
            waypoint_q=[np.zeros(7).tolist(), np.ones(7).tolist()],
            end_gap_m=.028, belt_local_y_m_s=[.05, .05]) for name, duration in names])
        (self.directory / "plan.json").write_text(json.dumps(source))
        rows, metadata, _ = command_timeline(self.meta, self.initial, 300, free_hz=60)
        np.testing.assert_array_equal(rows["dt"], np.diff(rows["time_edges"]))
        np.testing.assert_array_equal(rows["time_edges"], rows["time_edges_master_ticks"] / 300)
        self.assertEqual(metadata["master_hz"], 300)
        for phase, dt in zip(rows["phase"], rows["dt"]):
            self.assertAlmostEqual(dt, 1 / phase_frequency(phase, 300, 60), places=14)
        for name in ("cycle_open", "cycle_open_hold", "cycle_reclose", "cycle_open_final"):
            self.assertTrue(np.any(rows["phase"] == name))
            np.testing.assert_allclose(rows["dt"][rows["phase"] == name], 1 / 300, atol=1e-15, rtol=0)
        self.assertLess(rows["time"][-1], metadata["nominal_duration_s"] + 1e-12)
        self.assertGreaterEqual(rows["time_edges"][-1], metadata["nominal_duration_s"] - 1e-12)

    def test_unaligned_phase_boundary_retains_existing_solver_start_policy(self):
        source = dict(settle_seconds=0., phases=[dict(name=name, duration_s=duration,
            waypoint_q=[np.zeros(7).tolist(), np.ones(7).tolist()],
            end_gap_m=.028, belt_local_y_m_s=[0., 0.])
            for name, duration in [("prepare_open", .055), ("approach", .025)]])
        (self.directory / "plan.json").write_text(json.dumps(source))
        rows, _, _ = command_timeline(self.meta, self.initial, 300, free_hz=60)
        first = np.flatnonzero(rows["phase"] == "approach")[0]
        # Existing Session chooses a free step from the phase at its start;
        # it does not shorten that step at a nominal waypoint boundary.
        self.assertAlmostEqual(rows["time"][first], 4 / 60)
        self.assertAlmostEqual(rows["dt"][first], 1 / 300)
        for free in (0, 301, np.nan):
            with self.assertRaises(ValueError):
                command_timeline(self.meta, self.initial, 300, free_hz=free)
        with self.assertRaises(ValueError):
            command_timeline(self.meta, self.initial, 300.5, free_hz=60)

    def test_invalid_duration_and_nonmonotonic_source_rejected(self):
        self.recorded()
        for args in [dict(hz=0), dict(hz=60, seconds=0), dict(hz=60, start_s=1)]:
            with self.assertRaises(ValueError):
                command_timeline(self.meta, self.initial, command_source="recorded", **args)
        np.savez(self.directory / "states.npz", time=[0., .1, .1],
                 command=np.zeros((3, 10)), phase=["initial", "a", "b"],
                 belt_surface_velocity_local_y_m_s=np.zeros((3, 2)))
        with self.assertRaises(ValueError):
            command_timeline(self.meta, self.initial, 60, command_source="recorded")

    def test_source_hash_mismatch_is_not_accepted(self):
        with self.assertRaises(ValueError):
            _verified(self.manifest, "0" * 64)

    def test_display_metadata_preserves_raw_events_and_uses_actual_release_time(self):
        scene = self.directory / "scene.usda"
        scene.write_text("#usda 1.0\n")
        self.manifest.write_text(json.dumps(dict(scene_sha256=sha256(scene))))
        metadata = dict(source_assembly=str(self.directory), source_manifest=str(self.manifest),
                        source_manifest_sha256=sha256(self.manifest),
                        fruit_specs=[dict(name="fruit", joint="/World/ElasticTarget/HarvestJoints/fruit")])
        provenance = dict(source_body_to_native_name={"/World/ElasticTarget/fruit": "fruit"})
        output = self.directory / "trial"
        output.mkdir()
        events = [dict(world=world, name="fruit", solver_time_s=1., release_effective_time_s=1.003)
                  for world in (0, 1)]
        raw = output / "break_events.json"
        raw.write_text(json.dumps(events, indent=4))
        before = raw.read_bytes()
        summary = write_display_evidence(output, metadata, provenance, events)
        self.assertEqual(summary["model_sha256"], sha256(scene))
        self.assertEqual((output / "newton_break_events_all_worlds.json").read_bytes(), before)
        displayed = json.loads(raw.read_text())
        self.assertEqual(len(displayed), 1)
        self.assertEqual(displayed[0]["t"], 1.003)
        self.assertEqual(displayed[0]["joint"], metadata["fruit_specs"][0]["joint"])
        self.assertEqual(json.loads((output / "native_metadata.json").read_text())["robot_dofs"], DOF_NAMES)
        with self.assertRaises(FileExistsError):
            write_display_evidence(output, metadata, provenance, events)

    def test_xml_snapshot_resolves_original_mesh_without_changing_physics(self):
        (self.directory / "meshes").mkdir()
        asset = self.directory / "meshes/body.obj"
        asset.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
        source = self.directory / "scene.xml"
        source.write_text('<mujoco><compiler meshdir="meshes"/><option timestep=".01"/>'
                          '<asset><mesh name="m" file="body.obj"/></asset>'
                          '<worldbody><body name="x"><geom mesh="m" friction=".5 0 0"/>'
                          '</body></worldbody></mujoco>')
        before = sha256(source)
        destination = self.directory / "snapshot.xml"
        dependencies = _snapshot_xml(source, destination)
        root = ET.parse(destination).getroot()
        self.assertEqual(root.find("asset/mesh").get("file"), str(asset))
        self.assertEqual(root.find("option").get("timestep"), ".01")
        self.assertEqual(root.find("worldbody/body/geom").get("friction"), ".5 0 0")
        self.assertEqual(sha256(source), before)
        self.assertEqual(dependencies[0]["sha256"], sha256(asset))
        separated = self.directory / "render_separated.xml"
        _snapshot_xml(source, separated, discard_visual_geoms=True)
        self.assertEqual(ET.parse(separated).getroot().find("compiler").get("discardvisual"), "true")
        self.assertEqual(sha256(source), before)

    def test_discard_visual_only_preserves_physics_by_name_and_rejects_geometry_or_force_edits(self):
        import mujoco as mj
        text = '''<mujoco><compiler inertiafromgeom="false"/><asset>
          <mesh name="physical_tetrahedron" vertex="0 0 0 .1 0 0 0 .1 0 0 0 .1"/>
          </asset><worldbody><body name="arm"><inertial mass="1" pos="0 0 0" diaginertia=".1 .1 .1"/>
          <joint name="hinge" type="hinge"/><geom name="active_mesh" type="mesh" mesh="physical_tetrahedron"/>
          <geom name="visual_only" type="sphere" size="2" contype="0" conaffinity="0"/>
          <site name="anchor" pos="0 0 0"/></body></worldbody><actuator>
          <position name="drive" joint="hinge" kp="10" kv="1" forcerange="-2 2"/>
          </actuator></mujoco>'''
        spec = mj.MjSpec.from_string(text)
        source = spec.compile()
        variant, proof = discard_noncolliding_visual_geoms(source, spec)
        self.assertTrue(proof["restored_compiler_derived_fields"]["dof_length"]["changed_dof_indices"])
        np.testing.assert_array_equal(source.dof_length, variant.dof_length)
        self.assertTrue(proof["passed"])
        self.assertEqual(proof["removed_visual_geom_names"], ["visual_only"])
        self.assertEqual(proof["source_active_geoms"], proof["variant_active_geoms"])
        self.assertEqual(proof["source_semantic_sha256"], proof["variant_semantic_sha256"])
        self.assertEqual(proof["physical_mesh_assets_compared"], 1)
        variant.actuator_forcerange[0, 1] += .1
        with self.assertRaisesRegex(ValueError, "actuator_forcerange"):
            compare_physical_model(source, variant)
        variant.actuator_forcerange[:] = source.actuator_forcerange
        variant.body_mass[1] += .1
        with self.assertRaisesRegex(ValueError, "body_mass"):
            compare_physical_model(source, variant)
        variant.body_mass[:] = source.body_mass
        variant.geom_friction[0, 0] += .1
        with self.assertRaisesRegex(ValueError, "geom_friction"):
            compare_physical_model(source, variant)
        variant.geom_friction[0] = source.geom_friction[source.geom("active_mesh").id]
        variant.mesh_vert[0, 0] += .001
        with self.assertRaisesRegex(ValueError, "mesh_vert"):
            compare_physical_model(source, variant)

    def test_historical_body_omissions_require_native_break_and_checkpoint_proof(self):
        from native_route_checkpoint import _json_bytes
        checkpoint = self.directory / "checkpoint"
        checkpoint.mkdir()
        (checkpoint / "checkpoint.npz").write_bytes(b"historical measured arrays")
        event = dict(joint="fruit", t=25.)
        metadata = dict(break_events=[event], checkpoint_npz_sha256=sha256(checkpoint / "checkpoint.npz"))
        metadata["metadata_sha256"] = hashlib.sha256(_json_bytes(metadata)).hexdigest()
        (checkpoint / "checkpoint.json").write_text(json.dumps(metadata, indent=2))
        path = "/World/RouteFrozen/Plant_22/Truss_02_Fruit_01"
        self.manifest.write_text(json.dumps(dict(route_checkpoint=dict(checkpoint=str(checkpoint),
            outgoing=dict(checkpoint_metadata_sha256=metadata["metadata_sha256"],
                checkpoint_time_s=57., detached_fruits=[dict(path=path,
                    native_break_events=[event], source_plant="Plant_22")])))))
        rows = excluded_body_evidence(self.meta, [path, "/World/Unexpected"])
        self.assertEqual(rows[0]["classification"], "historical_native_detached_fruit")
        self.assertEqual(rows[1]["classification"], "unexplained_portable_export_omission")
        self.assertFalse(rows[0]["current_target_collision_coverage_claim"])
        self.assertNotEqual(rows[0]["checkpoint_metadata"]["file_sha256"], metadata["metadata_sha256"])
        self.assertTrue(rows[0]["historical_provenance_verified"])
        # Changed historical files do not excuse an actual core-input SHA
        # mismatch or remove the inline evidence already in the source manifest.
        metadata["break_events"] = []
        (checkpoint / "checkpoint.json").write_text(json.dumps(metadata))
        changed = excluded_body_evidence(self.meta, [path])[0]
        self.assertEqual(changed["classification"], "historical_native_detached_fruit_unverified_history")
        self.assertFalse(changed["historical_provenance_verified"])
        self.assertEqual(changed["native_break_events"], [event])
        self.assertFalse(changed["checkpoint_metadata"]["metadata_sha_verified"])
        self.assertEqual(changed["checkpoint_metadata"]["expected_metadata_sha256"], metadata["metadata_sha256"])


if __name__ == "__main__":
    unittest.main()
