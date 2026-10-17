"""CPU-only filesystem controls for parallel manifest verification and links."""
import hashlib
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest import mock

import newton_prepared_assets as assets


class PreparedAssetControls(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        self.source.mkdir()
        self.dest = self.root / "output"
        self.manifest = {}

    def add_file(self, name, data=b"asset\x00\xff"):
        path = self.source / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        self.manifest[name] = {"sha256": hashlib.sha256(data).hexdigest(), "path": str(path)}
        return path

    def verify(self, **kwargs):
        return assets.verify_manifest_files(self.source, self.manifest, **kwargs)

    def link(self, names=None):
        return assets.link_manifest_assets(self.source, self.dest,
                                           self.manifest if names is None else names)

    def test_all_files_including_model_are_hashed_with_stable_result_order(self):
        self.add_file("input_model.mjb", bytes(range(256)) * 5000)
        self.add_file("meshes/a.obj", b"mesh a")
        self.add_file("meshes/sub/b.obj", b"mesh b")
        expected = {name: value["sha256"] for name, value in self.manifest.items()}
        self.assertEqual(self.verify(), expected)
        self.assertEqual(list(self.verify()), list(self.manifest))
        self.assertEqual(self.verify(workers=1), expected)

    def test_default_hashing_runs_concurrently_with_at_most_eight_reads(self):
        for index in range(17):
            self.add_file(f"f{index}.obj", str(index).encode())
        original = assets._hash_file
        gate = threading.Barrier(8)
        lock = threading.Lock()
        state = dict(active=0, peak=0, entered=0)
        calls = []

        def observed(path):
            with lock:
                state["active"] += 1
                state["entered"] += 1
                first_group = state["entered"] <= 8
                state["peak"] = max(state["peak"], state["active"])
                calls.append(path.name)
            try:
                if first_group:
                    gate.wait(timeout=5)
                return original(path)
            finally:
                with lock:
                    state["active"] -= 1

        with mock.patch.object(assets, "_hash_file", side_effect=observed):
            result = self.verify()
        self.assertEqual(state["peak"], 8)
        self.assertEqual(state["active"], 0)
        self.assertCountEqual(calls, self.manifest)
        self.assertEqual(set(result), set(self.manifest))

    def test_worker_limit_is_explicit(self):
        for workers in (0, -1, 9, True, 1.5):
            with self.subTest(workers=workers):
                with self.assertRaisesRegex(ValueError, "between 1 and 8"):
                    self.verify(workers=workers)

    def test_sha_mismatch_is_not_hidden_by_other_successful_workers(self):
        for index in range(12):
            self.add_file(f"f{index}.obj")
        (self.source / "f7.obj").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch: f7.obj"):
            self.verify()

    def test_missing_or_invalid_manifest_digest_is_rejected(self):
        self.add_file("model.mjb")
        for evidence in ({}, {"sha256": "g" * 64}, {"sha256": "0" * 63}, None):
            with self.subTest(evidence=evidence):
                with self.assertRaisesRegex(ValueError, "valid SHA256"):
                    assets.verify_manifest_files(self.source, {"model.mjb": evidence})

    def test_uppercase_sha_is_accepted_and_returns_actual_lowercase_digest(self):
        self.add_file("model.mjb")
        expected = self.manifest["model.mjb"]["sha256"]
        self.manifest["model.mjb"]["sha256"] = expected.upper()
        self.assertEqual(self.verify(), {"model.mjb": expected})

    def test_invalid_paths_are_rejected_before_reading_or_creating_outputs(self):
        sha = hashlib.sha256(b"").hexdigest()
        bad_names = ("/absolute.obj", "../outside.obj", "meshes/../outside.obj",
                     "", ".", "./x.obj", "meshes//x.obj", "meshes/", "x\x00.obj",
                     "C:\\outside.obj", "C:/outside.obj")
        for name in bad_names:
            with self.subTest(name=name):
                with mock.patch.object(assets, "_hash_file", side_effect=AssertionError("unsafe read")):
                    with self.assertRaisesRegex(ValueError, "relative path"):
                        assets.verify_manifest_files(self.source, {name: {"sha256": sha}})
                with self.assertRaisesRegex(ValueError, "relative path"):
                    self.link([name])
                self.assertFalse(self.dest.exists())

    def test_shared_source_file_and_directory_symlinks_are_supported(self):
        real = self.root / "common"
        real.mkdir()
        (real / "x.obj").write_bytes(b"shared")
        (self.source / "meshes").symlink_to(real, target_is_directory=True)
        (self.source / "root.obj").symlink_to(real / "x.obj")
        sha = hashlib.sha256(b"shared").hexdigest()
        self.manifest = {name: {"sha256": sha} for name in ("meshes/x.obj", "root.obj")}
        self.assertEqual(self.verify(), {name: sha for name in self.manifest})
        evidence = self.link()
        self.assertEqual((self.dest / "meshes").readlink(), real)
        self.assertEqual((self.dest / "root.obj").readlink(), real / "x.obj")
        self.assertEqual(evidence["link_count"], 2)

    def test_nested_assets_share_one_top_directory_and_root_files_remain_separate(self):
        self.add_file("neighbor_meshes/a.obj", b"a")
        self.add_file("neighbor_meshes/deeper/b.obj", b"b")
        self.add_file("proxy/c.obj", b"c")
        self.add_file("input_model.mjb", b"compiled")
        self.add_file("source.xml", b"xml")
        verified = self.verify()
        evidence = self.link()
        self.assertEqual(evidence["asset_count"], 5)
        self.assertEqual(evidence["link_count"], 4)
        self.assertEqual(evidence["directory_link_count"], 2)
        self.assertEqual(evidence["root_file_link_count"], 2)
        self.assertFalse(evidence["existing_files_overwritten"])
        self.assertFalse(evidence["asset_bytes_changed"])
        self.assertCountEqual([p.name for p in self.dest.iterdir()],
                              ["neighbor_meshes", "proxy", "input_model.mjb", "source.xml"])
        self.assertTrue(all(path.is_symlink() for path in self.dest.iterdir()))
        self.assertEqual(assets.verify_manifest_files(self.dest, self.manifest), verified)

    def test_caller_can_exclude_mutable_root_files(self):
        self.add_file("meshes/a.obj")
        self.add_file("commands.npz", b"original commands")
        self.add_file("sampled_plan.json", b"original plan")
        self.link(["meshes/a.obj"])
        self.assertFalse((self.dest / "commands.npz").exists())
        self.assertFalse((self.dest / "sampled_plan.json").exists())
        (self.dest / "commands.npz").write_bytes(b"new commands")
        self.assertEqual((self.source / "commands.npz").read_bytes(), b"original commands")

    def test_existing_destination_preflight_does_not_create_other_links(self):
        self.add_file("first.obj")
        self.add_file("meshes/a.obj")
        self.dest.mkdir()
        existing = self.dest / "meshes"
        existing.mkdir()
        user_file = existing / "user.obj"
        user_file.write_bytes(b"keep")
        with self.assertRaisesRegex(FileExistsError, "already exists"):
            self.link()
        self.assertEqual(list(self.dest.iterdir()), [existing])
        self.assertEqual(user_file.read_bytes(), b"keep")

    def test_existing_destination_symlink_even_if_broken_is_preserved(self):
        self.add_file("x.obj")
        self.dest.mkdir()
        existing = self.dest / "x.obj"
        existing.symlink_to(self.root / "absent")
        with self.assertRaises(FileExistsError):
            self.link()
        self.assertEqual(existing.readlink(), self.root / "absent")

    def test_destination_directory_symlink_is_rejected(self):
        self.add_file("x.obj")
        target = self.root / "other-output"
        target.mkdir()
        self.dest.symlink_to(target, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "must not be a symlink"):
            self.link()
        self.assertEqual(list(target.iterdir()), [])

    def test_missing_sources_are_rejected_before_destination_creation(self):
        self.add_file("meshes/a.obj")
        for name in ("missing.obj", "meshes/missing.obj", "missing/x.obj"):
            with self.subTest(name=name):
                with self.assertRaises(FileNotFoundError):
                    self.link(["meshes/a.obj", name])
                self.assertFalse(self.dest.exists())
        with self.assertRaises(FileNotFoundError):
            assets.verify_manifest_files(self.root / "missing", {})
        with self.assertRaises(FileNotFoundError):
            assets.link_manifest_assets(self.root / "missing", self.dest, [])

    def test_nonregular_sources_are_rejected_without_fifo_blocking(self):
        fifo = self.source / "pipe"
        os.mkfifo(fifo)
        self.manifest["pipe"] = {"sha256": "0" * 64}
        with self.assertRaisesRegex(ValueError, "regular file"):
            self.verify()
        with self.assertRaisesRegex(ValueError, "regular file"):
            self.link()
        self.assertFalse(self.dest.exists())

    def test_named_directory_cannot_masquerade_as_root_asset_file(self):
        self.add_file("meshes/a.obj")
        with self.assertRaisesRegex(ValueError, "regular file"):
            self.link(["meshes", "meshes/a.obj"])
        self.assertFalse(self.dest.exists())

    def test_same_source_or_recursive_directory_link_is_rejected(self):
        self.add_file("meshes/a.obj")
        with self.assertRaisesRegex(ValueError, "must differ"):
            assets.link_manifest_assets(self.source, self.source, self.manifest)
        with self.assertRaisesRegex(ValueError, "inside a shared"):
            assets.link_manifest_assets(self.source, self.source / "meshes" / "new", self.manifest)
        self.assertFalse((self.source / "meshes" / "new").exists())

    def test_duplicate_assets_are_rejected_before_creating_output(self):
        self.add_file("a.obj")
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.link(["a.obj", "a.obj"])
        self.assertFalse(self.dest.exists())

    def test_concurrent_destination_creation_is_never_overwritten(self):
        self.add_file("x.obj")

        def raced_create(target, name, **kwargs):
            (self.dest / name).write_bytes(b"other writer")
            raise FileExistsError(name)

        with mock.patch.object(assets.os, "symlink", side_effect=raced_create):
            with self.assertRaises(FileExistsError):
                self.link()
        self.assertEqual((self.dest / "x.obj").read_bytes(), b"other writer")


if __name__ == "__main__":
    unittest.main()
