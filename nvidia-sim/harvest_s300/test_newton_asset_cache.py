"""CPU/filesystem controls for exact-byte, non-destructive local model staging."""
import hashlib
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import newton_asset_cache as cache


class LocalModelCacheControls(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "source.mjb"
        self.payload = b"model bytes\x00\xff" * 17
        self.source.write_bytes(self.payload)
        self.sha = hashlib.sha256(self.payload).hexdigest()
        self.directory = self.root / "cache"
        self.final = self.directory / (self.sha + ".mjb")

    def stage(self):
        return cache.verify_or_stage_model(self.source, self.sha, self.directory)

    def assert_no_temporary_entries(self):
        self.assertEqual(list(self.directory.glob(".model-*.tmp")), [])

    def existing(self, payload=None):
        self.directory.mkdir()
        self.final.write_bytes(self.payload if payload is None else payload)

    def test_miss_streams_exact_bytes_then_atomically_publishes(self):
        self.payload = bytes(range(256)) * (cache._CHUNK // 256 * 2 + 3)
        self.source.write_bytes(self.payload)
        self.sha = hashlib.sha256(self.payload).hexdigest()
        self.final = self.directory / (self.sha + ".mjb")
        link = os.link

        def inspect_publish(source, destination, **kwargs):
            self.assertEqual((self.directory / source).read_bytes(), self.payload)
            self.assertFalse(self.final.exists())
            return link(source, destination, **kwargs)

        with mock.patch.object(cache.os, "link", side_effect=inspect_publish):
            local, evidence = self.stage()
        self.assertEqual(local, self.final)
        self.assertEqual(local.read_bytes(), self.payload)
        self.assertEqual(self.source.read_bytes(), self.payload)
        self.assertTrue(evidence["source_verified"])
        self.assertTrue(evidence["cache_verified"])
        self.assertTrue(evidence["cache_populated"])
        self.assertFalse(evidence["cache_hit"])
        self.assertFalse(evidence["existing_files_overwritten"])
        self.assertFalse(evidence["model_bytes_changed"])
        self.assertEqual(evidence["source_bytes"], len(self.payload))
        self.assertEqual(evidence["cache_bytes"], len(self.payload))
        self.assertEqual(evidence["source_sha256"], self.sha)
        self.assertEqual(evidence["cache_sha256"], self.sha)
        self.assertEqual(local.stat().st_mode & 0o777, 0o444)
        self.assert_no_temporary_entries()

    def test_read_only_hit_verifies_both_files_without_mutating_cache(self):
        self.existing()
        self.final.chmod(0o444)
        self.directory.chmod(0o555)
        self.addCleanup(self.directory.chmod, 0o755)
        with mock.patch.object(cache, "_hash_stream", wraps=cache._hash_stream) as hashes, \
             mock.patch.object(cache.os, "link", side_effect=AssertionError("cache write")), \
             mock.patch.object(cache.os, "mkdir", side_effect=AssertionError("cache write")), \
             mock.patch.object(cache.os, "unlink", side_effect=AssertionError("cache write")), \
             mock.patch.object(cache.os, "fsync", side_effect=AssertionError("cache write")):
            local, evidence = self.stage()
        self.assertEqual(hashes.call_count, 2)
        self.assertEqual(local, self.final)
        self.assertTrue(evidence["cache_hit"])
        self.assertFalse(evidence["cache_populated"])

    def test_valid_cache_does_not_bypass_corrupt_source(self):
        self.existing()
        self.source.write_bytes(b"changed source")
        with self.assertRaisesRegex(ValueError, "Source model SHA mismatch"):
            self.stage()
        self.assertEqual(self.final.read_bytes(), self.payload)
        self.assert_no_temporary_entries()

    def test_corrupt_existing_cache_is_preserved_and_rejected(self):
        self.existing(b"do not overwrite this file")
        with self.assertRaisesRegex(ValueError, "cache SHA mismatch"):
            self.stage()
        self.assertEqual(self.final.read_bytes(), b"do not overwrite this file")
        self.assert_no_temporary_entries()

    def test_source_mismatch_removes_only_own_temporary_file(self):
        self.source.write_bytes(b"incorrect source")
        self.directory.mkdir()
        other = self.directory / "unrelated.mjb"
        other.write_bytes(b"user data")
        with self.assertRaisesRegex(ValueError, "Source model SHA mismatch"):
            self.stage()
        self.assertEqual(list(self.directory.iterdir()), [other])
        self.assertEqual(other.read_bytes(), b"user data")

    def test_cache_entry_symlink_is_rejected_without_touching_target(self):
        self.directory.mkdir()
        target = self.root / "user-file"
        target.write_bytes(self.payload)
        self.final.symlink_to(target)
        with self.assertRaisesRegex(ValueError, "non-symlink"):
            self.stage()
        self.assertTrue(self.final.is_symlink())
        self.assertEqual(target.read_bytes(), self.payload)

    def test_cache_directory_symlink_is_rejected(self):
        target = self.root / "real-directory"
        target.mkdir()
        self.directory.symlink_to(target, target_is_directory=True)
        with self.assertRaises(OSError):
            self.stage()
        self.assertEqual(list(target.iterdir()), [])

    def test_cache_ancestor_symlink_is_rejected(self):
        target = self.root / "real-directory"
        target.mkdir()
        self.directory.symlink_to(target, target_is_directory=True)
        with self.assertRaises(OSError):
            cache.verify_or_stage_model(self.source, self.sha, self.directory / "nested")
        self.assertEqual(list(target.iterdir()), [])

    def test_source_symlink_verifies_resolved_file_and_preserves_both_origins(self):
        target = self.root / "real-model.mjb"
        self.source.rename(target)
        self.source.symlink_to(target)
        local, evidence = self.stage()
        self.assertEqual(local.read_bytes(), self.payload)
        self.assertTrue(self.source.is_symlink())
        self.assertEqual(evidence["source_path"], str(self.source))
        self.assertEqual(evidence["origin_requested"], str(self.source))
        self.assertEqual(evidence["origin_resolved"], str(target))
        self.assertEqual(target.read_bytes(), self.payload)
        self.assert_no_temporary_entries()

    def test_source_symlink_target_must_match_expected_hash(self):
        target = self.root / "real-model.mjb"
        target.write_bytes(b"incorrect model")
        self.source.unlink()
        self.source.symlink_to(target)
        with self.assertRaisesRegex(ValueError, "Source model SHA mismatch"):
            self.stage()
        self.assertTrue(self.source.is_symlink())
        self.assertEqual(target.read_bytes(), b"incorrect model")
        self.assert_no_temporary_entries()

    def test_source_symlink_target_must_exist(self):
        self.source.unlink()
        self.source.symlink_to(self.root / "missing.mjb")
        with self.assertRaises(FileNotFoundError):
            self.stage()
        self.assertTrue(self.source.is_symlink())
        self.assertFalse(self.directory.exists())

    def test_nonregular_cache_entry_is_rejected_without_waiting_for_fifo(self):
        self.directory.mkdir()
        os.mkfifo(self.final)
        with self.assertRaisesRegex(ValueError, "regular"):
            self.stage()
        self.assertTrue(self.final.exists())

    def test_nonregular_source_is_rejected_without_waiting_for_fifo(self):
        self.source.unlink()
        os.mkfifo(self.source)
        with self.assertRaisesRegex(ValueError, "regular"):
            self.stage()
        self.assert_no_temporary_entries()

    def test_invalid_digest_is_rejected_before_creating_cache(self):
        for digest in (None, "0" * 63, "g" * 64, "../" + "0" * 61):
            with self.subTest(digest=digest):
                with self.assertRaisesRegex(ValueError, "64 hexadecimal"):
                    cache.verify_or_stage_model(self.source, digest, self.directory)
        self.assertFalse(self.directory.exists())

    def test_uppercase_digest_uses_same_content_address(self):
        local, evidence = cache.verify_or_stage_model(self.source, self.sha.upper(), self.directory)
        self.assertEqual(local, self.final)
        self.assertEqual(evidence["expected_sha256"], self.sha)

    def test_valid_concurrent_winner_is_verified_and_preserved(self):
        def concurrent_write(*args, **kwargs):
            self.final.write_bytes(self.payload)
            raise FileExistsError("another writer won")

        with mock.patch.object(cache.os, "link", side_effect=concurrent_write):
            local, evidence = self.stage()
        self.assertEqual(local.read_bytes(), self.payload)
        self.assertTrue(evidence["cache_hit"])
        self.assertTrue(evidence["concurrent_cache_winner"])
        self.assertFalse(evidence["cache_populated"])
        self.assert_no_temporary_entries()

    def test_corrupt_concurrent_winner_is_preserved_and_rejected(self):
        def concurrent_write(*args, **kwargs):
            self.final.write_bytes(b"concurrent user data")
            raise FileExistsError("another writer won")

        with mock.patch.object(cache.os, "link", side_effect=concurrent_write):
            with self.assertRaisesRegex(ValueError, "cache SHA mismatch"):
                self.stage()
        self.assertEqual(self.final.read_bytes(), b"concurrent user data")
        self.assert_no_temporary_entries()

    def test_temporary_name_collision_does_not_delete_existing_file(self):
        self.directory.mkdir()
        other = self.directory / ".model-fixed.tmp"
        other.write_bytes(b"user temporary file")
        with mock.patch.object(cache.uuid, "uuid4", return_value=SimpleNamespace(hex="fixed")):
            with self.assertRaises(FileExistsError):
                self.stage()
        self.assertEqual(other.read_bytes(), b"user temporary file")
        self.assertFalse(self.final.exists())

    def test_publication_failure_removes_own_temporary_file(self):
        with mock.patch.object(cache.os, "link", side_effect=OSError("publication failed")):
            with self.assertRaisesRegex(OSError, "publication failed"):
                self.stage()
        self.assertEqual(list(self.directory.iterdir()), [])
        self.assertEqual(self.source.read_bytes(), self.payload)

    def test_hash_rejects_file_that_changes_during_read(self):
        with self.source.open("rb") as source:
            class MutatingStream:
                changed = False

                def fileno(inner):
                    return source.fileno()

                def read(inner, count):
                    block = source.read(count)
                    if not inner.changed:
                        inner.changed = True
                        with self.source.open("ab") as output:
                            output.write(b"new content")
                    return block

            with self.assertRaisesRegex(ValueError, "changed while being read"):
                cache._hash_stream(MutatingStream())


if __name__ == "__main__":
    unittest.main()
