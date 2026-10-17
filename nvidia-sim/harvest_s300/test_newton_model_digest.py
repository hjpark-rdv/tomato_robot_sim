"""CPU-only controls: source-model identity retains its original byte format."""
import hashlib
from types import SimpleNamespace
import unittest
from unittest import mock

import mujoco
import numpy as np

from newton_mjcf import _digest_array_bytes, model_array_digest


def legacy_digest(model):
    result = hashlib.sha256()
    for label, obj in (("model", model), ("option", model.opt)):
        for name in sorted(dir(obj)):
            if name.startswith("_"):
                continue
            value = getattr(obj, name)
            if isinstance(value, np.ndarray):
                result.update(f"{label}.{name}:{value.dtype}:{value.shape}".encode())
                result.update(value.tobytes())
            elif isinstance(value, (int, float, bool, np.integer, np.floating)):
                result.update(f"{label}.{name}:{value!r}".encode())
    return result.hexdigest()


class SourceModelDigestControls(unittest.TestCase):
    def test_array_byte_format_matches_for_all_layouts(self):
        base = np.arange(24, dtype=np.float64).reshape(4, 6)
        cases = [base, base.T, base[:, ::2], base[::-1],
                 np.asfortranarray(base), np.zeros((0, 3)), np.array(3.5),
                 np.array([True, False]), np.array([1, 2], dtype=">i4"),
                 np.array([-0., 0., np.nan, np.inf], dtype=np.float32)]
        for value in cases:
            with self.subTest(shape=value.shape, strides=value.strides, dtype=value.dtype):
                actual = hashlib.sha256()
                _digest_array_bytes(actual, value)
                self.assertEqual(actual.digest(), hashlib.sha256(value.tobytes()).digest())

    def test_contiguous_array_reaches_hash_without_materialized_bytes(self):
        value = np.arange(24, dtype=np.float64).reshape(4, 6)
        digest = mock.Mock()
        _digest_array_bytes(digest, value)
        self.assertIs(digest.update.call_args.args[0], value)

    def test_complete_model_digest_retains_names_options_and_detects_changes(self):
        model = SimpleNamespace(nv=3, dof=np.arange(12).reshape(3, 4)[:, ::2],
                                empty=np.zeros((0, 3)), scalar=np.array(2.),
                                opt=SimpleNamespace(tolerance=1e-8, enabled=True,
                                                    wind=np.array([0., 0., 0.])))
        before = legacy_digest(model)
        self.assertEqual(model_array_digest(model), before)
        model.dof[0, 0] = 7
        self.assertNotEqual(model_array_digest(model), before)
        self.assertEqual(model_array_digest(model), legacy_digest(model))
        changed = model_array_digest(model)
        model.opt.tolerance = 1e-7
        self.assertNotEqual(model_array_digest(model), changed)

    def test_native_mujoco_model_has_identical_digest(self):
        model = mujoco.MjModel.from_xml_string("""
            <mujoco>
              <asset><mesh name="tetra" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/></asset>
              <worldbody><body name="moving"><freejoint/>
                <geom type="mesh" mesh="tetra"/>
              </body></worldbody>
            </mujoco>""")
        self.assertEqual(model_array_digest(model), legacy_digest(model))


if __name__ == "__main__":
    unittest.main()
