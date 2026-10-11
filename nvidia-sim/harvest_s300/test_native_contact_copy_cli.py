"""Exercise the actual CLI; contact value/guard equivalence has separate tests."""
from contextlib import redirect_stderr
import io
import unittest

from test_run_motion_conditioning import actual_cli


class NativeContactCopyCliTests(unittest.TestCase):
    def test_opt_in_preserves_physics_and_contact_criteria(self):
        base=vars(actual_cli(['--buffer-contact-events']))
        enabled=vars(actual_cli(['--buffer-contact-events','--native-contact-vector-copy']))
        self.assertFalse(base.pop('native_contact_vector_copy'))
        self.assertTrue(enabled.pop('native_contact_vector_copy'))
        self.assertEqual(base,enabled)

    def test_reject_unsupported_backend_instead_of_silently_ignoring(self):
        for flags in ([],['--buffer-contact-events','--contact-backend','tensor']):
            with self.subTest(flags=flags),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                actual_cli(flags+['--native-contact-vector-copy'])


if __name__=='__main__':unittest.main()
