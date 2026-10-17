import json
from pathlib import Path
import tempfile
import unittest

from run_newton_harvest import write


class FailureReportingTests(unittest.TestCase):
    def test_nonfinite_failure_remains_reportable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'summary.json'
            write(path,dict(status='stopped',stop='nonfinite_physics',finite_recorded_state=False,
                            body=dict(value=float('nan')),force=[float('inf'),.125]))
            record = json.loads(path.read_text())
            self.assertFalse(record['finite_recorded_state'])
            self.assertEqual(record['stop'],'nonfinite_physics')
            self.assertEqual(record['body']['value'],'nan')
            self.assertEqual(record['force'],['inf',.125])


if __name__ == '__main__':
    unittest.main()
