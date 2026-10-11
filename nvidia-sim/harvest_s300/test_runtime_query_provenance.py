"""The runner preserves lazy-import source bytes without changing launch files."""
import ast
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import native_leaf_scene_query


class RuntimeQueryProvenanceTests(unittest.TestCase):
    def writer(self):
        source=Path(__file__).with_name('run_motion.py');tree=ast.parse(source.read_text())
        nodes=[n for n in ast.walk(tree)if isinstance(n,ast.If) and ast.unparse(n.test)=='leaf_geometry is not None'
               and any(isinstance(x,ast.Constant) and x.value=='leaf_contact_resolution.json' for x in ast.walk(n))]
        self.assertEqual(len(nodes),1)
        self.assertFalse(any(isinstance(n,(ast.Import,ast.ImportFrom)) for n in ast.walk(nodes[0])))
        return compile(ast.Module(body=nodes,type_ignores=[]),str(source),'exec')

    def environment(self,output,modules):
        return dict(a=SimpleNamespace(output=output),json=json,
                    sys=SimpleNamespace(modules=modules),
                    leaf_geometry=SimpleNamespace(records=[],geometry_metadata=lambda:{},policy_metadata=lambda:{}))

    def test_runner_does_not_import_unused_helper_or_write_fake_provenance(self):
        with TemporaryDirectory() as directory:
            output=Path(directory);exec(self.writer(),self.environment(output,{}))
            self.assertTrue((output/'leaf_contact_resolution.json').exists())
            self.assertFalse((output/'runtime_code_sources').exists())
            self.assertFalse((output/'native_query_source_provenance.json').exists())

    def test_runner_saves_imported_bytes_and_preserves_distinct_launch_snapshot(self):
        captured=b'# immutable bytes captured when the actual helper was lazily imported\n'
        digest=hashlib.sha256(captured).hexdigest()
        with TemporaryDirectory() as directory:
            output=Path(directory);launch=output/'code'/'native_leaf_scene_query.py'
            launch.parent.mkdir();launch.write_bytes(b'# older launch snapshot\n')
            with patch.object(native_leaf_scene_query,'SOURCE_BYTES',captured),patch.object(native_leaf_scene_query,'SOURCE_SHA256',digest):
                exec(self.writer(),self.environment(output,{'native_leaf_scene_query':native_leaf_scene_query}))
            provenance=json.loads((output/'native_query_source_provenance.json').read_text())
            self.assertEqual(provenance['module_import_source_sha256'],digest)
            self.assertEqual((output/'runtime_code_sources'/'native_leaf_scene_query.py').read_bytes(),captured)
            self.assertEqual(launch.read_bytes(),b'# older launch snapshot\n')
            self.assertTrue(provenance['source_bytes_captured_at_import'])
            self.assertFalse(provenance['launch_snapshot_overwritten'])


if __name__=='__main__':unittest.main()
