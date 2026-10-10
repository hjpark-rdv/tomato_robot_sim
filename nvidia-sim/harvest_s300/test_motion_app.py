"""Static bootstrap tests: never import or launch a native Kit app."""
import os
from pathlib import Path
import tempfile
import tomllib
import unittest
from unittest.mock import patch

from motion_app import (PHYSICS_ONLY_APP_ARGS, PHYSICS_ONLY_EXTENSIONS,
                        create_motion_app, motion_app_options)


class FakeSimulationApp:
    def __init__(self, config, **kwargs):
        self.config = config
        self.kwargs = kwargs


class MotionAppTests(unittest.TestCase):
    def test_default_config_is_exact_existing_constructor(self):
        for headless in (False, True):
            for performance in ('baseline', 'optimized'):
                app = create_motion_app(FakeSimulationApp, headless=headless,
                                        performance=performance)
                self.assertEqual(app.config, dict(headless=headless, width=1280,
                    height=800, renderer='RaytracedLighting',
                    disable_viewport_updates=headless and performance == 'optimized'))
                self.assertEqual(app.kwargs, {})

    def test_physics_bootstrap_explicit_none_and_installed_extension_roots(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for folder in ('exts', 'extscache', 'extsDeprecated'):
                (root/folder).mkdir()
            with patch.dict(os.environ, {'ISAAC_PATH': str(root)}):
                app = create_motion_app(FakeSimulationApp, headless=True, physics_only=True)
            self.assertEqual(app.kwargs, {'experience': None})
            self.assertFalse(app.config['create_new_stage'])
            self.assertFalse(app.config['multi_gpu'])
            args = app.config['extra_args']
            enabled = [args[i+1] for i,value in enumerate(args) if value == '--enable']
            self.assertEqual(tuple(enabled), PHYSICS_ONLY_EXTENSIONS)
            roots = [args[i+1] for i,value in enumerate(args) if value == '--ext-folder']
            self.assertEqual(roots, [str(root/x) for x in ('exts','extscache','extsDeprecated')])
            for arg in PHYSICS_ONLY_APP_ARGS:
                self.assertIn(arg,args)
            for banned in ('omni.hydra.rtx','omni.kit.renderer.core','omni.kit.viewport.window',
                           'omni.replicator.core','omni.syntheticdata'):
                self.assertNotIn(banned,enabled)

    def test_gui_and_missing_sdk_rejected_before_app_creation(self):
        calls=[]
        factory=lambda *a,**k:calls.append((a,k))
        with self.assertRaisesRegex(ValueError,'headless'):
            create_motion_app(factory,headless=False,physics_only=True)
        with patch.dict(os.environ, {}, clear=True), self.assertRaisesRegex(ValueError,'ISAAC_PATH'):
            create_motion_app(factory,headless=True,physics_only=True)
        with tempfile.TemporaryDirectory() as tmp, self.assertRaisesRegex(ValueError,'Missing'):
            motion_app_options(headless=True,physics_only=True,sdk_path=tmp)
        self.assertFalse(calls)

    def test_no_physics_parameters_or_unknown_no_renderer_switches(self):
        # A bootstrap must not bypass physical validation or silently alter the
        # existing timestep, solver, margins, friction, collision masks or device.
        settings='\n'.join(PHYSICS_ONLY_APP_ARGS)
        for token in ('solver','armature','timestep','suppressReadback','margin','friction',
                      'collision','cudaDevice','noRendering','renderer/enabled='):
            self.assertNotIn(token,settings)

    def test_installed_required_dependency_closure_excludes_rtx(self):
        root=Path(os.environ.get('ISAAC_PATH','/root/isaac-sim-6.1.0'))
        if not (root/'exts/isaacsim.core.simulation_manager/config/extension.toml').is_file():
            self.skipTest('Installed Isaac SDK unavailable for static dependency analysis')
        configs={}
        for folder in ('exts','extscache','extsDeprecated','kit/exts','kit/extscore'):
            for path in (root/folder).glob('*/config/extension.toml'):
                name=path.parents[1].name.split('-')[0] if folder=='extscache' else path.parents[1].name
                try:
                    value=tomllib.loads(path.read_text())
                except (UnicodeDecodeError,tomllib.TOMLDecodeError):
                    continue
                configs.setdefault(name,value)
        seen=set();pending=list(PHYSICS_ONLY_EXTENSIONS)
        while pending:
            name=pending.pop()
            if name in seen:
                continue
            self.assertIn(name,configs,'Required local extension not found: '+name)
            seen.add(name)
            for child,value in configs[name].get('dependencies',{}).items():
                # This is a static unfiltered closure, not a substitute for Kit's
                # version/platform resolver or a runtime memory measurement.
                if not child.startswith('filter:') and not value.get('optional'):
                    pending.append(child)
        for name in ('omni.hydra.rtx','omni.kit.renderer.core','omni.kit.viewport.window',
                     'omni.replicator.core','omni.syntheticdata'):
            self.assertNotIn(name,seen)
        self.assertIn('omni.gpu_foundation',seen)  # no misleading zero-GPU promise


if __name__ == '__main__':
    unittest.main()
