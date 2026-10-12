import unittest

from quality_profiles import quality_profile


class QualityProfileTests(unittest.TestCase):
    def test_existing_fast_resolution_and_unchanged_physics(self):
        p=quality_profile('fast')
        self.assertEqual((p['backend'],p['width'],p['height']),('classic',960,720))
        self.assertFalse(p['physics_changed'])

    def test_ultra_uses_real_supersampling_and_separate_gui_dimensions(self):
        video=quality_profile('ultra');gui=quality_profile('ultra',gui=True)
        self.assertEqual((video['width'],video['height']),(3840,2160))
        self.assertEqual(video['internal_width'],7680)
        self.assertEqual((gui['width'],gui['height']),(1920,1080))
        self.assertFalse(video['offscreen_msaa_claimed'])
        self.assertEqual(video['backend'],'filament')

    def test_explicit_sizes_do_not_change_preset_or_physics(self):
        p=quality_profile('balanced',width=800,height=600,fps=20)
        self.assertEqual((p['width'],p['height'],p['fps']),(800,600,20))
        p['numeric_flags']['filament.ao.enabled']=0
        self.assertEqual(quality_profile('balanced')['numeric_flags']['filament.ao.enabled'],1)
        self.assertFalse(p['physics_changed'])

    def test_invalid_options_are_rejected_before_renderer_is_created(self):
        for kwargs in (dict(width=0),dict(height=-1),dict(fps=float('nan')),
                       dict(width=2.5),dict(fps=True)):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):quality_profile('ultra',**kwargs)
        with self.assertRaises(ValueError):quality_profile('unknown')


if __name__=='__main__':unittest.main()
