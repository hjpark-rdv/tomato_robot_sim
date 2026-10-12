"""Display presets for saved-state replay; never physics presets.

Filament's official offscreen target disables MSAA. Ultra therefore uses real
2x spatial supersampling rather than claiming that its MSAA flag is effective.
"""
from copy import deepcopy
import math


PROFILES={
    'fast':dict(backend='classic',video_size=(960,720),gui_size=(1280,720),
                supersample=1,restore_source_visuals=False,numeric_flags={},text_flags={}),
    'balanced':dict(backend='filament',video_size=(1920,1080),gui_size=(1280,720),
        supersample=1,restore_source_visuals=True,
        numeric_flags={'filament.ao.enabled':1,'filament.ao.quality':2,
            'filament.ao.low_pass_filter':2,'filament.ao.upsampling':2,
            'filament.ao.bent_normals':0,'filament.ao.ssct':0,
            'filament.shadows.type':0,'filament.shadows.map_size':2048,
            'filament.cg.exposure':1.,'filament.bloom.enabled':0},
        text_flags={'filament.cg.tone_mapping':'pbr_neutral'}),
    'ultra':dict(backend='filament',video_size=(3840,2160),gui_size=(1920,1080),
        supersample=2,restore_source_visuals=True,
        numeric_flags={'filament.ao.enabled':1,'filament.ao.quality':3,
            'filament.ao.low_pass_filter':3,'filament.ao.upsampling':3,
            'filament.ao.bent_normals':1,'filament.ao.ssct':1,
            'filament.shadows.type':3,'filament.shadows.map_size':8192,
            'filament.cg.exposure':1.,'filament.bloom.enabled':0},
        text_flags={'filament.cg.tone_mapping':'pbr_neutral'}),
}


def quality_profile(name='fast', *, gui=False, width=None, height=None, fps=None):
    if name not in PROFILES:raise ValueError('Unknown display quality: '+str(name))
    profile=deepcopy(PROFILES[name])
    defaults=profile['gui_size'] if gui else profile['video_size']
    width=defaults[0] if width is None else width
    height=defaults[1] if height is None else height
    fps=30 if fps is None else fps
    for label,value in (('width',width),('height',height),('fps',fps)):
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0 or int(value)!=value:
            raise ValueError('Display '+label+' must be a positive integer')
    profile.update(name=name,width=int(width),height=int(height),fps=int(fps),
        internal_width=int(width)*profile['supersample'],
        internal_height=int(height)*profile['supersample'],
        physics_changed=False,offscreen_msaa_claimed=False)
    return profile
