#!/usr/bin/env python3
"""Read-only authored-color SIM-GT labels, never a camera ripeness detector."""
import argparse
import colorsys
import json
from pathlib import Path

from harvest_group_policy import sha


def prepare(assembly, output, minimum=3):
    from pxr import Usd
    if isinstance(minimum,bool) or not isinstance(minimum,int) or not 2<=minimum<=10:
        raise ValueError('Minimum fruit count must be 2..10')
    assembly=Path(assembly);output=Path(output)
    manifest=json.loads((assembly/'motion_manifest.json').read_text())
    stage=Usd.Stage.Open(str(assembly/'composed.usdc'))
    rows=[]
    for fruit in manifest['plant']['fruits']:
        prim=stage.GetPrimAtPath(fruit['path']+'/Collider')
        color=prim.GetAttribute('primvars:displayColor').Get() if prim else None
        if color is None or len(color)!=1:
            raise ValueError('Missing unique authored fruit color: '+fruit['name'])
        rgb=list(map(float,color[0]))
        if not all(0<=v<=1 for v in rgb):raise ValueError('Invalid RGB color')
        # Check color against an actual visual fruit component, not the calyx
        # or the shared collider material (which may have another fruit color).
        visuals=[p for p in Usd.PrimRange(stage.GetPrimAtPath(fruit['path']))
                 if p.GetName().startswith('Visual_')]
        matches=[str(p.GetPath()) for p in visuals
                 if p.GetAttribute('primvars:displayColor').Get()==color]
        if not matches:raise ValueError('Collider color lacks matching visual evidence')
        h,s,v=colorsys.rgb_to_hsv(*rgb)
        eligible=(h*360<=20 or h*360>=340) and s>=.4 and v>=.15
        rows.append(dict(name=fruit['name'],path=fruit['path'],joint=fruit['joint'],
                         authored_rgb=rgb,hue_deg=h*360,matching_visuals=matches,
                         eligible=eligible))
    policy=dict(schema='s300-harvest-group-policy-v1',minimum_fruits=minimum,
        source_sha256={n:sha(assembly/n) for n in ('motion_manifest.json','composed.usdc')},
        eligibility_rule='Authored RGB hue within20deg of red, saturation>=0.4, value>=0.15',
        scope='SIM-GT color proxy only; no biological calibration or RGB detector; unknown color rejects',
        footprint='Existing96mm jaw opening,158mm belt depth,48mm belt center-height interval',
        fruit_eligibility=rows,training_eligible=False,harvest_success=None)
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream:json.dump(policy,stream,indent=2)
    return policy


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('assembly',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--min-fruits',type=int,default=3)
    a=p.parse_args();result=prepare(a.assembly,a.output,a.min_fruits)
    print(json.dumps(dict(output=str(a.output),eligible=[r['name'] for r in result['fruit_eligibility'] if r['eligible']]),indent=2))
