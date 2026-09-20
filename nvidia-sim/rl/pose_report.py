"""Validate and summarize a completed pose dataset/explicit physics-only run."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def report(root):
    summary=json.loads((root/'summary.json').read_text())
    rows=[json.loads(line) for line in (root/'metadata.jsonl').read_text().splitlines()]
    experiment=json.loads((root/'experiment.json').read_text())
    assert len(rows)==summary['total_candidates']
    assert len({r['candidate_id'] for r in rows})==len(rows)
    assert all(r['target_id']=='Tomato_05' and not r['domain_randomization'] for r in rows)
    source_matches={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()==digest
                    for name,digest in experiment['source_sha256'].items()}
    assert all(source_matches.values()),source_matches
    reset_max=0.;captures=0
    for row in rows:
        path=root/row['metadata_path'];on_disk=json.loads(path.read_text())
        assert on_disk['parameters']==row['parameters'] and on_disk['result']==row['result']
        checks=row['reset_check']
        for key in (None,'cross_process','after_observation'):
            check=checks if key is None else checks[key]
            assert check['passed']
            reset_max=max(reset_max,max(check['max_abs_by_field'].values()))
        if row['dataset_complete']:
            import imageio.v2 as imageio
            image=imageio.imread(root/row['rgb']);depth=np.load(root/row['depth'],allow_pickle=False)
            assert image.shape==(720,960,3) and depth.shape==(720,960)
            assert np.any(np.isfinite(depth)&(depth>0))
            captures+=1
        else:
            assert row['rgb'] is None and row['depth'] is None
        if row['hook_success']:
            assert row['retained_hook'] and not row['first_contact_non_target']
            assert not row['target_broken'] and not row['other_broken']
            assert row['hold_ticks']>=60 and row['hold_contact_ticks']>=3
        if row['result']=='ik_or_planning_failure': assert not (path.parent/'trace.json').exists()
    retained=[r for r in rows if r.get('retained_hook')]
    first_contacts=Counter(r.get('first_contact_object') or '(none)' for r in rows)
    details=dict(validated_rows=len(rows),rgbd_captures=captures,dataset_complete=captures==len(rows),
        maximum_initial_state_difference=reset_max,source_matches=source_matches,
        mechanically_retained_target_hook_count=len(retained),
        mechanically_retained_candidates=[dict(candidate_id=r['candidate_id'],strict_result=r['result'],
            first_contact_object=r['first_contact_object'],target_max_displacement_mm=1000*r['target_max_displacement_m'],
            main_stem_max_displacement_mm=1000*r['main_stem_max_displacement_m'],
            hold_contact_ticks=r['hold_contact_ticks'],hold_ticks=r['hold_ticks']) for r in retained],
        first_contact_object_counts=dict(first_contacts))
    negative=root/'negative_control/candidate.json'
    if negative.exists():
        test=json.loads(negative.read_text())
        assert test['result']=='ik_or_planning_failure' and not (negative.parent/'trace.json').exists()
        details['unreachable_negative_control_passed']=True
    (root/'validation_report.json').write_text(json.dumps(details,indent=2)+'\n')
    valid=[r for r in rows if r['result']!='ik_or_planning_failure']
    fig,axes=plt.subplots(2,1,figsize=(13,8),gridspec_kw={'height_ratios':[2,1]})
    x=np.arange(len(valid));target=np.array([r['target_max_displacement_m']*1000 for r in valid]);main=np.array([r['main_stem_max_displacement_m']*1000 for r in valid])
    axes[0].bar(x-.2,target,width=.4,label='Target maximum (60 Hz samples)',color='#d76343')
    axes[0].bar(x+.2,main,width=.4,label='Main-stem maximum (60 Hz samples)',color='#3e84b5')
    axes[0].axhline(20,color='#d76343',ls='--',lw=1);axes[0].axhline(30,color='#3e84b5',ls='--',lw=1)
    axes[0].set_xticks(x,[r['candidate_id'].removeprefix('coarse_') for r in valid],rotation=90,fontsize=8)
    axes[0].set_ylabel('Displacement (mm)');axes[0].legend();axes[0].grid(axis='y',alpha=.2)
    axes[0].set_title('Executed candidates; early-aborted trials are included')
    names=['Strict success','Non-target contact','Miss','Excess displacement','IK/planning failure']
    counts=[summary['primary_category_counts'][k] for k in ('success_target_hook','non_target_contact','miss','excessive_displacement','ik_or_planning_failure')]
    axes[1].barh(names,counts,color=['#3c995d','#d5a637','#999999','#d76343','#576ca5'])
    axes[1].set_xlabel('Primary label count (mutually exclusive)')
    for i,count in enumerate(counts):axes[1].text(count+.1,i,str(count),va='center')
    fig.suptitle('Tomato_05 | fixed elastic scene | '+('RGB-D dataset' if captures==len(rows) else 'CPU physics validation: RGB-D MISSING'))
    fig.tight_layout();fig.savefig(root/'pose_results.png',dpi=160);plt.close(fig)
    lines=['# Tomato_05 pose search results','',
        f"Candidates: {len(rows)}; valid: {summary['valid_candidates']}; strict successes: {summary['success_count']}.",
        f"RGB-D captures: {captures}/{len(rows)}. Dataset complete: {captures==len(rows)}.",
        f"Largest checked initial-state difference: {reset_max:g} (tolerance 1e-6).",'',
        '## Primary categories','', '| Category | Count |','|---|---:|']
    lines.extend(f'| {key} | {value} |' for key,value in summary['primary_category_counts'].items())
    lines+=['','## Mechanical retention versus strict success','',
        f'{len(retained)} candidates mechanically retained the target hook. This does not override non-target-first contact or other failure events.','']
    lines.extend(f"- {r['candidate_id']}: retained, strict result `{r['result']}`, first contact `{r['first_contact_object']}`." for r in retained)
    lines+=['','## Tolerance','',summary['tolerance_status'],
        'Successful pose extrema describe sampled points only, not a guaranteed continuous region.','',
        'Files: results.csv, metadata.jsonl, summary.json, validation_report.json, pose_results.png.',
        'A physics-only run deliberately contains null image paths and is not a completed RGB-D dataset.']
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(details,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(__doc__);parser.add_argument('directory',type=Path)
    report(parser.parse_args().directory)
