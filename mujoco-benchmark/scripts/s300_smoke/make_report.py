"""Summarize the bounded one-truss experiment, retaining failed controls."""
import argparse
import hashlib
import html
import json
from pathlib import Path

import numpy as np
import mujoco as mj


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def build(root):
    folders=['idle315_discrete60','inward60','verified_inward120','inward240','off120']
    rows=[];repeats={};videos=[]
    for name in folders:
        p=root/name
        result=json.loads((p/'summary.json').read_text())
        for s in result['runs']:
            folder=p/f'repeat_{s["repeat"]:02d}'
            # Reclassify against solver timestamps, before newly triggered release.
            # Early raw summaries used end-of-step release state for that one step.
            m=mj.MjModel.from_binary_path(str(p/'model.mjb'))
            release_times={b['name']:b['release_effective_time_s'] for b in s['breaks']}
            active_force=0.;released_force=0.;active_max=0.;release_max=0.;largest_force=None
            for t,g1,g2,dist,force,_ in json.loads((folder/'contacts.json').read_text())['rows']:
                bodies=[m.body(m.geom_bodyid[g]).name for g in (g1,g2)]
                released=any(t>=release_times.get(b,1e99)-1e-9 for b in bodies)
                pen=max(0.,-dist)
                if released:release_max=max(release_max,pen)
                else:active_max=max(active_max,pen)
                if any(b.startswith('Tool') for b in bodies):
                    if released:released_force=max(released_force,force)
                    else:active_force=max(active_force,force)
                    if largest_force is None or force>largest_force['force_n']:
                        largest_force=dict(t=t,bodies=bodies,force_n=force,penetration_m=pen,released=released)
            # All >1mm plant contacts were recorded; below-threshold maximum remains
            # available in the per-step raw metric even when not in contacts.json.
            active_max=max(active_max,s['active_tool_penetration_m'],s['active_plant_penetration_m'])
            rows.append(dict(experiment=name,**s,states_sha256=digest(folder/'states.npz'),
                             timestamp_reclassified=dict(active_penetration_m=active_max,released_penetration_m=release_max,
                                 active_tool_force_n=active_force,released_tool_force_n=released_force,largest_tool_force=largest_force)))
        if len(result['runs'])>1:
            a=np.load(p/'repeat_00/states.npz');b=np.load(p/'repeat_01/states.npz')
            repeats[name]={k:bool(np.array_equal(a[k],b[k])) for k in a.files}
        for video in p.rglob('*.mp4'):
            videos.append(dict(path=str(video),sha256=digest(video)))
    meta=json.loads((root/'model315/export.json').read_text())
    report=dict(schema='s300-mujoco-315-smoke-v1',root=str(root),results=rows,
                reproducibility=repeats,videos=videos,source=meta,
                training_eligible=False,harvest_success=None,full_physics_validated=False,
                conclusion='120 Hz selected-truss smoke runs faster than real time; full robot migration/equivalence is not established.',
                historical_attempts={
                    'idle240, idle240_implicit, idle315_960':'Pre-discrete attachment instability; all fruit released at idle. Rejected.',
                    'inward315_discrete60, inward315_discrete240':'API sign copied before measured PhysX/MuJoCo direction mapping. Actually outward; excluded from final inward comparison.',
                    'inward120':'First correctly directed 120Hz run. Final verified_inward120 repeats save code hashes and use start-of-step release classification.',
                },
                limitations=[
                    'Only one truss, main stem and prescribed gripper; no robot arm, lift, base, farm, robot tracking or arm collisions.',
                    'Per-mesh tool convex hulls differ from PhysX decomposition; collider equivalence not established.',
                    'Fruit-neck soft spring/break model and fruit inertia are uncalibrated porting approximations.',
                    'Prescribed mocap normal velocity and finite robot inertia are absent; native surfacevel covers tangent transport.',
                    'Gravity settlement differs from original three-truss continuation; initial poses are source geometry.',
                    '60Hz violates the retained 1mm active-contact criterion and releases nine fruit vs four at 120/240Hz. Not a valid speed setting.',
                    'Detached fruit/floor penetration is large, separately reported, not hidden as validated physics.',
                    'Prescribed jaw poses do not yield to new contact: released fruit tool-force spikes reach 315N at 120Hz and 1170N at 240Hz. Not a realistic gripper load result.',
                    'The 36.8s source trace stops during final opening. This is not a completed withdrawal or whole-truss harvest.',
                    'Belt-off still releases fruit by jaw/branch loading. Separation alone does not prove belt rotation harvesting.',
                ])
    (root/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    table=''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in [
        s['experiment']+f' #{s["repeat"]}',s['hz'],f'{s["loop_wall_s"]:.2f}',f'{s["realtime_factor"]:.2f}',
        ', '.join(x['name'].rsplit('_',1)[1] for x in s['breaks']) or '없음',
        f'{max(s["active_tool_penetration_m"],s["active_plant_penetration_m"])*1000:.3f}',
        f'{s["released_penetration_m"]*1000:.2f}'
    ])+'</tr>' for s in rows)
    movie=''.join(f'<section><h3>{html.escape(Path(v["path"]).parent.parent.name)} / {html.escape(Path(v["path"]).name)}</h3>'
                  f'<video controls preload="metadata" src="{html.escape(str(Path(v["path"]).relative_to(root)))}"></video>'
                  f'<p>{html.escape(v["path"])}</p></section>' for v in videos)
    text=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>MuJoCo 3.15 송이 하나 시험</title>
<style>body{{font:16px sans-serif;margin:30px;background:#f5f6f8;color:#17222e}}table{{border-collapse:collapse;background:white}}td,th{{border:1px solid #ccc;padding:8px}}video{{width:min(960px,100%)}}p{{overflow-wrap:anywhere}}section{{margin-top:28px}}.note{{background:#fff0cf;padding:16px}}</style>
<h1>MuJoCo 3.15.0 — 현재 벨트 그리퍼 + 송이 하나</h1>
<p>2026-10-12 / SIM-GT 진단 / training_eligible=false / harvest_success=null</p>
<p class="note">120Hz는 실시간보다 빠르게 동작하고 4개 분리. 60Hz는 빠르지만 관통 기준 초과 및 결과 차이로 채택 불가.
로봇 팔·리프트·기체·농장 전체를 이식한 결과가 아닙니다. 낙하 열매/바닥 관통과 충돌 분해·연결부 모델 차이가 남아 있습니다.
기록 자세를 강제 재생하는 그리퍼에서 분리된 열매 접촉력이 315N(120Hz), 1170N(240Hz)까지 관측되어, 이 힘과 수확 결과를 실물 성능으로 사용할 수 없습니다.</p>
<table><tr><th>시험 / 반복</th><th>Hz</th><th>실행 초</th><th>실시간 배율</th><th>분리 번호</th><th>부착 상태 최대 겹침 mm</th><th>분리 후 최대 겹침 mm</th></tr>{table}</table>
<p>주행 조건은 모두 저장된 36.8초 동작. idle만 5초. 속도는 상태 기록·접촉 검사 포함, 로딩·영상 렌더 제외.
영상을 틀 때 물리를 다시 실행하지 않습니다. 아래는 실제 qpos의 1배속 재생입니다.</p>
{movie}<h2>한계</h2><ul>{''.join('<li>'+html.escape(x)+'</li>' for x in report['limitations'])}</ul>
<p><a href="validation.json">원자료·해시·재현성·전체 시험표 JSON</a></p></html>'''
    (root/'index.html').write_text(text)
    print(json.dumps(dict(reproducibility=repeats,videos=videos),indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();build(a.root)
