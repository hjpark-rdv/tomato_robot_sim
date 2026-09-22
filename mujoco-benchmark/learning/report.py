"""Offline Korean model comparison with per-view recommendations and source labels."""
import html,json
from pathlib import Path
import numpy as np


def generate(out,rows,actions):
    runs=json.loads((out/'results.json').read_text());manifest=json.loads((out/'manifest.json').read_text());dataset=Path(manifest['dataset'])
    names=sorted(set(r['model'] for r in runs));table=[]
    for name in names:
        rr=[r for r in runs if r['model']==name]
        for alteration in (('normal',) if name=='action_only' else ('normal','shuffled_rgbd')):
            mm=[r['metrics']['test/unseen_actions/'+alteration] for r in rr]
            fmt=lambda key,scale=1:f'{np.mean([m[key] for m in mm])*scale:.2f} ± {np.std([m[key] for m in mm])*scale:.2f}'
            table.append(f'<tr><td>{name}</td><td>{"정상 입력" if alteration=="normal" else "다른 관측 RGB-D/K"}</td><td>{fmt("entry_average_precision")}</td><td>{fmt("displacement_mae_mm")}</td><td>{fmt("top1_entry_rate",100)}%</td><td>{fmt("top1_low_displacement_entry_rate",100)}%</td></tr>')
    # Pick by mean validation loss across seeds, then show seed 0, not best test seed.
    selected=min(names,key=lambda name:np.mean([r['validation_loss'] for r in runs if r['model']==name]));demo=next(r for r in runs if r['model']==selected)
    cards=[]
    for top in demo['metrics']['test/unseen_actions/normal']['top']:
        r=rows[top['view_index']];a=actions[top['candidate_index']];oid=r['observation_id'];cid=a['candidate_id']
        image=(dataset/'observations'/oid/'annotated_preview.jpg').as_uri();plan=(Path(manifest['source_run'])/'candidates'/cid/'result.json').as_uri()
        cards.append(f'<article><img src="{image}" alt="{oid}"><h3>{oid} → {cid}</h3><p>예측 진입 {top["predicted_entry_probability"]:.1%} · 예상 이동 {top["predicted_max_displacement_mm"]:.1f} mm</p><p>실제 중심 진입 {"예" if top["actual_center_entered"] else "아니오"} · 이동 {top["actual_max_displacement_mm"]:.1f} mm</p><a href="{plan}">원래 물리시험 결과</a></article>')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>Tomato05 영상·경로 학습 비교</title><style>
body{margin:0;background:#101923;color:#e7edf3;font:16px/1.6 system-ui}main{max-width:1250px;margin:auto;padding:40px}h1{font-size:32px}h2{margin-top:40px}.notice{padding:20px;border-left:4px solid #e7b561;background:#27313b}a{color:#7bdac2}table{border-collapse:collapse;width:100%;background:#18232f}th,td{padding:12px;text-align:left;border-bottom:1px solid #344454}th{color:#92e4c7}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:18px}article{background:#1a2734;padding:16px;border-radius:12px}img{width:100%;border-radius:7px}small{color:#aebbc8}</style><main>'''
    page+=f'<h1>Tomato_05 · 카메라 기준 후보 경로 학습</h1><p>3가지 모델 × {len(manifest["seeds"].split(","))}개 seed · GPU 학습 · 총 {manifest["wall_s"]:.1f}초</p>'
    page+='<div class="notice"><b>한 물리 장면의 예비 실험입니다.</b> 71장 × 1,000개 조합은 71,000개의 독립 수확 실험이 아닙니다. 학습하지 않은 시점과 후보를 동시에 평가했습니다. 표의 진입은 고리 중심 부분 진입이며 꼭지 걸림 성공이 아닙니다. 수치는 저장된 물리 결과와 비교한 것으로 새 시뮬레이션을 실행하지 않았습니다.</div>'
    page+='<h2>새 시점 + 새 후보 평가</h2><p>테스트 7개 시점. 평균 ± 표준편차는 학습 seed 간 차이이며 독립 식물에 대한 신뢰구간이 아닙니다. 추천은 진입확률 × exp(−예상 이동량/20mm)로 정렬합니다. 저변위 진입은 중심 진입과 최대 이동 ≤20mm를 모두 만족해야 합니다.</p><table><thead><tr><th>모델</th><th>영상 조건</th><th>진입 AP ↑</th><th>이동 MAE mm ↓</th><th>1순위 진입</th><th>1순위 저변위 진입</th></tr></thead><tbody>'+''.join(table)+'</tbody></table>'
    page+=f'<p>검증 손실 평균으로 선택한 모델: <b>{selected}</b>. RGB·Depth·크롭 K를 함께 바꾸는 검사입니다. 성능 저하는 관측 정합성에 대한 민감도를 보이지만 RGB 단독 기여를 증명하지 않습니다. 영상 없는 모델보다 낫더라도 새 토마토·실물 일반화를 증명하지 않습니다.</p>'
    base=manifest['known_scene_lookup_baseline'];page+=f'<p>참고: 같은 장면에서는 기존에 측정한 <b>{base["candidate_id"]}</b>를 항상 사용해도 진입하며 이동량은 <b>{base["max_displacement_mm"]:.1f}mm</b>입니다. 이는 카메라를 이해한 것이 아니며 새 후보 평가와 직접 비교할 수 없습니다.</p>'
    page+='<h2>대표 seed의 추천 결과</h2><p>검증으로 선택한 모델의 첫 seed를 표시합니다. 좋은 테스트 결과를 골라 표시하지 않습니다.</p><div class="grid">'+''.join(cards)+'</div>'
    page+='<h2>파일</h2><p><a href="manifest.json">분할·설정·한계</a> · <a href="results.json">전체 모델 평가</a> · <a href="feature_manifest.json">이미지·가중치 해시</a></p><ul>'
    for r in runs:
        path=f'{r["model"]}_seed{r["seed"]}';page+=f'<li>{path} · <a href="{path}/history.json">학습 곡선 수치</a> · <a href="{path}/metrics.json">시점/후보별 평가</a> · <a href="{path}/predictions.csv">모든 예측 CSV</a> · <a href="{path}/model.pt">모델</a></li>'
    page+='</ul></main></html>';(out/'index.html').write_text(page)
