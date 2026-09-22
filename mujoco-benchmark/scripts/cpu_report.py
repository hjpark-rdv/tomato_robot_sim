"""CPU 벤치마크 결과를 외부 의존성 없는 HTML로 표시한다."""
import argparse
import html
import json
import statistics
from pathlib import Path


def generate(root):
 root=Path(root)
 reports=json.loads((root/'summary.json').read_text())
 manifest=json.loads((root/'manifest.json').read_text())
 sections=[]
 for report in reports:
  n=report['workers']; rounds=report.get('rounds',[])
  if not rounds:continue
  walls=[r['wall_s'] for r in rounds]
  results=report.get('results',[])
  resource=root/f'cpu_{n:03d}'/'resources.jsonl'
  samples=[json.loads(line) for line in resource.read_text().splitlines() if line.strip()] if resource.exists() else []
  peak=max((sum(w['rss_mib'] for w in s.get('workers',[]))/1024 for s in samples),default=0)
  rows=''.join(f"<tr><td>{r['repeat']+1}</td><td>{r['wall_s']:.3f}</td><td>{r['aggregate_rtf']:.2f}</td><td>{r['episodes_per_s']:.2f}</td><td>{r['unstable_count']}</td></tr>" for r in rounds)
  bars=''.join(f"<div class='barrow'><span>{i+1}회</span><div class='bar' style='width:{v/max(walls)*80:.1f}%'>{v:.3f}초</div></div>" for i,v in enumerate(walls))
  timings={key:statistics.mean([r[key] for r in results if isinstance(r.get(key),(float,int))]) for key in ('control_s','physics_s','evaluation_s','wall_s') if any(isinstance(r.get(key),(float,int)) for r in results)}
  detail=html.escape(json.dumps(timings,ensure_ascii=False,indent=2))
  sections.append(f'''<section><h2>CPU {n}프로세스 · {n}환경</h2>
<div class="cards"><article>평균 배치 시간<strong>{statistics.mean(walls):.3f}초</strong>표준편차 {statistics.stdev(walls) if len(walls)>1 else 0:.3f}초</article>
<article>평균 합산 처리량<strong>{statistics.mean(r['aggregate_rtf'] for r in rounds):.2f}</strong>sim-s / wall-s</article>
<article>기록된 실행 수<strong>{len(results)}회</strong>동일 경로 반복</article>
<article>비정상 실행<strong>{sum(r['unstable_count'] for r in rounds)}회</strong>고리걸기 성공률과는 다름</article></div>
<p>준비·워밍업 {report['startup_and_warmup_wall_s']:.2f}초 · 프로세스 시작부터 종료까지 {report['process_total_wall_s']:.2f}초 · 작업자 RSS 합계 최대 {peak:.2f} GiB (샘플 기준)</p>
<h3>반복별 배치 실행 시간</h3>{bars}
<table><thead><tr><th>반복</th><th>배치 시간(초)</th><th>합산 처리량(sim-s/s)</th><th>실행 수/초</th><th>비정상</th></tr></thead><tbody>{rows}</tbody></table>
<details><summary>작업자별 평균 타이밍 (초)</summary><pre>{detail}</pre></details>
<p><a href="cpu_{n:03d}/timings.json">상세 타이밍 JSON</a> · <a href="cpu_{n:03d}/resources.jsonl">자원 시계열</a></p></section>''')
 args=html.escape(json.dumps(manifest.get('arguments',{}),ensure_ascii=False,indent=2))
 page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>MuJoCo CPU 성능 보고서</title>
<style>body{font:16px/1.6 system-ui;background:#101827;color:#e5edf6;max-width:1100px;margin:40px auto;padding:0 24px}h1,h2{color:#fff}section{background:#1a2638;padding:24px;border-radius:16px;margin:24px 0}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}article{background:#25364c;padding:16px;border-radius:10px}strong{display:block;font-size:29px;color:#67dfce}a{color:#83c4ff}table{width:100%;border-collapse:collapse;margin:24px 0}td,th{text-align:left;padding:10px;border-bottom:1px solid #415069}.barrow{display:flex;align-items:center;gap:12px;margin:12px 0}.bar{background:#246a92;padding:6px 12px;box-sizing:border-box;border-radius:6px}pre{white-space:pre-wrap;overflow-wrap:anywhere}.note{border-left:4px solid #f4c36b;padding:12px 18px;background:#273044}</style>
<h1>MuJoCo CPU 성능 보고서</h1>'''
 page+=f'<p>{html.escape(root.name)}</p><p class="note">동일한 기록 경로를 각 환경에서 반복한 성능 시험입니다. 서로 다른 경로 탐색이나 수확 성공 판정이 아닙니다. 배치 시간은 로딩·워밍업을 제외하며, 합산 처리량은 모든 환경의 시뮬레이션 시간을 합친 값입니다. 화면 렌더링은 꺼져 있습니다.</p>'
 cases=[{'workers':r['workers'],'repeat':v['repeat']+1,'env':v['worker'],'result':v} for r in reports for v in r.get('results',[])]
 import shlex
 command='DISPLAY=:0 '+shlex.quote(str(Path(__file__).resolve().parents[1]/'.venv/bin/python'))+' '+shlex.quote(str(Path(__file__).resolve().with_name('replay_cpu_case.py')))+' '+shlex.quote(str(root.resolve()))
 payload=json.dumps(cases,ensure_ascii=False).replace('<','\\u003c')
 page+='<section><h2>회차·환경 선택 및 재연</h2><p>반복은 1부터, 환경 번호는 0부터입니다. 선택한 실행의 원본 결과와 재연 명령을 표시합니다. 명령을 터미널에서 실행하면 모델·경로 해시와 버전을 확인하고 결과 비교 후 화면을 엽니다.</p><select id="case"></select><pre id="command"></pre><button id="copy">명령 복사</button><details><summary>선택한 실행의 전체 결과</summary><pre id="result"></pre></details><p>기존 결과에는 매 스텝 상태가 없으므로 물리를 다시 계산합니다. 원본 요약 지표와 다르면 재연을 중단합니다. 동일 경로를 반복한 현재 시험에서는 회차별 동작이 같을 수 있습니다.</p></section>'
 page+='<script>const cases='+payload+';const base='+json.dumps(command).replace('<','\\u003c')+';const sel=document.getElementById("case");cases.forEach((c,i)=>{const o=document.createElement("option");o.value=i;o.textContent=`${c.workers}환경 / ${c.repeat}회차 / 환경 ${c.env}`;sel.appendChild(o)});function show(){const c=cases[sel.value];if(!c)return;document.getElementById("command").textContent=base+` --workers ${c.workers} --repeat ${c.repeat} --env ${c.env}`;document.getElementById("result").textContent=JSON.stringify(c.result,null,2)}sel.onchange=show;show();document.getElementById("copy").onclick=()=>navigator.clipboard.writeText(document.getElementById("command").textContent).catch(()=>alert("명령을 선택해 복사해주세요."));</script>'
 page+=''.join(sections)+f'<details><summary>실행 설정</summary><pre>{args}</pre></details><p><a href="rounds.csv">반복별 CSV</a> · <a href="summary.json">전체 결과 JSON</a> · <a href="manifest.json">실행 환경·모델 해시</a></p></html>'
 target=root/'index.html';target.write_text(page,encoding='utf-8');return target

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);a=p.parse_args();print(generate(a.run_dir))
