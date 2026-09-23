'use strict';
const DATA=JSON.parse(document.getElementById('reportData').textContent), M=DATA.manifest, all=DATA.cases;
const $=id=>document.getElementById(id), esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const entryOnly=M.classification_rule==='center_entry_only_v2';
const labels={partial_center_entry:['부분 진입','#268167','#e9f3ec'],excessive_displacement:['과도 변위','#c16c51','#faf0ea'],miss:['미진입','#809396','#edf1f1'],ik_or_planning_failure:['계획 불가','#a88344','#f8f2e7'],invalid_physics:['물리 이상','#9d5976','#f6eaf0'],execution_error:['실행 오류','#97534c','#faeeeb']};
const paramDefs=[['approach_azimuth_deg','월드 기준 방위각','°',1],['entry_clearance_m','하부 여유','mm',1000],['lateral_offset_m','좌우 오프셋','mm',1000],['insertion_distance_m','삽입 거리','mm',1000],['lift_forward_angle_deg','상승 기울기','°',1],['lift_distance_m','상승 거리','mm',1000]];
const f=(v,n=1)=>Number.isFinite(v)?v.toLocaleString('ko-KR',{minimumFractionDigits:n,maximumFractionDigits:n}):'—';
const mm=r=>typeof r.target_center_max_displacement_m==='number'?r.target_center_max_displacement_m*1000:null;
const label=k=>labels[k]||[k,'#888','#eee'];
const badge=k=>{let l=label(k);return `<span class="badge" style="color:${l[1]};background:${l[2]}"><span class="dot" style="background:${l[1]};width:5px;height:5px;margin:0"></span>${esc(l[0])}</span>`};
const duration=s=>Number.isFinite(s)?`${Math.floor(s/60)}분 ${Math.round(s%60)}초`:'미기록';
const counts=Object.fromEntries(Object.keys(labels).map(k=>[k,all.filter(r=>r.result===k).length]));
const entered=all.filter(r=>r.center_entered===true).length, exceededEntry=all.filter(r=>r.center_entered&&r.result==='excessive_displacement').length;
let filtered=[],page=0,selected=null,returnFocus=null;const pageSize=20;
$('runName').textContent=DATA.run;$('configLine').textContent=`${M.target||'Tomato_05'} · ${M.workers??'—'}개 물리 작업자 · ${M.hz??'—'} Hz · MuJoCo ${M.mujoco||'—'}`;
$('runStatus').textContent=all.length>=(M.count??all.length)?'● 기록 완료':'◌ 수집 중';
const metrics=[['기록된 후보',f(all.length,0),`요청 ${f(M.count,0)}개 · 서로 다른 경로`],[entryOnly?'중심 진입 성공 · 변위 무관':'부분 진입 · 변위 20mm 이내',f(counts.partial_center_entry,0),`전체의 ${f(100*(counts.partial_center_entry||0)/Math.max(all.length,1))}% · 우선 검토 대상`],['과도한 열매 밀림',f(entryOnly?all.filter(r=>mm(r)>20).length:counts.excessive_displacement,0),entryOnly?'최대 열매 변위 20mm 초과 · 성공과 별도 집계':'최대 중심 변위 20mm 초과'],['전체 실험 소요',Number.isFinite(M.total_wall_s)?`${Math.floor(M.total_wall_s/60)}<em>분</em> ${Math.round(M.total_wall_s%60)}<em>초</em>`:'—','계획·실행·결과 저장 포함']];
$('metrics').innerHTML=metrics.map((m,i)=>`<article class="metric ${i===1?'accent':''}"><span class="metric-index">0${i+1}</span><div class="metric-title">${m[0]}</div><strong>${m[1]}</strong><small>${m[2]}</small></article>`).join('');
$('takeaway').textContent=counts.partial_center_entry?`먼저 ${f(counts.partial_center_entry,0)}개의 부분 진입 후보를 확인하세요.`:(entryOnly?'중심 진입 성공으로 분류된 후보가 아직 없습니다.':'허용 변위 내 부분 진입으로 분류된 후보가 아직 없습니다.');
$('explanation').textContent=entryOnly?'정상 물리 실행에서 중심 진입 이력이 있으면 열매 밀림과 관계없이 부분 진입 성공입니다. 최대 변위는 별도 지표입니다.':`중심 진입 이력이 있는 후보는 ${f(entered,0)}개입니다. 그중 ${f(exceededEntry,0)}개는 변위 한도를 초과해 ‘과도 변위’로 분류됐습니다. 진입 여부와 최종 판정을 함께 보세요.`;
$('distribution').innerHTML=Object.entries(labels).map(([k,l])=>`<button class="dist-row" data-result="${k}" aria-label="${l[0]} 필터"><span class="dist-label"><span>${l[0]}</span><b>${f(counts[k],0)}<small>${f(100*counts[k]/Math.max(all.length,1))}%</small></b></span><div class="bar-track"><div class="bar-fill" style="width:${100*counts[k]/Math.max(all.length,1)}%;background:${l[1]}"></div></div></button>`).join('');
$('distributionNote').textContent=entryOnly?'물리 이상을 먼저 제외하고 중심 진입 여부로 성공/미진입을 분류합니다. 꼭지 걸림이나 수확 성공 판정은 아닙니다.':'판정은 후보마다 하나입니다. 물리 이상 → 과도 변위 → 부분 진입 → 미진입 순으로 분류합니다. ‘물리 이상 0’이 수확 성공을 뜻하지는 않습니다.';
$('legend').innerHTML=Object.entries(labels).filter(([k])=>counts[k]).map(([k,l])=>`<span><i class="dot" style="background:${l[1]}"></i>${l[0]}</span>`).join('');
$('axis').innerHTML=paramDefs.map(([k,l,u])=>`<option value="${k}">${l} (${u})</option>`).join('');
$('resultFilter').innerHTML+=Object.entries(labels).map(([k,l])=>`<option value="${k}">${l[0]} (${counts[k]})</option>`).join('');
const lookup=new Map(all.map(r=>[r.candidate_id,r]));
function apply(){
 const term=$('search').value.trim().toLowerCase(),numeric=/^\d+$/.test(term),cat=$('resultFilter').value,entry=$('entryFilter').value;
 filtered=all.filter(r=>(!term||(numeric?Number(r.candidate_id.split('_').pop())===Number(term):r.candidate_id.toLowerCase().includes(term)))&&(cat==='all'||r.result===cat)&&(entry==='all'||(entry==='yes'?r.center_entered===true:entry==='no'?r.center_entered===false:r.center_entered==null)));
 const ord=Object.keys(labels);const compareId=(a,b)=>a.candidate_id.localeCompare(b.candidate_id);const disp=r=>mm(r)??Infinity;
 filtered.sort((a,b)=>{switch($('sort').value){case'id':return compareId(a,b);case'dispAsc':return disp(a)-disp(b)||compareId(a,b);case'dispDesc':return (mm(b)??-Infinity)-(mm(a)??-Infinity)||compareId(a,b);case'time':return (b.metrics?.rollout_wall_s??-Infinity)-(a.metrics?.rollout_wall_s??-Infinity)||compareId(a,b);default:return ord.indexOf(a.result)-ord.indexOf(b.result)||disp(a)-disp(b)||compareId(a,b)}});
 page=0;renderRows();plot();
}
function renderRows(){
 $('filteredCount').textContent=f(filtered.length,0);let rows=filtered.slice(page*pageSize,(page+1)*pageSize);
 $('rows').innerHTML=rows.map(r=>{let p=r.parameters||{},v=mm(r),i=Number(r.candidate_id.split('_').pop());return `<tr><td><span class="candidate-id">#${String(i).padStart(5,'0')}</span></td><td>${badge(r.result)}</td><td><span class="${r.center_entered?'entry-yes':'entry-no'}">${r.center_entered===true?'✓ 진입':r.center_entered===false?'— 없음':'미기록'}</span></td><td class="${v>20?'danger-value':''}">${f(v)} <span class="tiny">mm</span></td><td>${f(p.approach_azimuth_deg)}°</td><td>${f(p.insertion_distance_m*1000)} / ${f(p.lift_distance_m*1000)} <span class="tiny">mm</span></td><td>${f(r.metrics?.simulated_s??r.seconds)} <span class="tiny">s</span></td><td><button class="detail-button" data-case="${esc(r.candidate_id)}">상세 · 재연 ↗</button></td></tr>`}).join('')||'<tr><td colspan="8" class="empty">조건에 맞는 후보가 없습니다. 필터를 초기화하거나 다른 번호를 검색하세요.</td></tr>';
 $('pageInfo').textContent=filtered.length?`${f(page*pageSize+1,0)}–${f(Math.min((page+1)*pageSize,filtered.length),0)} / ${f(filtered.length,0)}개`:'0개';
 $('pageNumber').textContent=`${page+1} / ${Math.max(1,Math.ceil(filtered.length/pageSize))}`;$('previous').disabled=page===0;$('next').disabled=(page+1)*pageSize>=filtered.length;
}
function plot(){
 const [key,title,unit,scale]=paramDefs.find(p=>p[0]===$('axis').value),points=filtered.filter(r=>Number.isFinite(mm(r))&&Number.isFinite(r.parameters?.[key]));
 const W=680,H=295,L=55,T=22,R=18,B=42,xs=points.map(r=>r.parameters[key]*scale),ys=points.map(mm),xmin=xs.length?Math.min(...xs):0,xmax=xs.length?Math.max(...xs):1,ymax=Math.max(25,...ys)*1.07;
 const lo=xmin===xmax?xmin-1:xmin,hi=xmin===xmax?xmax+1:xmax;const X=x=>L+(x-lo)/(hi-lo)*(W-L-R),Y=y=>H-B-y/ymax*(H-T-B);
 let svg=`<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="${title}와 최대 열매 변위 산점도"><rect x="${L}" y="${Y(20)}" width="${W-L-R}" height="${Y(0)-Y(20)}" fill="#f0f6ed"/>`;
 for(let i=0;i<=4;i++){let y=ymax*i/4;svg+=`<line x1="${L}" x2="${W-R}" y1="${Y(y)}" y2="${Y(y)}" stroke="#e9eee8"/><text x="${L-10}" y="${Y(y)+4}" text-anchor="end" fill="#89968c" font-size="10">${f(y,0)}</text>`;let x=lo+(hi-lo)*i/4;svg+=`<text x="${X(x)}" y="${H-B+21}" text-anchor="middle" fill="#89968c" font-size="10">${f(x,1)}</text>`}
 svg+=`<line x1="${L}" x2="${W-R}" y1="${Y(20)}" y2="${Y(20)}" stroke="#8aaa77" stroke-dasharray="5 4"/><text x="${W-R-2}" y="${Y(20)-6}" text-anchor="end" fill="#6c8e57" font-size="10">허용 변위 20 mm</text><text x="${L}" y="12" fill="#839184" font-size="10">최대 변위 (mm)</text><text x="${W/2}" y="${H-3}" text-anchor="middle" fill="#839184" font-size="10">${title} (${unit})</text>`;
 svg+=points.map(r=>`<circle cx="${X(r.parameters[key]*scale)}" cy="${Y(mm(r))}" r="3" fill="${label(r.result)[1]}" opacity=".65" data-point="${esc(r.candidate_id)}"><title>${esc(r.candidate_id)} · ${label(r.result)[0]} · ${f(mm(r))} mm</title></circle>`).join('');
 if(!points.length)svg+='<text x="340" y="130" text-anchor="middle" fill="#88988c" font-size="12">표시할 변위 데이터가 없습니다</text>';
 $('scatter').innerHTML=svg+'</svg>';
}
function pathView(r){
 let w=(r.waypoints||[]).filter(p=>Array.isArray(p.position_xyz));if(!w.length)return '<p class="detail-note">실행 가능한 경유점이 없습니다.</p>';
 let xs=w.map(p=>p.position_xyz[0]),zs=w.map(p=>p.position_xyz[2]),xl=Math.min(...xs),xh=Math.max(...xs),zl=Math.min(...zs),zh=Math.max(...zs);const X=x=>50+(x-xl)/Math.max(xh-xl,.001)*380,Z=z=>148-(z-zl)/Math.max(zh-zl,.001)*112;const names={preapproach:'접근 준비',entry:'진입',insert:'삽입',rise:'상승'};
 let s='<svg viewBox="0 0 485 187" aria-label="계획한 고리 중심 경유점 X Z 투영"><rect x="0" y="0" width="485" height="187" rx="8" fill="#f5f8f3"/>';
 s+=`<polyline points="${w.map(p=>`${X(p.position_xyz[0])},${Z(p.position_xyz[2])}`).join(' ')}" fill="none" stroke="#7eac91" stroke-width="2" stroke-dasharray="5 4"/>`;
 w.forEach((p,i)=>{let x=X(p.position_xyz[0]),z=Z(p.position_xyz[2]);s+=`<circle cx="${x}" cy="${z}" r="5" fill="#267b5a"/><text x="${x}" y="${z+(i%2?23:-13)}" text-anchor="middle" fill="#57775e" font-size="10">${i+1}. ${names[p.phase]||esc(p.phase)}</text>`});
 return s+'<text x="15" y="176" fill="#849580" font-size="9">X–Z 투영 · 축별 자동 스케일 · 계획 경유점이며 실제 궤적 아님</text></svg>';
}
function openDetail(id){
 const r=lookup.get(id);if(!r)return;selected=r;returnFocus=document.activeElement;const p=r.parameters||{},m=r.metrics||{},v=mm(r);$('detailTitle').textContent=r.candidate_id;
 const paths=r.first_contact_candidate?.prim_paths||r.first_contact_candidate?.objects||[];
 const timing=[['경로 계획',r.planning_wall_s],['물리 계산',m.physics_s],['진단',m.evaluation_s],['상태 기록',m.recording_s]].filter(([,v])=>Number.isFinite(v));let max=Math.max(1,...timing.map(t=>t[1]));
 const linkbase=`candidates/${encodeURIComponent(r.candidate_id)}/`;
 $('detailBody').innerHTML=`${badge(r.result)}<p class="detail-note">${r.result==='partial_center_entry'?(entryOnly?'중심 진입 이력이 있어 성공으로 분류했습니다. 열매 최대 변위는 별도로 확인하세요.':'변위 기준 이내에서 중심 진입 이력이 있습니다. 재연으로 실제 걸림 가능성을 확인하세요.'):r.result==='excessive_displacement'?'목표 열매가 변위 기준 20mm보다 많이 움직였습니다. 진입 이력이 있어도 이 판정이 우선합니다.':r.result==='miss'?(entryOnly?'열매 중심 진입이 기록되지 않았습니다.':'변위 한도 내에서 열매 중심 진입이 기록되지 않았습니다.'):esc(r.error||r.preflight?.reason||'상세 진단을 확인하세요.')}</p>
 <div class="detail-metrics"><div><span>최대 중심 변위</span><b>${f(v)} <small>mm</small></b></div><div><span>중심 진입 이력</span><b>${r.center_entered===true?'있음':r.center_entered===false?'없음':'미기록'}</b></div><div><span>시뮬레이션 길이</span><b>${f(m.simulated_s??r.seconds)} <small>s</small></b></div></div>
 <div class="detail-section"><h3>원본 동작 재연</h3>${r.replay_available?`<p class="detail-note">저장된 원본 상태를 표시합니다. 새 실험은 30Hz 재연 데이터를 사용합니다. 아래 명령을 복사해 터미널에서 실행하세요. 화면은 DISPLAY=:0에서 열립니다.</p><pre id="replayCommand">${esc(r.command)}</pre><div class="command-actions"><button class="button primary" id="copyCommand">명령 복사</button><button class="button ghost" id="downloadScript">실행 스크립트 저장</button><button class="button primary" id="runCommand" style="background:#1b5e20;border-color:#1b5e20;color:#fff;">🚀 바로 실행 (도커)</button></div>`:'<p class="detail-note">저장된 상태 파일이 없어 재연할 수 없습니다.</p>'}</div>
 <div class="detail-section"><h3>재현용 경로 파라미터</h3><div class="parameter-list">${paramDefs.map(([k,l,u,s])=>`<div class="param"><span>${l}</span><b>${f(p[k]*s)} ${u}</b></div>`).join('')}</div></div>
 <div class="detail-section"><h3>고리 중심의 계획 경유점</h3><div class="detail-path">${pathView(r)}</div></div>
 <div class="detail-section"><h3>처음 기록된 접촉 후보</h3><p class="detail-note">${r.first_contact_candidate?`스텝 ${r.first_contact_candidate.step} · ${f(r.first_contact_candidate.step/(M.hz||120),2)}초`:'기록 없음'} · 비활성 후보도 포함하므로 실제 접촉력 발생을 의미하지 않습니다.</p>${paths.length?`<pre>${paths.map(esc).join('\n↕\n')}</pre>`:''}</div>
 <div class="detail-section"><h3>후보별 실행 시간</h3><div>${timing.map(([l,v])=>`<div class="timeline-row"><span>${l}</span><div class="bar-track"><div class="bar-fill" style="width:${v/max*100}%;background:#8bad99"></div></div><span>${f(v,2)}s</span></div>`).join('')}</div><p class="detail-note">물리 실행 벽시계 ${f(m.rollout_wall_s,2)}초 · 실행·저장 ${f(r.execution_and_save_wall_s,2)}초. 병렬 후보별 시간 합계는 실험 전체 소요 시간이 아닙니다.</p></div>
 <div class="detail-section"><h3>원본 데이터</h3><div class="links"><a href="${linkbase}plan.json">계획 JSON ↗</a><a href="${linkbase}result.json">결과 JSON ↗</a>${r.replay_available?`<a href="${linkbase}trace.json">관절 명령 ↗</a><a href="${linkbase}states.npz" download>상태 NPZ ↓</a>`:''}</div><details style="margin-top:18px"><summary>전체 진단과 해시 보기</summary><pre>${esc(JSON.stringify({action_camera:r.action_camera,preflight:r.preflight,metrics:r.metrics,hook_success:r.hook_success,trace_sha256:r.trace_sha256,states_sha256:r.states_sha256},null,2))}</pre></details></div>`;
 $('detail').hidden=false;$('backdrop').hidden=false;document.body.style.overflow='hidden';$('detail').scrollTop=0;$('closeDetail').focus();
 if(r.replay_available){
  $('copyCommand').onclick=()=>copy(r.command);
  $('downloadScript').onclick=()=>download(`#!/usr/bin/env bash\nset -e\n${r.command}\n`,`${r.candidate_id}_replay.sh`,'text/x-shellscript');
  $('runCommand').onclick=()=>executeReplay(r.command);
 }
 history.replaceState(null,'',`#candidate=${encodeURIComponent(id)}`);
}
function closeDetail(){ $('detail').hidden=true;$('backdrop').hidden=true;document.body.style.overflow='';history.replaceState(null,'','#candidates');returnFocus?.focus(); }
function notify(s){$('toast').textContent=s;$('toast').hidden=false;setTimeout(()=>$('toast').hidden=true,3200)}
async function executeReplay(cmd){
 notify('도커(humble_x64_env)에서 3D 리플레이 실행 요청 중...');
 try{
  const resp=await fetch('http://localhost:8766/replay',{
   method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({command:cmd})
  });
  const res=await resp.json();
  if(res.status==='ok'){
   notify('🎉 도커 내부 3D 리플레이 창이 열렸습니다! (DISPLAY=:0)');
  }else{
   notify('❌ 실행 실패: '+(res.error||'알 수 없는 오류'));
  }
 }catch(e){
  notify('⚠️ 리플레이 서버(8766) 미실행. 컨테이너에서 python replay_server.py 실행 필요');
 }
}
async function copy(s){try{if(navigator.clipboard&&window.isSecureContext)await navigator.clipboard.writeText(s);else{const area=document.createElement('textarea');area.value=s;document.body.append(area);area.select();const ok=document.execCommand('copy');area.remove();if(!ok)throw Error('copy')}notify('재연 명령을 복사했습니다.')}catch(e){notify('복사가 제한되어 있습니다. 명령을 직접 선택해 복사하세요.')}}
function download(text,name,type){const url=URL.createObjectURL(new Blob([text],{type})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)}
$('exportFiltered').onclick=()=>{let keys=['candidate_id','result','center_entered','target_max_displacement_mm',...paramDefs.map(p=>p[0])];const cell=v=>'"'+String(v??'').replace(/"/g,'""')+'"';let lines=[keys.map(cell).join(',')];filtered.forEach(r=>lines.push([r.candidate_id,r.result,r.center_entered,mm(r),...paramDefs.map(p=>r.parameters?.[p[0]])].map(cell).join(',')));download('\ufeff'+lines.join('\r\n'),'filtered_candidates.csv','text/csv;charset=utf-8');notify(`${filtered.length}개 후보를 CSV로 저장합니다.`)};
['search','resultFilter','entryFilter','sort'].forEach(id=>$(id).addEventListener(id==='search'?'input':'change',apply));$('axis').onchange=plot;
$('clearFilters').onclick=()=>{$('search').value='';$('resultFilter').value='all';$('entryFilter').value='all';$('sort').value='review';apply()};
$('previous').onclick=()=>{page--;renderRows()};$('next').onclick=()=>{page++;renderRows()};
$('rows').onclick=e=>{const b=e.target.closest('[data-case]');if(b)openDetail(b.dataset.case)};
$('scatter').onclick=e=>{const dot=e.target.closest('[data-point]');if(dot)openDetail(dot.dataset.point)};
$('scatter').onpointerover=e=>{const dot=e.target.closest('[data-point]');if(!dot)return;const r=lookup.get(dot.dataset.point);$('plotTooltip').textContent=`${r.candidate_id} · ${label(r.result)[0]} · 최대 변위 ${f(mm(r))} mm`;$('plotTooltip').hidden=false};$('scatter').onpointerleave=()=>$('plotTooltip').hidden=true;
$('distribution').onclick=e=>{const b=e.target.closest('[data-result]');if(!b)return;$('resultFilter').value=b.dataset.result;$('entryFilter').value='all';$('search').value='';apply();$('candidates').scrollIntoView({behavior:'smooth'})};
$('closeDetail').onclick=closeDetail;$('backdrop').onclick=closeDetail;
document.addEventListener('keydown',e=>{if($('detail').hidden)return;if(e.key==='Escape')closeDetail();if(e.key==='Tab'){const nodes=[...$('detail').querySelectorAll('button,a,summary')].filter(x=>!x.disabled);if(e.shiftKey&&document.activeElement===nodes[0]){e.preventDefault();nodes.at(-1).focus()}else if(!e.shiftKey&&document.activeElement===nodes.at(-1)){e.preventDefault();nodes[0].focus()}}});
const settings=[['대상',M.target||'Tomato_05'],['물리 엔진',`MuJoCo ${M.mujoco||'—'}`],['물리 / 계획 작업자',`${M.workers??'—'} / ${M.planning_workers??'—'}`],['물리 주파수',`${M.hz??'—'} Hz`],['후보 생성','Sobol · 6개 변수'],['Seed',M.seed??'—'],['식물 / 파단','최적화 모델 / 파단 비활성'],['꼭지 걸림 성공 판정','미구현 · 성공률 산출 안 함']];
$('settingsGrid').innerHTML=settings.map(([k,v])=>`<div class="setting"><span>${k}</span><b>${esc(v)}</b></div>`).join('');
$('provenance').innerHTML=`<p>모델 SHA-256</p><code>${esc(M.model_sha256||'미기록')}</code><p style="margin-top:12px">전체 시간 ${duration(M.total_wall_s)}는 계획·물리 실행·저장을 포함합니다. 후보별 물리 시간은 각 작업자에서 측정한 벽시계입니다. 로딩 및 계획을 제외한 전체 rollout 구간은 이 기록에서 별도로 집계되지 않았습니다.</p><p style="margin-top:8px">접촉 기록에는 비활성 접촉 후보가 포함됩니다. 고리의 외형은 원본 STL이며 물리는 분할 충돌체를 사용합니다. 부분 진입 판정만으로 안전한 수확 성공을 보장하지 않습니다.</p><p style="margin-top:8px"><a class="text-link" href="results.json">전체 결과 JSON ↗</a></p>`;
apply();const initial=location.hash.match(/^#candidate=(.+)$/);if(initial)openDetail(decodeURIComponent(initial[1]));
window.addEventListener('hashchange',()=>{const match=location.hash.match(/^#candidate=(.+)$/);if(match)openDetail(decodeURIComponent(match[1]));});
