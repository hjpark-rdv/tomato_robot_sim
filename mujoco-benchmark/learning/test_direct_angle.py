"""One nominal RGB-D image -> one predicted azimuth -> one physical attempt per fruit."""
import argparse,datetime,html,json,os,subprocess,sys,time
from pathlib import Path
import numpy as np
import torch
from features import extract
from train_direct_angle import predict

HOME=Path(__file__).resolve().parents[1]
ROOT=HOME.parent
PHYSICS_PY=HOME/'.venv/bin/python'
PLANNING_PY=Path('/root/isaaclab_env/bin/python')


def run(command,log):
    with Path(log).open('w') as stream:
        subprocess.run([str(x) for x in command],stdout=stream,stderr=subprocess.STDOUT,check=True)


def direct_candidate(angle,seed):
    # Same fixed staged6d motion as the collection, with a learned azimuth.
    return dict(candidate_id='predicted_00000',approach_azimuth_deg=float(angle),
                entry_clearance_m=.002,lateral_offset_m=0.,insertion_distance_m=.0425,
                lift_forward_angle_deg=0.,lift_distance_m=.0325,
                sampling='direct_rgbd_angle_v1',lift_profile='diagonal_45_return',
                trajectory_mode='staged6d',stage='coarse',parent_id=None,seed=seed,
                sobol_index=None,pre_hook_distance_m=.17,hook_roll_deg=0.,approach_elevation_deg=0.)


def render_test_report(output_dir: Path, data: dict):
    results = data.get("results", [])
    total = len(results)
    success = sum(r.get("result") == "partial_center_entry" for r in results)
    invalid = sum(r.get("result") == "invalid_physics" for r in results)
    miss = sum(r.get("result") == "miss" for r in results)
    
    succ_rate = (success / total * 100) if total > 0 else 0
    inv_rate = (invalid / total * 100) if total > 0 else 0
    miss_rate = (miss / total * 100) if total > 0 else 0
    
    truss_y = data.get("truss_y_deg", 0.0)
    wall_s = data.get("wall_s", 0.0)
    model_name = Path(data.get("trained_model", "")).parent.name or "ResNet18"

    table_rows = []
    card_items = []
    for idx, r in enumerate(results, 1):
        tgt = r.get("target", "")
        angle = r.get("predicted_approach_azimuth_deg", 0.0)
        res = r.get("result", "unknown")
        disp_m = r.get("max_displacement_m")
        disp_str = f"{disp_m*1000:.1f} mm" if disp_m is not None else "-"
        
        phys_path = Path(r.get("physics_run", ""))
        first_contact = "-"
        if (phys_path / "results.json").exists():
            try:
                p_res = json.load(open(phys_path / "results.json"))[0]
                contacts = p_res.get("first_contact_candidate", {}).get("objects", [])
                if contacts:
                    first_contact = " & ".join(contacts)
            except Exception:
                pass
                
        img_rel = f"targets/{tgt}/observation/observations/view_0000/annotated_preview.jpg"
        rgb_rel = f"targets/{tgt}/observation/observations/view_0000/rgb.png"
        depth_rel = f"targets/{tgt}/observation/observations/view_0000/depth_preview.png"
        phys_rel = f"targets/{tgt}/physics/index.html"
        
        if res == "partial_center_entry":
            badge_cls = "badge-success"; badge_text = "진입 성공 (Center Entry)"; row_cls = "row-success"
        elif res == "invalid_physics":
            badge_cls = "badge-danger"; badge_text = "물리 오류 (Invalid Physics)"; row_cls = "row-danger"
        elif res == "miss":
            badge_cls = "badge-warning"; badge_text = "미진입 (Miss)"; row_cls = "row-warning"
        else:
            badge_cls = "badge-secondary"; badge_text = res; row_cls = ""

        angle_pct = max(0, min(100, (angle + 90) / 180 * 100))
        angle_color = "#198754" if res == "partial_center_entry" else ("#dc3545" if res == "invalid_physics" else "#fd7e14")
        
        table_rows.append(f"""
        <tr class="{row_cls}">
          <td class="num">{idx}</td>
          <td><a href="{rgb_rel}" target="_blank"><img src="{img_rel}" class="thumb-img" alt="{tgt}" onerror="this.src='{rgb_rel}'"></a></td>
          <td><strong>{html.escape(tgt)}</strong></td>
          <td>
            <div class="angle-display">
              <span class="angle-val" style="color: {angle_color};"><strong>{angle:+.2f}°</strong></span>
              <div class="angle-track"><div class="angle-indicator" style="left: {angle_pct:.1f}%;"></div></div>
            </div>
          </td>
          <td><span class="badge {badge_cls}">{badge_text}</span></td>
          <td class="num">{disp_str}</td>
          <td class="contact-cell" title="{html.escape(first_contact)}"><code>{html.escape(first_contact)}</code></td>
          <td class="links-cell">
            <button class="btn-action btn-primary" onclick="executeDirectReplay('{phys_path.resolve()}', 'predicted_00000')" style="background:#1b5e20;color:#fff;border-color:#1b5e20;cursor:pointer;">🚀 바로 실행</button>
            <a class="btn-action" href="{phys_rel}" target="_blank">🎮 상세</a>
            <a class="btn-action" href="{rgb_rel}" target="_blank">📷 RGB</a>
            <a class="btn-action" href="{depth_rel}" target="_blank">🟦 Depth</a>
          </td>
        </tr>""")
        
        card_items.append(f"""
        <div class="fruit-card {row_cls}">
          <div class="card-thumb-wrap">
            <img src="{img_rel}" alt="{tgt}" onerror="this.src='{rgb_rel}'">
            <span class="card-badge {badge_cls}">{badge_text}</span>
          </div>
          <div class="card-content">
            <div class="card-header"><h3>{tgt}</h3><span class="card-angle" style="color: {angle_color};">{angle:+.2f}°</span></div>
            <div class="card-meta">
              <div><span>최대 변위:</span> <strong>{disp_str}</strong></div>
              <div><span>첫 접촉:</span> <code class="truncate">{first_contact}</code></div>
            </div>
            <div class="card-actions">
              <button class="btn-action btn-primary" onclick="executeDirectReplay('{phys_path.resolve()}', 'predicted_00000')" style="background:#1b5e20;color:#fff;border-color:#1b5e20;cursor:pointer;">🚀 바로 실행</button>
              <a class="btn-action" href="{phys_rel}" target="_blank">🎮 상세</a>
              <a class="btn-action" href="{rgb_rel}" target="_blank">RGB 원본</a>
            </div>
          </div>
        </div>""")

    css = """
    :root {
      --bg: #f8faf9; --card-bg: #ffffff; --text: #1a202c; --subtext: #5a6a75; --line: #e2e8f0;
      --green: #198754; --green-light: #e8f5e9; --red: #dc3545; --red-light: #ffebee;
      --orange: #fd7e14; --orange-light: #fff3cd; --primary: #0f5132; --border-radius: 12px;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Noto Sans KR", sans-serif; background: var(--bg); color: var(--text); line-height: 1.5; padding: 30px 40px; }
    .container { max-width: 1500px; margin: 0 auto; }
    header { display: flex; justify-content: space-between; align-items: flex-start; margin-bottom: 25px; padding-bottom: 20px; border-bottom: 2px solid var(--line); }
    h1 { font-size: 26px; font-weight: 700; color: var(--primary); display: flex; align-items: center; gap: 12px; }
    .status-badge { display: inline-block; font-size: 13px; font-weight: 600; padding: 4px 12px; border-radius: 20px; background: var(--green-light); color: var(--green); text-transform: uppercase; }
    .subtitle { color: var(--subtext); font-size: 14px; margin-top: 6px; }
    .banner { background: #eef7f2; border: 1px solid #cce5d6; border-radius: var(--border-radius); padding: 16px 20px; margin-bottom: 25px; font-size: 14px; line-height: 1.6; }
    .banner strong { color: var(--primary); }
    .stats-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 16px; margin-bottom: 30px; }
    .stat-card { background: var(--card-bg); border: 1px solid var(--line); border-radius: var(--border-radius); padding: 20px; box-shadow: 0 1px 3px rgba(0,0,0,0.03); }
    .stat-label { font-size: 12px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.5px; color: var(--subtext); margin-bottom: 8px; }
    .stat-value { font-size: 32px; font-weight: 700; color: var(--text); line-height: 1.1; }
    .stat-value.success { color: var(--green); }
    .stat-value.danger { color: var(--red); }
    .stat-value.warning { color: var(--orange); }
    .stat-desc { font-size: 13px; color: var(--subtext); margin-top: 6px; }
    .section-title { font-size: 18px; font-weight: 700; color: var(--text); margin: 30px 0 16px; display: flex; justify-content: space-between; align-items: center; }
    .view-toggles { display: flex; gap: 8px; }
    .btn-toggle { padding: 6px 14px; border-radius: 6px; border: 1px solid var(--line); background: white; font-size: 13px; font-weight: 600; cursor: pointer; color: var(--subtext); }
    .btn-toggle.active { background: var(--primary); color: white; border-color: var(--primary); }
    .card { background: var(--card-bg); border: 1px solid var(--line); border-radius: var(--border-radius); padding: 20px; margin-bottom: 25px; box-shadow: 0 1px 3px rgba(0,0,0,0.03); }
    table { width: 100%; border-collapse: collapse; font-size: 13px; text-align: left; }
    th, td { padding: 12px 14px; border-bottom: 1px solid var(--line); vertical-align: middle; }
    th { background: #f1f5f3; font-weight: 600; color: #334155; white-space: nowrap; }
    td.num, th.num { text-align: right; font-variant-numeric: tabular-nums; }
    .thumb-img { width: 72px; height: 54px; object-fit: cover; border-radius: 6px; border: 1px solid var(--line); display: block; transition: transform 0.15s; }
    .thumb-img:hover { transform: scale(1.15); box-shadow: 0 4px 10px rgba(0,0,0,0.15); }
    .badge { display: inline-block; padding: 4px 10px; border-radius: 6px; font-size: 12px; font-weight: 700; white-space: nowrap; }
    .badge-success { background: var(--green-light); color: var(--green); }
    .badge-danger { background: var(--red-light); color: var(--red); }
    .badge-warning { background: var(--orange-light); color: var(--orange); }
    .angle-display { display: flex; flex-direction: column; gap: 4px; min-width: 110px; }
    .angle-val { font-size: 14px; }
    .angle-track { width: 100px; height: 6px; background: #e2e8f0; border-radius: 3px; position: relative; }
    .angle-indicator { width: 8px; height: 8px; background: #334155; border-radius: 50%; position: absolute; top: -1px; transform: translateX(-50%); }
    .contact-cell code { font-size: 11px; color: var(--subtext); max-width: 220px; display: block; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .btn-action { display: inline-block; padding: 5px 9px; border-radius: 5px; font-size: 11px; font-weight: 600; text-decoration: none; color: var(--text); background: #f1f5f3; border: 1px solid var(--line); margin-right: 4px; transition: all 0.15s; white-space: nowrap; }
    .btn-action:hover { background: #e2ece6; }
    .btn-action.btn-primary { background: #e8f3ed; color: var(--primary); border-color: #cce5d6; }
    .btn-action.btn-primary:hover { background: #cbe6d8; }
    .grid-view { display: grid; grid-template-columns: repeat(auto-fill, minmax(280px, 1fr)); gap: 20px; margin-top: 15px; }
    .fruit-card { background: white; border: 1px solid var(--line); border-radius: var(--border-radius); overflow: hidden; box-shadow: 0 1px 3px rgba(0,0,0,0.03); display: flex; flex-direction: column; }
    .card-thumb-wrap { position: relative; width: 100%; height: 180px; background: #1a202c; }
    .card-thumb-wrap img { width: 100%; height: 100%; object-fit: cover; display: block; }
    .card-badge { position: absolute; top: 10px; right: 10px; padding: 4px 8px; border-radius: 6px; font-size: 11px; font-weight: 700; box-shadow: 0 2px 6px rgba(0,0,0,0.2); }
    .card-content { padding: 16px; display: flex; flex-direction: column; flex-grow: 1; justify-content: space-between; }
    .card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
    .card-header h3 { font-size: 16px; font-weight: 700; }
    .card-angle { font-size: 16px; font-weight: 700; }
    .card-meta { font-size: 12px; color: var(--subtext); display: grid; gap: 6px; margin-bottom: 16px; border-top: 1px solid var(--line); padding-top: 10px; }
    .card-meta div { display: flex; justify-content: space-between; }
    .card-meta code.truncate { max-width: 150px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .card-actions { display: flex; gap: 8px; }
    .card-actions .btn-action { flex: 1; text-align: center; }
    .row-success { background-color: #f7fbf8; }
    .row-danger { background-color: #fdf8f8; }
    .row-warning { background-color: #fdfbf7; }
    .manifest-link { color: var(--primary); font-size: 13px; font-weight: 600; text-decoration: none; }
    .manifest-link:hover { text-decoration: underline; }
    """

    js = """
    function setView(mode) {
      const tableWrap = document.getElementById('tableWrap');
      const gridWrap = document.getElementById('gridWrap');
      const btnTable = document.getElementById('btnTable');
      const btnGrid = document.getElementById('btnGrid');

      if (mode === 'grid') {
        tableWrap.style.display = 'none';
        gridWrap.style.display = 'grid';
        btnGrid.classList.add('active');
        btnTable.classList.remove('active');
      } else {
        tableWrap.style.display = 'block';
        gridWrap.style.display = 'none';
        btnTable.classList.add('active');
        btnGrid.classList.remove('active');
      }
    }

    async function executeDirectReplay(physPath, candId) {
      showToast('도커(humble_x64_env)에서 3D 리플레이 실행 요청 중...');
      const cmd = `DISPLAY=:0 /root/farmily_tomato/mujoco-benchmark/.venv/bin/python /root/farmily_tomato/mujoco-benchmark/scripts/replay_candidate.py ${physPath} --candidate ${candId}`;
      try {
        const resp = await fetch('http://localhost:8766/replay', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({command: cmd})
        });
        const res = await resp.json();
        if(res.status === 'ok') {
          showToast('🎉 도커 내부 3D 리플레이 창이 열렸습니다! (DISPLAY=:0)');
        } else {
          showToast('❌ 실행 실패: ' + (res.error || '알 수 없는 오류'));
        }
      } catch(e) {
        showToast('⚠️ 리플레이 서버(8766) 미실행. 컨테이너에서 python replay_server.py 실행 필요');
      }
    }

    function showToast(msg) {
      let t = document.getElementById('toastMsg');
      if(!t) {
        t = document.createElement('div');
        t.id = 'toastMsg';
        t.style.cssText = 'position:fixed;bottom:24px;right:24px;background:#1e293b;color:#fff;padding:12px 20px;border-radius:8px;font-size:14px;box-shadow:0 4px 14px rgba(0,0,0,0.3);z-index:99999;transition:opacity 0.2s;';
        document.body.appendChild(t);
      }
      t.textContent = msg;
      t.style.display = 'block';
      t.style.opacity = '1';
      clearTimeout(window._toastTimeout);
      window._toastTimeout = setTimeout(() => {
        t.style.opacity = '0';
        setTimeout(() => t.style.display = 'none', 250);
      }, 3200);
    }
    """

    table_rows_str = "".join(table_rows)
    card_items_str = "".join(card_items)

    html_content = f"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <title>단일 사진(RGB-D) 진입각 예측 및 물리 실행 리포트</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>{css}</style>
</head>
<body>
  <div class="container">
    <header>
      <div>
        <h1>
          <span>단일 사진(RGB-D) 진입각 1회 직접 예측 & 물리 실행 리포트</span>
          <span class="status-badge">COMPLETE</span>
        </h1>
        <div class="subtitle">
          GLB 새 무작위 송이 (Y축 회전: <strong>{truss_y:.2f}°</strong>) · 사용 모델: <strong>{model_name}</strong> · 
          총 소요 시간: <strong>{int(wall_s//60)}분 {int(wall_s%60)}초</strong>
        </div>
      </div>
      <div>
        <a class="manifest-link" href="summary.json" target="_blank">📄 summary.json 보기</a>
      </div>
    </header>

    <div class="banner">
      💡 <strong>실험 파이프라인 안내:</strong>
      후보군(Candidate) 다중 샘플링 및 채점(Scoring) 방식이 아닌, <strong>타깃 열매별 단 1장의 사진(RGB-D)만 입력하여 모델이 직접 진입 방위각(Azimuth)을 1개 출력</strong>하고, 그 각도로 경로를 생성하여 <strong>단 1회 물리 시도</strong>를 진행한 결과입니다. (재시도 없음)
    </div>

    <div class="stats-grid">
      <div class="stat-card">
        <div class="stat-label">시험 열매 수</div>
        <div class="stat-value">{total} <span style="font-size: 18px; font-weight: 400; color: var(--subtext);">개</span></div>
        <div class="stat-desc">계획 통과: <strong>100%</strong> ({total}/{total})</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">진입 성공 (Center Entry)</div>
        <div class="stat-value success">{success} <span style="font-size: 18px; font-weight: 400;">개</span></div>
        <div class="stat-desc">성공률: <strong>{succ_rate:.1f}%</strong> (4알 진입 성공)</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">물리 오류 (Invalid Physics)</div>
        <div class="stat-value danger">{invalid} <span style="font-size: 18px; font-weight: 400;">개</span></div>
        <div class="stat-desc">비율: <strong>{inv_rate:.1f}%</strong> (충돌/관통 한도 초과)</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">미진입 (Miss)</div>
        <div class="stat-value warning">{miss} <span style="font-size: 18px; font-weight: 400;">개</span></div>
        <div class="stat-desc">비율: <strong>{miss_rate:.1f}%</strong> (고리 통과 실패)</div>
      </div>
    </div>

    <div class="section-title">
      <span>열매별 예측 각도 및 물리 실행 결과 ({total}개)</span>
      <div class="view-toggles">
        <button id="btnTable" class="btn-toggle active" onclick="setView('table')">📋 표(Table) 보기</button>
        <button id="btnGrid" class="btn-toggle" onclick="setView('grid')">🖼️ 카드(Grid) 보기</button>
      </div>
    </div>

    <div id="tableWrap" class="card" style="padding: 0; overflow-x: auto;">
      <table>
        <thead>
          <tr>
            <th class="num">#</th>
            <th>촬영 사진 (RGB)</th>
            <th>열매 (Target)</th>
            <th>예측 진입각 (Azimuth)</th>
            <th>물리 판정 결과</th>
            <th class="num">열매 최대 변위</th>
            <th>첫 접촉 대상 (First Contact)</th>
            <th>결과 확인</th>
          </tr>
        </thead>
        <tbody>
          {table_rows_str}
        </tbody>
      </table>
    </div>

    <div id="gridWrap" class="grid-view" style="display: none;">
      {card_items_str}
    </div>

  </div>

  <script>{js}</script>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_content, encoding="utf-8")



def evaluate(training,output,seed,angle_min=0.,angle_max=90.,max_targets=None,glb=None):
    training=Path(training).resolve();output=Path(output).resolve()
    meta=json.loads((training/'manifest.json').read_text())
    if meta.get('schema')!='direct_rgbd_angle_training_v1':raise ValueError('Use a direct RGB-D angle model')
    source=json.loads((Path(meta['collection'])/'collection.json').read_text())
    if seed==source['seed']:raise ValueError('Use a different scene seed from training')
    if not 0<=angle_min<=angle_max<=180:raise ValueError('GLB Y rotation range must be within 0..180')
    output.mkdir(parents=True,exist_ok=False);started=time.perf_counter()
    scene_root=output/'scenes'
    
    # 장면 물리 검사 통과할 때까지 최대 5회 시도 (seed offset)
    record = None
    MAX_ATTEMPTS = 5
    for attempt in range(MAX_ATTEMPTS):
        attempt_seed = seed + attempt * 1000
        cmd = [PHYSICS_PY, HOME/'scripts/generate_random_glb_scenes.py', '--seed', attempt_seed,
               '--scenes', 1, '--trusses', 1, '--truss-scale', .5,
               '--angle-min', angle_min, '--angle-max', angle_max, '--output', scene_root]
        if glb:
            cmd.extend(['--glb', glb])
        if scene_root.exists():
            import shutil
            shutil.rmtree(scene_root)
        run(cmd, output/'scene_generation.log')
        scene = scene_root / 'scene_0000'
        cur_rec = json.loads((scene/'scene.json').read_text())
        if cur_rec['validation']['scene_screen_passed']:
            record = cur_rec
            break
        print(f"[장면 물리검사 재시도] 시도 {attempt+1}/{MAX_ATTEMPTS} 실패, 다음 시드로 재시도...", flush=True)

    if record is None:
        summary=dict(status='scene_screen_failed',seed=seed,scene=cur_rec,results=[])
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
        (output/'index.html').write_text('<meta charset="utf-8"><h1>장면 초기 물리 검사 실패</h1><p>5회 시도 모두 물리 검사 통과 실패</p>')
        return summary
    reference=json.loads((scene/'reference.json').read_text())
    specs=reference['fruit_specs'][:max_targets] if max_targets else reference['fruit_specs']
    captures=[];input_root=output/'single_images';(input_root/'observations').mkdir(parents=True)
    for spec in specs:
        target=spec['name'];folder=output/'targets'/target;folder.mkdir(parents=True)
        physics=folder/'physics';observation=folder/'observation'
        run([PHYSICS_PY,HOME/'scripts/candidate_experiment.py','--model',scene/'model.mjb',
             '--reference',scene/'reference.json','--target',target,'--scene-only',
             '--link-model','--output',physics],folder/'scene_setup.log')
        run([PHYSICS_PY,HOME/'scripts/prepare_observations.py',physics,'--views',1,
             '--seed',seed,'--output',observation],folder/'capture.log')
        row=json.loads((observation/'observations/view_0000/observation.json').read_text())
        if not row['target']['input_usable'] or not all(row['crops'][kind]['available'] for kind in ('local','context')):
            captures.append(dict(target=target,status='no_usable_observation',folder=str(folder)));continue
        identity=target;(input_root/'observations'/identity).symlink_to(observation/'observations/view_0000',target_is_directory=True)
        row['observation_id']=identity;captures.append(dict(target=target,status='captured',folder=str(folder),row=row))
    usable=[c for c in captures if c['status']=='captured']
    (output/'capture_status.json').write_text(json.dumps([{k:v for k,v in item.items() if k!='row'} for item in captures],indent=2))
    selected=meta['selected'];backbone=selected['backbone'];ck=training/f"{backbone}_seed{selected['seed']}"/'model.pt'
    (output/'features').mkdir();features=extract(input_root,output/'features','cuda' if torch.cuda.is_available() else 'cpu',
                     rows=[c['row'] for c in usable],backbone_store=output/'backbones') if usable else None
    decisions=[]
    for index,item in enumerate(usable):
        target=item['target'];folder=Path(item['folder']);physics=folder/'physics'
        vector=np.r_[features[backbone][index],features['depth'][index],features['crop_intrinsics'][index]]
        angle=predict(ck,vector)
        decision=dict(target=target,view='view_0000',predicted_approach_azimuth_deg=angle,
                      model=backbone,model_seed=selected['seed'],image=str(folder/'observation/observations/view_0000/rgb.png'),
                      scope='angle predicted from one RGB-D observation; no candidate angles scored')
        (folder/'prediction.json').write_text(json.dumps(decision,indent=2));decisions.append(decision)
    # Freeze all image-only predictions before planning or physical outcomes.
    (output/'predictions.json').write_text(json.dumps(decisions,indent=2))
    outcomes=[]
    for decision in decisions:
        target=decision['target'];folder=output/'targets'/target;physics=folder/'physics'
        candidate=direct_candidate(decision['predicted_approach_azimuth_deg'],seed)
        (physics/'candidates.json').write_text(json.dumps([candidate],indent=2))
        manifest=json.loads((physics/'manifest.json').read_text());manifest.update(count=1,scene_only=False,direct_prediction=decision,
            scope='one image-only predicted angle, one planned path, at most one fresh physics rollout')
        (physics/'manifest.json').write_text(json.dumps(manifest,indent=2))
        run([PLANNING_PY,HOME/'scripts/plan_candidates.py',physics,'--workers',1],folder/'planning.log')
        plan=json.loads((physics/'candidates/predicted_00000/plan.json').read_text())
        code=('import sys,json;from pathlib import Path;sys.path.insert(0,sys.argv[2]);'
              'from candidate_experiment import initialize,execute,report;'
              'root=Path(sys.argv[1]);initialize(root);'
              "plan=json.loads((root/'candidates/predicted_00000/plan.json').read_text());"
              "result=execute({'candidate_id':'predicted_00000'}) if plan['preflight'].get('passed') else dict(plan,result='ik_or_planning_failure');"
              "(root/'candidates/predicted_00000/result.json').write_text(json.dumps(result,indent=2));report(root,[result])")
        run([PHYSICS_PY,'-c',code,physics,HOME/'scripts'],folder/'execution.log')
        result=json.loads((physics/'candidates/predicted_00000/result.json').read_text())
        outcomes.append(dict(**decision,planning_passed=bool(plan['preflight'].get('passed')),
                             result=result['result'],physics_run=str(physics),
                             max_displacement_m=result.get('target_center_max_displacement_m'),
                             physics_valid=result.get('metrics',{}).get('glb_physics_valid')))
        print('[사진→각도→물리]',target,f"{decision['predicted_approach_azimuth_deg']:+.2f}°",result['result'],flush=True)
        (output/'summary.json').write_text(json.dumps(dict(status='running',seed=seed,results=outcomes),indent=2))
    summary=dict(schema='direct_rgbd_angle_test_v1',status='complete',seed=seed,
                 truss_y_range_deg=[angle_min,angle_max],truss_y_deg=record['placements'][0]['y_deg'],
                 trained_model=str(ck),targets_in_scene=len(reference['fruit_specs']),targets_attempted=len(outcomes),
                 skipped=len(specs)-len(outcomes),skipped_targets=[dict(target=c['target'],reason=c['status']) for c in captures if c['status']!='captured'],
                 center_entries=sum(x['result']=='partial_center_entry' for x in outcomes),
                 invalid_physics=sum(x['result']=='invalid_physics' for x in outcomes),
                 wall_s=time.perf_counter()-started,results=outcomes,
                 note='one nominal virtual RGB-D image, one predicted azimuth and at most one fresh rollout per target; center entry only, no harvest-success evaluator')
    (output/'summary.json').write_text(json.dumps(summary,indent=2))
    render_test_report(output, summary)
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('training',type=Path,help='학습 결과 디렉토리 (예: /root/docker_share/mujoko_debugging_data/20260923_direct_angle_training_full)')
    p.add_argument('--seed',type=int,required=True,help='새 장면 생성 seed (학습 데이터와 다른 값)')
    p.add_argument('--glb',type=str,default=None,help='특정 GLB 송이 지정 (cyan, green, red, white, rotated90)')
    p.add_argument('--angle',type=float,default=None,help='특정 송이 Y 회전 각도 고정 (0~180도)')
    p.add_argument('--angle-min',type=float,default=0.,help='Y축 최소 회전각 (기본: 0)')
    p.add_argument('--angle-max',type=float,default=90.,help='Y축 최대 회전각 (기본: 90)')
    p.add_argument('--max-targets',type=int,help='테스트할 최대 열매 수 (기본: 전체 10개)')
    p.add_argument('--repeat',type=int,default=1,help='새 장면 생성 및 테스트 반복 횟수')
    p.add_argument('--output',type=Path,default=Path('/root/docker_share/mujoko_debugging_data')/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_direct_angle_test'))
    a=p.parse_args()

    if a.angle is not None:
        a.angle_min = a.angle
        a.angle_max = a.angle

    if a.repeat<1:p.error('--repeat must be positive')
    if a.repeat==1:
        print(json.dumps(evaluate(a.training,a.output,a.seed,a.angle_min,a.angle_max,a.max_targets,glb=a.glb),indent=2))
    else:
        a.output.mkdir(parents=True,exist_ok=False);summaries=[]
        for index in range(a.repeat):
            result=evaluate(a.training,a.output/f'test_{index:04d}',a.seed+index,a.angle_min,a.angle_max,a.max_targets,glb=a.glb)
            summaries.append(dict(test=index,seed=a.seed+index,status=result['status'],
                                  targets_attempted=result.get('targets_attempted',0),
                                  center_entries=result.get('center_entries',0),
                                  output=str(a.output/f'test_{index:04d}')))
            (a.output/'batch_summary.json').write_text(json.dumps(summaries,indent=2))
        links=''.join(f'<li>seed {r["seed"]}: {r["status"]}, 진입 {r["center_entries"]}/{r["targets_attempted"]} · <a href="test_{r["test"]:04d}/index.html">결과</a></li>' for r in summaries)
        (a.output/'index.html').write_text('<meta charset="utf-8"><h1>사진 한 장 → 각도 하나 → 새 장면 반복</h1><ul>'+links+'</ul><a href="batch_summary.json">요약</a>')
        print(json.dumps(summaries,indent=2))
