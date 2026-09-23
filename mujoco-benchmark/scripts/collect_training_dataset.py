"""GLB 5종 각 1송이 학습 데이터 수집 스크립트.

토마토 종류별(총 5송이) 1번씩, 알맹이 1개당 20회 후보 실험 + 20장 RGB-D 관측 수집.

제약 조건:
  - Y축 회전 0~20도 (GLB 대칭상 ±20도와 동등)
  - 부착 segment 6~10 (너무 높거나 낮은 위치 제외)
  - candidates=20, views=20
"""
import argparse
import datetime
import fcntl
import hashlib
import html
import json
import sys
import time
from pathlib import Path

from collect_tomatoes import log_status, run_step


# GLB 5종 파일명과 짧은 식별자 목록 (순서 고정)
GLB_VARIANTS = [
    ("tomato_master_v10_cluster_curve_cyan.glb",    "cyan"),
    ("tomato_master_v10_cluster_curve_green.glb",   "green"),
    ("tomato_master_v10_cluster_curve_red.glb",     "red"),
    ("tomato_master_v10_cluster_curve_white.glb",   "white"),
    ("tomato_master_v10_cluster_rotated90.glb",     "rotated90"),
]

# 수집 설정
ANGLE_MIN   = 0.0    # Y축 회전 최솟값(도) — GLB 대칭상 ±20도와 동등
ANGLE_MAX   = 20.0   # Y축 회전 최댓값(도)
SEGMENT_MIN = 6      # 부착 segment 하한 (너무 낮은 위치 제외)
SEGMENT_MAX = 10     # 부착 segment 상한 (너무 높은 위치 제외)
VIEWS_ARG   = 71     # prepare_observations 에 전달할 --views (9 또는 71만 지원)
VIEWS_LIMIT = 20     # 실제로 수집할 이미지 수 (--limit으로 앞에서 자름)


def format_duration(seconds):
    s = int(seconds)
    m, s = divmod(s, 60)
    h, m = divmod(m, 60)
    if h > 0:
        return f"{h}시간 {m}분 {s}초"
    elif m > 0:
        return f"{m}분 {s}초"
    return f"{s}초"


def render_index_html(output_dir: Path, manifest: dict):
    targets = manifest.get("targets", [])
    total_targets = len(targets)
    total_candidates = sum(t.get("candidates", 0) for t in targets)
    total_success = sum(t.get("center_entries", 0) for t in targets)
    total_failed = total_candidates - total_success
    overall_rate = (total_success / total_candidates * 100) if total_candidates > 0 else 0
    total_views = sum(t.get("views", 0) for t in targets)
    total_planned = sum(t.get("planned", 0) for t in targets)
    wall_s = manifest.get("wall_s", 0)
    status = manifest.get("status", "running")

    scenes = {}
    for t in targets:
        sc = t.get("scene", "unknown")
        if sc not in scenes:
            scenes[sc] = {"targets": 0, "candidates": 0, "success": 0, "views": 0, "planned": 0}
        scenes[sc]["targets"] += 1
        scenes[sc]["candidates"] += t.get("candidates", 0)
        scenes[sc]["success"] += t.get("center_entries", 0)
        scenes[sc]["views"] += t.get("views", 0)
        scenes[sc]["planned"] += t.get("planned", 0)

    scene_summary_rows = []
    for sc, sdata in scenes.items():
        sc_cands = sdata["candidates"]
        sc_succ = sdata["success"]
        sc_fail = sc_cands - sc_succ
        sc_rate = (sc_succ / sc_cands * 100) if sc_cands > 0 else 0
        scene_summary_rows.append(f"""
        <tr>
          <td><span class="scene-tag scene-{html.escape(sc)}"><strong>{html.escape(sc)}</strong></span></td>
          <td class="num">{sdata['targets']}</td>
          <td class="num">{sc_cands}</td>
          <td class="num text-success"><strong>{sc_succ}</strong></td>
          <td class="num text-danger">{sc_fail}</td>
          <td>
            <div class="rate-bar-container">
              <div class="rate-bar-bg"><div class="rate-bar" style="width: {sc_rate:.1f}%;"></div></div>
              <span class="rate-text">{sc_rate:.1f}%</span>
            </div>
          </td>
          <td class="num">{sdata['planned']}</td>
          <td class="num">{sdata['views']}</td>
        </tr>""")

    target_rows = []
    for idx, t in enumerate(targets, 1):
        sc = t.get("scene", "")
        tgt = t.get("target", "")
        cands = t.get("candidates", 0)
        succ = t.get("center_entries", 0)
        fail = cands - succ
        rate = (succ / cands * 100) if cands > 0 else 0
        planned = t.get("planned", 0)
        views = t.get("views", 0)
        rc = t.get("result_counts", {})
        rc_str = ", ".join(f"{k}: {v}" for k, v in rc.items())
        phys_url = f"scene_{sc}/targets/{tgt}/physics/index.html"
        obs_url = f"scene_{sc}/targets/{tgt}/observations/index.html"

        rate_badge_cls = "badge-success" if rate >= 50 else ("badge-warning" if rate > 0 else "badge-danger")

        target_rows.append(f"""
        <tr data-scene="{html.escape(sc)}" data-target="{html.escape(tgt)}">
          <td class="num">{idx}</td>
          <td><span class="scene-tag scene-{html.escape(sc)}">{html.escape(sc)}</span></td>
          <td><strong>{html.escape(tgt)}</strong></td>
          <td class="num">{cands}</td>
          <td class="num text-success"><strong>{succ}</strong></td>
          <td class="num text-danger">{fail}</td>
          <td>
            <div class="rate-bar-container">
              <span class="badge {rate_badge_cls}">{rate:.0f}%</span>
              <div class="rate-bar-bg" style="width: 50px;"><div class="rate-bar" style="width: {rate:.1f}%;"></div></div>
            </div>
          </td>
          <td class="num">{planned}</td>
          <td class="num">{views}</td>
          <td class="breakdown-cell" title="{html.escape(rc_str)}"><span class="tiny-text">{html.escape(rc_str)}</span></td>
          <td class="links-cell">
            <a class="btn-link" href="{phys_url}">물리 결과</a>
            <a class="btn-link" href="{obs_url}">RGB-D ({views})</a>
          </td>
        </tr>""")

    css_content = """
    :root {
      --bg: #f8faf9;
      --card-bg: #ffffff;
      --text: #1a202c;
      --subtext: #5a6a75;
      --line: #e2e8f0;
      --green: #198754;
      --green-light: #e8f5e9;
      --red: #dc3545;
      --red-light: #ffebee;
      --orange: #fd7e14;
      --orange-light: #fff3cd;
      --primary: #0f5132;
      --accent: #2e7d32;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Noto Sans KR", sans-serif;
      background: var(--bg);
      color: var(--text);
      line-height: 1.5;
      padding: 30px 40px;
    }
    .container {
      max-width: 1500px;
      margin: 0 auto;
    }
    header {
      display: flex;
      justify-content: space-between;
      align-items: flex-start;
      margin-bottom: 25px;
      padding-bottom: 20px;
      border-bottom: 2px solid var(--line);
    }
    h1 {
      font-size: 26px;
      font-weight: 700;
      color: var(--primary);
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .status-badge {
      display: inline-block;
      font-size: 13px;
      font-weight: 600;
      padding: 4px 12px;
      border-radius: 20px;
      background: var(--green-light);
      color: var(--green);
      text-transform: uppercase;
    }
    .subtitle {
      color: var(--subtext);
      font-size: 14px;
      margin-top: 6px;
    }
    .stats-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 16px;
      margin-bottom: 30px;
    }
    .stat-card {
      background: var(--card-bg);
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 20px;
      box-shadow: 0 1px 3px rgba(0,0,0,0.03);
    }
    .stat-label {
      font-size: 12px;
      font-weight: 600;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      color: var(--subtext);
      margin-bottom: 8px;
    }
    .stat-value {
      font-size: 32px;
      font-weight: 700;
      color: var(--text);
      line-height: 1.1;
    }
    .stat-value.success { color: var(--green); }
    .stat-value.danger { color: var(--red); }
    .stat-desc {
      font-size: 13px;
      color: var(--subtext);
      margin-top: 6px;
    }
    .section-title {
      font-size: 18px;
      font-weight: 700;
      color: var(--text);
      margin: 25px 0 14px;
      display: flex;
      align-items: center;
      justify-content: space-between;
    }
    .card {
      background: var(--card-bg);
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 20px;
      margin-bottom: 25px;
      box-shadow: 0 1px 3px rgba(0,0,0,0.03);
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
      text-align: left;
    }
    th, td {
      padding: 12px 14px;
      border-bottom: 1px solid var(--line);
    }
    th {
      background: #f1f5f3;
      font-weight: 600;
      color: #334155;
      cursor: pointer;
      user-select: none;
      white-space: nowrap;
    }
    th:hover {
      background: #e2ece6;
    }
    th::after {
      content: " ↕";
      opacity: 0.3;
      font-size: 11px;
    }
    tr:hover td {
      background: #fbfdfc;
    }
    td.num, th.num {
      text-align: right;
      font-variant-numeric: tabular-nums;
    }
    .text-success { color: var(--green); }
    .text-danger { color: var(--red); }
    .badge {
      display: inline-block;
      padding: 3px 8px;
      border-radius: 6px;
      font-size: 11px;
      font-weight: 700;
      text-align: center;
    }
    .badge-success { background: var(--green-light); color: var(--green); }
    .badge-warning { background: var(--orange-light); color: var(--orange); }
    .badge-danger { background: var(--red-light); color: var(--red); }
    .scene-tag {
      display: inline-block;
      padding: 2px 8px;
      border-radius: 4px;
      font-size: 11px;
      font-weight: 600;
      background: #edf2f7;
      color: #4a5568;
    }
    .scene-cyan { background: #e0f7fa; color: #00838f; }
    .scene-green { background: #e8f5e9; color: #2e7d32; }
    .scene-red { background: #ffebee; color: #c62828; }
    .scene-white { background: #eceff1; color: #455a64; border: 1px solid #cfd8dc; }
    .scene-rotated90 { background: #ede7f6; color: #512da8; }
    .rate-bar-container {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .rate-bar-bg {
      height: 8px;
      background: #e2e8f0;
      border-radius: 4px;
      overflow: hidden;
      flex-grow: 1;
      min-width: 60px;
    }
    .rate-bar {
      height: 100%;
      background: var(--green);
      border-radius: 4px;
    }
    .rate-text {
      font-size: 12px;
      font-weight: 600;
      min-width: 45px;
      text-align: right;
    }
    .btn-link {
      display: inline-block;
      padding: 4px 8px;
      border-radius: 4px;
      font-size: 11px;
      font-weight: 600;
      text-decoration: none;
      color: var(--primary);
      background: #e8f3ed;
      margin-right: 4px;
      transition: background 0.15s;
    }
    .btn-link:hover {
      background: #cbe6d8;
    }
    .breakdown-cell {
      max-width: 250px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .tiny-text {
      font-size: 11px;
      color: var(--subtext);
    }
    .filter-bar {
      display: flex;
      gap: 16px;
      align-items: center;
      flex-wrap: wrap;
    }
    .filter-group {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .filter-label {
      font-size: 13px;
      font-weight: 600;
      color: var(--subtext);
    }
    select, input[type="text"] {
      padding: 6px 12px;
      border: 1px solid var(--line);
      border-radius: 6px;
      font-size: 13px;
      background: white;
      color: var(--text);
    }
    input[type="text"] { min-width: 200px; }
    .manifest-link {
      color: var(--primary);
      font-size: 13px;
      font-weight: 600;
      text-decoration: none;
    }
    .manifest-link:hover { text-decoration: underline; }
    """

    js_content = """
    const sceneSelect = document.getElementById('scene-select');
    const searchInput = document.getElementById('search-input');
    const statusFilter = document.getElementById('status-filter');
    const tableBody = document.getElementById('table-body');
    const filterCount = document.getElementById('filter-count');
    const rows = Array.from(tableBody.querySelectorAll('tr'));

    function applyFilter() {
      const sceneVal = sceneSelect.value.toLowerCase();
      const searchVal = searchInput.value.trim().toLowerCase();
      const statusVal = statusFilter.value;

      let visible = 0;
      rows.forEach(r => {
        const rScene = (r.getAttribute('data-scene') || '').toLowerCase();
        const rTarget = (r.getAttribute('data-target') || '').toLowerCase();
        const succ = parseInt(r.children[4].textContent, 10) || 0;

        let matchScene = (sceneVal === 'all' || rScene === sceneVal);
        let matchSearch = (!searchVal || rTarget.includes(searchVal) || rScene.includes(searchVal));
        let matchStatus = true;
        if (statusVal === 'success_only') matchStatus = (succ > 0);
        if (statusVal === 'fail_only') matchStatus = (succ === 0);

        if (matchScene && matchSearch && matchStatus) {
          r.style.display = '';
          visible++;
        } else {
          r.style.display = 'none';
        }
      });
      filterCount.textContent = `표시 중: ${visible} / ${rows.length}개 열매`;
    }

    sceneSelect.addEventListener('change', applyFilter);
    searchInput.addEventListener('input', applyFilter);
    statusFilter.addEventListener('change', applyFilter);

    const headers = document.querySelectorAll('#detail-table th[data-col]');
    let sortDir = {};

    headers.forEach(th => {
      th.addEventListener('click', () => {
        const colIdx = parseInt(th.getAttribute('data-col'), 10);
        const currentDir = sortDir[colIdx] === 'asc' ? 'desc' : 'asc';
        sortDir = {};
        sortDir[colIdx] = currentDir;

        const sorted = rows.slice().sort((a, b) => {
          let aVal = a.children[colIdx].textContent.trim();
          let bVal = b.children[colIdx].textContent.trim();

          let aNum = parseFloat(aVal.replace(/[^0-9.-]/g, ''));
          let bNum = parseFloat(bVal.replace(/[^0-9.-]/g, ''));

          if (!isNaN(aNum) && !isNaN(bNum)) {
            return currentDir === 'asc' ? aNum - bNum : bNum - aNum;
          }
          return currentDir === 'asc' ? aVal.localeCompare(bVal) : bVal.localeCompare(aVal);
        });

        sorted.forEach(r => tableBody.appendChild(r));
      });
    });
    """

    avg_views = total_views // max(1, total_targets)
    scene_options = ''.join(f'<option value="{html.escape(sc)}">{html.escape(sc)}</option>' for sc in scenes)
    scene_rows_str = ''.join(scene_summary_rows)
    target_rows_str = ''.join(target_rows)

    html_content = f"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <title>토마토 학습 데이터 수집 리포트</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>{css_content}</style>
</head>
<body>
  <div class="container">
    <header>
      <div>
        <h1>
          <span>토마토 학습 데이터 수집 리포트</span>
          <span class="status-badge">{status.upper()}</span>
        </h1>
        <div class="subtitle">
          GLB 5종(cyan, green, red, white, rotated90) · Y축 회전 ±20° · Segment {SEGMENT_MIN}~{SEGMENT_MAX} · 
          총 소요 시간: <strong>{format_duration(wall_s)}</strong>
        </div>
      </div>
      <div>
        <a class="manifest-link" href="collection.json" target="_blank">📄 collection.json 보기</a>
      </div>
    </header>

    <!-- 종합 통계 카드 -->
    <div class="stats-grid">
      <div class="stat-card">
        <div class="stat-label">총 토마토 열매</div>
        <div class="stat-value">{total_targets} <span style="font-size: 18px; font-weight: 400; color: var(--subtext);">개</span></div>
        <div class="stat-desc">5종 송이 × 10개 열매</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">총 후보 시도</div>
        <div class="stat-value">{total_candidates} <span style="font-size: 18px; font-weight: 400; color: var(--subtext);">회</span></div>
        <div class="stat-desc">열매당 {manifest.get('candidates_per_target', 10)}회 시도</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">진입 성공 (성공)</div>
        <div class="stat-value success">{total_success} <span style="font-size: 18px; font-weight: 400;">회</span></div>
        <div class="stat-desc">전체 성공률: <strong>{overall_rate:.1f}%</strong></div>
      </div>
      <div class="stat-card">
        <div class="stat-label">미진입 / 오류 (실패)</div>
        <div class="stat-value danger">{total_failed} <span style="font-size: 18px; font-weight: 400;">회</span></div>
        <div class="stat-desc">전체 실패율: <strong>{(100-overall_rate):.1f}%</strong></div>
      </div>
      <div class="stat-card">
        <div class="stat-label">관측 RGB-D</div>
        <div class="stat-value">{total_views:,} <span style="font-size: 18px; font-weight: 400; color: var(--subtext);">장</span></div>
        <div class="stat-desc">열매당 평균 {avg_views}시점</div>
      </div>
    </div>

    <!-- 송이별 요약 테이블 -->
    <div class="section-title">
      <span>송이(Scene)별 요약</span>
    </div>
    <div class="card" style="padding: 0; overflow: hidden;">
      <table>
        <thead>
          <tr>
            <th>송이 (Scene)</th>
            <th class="num">열매 수</th>
            <th class="num">후보 시도</th>
            <th class="num">성공 (진입)</th>
            <th class="num">실패</th>
            <th>성공률</th>
            <th class="num">계획 통과</th>
            <th class="num">관측 이미지</th>
          </tr>
        </thead>
        <tbody>
          {scene_rows_str}
        </tbody>
      </table>
    </div>

    <!-- 상세 테이블 섹션 -->
    <div class="section-title">
      <span>열매별 수집 및 판정 상세</span>
      <span id="filter-count" style="font-size: 13px; font-weight: normal; color: var(--subtext);">전체 {total_targets}개 열매</span>
    </div>

    <!-- 필터 및 검색 툴바 -->
    <div class="card" style="padding: 14px 20px;">
      <div class="filter-bar">
        <div class="filter-group">
          <label class="filter-label" for="scene-select">송이 선택:</label>
          <select id="scene-select">
            <option value="all">전체 송이 (All Scenes)</option>
            {scene_options}
          </select>
        </div>
        <div class="filter-group">
          <label class="filter-label" for="search-input">열매 검색:</label>
          <input type="text" id="search-input" placeholder="예: Tomato_05">
        </div>
        <div class="filter-group">
          <label class="filter-label" for="status-filter">결과 필터:</label>
          <select id="status-filter">
            <option value="all">전체 보기</option>
            <option value="success_only">성공 1회 이상</option>
            <option value="fail_only">성공 0회 (미발견)</option>
          </select>
        </div>
      </div>
    </div>

    <!-- 열매별 상세 테이블 -->
    <div class="card" style="padding: 0; overflow-x: auto;">
      <table id="detail-table">
        <thead>
          <tr>
            <th class="num" data-col="0">#</th>
            <th data-col="1">송이</th>
            <th data-col="2">열매 (Target)</th>
            <th class="num" data-col="3">후보 시도</th>
            <th class="num" data-col="4">성공 (진입)</th>
            <th class="num" data-col="5">실패</th>
            <th data-col="6">성공률</th>
            <th class="num" data-col="7">계획 통과</th>
            <th class="num" data-col="8">시점 (Views)</th>
            <th data-col="9">판정 내역 (Breakdown)</th>
            <th>상세 링크</th>
          </tr>
        </thead>
        <tbody id="table-body">
          {target_rows_str}
        </tbody>
      </table>
    </div>
  </div>

  <script>{js_content}</script>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_content, encoding="utf-8")


def complete_target(folder, model_hash, target, count, n_views):
    """이미 완료된 target인지 확인한다."""
    try:
        physics = folder / "physics"
        observations = folder / "observations"
        run = json.loads((physics / "manifest.json").read_text())
        results = json.loads((physics / "results.json").read_text())
        meta = json.loads((observations / "dataset.json").read_text())
        rows = [json.loads(line)
                for line in (observations / "observations.jsonl").read_text().splitlines()]
        return (run["model_sha256"] == model_hash
                and run["target"] == target
                and run["count"] == count
                and len(results) == count
                and meta["target"] == target
                and meta["observations"] == n_views
                and len(rows) == n_views
                and all(
                    (observations / "observations" / row["observation_id"] / "rgb.png").is_file()
                    and (observations / "observations" / row["observation_id"] / "depth_aligned_to_color_m.npy").is_file()
                    for row in rows
                )
                and meta["source_results_sha256"] == hashlib.sha256(
                    (physics / "results.json").read_bytes()).hexdigest()
                and (meta.get("schema") == "farmily_observation_v2"
                     or meta.get("pose_label_actions", 0) == 0))
    except (OSError, KeyError, ValueError):
        return False


def build_single_glb_scene(output_dir, glb_path, label, label_index, seed,
                            segment_min, segment_max,
                            angle_min, angle_max,
                            idle_seconds, truss_scale, render):
    """단일 GLB 파일로 장면 하나를 생성한다.

    generate_random_glb_scenes.generate()의 핵심 로직을 단일 GLB에 대해 실행.
    """
    import copy
    import hashlib as _hs
    import shutil
    import xml.etree.ElementTree as ET
    import numpy as np
    import mujoco as mj
    from PIL import Image
    from build_glb_physics import build
    from generate_random_glb_scenes import inspect, preview, WHITE_OFFSET
    from robot_engine import RobotEngine  # noqa: F401 (render_backend must be loaded first)

    scene_dir = output_dir / f"scene_{label}"
    scene_dir.mkdir(parents=True, exist_ok=False)

    rng = np.random.default_rng(np.random.SeedSequence([seed, label_index]))

    # GLB 프로파일 결정
    white = glb_path.name == "tomato_master_v10_cluster_curve_white.glb"
    fruit_offsets = {2: WHITE_OFFSET} if white else {}
    rachis_stiffness_scale = 2.0 if white else 1.0
    remove_fruits = [6]

    # segment와 Y 회전 샘플링 (구간 중간에서 1개)
    # fraction을 0.25~0.75로 좁혀 segment 경계 근처 불안정 배치 방지
    boundaries = np.linspace(segment_min, segment_max + 1, 2)
    low, high = boundaries[0], boundaries[1]
    position = float(rng.uniform(low, high))
    segment = min(int(np.floor(position)), segment_max)
    fraction = float(np.clip(position - segment, .25, .75))
    y_deg = float(rng.uniform(angle_min, angle_max))

    placement = dict(
        source_glb=str(glb_path.resolve()),
        source_sha256=_hs.sha256(glb_path.read_bytes()).hexdigest(),
        stem_segment=segment,
        stem_fraction=fraction,
        y_deg=y_deg,
        remove_fruits=remove_fruits,
        fruit_offsets=fruit_offsets,
        rachis_stiffness_scale=rachis_stiffness_scale,
        truss_scale=truss_scale,
    )

    log_status(f"  장면 생성: {label} | segment={segment}+{fraction:.2f} | Y={y_deg:.1f}°")

    part = scene_dir / "parts" / "truss_00"
    build(glb_path, part,
          y_deg=y_deg, segment=segment, stem_fraction=fraction,
          remove_fruits=remove_fruits, fruit_offsets=fruit_offsets,
          rachis_stiffness_scale=rachis_stiffness_scale,
          truss_scale=truss_scale)

    metadata = json.loads((part / "build.json").read_text())
    placement["truss_scale"] = truss_scale
    placement["attachment_world_m"] = metadata["attachment_world_m"]

    for filename in ("model.xml", "model.mjb", "reference.json"):
        (scene_dir / filename).write_bytes((part / filename).read_bytes())
    shutil.rmtree(scene_dir / "parts")

    model = mj.MjModel.from_binary_path(str(scene_dir / "model.mjb"))
    data = mj.MjData(model)
    mj.mj_forward(model, data)

    check = inspect(scene_dir / "model.mjb", scene_dir / "reference.json",
                    seconds=idle_seconds)
    visual = preview(model, data, scene_dir, [placement]) if render else None

    record = dict(
        scene=scene_dir.name, label=label, glb=glb_path.name, seed=seed,
        placements=[placement], validation=check, preview=visual,
        model_sha256=_hs.sha256((scene_dir / "model.mjb").read_bytes()).hexdigest(),
        physics_ready=False,
        note="학습 데이터 수집용 단일-GLB 장면",
    )
    (scene_dir / "scene.json").write_text(json.dumps(record, indent=2))

    print(f"  {label}: segment={segment}+{fraction:.2f} Y={y_deg:.1f}° "
          f"screen={check['scene_screen_passed']} "
          f"max_depth_mm={round(check['max_penetration_m']*1000,3)}",
          flush=True)
    return record


def collect(args):
    """5종 GLB 학습 데이터 수집 메인 로직."""
    if min(args.candidates, args.workers, args.planning_workers,
           args.postprocess_workers, args.hz) < 1:
        raise ValueError("All counts and Hz must be positive")

    source_dir = args.source_dir.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=args.resume)

    # 중복 실행 방지 잠금
    lock = (output / "collection.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

    n_views = args.views_limit if not args.smoke else 9

    manifest_path = output / "collection.json"
    if args.resume:
        manifest = json.loads(manifest_path.read_text())
    else:
        manifest = dict(
            schema="training_dataset_v1",
            seed=args.seed,
            angle_min=ANGLE_MIN, angle_max=ANGLE_MAX,
            segment_min=SEGMENT_MIN, segment_max=SEGMENT_MAX,
            candidates_per_target=args.candidates,
            views_per_target=n_views,
            truss_scale=args.truss_scale,
            hz=args.hz,
            status="running",
            glb_scenes=[],
            targets=[],
            failures=[],
        )
        manifest_path.write_text(json.dumps(manifest, indent=2))

    python = sys.executable
    started = time.perf_counter()

    # smoke 모드: cyan 1종, Tomato_05만, candidates=2
    variants = (GLB_VARIANTS[:1] if args.smoke else GLB_VARIANTS)
    smoke_target_only = args.smoke

    scenes_dir = output / "scenes"
    scenes_dir.mkdir(exist_ok=True)

    # ── 장면 생성 단계 ──────────────────────────────────────────────
    log_status("=== 장면 생성 단계 ===")
    existing_labels = {s["label"] for s in manifest.get("glb_scenes", [])}
    records = list(manifest.get("glb_scenes", []))

    MAX_RETRIES = 5  # 물리 검사 실패 시 최대 재시도 횟수 (seed offset 변경)
    for glb_idx, (glb_name, label) in enumerate(variants):
        if args.resume and label in existing_labels:
            log_status(f"  장면 재사용: {label}")
            continue
        glb_path = source_dir / glb_name
        if not glb_path.exists():
            raise FileNotFoundError(f"GLB 파일 없음: {glb_path}")

        record = None
        for attempt in range(MAX_RETRIES):
            attempt_seed = args.seed + attempt * 1000  # seed offset으로 다른 배치 시도
            scene_label = label if attempt == 0 else f"{label}_attempt{attempt}"
            # 이전 실패한 scene 디렉토리 정리
            failed_dir = scenes_dir / f"scene_{scene_label}"
            if failed_dir.exists() and attempt > 0:
                import shutil as _shutil
                _shutil.rmtree(failed_dir)
            try:
                record = build_single_glb_scene(
                    scenes_dir, glb_path, scene_label, glb_idx, attempt_seed,
                    SEGMENT_MIN, SEGMENT_MAX, ANGLE_MIN, ANGLE_MAX,
                    args.idle_seconds, args.truss_scale, not args.no_render,
                )
            except Exception as e:
                log_status(f"  장면 생성 오류 ({label} 시도 {attempt+1}/{MAX_RETRIES}): {e}")
                continue
            if record["validation"]["scene_screen_passed"]:
                # 성공: 디렉토리 이름을 label로 정규화
                if scene_label != label:
                    final_dir = scenes_dir / f"scene_{label}"
                    (scenes_dir / f"scene_{scene_label}").rename(final_dir)
                    record["scene"] = f"scene_{label}"
                    record["label"] = label
                log_status(f"  장면 물리 검사 통과: {label} (시도 {attempt+1})")
                break
            else:
                log_status(f"  물리 검사 실패, 재시도 ({label} {attempt+1}/{MAX_RETRIES}): "
                           f"idle_motion={record['validation']['max_idle_motion_m']*1000:.2f}mm")
                # 실패한 디렉토리 정리 후 재시도
                failed_dir = scenes_dir / f"scene_{scene_label}"
                if failed_dir.exists():
                    import shutil as _shutil
                    _shutil.rmtree(failed_dir)
                record = None
        if record is None:
            raise RuntimeError(f"GLB 장면 물리 검사 {MAX_RETRIES}회 모두 실패: {label}")

        records.append(record)
        manifest["glb_scenes"] = records
        manifest_path.write_text(json.dumps(manifest, indent=2))

    # ── 물리 + 관측 수집 단계 ─────────────────────────────────────
    log_status("=== 물리·관측 수집 단계 ===")
    try:
        for record in records:
            label = record["label"]
            scene_dir = scenes_dir / f"scene_{label}"
            reference = json.loads((scene_dir / "reference.json").read_text())
            model_hash = record["model_sha256"]

            if not record["validation"]["scene_screen_passed"]:
                manifest["failures"].append(dict(scene=label, reason="scene_screen_failed"))
                log_status(f"  초기 물리 검사 실패, 제외: {label}")
                manifest_path.write_text(json.dumps(manifest, indent=2))
                continue

            fruit_specs = reference["fruit_specs"]
            if smoke_target_only:
                # smoke: Tomato_05만 (또는 첫 번째 열매)
                fruit_specs = [s for s in fruit_specs if s["name"] == "Tomato_05"]
                if not fruit_specs:
                    fruit_specs = reference["fruit_specs"][:1]
                candidates_n = min(2, args.candidates)
            else:
                candidates_n = args.candidates

            for index, spec in enumerate(fruit_specs):
                target = spec["name"]
                folder = output / f"scene_{label}" / "targets" / target
                folder.mkdir(parents=True, exist_ok=True)
                physics = folder / "physics"
                observations = folder / "observations"

                if args.resume and complete_target(folder, model_hash, target,
                                                   candidates_n, n_views):
                    log_status(f"  완료 재사용: {label}/{target}")
                    continue
                if args.resume and observations.exists():
                    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    observations.rename(folder / f"observations_interrupted_{ts}")

                # 물리 후보 실험
                if physics.exists():
                    run_meta = json.loads((physics / "manifest.json").read_text())
                    if (run_meta["model_sha256"] != model_hash
                            or run_meta["target"] != target
                            or run_meta["count"] != candidates_n):
                        raise ValueError(f"기존 물리 실행 설정 불일치: {physics}")
                    command = [python,
                               str(Path(__file__).with_name("resume_candidates.py")),
                               str(physics),
                               "--workers", str(args.workers),
                               "--planning-workers", str(args.planning_workers)]
                else:
                    command = [python,
                               str(Path(__file__).with_name("candidate_experiment.py")),
                               "--model", str(scene_dir / "model.mjb"),
                               "--reference", str(scene_dir / "reference.json"),
                               "--target", target,
                               "--candidates", str(candidates_n),
                               "--workers", str(args.workers),
                               "--planning-workers", str(args.planning_workers),
                               "--hz", str(args.hz),
                               "--seed", str(args.seed + index),
                               "--link-model",
                               "--output", str(physics)]

                log_status(f"  물리 실험: {label}/{target} ({candidates_n}회)")
                with (folder / "physics.log").open("a" if args.resume else "w") as log:
                    run_step(command, log)

                # RGB-D 관측 수집
                if args.smoke or args.views_limit == 9:
                    views_arg = "9"
                    limit_arg = None
                else:
                    views_arg = "71"
                    limit_arg = str(args.views_limit)

                capture = [python,
                           str(Path(__file__).with_name("prepare_observations.py")),
                           str(physics),
                           "--views", views_arg,
                           "--seed", str(args.seed + index),
                           "--postprocess-workers", str(args.postprocess_workers),
                           "--output", str(observations)]
                if limit_arg is not None:
                    capture.extend(["--limit", limit_arg])

                log_status(f"  관측 촬영: {label}/{target} ({n_views}장)")
                with (folder / "observations.log").open("w") as log:
                    run_step(capture, log)

                results = json.loads((physics / "results.json").read_text())
                meta = json.loads((observations / "dataset.json").read_text())
                row = dict(
                    scene=label, target=target,
                    physics=str(physics), observations=str(observations),
                    candidates=len(results),
                    planned=sum(
                        bool(r.get("preflight", {}).get("passed")) for r in results),
                    center_entries=sum(
                        r["result"] == "partial_center_entry" for r in results),
                    result_counts={
                        lbl: sum(r["result"] == lbl for r in results)
                        for lbl in sorted({r["result"] for r in results})},
                    views=meta["observations"],
                    model_sha256=model_hash,
                    search_outcome=(
                        "entry_found"
                        if any(r["result"] == "partial_center_entry" for r in results)
                        else "not_found_in_tested_candidates"),
                )
                manifest["targets"] = (
                    [r for r in manifest["targets"]
                     if (r["scene"], r["target"]) != (label, target)]
                    + [row])
                manifest_path.write_text(json.dumps(manifest, indent=2))
                log_status(f"  완료: {label}/{target} "
                           f"| 진입={row['center_entries']}/{candidates_n} "
                           f"| 관측={row['views']}장")

        manifest["status"] = "complete"
    except BaseException as error:
        manifest["status"] = "interrupted_or_failed"
        manifest["error"] = repr(error)
        raise
    finally:
        manifest["wall_s"] = manifest.get("wall_s", 0) + time.perf_counter() - started
        manifest_path.write_text(json.dumps(manifest, indent=2))

        render_index_html(output, manifest)
        log_status(f"리포트 생성 완료: {output / 'index.html'}")
    return manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path,
                   default=Path("/root/docker_share/mujoko_debugging_data") /
                   (datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + "_training_dataset"))
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--candidates", type=int, default=20,
                   help="열매당 물리 후보 실험 횟수")
    p.add_argument("--views-limit", type=int, default=VIEWS_LIMIT,
                   help="열매당 관측 이미지 수 (prepare_observations --limit)")
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--planning-workers", type=int, default=4)
    p.add_argument("--postprocess-workers", type=int, default=4)
    p.add_argument("--hz", type=int, default=240)
    p.add_argument("--truss-scale", type=float, default=0.5,
                   help="송이 크기 배율 (0.5=반 크기)")
    p.add_argument("--idle-seconds", type=float, default=2.0,
                   help="장면 초기 물리 검사 시간(초)")
    p.add_argument("--source-dir", type=Path,
                   default=Path(__file__).resolve().parents[2] /
                   "nvidia-sim/env_usd/tomato_rotate_glb",
                   help="GLB 파일 디렉토리")
    p.add_argument("--no-render", action="store_true",
                   help="장면 preview.png 렌더링 생략")
    p.add_argument("--resume", action="store_true",
                   help="완료된 대상은 건너뛰고 이어서 수집")
    p.add_argument("--smoke", action="store_true",
                   help="smoke 테스트: cyan 1종 · Tomato_05만 · candidates=2 · views=9")
    args = p.parse_args()

    log_status(f"=== 토마토 학습 데이터 수집 시작 ===")
    log_status(f"  GLB {len(GLB_VARIANTS)}종 × 열매 10개 × "
               f"후보 {args.candidates}회 × 관측 {args.views_limit}장")
    log_status(f"  Y축 {ANGLE_MIN}~{ANGLE_MAX}° (±20° 동등) | "
               f"segment {SEGMENT_MIN}~{SEGMENT_MAX}")
    log_status(f"  출력 디렉토리: {args.output}")
    if args.smoke:
        log_status("  [SMOKE 모드] cyan 1종 · Tomato_05 · candidates=2 · views=9")

    result = collect(args)
    total = sum(t.get("views", 0) for t in result.get("targets", []))
    log_status(f"=== 완료: 상태={result['status']} | 총 이미지={total}장 ===")
    log_status(f"  결과: {args.output}/index.html")


if __name__ == "__main__":
    main()
