import argparse
import html
import json
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from statistics import mean


def load_results(session_directory: Path) -> tuple[dict, list[dict]]:
    session_path = session_directory / "session.json"
    results_path = session_directory / "results.jsonl"
    if not session_path.is_file():
        raise FileNotFoundError(f"session.json not found: {session_path}")
    if not results_path.is_file():
        raise FileNotFoundError(f"results.jsonl not found: {results_path}")
    session = json.loads(session_path.read_text(encoding="utf-8"))
    results = []
    for line_number, line in enumerate(
        results_path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        if not line.strip():
            continue
        try:
            results.append(json.loads(line))
        except json.JSONDecodeError as error:
            raise ValueError(
                f"Invalid JSON at {results_path}:{line_number}: {error}"
            ) from error
    return session, results


def _percentage(numerator: int, denominator: int) -> float:
    return 100.0 * numerator / denominator if denominator else 0.0


def _bar_rows(items, total: int, color: str = "#e3b341") -> str:
    rows = []
    maximum = max((value for _label, value in items), default=1)
    for label, value in items:
        width = 100.0 * value / maximum if maximum else 0.0
        rows.append(
            "<div class='bar-row'>"
            f"<div class='bar-label'>{html.escape(str(label))}</div>"
            "<div class='bar-track'>"
            f"<div class='bar-fill' style='width:{width:.2f}%;background:{color}'></div>"
            "</div>"
            f"<div class='bar-value'>{value} ({_percentage(value, total):.1f}%)</div>"
            "</div>"
        )
    return "".join(rows) or "<p class='muted'>데이터 없음</p>"


def _success_rate_rows(grouped: dict) -> str:
    rows = []
    for label, values in sorted(grouped.items()):
        total = len(values)
        success = sum(bool(value) for value in values)
        rate = _percentage(success, total)
        color = "#2ea043" if rate >= 70.0 else "#e3b341" if rate >= 40.0 else "#f85149"
        rows.append(
            "<div class='bar-row'>"
            f"<div class='bar-label'>{html.escape(str(label))}</div>"
            "<div class='bar-track'>"
            f"<div class='bar-fill' style='width:{rate:.2f}%;background:{color}'></div>"
            "</div>"
            f"<div class='bar-value'>{success}/{total} ({rate:.1f}%)</div>"
            "</div>"
        )
    return "".join(rows) or "<p class='muted'>데이터 없음</p>"


def _rotation_groups(results: list[dict]) -> dict:
    grouped = defaultdict(list)
    for result in results:
        rotation = float(result.get("rotation_deg", 0.0)) % 360.0
        lower = int(rotation // 30.0) * 30
        grouped[f"{lower:03d}–{lower + 30:03d}°"].append(
            bool(result.get("success"))
        )
    return grouped


def _failure_table(failures: list[dict], limit: int = 200) -> str:
    sorted_failures = sorted(
        failures,
        key=lambda item: (
            float(item.get("cartesian_fraction"))
            if item.get("cartesian_fraction") not in ("", None)
            else 2.0,
            int(item.get("case", 0)),
        ),
    )
    rows = []
    for result in sorted_failures[:limit]:
        fraction = result.get("cartesian_fraction", "")
        fraction_text = (
            f"{float(fraction):.3f}" if fraction not in ("", None) else "—"
        )
        rows.append(
            "<tr>"
            f"<td>{int(result.get('case', 0))}</td>"
            f"<td>{int(result.get('tomato', 0))}</td>"
            f"<td>{float(result.get('x', 0.0)):.4f}</td>"
            f"<td>{float(result.get('y', 0.0)):.4f}</td>"
            f"<td>{float(result.get('z', 0.0)):.4f}</td>"
            f"<td>{float(result.get('rotation_deg', 0.0)):.1f}°</td>"
            f"<td>{html.escape(str(result.get('failure_stage') or 'UNKNOWN'))}</td>"
            f"<td>{html.escape(str(result.get('failure_reason') or '—'))}</td>"
            f"<td>{fraction_text}</td>"
            f"<td>{float(result.get('duration_sec', 0.0)):.3f}</td>"
            "</tr>"
        )
    return "".join(rows) or "<tr><td colspan='10'>실패 사례 없음</td></tr>"


def _position_scatter(results: list[dict]) -> str:
    if not results:
        return "<p class='muted'>데이터 없음</p>"
    width, height = 760, 300
    padding = 35
    rotations = [float(item.get("rotation_deg", 0.0)) % 360.0 for item in results]
    y_values = [float(item.get("y", 0.0)) for item in results]
    y_min, y_max = min(y_values), max(y_values)
    if y_min == y_max:
        y_min -= 0.005
        y_max += 0.005
    points = []
    for item, rotation, y_value in zip(results, rotations, y_values):
        x_pixel = padding + rotation / 360.0 * (width - 2 * padding)
        y_pixel = (
            height
            - padding
            - (y_value - y_min) / (y_max - y_min) * (height - 2 * padding)
        )
        color = "#2ea043" if item.get("success") else "#f85149"
        tooltip = html.escape(
            f"case={item.get('case')} tomato={item.get('tomato')} "
            f"rotation={rotation:.1f} y={y_value:.4f} "
            f"stage={item.get('failure_stage') or 'SUCCESS'}"
        )
        points.append(
            f"<circle cx='{x_pixel:.2f}' cy='{y_pixel:.2f}' r='3.2' "
            f"fill='{color}' opacity='0.72'><title>{tooltip}</title></circle>"
        )
    return (
        f"<svg viewBox='0 0 {width} {height}' class='scatter' "
        "role='img' aria-label='회전과 Y 위치별 성공 실패'>"
        f"<line x1='{padding}' y1='{height-padding}' x2='{width-padding}' "
        f"y2='{height-padding}' class='axis'/>"
        f"<line x1='{padding}' y1='{padding}' x2='{padding}' "
        f"y2='{height-padding}' class='axis'/>"
        f"<text x='{padding}' y='{height-8}' class='axis-text'>0°</text>"
        f"<text x='{width-padding-25}' y='{height-8}' class='axis-text'>360°</text>"
        f"<text x='4' y='{padding+4}' class='axis-text'>{y_max:.3f}m</text>"
        f"<text x='4' y='{height-padding}' class='axis-text'>{y_min:.3f}m</text>"
        + "".join(points)
        + "</svg>"
    )


def build_report(session_directory: Path) -> str:
    session, results = load_results(session_directory)
    total = len(results)
    successes = [item for item in results if item.get("success")]
    failures = [item for item in results if not item.get("success")]
    recovered = [item for item in results if item.get("recovery_success")]
    durations = [float(item.get("duration_sec", 0.0)) for item in results]
    failure_stages = Counter(
        str(item.get("failure_stage") or "UNKNOWN") for item in failures
    )
    failure_reasons = Counter(
        str(item.get("failure_reason") or "UNKNOWN") for item in failures
    )
    by_tomato = defaultdict(list)
    for item in results:
        by_tomato[int(item.get("tomato", 0))].append(bool(item.get("success")))

    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")
    created_at = session.get("created_at", "—")
    pipeline = session.get("pipeline", "—")
    planner_id = session.get("planner_id", "—")
    preapproach_mode = session.get("preapproach_mode", "—")
    success_rate = _percentage(len(successes), total)
    average_duration = mean(durations) if durations else 0.0
    stage_rows = _bar_rows(failure_stages.most_common(), len(failures), "#f85149")
    reason_rows = _bar_rows(failure_reasons.most_common(), len(failures), "#db6d28")
    tomato_rows = _success_rate_rows(by_tomato)
    rotation_rows = _success_rate_rows(_rotation_groups(results))
    failure_rows = _failure_table(failures)
    scatter = _position_scatter(results)

    return f"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Farmily 수확 Plan 분석 보고서</title>
<style>
:root {{ color-scheme: dark; --bg:#0d1117; --panel:#161b22; --line:#30363d;
  --text:#e6edf3; --muted:#8b949e; --green:#2ea043; --red:#f85149; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--bg); color:var(--text);
  font-family:"Noto Sans KR","NanumGothic",system-ui,sans-serif; }}
main {{ max-width:1500px; margin:auto; padding:28px; }}
h1 {{ margin:0 0 8px; font-size:28px; }}
h2 {{ margin:0 0 16px; font-size:18px; }}
.muted {{ color:var(--muted); }}
.cards {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(180px,1fr));
  gap:12px; margin:22px 0; }}
.card,.panel {{ background:var(--panel); border:1px solid var(--line);
  border-radius:10px; padding:18px; }}
.card .value {{ font-size:28px; font-weight:700; margin-top:6px; }}
.good {{ color:#3fb950; }} .bad {{ color:#ff7b72; }} .warn {{ color:#e3b341; }}
.grid {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(440px,1fr));
  gap:14px; margin-bottom:14px; }}
.panel {{ overflow:hidden; }}
.bar-row {{ display:grid; grid-template-columns:190px 1fr 125px; gap:10px;
  align-items:center; margin:9px 0; font-size:13px; }}
.bar-label {{ white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }}
.bar-track {{ height:13px; background:#21262d; border-radius:5px; overflow:hidden; }}
.bar-fill {{ height:100%; min-width:2px; }}
.bar-value {{ text-align:right; color:var(--muted); }}
.table-wrap {{ overflow:auto; max-height:650px; }}
table {{ width:100%; border-collapse:collapse; font-size:12px; }}
th {{ position:sticky; top:0; background:#21262d; text-align:left; }}
th,td {{ padding:8px 10px; border-bottom:1px solid var(--line); white-space:nowrap; }}
tr:hover td {{ background:#1f242c; }}
.scatter {{ width:100%; min-height:260px; background:#0d1117; border-radius:8px; }}
.axis {{ stroke:#8b949e; stroke-width:1; }} .axis-text {{ fill:#8b949e; font-size:11px; }}
.legend span {{ margin-right:16px; }} .dot {{ display:inline-block; width:9px;
  height:9px; border-radius:50%; margin-right:5px; }}
@media (max-width:700px) {{ main {{ padding:14px; }} .grid {{ grid-template-columns:1fr; }}
  .bar-row {{ grid-template-columns:130px 1fr 95px; }} }}
</style>
</head>
<body><main>
<h1>Farmily 수확 Plan 분석 보고서</h1>
<div class="muted">세션: {html.escape(session_directory.name)} · 생성: {generated_at}</div>
<div class="muted">테스트 시작: {html.escape(str(created_at))} ·
Pipeline: {html.escape(str(pipeline))} · Planner: {html.escape(str(planner_id))} ·
Pre-approach: {html.escape(str(preapproach_mode))}</div>
<section class="cards">
  <div class="card"><div class="muted">전체 Plan</div><div class="value">{total}</div></div>
  <div class="card"><div class="muted">성공</div><div class="value good">{len(successes)}</div></div>
  <div class="card"><div class="muted">실패</div><div class="value bad">{len(failures)}</div></div>
  <div class="card"><div class="muted">성공률</div><div class="value">{success_rate:.1f}%</div></div>
  <div class="card"><div class="muted">Cartesian 대체 복귀</div>
    <div class="value warn">{len(recovered)}</div></div>
  <div class="card"><div class="muted">평균 Plan 시간</div>
    <div class="value">{average_duration:.3f}s</div></div>
</section>
<section class="grid">
  <div class="panel"><h2>실패 단계</h2>{stage_rows}</div>
  <div class="panel"><h2>실패 이유</h2>{reason_rows}</div>
  <div class="panel"><h2>토마토 번호별 성공률</h2>{tomato_rows}</div>
  <div class="panel"><h2>회전 각도 30° 구간별 성공률</h2>{rotation_rows}</div>
</section>
<section class="panel" style="margin-bottom:14px">
  <h2>회전–Y 위치별 결과 분포</h2>
  <div class="legend"><span><i class="dot" style="background:#2ea043"></i>성공</span>
    <span><i class="dot" style="background:#f85149"></i>실패</span></div>
  {scatter}
</section>
<section class="panel">
  <h2>실패 상세 — Cartesian fraction 낮은 순, 최대 200건</h2>
  <div class="table-wrap"><table>
    <thead><tr><th>케이스</th><th>토마토</th><th>X</th><th>Y</th><th>Z</th>
      <th>회전</th><th>실패 단계</th><th>이유</th><th>Fraction</th><th>시간(s)</th></tr></thead>
    <tbody>{failure_rows}</tbody>
  </table></div>
</section>
</main></body></html>"""


def write_report(session_directory: Path, output: Path | None = None) -> Path:
    session_directory = session_directory.expanduser().resolve()
    output_path = (
        output.expanduser().resolve()
        if output is not None
        else session_directory / "report.html"
    )
    output_path.write_text(build_report(session_directory), encoding="utf-8")
    return output_path


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Generate a self-contained HTML report from a harvest session."
    )
    parser.add_argument("session_directory", type=Path)
    parser.add_argument("-o", "--output", type=Path)
    arguments = parser.parse_args(argv)
    report_path = write_report(arguments.session_directory, arguments.output)
    print(report_path)


if __name__ == "__main__":
    main()
