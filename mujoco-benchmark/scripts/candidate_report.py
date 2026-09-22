"""Standalone, offline Korean experiment dashboard; no simulation imports."""
import argparse
import json
import shlex
from pathlib import Path


def generate(root, results=None):
    root = Path(root).resolve()
    results = json.loads((root / 'results.json').read_text()) if results is None else results
    manifest = json.loads((root / 'manifest.json').read_text())
    home = Path(__file__).resolve().parents[1]
    cases = []
    for row in results:
        item = dict(row)
        folder = root / 'candidates' / row['candidate_id']
        item['replay_available'] = (folder / 'states.npz').is_file()
        item['command'] = 'DISPLAY=:0 ' + ' '.join(shlex.quote(str(x)) for x in (
            home / '.venv/bin/python', home / 'scripts/replay_candidate.py', root,
            '--candidate', row['candidate_id']))
        cases.append(item)
    payload = json.dumps(dict(manifest=manifest, cases=cases, run=root.name), ensure_ascii=False).replace('<', '\\u003c')
    assets = Path(__file__).with_name('report_assets')
    template = (assets / 'candidate.html').read_text()
    page = template.replace('/*REPORT_CSS*/', (assets / 'candidate.css').read_text()).replace('/*REPORT_JS*/', (assets / 'candidate.js').read_text()).replace('/*REPORT_DATA*/', payload)
    temporary = root / 'index.html.tmp'
    temporary.write_text(page, encoding='utf-8')
    temporary.replace(root / 'index.html')
    return root / 'index.html'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    print(generate(parser.parse_args().run_dir))
