#!/usr/bin/env python3
"""Build an offline, self-contained candidate dashboard from a pool CSV and records."""
import argparse
import base64
import csv
import datetime
import json
from pathlib import Path


def build(root):
    with (root / 'candidates.csv').open(newline='') as stream:
        index = list(csv.DictReader(stream))
    records, pictures = [], {}
    for row in index:
        record_path = row.get('candidate_record_path') or f"results/{row['candidate_id']}/candidate.json"
        record = json.loads((root / record_path).read_text())
        record['record_path'] = record_path
        worker = row.get('worker_dataset_root') or ''
        observation = root / worker / record['observation_path']
        key = str(observation.parent.relative_to(root))
        record['picture_key'] = key
        if key not in pictures:
            pictures[key] = {}
            for name in ['rgb.png', 'local_rgb.png']:
                path = observation.parent / name
                if path.exists():
                    pictures[key][name] = 'data:image/png;base64,' + base64.b64encode(path.read_bytes()).decode()
        records.append(record)
    backend_path = root / 'backend.json'
    backend = json.loads(backend_path.read_text()) if backend_path.exists() else None
    payload = {'backend': backend, 'records': records, 'pictures': pictures,
               'summary': json.loads((root / 'summary.json').read_text()),
               'run': root.name, 'generated': datetime.datetime.now().astimezone().isoformat(timespec='seconds')}
    template = Path(__file__).with_suffix('.html').read_text()
    output = root / 'dashboard.html'
    output.write_text(template.replace('__DATA__', json.dumps(payload, ensure_ascii=False).replace('</', '<\\/')))
    print(f'{output}\n{len(records)} candidates; {output.stat().st_size / 1024 / 1024:.1f} MiB')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    build(parser.parse_args().run_dir.resolve())
