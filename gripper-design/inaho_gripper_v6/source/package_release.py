#!/usr/bin/env python3
"""Archive V6 sources and exports with exact filenames and content hashes."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]
check = json.loads((ROOT/'checks/v6_receiver.json').read_text())
assert check['passed'] and check['assembly_step']['solids'] == 356
assert (ROOT/'docs/inaho_gripper_v6_assembly_guide_KO.pdf').is_file()
assert len(list((ROOT/'stl').glob('*.stl'))) == 71
files = sorted(p for p in ROOT.rglob('*') if p.is_file()
    and p.suffix not in {'.pyc', '.zip', '.tmp', '.log'} and '__pycache__' not in p.parts
    and p.name not in {'FILE_SHA256.json', 'package_checks.json'})
hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
(ROOT/'FILE_SHA256.json').write_text(json.dumps(hashes, indent=2) + '\n')
files.append(ROOT/'FILE_SHA256.json')
archive = ROOT/'inaho_gripper_v6_printable.zip'
temporary = archive.with_suffix('.tmp')
with zipfile.ZipFile(temporary, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
    for p in files:
        z.write(p, ROOT.name+'/'+str(p.relative_to(ROOT)))
with zipfile.ZipFile(temporary) as z:
    assert z.testzip() is None
    for name, digest in hashes.items():
        assert hashlib.sha256(z.read(ROOT.name+'/'+name)).hexdigest() == digest
    assert len([name for name in z.namelist() if name.endswith('.stl')]) == 71
    assert not any('L_T01_sloped_receiver.stl' in name for name in z.namelist())
temporary.replace(archive)
result = dict(passed=True, entries=len(files), stl_files=71,
    archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(), bytes=archive.stat().st_size)
(ROOT/'checks/package_checks.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
