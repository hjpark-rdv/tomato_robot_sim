#!/usr/bin/env python3
"""Package only validated final V5 artifacts; verify ZIP contents and SHA-256."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil,zipfile
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT.parent/'output';OUT.mkdir(exist_ok=True)
read=lambda n:json.loads((ROOT/'checks'/f'{n}.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
inter=read('interference_full');fits=read('interfaces_and_tension');ex=read('export_checks')
paths=read('assembly_paths');audit=read('double_check');browser=read('browser_checks');fallback=read('browser_fallback_checks');pdf=read('pdf_checks')
manifest=json.loads((ROOT/'parts_manifest.json').read_text())
parameters=json.loads((ROOT/'parameters.json').read_text())
instances=len(manifest);printed=sum(p['kind']=='print' for p in manifest)
assert instances==356 and printed==66, 'Unexpected V5.1 inventory'
repair=read('repair_audit');explosion=read('explosion_audit')
assert repair['all_checks_pass'] and explosion['all_checks_pass']
assert repair['inventory']['instances']==instances and explosion['instances']==instances
assert repair['revision']==explosion['revision']==parameters['revision']
assert inter['all_sampled_positions_clear'] and not inter['invalid'] and not inter['multiple_solid_prints']
assert all(fits[k] for k in ['all_thread_stacks_pass','all_tension_positions_clear','all_timing_adjustment_positions_clear','all_tool_paths_clear'])
assert ex['all_print_meshes_pass'] and ex['all_step_reopened_valid']
assert paths['all_sampled_paths_clear'] and paths['all_service_tools_clear']
assert audit['all_assertions_pass'] and audit['all_rotations_clear']
assert sha(ROOT/'source/build_v5.py')==audit['source_sha256']
assert len(ex['stl'])==71 and len(ex['step'])==76 and ex['glb']['geometry_count']==instances
assert len(ex['stl'])==printed+parameters['fit_coupon_count']
for report in [browser,fallback]:
 assert not report['errors'] and not report['external_requests']
 assert report['cycle_changed'] and report['belt_rotation_changed'] and report['belt_mode_changed']
 assert report['idle_render_frames']<=1 and report['snapshot_download_pass']
 assert report['viewer_sha256']==sha(ROOT/'inaho_gripper_v5_viewer.html')
 assert all(s['parts']==instances for s in report['states'])
assert browser['render_path']=='WebGL' and fallback['render_path']=='Canvas'
assert pdf['text_bounds_pass'] and pdf['visual_review_pass'] and not pdf['missing_expected_text'] and pdf['missing_glyph_count']==0
assert pdf['pdf_sha256']==sha(ROOT/'docs/inaho_gripper_v5_assembly_guide_KO.pdf')
release=dict(revision=parameters['revision'],packaged_utc=datetime.now(timezone.utc).isoformat(),units_CAD_STL='mm',units_GLTF='m / Y-up',
 assembly_instances=instances,installed_printed_components=printed,fit_coupons=5,STL_count=71,STEP_count=76,
 body_height_difference_mm=0,belt_z_ranges_mm=[[9,61],[13,65]],belt_offset_is_design_value_not_original_measurement=True,
 gap_samples=len(inter['positions']),gap_range_mm=[28,96],interference_threshold_mm3=.01,
 exact_intersection_tests=inter['exact_boolean_tests'],rotation_samples=len(audit['rotation_samples']),
 nose_travel_samples_mm=[-3,0,3],motor_plate_travel_samples_mm=[-3,0,3],motor_travel_checked_with_belt_removed=True,
 thread_stacks_checked=len(fits['interfaces']),assembly_path_samples=sum(len(p['positions']) for p in paths['paths']),
 browser_render_paths_tested=['WebGL','Canvas'],browser_external_requests=0,browser_orthographic_views=['top','front','rear','side'],
 guide_pages=pdf['pages'],guide_visual_reviewed=True,
 physical_print_assembly_tested=False,native_SolidWorks_tested=False,FEA_fatigue_thermal_tested=False,
 belt_tracking_tension_life_tested=False,crop_grip_damage_tested=False,
 CAD_source_sha256=audit['source_sha256'],viewer_sha256=browser['viewer_sha256'],
 scope='Original-form inspired mechanical prototype. Purchased hardware required. Original production dimensions unavailable. Host robot interface/electronics excluded.')
(ROOT/'checks/RELEASE_SUMMARY.json').write_text(json.dumps(release,ensure_ascii=False,indent=2))
check_names={f'{s}.json' for s in ['interference_full','interfaces_and_tension','assembly_paths','export_checks','double_check','browser_checks','browser_fallback_checks','pdf_checks','RELEASE_SUMMARY','repair_audit','explosion_audit']}
allowed_root={'README_KO.txt','inaho_gripper_v5_viewer.html','parameters.json','parts_manifest.json'}
def include(p):
 r=p.relative_to(ROOT)
 if '__pycache__' in r.parts or p.suffix in ['.pyc','.log']:return False
 if len(r.parts)==1:return p.name in allowed_root
 if r.parts[0]=='checks':return p.name in check_names and len(r.parts)==2
 if r.parts[0]=='docs':return p.name not in ['omronD2F.pdf','robotdigg20.png']
 if r.parts[0]=='previews':return p.suffix=='.png' and p.name[:2].isdigit()
 return r.parts[0] in ['source','vendor','models','stl','step_parts']
files=sorted(p for p in ROOT.rglob('*') if p.is_file() and include(p))
hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
(ROOT/'FILE_SHA256.json').write_text(json.dumps(hashes,indent=2))
files.append(ROOT/'FILE_SHA256.json')
full=OUT/'inaho_gripper_v5_printable.zip'
with zipfile.ZipFile(full,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in files:z.write(p,f'{ROOT.name}/{p.relative_to(ROOT).as_posix()}')
print_files=sorted((ROOT/'stl').glob('*.stl'))+[ROOT/'README_KO.txt',ROOT/'docs/PRINT_LIST.csv',ROOT/'docs/PURCHASE_BOM.csv',ROOT/'docs/AUXILIARY_BOM.txt',ROOT/'docs/JOINT_STACKS.csv',ROOT/'docs/inaho_gripper_v5_assembly_guide_KO.pdf']
stlzip=OUT/'inaho_gripper_v5_STL_print_pack.zip'
with zipfile.ZipFile(stlzip,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in print_files:z.write(p,f'inaho_gripper_v5_STL_print_pack/{p.relative_to(ROOT).as_posix()}')
for rel in ['models/inaho_gripper_v5_assembly.step','models/inaho_gripper_v5_assembly.glb','inaho_gripper_v5_viewer.html','docs/inaho_gripper_v5_assembly_guide_KO.pdf']:
 shutil.copyfile(ROOT/rel,OUT/Path(rel).name)
shutil.copyfile(ROOT/'previews/00_model.png',OUT/'inaho_gripper_v5_preview.png')
checks=[]
for p in [full,stlzip]:
 with zipfile.ZipFile(p) as z:
  assert z.testzip() is None
  names=z.namelist();assert len(names)==len(set(names))
  assert len([n for n in names if n.endswith('.stl')])==71
  if p==full:
   assert len([n for n in names if n.endswith('.step')])==76
   stored=json.loads(z.read('inaho_gripper_v5/FILE_SHA256.json'))
   assert set(stored)=={n.removeprefix('inaho_gripper_v5/') for n in names if not n.endswith('/FILE_SHA256.json')}
   for n,h in stored.items():assert hashlib.sha256(z.read('inaho_gripper_v5/'+n)).hexdigest()==h,n
  checks.append(dict(file=p.name,bytes=p.stat().st_size,entries=len(names),crc_pass=True,stl_count=71,sha256=sha(p)))
result={'archives':checks,'full_archive_hash_manifest_verified':True,'all_required_deliverables_present':True}
(OUT/'inaho_gripper_v5_delivery_checks.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
