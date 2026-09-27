"""Archive complete bounded experiment evidence without large simulator assets."""
from pathlib import Path
import hashlib,json,shutil,tarfile
root=Path(__file__).resolve().parent
out=Path('/root/farmily_tomato/mujoco-benchmark/validation/multifamily_server')
out.mkdir(parents=True,exist_ok=True)
# Campaign/derived tables remain directly readable by reviewers.
for p in root.glob('*'):
 if p.is_file() and p.suffix in ('.json','.py','.log','.md'):
  shutil.copy2(p,out/p.name)
for name in ('smoke_v2','pilot','side_repair'):
 for fn in ('campaign.json','results.json','results.csv','video_queue.json'):
  p=root/name/fn
  if p.exists():
   dest=out/name/fn;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
for cohort in ('smoke_prep','pilot_prep'):
 prep=root/'assets'/cohort
 for p in [prep/'cases.json',*sorted((prep/'scenes').glob('scene_*/scene.json'))]:
  if p.exists():
   dest=out/'inputs'/cohort/p.relative_to(prep);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
files=[]
# Includes every candidate, not only video-selected ones. No symlink traversal to model assets.
for cohort in ('smoke','smoke_v2','pilot','side_repair'):
 for p in (root/cohort).rglob('*'):
  if not p.is_file() or p.is_symlink():continue
  if 'replay_assets' in p.parts:continue
  if p.suffix not in ('.json','.gz','.npz','.log','.py','.md','.csv'):continue
  if p.stat().st_size>25*1024*1024:raise RuntimeError(f'Unexpected large evidence file {p}')
  files.append(p)
for cohort in ('executor_control','rachis_contact_control','azimuth_baseline'):
 files += [p for p in (root/cohort).rglob('*') if p.is_file() and not p.is_symlink() and p.suffix in ('.json','.gz','.npz','.log')]
# Small original control commands are required to understand/review the saved qpos.
source=Path('/root/docker_share/mujoko_debugging_data/20260927_hook_seating_server/robot_under')
for cid in ('seating_00_under','seating_03_under'):
 for fn in ('plan.json','trace.json'):
  dest=root/'control_inputs'/cid/fn;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/'candidates'/cid/fn,dest);files.append(dest)
with tarfile.open(out/'all_candidate_evidence.tar.gz','w:gz',compresslevel=6) as archive:
 for p in sorted(set(files)):archive.add(p,arcname=str(p.relative_to(root)),recursive=False)
# Videos and metadata are directly browsable in Git; nominal state arrays kept in tar.
video_paths=[]
for directory in ('smoke_videos','pilot_supplement_videos'):
 video_paths += list((root/directory).glob('*.mp4'))
for cohort in ('pilot','side_repair'):
 video_paths += list((root/cohort).rglob('*.mp4'))
for video in sorted(set(video_paths)):
 relative=video.relative_to(root);dest=out/'videos'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(video,dest)
 for sidecar in [video.with_suffix('.json'),*video.parent.glob(video.stem+'_frame*.png')]:
  if sidecar.exists():shutil.copy2(sidecar,dest.parent/sidecar.name)
index=[]
for p in sorted(out.rglob('*')):
 if p.is_file() and p.name!='SHA256.json':index.append(dict(path=str(p.relative_to(out)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
(out/'SHA256.json').write_text(json.dumps(index,indent=2)+'\n')
print('Archived',len(files),'raw files;',len(video_paths),'videos;',sum(r['bytes'] for r in index)/1024**2,'MiB')
