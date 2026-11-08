"""Preserve an interrupted native trace as explicitly incomplete evidence.

No simulator imports, retries, checkpoint publication, or normal-file writes.
"""
from collections.abc import Mapping
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path

import numpy as np


def _sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def _numeric(value,name):
    array=np.asarray(value)
    if array.dtype.kind not in 'fiu' or not np.isfinite(array).all():
        raise ValueError('Partial trace '+name+' must contain finite numeric values')
    return array


def _json_value(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,Path):return str(value)
    raise TypeError('Unsupported partial-trace JSON value: '+type(value).__name__)


def preserve_partial_trace(output,runtime_dict):
    """Save available aligned actual samples, never a finalized motion result.

    Accepts the native module's globals/locals mapping. Before ``times`` has a
    recorded sample it returnsNone without creating output. Once recording has
    started, malformed mandatory arrays fail closed rather than inventing data.
    """
    if not isinstance(runtime_dict,Mapping):raise TypeError('Native runtime mapping required')
    if 'times' not in runtime_dict or len(runtime_dict['times'])==0:return None
    required=('times','trace','qtrace','body_paths','phases_trace','cycle_trace')
    if any(name not in runtime_dict for name in required):
        raise ValueError('Started partial trace lacks mandatory measured arrays')
    times=_numeric(runtime_dict['times'],'times')
    if times.ndim!=1 or times[0]<0 or np.any(np.diff(times)<=0):
        raise ValueError('Partial trace timestamps must be nonnegative and strictly increasing')
    count=len(times)
    poses=_numeric(runtime_dict['trace'],'trace');q=_numeric(runtime_dict['qtrace'],'qtrace')
    paths=np.asarray(runtime_dict['body_paths']);phases=np.asarray(runtime_dict['phases_trace'])
    cycles=_numeric(runtime_dict['cycle_trace'],'cycle_trace')
    if (paths.ndim!=1 or not len(paths) or paths.dtype.kind not in 'US' or
            len(set(paths.tolist()))!=len(paths) or
            any(not path.startswith('/World/') or path.endswith('/') for path in paths.tolist())):
        raise ValueError('Partial trace requires unique actual native /World/ body paths')
    if poses.shape!=(count,len(paths),7):raise ValueError('Partial measured body poses are not time/body aligned')
    if q.ndim!=2 or q.shape[0]!=count or q.shape[1]<1:
        raise ValueError('Partial measured robot joints are not time aligned')
    if phases.shape!=(count,) or phases.dtype.kind not in 'US' or any(not str(value) for value in phases):
        raise ValueError('Partial phase labels are not time aligned')
    if cycles.shape!=(count,) or np.any(cycles<0) or np.any(cycles!=np.floor(cycles)):
        raise ValueError('Partial cycle indices must be aligned nonnegative integers')
    arrays=dict(time=times,poses=poses,robot_q=q,body_paths=paths,phase=phases,
                cycle_index=cycles.astype(np.int64))
    omitted={}
    optional={'command':'command','lift_desired_commands':'desired_command','batch_qtrace':'all_environment_robot_q',
              'belt_commands':'belt_surface_velocity_local_y_m_s','force_commands':'external_force_world_n'}
    for source,target in optional.items():
        if source not in runtime_dict:continue
        try:
            value=_numeric(runtime_dict[source],source)
            if value.ndim<1 or value.shape[0]!=count:
                raise ValueError('Optional array length differs from recorded actual samples')
            arrays[target]=value
        except ValueError as error:omitted[source]=str(error)
    output=Path(output)
    paths_to_write=[output/'states_partial.npz',output/'partial_trace.json']
    breaks=runtime_dict.get('breaks')
    break_text=None
    if breaks is not None:
        break_text=json.dumps(breaks,indent=2,default=_json_value,allow_nan=False)+'\n'
        paths_to_write.append(output/'break_events_partial.json')
    for path in paths_to_write:
        if path.exists():raise FileExistsError('Preserve existing partial evidence: '+str(path))
    argument=runtime_dict.get('a');assembly=getattr(argument,'assembly',runtime_dict.get('assembly'))
    manifest=runtime_dict.get('manifest') or {}
    assembly=Path(assembly).resolve() if assembly is not None else None
    source_scene=runtime_dict.get('scene')
    source_scene=Path(source_scene).resolve() if isinstance(source_scene,(str,Path)) else (
        assembly/'scene.usda' if assembly is not None else None)
    report=dict(schema='s300-interrupted-actual-trace-v1',complete=False,completed=False,
        training_eligible=False,harvest_success=None,checkpoint_eligible=False,
        source='Already recorded actual native samples before an unexpected exception; no physics rerun',
        initial_state_restored=False,finalized_native_audit_claimed=False,
        samples=count,first_timestamp_s=float(times[0]),last_timestamp_s=float(times[-1]),
        source_assembly=None if assembly is None else str(assembly),
        source_scene=None if source_scene is None else str(source_scene),
        declared_wrapper_scene_sha256=manifest.get('scene_sha256'),
        actual_source_scene_sha256=_sha(source_scene) if source_scene is not None and source_scene.is_file() else None,
        actual_manifest_sha256=_sha(assembly/'motion_manifest.json') if assembly is not None and (assembly/'motion_manifest.json').is_file() else None,
        normal_states_preserved=(output/'states.npz').is_file(),
        normal_summary_preserved=(output/'summary.json').is_file(),
        omitted_optional_arrays=omitted,array_shapes={key:list(value.shape)for key,value in arrays.items()},
        written_at_utc=datetime.now(timezone.utc).isoformat(),
        states='states_partial.npz',break_events='break_events_partial.json' if breaks is not None else None)
    output.mkdir(parents=True,exist_ok=True)
    with (output/'states_partial.npz').open('xb') as stream:np.savez_compressed(stream,**arrays)
    report['states_sha256']=_sha(output/'states_partial.npz')
    if break_text is not None:
        with (output/'break_events_partial.json').open('x') as stream:stream.write(break_text)
    with (output/'partial_trace.json').open('x') as stream:
        stream.write(json.dumps(report,indent=2,default=_json_value,allow_nan=False)+'\n')
    return report
