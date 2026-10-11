"""Write-once measured native state for an explicitly documented route handoff.

This module does not step physics, author USD or infer velocities. It preserves
native articulation order, including the robot's jaws. A checkpoint contains
measured state, not PhysX solver/contact warm-start caches or a bitwise engine
restart. Consumers must document any dynamics approximation when switching the
active plant window. Optional native drive targets are separate controller
state; measured q/qvel never substitute for them.
"""
import hashlib
import json
import os
from pathlib import Path
import re

import numpy as np


SCHEMA = 'native_route_checkpoint_v1'
ARRAY_KEYS = frozenset(('body_paths', 'body_poses_world', 'body_velocities_world',
                        'robot_dof_names', 'robot_q', 'robot_qvel',
                        'plant_dof_names', 'plant_q', 'plant_qvel', 'preload'))
CONTROLLER_SCHEMA = 'native_robot_drive_targets_v1'
CONTROLLER_UNITS = dict(position='native_dof_si_rad_or_m',velocity='native_dof_si_rad_s_or_m_s')


def _sha(path):
    result=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):result.update(chunk)
    return result.hexdigest()


def _json_bytes(value):
    return json.dumps(value,ensure_ascii=False,sort_keys=True,allow_nan=False,
                      separators=(',',':')).encode('utf-8')


def _json_copy(value):
    # The manifest and sequence are provenance, not mutable references to caller
    # state. Reject nonfinite JSON values rather than silently coercing them.
    return json.loads(_json_bytes(value))


def _names(value,label,*,paths=False):
    array=np.asarray(value)
    if array.ndim!=1 or not len(array) or array.dtype.kind not in 'US':
        raise ValueError(label+' must be a nonempty one-dimensional string mapping')
    result=array.astype(str)
    if any(not name or name.strip()!=name for name in result) or len(set(result.tolist()))!=len(result):
        raise ValueError(label+' contains empty, padded or duplicate identities')
    if paths and any(not name.startswith('/World/') or '//' in name or name.endswith('/') for name in result):
        raise ValueError(label+' requires exact absolute native /World/ body paths')
    return result.copy()


def _numeric(value,shape,label):
    array=np.asarray(value)
    if array.shape!=shape or array.dtype.kind not in 'fi' or not np.isfinite(array).all():
        raise ValueError(label+' requires finite measured numeric values with shape '+str(shape))
    return np.array(array,dtype=np.float64,copy=True)


def _declared_joint_paths(value):
    result=set()
    if isinstance(value,dict):
        for key,item in value.items():
            if key=='joint' and isinstance(item,str) and item.startswith('/World/'):
                result.add(item)
            result.update(_declared_joint_paths(item))
    elif isinstance(value,list):
        for item in value:result.update(_declared_joint_paths(item))
    return result


def _controller_numeric(value,shape,label):
    if any(isinstance(item,(bool,np.bool_)) for item in np.asarray(value,dtype=object).flat):
        raise ValueError(label+' cannot contain boolean targets')
    return _numeric(value,shape,label)


def _validate_controller_state(state,robot_names):
    if not isinstance(state,dict) or state.get('schema')!=CONTROLLER_SCHEMA:
        raise ValueError('Unsupported native robot controller state schema')
    expected={'schema','robot_dof_names','position_targets','velocity_targets',
              'position_source','velocity_source','units'}
    if set(state)!=expected:raise ValueError('Native controller state descriptor schema mismatch')
    names=_names(state['robot_dof_names'],'controller_state robot_dof_names')
    if names.tolist()!=robot_names.tolist():raise ValueError('Controller native DOF names/order mismatch')
    _controller_numeric(state['position_targets'],(len(robot_names),),'controller native position_targets')
    _controller_numeric(state['velocity_targets'],(len(robot_names),),'controller native velocity_targets')
    if (state['position_source']!='native_get_dof_position_targets' or
            state['velocity_source']!='native_get_dof_velocity_targets'):
        raise ValueError('Controller targets require exact native getter provenance, not measured state')
    if state['units']!=CONTROLLER_UNITS:raise ValueError('Controller native SI unit schema mismatch')


def validate_checkpoint(metadata,arrays,*,expected_scene_sha256=None,check_source_scene=True):
    """Fail closed on schema/hash/identity/finite-state mismatches; no mutation."""
    if not isinstance(metadata,dict) or metadata.get('schema')!=SCHEMA:
        raise ValueError('Unsupported native route checkpoint schema')
    digest=metadata.get('metadata_sha256')
    if digest is not None:
        unsigned={key:value for key,value in metadata.items() if key!='metadata_sha256'}
        if hashlib.sha256(_json_bytes(unsigned)).hexdigest()!=digest:
            raise ValueError('Checkpoint metadata SHA mismatch')
    if set(arrays)!=ARRAY_KEYS:raise ValueError('Checkpoint array schema mismatch')
    scene_sha=metadata.get('scene_sha256')
    if not isinstance(scene_sha,str) or re.fullmatch('[0-9a-f]{64}',scene_sha) is None:
        raise ValueError('Exact scene SHA256 required')
    if expected_scene_sha256 is not None and scene_sha!=expected_scene_sha256:
        raise ValueError('Checkpoint expected scene SHA mismatch')
    assembly=Path(metadata.get('source_assembly',''))
    if not assembly.is_absolute() or metadata.get('source_scene')!=str(assembly/'scene.usda'):
        raise ValueError('Checkpoint source assembly/scene mapping mismatch')
    if check_source_scene and _sha(assembly/'scene.usda')!=scene_sha:
        raise ValueError('Checkpoint source scene SHA mismatch')
    time_s=metadata.get('time_s')
    if isinstance(time_s,bool) or not isinstance(time_s,(int,float)) or not np.isfinite(time_s) or time_s<0:
        raise ValueError('Finite nonnegative checkpoint timestamp required')
    if metadata.get('quaternion_order')!='xyzw' or metadata.get('body_velocity_order')!=['linear_x','linear_y','linear_z','angular_x','angular_y','angular_z']:
        raise ValueError('Checkpoint native pose/velocity convention mismatch')
    body_paths=_names(arrays['body_paths'],'body_paths',paths=True)
    robot_names=_names(arrays['robot_dof_names'],'robot_dof_names')
    plant_names=_names(arrays['plant_dof_names'],'plant_dof_names')
    if 'controller_state' in metadata:_validate_controller_state(metadata['controller_state'],robot_names)
    if not {'jaw_L','jaw_R'}<=set(robot_names):
        raise ValueError('Robot native articulation mapping must include both jaw DOFs')
    poses=_numeric(arrays['body_poses_world'],(len(body_paths),7),'body_poses_world')
    quaternion_norm=np.linalg.norm(poses[:,3:],axis=1)
    if not np.allclose(quaternion_norm,1.,rtol=0.,atol=1e-4):
        raise ValueError('Measured native body quaternion is not unit XYZW')
    _numeric(arrays['body_velocities_world'],(len(body_paths),6),'body_velocities_world')
    for key,names in (('robot_q',robot_names),('robot_qvel',robot_names),
                      ('plant_q',plant_names),('plant_qvel',plant_names),('preload',plant_names)):
        _numeric(arrays[key],(len(names),),key)
    mappings=metadata.get('mappings',{})
    for key,values in (('body_paths',body_paths),('robot_dof_names',robot_names),('plant_dof_names',plant_names)):
        if mappings.get(key)!=values.tolist():raise ValueError('Checkpoint '+key+' mapping/order mismatch')
    specs=metadata.get('arrays',{})
    if set(specs)!=ARRAY_KEYS:raise ValueError('Checkpoint metadata array descriptors mismatch')
    for key,array in arrays.items():
        if specs[key]!=dict(shape=list(array.shape),dtype=str(array.dtype)):
            raise ValueError('Checkpoint '+key+' array shape/dtype descriptor mismatch')
    manifest=metadata.get('manifest')
    if not isinstance(manifest,dict) or hashlib.sha256(_json_bytes(manifest)).hexdigest()!=metadata.get('manifest_sha256'):
        raise ValueError('Checkpoint manifest SHA mismatch')
    manifest_scene=manifest.get('scene_sha256')
    if manifest_scene is not None and manifest_scene!=scene_sha:
        raise ValueError('Checkpoint manifest/scene SHA mismatch')
    compact=manifest.get('compact_assets')
    if compact:
        dependency=compact.get('materialized_dependency',{})
        relative=Path(dependency.get('relative_path',''))
        if relative.is_absolute() or '..' in relative.parts or not relative.parts:
            raise ValueError('Checkpoint compact dependency path is not inside source assembly')
        if check_source_scene and _sha(assembly/relative)!=dependency.get('sha256'):
            raise ValueError('Checkpoint source compact dependency SHA mismatch')
    plant=manifest.get('plant',{})
    root=plant.get('root')
    source_plant=metadata.get('source_plant')
    if (not isinstance(root,str) or not root.startswith('/World/') or
            not isinstance(source_plant,str) or not source_plant.startswith('/World/') or
            source_plant!=plant.get('source_plant')):
        raise ValueError('Exact source/active plant identity mapping required')
    expected_bodies={body.get('path') for body in plant.get('bodies',[])}
    expected_fruits={fruit.get('path') for fruit in plant.get('fruits',[])}
    expected=expected_bodies|expected_fruits
    if not expected or any(not isinstance(path,str) or not path.startswith(root+'/') for path in expected):
        raise ValueError('Checkpoint declared active plant body mapping is malformed')
    if not expected<=set(body_paths):raise ValueError('Checkpoint missing declared active plant/fruit body')
    target=manifest.get('target',{}).get('path')
    if target is not None and target not in expected_fruits:
        raise ValueError('Checkpoint target fruit mapping mismatch')
    declared_joints=_declared_joint_paths(manifest)
    breaks=metadata.get('break_events')
    if not isinstance(breaks,list):raise ValueError('Checkpoint break event list required')
    for event in breaks:
        if not isinstance(event,dict) or event.get('joint') not in declared_joints:
            raise ValueError('Checkpoint break joint is not in exact manifest inventory')
        timestamp=event.get('t')
        if (isinstance(timestamp,bool) or not isinstance(timestamp,(int,float)) or
                not np.isfinite(timestamp) or not 0<=timestamp<=time_s+1e-9):
            raise ValueError('Checkpoint break timestamp is invalid or later than capture')
    # Strict serialization also traverses sequence/break extras for NaN/Inf.
    _json_bytes(metadata)
    if metadata.get('training_eligible') is not False:
        raise ValueError('Native route checkpoint is diagnostic and cannot be training eligible')
    return metadata


def save_checkpoint(output,*,assembly,scene_sha256,time_s,body_paths,
                    body_poses_world,body_velocities_world,robot_dof_names,
                    robot_q,robot_qvel,plant_dof_names,plant_q,plant_qvel,
                    preload,breaks,manifest,sequence=None,source_plant=None,
                    robot_position_targets=None,robot_velocity_targets=None):
    """Save one measured single-environment state; refuse every overwrite.

    Required velocities must come from native body/articulation getters. Arrays
    are copied exactly in the caller's mapping order; no normalization, sorting,
    zero-fill, or detached-fruit reset is performed.
    Optional drive targets must both come from the named native getters and
    retain the same native DOF order; neither is inferred from measured q/qvel.
    """
    if isinstance(time_s,(bool,np.bool_)):raise ValueError('Checkpoint timestamp cannot be boolean')
    assembly=Path(assembly).resolve()
    names=dict(body_paths=_names(body_paths,'body_paths',paths=True),
               robot_dof_names=_names(robot_dof_names,'robot_dof_names'),
               plant_dof_names=_names(plant_dof_names,'plant_dof_names'))
    arrays=dict(names)
    arrays['body_poses_world']=_numeric(body_poses_world,(len(names['body_paths']),7),'body_poses_world')
    arrays['body_velocities_world']=_numeric(body_velocities_world,(len(names['body_paths']),6),'body_velocities_world')
    for key,value,name_key in (('robot_q',robot_q,'robot_dof_names'),('robot_qvel',robot_qvel,'robot_dof_names'),
                               ('plant_q',plant_q,'plant_dof_names'),('plant_qvel',plant_qvel,'plant_dof_names'),
                               ('preload',preload,'plant_dof_names')):
        arrays[key]=_numeric(value,(len(names[name_key]),),key)
    snapshot=_json_copy(manifest)
    metadata=dict(schema=SCHEMA,source_assembly=str(assembly),source_scene=str(assembly/'scene.usda'),
                  scene_sha256=scene_sha256,time_s=float(time_s),source_plant=source_plant if source_plant is not None else snapshot.get('plant',{}).get('source_plant'),
                  quaternion_order='xyzw',position_units='m',
                  body_velocity_order=['linear_x','linear_y','linear_z','angular_x','angular_y','angular_z'],
                  linear_velocity_units='m/s',angular_velocity_units='rad/s',
                  mappings={key:value.tolist() for key,value in names.items()},
                  arrays={key:dict(shape=list(value.shape),dtype=str(value.dtype)) for key,value in arrays.items()},
                  manifest=snapshot,manifest_sha256=hashlib.sha256(_json_bytes(snapshot)).hexdigest(),
                  break_events=_json_copy(breaks),sequence=_json_copy(sequence),
                  training_eligible=False,physics_mutated=False,velocities_inferred=False,
                  capture_scope='Measured native rigid/articulation state and fixed preload; no solver/contact warm-start cache snapshot',
                  robot_articulation_scope='One native articulation including jaws; no separate assumed tool articulation')
    if (robot_position_targets is None)!=(robot_velocity_targets is None):
        raise ValueError('Both native position and velocity targets are required together')
    if robot_position_targets is not None:
        metadata['controller_state']=dict(schema=CONTROLLER_SCHEMA,
            robot_dof_names=names['robot_dof_names'].tolist(),
            position_targets=_controller_numeric(robot_position_targets,(len(names['robot_dof_names']),),'robot_position_targets').tolist(),
            velocity_targets=_controller_numeric(robot_velocity_targets,(len(names['robot_dof_names']),),'robot_velocity_targets').tolist(),
            position_source='native_get_dof_position_targets',velocity_source='native_get_dof_velocity_targets',
            units=dict(CONTROLLER_UNITS))
    validate_checkpoint(metadata,arrays)
    output=Path(output)
    # Exclusive directory creation reserves the checkpoint. Existing outputs,
    # including incomplete captures, require a new path and are never replaced.
    output.mkdir(parents=True,exist_ok=False)
    data_path=output/'checkpoint.npz'
    with data_path.open('xb') as stream:
        np.savez_compressed(stream,**arrays);stream.flush();os.fsync(stream.fileno())
    metadata['checkpoint_npz_sha256']=_sha(data_path)
    metadata['metadata_sha256']=hashlib.sha256(_json_bytes(metadata)).hexdigest()
    with (output/'checkpoint.json').open('x',encoding='utf-8') as stream:
        json.dump(metadata,stream,ensure_ascii=False,sort_keys=True,allow_nan=False,indent=2)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    data_path.chmod(0o444);(output/'checkpoint.json').chmod(0o444)
    return metadata


def load_checkpoint(folder,*,expected_scene_sha256=None,check_source_scene=True):
    """Return verified ``(metadata, arrays)``; require explicit source provenance."""
    folder=Path(folder)
    metadata=json.loads((folder/'checkpoint.json').read_text(encoding='utf-8'))
    data_path=folder/'checkpoint.npz'
    if _sha(data_path)!=metadata.get('checkpoint_npz_sha256'):
        raise ValueError('Checkpoint NPZ SHA mismatch')
    if 'metadata_sha256' not in metadata:raise ValueError('Checkpoint metadata SHA missing')
    with np.load(data_path,allow_pickle=False) as data:
        arrays={key:np.array(data[key],copy=True) for key in data.files}
    validate_checkpoint(metadata,arrays,expected_scene_sha256=expected_scene_sha256,
                        check_source_scene=check_source_scene)
    return metadata,arrays
