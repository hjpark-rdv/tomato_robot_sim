"""Measured robot/released-fruit restore across an explicit active-plant window.

The newly built active plant retains its own initialized q/qvel and preload.
Only the old robot state and exactly preserved detached fruit are restored.
Native solver/contact caches restart; this is not a bit-continuous engine.
"""
import copy
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from compact_motion_assets import file_sha256, verify_compact_dependencies
from native_route_checkpoint import load_checkpoint
from resume_native_state import _array, require_completed_checkpoint, verify_resume


ROBOT = '/World/HarvestRobot/Robot'
TOOL = '/World/ActuatedTool'


def prepare_route_start(stage, assembly, manifest):
    """Verify exact measured parent provenance and remapped native body IDs."""
    assembly = Path(assembly).resolve()
    if file_sha256(assembly/'scene.usda') != manifest.get('scene_sha256'):
        raise ValueError('Route-window source model SHA mismatch')
    verify_compact_dependencies(assembly, manifest)
    route = manifest.get('route_checkpoint')
    if not route or route.get('schema') != 's300-active-window-checkpoint-v1':
        raise ValueError('Explicit native active-window checkpoint mapping required')
    checkpoint = Path(route['checkpoint'])
    if not checkpoint.is_absolute(): raise ValueError('Exact absolute route checkpoint path required')
    metadata, arrays = load_checkpoint(checkpoint, expected_scene_sha256=route['source_scene_sha256'])
    from robot_controller_state import resolve_robot_controller_state,rebase_rail_controller_state
    controller_state=resolve_robot_controller_state(metadata,arrays,checkpoint)
    if controller_state['position_targets'] is None:
        raise ValueError('Route continuation requires proven original controller position targets')
    require_completed_checkpoint(metadata)
    outgoing = route['outgoing']
    if (metadata['metadata_sha256'] != outgoing['checkpoint_metadata_sha256'] or
            metadata['source_plant'] != outgoing['source_plant'] or
            metadata['time_s'] != outgoing['checkpoint_time_s'] or
            route['checkpoint_time_s'] != metadata['time_s']):
        raise ValueError('Route-window native parent state proof mismatch')
    if manifest['plant']['source_plant'] == metadata['source_plant']:
        raise ValueError('Route window may not rebuild/reset its previous active plant')
    source = {str(path): i for i, path in enumerate(arrays['body_paths'])}
    mapping = {path: path for path in source if path.startswith(ROBOT+'/') or path.startswith(TOOL+'/')}
    current_robot_bodies=set()
    for root in (ROBOT,TOOL):
        prim=stage.GetPrimAtPath(root)
        if not prim:raise ValueError('Current route robot/tool root is missing')
        current_robot_bodies.update(str(p.GetPath()) for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI))
    if set(mapping)!=current_robot_bodies or ROBOT+'/world' not in mapping:
        raise ValueError('Current robot/tool body inventory differs from complete measured parent state')
    expected_standalone = set()
    carrier = manifest.get('rail', {}).get('carrier_body')
    if not carrier or carrier != metadata['manifest'].get('rail', {}).get('carrier_body'):
        raise ValueError('Exact measured physical S300 carrier identity required')
    expected_standalone.add(carrier)
    from background_foliage import declared_bodies
    current_background=declared_bodies(stage,manifest)
    source_background={row['path']:row for row in metadata['manifest'].get('background_compliant_foliage',{}).get('bodies',[])}
    background_keys=('path','joint','source_stem','source_group','meshes','mass','inertia','stiffness','damping')
    for path,row in source_background.items():
        if path not in current_background or any(current_background[path].get(key)!=row.get(key) for key in background_keys):
            raise ValueError('Measured passive background leaf identity/physical hypothesis changed')
        if path not in source:raise ValueError('Route parent checkpoint omitted a measured passive background leaf')
        mapping[path]=path;expected_standalone.add(path)
    source_active = metadata['manifest']['plant']['root']
    for old in route['outgoing_history']:
        for fruit in old.get('detached_fruits', []):
            path = fruit['path']
            if not path.startswith(old['frozen_root']+'/') or path.startswith(manifest['plant']['root']+'/'):
                raise ValueError('Route-window released fruit aliases the new active plant')
            prior_path = source_active+path[len(old['frozen_root']):] if old['source_plant'] == metadata['source_plant'] else path
            if prior_path not in source:
                raise ValueError('Route-window preserved fruit has no current native measured state')
            if path in mapping and mapping[path] != prior_path:
                raise ValueError('Ambiguous preserved fruit state identity')
            mapping[path] = prior_path; expected_standalone.add(path)
    for path, prior_path in mapping.items():
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError('Preserved native body is missing from current window: '+path)
        if prior_path not in source:
            raise ValueError('Measured preserved body path is absent')
    if carrier not in mapping: raise ValueError('Current native checkpoint omitted the physical S300 carrier')
    names = list(map(str, arrays['robot_dof_names']))
    rail = manifest['rail']; name = rail['dof_name']
    if name not in names: raise ValueError('Route parent checkpoint lacks the physical rail DOF')
    rebase = rail.get('checkpoint_rebase', {})
    if rebase.get('world_station_preserved') is not True or rail.get('initial_displacement_m') != 0.:
        raise ValueError('Documented zero-relative rail rebase required')
    if abs(rebase['previous_native_q_m']-arrays['robot_q'][names.index(name)]) > 1e-9:
        raise ValueError('Rail rebase does not match measured source q')
    robot_q = arrays['robot_q'].copy(); robot_q[names.index(name)] = 0.
    controller_state=rebase_rail_controller_state(controller_state,arrays,name)
    cache=UsdGeom.XformCache();new_rest={}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(manifest['plant']['root'])):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):continue
        world=np.asarray(cache.GetLocalToWorldTransform(prim)).T
        new_rest[str(prim.GetPath())]=np.r_[world[:3,3],Rotation.from_matrix(world[:3,:3]).as_quat()].tolist()
    return dict(metadata=metadata, source_arrays=arrays, body_mapping=mapping,
        new_active_authored_rest=new_rest,
        controller_state=controller_state,
        standalone_paths=sorted(expected_standalone), robot_q=robot_q,
        robot_qvel=arrays['robot_qvel'].copy(), robot_dof_names=names,
        report=dict(schema='s300-native-active-window-start-v1', checkpoint=str(checkpoint),
            source_scene_sha256=metadata['scene_sha256'], current_scene_sha256=manifest['scene_sha256'],
            source_materialized_dependency=metadata['manifest'].get('compact_assets',{}).get('materialized_dependency'),
            current_materialized_dependency=manifest.get('compact_assets',{}).get('materialized_dependency'),
            hash_scope='Wrapper SHA can match across derived assets; separately validated materialized USDC dependency SHA identifies actual physical content',
            checkpoint_metadata_sha256=metadata['metadata_sha256'],
            controller_state=controller_state['provenance'],
            measured_parent_body_mapping=mapping,
            restored_standalone_paths=sorted(expected_standalone),
            restored_passive_background_leaf_paths=sorted(source_background),
            new_passive_background_leaf_paths=sorted(set(current_background)-set(source_background)),
            rail_rebase=dict(name=name, source_q_m=float(arrays['robot_q'][names.index(name)]),
                            current_relative_q_m=0., measured_world_station_unchanged=True),
            new_active_plant_reset=False, old_attached_frozen_plant_modified=False,
            native_initialization_validation='pending', bitwise_engine_continuity=False,
            training_eligible=False, harvest_success=None))


def apply_route_start(context, robot, plant, bodies, *, to_native=lambda a: a,
                      indices_to_native=lambda a: a, environment_indices=None,
                      refresh_articulations=None):
    """Restore old robot and standalone state after warmup, without a physics step."""
    if robot.count != 1 or plant.count != 1:
        raise ValueError('Measured route-window continuation requires one environment')
    if list(map(str, robot.shared_metatype.dof_names)) != context['robot_dof_names']:
        raise ValueError('Route-window native robot DOF order/identity mismatch')
    paths = list(map(str, bodies.prim_paths)); index = {p: i for i, p in enumerate(paths)}
    if len(index) != len(paths) or not set(context['body_mapping']) <= set(index):
        raise ValueError('Route-window native preserved body coverage mismatch')
    source_arrays = context['source_arrays']; source = {str(p): i for i, p in enumerate(source_arrays['body_paths'])}
    plant_q = _array(plant.get_dof_positions()).copy(); plant_qvel = _array(plant.get_dof_velocities()).copy()
    initial_poses = _array(bodies.get_transforms()).copy(); initial_velocities = _array(bodies.get_velocities()).copy()
    expected_poses = initial_poses.copy(); expected_velocities = initial_velocities.copy()
    for path, prior in context['body_mapping'].items():
        expected_poses[index[path]] = source_arrays['body_poses_world'][source[prior]]
        expected_velocities[index[path]] = source_arrays['body_velocities_world'][source[prior]]
    ids = indices_to_native(np.array([0], np.uint32)) if environment_indices is None else environment_indices
    robot.set_dof_positions(to_native(context['robot_q'][None].copy()), ids)
    robot.set_dof_velocities(to_native(context['robot_qvel'][None].copy()), ids)
    from robot_controller_state import restore_controller_targets
    controller_report=restore_controller_targets(robot,context['controller_state'],ids,
        to_native=to_native,fallback_position=context['robot_q'])
    if refresh_articulations is not None: refresh_articulations()
    poses = _array(bodies.get_transforms()).copy(); velocities = _array(bodies.get_velocities()).copy()
    for path in context['standalone_paths']:
        poses[index[path]] = expected_poses[index[path]]
        velocities[index[path]] = expected_velocities[index[path]]
    selected_ids = indices_to_native(np.array([index[p] for p in context['standalone_paths']], np.uint32))
    bodies.set_transforms(to_native(poses), selected_ids)
    bodies.set_velocities(to_native(velocities), selected_ids)
    # Reuse the exact state-comparison implementation. New active plant values
    # are their OWN initialized baseline, not old active-plant aliases.
    comparison_arrays = dict(body_paths=np.array(paths), body_poses_world=expected_poses,
        body_velocities_world=expected_velocities, robot_dof_names=np.array(context['robot_dof_names']),
        robot_q=context['robot_q'], robot_qvel=context['robot_qvel'],
        plant_dof_names=np.array(list(map(str, plant.shared_metatype.dof_names))),
        plant_q=plant_q[0], plant_qvel=plant_qvel[0])
    comparison = verify_resume(dict(arrays=comparison_arrays), robot, plant, bodies)
    report = copy.deepcopy(context['report'])
    report['restored_controller_state']=controller_report
    preserved = [row for row in comparison['body_errors'] if row['path'] in context['body_mapping']]
    # Native quaternions are float32. Compare normalized rotations instead of
    # letting their harmless norm rounding appear as an acos angular error.
    measured_poses = _array(bodies.get_transforms())
    for row in preserved:
        native_quaternion=measured_poses[index[row['path']],3:].astype(np.float64)
        expected_quaternion=expected_poses[index[row['path']],3:].astype(np.float64)
        row['orientation_error_rad']=float((Rotation.from_quat(expected_quaternion).inv()*
            Rotation.from_quat(native_quaternion)).magnitude())
    report['preserved_body_errors'] = preserved
    for key in ('position_error_m', 'orientation_error_rad', 'linear_velocity_error_m_s', 'angular_velocity_error_rad_s'):
        worst = max(preserved, key=lambda row: row[key])
        report['max_'+key] = worst[key]; report['worst_'+key+'_body'] = worst['path']
    standalone=[row for row in preserved if row['path'] in context['standalone_paths']]
    articulated=[row for row in preserved if row['path'] not in context['standalone_paths']]
    report['standalone_body_errors']=standalone;report['articulated_body_errors']=articulated
    for group,rows in (('standalone',standalone),('articulated',articulated)):
        for key in ('linear_velocity_error_m_s','angular_velocity_error_rad_s'):
            report[group+'_max_'+key]=max((row[key] for row in rows),default=0.)
    for key in ('robot_q_max_abs_error', 'robot_qvel_max_abs_error', 'plant_q_max_abs_error', 'plant_qvel_max_abs_error'):
        report[key] = comparison[key]
    if comparison['plant_q_max_abs_error'] != 0. or comparison['plant_qvel_max_abs_error'] != 0.:
        raise ValueError('Route-window restore changed new active-plant native state')
    report.update(new_active_plant_native_state_set=False, new_active_plant_preload_set=False,
        articulated_link_transforms_set_directly=False,
        native_articulation_kinematics_refreshed=refresh_articulations is not None,
        body_count=len(paths), preserved_body_count=len(preserved),
        verification_scope='Measured parent robot/standalone state immediately before travel; new active plant retains its initialized state')
    report['articulated_velocity_scope']='Diagnostic: exact q/qvel replay plus native FK refresh may differ from previous cached solver link velocities; link velocities are never directly overwritten'
    rest_diagnostics=[]
    for path,expected in context.get('new_active_authored_rest',{}).items():
        if path not in index:raise ValueError('New active native body missing after window initialization')
        actual=initial_poses[index[path]];expected=np.array(expected)
        angle=float((Rotation.from_quat(expected[3:]).inv()*Rotation.from_quat(actual[3:])).magnitude())
        rest_diagnostics.append(dict(path=path,authored_rest_position_error_m=float(np.linalg.norm(actual[:3]-expected[:3])),
            authored_rest_orientation_error_rad=angle,
            measured_initial_velocity_world=initial_velocities[index[path]].tolist()))
    report['new_active_rest_diagnostics']=dict(bodies=rest_diagnostics,
        scope='Authored zero-joint rest versus native initialization after warmup; diagnostic only, measured velocity is not inferred as zero')
    return report
