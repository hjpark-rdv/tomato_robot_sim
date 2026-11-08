"""Opt-in arm-only belt entry backoff; no change to plant or contact geometry."""
import numpy as np


def retract_translation(translation,direction,amount_mm):
    """Keep the selected yaw and shift the existing TCP goal away from fruit."""
    if (isinstance(amount_mm,(bool,np.bool_)) or not isinstance(amount_mm,(int,float)) or
            not np.isfinite(amount_mm) or not 0<=amount_mm<=15):
        raise ValueError('Belt entry retraction must be finite0..15mm')
    t=np.asarray(translation,float);d=np.asarray(direction,float)
    if (t.shape!=(3,) or d.shape!=(3,) or not np.isfinite(np.r_[t,d]).all() or
            abs(d[2])>1e-9 or np.linalg.norm(d)<1e-9):
        raise ValueError('Belt entry retraction requires a finite horizontal existing entry direction')
    d=d/np.linalg.norm(d);delta=-d*(amount_mm/1000)
    return t+delta,dict(amount_mm=float(amount_mm),translation_world_m=delta.tolist(),
        entry_direction_world=d.tolist(),plant_pose_modified=False,physical_geometry_modified=False,
        source='Arm goal/preparation translation after ordinary group selection; belt may draw partially entered fruit inward',
        selected_group_full_containment_is_not_assumed=True,physical_contact_verified=False,
        detachment='unknown_until_native_physics',training_eligible=False)


def measured_entry_window(group,fruits,goal_pose):
    """Describe actual selected spheres at the retracted goal, without credit."""
    if group is None:raise ValueError('Belt entry backoff requires its exact selected fruit group')
    names=group['fruits'];points=np.asarray(group['measured_centers_world_m'],float)
    pose=np.asarray(goal_pose,float)
    if (points.shape!=(len(names),3) or pose.shape!=(4,4) or
            not np.isfinite(np.r_[points.ravel(),pose.ravel()]).all()):
        raise ValueError('Finite measured group and actual goal pose required')
    inventory={f['name']:f for f in fruits}
    if len(inventory)!=len(fruits) or len(set(names))!=len(names) or not set(names)<=set(inventory):
        raise ValueError('Exact unambiguous selected fruit identities required')
    radii=np.asarray([max(inventory[name]['dimensions_m'])/2 for name in names])
    if not np.isfinite(radii).all() or np.any(radii<=0):raise ValueError('Positive fruit sphere bounds required')
    local=(points-pose[:3,3])@pose[:3,:3]
    axial_overlap=(local[:,1]+radii>0)&(local[:,1]-radii<.158)
    axial_full=(local[:,1]-radii>=-1e-10)&(local[:,1]+radii<=.158+1e-10)
    if not axial_overlap.all():raise ValueError('Retracted selected fruit does not overlap the existing belt depth span')
    return dict(source='Measured selected centers transformed by actual retracted planned goal; no new camera observation',
        selection_footprint_scope='Original full-containment group proposal before arm-only backoff; not the actual entry claim',
        belt_depth_span_m=[0.,.158],open_half_width_m=.048,existing_height_center_span_m=[.013,.061],
        fruit_names=list(names),fruit_centers_tool_m=local.tolist(),sphere_radii_m=radii.tolist(),
        belt_depth_sphere_intervals_m=np.column_stack((local[:,1]-radii,local[:,1]+radii)).tolist(),
        overlaps_belt_depth_span=axial_overlap.tolist(),fully_inside_belt_depth_span=axial_full.tolist(),
        fits_original_open_width=(np.abs(local[:,0])+radii<=.048+1e-10).tolist(),
        within_original_height_center_span=((local[:,2]>=.013-1e-10)&(local[:,2]<=.061+1e-10)).tolist(),
        all_selected_fruits_fully_inside_belt_depth_span=bool(axial_full.all()),physical_contact_verified=False,
        harvest_success=None,training_eligible=False)
