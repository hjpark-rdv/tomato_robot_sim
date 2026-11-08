"""Observation-driven rail patrol decisions for the existing native runner.

This module owns no physics, IK, motion, or success criterion.  It remembers
visible SIM-GT fruit and proposes bounded forward moves or a target for the
existing camera-region planner.  An old observation is never a collision
certificate, and a finite rejection is never a claim of impossibility.
"""
import copy
import hashlib
import json
from pathlib import Path
import re

import numpy as np

from row_harvest_targets import select_one_bed


_FRUIT=re.compile(r'(/World/Greenhouse/Plants/Plant_(\d+))/(Truss_(\d+))/Fruit_(\d+)$')
_SCHEMA='s300-visible-camera-patrol-v1'
_SHARED_INVALID_INITIAL_MODEL='not_attempted_shared_invalid_initial_model'
# Existing rail_motion.rail_arrival_check uses2mm for measured rail tracking.
# This accepts a measured endpoint envelope only; commands/limits and contact
# geometry remain unchanged, and actual measured positions are never clamped.
_RAIL_MEASUREMENT_ENVELOPE_M=.002


def _finite(value,name):
    if isinstance(value,(bool,np.bool_)) or not isinstance(value,(float,int,np.number)) or not np.isfinite(value):
        raise ValueError(name+' must be a finite number')
    return float(value)


def _count(value,name,minimum=1):
    if isinstance(value,(bool,np.bool_)) or not isinstance(value,(int,np.integer)) or value<minimum:
        raise ValueError(name+' must be an integer >= '+str(minimum))
    return int(value)


def _inventory_sha(inventory):
    return hashlib.sha256(json.dumps(inventory,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


class CameraPatrol:
    """Persistent visible-truss queue, with forward-only <=200 mm decisions.

    ``camera_foreaft_m`` is the calibrated world-Y displacement from the
    camera to the intended service center at the current robot scan pose.
    The service station is derived from measured depth and that calibration,
    never the inventory's authored fruit centroid.  Inventory centroids are
    used only by the existing one-bed/near-side placement filter.
    """
    def __init__(self,inventory,bed_gutter_center_x,*,rail_x_m=0.,step_m=.15,
                 max_step_m=.20,station_limits_m=(0.,5.),forward_sign=1,
                 camera_foreaft_m=0.,camera_to_grasp_height_m=0.,
                 station_tolerance_m=.06,minimum_fruits=2,min_visible_pixels=12,
                 min_valid_depth_pixels=None,max_observation_age_s=1.,
                 visible_service_offset_proposal_m=.20):
        self.inventory=copy.deepcopy(inventory)
        self.inventory_sha256=_inventory_sha(inventory)
        step_m=_finite(step_m,'step_m');max_step_m=_finite(max_step_m,'max_step_m')
        if not 0<step_m<=max_step_m<=.20:raise ValueError('Positive patrol steps must satisfy step <= max <=0.20m')
        if isinstance(forward_sign,bool) or forward_sign not in (-1,1):raise ValueError('forward_sign must be -1 or 1')
        limits=np.asarray(station_limits_m,float)
        if limits.shape!=(2,) or not np.isfinite(limits).all() or limits[0]>=limits[1]:
            raise ValueError('Finite ordered station limits required')
        tolerance=_finite(station_tolerance_m,'station_tolerance_m')
        if not 0<tolerance<=max_step_m:raise ValueError('Station tolerance must be positive and no greater than max step')
        age=_finite(max_observation_age_s,'max_observation_age_s')
        if age<0:raise ValueError('Nonnegative observation age required')
        service_offset=_finite(visible_service_offset_proposal_m,'visible_service_offset_proposal_m')
        if not 0<=service_offset<=.20:raise ValueError('Visible arm service proposal offset must be within0..0.20m')
        self.config=dict(bed_gutter_center_x=_finite(bed_gutter_center_x,'bed_gutter_center_x'),
            rail_x_m=_finite(rail_x_m,'rail_x_m'),step_m=step_m,max_step_m=max_step_m,
            station_limits_m=limits.tolist(),forward_sign=int(forward_sign),
            camera_foreaft_m=_finite(camera_foreaft_m,'camera_foreaft_m'),
            camera_to_grasp_height_m=_finite(camera_to_grasp_height_m,'camera_to_grasp_height_m'),
            station_tolerance_m=tolerance,minimum_fruits=_count(minimum_fruits,'minimum_fruits',2),
            min_visible_pixels=_count(min_visible_pixels,'min_visible_pixels'),
            min_valid_depth_pixels=_count(min_visible_pixels if min_valid_depth_pixels is None else
                min_valid_depth_pixels,'min_valid_depth_pixels'),max_observation_age_s=age,
            visible_service_offset_proposal_m=service_offset)
        proposals=[dict(plant_id=row['plant_id'],truss_index=row['truss_index']) for row in inventory.get('trusses',[])]
        selected=select_one_bed(proposals,inventory,self.config['bed_gutter_center_x'],self.config['rail_x_m'])
        self.eligible={(row['plant_id'],row['truss_index']) for row in selected['stations']}
        self.excluded={(row['plant_id'],row['truss_index']):row['selection_exclusion_reason'] for row in selected['excluded']}
        self.inventory_identities={(row['plant_id'],row['truss_index']) for row in proposals}
        self.queue={};self.native_detached_keys=set();self.observations=[];self.decisions=[]
        self.plant_quarantines={}

    def _identity(self,key):
        match=_FRUIT.fullmatch(str(key))
        if match is None:raise ValueError('Exact source fruit key required: '+str(key))
        source,plant,truss,index,fruit=match.groups()
        identity=(int(plant),int(index))
        if identity not in self.inventory_identities:raise ValueError('Fruit source is absent from exact inventory')
        return source,identity,truss,int(fruit)

    def _source_plant(self,plant_path):
        sources={f'/World/Greenhouse/Plants/Plant_{plant}' for plant,_ in self.inventory_identities}
        if not isinstance(plant_path,str) or plant_path not in sources:
            raise ValueError('Exact inventory source plant path required for operational quarantine')
        return plant_path

    def quarantine_plant(self,plant_path,evidence):
        """Defer one source after a decisive shared full-plant startup failure.

        The caller must establish that the unused full-plant initial model is
        invalid.  This is not an IK/reachability verdict or a physical harvest
        attempt.  Visible entries and future observations remain recorded;
        changing truss alone cannot repair this shared input.  No unquarantine
        or model/physics change is performed here.
        """
        source=self._source_plant(plant_path)
        if not isinstance(evidence,dict) or not evidence:
            raise ValueError('Source plant quarantine requires concrete startup evidence')
        json.dumps(evidence,allow_nan=False)
        record=self.plant_quarantines.get(source)
        if record is None:
            record=dict(source_plant=source,reason=_SHARED_INVALID_INITIAL_MODEL,
                evidence=copy.deepcopy(evidence),evidence_history=[],
                operational_only=True,reachability='unknown; shared initial model was not valid for this trial',
                impossible=False,training_eligible=False,harvest_success=None)
            self.plant_quarantines[source]=record
        record['evidence_history'].append(dict(evidence=copy.deepcopy(evidence),
            observation_count=len(self.observations),decision_count=len(self.decisions)))
        station=(self.observations[-1]['station_y_m'] if self.observations else self.config['station_limits_m'][0])
        for item in self.queue.values():
            if item['source_plant']==source:self._refresh_status(item,station)
        return copy.deepcopy(record)

    def observe(self,observation,*,current_station_y_m,current_lift_m=None,native_detached_keys=()):
        """Ingest one actual exposure; retain previous targets out of view."""
        station=self._station(current_station_y_m)
        lift=None if current_lift_m is None else _finite(current_lift_m,'current_lift_m')
        if not isinstance(observation,dict):raise ValueError('Measured camera observation required')
        camera=np.asarray(observation.get('world_from_camera_optical'),float)
        if (camera.shape!=(4,4) or not np.isfinite(camera).all() or
                not np.allclose(camera[3],[0,0,0,1],rtol=0,atol=1e-8) or
                not np.allclose(camera[:3,:3].T@camera[:3,:3],np.eye(3),rtol=0,atol=1e-6) or
                np.linalg.det(camera[:3,:3])<.999999):
            raise ValueError('Finite rigid world-from-camera transform required')
        objects=observation.get('objects',observation.get('objects_path'))
        if isinstance(objects,(str,Path)):objects=json.loads(Path(objects).read_text())
        if isinstance(objects,dict):objects=objects.get('objects')
        if not isinstance(objects,list):raise ValueError('Visible objects list required')
        provenance={name:copy.deepcopy(observation[name]) for name in
            ('observation_id','timestamp_s','current_timestamp_s','physics_step','objects_path','pose_source') if name in observation}
        age=observation.get('age_s')
        if age is None and 'timestamp_s' in observation and 'current_timestamp_s' in observation:
            age=_finite(observation['current_timestamp_s'],'current_timestamp_s')-_finite(observation['timestamp_s'],'timestamp_s')
        stale=(observation.get('stale') is True or observation.get('valid') is False or
            observation.get('synchronized') is False)
        if age is not None:
            age=_finite(age,'age_s');stale=stale or age < -1e-8 or age>self.config['max_observation_age_s']
        record=dict(index=len(self.observations),station_y_m=station,current_lift_m=lift,
            provenance=provenance,age_s=age,stale=stale,rejected=[],accepted=[],queued=[],quarantined=[])
        self.record_native_detachments(native_detached_keys)
        if stale:
            record['reason']='stale_or_unsynchronized_observation';self.observations.append(record);return copy.deepcopy(record)
        for item in self.queue.values():item['visible_now']=False
        groups={};seen=set();seen_ids=set()
        for row in objects:
            if row.get('class_name')!='fruit':continue
            key=row.get('key');reason=None
            try:source,identity,truss,_=self._identity(key)
            except ValueError:reason='unknown_exact_source_fruit_identity'
            if key in seen or row.get('instance_id') is not None and row['instance_id'] in seen_ids:
                raise ValueError('Duplicate visible fruit identity')
            seen.add(key)
            if row.get('instance_id') is not None:seen_ids.add(row['instance_id'])
            if reason is None:
                if identity not in self.eligible:reason=self.excluded.get(identity,'outside_selected_bed')
                elif key in self.native_detached_keys:reason='native_detached'
                elif row.get('red_proxy') is not True:reason='not_observed_red'
                elif not self._valid_pixels(row.get('visible_pixels'),self.config['min_visible_pixels']):reason='insufficient_visible_pixels'
                elif not self._valid_pixels(row.get('valid_depth_pixels'),self.config['min_valid_depth_pixels']):reason='insufficient_valid_depth_pixels'
            surface=np.asarray(row.get('visible_surface_median_world_m'),float)
            if reason is None and (surface.shape!=(3,) or not np.isfinite(surface).all()):reason='invalid_visible_depth'
            if reason is None and np.dot(surface-camera[:3,3],camera[:3,2])<=0:reason='surface_behind_camera'
            if reason is not None:
                record['rejected'].append(dict(key=key,reason=reason));continue
            radius=row.get('known_bounding_radius_m',row.get('bounding_radius_m',0.))
            radius=_finite(radius,'known_bounding_radius_m')
            if radius<0:raise ValueError('Bounding radius cannot be negative')
            estimate=surface+radius*camera[:3,2]
            accepted=dict(key=key,instance_id=row.get('instance_id'),estimated_center_world_m=estimate.tolist(),
                visible_surface_median_world_m=surface.tolist(),known_bounding_radius_m=radius,
                position_estimate_exact=False,uses_hidden_gt_center=False)
            record['accepted'].append(accepted)
            target=source+'/'+truss
            groups.setdefault(target,dict(source_plant=source,plant_id=identity[0],truss_index=identity[1],
                truss_key=truss,fruits=[]))['fruits'].append(accepted)
        for key,group in groups.items():
            if len(group['fruits'])<self.config['minimum_fruits']:
                record['rejected'].append(dict(key=key,reason='below_visible_group_minimum',visible_fruits=len(group['fruits'])))
                continue
            centers=np.array([fruit['estimated_center_world_m'] for fruit in group['fruits']]);aim=np.median(centers,axis=0)
            goal=station+float(aim[1]-camera[1,3])-self.config['camera_foreaft_m']
            lift_goal=None if lift is None else lift+float(aim[2]-camera[2,3])-self.config['camera_to_grasp_height_m']
            previous=self.queue.get(key)
            item=previous if previous is not None else dict(key=key,source_plant=group['source_plant'],
                plant_id=group['plant_id'],truss_index=group['truss_index'],truss_key=group['truss_key'],
                first_observation_index=record['index'],fruit_evidence={},blockers=[],planning_failures=[],
                service_probe_offset_m=0.,status='pending')
            for fruit in group['fruits']:item['fruit_evidence'][fruit['key']]=fruit
            item.update(last_observation_index=record['index'],last_observation=provenance,
                visible_now=True,last_visible_fruit_keys=[fruit['key'] for fruit in group['fruits']],
                aim_world_m=aim.tolist(),observed_service_station_y_m=goal,
                service_station_y_m=(item['service_probe_station_y_m'] if item.get('service_probe_station_y_m') is not None
                    else goal+item.get('service_probe_offset_m',0.)),suggested_lift_m=lift_goal,
                observed_camera_world_m=camera[:3,3].tolist(),
                calibration=dict(camera_foreaft_m=self.config['camera_foreaft_m'],
                    camera_to_grasp_height_m=self.config['camera_to_grasp_height_m']),
                target_position_source='visible_depth_surface_median_plus_optional_known_radius',
                uses_inventory_centroid_for_aim=False)
            if item.get('service_probe_station_y_m') is not None:
                item['service_probe_arrived']=abs(item['service_probe_station_y_m']-station)<=_RAIL_MEASUREMENT_ENVELOPE_M
            self.queue[key]=item;self._refresh_status(item,station)
            record['queued'].append(key)
            if item['status']==_SHARED_INVALID_INITIAL_MODEL:
                record['quarantined'].append(dict(key=key,source_plant=item['source_plant'],
                    reason=_SHARED_INVALID_INITIAL_MODEL,visible_evidence_retained=True))
        for item in self.queue.values():self._refresh_status(item,station)
        self.observations.append(record)
        return copy.deepcopy(record)

    @staticmethod
    def _valid_pixels(value,minimum):
        return isinstance(value,(int,np.integer)) and not isinstance(value,(bool,np.bool_)) and value>=minimum

    def _station(self,value):
        value=_finite(value,'current_station_y_m');lower,upper=self.config['station_limits_m']
        if not lower-_RAIL_MEASUREMENT_ENVELOPE_M<=value<=upper+_RAIL_MEASUREMENT_ENVELOPE_M:
            raise ValueError('Measured station outside physical rail limits and existing2mm measurement envelope')
        return value

    def _refresh_status(self,item,station):
        attached=[key for key in item['fruit_evidence'] if key not in self.native_detached_keys]
        item['observed_attached_fruit_keys']=sorted(attached)
        item['native_detached_fruit_keys']=sorted(set(item['fruit_evidence'])&self.native_detached_keys)
        quarantine=self.plant_quarantines.get(item['source_plant'])
        if quarantine is not None:
            item.update(operational_exclusion_reason=_SHARED_INVALID_INITIAL_MODEL,
                operational_quarantine_source_plant=item['source_plant'],impossible=False)
        if not attached:item['status']='observed_fruits_native_detached'
        elif len(attached)<self.config['minimum_fruits']:item['status']='below_group_minimum_after_native_detachment'
        elif quarantine is not None:item['status']=_SHARED_INVALID_INITIAL_MODEL
        elif item['blockers']:item['status']='finite_blocked'
        elif self.config['forward_sign']*(item['service_station_y_m']-station)<-self._target_station_tolerance(item):
            item['status']='revisit_required'
        else:item['status']='pending'
        item['reachability']='unknown; requires existing IK, full path audit and actual physics'

    def _target_station_tolerance(self,item):
        # A deliberate50mm changed-view probe must actually move, rather than
        # be swallowed by the ordinary60mm first-service acceptance window.
        return (min(self.config['station_tolerance_m'],_RAIL_MEASUREMENT_ENVELOPE_M)
            if item.get('service_probe_station_y_m') is not None or item.get('service_probe_offset_m',0.)
            else self.config['station_tolerance_m'])

    def record_native_detachments(self,keys):
        """Only caller-verified exact native joint evidence may detach fruit."""
        for key in keys:
            self._identity(key);self.native_detached_keys.add(key)

    def record_blocker(self,target_key,reason,evidence,*,finite=True):
        if target_key not in self.queue:raise ValueError('Blocker requires a previously observed exact target')
        if finite is not True:raise ValueError('Only explicit finite operational blockers are supported')
        if not isinstance(reason,str) or not reason.strip() or not isinstance(evidence,dict) or not evidence:
            raise ValueError('Finite blocker requires a reason and concrete evidence')
        json.dumps(evidence,allow_nan=False)
        self.queue[target_key]['blockers'].append(dict(reason=reason,evidence=copy.deepcopy(evidence),
            finite=True,reachability='unknown',impossible=False))
        self.queue[target_key]['status']='finite_blocked'

    def record_local_failure(self,target_key,reason,evidence,*,current_station_y_m,
                             max_forward_views=3,forward_probe_m=.05,native_observation_executed=False):
        """Try a bounded new forward view before explicitly deferring a group.

        Call only after the existing planner exhausted its finite alternatives
        at this station, never after an unvalidated physical stop.  At most
        ``max_forward_views`` distinct actual rail stations are evaluated.
        This proposes a changed robot service position, not changed geometry.
        """
        if target_key not in self.queue:raise ValueError('Local failure requires an observed target')
        if type(native_observation_executed)is not bool:raise ValueError('Explicit native observation execution boolean required')
        station=self._station(current_station_y_m)
        budget=_count(max_forward_views,'max_forward_views')
        if budget>3:raise ValueError('Initial patrol local search allows at most three forward views')
        probe=_finite(forward_probe_m,'forward_probe_m')
        if not 0<probe<=self.config['max_step_m']:raise ValueError('Forward service probe must fit the configured max step')
        if not isinstance(reason,str) or not reason.strip() or not isinstance(evidence,dict) or not evidence:
            raise ValueError('Local failure requires a reason and concrete evidence')
        json.dumps(evidence,allow_nan=False)
        item=self.queue[target_key]
        proposal=item.get('current_station_service_proposal')
        matching_proposal=(isinstance(proposal,dict) and proposal.get('decision_index')==len(self.decisions)-1 and
            abs(proposal['actual_station_y_m']-station)<=_RAIL_MEASUREMENT_ENVELOPE_M and
            proposal.get('requires_existing_ik_full_audit')is True)
        if abs(item['service_station_y_m']-station)>self._target_station_tolerance(item) and not matching_proposal:
            raise ValueError('Local planning failure must belong to this target service station')
        failures=item.setdefault('planning_failures',[])
        failures.append(dict(station_y_m=station,reason=reason,evidence=copy.deepcopy(evidence),
            native_physics_executed=native_observation_executed,
            native_observation_executed=native_observation_executed,harvest_motion_executed=False,
            reachability='unknown; finite planning proposal failure'))
        distinct=[]
        for failure in failures:
            if all(abs(failure['station_y_m']-value)>1e-6 for value in distinct):distinct.append(failure['station_y_m'])
        sign=self.config['forward_sign'];next_station=station+sign*probe
        end=self.config['station_limits_m'][1 if sign>0 else 0]
        if len(distinct)>=budget or sign*(end-next_station)<0:
            self.record_blocker(target_key,'finite_local_forward_view_budget_exhausted',
                dict(max_forward_views=budget,evaluated_distinct_stations_y_m=distinct,
                    failures=copy.deepcopy(failures),rail_limit_prevents_next_probe=sign*(end-next_station)<0))
            return dict(status='finite_deferred',target_key=target_key,reachability='unknown',
                evaluated_distinct_stations_y_m=distinct,harvest_success=None)
        base=item.get('observed_service_station_y_m',item['service_station_y_m']-item.get('service_probe_offset_m',0.))
        item.update(service_probe_offset_m=next_station-base,service_station_y_m=next_station,
            service_probe_station_y_m=next_station,service_probe_arrived=False,
            visible_now=False,status='pending')
        item.pop('current_station_service_proposal',None)
        return dict(status='forward_view_retry_pending',target_key=target_key,
            next_station_y_m=next_station,evaluated_distinct_stations_y_m=distinct,
            must_reobserve_before_harvest=True,reachability='unknown',harvest_success=None)

    def decide(self,*,current_station_y_m,active_source_plant=None,frozen_source_plants=(),
               active_collision_target_key=None,forward_pass=False):
        """Return one proposal; caller must execute/audit it and observe again."""
        if type(forward_pass)is not bool:raise ValueError('Forward pass requires an explicit boolean')
        if active_collision_target_key is not None and (not isinstance(active_collision_target_key,str) or
                not re.fullmatch(r'/World/Greenhouse/Plants/Plant_\d+/Truss_\d+',active_collision_target_key) or
                active_collision_target_key.rsplit('/',1)[0]!=active_source_plant):
            raise ValueError('Active collision target preference requires an exact current source/truss identity')
        station=self._station(current_station_y_m);sign=self.config['forward_sign'];tol=self.config['station_tolerance_m']
        frozen=set(frozen_source_plants)
        for item in self.queue.values():self._refresh_status(item,station)
        pending=[row for row in self.queue.values() if row['status'] in ('pending','revisit_required')]
        # Exact previous active windows cannot be rebuilt from authored rest.
        for item in pending:
            if item['source_plant'] in frozen:
                item['status']='revisit_requires_measured_window_recovery'
        pending=[row for row in pending if row['source_plant'] not in frozen]
        active=[row for row in pending if row['source_plant']==active_source_plant]
        candidates=active if active else pending
        near=[row for row in candidates if abs(row['service_station_y_m']-station)<=self._target_station_tolerance(row)]
        def current_visible(row):
            return bool(row['visible_now'] and self.observations and not self.observations[-1]['stale'] and
                row['last_observation_index']==len(self.observations)-1 and
                abs(self.observations[-1]['station_y_m']-station)<=_RAIL_MEASUREMENT_ENVELOPE_M)
        offset_candidates=[row for row in candidates if current_visible(row) and
            row.get('service_probe_station_y_m') is None and not row.get('service_probe_offset_m',0.) and
            self._target_station_tolerance(row)<abs(row['service_station_y_m']-station)<=self.config['visible_service_offset_proposal_m']]
        serviceable=near+offset_candidates
        if self.observations and self.observations[-1]['stale']:
            result=dict(action='scan',reason='reacquire_fresh_synchronized_camera_before_movement_or_harvest',
                current_station_y_m=station,next_station_y_m=station)
        elif serviceable:
            preferred=[row for row in serviceable if row['key']==active_collision_target_key and current_visible(row)]
            target=min(preferred or serviceable,key=lambda row:(row not in near,-row['aim_world_m'][2],
                abs(row['service_station_y_m']-station),row['key']))
            result=dict(action='harvest' if target['visible_now'] else 'scan',reason='visible_pending_group_at_service_station' if
                target['visible_now'] else 'reacquire_remembered_pending_group_before_harvest',target_key=target['key'],
                target=copy.deepcopy(target),current_station_y_m=station,next_station_y_m=station,
                requires_window_switch=target['source_plant']!=active_source_plant)
            if target not in near:
                proposal=dict(decision_index=len(self.decisions),actual_station_y_m=station,
                    observed_service_station_y_m=target['observed_service_station_y_m'],
                    service_station_y_m=target['service_station_y_m'],
                    along_row_offset_m=target['service_station_y_m']-station,
                    maximum_proposal_offset_m=self.config['visible_service_offset_proposal_m'],
                    ordinary_arrival_tolerance_m=self.config['station_tolerance_m'],
                    arrival_tolerance_changed=False,rail_move_requested=False,
                    requires_existing_ik_full_audit=True,uses_current_visible_depth=True,
                    reachability='proposal only; existing robot planner and actual physics must decide')
                target['current_station_service_proposal']=copy.deepcopy(proposal)
                result.update(reason='current_visible_group_bounded_arm_service_proposal',
                    service_offset_proposal=proposal,target=copy.deepcopy(target))
        else:
            ahead=[row for row in candidates if sign*(row['service_station_y_m']-station)>self._target_station_tolerance(row)]
            target=min(ahead,key=lambda row:(sign*(row['service_station_y_m']-station),-row['aim_world_m'][2],row['key'])) if ahead else None
            revisit=[row['key'] for row in (active if active else self.queue.values()) if row['status'] in
                ('revisit_required','revisit_requires_measured_window_recovery')]
            if revisit and not (forward_pass and not active):
                # Do not hide an observed eligible target by calling it empty.
                result=dict(action='stop',reason='pending_target_requires_explicit_revisit_or_finite_blocker',
                    pending_target_keys=sorted(revisit),current_station_y_m=station,next_station_y_m=station)
            else:
                end=self.config['station_limits_m'][1 if sign>0 else 0]
                distance=min(self.config['step_m'],max(0.,sign*(end-station)))
                if target is not None:distance=min(distance,max(0.,sign*(target['service_station_y_m']-station)))
                lower,upper=self.config['station_limits_m']
                next_station=float(np.clip(station+sign*distance,lower,upper))
                if distance<1e-9:result=dict(action='stop',reason='physical_rail_end_reached',current_station_y_m=station,next_station_y_m=station)
                else:
                    result=dict(action='move',reason='short_step_toward_observed_pending_group' if target else
                        'short_forward_step_then_observe',current_station_y_m=station,next_station_y_m=next_station,
                        movement_m=distance,requires_camera_observation_after_motion=True)
                    if target is not None:result.update(target_key=target['key'],target=copy.deepcopy(target))
        if forward_pass:
            result['forward_pass']=dict(enabled=True,
                inactive_revisit_target_keys_preserved=sorted(row['key'] for row in self.queue.values()
                    if row['source_plant']!=active_source_plant and row['status'] in
                        ('revisit_required','revisit_requires_measured_window_recovery')),
                active_pending_groups_not_deferred=True,
                deferred_revisits_are_not_completed=True,harvest_credit=0)
        if active_collision_target_key is not None:
            result['active_truss_cost_preference']=dict(target_key=active_collision_target_key,
                eligible_current_visible_target_keys=sorted(row['key']for row in serviceable if current_visible(row)),
                applied=result.get('action')=='harvest' and result.get('target_key')==active_collision_target_key,
                scope='Ranking among existing fresh visible serviceable candidates only; no new eligibility or geometry',
                other_pending_targets_preserved=True,harvest_credit=0)
        result['active_window_priority']=dict(source_plant=active_source_plant,
            active_pending_target_keys=sorted(row['key'] for row in active),
            other_pending_targets_retained=sorted(row['key'] for row in pending if row['source_plant']!=active_source_plant),
            avoids_freezing_unresolved_active_groups=True)
        result.update(index=len(self.decisions),training_eligible=False,harvest_success=None,
            physics_execution_required=True,finite_blocked_target_keys=sorted(row['key'] for row in self.queue.values() if row['status']=='finite_blocked'),
            operational_quarantined_source_plants=sorted(self.plant_quarantines),
            quarantined_target_keys=sorted(row['key'] for row in self.queue.values() if row['status']==_SHARED_INVALID_INITIAL_MODEL))
        lower,upper=self.config['station_limits_m']
        result['rail_measurement']=dict(actual_station_y_m=station,nominal_physical_bounds_m=[lower,upper],
            existing_measurement_envelope_m=_RAIL_MEASUREMENT_ENVELOPE_M,
            outside_nominal_bounds=not lower<=station<=upper,actual_measurement_clamped=False,
            physical_limits_changed=False,collision_tolerances_changed=False)
        self.decisions.append(copy.deepcopy(result));return result

    def to_dict(self):
        value=dict(schema=_SCHEMA,inventory_sha256=self.inventory_sha256,config=copy.deepcopy(self.config),
            queue=copy.deepcopy(self.queue),native_detached_keys=sorted(self.native_detached_keys),
            plant_quarantines=copy.deepcopy(self.plant_quarantines),
            observations=copy.deepcopy(self.observations),decisions=copy.deepcopy(self.decisions),
            training_eligible=False,harvest_success=None,
            success_counting='Outer run_row_harvest.measured_harvested_trusses only; this ledger grants no harvest credit')
        json.dumps(value,allow_nan=False);return value

    @classmethod
    def from_dict(cls,inventory,value):
        if value.get('schema')!=_SCHEMA or value.get('inventory_sha256')!=_inventory_sha(inventory):
            raise ValueError('Camera patrol ledger/inventory identity mismatch')
        result=cls(inventory,**copy.deepcopy(value['config']))
        result.queue=copy.deepcopy(value['queue']);result.observations=copy.deepcopy(value['observations'])
        result.decisions=copy.deepcopy(value['decisions']);result.record_native_detachments(value['native_detached_keys'])
        result.plant_quarantines=copy.deepcopy(value.get('plant_quarantines',{}))
        if not isinstance(result.plant_quarantines,dict):raise ValueError('Invalid source plant quarantine ledger')
        for source,record in result.plant_quarantines.items():
            result._source_plant(source)
            if (not isinstance(record,dict) or record.get('source_plant')!=source or
                    record.get('reason')!=_SHARED_INVALID_INITIAL_MODEL or record.get('operational_only')is not True or
                    record.get('impossible')is not False or not isinstance(record.get('evidence'),dict) or
                    not record['evidence'] or not isinstance(record.get('evidence_history'),list) or not record['evidence_history']):
                raise ValueError('Invalid source plant quarantine evidence')
        for key,item in result.queue.items():
            if key!=item['source_plant']+'/'+item['truss_key'] or not item['fruit_evidence']:
                raise ValueError('Invalid exact camera queue target identity')
            for fruit in item['fruit_evidence']:
                source,identity,truss,_=result._identity(fruit)
                if source!=item['source_plant'] or truss!=item['truss_key'] or identity not in result.eligible:
                    raise ValueError('Camera queue fruit/source/bed identity mismatch')
        result.to_dict();return result
