"""Visible SIM-GT RGB-D proposals for the existing harvest-group selector.

This is an observation filter and approximate aim estimator. It does not infer
hidden fruit, certify a collision-free approach, or change physics geometry.
"""
import copy
import json
from pathlib import Path
import re

import numpy as np

from harvest_group_policy import HarvestGroupSelector
from row_harvest_targets import truss_key


class CameraRegionSelector:
    """Restrict ripe-group proposals to this plant's observed visible fruit.

    ``observe(body_paths, poses)`` returns a dict with ``objects`` (the list in
    mounted GT objects.json, or that JSON's path), a world-from-optical camera
    matrix, and observation provenance. Each accepted surface median receives
    one known model bounding radius along optical +Z. This is explicitly an
    approximate center estimate; it never substitutes a hidden GT center.
    """
    def __init__(self, policy, direction, observe, *, depth_m=.065,
                 height_offset_m=0., output=None, plant_source,
                 min_visible_pixels=12, min_valid_depth_pixels=None,
                 selected_truss=None, max_observation_age_s=1.,
                 prefer_unattempted_trusses=False, yaw_offset_deg=0.):
        if not re.fullmatch(r'/World/Greenhouse/Plants/Plant_\d+',str(plant_source)):
            raise ValueError('Exact source plant path required')
        if selected_truss is not None and not re.fullmatch(r'Truss_\d+',str(selected_truss)):
            raise ValueError('Exact selected Truss_nn identity required')
        if not isinstance(prefer_unattempted_trusses,bool):
            raise ValueError('Explicit boolean unattempted-truss preference required')
        self.prefer_unattempted_trusses=prefer_unattempted_trusses
        self.yaw_offset_deg=yaw_offset_deg
        self.minimum=int(policy['minimum_fruits'])
        self.min_visible_pixels=self._count(min_visible_pixels,'min_visible_pixels')
        self.min_valid_depth_pixels=self._count(
            min_visible_pixels if min_valid_depth_pixels is None else min_valid_depth_pixels,
            'min_valid_depth_pixels')
        if not np.isfinite(max_observation_age_s) or max_observation_age_s<0:
            raise ValueError('Nonnegative finite observation age required')
        self.max_observation_age_s=float(max_observation_age_s)
        self.policy=copy.deepcopy(policy)
        self.original_eligible={row['name'] for row in policy['fruit_eligibility'] if row['eligible']}
        self.direction_hint=direction;self.observe=observe
        self.plant_source=str(plant_source);self.selected_truss=selected_truss
        self.depth_m=depth_m;self.height_offset_m=height_offset_m
        self.output=Path(output) if output else None
        self.history=[];self.last_decision=None;self._directions={}
        # A fully hidden truss has no eligible seed, so this harmless fallback
        # only permits the base selector to enumerate its ineligible obstacles.
        self.base=HarvestGroupSelector(policy,lambda key,*args:self._directions.get(key,np.array([0.,1.,0.])),
            depth_m=depth_m,height_offset_m=height_offset_m)

    @staticmethod
    def _validated_yaw_offset(value):
        # This bounds a local proposal search, not plant/physics validity.
        if (isinstance(value,(bool,np.bool_)) or not isinstance(value,(int,float,np.integer,np.floating)) or
                not np.isfinite(value) or abs(value)>90.):
            raise ValueError('Camera entry yaw offset must be finite within -90..90 degrees')
        return float(value)

    @property
    def yaw_offset_deg(self):
        return self._yaw_offset_deg

    @yaw_offset_deg.setter
    def yaw_offset_deg(self,value):
        self._yaw_offset_deg=self._validated_yaw_offset(value)

    def set_yaw_offset_deg(self,value):
        """Change only the next visible-heading proposal; footprint is rerun."""
        self.yaw_offset_deg=value

    @staticmethod
    def _count(value,name):
        if isinstance(value,bool) or not isinstance(value,(int,np.integer)) or value<1:
            raise ValueError(name+' must be a positive integer')
        return int(value)

    def _observation(self, body_paths, poses):
        observation=self.observe(body_paths,poses)
        if not isinstance(observation,dict):raise ValueError('Camera observation must be a mapping')
        data=observation.get('objects',observation.get('objects_path'))
        objects_path=None
        if isinstance(data,(str,Path)):
            objects_path=str(Path(data).resolve())
            loaded=json.loads(Path(data).read_text())
            data=loaded['objects'] if isinstance(loaded,dict) else loaded
        elif isinstance(data,dict):data=data.get('objects')
        if not isinstance(data,list):raise ValueError('Mounted GT visible objects list required')
        matrix=np.asarray(observation.get('world_from_camera_optical'),float)
        if (matrix.shape!=(4,4) or not np.isfinite(matrix).all() or
                not np.allclose(matrix[3],[0.,0.,0.,1.],rtol=0,atol=1e-8) or
                not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),rtol=0,atol=1e-6) or
                np.linalg.det(matrix[:3,:3])<.999999):
            raise ValueError('Finite rigid world-from-optical camera transform required')
        stale=(observation.get('stale') is True or observation.get('valid') is False or
               observation.get('synchronized') is False)
        age=observation.get('age_s')
        if age is None and all(k in observation for k in ('timestamp_s','current_timestamp_s')):
            age=float(observation['current_timestamp_s'])-float(observation['timestamp_s'])
        if age is not None:
            stale=stale or not np.isfinite(age) or age < -1e-8 or age>self.max_observation_age_s
        provenance={k:observation[k] for k in ('observation_id','observation_path','path',
            'timestamp_s','current_timestamp_s','physics_step','pose_source') if k in observation}
        if objects_path is not None:provenance['objects_path']=objects_path
        for name in ('timestamp_s','current_timestamp_s'):
            value=provenance.get(name)
            if isinstance(value,(int,float,np.number)) and not np.isfinite(value):
                provenance[name]=None;stale=True
        provenance.update(age_s=float(age) if age is not None and np.isfinite(age) else None,
            stale=bool(stale),world_from_camera_optical=matrix.tolist())
        return data,matrix,stale,provenance

    def __call__(self,fruits,breaks,body_paths,poses,attempts,max_attempts=2,
                 order='highest-first',single_fruit_when_remaining=0,rejected_groups=()):
        yaw_offset=self._validated_yaw_offset(self.yaw_offset_deg)
        paths=list(body_paths);working=np.asarray(poses,float).copy()
        if len(paths)!=len(set(paths)) or working.ndim!=2 or working.shape[0]!=len(paths) or working.shape[1]<3:
            raise ValueError('Unique body paths and finite measured positions required')
        if not np.isfinite(working).all():raise ValueError('Finite measured body states required')
        records,camera,stale,provenance=self._observation(paths,poses)
        by_key={};by_name={};broken={row['joint'] for row in breaks}
        for fruit in fruits:
            name=fruit['name'];key=truss_key(name)
            source=f'{self.plant_source}/{key}/Fruit_'+name.split('_Fruit_')[1]
            if source in by_key or name in by_name:raise ValueError('Duplicate exact target fruit identity')
            by_key[source]=fruit;by_name[name]=fruit
        accepted=[];rejected=[];seen_keys=set();seen_ids=set()
        for row in records:
            if row.get('class_name')!='fruit':continue
            key=row.get('key');identity=row.get('instance_id');fruit=by_key.get(key)
            reason=None
            if not isinstance(key,str) or not key.startswith(self.plant_source+'/'):
                reason='other_source_plant'
            elif fruit is None:reason='unknown_target_fruit_key'
            elif key in seen_keys or identity is not None and identity in seen_ids:
                raise ValueError('Duplicate visible fruit identity')
            else:
                seen_keys.add(key)
                if identity is not None:seen_ids.add(identity)
                truss=truss_key(fruit['name'])
                if self.selected_truss is not None and truss!=self.selected_truss:reason='outside_selected_truss'
                elif stale:reason='stale_or_unsynchronized_observation'
                elif fruit['joint'] in broken:reason='already_detached'
                elif fruit['name'] not in self.original_eligible:reason='original_policy_ineligible'
                elif row.get('red_proxy') is not True:reason='not_observed_red'
                elif not self._valid_count(row.get('visible_pixels'),self.min_visible_pixels):reason='insufficient_visible_pixels'
                elif not self._valid_count(row.get('valid_depth_pixels'),self.min_valid_depth_pixels):reason='insufficient_valid_depth_pixels'
            surface=np.asarray(row.get('visible_surface_median_world_m'),float)
            if reason is None and (surface.shape!=(3,) or not np.isfinite(surface).all()):reason='invalid_surface_depth'
            if reason is None and np.dot(surface-camera[:3,3],camera[:3,2])<=0:reason='surface_behind_camera'
            if reason is not None:
                rejected.append(dict(instance_id=identity,key=key,reason=reason));continue
            radius=float(max(fruit['dimensions_m'])/2)
            if not np.isfinite(radius) or radius<=0:raise ValueError('Positive finite fruit model radius required')
            estimate=surface+radius*camera[:3,2]
            if fruit['path'] not in paths:raise ValueError('Observed target absent from measured body identities')
            working[paths.index(fruit['path']),:3]=estimate
            accepted.append(dict(instance_id=identity,key=key,name=fruit['name'],truss=truss_key(fruit['name']),
                visible_pixels=int(row['visible_pixels']),valid_depth_pixels=int(row['valid_depth_pixels']),
                surface_median_world_m=surface.tolist(),known_bounding_radius_m=radius,
                estimated_center_world_m=estimate.tolist(),position_estimate_exact=False))
        self._directions={};direction_evidence={};direction_rejected=[]
        for key in sorted({row['truss'] for row in accepted}):
            selected=[row for row in accepted if row['truss']==key]
            if len(selected)<2:
                direction_rejected.extend(dict(instance_id=row['instance_id'],key=row['key'],
                    reason='fewer_than_two_visible_fruits_for_direction') for row in selected)
                continue
            subset=[by_name[row['name']] for row in selected]
            points=np.array([row['estimated_center_world_m'] for row in selected])
            hint=None
            if callable(self.direction_hint):
                # The helper receives only observed fruit, never hidden members.
                hint=np.asarray(self.direction_hint(key,subset,breaks,paths,working),float)
            elif self.direction_hint is not None:hint=np.asarray(self.direction_hint,float)
            if hint is None or hint.shape!=(3,) or not np.isfinite(hint).all() or np.linalg.norm(hint[:2])<1e-9:
                hint=points.mean(0)-camera[:3,3]
            hint=hint.copy();hint[2]=0.
            if np.linalg.norm(hint)<1e-9:
                direction_rejected.extend(dict(instance_id=row['instance_id'],key=row['key'],
                    reason='no_observable_horizontal_direction') for row in selected);continue
            # A principal axis is an UNSIGNED line. A noisy fruit PCA hint
            # must never choose its sign against the current observed inward
            # ray, which can put staging on the far side of the crop.
            inward=points.mean(0)-camera[:3,3];inward[2]=0.
            sign_reference=inward if np.linalg.norm(inward)>1e-9 else hint
            centered=points[:,:2]-points[:,:2].mean(0)
            eigenvalues,eigenvectors=np.linalg.eigh(centered.T@centered)
            rachis=[]
            for record in records:
                if (record.get('class_name')!='rachis' or record.get('truss')!=key or
                        record.get('key')!=f'{self.plant_source}/{key}/Rachis' or
                        not self._valid_count(record.get('visible_pixels'),self.min_visible_pixels) or
                        not self._valid_count(record.get('valid_depth_pixels'),self.min_valid_depth_pixels)):
                    continue
                candidate=np.asarray(record.get('visible_surface_xy_axis_world'),float)
                ratio=record.get('visible_surface_xy_anisotropy')
                if (candidate.shape==(3,) and np.isfinite(candidate).all() and
                        abs(candidate[2])<1e-9 and abs(np.linalg.norm(candidate)-1)<1e-6 and
                        isinstance(ratio,(int,float)) and np.isfinite(ratio) and ratio>=4.):
                    rachis.append((candidate,record))
            if len(rachis)>1:raise ValueError('Duplicate exact visible Rachis heading')
            if rachis:
                axis,rachis_record=rachis[0]
                axis=axis.copy()
                if np.dot(axis,sign_reference)<0:axis=-axis
                method='horizontal_PCA_of_visible_Rachis_depth_pixels'
                horizontal_fraction=rachis_record.get('visible_surface_long_axis_horizontal_fraction')
                if (isinstance(horizontal_fraction,(int,float)) and np.isfinite(horizontal_fraction) and
                        0<=horizontal_fraction<np.sin(np.deg2rad(15.))):
                    # For a nearly vertical observed branch, top-view yaw is
                    # poorly determined by its small curved surface segment.
                    # Propose the observed camera-facing direction instead;
                    # the existing footprint/IK/physics checks still decide.
                    axis=sign_reference/np.linalg.norm(sign_reference)
                    method='camera_to_visible_centroid_for_near_vertical_Rachis'
            elif eigenvalues[-1]>1e-12:
                axis=np.r_[eigenvectors[:,-1],0.]
                if np.dot(axis,sign_reference)<0:axis=-axis
                method='horizontal_PCA_of_visible_approximate_centers'
            else:axis=hint/np.linalg.norm(hint);method='camera_to_visible_centroid_horizontal_fallback'
            unrotated_axis=axis.copy()
            if yaw_offset:
                angle=np.deg2rad(yaw_offset);c,s=np.cos(angle),np.sin(angle)
                axis=np.array([c*axis[0]-s*axis[1],s*axis[0]+c*axis[1],axis[2]])
            self._directions[key]=axis
            direction_evidence[key]=dict(method=method,visible_fruit_count=len(selected),
                direction_world=axis.tolist(),visible_instance_ids=[row['instance_id'] for row in selected],
                axis_sign_source='current camera to observed fruit centroid; direction hint only if ray is degenerate',
                camera_to_observed_centroid_horizontal_m=inward.tolist(),
                inward_alignment_cosine=float(np.dot(axis,sign_reference)/np.linalg.norm(sign_reference)))
            if yaw_offset:
                direction_evidence[key].update(unrotated_direction_world=unrotated_axis.tolist(),
                    yaw_offset_deg=yaw_offset,rotation_axis_world=[0.,0.,1.],
                    entry_direction_already_yaw_rotated=True,
                    rotation_stage='Before existing group footprint and lateral centering; no pitch or hidden aim')
            if rachis:
                direction_evidence[key].update(rachis_instance_id=rachis_record['instance_id'],
                    rachis_key=rachis_record['key'],rachis_visible_pixels=rachis_record['visible_pixels'],
                    rachis_xy_anisotropy=rachis_record['visible_surface_xy_anisotropy'],
                    uses_hidden_geometry=False,
                    rachis_long_axis_horizontal_fraction=rachis_record.get('visible_surface_long_axis_horizontal_fraction'),
                    near_vertical_proposal_threshold_deg=15.,
                    near_vertical_threshold_is_physics_validity_gate=False)
        eligible={row['name'] for row in accepted if row['truss'] in self._directions}
        self.base.eligible=eligible;self.base.depth_m=self.depth_m
        self.base.height_offset_m=self.height_offset_m
        attempted_trusses={truss_key(fruit['name']) for fruit in fruits
                           if attempts.get(fruit['name'],0)>0}
        new_trusses={truss_key(name) for name in eligible}-attempted_trusses
        preferred={name for name in eligible if truss_key(name) in new_trusses}
        priority_probe=self.prefer_unattempted_trusses and bool(preferred) and preferred!=eligible
        fallback=False
        if priority_probe:
            # A failed priority probe is not a separate observation, physical
            # attempt, or consumed initial planning rejection. Restore those
            # bookkeeping fields before evaluating the ordinary fallback.
            history_before=list(self.base.history);decision_before=self.base.last_decision
            self.base.eligible=preferred
            selected=self.base(fruits,breaks,paths,working,attempts,max_attempts=max_attempts,order=order,
                single_fruit_when_remaining=single_fruit_when_remaining,rejected_groups=rejected_groups)
            if selected is None:
                self.base.history[:]=history_before;self.base.last_decision=decision_before
                self.base.eligible=eligible;fallback=True
                selected=self.base(fruits,breaks,paths,working,attempts,max_attempts=max_attempts,order=order,
                    single_fruit_when_remaining=single_fruit_when_remaining,rejected_groups=rejected_groups)
        else:
            selected=self.base(fruits,breaks,paths,working,attempts,max_attempts=max_attempts,order=order,
                single_fruit_when_remaining=single_fruit_when_remaining,rejected_groups=rejected_groups)
        # Leave public eligibility equal to the complete currently observable
        # set; the priority pass only ranks proposals and never bans a truss.
        self.base.eligible=eligible
        exploration=dict(enabled=self.prefer_unattempted_trusses,
            attempted_trusses=sorted(attempted_trusses),
            observable_unattempted_trusses=sorted(new_trusses),
            priority_probe=priority_probe,fallback_to_ordinary_ranking=fallback,
            attempt_source='caller physical fruit attempt ledger only; planning rejections do not count',
            selected_truss=None if selected is None else selected['truss_key'],
            selected_truss_attempted_before=False if selected is None else selected['truss_key'] in attempted_trusses)
        evidence=dict(schema='s300-camera-region-decision-v1',plant_source=self.plant_source,
            selected_truss=self.selected_truss,observation=provenance,
            center_estimator='visible_surface_median_plus_known_bounding_radius_along_optical_forward',
            heading_policy='visible_Rachis_line_camera_inward_sign_near_vertical_radial_v2',
            position_estimate_exact=False,uses_hidden_gt_centers_for_aim=False,
            minimum_visible_pixels=self.min_visible_pixels,minimum_valid_depth_pixels=self.min_valid_depth_pixels,
            accepted_observations=accepted,rejected_observations=rejected+direction_rejected,
            unobserved_target_fruits=sorted(fruit['name'] for key,fruit in by_key.items() if key not in seen_keys),
            direction_estimates=direction_evidence,observable_eligible_fruits=sorted(eligible),
            truss_exploration=exploration,
            hidden_and_ineligible_fruit_preserved_as_footprint_obstacles=True,
            planning_and_physics_required=True,training_eligible=False)
        if yaw_offset:
            evidence.update(entry_yaw_offset_deg=yaw_offset,entry_direction_already_yaw_rotated=True,
                yaw_proposal_footprint_recomputed=True)
        if selected is not None:
            selected['camera_region']=copy.deepcopy(evidence)
            selected['group_selection']['maturity_source']='visible_SIM_GT_red_proxy_intersected_with_original_policy'
            selected['group_selection']['center_source']=evidence['center_estimator']
        self.last_decision=copy.deepcopy(self.base.last_decision)
        if stale:self.last_decision['reason']='camera_observation_stale_or_unsynchronized'
        elif not eligible:self.last_decision['reason']='no_visible_qualifying_group'
        self.last_decision['camera_region']=evidence
        self.history.append(self.last_decision)
        if self.output:
            self.output.parent.mkdir(parents=True,exist_ok=True)
            self.output.write_text(json.dumps(self.history,indent=2,allow_nan=False)+'\n')
        return selected

    @staticmethod
    def _valid_count(value,minimum):
        return not isinstance(value,bool) and isinstance(value,(int,np.integer)) and value>=minimum
