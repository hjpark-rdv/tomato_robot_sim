"""Bounded retries of freshly visible V6 fruit, including single-fruit cleanup."""
import copy

import numpy as np

from camera_patrol import CameraPatrol
from side_camera_motion import PlanningError
from run_newton_camera_patrol import select_group,source_fruit_manifest,read


class SideCoveragePatrol(CameraPatrol):
    """Keep the common patrol's two-fruit default unchanged elsewhere."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        # V6 cleanup is an explicit observation policy; all original pixel,
        # ripeness, depth, motion and collision checks still apply.
        self.config.update(minimum_fruits=1,single_visible_fruit_cleanup=True)
        self.reopened_views=[]

    def reconsider_visible(self,station):
        """Allow one genuinely changed forward view of a deferred local search."""
        if not self.observations or self.observations[-1]['stale']:return []
        reopened=[]
        for item in self.queue.values():
            if (item['status']!='finite_blocked' or not item.get('visible_now') or
                    item.get('last_observation_index')!=len(self.observations)-1 or
                    item.get('changed_view_retries',0)>=1 or
                    item['source_plant']in self.plant_quarantines):continue
            blockers=item['blockers'];failures=item.get('planning_failures',[])
            if (not blockers or not failures or
                    any(row['reason']!='finite_local_forward_view_budget_exhausted'for row in blockers) or
                    station-max(row['station_y_m']for row in failures)<.12):continue
            visible=set(item.get('last_visible_fruit_keys',[]))-self.native_detached_keys
            if not visible:continue
            # Bound this late retry to the existing near-side arm workspace.
            relative_y=item['aim_world_m'][1]-station
            if not -.10<=relative_y<=.55:continue
            evidence=dict(target=item['key'],station_m=station,observation=self.observations[-1]['provenance'],
                visible_remaining=sorted(visible),previous_blockers=copy.deepcopy(blockers),
                reachability='unknown; requires fresh native IK and complete physical motion')
            self.reopened_views.append(evidence);reopened.append(item['key'])
            item.setdefault('deferred_history',[]).extend(copy.deepcopy(blockers));item['blockers']=[]
            item.update(changed_view_retries=item.get('changed_view_retries',0)+1,
                service_probe_station_y_m=station,service_station_y_m=station,
                service_probe_offset_m=station-item['observed_service_station_y_m'],service_probe_arrived=True)
            self._refresh_status(item,station)
        return reopened


def plan_visible_alternatives(motion,q,observation,source,meta,poses,collected,attempts,target,output,
                              timestamp,*,budget=6):
    """Try fresh depth/subset alternatives; planning earns no harvest credit."""
    rejected=[];evidence=[];minimum=2
    # Reuse only immutable source data inside this one observed-state search.
    # The catalog hash is still verified for every new planning call.
    selection_source=(source_fruit_manifest(source),read(source['policy']))
    for index in range(budget):
        # Before discarding a whole group, reduce only its penetration into
        # the crop. The fruit stays inside the unchanged roller footprint.
        depth=(.050,.025,.015)[index]if index<3 else .025
        if index>=3:minimum=1
        exclusions=rejected if index>=3 else ()
        group,selection=select_group(observation,source,meta,poses,collected,attempts,target,
            output/f'candidate_{index:02d}.json',entry_forward_world=np.array([-1.,0.,0.]),
            depth_m=depth,height_offset_m=0.,minimum_fruits=minimum,rejected_groups=exclusions,max_attempts=3,
            selection_source=selection_source)
        if group is None and minimum==2:
            minimum=1
            group,selection=select_group(observation,source,meta,poses,collected,attempts,target,
                output/f'candidate_{index:02d}_cleanup.json',entry_forward_world=np.array([-1.,0.,0.]),
                depth_m=depth,height_offset_m=0.,minimum_fruits=1,rejected_groups=exclusions,max_attempts=3,
                selection_source=selection_source)
        if group is None:
            evidence.append(dict(candidate=index,minimum_fruits=minimum,reason=selection['reason']));break
        entry=dict(candidate=index,minimum_fruits=minimum,depth_m=depth,fruits=list(group['fruits']),
            aim_world_m=group['group_selection']['belt_aim_world_m'],planning_is_not_a_physical_attempt=True)
        group['coverage_proposal']=dict(candidate=index,depth_m=depth,minimum_fruits=minimum)
        try:
            plan=motion.grip(q,group,timestamp)
        except PlanningError as error:
            entry['reason']=str(error);evidence.append(entry);rejected.append(list(group['fruits']));continue
        entry['planned']=True;evidence.append(entry)
        return plan,group,evidence
    return None,None,evidence
