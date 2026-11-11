"""Opt-in pair/phase stop policies; no collision or force calculation is changed.

The original controller's 1 mm audit always remains authoritative as a recorded
reference diagnostic. This additional audit selects only explicitly identified
belt collision geometries, and supplies a separate experiment stopping event.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import warp as wp
from mujoco_warp._src.support import contact_force_fn
from mujoco_warp._src.types import vec5


DEFAULT_PHASES = ('close', 'cycle_reclose')
OPTIONAL_HELD_PHASES = ('belt_finish',)
OPTIONAL_OPEN_STROKE_PHASES = ('cycle_open', 'cycle_open_final')
OPEN_PHASES = ('cycle_open', 'cycle_open_final')


def validate_config(config):
    """Reject accidental global relaxation and unknown policy fields."""
    allowed = {'default_m', 'belt_fruit_m', 'belt_pedicel_m', 'relaxed_phases',
               'residual_open_seconds', 'explicit_other_pairs', 'record_contacts', 'label'}
    unknown = set(config) - allowed
    if unknown:
        raise ValueError('Unknown contact policy fields: ' + repr(sorted(unknown)))
    result = dict(default_m=.001, belt_fruit_m=.001, belt_pedicel_m=.001,
                  relaxed_phases=list(DEFAULT_PHASES), residual_open_seconds=0.,
                  explicit_other_pairs=[], record_contacts=False,label='independent contact-policy diagnostic')
    result.update(config)
    if not isinstance(result['record_contacts'],bool):
        raise ValueError('record_contacts must be an explicit boolean')
    for name in ('default_m', 'belt_fruit_m', 'belt_pedicel_m'):
        if not np.isfinite(result[name]) or result[name] <= 0:
            raise ValueError('Positive finite threshold required: ' + name)
    if result['default_m'] != .001:
        raise ValueError('This diagnostic keeps all non-belt contacts at reference 1 mm')
    if not np.isfinite(result['residual_open_seconds']) or result['residual_open_seconds'] < 0:
        raise ValueError('Nonnegative finite opening residual duration required')
    phases = result['relaxed_phases']
    if not phases or set(phases) - set(DEFAULT_PHASES + OPTIONAL_HELD_PHASES + OPTIONAL_OPEN_STROKE_PHASES):
        raise ValueError('Only closing and explicit belt_finish/opening-stroke relaxation are implemented')
    entries=result['explicit_other_pairs']
    if not isinstance(entries,list):
        raise ValueError('explicit_other_pairs must be a list of exact geom pairs')
    seen=set()
    for entry in entries:
        if not isinstance(entry,dict) or set(entry)-{'geoms','limit_m','phases','reason'}:
            raise ValueError('Unknown explicit pair policy field')
        geoms=entry.get('geoms')
        if (not isinstance(geoms,list) or len(geoms)!=2 or
                not all(isinstance(x,str) and x for x in geoms) or geoms[0]==geoms[1]):
            raise ValueError('Two distinct exact geometry names required')
        key=tuple(sorted(geoms))
        if key in seen:
            raise ValueError('Duplicate explicit pair policy')
        seen.add(key)
        limit=entry.get('limit_m',0)
        if not np.isfinite(limit) or limit<=0:
            raise ValueError('Positive finite explicit pair threshold required')
        phases=entry.get('phases',['*'])
        if (not isinstance(phases,list) or not phases or
                not all(isinstance(x,str) and x for x in phases) or ('*' in phases and phases!=['*'])):
            raise ValueError('Explicit pair phases must be named phases or singleton [*]')
    return result


def active_harvest_fruit_names(metadata):
    """Exact attachment subset eligible for the active target's belt band.

    Other persistent-window fruit retain the ordinary 1 mm policy. Every
    attachment still participates in dynamics and the existing released-fruit
    exceptions; this scope only assigns belt-policy roles.
    """
    all_names = {entry['name'] for entry in metadata['attachments']}
    if 'active_harvest_attachment_names' not in metadata:
        return all_names
    requested = metadata['active_harvest_attachment_names']
    if (not isinstance(requested, list) or not requested
            or any(not isinstance(name, str) or not name for name in requested)):
        raise ValueError('active_harvest_attachment_names must be a nonempty list of exact native attachment names')
    if len(set(requested)) != len(requested):
        raise ValueError('Duplicate active_harvest_attachment_names')
    unknown = set(requested) - all_names
    if unknown:
        raise ValueError('Unknown active_harvest_attachment_names: ' + repr(sorted(unknown)))
    return set(requested)


def identify_geometries(model, metadata):
    """Identify exact moving belt owners, fruit bodies and thin pedicel bodies.

    CAD belt meshes and the explicitly authored PrototypeTread boxes both have
    prescribed surface velocities in the existing controller. Jaw/top plates
    on Tool_L/Tool_R/Tool_fixed are deliberately outside this set.
    """
    belts = set(metadata['robot']['belt_body_names'])
    if not belts or any(not name.endswith('_belt') for name in belts):
        raise ValueError('Expected exact named moving belt bodies')
    sources = {entry['name']: entry for entry in metadata['robot']['geometry']}
    fruit_bodies = active_harvest_fruit_names(metadata)
    target_trusses={name.split('_Fruit_',1)[0] for name in fruit_bodies if '_Fruit_' in name}
    ids, rows = [], []
    roles = np.zeros(model.ngeom, np.int32)
    for g in range(model.ngeom):
        body = model.body(int(model.geom_bodyid[g])).name
        if not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        if body in belts:
            geom = model.geom(g).name
            record = sources.get(geom)
            expected_owner = '/L_belt/' if body.endswith('L_belt') else '/R_belt/'
            if (record is None or not record.get('collision') or
                    record.get('owner') != body or expected_owner not in record.get('source', '')):
                raise ValueError('Unverified moving-belt collision geometry: '+str(geom))
            ids.append(g)
            rows.append(dict(geom_id=g, geom=geom, body=body, source=record['source']))
        if body in fruit_bodies:
            roles[g] = 1
        elif any(body.startswith(truss+'_Pedicel_') for truss in target_trusses):
            roles[g] = 2
    ideal = metadata.get('geometric_harvest',{}).get('mode') == 'main_stem_only_geometric_harvest'
    # In the user-authorized ideal experiment these contact roles must be
    # absent. Keep the common policy/audit and its ordinary 1mm obstacle guard.
    if ideal and np.any(roles):
        raise ValueError('Ideal harvest model still has active fruit/pedicel collision geometry')
    if not ids or (not ideal and (not np.any(roles == 1) or not np.any(roles == 2))):
        raise ValueError('Incomplete belt/fruit/pedicel identification')
    lookup = np.full(model.ngeom, -1, np.int32)
    lookup[ids] = np.arange(len(ids), dtype=np.int32)
    return lookup, roles, rows


@wp.kernel
def clear_pair_depth(depth: wp.array2d[float]):
    w, p = wp.tid()
    depth[w, p] = 0.0


@wp.kernel
def classify_contacts(tick: wp.array[int], nacon: wp.array[int],
        worlds: wp.array[int], geoms: wp.array[wp.vec2i], distances: wp.array[float],
        addresses: wp.array2d[int], geom_body: wp.array[int], body_fruit: wp.array[int],
        allowed_release: wp.array2d[bool], attached: wp.array2d[bool],
        belt_index: wp.array[int], roles: wp.array[int], ngeom: int,
        explicit_lookup: wp.array[int], explicit_limits: wp.array2d[float],
        pair_depth: wp.array2d[float], class_max: wp.array3d[float],
        default_limit: float, first_event: wp.array[wp.uint64]):
    c = wp.tid()
    if c < nacon[0] and addresses[c, 0] >= 0:
        w = worlds[c]
        pair = geoms[c]
        if pair[0] >= 0 and pair[1] >= 0:
            fruit0 = body_fruit[geom_body[pair[0]]]
            fruit1 = body_fruit[geom_body[pair[1]]]
            exception = bool(False)
            if fruit0 >= 0 and not attached[w, fruit0]:
                exception = allowed_release[fruit0, pair[1]]
            if fruit1 >= 0 and not attached[w, fruit1]:
                exception = exception or allowed_release[fruit1, pair[0]]
            if not exception:
                bi = int(-1)
                other = int(-1)
                if belt_index[pair[0]] >= 0 and roles[pair[1]] > 0:
                    bi = belt_index[pair[0]]
                    other = pair[1]
                elif belt_index[pair[1]] >= 0 and roles[pair[0]] > 0:
                    bi = belt_index[pair[1]]
                    other = pair[0]
                pen = wp.max(0.0, -distances[c])
                category = int(0)
                if bi >= 0:
                    category = roles[other]
                    wp.atomic_max(pair_depth, w, bi*ngeom+other, pen)
                else:
                    limit=default_limit
                    explicit=explicit_lookup[wp.min(pair[0],pair[1])*ngeom+wp.max(pair[0],pair[1])]
                    if explicit>=0:
                        limit=explicit_limits[tick[0],explicit]
                        category=3
                    if pen>limit:
                        key = wp.uint64(wp.min(pair[0], pair[1]))*wp.uint64(65536)+wp.uint64(wp.max(pair[0], pair[1]))
                        wp.atomic_min(first_event, w, (wp.uint64(tick[0]) << wp.uint64(32)) | key)
                wp.atomic_max(class_max, w, tick[0], category, pen)


@wp.kernel
def evaluate_pair_policy(tick: wp.array[int], phase: wp.array[int],
        actual_time: wp.array[float], thresholds: wp.array[float],
        pair_depth: wp.array2d[float], latched: wp.array2d[bool],
        open_start: wp.array2d[float], residual_seconds: float, default_limit: float,
        belt_geoms: wp.array[int], roles: wp.array[int], ngeom: int,
        allowed_steps: wp.array2d[int], residual_steps: wp.array2d[int],
        first_event: wp.array[wp.uint64]):
    w, p = wp.tid()
    other = p % ngeom
    role = roles[other]
    if role > 0:
        t = tick[0]
        pen = pair_depth[w, p]
        limit = default_limit
        residual = bool(False)
        if pen <= default_limit:
            latched[w, p] = False
            open_start[w, p] = -1.0
        elif phase[t] == 1:
            limit = thresholds[role]
            latched[w, p] = pen <= limit
            open_start[w, p] = -1.0
        elif phase[t] == 2 and latched[w, p] and residual_seconds > 0.0:
            if open_start[w, p] < 0.0:
                open_start[w, p] = actual_time[w]
            if actual_time[w]-open_start[w, p] < residual_seconds:
                limit = thresholds[role]
                residual = True
            else:
                latched[w, p] = False
        else:
            latched[w, p] = False
            open_start[w, p] = -1.0
        if pen > default_limit and pen <= limit:
            wp.atomic_add(allowed_steps, w, role, 1)
            if residual:
                wp.atomic_add(residual_steps, w, role, 1)
        if pen > limit:
            belt = belt_geoms[p//ngeom]
            key = wp.uint64(wp.min(belt, other))*wp.uint64(65536)+wp.uint64(wp.max(belt, other))
            wp.atomic_min(first_event, w, (wp.uint64(t) << wp.uint64(32)) | key)


@wp.kernel
def record_contact_witness(tick:wp.array[int],nacon:wp.array[int],njmax:int,cone:int,
        worlds:wp.array[int],geoms:wp.array[wp.vec2i],distances:wp.array[float],
        frames:wp.array[wp.mat33],positions:wp.array[wp.vec3],refs:wp.array[wp.vec2],
        friction:wp.array[vec5],dimensions:wp.array[int],addresses:wp.array2d[int],
        adhesion:wp.array[float],forces:wp.array2d[float],
        geom_xpos:wp.array2d[wp.vec3],geom_xmat:wp.array2d[wp.mat33],
        geom_body:wp.array[int],body_fruit:wp.array[int],attached:wp.array2d[bool],
        pairs_out:wp.array2d[wp.vec2i],distance_out:wp.array2d[float],
        normal_out:wp.array2d[wp.vec3],position_out:wp.array2d[wp.vec3],
        wrench_out:wp.array2d[wp.spatial_vector],ref_out:wp.array2d[wp.vec2],
        delta0_out:wp.array2d[wp.vec3],delta1_out:wp.array2d[wp.vec3],
        attached_out:wp.array2d[wp.vec2i],active_out:wp.array2d[bool]):
    c=wp.tid()
    if c<nacon[0]:
        t=tick[0];w=worlds[c];pair=geoms[c]
        if pair[0]>=0 and pair[1]>=0:
            pairs_out[t,c]=pair
            distance_out[t,c]=distances[c]
            normal_out[t,c]=wp.vec3(frames[c][0,0],frames[c][0,1],frames[c][0,2])
            position_out[t,c]=positions[c]
            active_out[t,c]=addresses[c,0]>=0
            ref_out[t,c]=refs[c]
            wrench_out[t,c]=contact_force_fn(cone,frames,friction,dimensions,addresses,adhesion,
                forces,njmax,nacon,w,c,True)
            delta=geom_xpos[w,pair[1]]-geom_xpos[w,pair[0]]
            delta0_out[t,c]=wp.transpose(geom_xmat[w,pair[0]])*delta
            delta1_out[t,c]=wp.transpose(geom_xmat[w,pair[1]])*(-delta)
            state=wp.vec2i(-1,-1)
            for side in range(2):
                fruit=body_fruit[geom_body[pair[side]]]
                if fruit>=0:
                    state[side]=int(attached[w,fruit])
            attached_out[t,c]=state


class ContactPolicyGPU:
    """Call audit before the original controller's finish_step increments tick."""
    def __init__(self, solver, controller, timeline, config, metadata, worlds=1):
        self.config = validate_config(config)
        self.controller, self.solver = controller, solver
        self.device, self.worlds = controller.device, worlds
        model = solver.mj_model
        lookup, roles, rows = identify_geometries(model, metadata)
        self.rows = rows
        self.ngeom = model.ngeom
        self.steps = controller.steps
        def arr(value, dtype=float):
            return wp.array(np.asarray(value), dtype=dtype, device=self.device)
        self.belt_index, self.roles = arr(lookup, int), arr(roles, int)
        self.belt_geoms = arr([row['geom_id'] for row in rows], int)
        self.thresholds = arr([self.config['default_m'], self.config['belt_fruit_m'],
                               self.config['belt_pedicel_m']])
        phases = np.array([1 if x in self.config['relaxed_phases'] else 2 if x in OPEN_PHASES else 0
                           for x in timeline['phase'][:self.steps]], np.int32)
        self.phase = arr(phases, int)
        self.start_time = arr(timeline['time_edges'][:self.steps])
        explicit_lookup=np.full(model.ngeom*model.ngeom,-1,np.int32)
        entries=self.config['explicit_other_pairs']
        explicit_limits=np.full((self.steps,max(1,len(entries))),self.config['default_m'])
        explicit_rows=[]
        for index,entry in enumerate(entries):
            try:ids=sorted([model.geom(name).id for name in entry['geoms']])
            except KeyError as exc:raise ValueError('Unknown exact policy geometry') from exc
            if any(lookup[g]>=0 for g in ids):
                raise ValueError('explicit_other_pairs must not overlap exact moving belt policies')
            if any(not(model.geom_contype[g] or model.geom_conaffinity[g]) for g in ids):
                raise ValueError('Explicit policy references inactive collision geometry')
            explicit_lookup[ids[0]*model.ngeom+ids[1]]=index
            requested=entry.get('phases',['*'])
            known=set(str(x) for x in timeline['phase'][:self.steps])
            if requested!=['*'] and set(requested)-known:
                raise ValueError('Explicit pair phase absent from command timeline')
            selected=np.ones(self.steps,bool) if requested==['*'] else np.isin(timeline['phase'][:self.steps],requested)
            explicit_limits[selected,index]=entry['limit_m']
            explicit_rows.append(dict(entry,geom_ids=ids,bodies=[model.body(int(model.geom_bodyid[g])).name for g in ids]))
        self.explicit_lookup=arr(explicit_lookup,int)
        self.explicit_limits=arr(explicit_limits)
        shape = (worlds, len(rows)*model.ngeom)
        self.pair_depth = wp.zeros(shape, float, device=self.device)
        self.latched = wp.zeros(shape, bool, device=self.device)
        self.open_start = wp.full(shape, -1., float, device=self.device)
        self.class_max = wp.zeros((worlds,self.steps,4), float, device=self.device)
        self.first_event = arr(np.full(worlds,np.iinfo(np.uint64).max,np.uint64),wp.uint64)
        self.allowed_steps = wp.zeros((worlds,3),int,device=self.device)
        self.residual_steps = wp.zeros_like(self.allowed_steps)
        self.contact_witness=None
        if self.config['record_contacts']:
            if worlds!=1:
                raise ValueError('Per-contact witness currently supports one world only')
            shape=(self.steps,solver.mjw_data.naconmax)
            self.contact_witness=[wp.full(shape,wp.vec2i(-1,-1),wp.vec2i,device=self.device),
                wp.zeros(shape,float,device=self.device),wp.zeros(shape,wp.vec3,device=self.device),
                wp.zeros(shape,wp.vec3,device=self.device),wp.zeros(shape,wp.spatial_vector,device=self.device),
                wp.zeros(shape,wp.vec2,device=self.device),wp.zeros(shape,wp.vec3,device=self.device),
                wp.zeros(shape,wp.vec3,device=self.device),
                wp.full(shape,wp.vec2i(-1,-1),wp.vec2i,device=self.device),
                wp.zeros(shape,bool,device=self.device)]
        self.evidence = dict(schema='s300-contact-stop-policy-v1', config=self.config,
            exact_belt_geometries=rows, role_geom_counts={'fruit':int(np.sum(roles==1)),
            'pedicel':int(np.sum(roles==2))}, physics_changed=False, stopping_policy_changed=True,
            exact_other_pairs=explicit_rows,
            active_harvest_attachment_names=sorted(active_harvest_fruit_names(metadata)),
            target_scope_explicit='active_harvest_attachment_names' in metadata,
            diagnostic_instrumented=self.config['record_contacts'],
            raw_reference_1mm_preserved=True,
            residual_rule='Only exact pairs already above 1 mm during explicitly selected relaxed phases (default: closing only); opening only, cleared on <=1mm/separation/other phase/deadline. Deadline uses actual post-integration backend d.time for each world, not nominal schedule.',
            explicit_open_stroke_rule='Only when named in relaxed_phases, cycle_open/cycle_open_final use the exact belt-pair limit without requiring a prior latch. Fully-open holds, approach and transport remain outside this band.',
            audit_order='pre-integration contact geometry/constraint solution, before controller tick increment; original released-fruit exception map',
            counters_scope='sum of pair-steps, not distinct physics steps', training_eligible=False)

    def audit(self):
        c, d = self.controller, self.solver.mjw_data
        wp.launch(clear_pair_depth,self.pair_depth.shape,inputs=[self.pair_depth],device=self.device)
        wp.launch(classify_contacts,d.naconmax,inputs=[c.tick,d.nacon,d.contact.worldid,d.contact.geom,
            d.contact.dist,d.contact.efc_address,c.geom_body,c.body_fruit,c.allowed_release,c.attached,
            self.belt_index,self.roles,self.ngeom,self.explicit_lookup,self.explicit_limits,self.pair_depth,self.class_max,
            self.config['default_m'],self.first_event],device=self.device)
        wp.launch(evaluate_pair_policy,self.pair_depth.shape,inputs=[c.tick,self.phase,d.time,
            self.thresholds,self.pair_depth,self.latched,self.open_start,self.config['residual_open_seconds'],
            self.config['default_m'],self.belt_geoms,self.roles,self.ngeom,self.allowed_steps,
            self.residual_steps,self.first_event],device=self.device)
        if self.contact_witness is not None:
            wp.launch(record_contact_witness,d.naconmax,inputs=[c.tick,d.nacon,d.njmax,
                int(self.solver.mj_model.opt.cone),d.contact.worldid,d.contact.geom,d.contact.dist,
                d.contact.frame,d.contact.pos,d.contact.solref,d.contact.friction,d.contact.dim,
                d.contact.efc_address,d.contact.adhesion,d.efc.force,d.geom_xpos,d.geom_xmat,
                c.geom_body,c.body_fruit,c.attached,*self.contact_witness],device=self.device)

    def report(self, actual):
        maximum = self.class_max.numpy()[:,:actual]
        return dict(self.evidence, first_policy_event=self.first_event.numpy().tolist(),
            policy_passed=bool(np.all(self.first_event.numpy()==np.iinfo(np.uint64).max)),
            maximum_penetration_m_by_class={name:float(maximum[:,:,i].max(initial=0))
                for i,name in enumerate(('other','belt_fruit','belt_pedicel','explicit_other_pair'))},
            allowed_pair_steps=self.allowed_steps.numpy().tolist(),
            residual_allowed_pair_steps=self.residual_steps.numpy().tolist())

    def save(self, output, actual):
        output = Path(output)
        np.savez_compressed(output/'contact_policy_metrics.npz',
            class_max_penetration_m=self.class_max.numpy()[:,:actual],
            first_policy_event=self.first_event.numpy(), allowed_pair_steps=self.allowed_steps.numpy(),
            residual_allowed_pair_steps=self.residual_steps.numpy())
        (output/'contact_policy.json').write_text(json.dumps(self.report(actual),indent=2)+'\n')
        if self.contact_witness is not None:
            names=('geom_pairs','signed_distance_m','normal_world','contact_position_world_m',
                   'world_wrench_on_second_geom','actual_mixed_solref',
                   'second_geom_center_in_first_geom_frame_m','first_geom_center_in_second_geom_frame_m',
                   'fruit_attached_before_release','constraint_active')
            np.savez_compressed(output/'contact_witness.npz',
                **{name:array.numpy()[:actual] for name,array in zip(names,self.contact_witness)},
                nacon=self.controller.nacon_log.numpy()[:actual],
                actual_step_end_time_s=self.controller.time_log.numpy()[0,:actual])
            model=self.solver.mj_model
            (output/'contact_witness_schema.json').write_text(json.dumps(dict(
                diagnostic_instrumented=True,physics_changed=False,throughput_eligible=False,
                time_scope='Actual post-integration step-end clock; contact geometry/frame/force refer to solve before that integration. Attached state is the controller cached c.attached from pre-integration attachment_torques. release_attachments runs before audit and may already set eq_active=False, but does not update c.attached until the next control callback.',
                force_scope='Actual solved contact wrench; spatial_top force in N, spatial_bottom torque in Nm, world coordinates, acting on second geom (MuJoCo convention).',
                samples_scope='Use slots<nacon[step]; includes inactive positive-gap contacts withzero constraintforce. No physics is rerun for this witness.',
                geoms=[dict(id=g,name=model.geom(g).name,body=model.body(int(model.geom_bodyid[g])).name,
                    type=int(model.geom_type[g]),size=model.geom_size[g].tolist()) for g in range(model.ngeom)]),indent=2)+'\n')
