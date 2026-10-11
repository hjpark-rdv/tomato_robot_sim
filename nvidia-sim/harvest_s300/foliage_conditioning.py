"""Bounded native idle settling of explicitly compliant foliage.

This helper changes neither physics nor operational validity. During the
explicitly invalid initialization interval, it records certain raw leaf-pair
violations before the expensive clearance reader; no clearance is claimed.
A canonical start needs a subsequent uninterrupted strict native second.
Reaction forces may move attached fruit/branches too: all measured velocities
are retained and the final-window motion diagnostics are disclosed.
"""
import hashlib
import math
import re
from pathlib import Path

import numpy as np

SOURCE_BYTES = Path(__file__).read_bytes()
SOURCE_SHA256 = hashlib.sha256(SOURCE_BYTES).hexdigest()
THRESHOLD_M = .0005


def _number(value):
    return not isinstance(value, (bool, np.bool_)) and isinstance(value, (int, float, np.number)) and math.isfinite(float(value))


def _owner(prim):
    from pxr import UsdPhysics
    while prim and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return str(prim.GetPath())
        prim = prim.GetParent()
    return None


def _enabled(prim):
    from pxr import UsdPhysics
    return bool(prim and prim.IsActive() and prim.HasAPI(UsdPhysics.CollisionAPI) and
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True)


def conditioning_inventory(stage, manifest, body_paths):
    """Source-identified enabled leaves and exact eligible counterpart colliders.

    All identities are resolved once from the current stage. Namespace alone
    cannot authorize an unknown mesh. Static plant source names must identify
    the actual leaf/stem/fruit/calyx/rod export; structural scope is limited to
    existing static GutterSuspensionWire Mesh/Capsule colliders. Robot and
    unknown geometry are never admitted.
    """
    from pxr import Usd, UsdGeom, UsdPhysics
    from compliant_foliage import canonical_leaf_group
    from harvest_contact_scope import background_leaf_colliders_from_manifest
    measured = set(body_paths)
    if len(measured) != len(body_paths):
        raise ValueError('Duplicate measured native body identity')
    root = manifest.get('plant', {}).get('root', '').rstrip('/')
    if root != '/World/ElasticTarget':
        raise ValueError('Exact active plant root required')
    leaves = {}; counterparts = {}
    for row in manifest.get('compliant_foliage', {}).get('leaves', []):
        body = row.get('path'); group = row.get('source_name')
        joint = row.get('joint'); j = UsdPhysics.Joint(stage.GetPrimAtPath(joint or ''))
        if (not isinstance(body, str) or body != root + '/Leaf_' + str(group) or
                canonical_leaf_group(group or '') != group or body not in measured or
                not j or [str(x) for x in j.GetBody1Rel().GetTargets()] != [body]):
            raise ValueError('Exact active compliant leaf body/joint declaration mismatch')
        for path in row.get('meshes', []):
            prim = stage.GetPrimAtPath(path)
            if '/Appendage_' not in path:
                continue
            source_group=canonical_leaf_group(prim.GetCustomDataByKey('sourceName') or '') if prim else None
            source_visual=None
            if source_group is None:
                match=re.fullmatch(re.escape(body)+r'/Appendage_(\d+)_\d+',path)
                source_visual=body+'/Visual_'+match.group(1) if match else None
                visual=stage.GetPrimAtPath(source_visual or '')
                if source_visual not in row.get('meshes',[]) or not visual or not visual.IsA(UsdGeom.Mesh):
                    raise ValueError('Exact active compliant leaf visual/collider source mapping mismatch')
                source_group=canonical_leaf_group(visual.GetCustomDataByKey('sourceName') or '')
            if (not _enabled(prim) or not prim.IsA(UsdGeom.Mesh) or _owner(prim) != body or source_group != group):
                raise ValueError('Exact active compliant leaf collider/source mapping mismatch')
            leaves[path] = dict(role='active_compliant_leaf', body=body, source_group=group, joint=joint,source_visual=source_visual)
    for path, proof in background_leaf_colliders_from_manifest(stage, manifest, body_paths).items():
        leaves[path] = dict(role='background_compliant_leaf', **proof)
    if not leaves:
        raise ValueError('No exact measured compliant leaf colliders')
    counterparts.update(leaves)
    known = {row.get('path'): row for row in manifest['plant'].get('bodies', [])}
    known.update({row.get('path'): row for row in manifest['plant'].get('fruits', [])})
    active_name = re.compile(r'(?:Stem_MainStem_\d+|Truss_\d+_(?:Fruit_\d+|Rachis_\d+|Pedicel_(?:proximal|distal)_\d+_\d+|Truss_\d+_Peduncle_\d+))')
    for body, row in known.items():
        name = row.get('name', '')
        if body != root + '/' + name or not active_name.fullmatch(name):
            continue
        path = body + '/Collider'; prim = stage.GetPrimAtPath(path)
        if _enabled(prim) and _owner(prim) == body and body in measured and (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Capsule)):
            counterparts[path] = dict(role='active_plant_fruit' if '_Fruit_' in name else 'active_plant_branch_or_stem', body=body, native_name=name)
    static_path = re.compile(r'/World/Greenhouse/Plants/Plant_\d+/(Stem|Truss_\d+)/mesh_\d+')
    source_rod = re.compile(r'(?:Rachis|Truss_\d+_Peduncle|Pedicel_(?:proximal|distal)_\d+|Empty_Pedicel_\d+)(?:_Trichomes)?')
    source_calyx = re.compile(r'(?:Calyx_\d+_(?:Receptacle|Sepal_\d+)|Calyx_Hub_\d+|Empty_Calyx_\d+_\d+)')
    wire_path = re.compile(r'/World/Greenhouse/House/Environment/GutterSuspensionWire_\d+/(?:Mesh|Collider)')
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        match = static_path.fullmatch(path)
        if not match and not wire_path.fullmatch(path):
            continue
        if not _enabled(prim) or _owner(prim) is not None or not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Capsule)):
            continue
        source = prim.GetCustomDataByKey('sourceName') or ''
        role = None
        if wire_path.fullmatch(path):
            role = 'static_structural_suspension_wire'
        elif match.group(1) == 'Stem':
            if canonical_leaf_group(source): role = 'static_neighbor_leaf'
            elif re.fullmatch(r'MainStem(?:_[0-9a-fA-F]{6})?', source): role = 'static_neighbor_main_stem'
        else:
            if re.fullmatch(r'Fruit_\d+', source): role = 'static_neighbor_fruit'
            elif source_calyx.fullmatch(source): role = 'static_neighbor_calyx'
            elif source_rod.fullmatch(source): role = 'static_neighbor_branch'
        if role:
            counterparts[path] = dict(role=role, source_name=source, static_native_owner=True)
    return leaves, counterparts


class FoliageConditioning:
    """Two-stage pure state machine. Runner keeps all actual native collisions.

    Call begin() at the measured idle start, then before_geometry() once after
    every fresh native fetch. During conditioning its remaining_invalid list
    skips exact recorded leaf violations only. Call after_geometry() with the
    remaining unresolved violations after the unchanged strict reader. The
    report is not a harvest or damage-safety certificate.
    """
    def __init__(self, stage, manifest, body_paths, *, seconds, hz, strict_seconds=1.,
                 mode='idle', callback_complete=True, source_context=None):
        if mode != 'idle' or callback_complete is not True:
            raise ValueError('Conditioning requires idle and complete per-step native callbacks')
        if not _number(seconds) or not 0 < seconds <= 3:
            raise ValueError('Conditioning duration must be >0 and <=3 seconds')
        if not _number(strict_seconds) or strict_seconds < 1:
            raise ValueError('A strict final native window of at least one second is required')
        if not _number(hz) or hz <= 0:
            raise ValueError('Positive finite native Hz required')
        self.seconds=float(seconds); self.hz=float(hz); self.strict_seconds=float(strict_seconds)
        self.body_paths=tuple(body_paths);self.leaves,self.counterparts=conditioning_inventory(stage,manifest,body_paths)
        self.source_context=dict(source_context or {})
        self.initial_time=None; self.last_time=None; self.abort_reason=None; self.initial_breaks=None
        self.pending=None; self.strict_samples=[];self.deferred=[];self.step_records=[]
        self.raw_min_separation_m=None;self.maximum_force_n=0.

    @property
    def end_time_s(self):
        return None if self.initial_time is None else self.initial_time+self.seconds+self.strict_seconds

    def _state(self, poses, velocities):
        p=np.asarray(poses,float);v=np.asarray(velocities,float)
        if p.shape!=(len(self.body_paths),7) or v.shape!=(len(self.body_paths),6):
            raise ValueError('Exact measured body pose/velocity shapes required')
        if not np.isfinite(p).all() or not np.isfinite(v).all():
            raise ValueError('Nonfinite measured body pose/velocity')
        lengths=np.linalg.norm(p[:,3:],axis=1)
        if not np.all(np.abs(lengths-1.)<=1e-5):
            raise ValueError('Measured native body quaternion is not unit XYZW')
        return p.copy(),v.copy()

    def begin(self, time_s, poses, velocities, breaks=()):
        if self.initial_time is not None:raise ValueError('Conditioning cannot restart or reset rest state')
        if not _number(time_s) or time_s<0:raise ValueError('Finite nonnegative measured start timestamp required')
        p,v=self._state(poses,velocities)
        if breaks:
            self.abort_reason='native_break_in_initializer_or_initial_state'
        self.initial_breaks=tuple((event.get('joint'),event.get('t')) for event in breaks)
        self.initial_time=self.last_time=float(time_s);self.initial_poses=p;self.initial_velocities=v
        return self.abort_reason

    def _fatal(self, reason):
        self.abort_reason=self.abort_reason or reason
        return dict(remaining_invalid=[],abort_reason=self.abort_reason,conditioning=False,query_skipped=False)

    def classify(self, event):
        a,b=event.get('a'),event.get('b');t=event.get('t');sep=event.get('separation_m')
        if not _number(t) or not _number(sep):return None
        if a not in self.leaves and b not in self.leaves:return None
        if a not in self.counterparts or b not in self.counterparts:return None
        return dict(a_role=self.counterparts[a]['role'],b_role=self.counterparts[b]['role'])

    def before_geometry(self, time_s, invalid, breaks, poses, velocities):
        """Filter bounded invalid leaf pairs, without producing clearance proof."""
        if self.initial_time is None:raise ValueError('Call begin before the first native step')
        if self.abort_reason:return self._fatal(self.abort_reason)
        if self.pending is not None:return self._fatal('missing_strict_after_geometry_step')
        if not _number(time_s):return self._fatal('nonfinite_native_timestamp')
        dt=float(time_s)-self.last_time
        tolerance=max(1e-10,1/self.hz*1e-6)
        if abs(dt-1/self.hz)>tolerance:return self._fatal('missing_or_duplicate_native_timestamp')
        try:p,v=self._state(poses,velocities)
        except ValueError as error:return self._fatal(str(error))
        identities=tuple((event.get('joint'),event.get('t')) for event in breaks)
        if identities!=self.initial_breaks:return self._fatal('new_native_break_during_idle_conditioning')
        t=float(time_s); elapsed=t-self.initial_time; conditioning=elapsed<self.seconds-tolerance
        remaining=[];deferred=0
        for event in invalid:
            if (not _number(event.get('t')) or abs(float(event['t'])-t)>tolerance or
                    not _number(event.get('separation_m'))):
                return self._fatal('missing_stale_or_nonfinite_contact_evidence')
            sep=float(event['separation_m'])
            if sep>=-THRESHOLD_M:raise ValueError('Only existing strict 0.5mm violations belong in invalid')
            self.raw_min_separation_m=sep if self.raw_min_separation_m is None else min(self.raw_min_separation_m,sep)
            force=event.get('force_N',event.get('force_n',0.))
            if not _number(force):return self._fatal('nonfinite_native_contact_force')
            self.maximum_force_n=max(self.maximum_force_n,float(force))
            proof=self.classify(event) if conditioning else None
            if proof:
                self.deferred.append(dict(event,conditioning_pair=proof,
                    query_skipped_reason='bounded invalid conditioning, no clearance certificate',
                    operational_physics_valid=False,clearance_certificate=False))
                deferred+=1
            else:remaining.append(event)
        self.last_time=t;self.pending=(t,p,v,conditioning)
        self.step_records.append(dict(t=t,conditioning=conditioning,raw_invalid_count=len(invalid),
                                      deferred_count=deferred,strict_reader_required=not conditioning or bool(remaining)))
        # Even a potential raw callback ghost outside exact foliage scope is
        # conservative-fatal during conditioning; no policy broadening occurs.
        if conditioning and remaining:self.abort_reason='nonleaf_or_undeclared_collision_during_conditioning'
        return dict(remaining_invalid=remaining,abort_reason=self.abort_reason,
                    conditioning=conditioning,query_skipped=bool(deferred))

    def after_geometry(self, time_s, unresolved_invalid):
        """Record existing strict reader outcome; unresolved terminal pairs fail."""
        if self.abort_reason:return self.abort_reason
        if self.pending is None or float(time_s)!=self.pending[0]:
            self.abort_reason='missing_or_mismatched_after_geometry_timestamp';return self.abort_reason
        t,p,v,conditioning=self.pending;self.pending=None
        if unresolved_invalid:
            self.abort_reason='strict_penetration_over_0_5mm';return self.abort_reason
        self.step_records[-1].update(strict_reader_passed=True,unresolved_invalid_count=0)
        if not conditioning:self.strict_samples.append((t,p,v))
        return None

    def _stability(self):
        if len(self.strict_samples)<3:return dict(complete=False,no_increasing_drift=False,bodies=[])
        times=np.array([x[0] for x in self.strict_samples]);p=np.array([x[1] for x in self.strict_samples]);v=np.array([x[2] for x in self.strict_samples])
        midpoint=(times[0]+times[-1])/2.;middle=int(np.searchsorted(times,midpoint))
        if middle==0 or middle==len(times)-1:return dict(complete=False,no_increasing_drift=False,bodies=[])
        linear=np.linalg.norm(np.diff(p[:,:,:3],axis=0),axis=2)
        quaternions=p[:,:,3:]/np.linalg.norm(p[:,:,3:],axis=2,keepdims=True)
        dots=np.abs(np.sum(quaternions[:-1]*quaternions[1:],axis=2));angular=2*np.arccos(np.clip(dots,-1.,1.))
        dt1=times[middle]-times[0];dt2=times[-1]-times[middle]
        rates1=linear[:middle].sum(0)/dt1;rates2=linear[middle:].sum(0)/dt2
        omega1=angular[:middle].sum(0)/dt1;omega2=angular[middle:].sum(0)/dt2
        # Numerical guards, not biological motion thresholds. The measured
        # rates and all absolute velocities remain exposed, even for a pass.
        translation_guard=1e-9;rotation_guard=1e-7
        rows=[]
        for i,path in enumerate(self.body_paths):
            stable=bool(rates2[i]<=rates1[i]+translation_guard and omega2[i]<=omega1[i]+rotation_guard)
            rows.append(dict(path=path,first_half_translation_rate_m_s=float(rates1[i]),
                second_half_translation_rate_m_s=float(rates2[i]),first_half_rotation_rate_rad_s=float(omega1[i]),
                second_half_rotation_rate_rad_s=float(omega2[i]),no_increasing_drift=stable,
                terminal_linear_velocity_m_s=v[-1,i,:3].tolist(),terminal_angular_velocity_rad_s=v[-1,i,3:].tolist(),
                maximum_linear_speed_m_s=float(np.linalg.norm(v[:,i,:3],axis=1).max()),
                maximum_angular_speed_rad_s=float(np.linalg.norm(v[:,i,3:],axis=1).max()),
                terminal_position_delta_from_initial_m=(p[-1,i,:3]-self.initial_poses[i,:3]).tolist()))
        return dict(complete=True,no_increasing_drift=all(x['no_increasing_drift'] for x in rows),
            criterion='second-half measured pose total-variation rates do not exceed first half; no calibrated equilibrium/safe-motion threshold claimed',
            numerical_translation_rate_guard_m_s=translation_guard,numerical_rotation_rate_guard_rad_s=rotation_guard,bodies=rows)

    def report(self):
        times=np.array([x[0] for x in self.strict_samples]);duration=float(times[-1]-times[0]) if len(times)>1 else 0.
        maximum_gap=float(np.diff(times).max()) if len(times)>1 else None
        tol=max(1e-10,1/self.hz*1e-6)
        complete=bool(len(times)>1 and duration>=self.strict_seconds-tol and maximum_gap<=1/self.hz+tol and self.pending is None)
        stability=self._stability()
        accepted=bool(complete and self.abort_reason is None and stability['no_increasing_drift'])
        return dict(schema='native_foliage_conditioning_v1',canonical_conditioned_start=accepted,
            conditioning_seconds=self.seconds,strict_required_seconds=self.strict_seconds,native_hz=self.hz,
            strict_penetration_threshold_m=THRESHOLD_M,initial_time_s=self.initial_time,final_time_s=self.last_time,
            strict_window=dict(start_time_s=float(times[0]) if len(times) else None,end_time_s=float(times[-1]) if len(times) else None,
                duration_s=duration,sample_count=len(times),maximum_gap_s=maximum_gap,complete=complete),
            source_context=self.source_context,helper_source_sha256=SOURCE_SHA256,abort_reason=self.abort_reason,
            exact_compliant_leaf_colliders=self.leaves,exact_counterpart_colliders=self.counterparts,
            deferred_raw_events=self.deferred,native_step_records=self.step_records,stability=stability,
            maximum_raw_force_n=self.maximum_force_n,minimum_raw_separation_m=self.raw_min_separation_m,
            initial_native_break_count=len(self.initial_breaks or ()),new_native_break_count=0 if self.abort_reason!='new_native_break_during_idle_conditioning' else None,
            no_new_native_breaks=self.abort_reason not in ('native_break_in_initializer_or_initial_state','new_native_break_during_idle_conditioning'),
            finite_state=self.abort_reason not in ('Nonfinite measured body pose/velocity','nonfinite_native_timestamp','missing_stale_or_nonfinite_contact_evidence','nonfinite_native_contact_force'),
            no_robot_or_nonleaf_exemptions=all((e['a'] in self.leaves or e['b'] in self.leaves) and
                e['a'] in self.counterparts and e['b'] in self.counterparts for e in self.deferred),
            raw_conditioning_operational_validity=False,operational_strict_window_verified=complete and self.abort_reason is None,
            no_physics_or_contact_policy_changes=True,completed_cycle=False,training_eligible=False,harvest_success=None,
            interpretation='Bounded initialization experiment; conditioning is not harvest, collection, damage safety, or a geometric clearance certificate. Measured velocities must be checkpointed without reset.')


def save_imported_source(output):
    path=Path(output)/'runtime_code_sources'/'foliage_conditioning.py';path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists() and path.read_bytes()!=SOURCE_BYTES:raise ValueError('Refusing to overwrite imported conditioning source evidence')
    if not path.exists():path.write_bytes(SOURCE_BYTES)
    return dict(path=str(path),sha256=SOURCE_SHA256)


def validate_report(report):
    """Validate canonical evidence structure; sidecar/source SHA binding is separate.

    Failed/incomplete initialization can be saved but cannot be approved by this
    predicate. A checkpoint must retain actual measured q, controller targets,
    velocities and source files; this helper does not replace those checks.
    """
    if (report.get('schema')!='native_foliage_conditioning_v1' or
            report.get('canonical_conditioned_start') is not True or report.get('abort_reason') is not None or
            report.get('completed_cycle') is not False or report.get('training_eligible') is not False or
            report.get('harvest_success') is not None):
        raise ValueError('Canonical conditioning evidence is failed/incomplete or wrongly labeled harvest')
    for flag in ('no_new_native_breaks','finite_state','no_robot_or_nonleaf_exemptions',
                 'operational_strict_window_verified','no_physics_or_contact_policy_changes'):
        if report.get(flag) is not True:raise ValueError('Missing conditioning validation flag: '+flag)
    if report.get('initial_native_break_count')!=0 or report.get('new_native_break_count')!=0:
        raise ValueError('Native initialization breaks cannot be conditioning/harvest credit')
    duration=report.get('conditioning_seconds');strict=report.get('strict_required_seconds');hz=report.get('native_hz')
    if not _number(duration) or not 0<duration<=3 or not _number(strict) or strict<1 or not _number(hz) or hz<=0:
        raise ValueError('Invalid bounded conditioning/strict duration or native Hz')
    if report.get('strict_penetration_threshold_m')!=THRESHOLD_M:
        raise ValueError('Existing strict 0.5mm native criterion changed')
    w=report.get('strict_window',{});tol=max(1e-10,1/hz*1e-6)
    if (w.get('complete') is not True or not all(_number(w.get(k)) for k in ('start_time_s','end_time_s','duration_s','maximum_gap_s')) or
            w['duration_s']<strict-tol or abs(w['end_time_s']-w['start_time_s']-w['duration_s'])>tol or
            w['maximum_gap_s']>1/hz+tol):
        raise ValueError('Strict native window is incomplete or has a timestamp gap')
    steps=[x for x in report.get('native_step_records',[]) if not x.get('conditioning')]
    if len(steps)!=w.get('sample_count') or len(steps)<2:
        raise ValueError('Strict native per-step evidence count mismatch')
    times=np.array([x.get('t',float('nan')) for x in steps],float)
    if (not np.isfinite(times).all() or abs(times[0]-w['start_time_s'])>tol or abs(times[-1]-w['end_time_s'])>tol or
            np.any(np.abs(np.diff(times)-1/hz)>tol) or
            any(x.get('strict_reader_passed') is not True or x.get('deferred_count')!=0 or
                x.get('unresolved_invalid_count')!=0 for x in steps)):
        raise ValueError('Strict window has omitted/invalid/waived native steps')
    stability=report.get('stability',{})
    if stability.get('complete') is not True or stability.get('no_increasing_drift') is not True or not stability.get('bodies'):
        raise ValueError('Final measured stationarity diagnostic did not pass')
    for body in stability['bodies']:
        if body.get('no_increasing_drift') is not True:raise ValueError('Increasing measured terminal drift')
        for key in ('first_half_translation_rate_m_s','second_half_translation_rate_m_s',
                    'first_half_rotation_rate_rad_s','second_half_rotation_rate_rad_s',
                    'maximum_linear_speed_m_s','maximum_angular_speed_rad_s'):
            if not _number(body.get(key)):raise ValueError('Nonfinite measured motion diagnostics')
    leaves=report.get('exact_compliant_leaf_colliders',{});counterparts=report.get('exact_counterpart_colliders',{})
    if not leaves or not set(leaves)<=set(counterparts):raise ValueError('Missing exact compliant leaf inventory')
    start=report.get('initial_time_s');end=report.get('final_time_s')
    if not _number(start) or not _number(end) or w['start_time_s']<start+duration-tol or end<w['end_time_s']-tol:
        raise ValueError('Invalid conditioning/strict timeline')
    for event in report.get('deferred_raw_events',[]):
        a,b=event.get('a'),event.get('b')
        if ((a not in leaves and b not in leaves) or a not in counterparts or b not in counterparts or
                any('/HarvestRobot/' in str(p) or '/ActuatedTool/' in str(p) for p in (a,b)) or
                not _number(event.get('t')) or not start<event['t']<start+duration-tol or
                event.get('clearance_certificate') is not False or event.get('operational_physics_valid') is not False or
                event.get('query_skipped_reason')!='bounded invalid conditioning, no clearance certificate'):
            raise ValueError('Robot/nonleaf/unmapped or terminal strict event was exempted')
    return dict(schema='validated_native_foliage_conditioning_v1',canonical_conditioned_start=True,
                strict_duration_s=w['duration_s'],native_step_count=len(steps),
                source_context=report.get('source_context',{}),helper_source_sha256=report.get('helper_source_sha256'),
                harvest_success=None,training_eligible=False,completed_cycle=False)
