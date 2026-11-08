"""Opt-in numerical overlap budget for exact tool/fruit pairs, not material softness."""
import math
import re
import copy

from motion_contact_buffer import ACTIVE_PATH_MARKERS


class TPUBeltIndentationDiagnostic:
    """Uncalibrated 2mm TPU indentation proxy during closing only.

    This is a post-filter of the unchanged raw 1mm guard, never its cached
    pair_limit callback. Exact native contacts, forces and release mechanics
    remain unchanged. It does not implement deformable belt physics.
    """
    PHASES=frozenset(('close','cycle_reclose'))  # Existing Newton DEFAULT_PHASES.

    def __init__(self, manifest, stage):
        from pxr import Usd,UsdGeom,UsdPhysics
        from passive_collection_surfaces import _geometry_identity
        if stage is None:raise ValueError('TPU indentation diagnostic requires the exact current USD stage')
        self.belts={};self.fruits={};self.records=[]
        for side in ('L','R'):
            body='/World/ActuatedTool/'+side+'_belt';owner=stage.GetPrimAtPath(body)
            if not owner or not owner.HasAPI(UsdPhysics.RigidBodyAPI):raise ValueError('Missing exact native TPU belt owner: '+body)
            schemas=owner.GetMetadata('apiSchemas')
            if (schemas is None or 'PhysxSurfaceVelocityAPI' not in schemas.GetAppliedItems() or
                    owner.GetAttribute('physxSurfaceVelocity:surfaceVelocityLocalSpace').Get() is not True):
                raise ValueError('Exact TPU belt owner lacks the existing local surface-velocity API')
            matches=[p for p in Usd.PrimRange(owner) if p.GetCustomDataByKey('sourceName')==side+'_C07_contact_belt_TPU']
            if len(matches)!=1:raise ValueError('Exact TPU source-name inventory mismatch: '+side)
            prim=matches[0];path=str(prim.GetPath());rigid=prim
            while rigid and not rigid.IsPseudoRoot() and not rigid.HasAPI(UsdPhysics.RigidBodyAPI):rigid=rigid.GetParent()
            if (not re.fullmatch(re.escape(body)+r'/part_\d+',path) or not prim.IsA(UsdGeom.Mesh) or
                    not rigid or str(rigid.GetPath())!=body or not prim.HasAPI(UsdPhysics.CollisionAPI) or
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True):
                raise ValueError('Exact enabled TPU collider/rigid-owner mismatch: '+path)
            digest,counts=_geometry_identity(UsdGeom.Mesh(prim))
            self.belts[path]=dict(body=body,source_name=side+'_C07_contact_belt_TPU',
                authored_geometry_sha256=digest,source_vertices=counts[0],source_triangles=counts[1])
        root=manifest['plant']['root'].rstrip('/')
        for fruit in manifest['plant']['fruits']:
            name=fruit['name'];body=fruit['path'];joint=fruit['joint'];path=body+'/Collider'
            prim=stage.GetPrimAtPath(path);rigid=prim
            while rigid and not rigid.IsPseudoRoot() and not rigid.HasAPI(UsdPhysics.RigidBodyAPI):rigid=rigid.GetParent()
            if (not re.fullmatch(r'Truss_\d+_Fruit_\d+',name) or body!=root+'/'+name or
                    joint!=root+'/HarvestJoints/'+name or path in self.fruits or
                    not prim or not prim.IsA(UsdGeom.Mesh) or not rigid or str(rigid.GetPath())!=body or
                    not prim.HasAPI(UsdPhysics.CollisionAPI) or
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True):
                raise ValueError('Exact active-plant TPU/fruit inventory mismatch: '+path)
            self.fruits[path]=dict(body=body,joint=joint)
        if not self.fruits:raise ValueError('TPU indentation diagnostic requires exact active-plant fruit')

    def filter_invalid(self, events, breaks):
        remaining=[]
        for event in events:
            left,right=event['a'],event['b'];phase=event.get('phase');t=event.get('t');d=event.get('separation_m')
            belt,fruit=(left,right) if left in self.belts else (right,left)
            allowed=(belt in self.belts and fruit in self.fruits and phase in self.PHASES and
                     isinstance(t,(int,float)) and math.isfinite(t) and t>=0 and
                     isinstance(d,(int,float)) and math.isfinite(d) and -.002<=d<-.001)
            if not allowed:remaining.append(event);continue
            joint=self.fruits[fruit]['joint']
            release_times=[b['t'] for b in breaks if b.get('joint')==joint and
                isinstance(b.get('t'),(int,float)) and math.isfinite(b['t']) and 0<=b['t']<=t]
            self.records.append(dict(raw_event=copy.deepcopy(event),belt_collider=belt,fruit_collider=fruit,
                native_joint=joint,native_release_verified=bool(release_times),
                native_break_t=min(release_times) if release_times else None,
                raw_penetration_m=-d,reference_limit_m=.001,experimental_limit_m=.002,
                scope='experimental_TPU_fruit_closing_indentation_proxy',
                reference_1mm_violation_retained=True,continuation_authorized=True,
                elastic_belt_physics_implemented=False,physical_collision_response_changed=False,
                physical_accuracy_verified=False,training_eligible=False))
        return remaining

    def metadata(self):
        return dict(enabled=True,default_enabled=False,reference_limit_m=.001,experimental_limit_m=.002,
            relaxed_phases=sorted(self.PHASES),exact_TPU_belt_colliders=self.belts,
            exact_active_plant_fruit_colliders=self.fruits,prototype_treads_included=False,
            interpretation='Uncalibrated TPU indentation proxy; no deformable belt model or damage certification',
            application='After unchanged raw guard, leaf geometry and released-fruit lifecycle; never cached pair limit',
            all_other_pairs_and_phases_limit_m=.001,raw_contact_generation_and_response_changed=False,
            physical_accuracy_verified=False,training_eligible=False,
            deferred_reference_1mm_violations=len(self.records),records=self.records)


class FruitContactGuard:
    def __init__(self, manifest, tool_colliders, *, fruit_limit_m=.0005, fruit_fruit_limit_m=.0005, default_limit_m=.0005):
        if any(not math.isfinite(v) or v < .0005 for v in (fruit_limit_m,fruit_fruit_limit_m,default_limit_m)):
            raise ValueError('Fruit contact limit must be finite and at least 0.5mm')
        match = re.fullmatch(r'(Truss_\d+)_Fruit_\d+', manifest['target']['name'])
        if not match:
            raise ValueError('An exact target truss fruit is required')
        root = manifest['plant']['root'].rstrip('/')
        prefix = match[1] + '_Fruit_'
        self.fruits = frozenset(f['path'] + '/Collider' for f in manifest['plant']['fruits']
                                if f['name'].startswith(prefix) and
                                f['path'] == root + '/' + f['name'])
        self.tools = frozenset(tool_colliders)
        if not self.fruits or manifest['target']['path'] + '/Collider' not in self.fruits:
            raise ValueError('Target fruit collider is missing from the source inventory')
        if not self.tools or any(not p.startswith('/World/ActuatedTool/') for p in self.tools):
            raise ValueError('Only exact actuated-tool collider paths are allowed')
        self.default_limit_m = default_limit_m
        self.fruit_limit_m = fruit_limit_m
        self.fruit_fruit_limit_m = fruit_fruit_limit_m

    def limit(self, left, right):
        if left!=right and left in self.fruits and right in self.fruits:
            return self.fruit_fruit_limit_m
        if ((left in self.fruits and right in self.tools) or
                (right in self.fruits and left in self.tools)):
            return self.fruit_limit_m
        return self.default_limit_m

    def invalid(self, event):
        return (any(m in event['a'] or m in event['b'] for m in ACTIVE_PATH_MARKERS)
                and event['separation_m'] < -self.limit(event['a'], event['b']))

    def metadata(self):
        return dict(default_limit_mm=self.default_limit_m*1000, tool_current_truss_fruit_limit_mm=self.fruit_limit_m*1000,
                    current_truss_fruit_fruit_limit_mm=self.fruit_fruit_limit_m*1000,
                    fruit_colliders=sorted(self.fruits), tool_colliders=sorted(self.tools),
                    initialization_limit_mm=self.default_limit_m*1000,
                    scope='Exact current-truss fruit/tool and optional same-truss fruit/fruit budgets; all other active pairs use the recorded common threshold',
                    interpretation='Numerical overlap tolerance only. Native contacts and all raw evidence remain. Not calibrated fruit compression.')

    def summarize(self, events, hz):
        groups = {}
        violations = 0
        for e in events:
            if not any(m in e['a'] or m in e['b'] for m in ACTIVE_PATH_MARKERS):
                continue
            violations += self.invalid(e)
            if e['separation_m'] >= -.0005:
                continue
            key = tuple(sorted((e['a'], e['b'])))
            row = groups.setdefault(key, dict(a=key[0], b=key[1], times=set(),
                max_penetration_mm=0., max_point_force_n=0., contact_points=0))
            row['times'].add(float(e['t']))
            row['max_penetration_mm'] = max(row['max_penetration_mm'], -e['separation_m']*1000)
            row['max_point_force_n'] = max(row['max_point_force_n'], e['force_n'])
            row['contact_points'] += 1
        rows = []
        for row in groups.values():
            times = sorted(row.pop('times')); longest = run = 1
            for left, right in zip(times, times[1:]):
                run = run+1 if right-left <= 1.5/hz else 1
                longest = max(longest, run)
            row.update(first_t=times[0], last_t=times[-1], physics_samples=len(times),
                sampled_exceedance_s=len(times)/hz, longest_sampled_exceedance_s=longest/hz,
                configured_limit_mm=1000*self.limit(row['a'], row['b']))
            rows.append(row)
        return dict(**self.metadata(), strict_0_5mm_passed=not rows,
                    configured_violation_contact_points=violations,
                    exceedance_pairs=rows,
                    duration_scope='Distinct physics timestamps per pair times dt; gaps break continuity; no swept-duration claim')
