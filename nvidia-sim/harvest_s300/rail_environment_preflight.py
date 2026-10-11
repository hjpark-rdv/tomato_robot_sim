"""Opt-in rail transit screen using existing S300 FK/FCL and motion sampling.

Only the initial rail_move is screened. The farm is frozen at its immutable
source geometry, with exact saved owner poses when available. Native contact,
deformation, tracking and the unchanged 0.5mm guard remain separate evidence.
Whole CAD-part envelopes are conservative; a blocked proposal is not impossible.
"""
import argparse
import copy
import hashlib
import json
import re
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics

from compact_motion_assets import file_sha256, verify_compact_dependencies
from compliant_foliage import canonical_leaf_group
from motion_self_collision import MotionSelfCollision, check_plan
from motion_continuation import initial_plan_rail_position
from rail_assets import enabled_colliders
from leaf_contact_geometry import triangle_components, point_inside_closed_triangles


MOVING_ROOTS = ('/World/HarvestRobot/Robot', '/World/ActuatedTool')
SOURCE_BYTES = Path(__file__).read_bytes()
SOURCE_SHA256 = hashlib.sha256(SOURCE_BYTES).hexdigest()


def below(path, root):
    return path == root or path.startswith(root + '/')


def matrix(pose):
    pose = np.asarray(pose, float)
    if pose.shape != (7,) or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1) > 1e-4:
        raise ValueError('Finite measured owner position and unit XYZW quaternion required')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def allowed_truss(manifest, plan, explicit=None):
    from row_harvest_targets import truss_key
    known = {truss_key(f['name']) for f in manifest['plant']['fruits']}
    key = explicit or plan.get('truss_axis', {}).get('truss_key') or truss_key(manifest['target']['name'])
    if key not in known:
        raise ValueError('Allowed truss must be an exact current active-plant truss')
    return key


def obstacle_class(prim, manifest, truss):
    path = str(prim.GetPath())
    name = prim.GetCustomDataByKey('sourceName') or ''
    if canonical_leaf_group(name):
        return 'soft_leaf', False
    root = manifest['plant']['root'].rstrip('/')
    # Permissions cover only exact own-truss fruits/branches, never main stem
    # or a coincident foreign truss name in a different source plant.
    if re.fullmatch(re.escape(root) + '/' + re.escape(truss) +
                    r'_(?:Fruit_\d+|(?:Rachis|Pedicel|Peduncle)_[^/]+)(?:/[^/]+)*', path):
        return 'current_allowed_truss', False
    text = (path + '/' + name).lower()
    for token, label in (('groundcover', 'structural_ground_support'), ('gutter', 'gutter'),
                         ('rail', 'structural_rail_support'), ('mainstem', 'main_stem'),
                         ('main_stem', 'main_stem'), ('rachis', 'foreign_rachis'),
                         ('pedicel', 'foreign_pedicel'), ('peduncle', 'foreign_peduncle'),
                         ('fruit', 'foreign_fruit')):
        if token in text:
            return label, True
    return 'structural_or_unknown_obstacle', True


def preparation_leaf_class(stage, prim, owner, manifest):
    """A leaf label alone cannot turn a physically static obstacle flexible."""
    if owner is None:
        return 'static_leaf_obstacle', True
    body = stage.GetPrimAtPath(owner)
    api = UsdPhysics.RigidBodyAPI(body)
    if (not api or api.GetRigidBodyEnabledAttr().Get() is not True or
            api.GetKinematicEnabledAttr().Get() is True):
        return 'static_or_kinematic_leaf_obstacle', True
    declared = [row for key in ('compliant_foliage', 'background_compliant_foliage')
                for row in manifest.get(key, {}).get('leaves', []) if row.get('path') == owner]
    if len(declared) != 1:
        return 'unverified_dynamic_leaf_obstacle', True
    row = declared[0]; joint = stage.GetPrimAtPath(row.get('joint', ''))
    # Existing leaf converters author general D6 joints, not fixed joints.
    if (not joint or not joint.IsA(UsdPhysics.Joint) or joint.IsA(UsdPhysics.FixedJoint) or
            list(map(str, UsdPhysics.Joint(joint).GetBody1Rel().GetTargets())) != [owner] or
            row.get('stiffness', 0.) <= 0 or str(prim.GetPath()) not in row.get('meshes', [])):
        return 'unverified_dynamic_leaf_obstacle', True
    return 'verified_compliant_leaf_diagnostic', False


def scoped_plan(plan, scope):
    """Select an existing prefix; never alter its positions, gaps or timing."""
    if scope not in ('rail', 'preparation'):
        raise ValueError('Environment scope must be rail or preparation')
    phases = plan.get('phases', [])
    if scope == 'rail':
        indices = [i for i, p in enumerate(phases) if p.get('name') == 'rail_move']
        if not indices:
            return None
        if indices != [0]:
            raise ValueError('Only one initial rail_move is supported')
        selected = phases[:1]
    else:
        indices = [i for i, p in enumerate(phases) if p.get('name') == 'approach']
        if not indices:
            raise ValueError('Preparation scope requires the first approach boundary')
        selected = phases[:indices[0]]
        if not selected:
            return None
    result = copy.deepcopy(plan); result['phases'] = copy.deepcopy(selected)
    if scope == 'preparation':
        # Supply explicit constant rail coordinates for phases which inherited
        # the rail position. This preserves check_plan's original semantics
        # while allowing the same environment callback for non-travel prefixes.
        previous = initial_plan_rail_position(result)
        for phase in result['phases']:
            phase.setdefault('rail_waypoint_positions_m', [previous]*len(phase['waypoint_q']))
            previous = phase['rail_waypoint_positions_m'][-1]
    return result


class ExactFilters:
    """Lazy exact ancestor filteredPairs and USD collection group membership."""
    def __init__(self, stage):
        self.rules = {}; self.groups = {}; self.cache = {}
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            attr = prim.GetAttribute('physxScene:invertCollisionGroupFilter')
            if attr and attr.Get() is True:
                raise ValueError('Inverted collision-group policy is unsupported by this adapter')
            rel = prim.GetRelationship('physics:filteredPairs')
            if rel and prim.HasAPI(UsdPhysics.FilteredPairsAPI):
                self.rules[str(prim.GetPath())] = tuple(map(str, rel.GetTargets()))
            if prim.IsA(UsdPhysics.CollisionGroup):
                group = UsdPhysics.CollisionGroup(prim)
                inverted = prim.GetAttribute('physics:invertFilteredGroups')
                merged = prim.GetAttribute('physics:mergeGroupName')
                if (inverted and inverted.Get() is True) or (merged and merged.Get()):
                    raise ValueError('Inverted/merged collision groups need a separate verified adapter')
                self.groups[str(prim.GetPath())] = (
                    group.GetCollidersCollectionAPI().ComputeMembershipQuery(),
                    set(map(str, group.GetFilteredGroupsRel().GetTargets())))

    def excluded(self, a, b):
        key = tuple(sorted((a, b)))
        if key in self.cache:
            return self.cache[key]
        filtered = any((below(a, root) and any(below(b, t) for t in targets)) or
                       (below(b, root) and any(below(a, t) for t in targets))
                       for root, targets in self.rules.items())
        left = {p for p, (q, _) in self.groups.items() if q.IsPathIncluded(a)}
        right = {p for p, (q, _) in self.groups.items() if q.IsPathIncluded(b)}
        filtered |= any(other in self.groups[p][1] for p in left for other in right)
        filtered |= any(other in self.groups[p][1] for p in right for other in left)
        self.cache[key] = bool(filtered)
        return bool(filtered)


def collider_geometry(prim, transform):
    """Same exact source primitive/triangle convention as scan_external_rest.

    Dynamic convexDecomposition uses the conservative whole source-part hull;
    no claim is made that native cooked convex pieces were reconstructed.
    """
    import fcl
    import trimesh
    t = np.asarray(transform, float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError('Finite source collider transform required')
    kind = prim.GetTypeName(); scale = np.linalg.norm(t[:3, :3], axis=0)
    if np.any(scale <= 0):
        raise ValueError('Degenerate collider scale')
    rotation = t[:3, :3] / scale
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0):
        raise ValueError('Sheared collider transform unsupported')
    triangles = None; components = []
    if kind == 'Mesh':
        mesh = UsdGeom.Mesh(prim); v = np.asarray(mesh.GetPointsAttr().Get(), float)
        v = v @ t[:3, :3].T + t[:3, 3]
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), int)
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), int)
        faces = []; offset = 0
        for count in counts:
            if count < 3: raise ValueError('Degenerate source polygon')
            face = indices[offset:offset+count]
            faces.extend((face[0], face[i], face[i+1]) for i in range(1, count-1)); offset += count
        if offset != len(indices) or not len(v) or not faces or not np.isfinite(v).all():
            raise ValueError('Invalid complete source mesh')
        faces = np.asarray(faces, np.int32)
        if np.any(faces < 0) or np.any(faces >= len(v)):
            raise ValueError('Source mesh index outside vertex array')
        approximation = prim.GetAttribute('physics:approximation').Get()
        if approximation in ('convexHull', 'convexDecomposition'):
            hull = trimesh.convex.convex_hull(v)
            geometry = fcl.Convex(hull.vertices, len(hull.faces),
                                 np.column_stack((np.full(len(hull.faces), 3), hull.faces)).reshape(-1))
            representation = 'source_part_convex_envelope_native_cooking_unverified'
        elif approximation in (None, 'none'):
            geometry = fcl.BVHModel(); geometry.beginModel(len(v), len(faces))
            geometry.addSubModel(v, faces); geometry.endModel()
            triangles = v[faces]; components = triangle_components(v, faces)
            representation = 'exact_source_triangle_surface_and_closed_volume'
        else:
            raise ValueError('Unsupported native mesh approximation: ' + str(approximation))
        obj = fcl.CollisionObject(geometry, fcl.Transform())
        bounds = np.array([v.min(0), v.max(0)])
    else:
        if kind == 'Cube':
            size = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) * scale
            if not np.isfinite(size).all() or np.any(size <= 0): raise ValueError('Invalid cube size')
            geometry = fcl.Box(*size); extent = abs(rotation) @ (size / 2)
        elif kind == 'Sphere':
            if not np.allclose(scale, scale[0]): raise ValueError('Nonuniform source sphere')
            radius = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get()) * scale[0]
            if not np.isfinite(radius) or radius <= 0: raise ValueError('Invalid sphere radius')
            geometry = fcl.Sphere(radius); extent = np.full(3, radius)
        elif kind in ('Capsule', 'Cylinder'):
            shape = UsdGeom.Capsule(prim) if kind == 'Capsule' else UsdGeom.Cylinder(prim)
            axis = 'XYZ'.index(str(shape.GetAxisAttr().Get())); radial = np.delete(scale, axis)
            if not np.allclose(radial, radial[0]) or kind == 'Capsule' and not np.allclose(scale, scale[0]):
                raise ValueError('Nonuniform source capsule/cylinder')
            radius = float(shape.GetRadiusAttr().Get()) * radial[0]
            height = float(shape.GetHeightAttr().Get()) * scale[axis]
            if not np.isfinite([radius, height]).all() or radius <= 0 or height < 0 or kind == 'Cylinder' and height == 0:
                raise ValueError('Invalid capsule/cylinder dimensions')
            if axis != 2:
                rotation = rotation @ Rotation.from_euler('y' if axis == 0 else 'x',
                                                           90 if axis == 0 else -90, degrees=True).as_matrix()
            geometry = (fcl.Capsule if kind == 'Capsule' else fcl.Cylinder)(radius, height)
            extent = abs(rotation[:, 2]) * height / 2 + radius
        else:
            raise ValueError('Unsupported enabled collider: ' + kind)
        if not np.isfinite(np.r_[scale, extent]).all() or np.any(extent <= 0):
            raise ValueError('Invalid source primitive dimensions')
        obj = fcl.CollisionObject(geometry, fcl.Transform(rotation, t[:3, 3]))
        bounds = np.array([t[:3, 3]-extent, t[:3, 3]+extent])
        representation = 'exact_source_' + kind.lower()
    return obj, bounds, dict(representation=representation, native_cooking_verified=False), triangles, components


def verified_release_context(stage, manifest, checkpoint_proof):
    """Reuse native lifecycle policy, without expanding its permitted pairs."""
    from harvest_contact_scope import HarvestContactScope, collection_colliders_from_stage
    approved = collection_colliders_from_stage(stage, manifest)
    scope = HarvestContactScope(manifest, approved, enabled=True, stage=stage)
    breaks = [] if checkpoint_proof is None else checkpoint_proof.get('native_break_events', [])
    by_joint = {f['joint']:f for f in manifest['plant']['fruits']}
    for event in breaks:
        fruit = by_joint.get(event['joint'])
        if fruit is None: raise ValueError('Measured native break aliases an unknown active fruit')
        prim = stage.GetPrimAtPath(fruit['joint'])
        if not prim or not prim.IsA(UsdPhysics.Joint):
            raise ValueError('Measured released fruit has no exact source attachment')
        joint = UsdPhysics.Joint(prim)
        if (list(map(str, joint.GetBody0Rel().GetTargets())) != [fruit['anchor']] or
                list(map(str, joint.GetBody1Rel().GetTargets())) != [fruit['path']]):
            raise ValueError('Measured released fruit/source joint body mapping mismatch')
    return dict(scope=scope, breaks=breaks, passive_colliders=set(approved),
                metadata=dict(enabled=True, source='Existing HarvestContactScope and exact passive S300 source inventory',
                    verified_exact_collection_colliders=sorted(approved),
                    verified_passive_source_cad_proofs=scope.collection_surface_proofs,
                    verified_prior_route_releases=scope.prior_releases,
                    current_native_breaks=breaks, collection_verified=False,
                    released_fruit_physics_verified=False, physics_response_unchanged=True))


def verify_initial_alignment(checker, plan, manifest, checkpoint_poses=None, tolerance_m=1e-4):
    """Reuse native FK positional tolerance; never align by changing geometry."""
    phase = plan['phases'][0]; q = np.asarray(phase['waypoint_q'][0], float)
    rail = initial_plan_rail_position(plan)
    predicted = checker.poses(q, .044, rail_position_m=rail)
    base = manifest['rail']['base_body']
    expected = np.asarray(manifest['rail']['source_base_position_world_m'], float) + [0., rail, 0.]
    base_error = float(np.linalg.norm(predicted[base][:3, 3]-expected))
    if base_error > tolerance_m:
        raise ValueError('Initial FK rail origin differs from authored source; possible double rail shift')
    errors = {}
    if checkpoint_poses is not None:
        required = set(checker.bodies)
        if not required <= set(checkpoint_poses):
            raise ValueError('Measured start lacks complete robot/carrier/tool poses')
        for body in set(predicted) & set(checkpoint_poses):
            if body not in checker.bodies: continue
            errors[body] = float(np.linalg.norm(predicted[body][:3, 3]-checkpoint_poses[body][:3, 3]))
        if errors and max(errors.values()) > tolerance_m:
            raise ValueError('Initial measured/FK pose mismatch; do not apply rail displacement twice')
    return dict(source_base_error_m=base_error, measured_position_errors_m=errors,
                numerical_fk_position_tolerance_m=tolerance_m,
                rail_geometry_origin='measured rail_waypoint_positions_m; native drive targets unused',
                geometry_shift_applied_once=True, measured_start_available=checkpoint_poses is not None)


class RailEnvironmentCheck:
    def __init__(self, stage, manifest, moving, *, allowed_truss_key, checkpoint_poses=None,
                 max_distance_queries=200000, release_context=None, released_colliders=(), scope='rail'):
        if scope not in ('rail', 'preparation'):
            raise ValueError('Environment scope must be rail or preparation')
        if isinstance(max_distance_queries, bool) or not isinstance(max_distance_queries, (int, np.integer)) or max_distance_queries < 1:
            raise ValueError('Positive finite query budget required')
        self.moving = moving; self.fcl = moving.fcl; self.rows = []; self.objects = {}
        self.queries = 0; self.max_queries = max_distance_queries; self.hits = []; self.diagnostics = []
        self.hit_pairs = {}; self.diagnostic_pairs = {}
        self.total_hits = self.total_diagnostics = self.filtered_pairs = 0
        self.sample_index = 0; self.current_time_s = None
        self.state_queries = self.state_filtered_pairs = 0
        self.scope = scope; self.current_phase = 'rail_move'; self.sample_schedule = None
        self.release_context = release_context; self.release_diagnostics = 0
        self.released_colliders = set(released_colliders)
        if release_context is not None:
            self.released_colliders.update(row['fruit_collider'] for row in
                release_context['scope'].metadata(release_context['breaks'])['native_releases'])
        self.filters = ExactFilters(stage); cache = UsdGeom.XformCache()
        declared_leaf_owners = {row.get('path') for key in ('compliant_foliage', 'background_compliant_foliage')
                                for row in manifest.get(key, {}).get('leaves', []) if row.get('path')}
        moving_paths = {shape['path'] for shape in moving.shapes}
        all_paths = enabled_colliders(stage)
        if {p for p in all_paths if any(below(p, root) for root in MOVING_ROOTS)} != moving_paths:
            raise ValueError('Moving robot/chassis/tool collider coverage mismatch')
        for path in sorted(all_paths-moving_paths):
            prim = stage.GetPrimAtPath(path); transform = np.asarray(cache.GetLocalToWorldTransform(prim)).T
            owner = prim
            while owner and not owner.IsPseudoRoot() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                owner = owner.GetParent()
            body = str(owner.GetPath()) if owner and not owner.IsPseudoRoot() else None
            measured = body is not None and checkpoint_poses is not None and body in checkpoint_poses
            if measured:
                transform = checkpoint_poses[body] @ np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(owner)).T) @ transform
            label, hard = obstacle_class(prim, manifest, allowed_truss_key)
            if scope == 'preparation' and (label == 'soft_leaf' or body in declared_leaf_owners):
                label, hard = preparation_leaf_class(stage, prim, body, manifest)
            # Source bounds are derived from the actual primitive/mesh points,
            # never authored display extents. Geometry is built only nearby.
            kind = prim.GetTypeName()
            if kind == 'Mesh':
                points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), float)
                world = points @ transform[:3, :3].T + transform[:3, 3]
                if not len(world) or not np.isfinite(world).all(): raise ValueError('Invalid source mesh bounds')
                bounds = np.array([world.min(0), world.max(0)])
            else:
                _, bounds, _, _, _ = collider_geometry(prim, transform)
            self.rows.append(dict(path=path, prim=prim, transform=transform, bounds=bounds,
                                  owner=body, obstacle_class=label, hard=hard,
                                  owner_pose_source='measured_checkpoint' if measured else 'immutable_source'))
        if not self.rows: raise ValueError('Empty environment collider inventory')
        self.low = np.array([r['bounds'][0] for r in self.rows]); self.high = np.array([r['bounds'][1] for r in self.rows])
        self.candidate_indices = np.arange(len(self.rows))
        self.metadata = dict(moving_shapes=len(moving_paths), environment_shapes=len(self.rows),
            environment_extra_clearance_m=0., source_filtered_pairs_unchanged=True,
            native_penetration_guard_m=.0005, native_physics_unmodified=True,
            current_truss_contact_permission='ActuatedTool only; arm/lift/chassis remain obstacles',
            released_fruit_contact_permission='Exact optional native lifecycle policy only; no automatic whole-chassis exemption',
            released_fruit_collection_diagnostic=None if release_context is None else release_context['metadata'],
            support_contact_permission='Exact authored filters only; ground/rail support is never silently exempted',
            scope='Initial rail_move only; frozen source/checkpoint geometry; conservative moving CAD envelopes')
        if scope == 'preparation':
            self.metadata.update(scope='All existing phases before first approach; frozen source/checkpoint geometry; conservative moving CAD envelopes',
                leaf_obstacle_policy='Static/kinematic/unverified leaf owners are obstacles; only exact declared dynamic D6 leaf groups are diagnostic')
        if moving.metadata.get('native_tool_hull_import') is not None:
            self.metadata.update(native_tool_hull_import=moving.metadata['native_tool_hull_import'],
                moving_shape_pieces=len(moving.shapes),
                moving_tool_representation='Exact scoped existing native cooked piece unions; unscoped tool parts keep existing conservative envelopes')

    def prepare_sweep(self, phase, hz):
        self.duration = float(phase['duration_s']); self.intervals = int(np.ceil(self.duration * hz))
        self.sample_index = 0
        qs = np.asarray(phase['waypoint_q'], float)
        rails = np.asarray(phase['rail_waypoint_positions_m'], float)
        if np.array_equal(qs, np.repeat(qs[:1], len(qs), axis=0)) and phase['end_gap_m'] == .044:
            # A rigid translation's complete swept AABB equals the union of
            # endpoint AABBs. This only prunes provably distant source shapes.
            endpoints = []
            for rail in (rails.min(), rails.max()):
                transforms = self.moving.poses(qs[0], .044, rail_position_m=float(rail))
                t = np.array([transforms[b] for b in self.moving.bodies])[self.moving.body_ids]
                endpoints.append(np.einsum('nij,nkj->nki', t[:, :3, :3], self.moving.corners) + t[:, None, :3, 3])
            world = np.concatenate(endpoints).reshape(-1, 3)
            low, high = world.min(0), world.max(0)
            self.candidate_indices = np.flatnonzero(((self.low <= high) & (self.high >= low)).all(1))
        self.metadata['swept_aabb_environment_candidates'] = len(self.candidate_indices)

    def prepare_path_sweep(self, plan, hz):
        """Bound every sampled joint arc using the existing exact sampler."""
        self.sample_schedule = []; elapsed = 0.
        for phase in plan['phases']:
            duration = float(phase['duration_s'])
            if not np.isfinite(duration) or duration <= 0:
                raise ValueError('Finite positive preparation duration required')
            intervals = int(np.ceil(duration*hz))
            if len(self.sample_schedule)+intervals+1 > 200000:
                raise RuntimeError('Finite preparation sample budget exhausted')
            self.sample_schedule.extend((phase['name'], elapsed+duration*i/intervals)
                                        for i in range(intervals+1))
            elapsed += duration
        parent = self
        class SweptBounds:
            metadata = dict(scope='Existing check_plan sample-wise swept bounds only')
            def __init__(self):
                self.low = np.full((len(parent.moving.shapes), 3), np.inf)
                self.high = np.full_like(self.low, -np.inf)
            def check(self, q, gap, *, rail_position_m):
                poses = parent.moving.poses(q, gap, rail_position_m=rail_position_m)
                transforms = np.array([poses[b] for b in parent.moving.bodies])[parent.moving.body_ids]
                corners = np.einsum('nij,nkj->nki', transforms[:, :3, :3], parent.moving.corners) + transforms[:, None, :3, 3]
                self.low = np.minimum(self.low, corners.min(1))
                self.high = np.maximum(self.high, corners.max(1))
                return []
        recorder = SweptBounds()
        sampled = check_plan(recorder, plan, hz=hz, cache_exact_repeats=False)
        if not sampled['complete'] or sampled['samples'] != len(self.sample_schedule):
            raise RuntimeError('Incomplete preparation swept-bound pass')
        candidates = np.zeros(len(self.rows), bool)
        for low, high in zip(recorder.low, recorder.high):
            candidates |= ((self.low <= high) & (self.high >= low)).all(1)
        self.candidate_indices = np.flatnonzero(candidates)
        self.sample_index = 0
        self.metadata.update(swept_aabb_environment_candidates=len(self.candidate_indices),
            swept_bounds_sample_count=sampled['samples'],
            swept_bounds_source='Every existing check_plan FK sample, including intermediate joint arcs')

    def check(self, q, gap, *, rail_position_m):
        if self.sample_schedule is None:
            self.current_time_s = self.duration * self.sample_index / self.intervals
        else:
            self.current_phase, self.current_time_s = self.sample_schedule[self.sample_index]
        self.sample_index += 1
        self._check_configuration(q, gap, rail_position_m=rail_position_m,
            candidates=self.candidate_indices, phase=self.current_phase,
            time_s=self.current_time_s, record=self._record_sample_hit)
        # Keep the existing sampler visiting every timestamp even after a hit.
        # Passing this sampling callback is not an environment-clear result.
        return []

    def check_state(self, q, gap, *, rail_position_m, phase='prefix_search_state'):
        """Check one exploratory configuration without a path/sweep assumption.

        State queries share the finite FCL budget and source objects, but do
        not consume sample timestamps or mutate the sampled path's evidence.
        A clear state is only an OMPL predicate; the final path still needs its
        complete 120Hz self and preparation-environment audits.
        """
        result = dict(schema='s300-environment-state-query-v1', hard_clear=False,
            complete=False, status='inconclusive', phase=phase, time_s=None,
            timestamp_scope='Exploratory state; no motion timestamp',
            hard_hits=[], diagnostic_only_hits=[], hard_intersection_pairs=0,
            diagnostic_intersection_pairs=0, all_hard_pair_counts=[],
            all_diagnostic_pair_counts=[], full_environment_inventory=True,
            broadphase='Full inventory with current-state inclusive AABBs; no path/sweep pruning',
            final_path_audit_required=True, actual_arrival_proven=False,
            collection_verified=False, released_fruit_physics_verified=False)
        before = self.queries; filtered_before = self.state_filtered_pairs
        pairs = {True: {}, False: {}}
        def record(hit, hard):
            count = 'hard_intersection_pairs' if hard else 'diagnostic_intersection_pairs'
            details = result['hard_hits' if hard else 'diagnostic_only_hits']
            result[count] += 1
            if len(details) < 64: details.append(hit)
            key = (hit['robot_geom'], hit['environment_geom'], hit['obstacle_class'])
            item = pairs[hard].setdefault(key, dict(robot_geom=key[0], environment_geom=key[1],
                obstacle_class=key[2], intersection_pairs=0))
            item['intersection_pairs'] += 1
        try:
            q = np.asarray(q, float)
            if q.shape != (7,) or not np.isfinite(q).all():
                raise ValueError('Finite seven-DOF state required')
            if (isinstance(gap, bool) or not np.isscalar(gap) or not np.isfinite(gap) or gap <= 0 or
                    isinstance(rail_position_m, bool) or not np.isscalar(rail_position_m) or not np.isfinite(rail_position_m)):
                raise ValueError('Finite positive jaw gap and finite rail position required')
            if not isinstance(phase, str) or not phase:
                raise ValueError('Named exploratory state phase required')
            self.state_queries += 1
            self._check_configuration(q, gap, rail_position_m=rail_position_m,
                candidates=np.arange(len(self.rows)), phase=phase, time_s=None,
                record=record, lifecycle_boundary=True)
            result.update(complete=True, hard_clear=not result['hard_intersection_pairs'],
                          status='blocked' if result['hard_intersection_pairs'] else 'state_clear')
        except Exception as error:
            result.update(error=type(error).__name__ + ': ' + str(error),
                status='blocked' if result['hard_intersection_pairs'] else 'inconclusive')
        result.update(distance_queries=self.queries-before, total_distance_queries=self.queries,
            state_queries=self.state_queries, details_limit=64,
            filtered_pairs=self.state_filtered_pairs-filtered_before,
            all_hard_pair_counts=list(pairs[True].values()),
            all_diagnostic_pair_counts=list(pairs[False].values()))
        return result

    def _check_configuration(self, q, gap, *, rail_position_m, candidates, phase,
                             time_s, record, lifecycle_boundary=False):
        poses = self.moving.poses(q, gap, rail_position_m=rail_position_m)
        transforms = np.array([poses[b] for b in self.moving.bodies])[self.moving.body_ids]
        corners = np.einsum('nij,nkj->nki', transforms[:, :3, :3], self.moving.corners) + transforms[:, None, :3, 3]
        if lifecycle_boundary:
            # Exact current-state broadphase only. A shape intersecting any
            # moving part must overlap this whole-configuration AABB. Prior
            # path/endpoint sweep bounds are deliberately never used here.
            whole_low = corners.min((0, 1)); whole_high = corners.max((0, 1))
            candidates = candidates[((self.low[candidates] <= whole_high) &
                                     (self.high[candidates] >= whole_low)).all(1)]
        for i, shape in enumerate(self.moving.shapes):
            low, high = corners[i].min(0), corners[i].max(0)
            nearby = candidates[((self.low[candidates] <= high) & (self.high[candidates] >= low)).all(1)]
            if not len(nearby): continue
            t = transforms[i] @ shape['geometry_local']
            self.moving.objects[i].setTransform(self.fcl.Transform(t[:3, :3], t[:3, 3]))
            for index in nearby:
                row = self.rows[index]
                if self.filters.excluded(shape['path'], row['path']):
                    if lifecycle_boundary: self.state_filtered_pairs += 1
                    else: self.filtered_pairs += 1
                    continue
                if self.queries >= self.max_queries: raise RuntimeError('Finite environment query budget exhausted')
                self.queries += 1
                if index not in self.objects:
                    self.objects[index] = collider_geometry(row['prim'], row['transform'])
                obj, _, representation, triangles, components = self.objects[index]
                collision_result = self.fcl.CollisionResult()
                collision = bool(self.fcl.collide(self.moving.objects[i], obj,
                    self.fcl.CollisionRequest(enable_contact=self.release_context is not None), collision_result))
                contained = False
                if not collision and triangles is not None:
                    for indices, closed in components:
                        if closed and point_inside_closed_triangles(corners[i].mean(0), triangles[indices]):
                            collision = contained = True; break
                if collision:
                    label, hard = row['obstacle_class'], row['hard']
                    if row['path'] in self.released_colliders:
                        label, hard = 'released_fruit_noncollection_contact', True
                    # Own-truss permission concerns the gripper, never the
                    # arm/lift/carrier just because it is harvesting that truss.
                    if label == 'current_allowed_truss' and not below(shape['body'], '/World/ActuatedTool'):
                        label, hard = 'current_truss_robot_or_chassis', True
                    hit = dict(robot_geom=shape['path'], environment_geom=row['path'],
                        obstacle_class=label, rail_position_m=float(rail_position_m),
                        command_q=np.asarray(q).tolist(), gap_m=float(gap),
                        closed_volume_containment=contained, phase=phase,
                        time_s=time_s, timestamp_scope=('Exploratory state; no motion timestamp' if lifecycle_boundary else
                            'relative to rail_move; native settle is separate' if self.scope == 'rail'
                            else 'relative to existing plan start; initial native settle is separate'),
                        **representation)
                    if self.release_context is not None and shape['path'] in self.release_context['passive_colliders']:
                        # This distance belongs to the conservative FCL query,
                        # never to a fabricated native contact callback.
                        contacts = collision_result.contacts
                        separation = -float(contacts[0].penetration_depth) if contacts else None
                        classification = self.release_context['scope'].classify(dict(
                            a=shape['path'], b=row['path'], t=0. if lifecycle_boundary else time_s,
                            phase=phase, separation_m=separation), self.release_context['breaks'])
                        hit['released_fruit_collection_evidence'] = dict(classification,
                            contact_source='Nominal FCL source envelopes, not actual native contact or force')
                        if lifecycle_boundary:
                            hit['released_fruit_collection_evidence'].update(
                                lifecycle_reference='Verified releases already present at imported checkpoint boundary; not a proposed motion time',
                                proposal_time_s=None, lifecycle_reference_time_s=0.)
                        if classification['continuation_authorized']:
                            hard = False; hit['obstacle_class'] = 'released_fruit_passive_carrier_diagnostic'
                    record(hit, hard)

    def _record_sample_hit(self, hit, hard):
        if hit['obstacle_class'] == 'released_fruit_passive_carrier_diagnostic':
            self.release_diagnostics += 1
        if hard:
            self.total_hits += 1
            if len(self.hits) < 64: self.hits.append(hit)
            pairs = self.hit_pairs
        else:
            self.total_diagnostics += 1
            if len(self.diagnostics) < 64: self.diagnostics.append(hit)
            pairs = self.diagnostic_pairs
        key = (hit['robot_geom'], hit['environment_geom'], hit['obstacle_class'])
        item = pairs.setdefault(key, dict(robot_geom=key[0], environment_geom=key[1],
            obstacle_class=key[2], sample_pairs=0, first_time_s=self.current_time_s,
            last_time_s=self.current_time_s))
        item['sample_pairs'] += 1; item['last_time_s'] = self.current_time_s
        if self.scope == 'preparation':
            counts = item.setdefault('phase_sample_pairs', {})
            counts[self.current_phase] = counts.get(self.current_phase, 0)+1


def screen_rail_plan(adapter, plan, manifest, *, checkpoint_poses=None, hz=120, scope='rail'):
    result = dict(schema='s300-rail-environment-preflight-v1', passed=False, complete=False,
                  status='inconclusive', scope='initial rail_move only', native_runs=0,
                  physics_unchanged=True, training_eligible=False, harvest_success=None)
    if scope == 'preparation':
        result.update(scope='all existing phases before first approach', requested_hz=hz,
            strict_native_period_sampling=hz == 120, eligible_for_native_execution=False)
    try:
        selected = scoped_plan(plan, scope)
        if selected is None:
            return dict(result, status='not_applicable', reason='No physical rail_move phase' if scope == 'rail' else 'Empty preparation prefix')
        if adapter.scope != scope:
            raise ValueError('Environment inventory policy and requested scope mismatch')
        if (isinstance(hz, bool) or not isinstance(hz, (int, np.integer)) or
                hz < 1 or hz > 120 or scope == 'rail' and hz != 120):
            raise ValueError('Rail requires actual120Hz; preparation supports1..120Hz diagnostic samples')
        result['initial_alignment'] = verify_initial_alignment(adapter.moving, selected, manifest, checkpoint_poses)
        if scope == 'rail':
            adapter.prepare_sweep(selected['phases'][0], hz)
        else:
            result.update(scope='all existing phases before first approach', requested_hz=hz,
                nominal_prefix_duration_s=sum(p['duration_s'] for p in selected['phases']),
                selected_phases=[p['name'] for p in selected['phases']],
                strict_native_period_sampling=hz == 120, eligible_for_native_execution=False)
            adapter.prepare_path_sweep(selected, hz)
        sampled = check_plan(adapter, selected, hz=hz)
        result.update(complete=sampled['complete'], samples=sampled['samples'], sampling=sampled,
                      status='blocked' if adapter.total_hits else 'sampled_clear', passed=not adapter.total_hits)
        if scope == 'preparation':
            result['eligible_for_native_execution'] = bool(result['complete'] and result['passed'] and hz == 120)
    except Exception as error:
        result.update(error=type(error).__name__ + ': ' + str(error), status='blocked' if adapter.total_hits else 'inconclusive')
    result.update(hard_intersection_sample_pairs=adapter.total_hits, hard_hits=adapter.hits,
                  all_hard_pair_counts=list(adapter.hit_pairs.values()),
                  diagnostic_intersection_sample_pairs=adapter.total_diagnostics,
                  diagnostic_only_hits=adapter.diagnostics, distance_queries=adapter.queries,
                  collision_queries=adapter.queries,
                  all_diagnostic_pair_counts=list(adapter.diagnostic_pairs.values()),
                  last_sample_time_s=adapter.current_time_s,
                  details_limit=64, filtered_sample_pairs=adapter.filtered_pairs,
                  geometry=adapter.metadata, actual_arrival_proven=False,
                  released_fruit_collection_diagnostic_sample_pairs=adapter.release_diagnostics,
                  collection_verified=False, released_fruit_physics_verified=False,
                  strict_whole_scene_physics_passed=None)
    return result


def screen_preparation_plan(adapter, plan, manifest, *, checkpoint_poses=None, hz=120):
    return screen_rail_plan(adapter, plan, manifest, checkpoint_poses=checkpoint_poses, hz=hz, scope='preparation')


def start_checkpoint(assembly, manifest, checkpoint, *, allow_attachment_transfer=False):
    from native_route_checkpoint import load_checkpoint
    from resume_native_state import require_completed_checkpoint, imported_break_markers
    route = manifest.get('route_checkpoint')
    path = Path(checkpoint) if checkpoint is not None else Path(route['checkpoint']) if route else None
    if path is None: return None, None
    metadata, arrays = load_checkpoint(path)
    require_completed_checkpoint(metadata)
    def dependency_sha(value):
        return value.get('compact_assets', {}).get('materialized_dependency', {}).get('sha256')
    # Thin scene.usda wrappers can be byte-identical while materialized plant
    # geometry differs. Wrapper SHA alone cannot identify the physical model.
    same = (metadata['scene_sha256'] == manifest['scene_sha256'] and
            dependency_sha(metadata['manifest']) == dependency_sha(manifest))
    if same and metadata['source_assembly'] != str(Path(assembly).resolve()):
        raise ValueError('Same-model checkpoint assembly identity mismatch')
    if same and metadata['manifest'] != manifest:
        raise ValueError('Same-model checkpoint manifest identity mismatch')
    revision = None
    if not same and allow_attachment_transfer:
        from eligible_attachment_transfer import validate_resume_revision
        revision = validate_resume_revision(assembly, manifest, metadata)
    if not same and revision is None and (not route or path.resolve() != Path(route['checkpoint']).resolve() or
                     metadata['scene_sha256'] != route['source_scene_sha256'] or
                     metadata['metadata_sha256'] != route['outgoing']['checkpoint_metadata_sha256']):
        raise ValueError('Current model/checkpoint SHA mismatch outside exact declared active-window handoff')
    poses = {str(p):matrix(v) for p, v in zip(arrays['body_paths'], arrays['body_poses_world'])}
    if not same and revision is None:
        original = metadata['manifest']['plant']['root']; frozen = route['outgoing']['frozen_root']
        poses = {(frozen + p[len(original):] if below(p, original) else p):v for p, v in poses.items()}
    names = list(map(str, arrays['robot_dof_names']))
    names_expected = ['farmily_lift_height_joint', 'base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
    if not set(names_expected) <= set(names): raise ValueError('Checkpoint lacks complete measured lift/arm DOFs')
    proof = dict(checkpoint=str(path.resolve()), metadata_sha256=metadata['metadata_sha256'],
        checkpoint_json_sha256=file_sha256(path/'checkpoint.json'), npz_sha256=file_sha256(path/'checkpoint.npz'),
        initial_lift_arm_q=[float(arrays['robot_q'][names.index(n)]) for n in names_expected],
        source_materialized_dependency_sha256=dependency_sha(metadata['manifest']),
        current_materialized_dependency_sha256=dependency_sha(manifest),
        native_break_events=imported_break_markers(metadata) if same or revision is not None else [],
        model_relation='same immutable model' if same else 'exact declared active-window handoff')
    if revision is not None:
        proof.update(explicit_attachment_revision=revision,
            model_relation='Explicit approved attachment transfer; exact preserved active body/joint mapping')
    return poses, proof


def audit_external_rail(assembly, output, *, plan, checkpoint=None, hz=120,
                        allowed_truss_key=None, max_distance_queries=200000,
                        released_fruit_collection_diagnostic=False, scope='rail', allow_attachment_transfer=False,
                        native_tool_cache=None):
    """Use the existing asset-Python boundary; refuse incomplete/blocked travel.

    This is an opt-in preflight, not a replacement executor or path planner.
    Call after the existing self-check and before any commanded rail motion.
    """
    import os
    import subprocess
    output = Path(output)
    log = output.with_suffix('.log')
    if output.exists() or log.exists():
        raise FileExistsError('Use a new rail preflight output; previous evidence is immutable: '+str(output))
    python = Path(__file__).resolve().parents[1]/'.asset-venv/bin/python'
    env = os.environ.copy()
    for key in ('PYTHONHOME', 'PYTHONPATH', 'PXR_PLUGINPATH_NAME'):
        env.pop(key, None)
    command = [str(python), str(Path(__file__).resolve()), str(assembly),
               '--plan', str(plan), '--output', str(output), '--hz', str(hz),
               '--max-distance-queries', str(max_distance_queries)]
    if scope != 'rail': command += ['--scope', scope]
    if allow_attachment_transfer: command += ['--allow-resume-attachment-transfer']
    if checkpoint is not None: command += ['--checkpoint', str(checkpoint)]
    if allowed_truss_key is not None: command += ['--allowed-truss-key', allowed_truss_key]
    if released_fruit_collection_diagnostic: command += ['--released-fruit-collection-diagnostic']
    if native_tool_cache is not None: command += ['--native-tool-cache', str(native_tool_cache)]
    process = subprocess.run(command, env=env, capture_output=True, text=True)
    with log.open('x') as stream: stream.write(process.stdout+process.stderr)
    if process.returncode not in (0, 2) or not output.exists():
        raise RuntimeError('Rail environment preflight could not complete: '+str(log))
    report = json.loads(output.read_text())
    if report.get('status') == 'not_applicable': return report
    if (not report.get('complete') or not report.get('passed') or
            scope == 'preparation' and not report.get('eligible_for_native_execution')):
        raise RuntimeError('Rail environment preflight blocked commanded travel: '+str(output))
    return report


# The short name was used by the initial fixture tests; both use the same
# external boundary and cannot invoke native FCL inside Isaac's Python ABI.
audit_external = audit_external_rail


def _finish(result, started, output):
    result['wall_s'] = time.perf_counter()-started
    result['code_sha256'] = SOURCE_SHA256
    result['code_hash_scope'] = 'Exact helper bytes captured at Python module import'
    if output is not None:
        with Path(output).open('x') as stream:
            stream.write(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


def audit_rail_transit(assembly, plan_path, *, checkpoint=None, output=None, hz=120,
                      allowed_truss_key=None, max_distance_queries=200000,
                      released_fruit_collection_diagnostic=False, scope='rail', allow_attachment_transfer=False,
                      native_tool_cache=None):
    """Read-only source/trace/checkpoint verification and the existing FK sampler."""
    started = time.perf_counter(); assembly = Path(assembly).resolve(); plan_path = Path(plan_path)
    result = dict(schema='s300-rail-environment-preflight-v1', passed=False, complete=False,
                  status='inconclusive', native_runs=0, training_eligible=False, harvest_success=None)
    try:
        plan_hash = file_sha256(plan_path); plan = json.loads(plan_path.read_text())
        if scoped_plan(plan, scope) is None:
            result.update(status='not_applicable', reason='No physical rail_move phase',
                plan_sha256=plan_hash, geometry_loaded=False,
                source_model_and_checkpoint_validation='Not needed for absent rail motion; no environment certificate issued')
            return _finish(result, started, output)
        manifest_path = assembly/'motion_manifest.json'; manifest_hash = file_sha256(manifest_path)
        manifest = json.loads(manifest_path.read_text())
        scene = assembly/'scene.usda'
        if file_sha256(scene) != manifest['scene_sha256']: raise ValueError('Source scene SHA mismatch')
        dependency = verify_compact_dependencies(assembly, manifest)
        for key in ('scene_sha256', 'model_sha256'):
            if key in plan and plan[key] != manifest['scene_sha256']:
                raise ValueError('Plan/model SHA mismatch')
        poses, checkpoint_proof = start_checkpoint(assembly, manifest, checkpoint,
            allow_attachment_transfer=allow_attachment_transfer)
        if checkpoint_proof is not None and plan.get('phases'):
            initial = np.asarray(plan['phases'][0]['waypoint_q'][0], float)
            if initial.shape != (7,) or not np.allclose(initial, checkpoint_proof['initial_lift_arm_q'], atol=1e-9, rtol=0):
                raise ValueError('Rail command does not start from exact measured checkpoint lift/arm q')
        from motion_kinematics import ToolKinematics
        native_layer = assembly/dependency['relative_path'] if dependency else scene
        stage = Usd.Stage.Open(str(native_layer))
        cache_hash=None if native_tool_cache is None else file_sha256(native_tool_cache)
        moving = (MotionSelfCollision(stage, ToolKinematics(stage)) if native_tool_cache is None else
            MotionSelfCollision(stage, ToolKinematics(stage), native_tool_cache=native_tool_cache, assembly=assembly))
        truss = allowed_truss(manifest, plan, allowed_truss_key)
        release_context = verified_release_context(stage, manifest, checkpoint_proof) if released_fruit_collection_diagnostic else None
        current_breaks = set(e['joint'] for e in (checkpoint_proof or {}).get('native_break_events', []))
        released_colliders = {f['path']+'/Collider' for f in manifest['plant']['fruits']
                              if f.get('joint') in current_breaks}
        adapter = RailEnvironmentCheck(stage, manifest, moving, allowed_truss_key=truss,
            checkpoint_poses=poses, max_distance_queries=max_distance_queries, release_context=release_context,
            released_colliders=released_colliders, scope=scope)
        result = screen_rail_plan(adapter, plan, manifest, checkpoint_poses=poses, hz=hz, scope=scope)
        result.update(allowed_truss_key=truss, checkpoint_proof=checkpoint_proof,
            source_sha256=dict(scene=manifest['scene_sha256'], manifest=manifest_hash,
                              materialized_dependency=None if dependency is None else dependency['sha256'], plan=plan_hash),
            obstacle_inventory=[{k:r[k] for k in ('path','owner','obstacle_class','hard','owner_pose_source')} for r in adapter.rows])
        if native_tool_cache is not None:
            result['native_tool_hull_import']=moving.metadata['native_tool_hull_import']
            result['native_tool_cache']=dict(path=str(Path(native_tool_cache).resolve()),sha256=cache_hash,
                source_model_bound=True,physical_proxy_unchanged=True)
        if (file_sha256(scene) != manifest['scene_sha256'] or file_sha256(plan_path) != plan_hash or
                file_sha256(manifest_path) != manifest_hash):
            raise ValueError('Source scene/manifest/trace changed during rail preflight')
        if checkpoint_proof is not None:
            cp = Path(checkpoint_proof['checkpoint'])
            if (file_sha256(cp/'checkpoint.json') != checkpoint_proof['checkpoint_json_sha256'] or
                    file_sha256(cp/'checkpoint.npz') != checkpoint_proof['npz_sha256']):
                raise ValueError('Measured checkpoint changed during rail preflight')
        verify_compact_dependencies(assembly, manifest)
        if native_tool_cache is not None and file_sha256(native_tool_cache)!=cache_hash:
            raise ValueError('Native tool cache changed during environment preflight')
    except Exception as error:
        result.update(passed=False, complete=False, status='inconclusive', error=type(error).__name__ + ': ' + str(error))
    return _finish(result, started, output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('assembly', type=Path)
    parser.add_argument('--plan', type=Path, required=True); parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--output', type=Path, required=True); parser.add_argument('--hz', type=int, default=120)
    parser.add_argument('--allowed-truss-key'); parser.add_argument('--max-distance-queries', type=int, default=200000)
    parser.add_argument('--scope', choices=('rail', 'preparation'), default='rail')
    parser.add_argument('--allow-resume-attachment-transfer', action='store_true')
    parser.add_argument('--released-fruit-collection-diagnostic', action='store_true')
    parser.add_argument('--native-tool-cache',type=Path,
        help='Opt-in exact existing native cooked tool pieces, bound to this immutable assembly/model; no physical collider change')
    args = parser.parse_args(); report = audit_rail_transit(args.assembly, args.plan, checkpoint=args.checkpoint,
        output=args.output, hz=args.hz, allowed_truss_key=args.allowed_truss_key,
        max_distance_queries=args.max_distance_queries,
        released_fruit_collection_diagnostic=args.released_fruit_collection_diagnostic, scope=args.scope,
        allow_attachment_transfer=args.allow_resume_attachment_transfer,native_tool_cache=args.native_tool_cache)
    print(json.dumps({k:report[k] for k in ('status','passed','complete')}), flush=True)
    raise SystemExit(0 if report['passed'] or report['status'] == 'not_applicable' else 2)
