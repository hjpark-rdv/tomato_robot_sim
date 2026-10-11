"""Opt-in numerical overlap budget for exact tool/fruit pairs, not material softness."""
import math
import re

from motion_contact_buffer import ACTIVE_PATH_MARKERS


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
