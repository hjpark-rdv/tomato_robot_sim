"""Compact cherry-tomato geometry without scaling fruit or main stems.

Input is the existing, corrected, metre / Z-up truss template. Fruit and
calyces translate rigidly. Tube rings follow their centreline translations,
retaining their thickness. This changes plant geometry, not contact tolerances.
"""
import re

import numpy as np


PROFILE = dict(name='dense_cherry_v1', fruit_surface_clearance_m=.002,
               lateral_scale=.78, peduncle_length_scale=.70)


def compact_truss(meshes, *, clearance=.002, lateral_scale=.78, peduncle_scale=.70):
    from elastic_geometry import tube_centerline
    if not (0 < clearance < .02 and 0 < lateral_scale <= 1 and 0 < peduncle_scale <= 1):
        raise ValueError('Invalid compact-truss dimensions')
    named = {n: v for n, v, _, _ in meshes}
    fruits = {n: ((v.min(0) + v.max(0)) / 2, np.ptp(v, axis=0))
              for n, v in named.items() if re.fullmatch(r'Fruit_\d+', n)}
    order = sorted(fruits, key=lambda n: -fruits[n][0][2])
    # Enclosing spheres provide a conservative clearance for every fruit pair,
    # including fruit separated by a missing/removed fruit in the source GLB.
    radii = {n: float(np.linalg.norm(named[n] - fruits[n][0], axis=1).max()) for n in order}
    ped, _ = tube_centerline(named['Truss_01_Peduncle'], 14)
    rachis, _ = tube_centerline(named['Rachis'], 14)
    new_ped = ped[0] + (ped - ped[0]) * peduncle_scale
    old_root, new_root = rachis[0], rachis[0] + new_ped[-1] - ped[-1]
    centers = {}
    for i, name in enumerate(order):
        old, _ = fruits[name]
        new = new_root + (old - old_root) * np.array([lateral_scale, lateral_scale, 1.])
        if not i:
            new[2] = new_root[2] - radii[name] - .014
        else:
            new[2] = centers[order[i - 1]][2] - .002
            for other, center in centers.items():
                horizontal = np.linalg.norm(new[:2] - center[:2])
                needed = radii[name] + radii[other] + clearance
                if horizontal < needed:
                    new[2] = min(new[2], center[2] - np.sqrt(needed**2 - horizontal**2))
        centers[name] = new

    # One monotone axial map for the rachis and its unoccupied branches.
    old_z = [old_root[2]] + [fruits[n][0][2] for n in order]
    new_z = [new_root[2]] + [centers[n][2] for n in order]
    old_z = np.asarray(old_z[::-1]); new_z = np.asarray(new_z[::-1])
    if np.any(np.diff(old_z) <= 0) or np.any(np.diff(new_z) <= 0):
        raise ValueError('Truss needs a monotone downward fruit order')

    def warp(points):
        out = new_root + (points - old_root) * np.array([lateral_scale, lateral_scale, 1.])
        out[:, 2] = np.interp(points[:, 2], old_z, new_z)
        for edge, neighbor, mask in [(0, 1, points[:, 2] < old_z[0]),
                                      (-1, -2, points[:, 2] > old_z[-1])]:
            slope = (new_z[neighbor] - new_z[edge]) / (old_z[neighbor] - old_z[edge])
            out[mask, 2] = new_z[edge] + slope * (points[mask, 2] - old_z[edge])
        return out

    curves = {'Truss_01_Peduncle': (ped, new_ped), 'Rachis': (rachis, warp(rachis))}
    for name in named:
        match = re.fullmatch(r'Pedicel_proximal_(\d+)', name)
        if not match:
            continue
        suffix = match[1]; distal = 'Pedicel_distal_' + suffix
        prox, _ = tube_centerline(named[name], 12)
        dist, _ = tube_centerline(named[distal], 12)
        fruit = 'Fruit_' + suffix
        if fruit not in centers:
            curves[name] = (prox, warp(prox)); curves[distal] = (dist, warp(dist))
            continue
        start_delta = warp(prox[:1])[0] - prox[0]
        end_delta = centers[fruit] - fruits[fruit][0]
        lp = np.r_[0., np.cumsum(np.linalg.norm(np.diff(prox, axis=0), axis=1))]
        ld = np.r_[0., np.cumsum(np.linalg.norm(np.diff(dist, axis=0), axis=1))]
        length = lp[-1] + np.linalg.norm(dist[0] - prox[-1]) + ld[-1]
        for label, points, arc in [(name, prox, lp), (distal, dist, length - ld[-1] + ld)]:
            delta = start_delta + (end_delta - start_delta) * (arc / length)[:, None]
            curves[label] = (points, points + delta)

    result = []
    for name, v, f, c in meshes:
        match = re.match(r'(?:Fruit_|Calyx_(?:Hub_)?)(\d+)', name)
        base = name.removesuffix('_Trichomes')
        if match and 'Fruit_' + match[1] in centers:
            fruit = 'Fruit_' + match[1]
            new = v + centers[fruit] - fruits[fruit][0]
        elif base in curves:
            old, moved = curves[base]
            # Existing tubes are ordered rings. Nearest-ring displacement also
            # carries hairs, without scaling the cross-section of thin stems.
            ids = np.linalg.norm(v[:, None] - old[None], axis=2).argmin(axis=1)
            new = v + (moved - old)[ids]
        elif name.startswith('Empty_'):
            # Tiny old flower remnants have no fruit/body attachment. Carry
            # each remnant along the same axial map rather than leaving it
            # at the former, much longer rachis tip.
            new = warp(v)
        else:
            raise ValueError('Unmapped truss primitive: ' + name)
        result.append((name, new, f, c))
    old_centers = np.array([fruits[n][0] for n in order])
    new_centers = np.array([centers[n] for n in order])
    nearest = lambda points: [min(np.linalg.norm(points[i] - points[j]) - radii[n] - radii[m]
                                  for j, m in enumerate(order) if i != j)
                             for i, n in enumerate(order)]
    report = dict(profile=dict(PROFILE, fruit_surface_clearance_m=clearance,
                               lateral_scale=lateral_scale, peduncle_length_scale=peduncle_scale),
                  fruit_count=len(order), fruit_names=order,
                  old_centers_m=old_centers.tolist(), new_centers_m=new_centers.tolist(),
                  fruit_diameters_m=[fruits[n][1].tolist() for n in order],
                  nearest_bounding_sphere_gap_before_m=nearest(old_centers),
                  nearest_bounding_sphere_gap_after_m=nearest(new_centers),
                  clearance_metric='Conservative enclosing spheres; actual mesh surfaces can be farther apart',
                  fruit_center_height_span_before_m=float(np.ptp(old_centers[:, 2])),
                  fruit_center_height_span_after_m=float(np.ptp(new_centers[:, 2])),
                  lengths_before_m={n: float(np.linalg.norm(np.diff(curves[n][0], axis=0), axis=1).sum())
                                    for n in ('Truss_01_Peduncle', 'Rachis')},
                  lengths_after_m={n: float(np.linalg.norm(np.diff(curves[n][1], axis=0), axis=1).sum())
                                   for n in ('Truss_01_Peduncle', 'Rachis')},
                  fruit_shapes_sizes_colors_unchanged=True, stem_root_unchanged=True)
    return result, report
