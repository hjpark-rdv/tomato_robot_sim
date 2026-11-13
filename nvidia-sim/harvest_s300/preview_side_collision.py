"""Conservative CAD obstacle boxes for the V6 preview only.

Three native chassis meshes combine disconnected wheels, deck and console parts.
A single convex hull fills the empty space between them. Keep every CAD part,
but bound connected parts separately. The lift feet are sliced into horizontal
bands so the empty corner of their L profile does not become a solid triangle.
These boxes remain conservative; no real obstacle is deleted or shrunk.
"""
from __future__ import annotations

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from newton_viser import geom_mesh


CHASSIS_ASSEMBLIES = ('s300_g0045', 's300_g0050', 's300_g0052')
SLICED_LIFT_FEET = ('s300_g0013', 's300_g0014')
PADDING = .001


def component_bounds(mesh):
    mesh = mesh.copy()
    mesh.merge_vertices()
    edges = mesh.edges
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                       shape=(len(mesh.vertices), len(mesh.vertices)))
    count, labels = connected_components(graph, directed=False)
    return [(mesh.vertices[labels == i].min(0), mesh.vertices[labels == i].max(0))
            for i in range(count)]


def sliced_bounds(mesh, height=.025):
    vertices = np.asarray(mesh.vertices)
    edges = vertices[np.asarray(mesh.edges_unique)]
    bottom, top = vertices[:, 2].min(), vertices[:, 2].max()
    bounds = []
    for low in np.arange(bottom, top, height):
        high = min(top, low + height)
        parts = [vertices[(vertices[:, 2] >= low) & (vertices[:, 2] <= high)]]
        # Clip all crossing edges, so a triangle with no vertex in the band
        # still contributes its exact extrema to the conservative box.
        for z in (low, high):
            a, b = edges[:, 0], edges[:, 1]
            crossing = ((a[:, 2] <= z) & (b[:, 2] >= z) |
                        (b[:, 2] <= z) & (a[:, 2] >= z)) & (np.abs(b[:, 2]-a[:, 2]) > 1e-12)
            a, b = a[crossing], b[crossing]
            parts.append(a + ((z-a[:, 2])/(b[:, 2]-a[:, 2]))[:, None]*(b-a))
        points = np.concatenate(parts)
        if len(points):
            bounds.append((points.min(0), points.max(0)))
    return bounds


def cad_boxes(scene, gid, pose, *, sliced=False, prefix='chassis/'):
    mesh = geom_mesh(scene, gid).copy()
    mesh.vertices = np.asarray(mesh.vertices) @ pose[:3, :3].T + pose[:3, 3]
    bounds = sliced_bounds(mesh) if sliced else component_bounds(mesh)
    # Drop fully contained boxes only; their complete volume remains covered.
    unique = []
    for low, high in sorted(bounds, key=lambda b: -float(np.prod(b[1]-b[0]))):
        low, high = low-PADDING, high+PADDING
        if not any(np.all(low >= a) and np.all(high <= b) for a, b in unique):
            unique.append((low, high))
    # Keep enough spare native geometry slots for the bin and moving lift.
    # Merge the pair adding the least empty volume; union boxes only expand
    # coverage and never omit source geometry.
    while not sliced and len(unique) > 64:
        low=np.array([a for a,_ in unique]);high=np.array([b for _,b in unique])
        union_low=np.minimum(low[:,None],low[None]);union_high=np.maximum(high[:,None],high[None])
        volumes=np.prod(high-low,axis=1)
        cost=np.prod(union_high-union_low,axis=2)-volumes[:,None]-volumes[None]
        cost[np.tril_indices(len(unique))]=np.inf
        a,b=np.unravel_index(np.argmin(cost),cost.shape)
        unique[a]=(union_low[a,b],union_high[a,b]);unique.pop(b)
    return [dict(name=f'{prefix}{scene.geom(gid).name}/part_{i:03d}',
                 position=((low+high)/2).tolist(), dimensions=(high-low).tolist())
            for i, (low, high) in enumerate(unique)]
