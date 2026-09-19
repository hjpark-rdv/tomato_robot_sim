"""Geometry helpers for the original truss's spring-linked rod approximation."""
import numpy as np


def tube_centerline(vertices, sides):
    """Recover ordered rings from the source's split-normal tube vertices."""
    vertices = np.asarray(vertices, dtype=float)
    _, indices = np.unique(np.round(vertices, 7), axis=0, return_index=True)
    unique = vertices[np.sort(indices)]
    if len(unique) % sides or len(unique) < sides * 2:
        raise ValueError('Source tube topology changed; cannot recover its rings')
    rings = unique.reshape(-1, sides, 3)
    centers = rings.mean(axis=1)
    radii = np.linalg.norm(rings - centers[:, None], axis=2).mean(axis=1)
    if np.any(radii <= 0) or np.max(radii) > .01:
        raise ValueError('Invalid original stem cross section')
    return centers, radii


def resample_rod(points, radii, segments):
    points = np.asarray(points, dtype=float)
    lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if lengths[-1] <= 0 or np.any(np.diff(lengths) <= 0):
        raise ValueError('Rod centerline must contain distinct consecutive points')
    samples = np.linspace(0., lengths[-1], segments + 1)
    return (np.column_stack([np.interp(samples, lengths, points[:, i]) for i in range(3)]),
            np.interp(samples, lengths, radii))


def point_segment_distances(points, starts, ends):
    points, starts, ends = map(np.asarray, (points, starts, ends))
    delta = ends - starts
    u = np.clip(np.einsum('nsi,si->ns', points[:, None] - starts, delta)
                / np.maximum(np.sum(delta * delta, axis=1), 1e-12), 0., 1.)
    return np.linalg.norm(points[:, None] - (starts + u[:, :, None] * delta), axis=2)


def bind_terminal_frame(points, ids, weights, last_center, endpoint, frame_id, cap_ids=()):
    """Blend the last half rod into its attachment frame; weld the cut surface.

    Distance-only weights leave other rods influencing the cut surface, opening
    a visible crack even when the physics attachment remains intact. Explicit
    cap indices also handle curved tubes whose final ring is not perpendicular
    to the coarser physics segment.
    """
    direction = np.asarray(endpoint) - last_center
    length_sq = float(direction @ direction)
    if length_sq <= 1e-14:
        raise ValueError('Terminal frame must be separated from the last rod center')
    u = np.clip((np.asarray(points)-last_center) @ direction / length_sq, 0., 1.)
    blend = u*u*(3.-2.*u)
    blend[np.asarray(cap_ids, dtype=int)] = 1.
    return (np.column_stack([ids, np.full(len(points), frame_id, dtype=int)]),
            np.column_stack([weights*(1.-blend[:, None]), blend]))


def skin_points(points, rest_positions, rotations, positions, ids, weights):
    """World-space skinning with rotations relative to the authored rest pose."""
    transformed = np.einsum('nkij,nkj->nki', rotations[ids], points[:, None]-rest_positions[ids])
    return ((transformed+positions[ids])*weights[:, :, None]).sum(axis=1)


def weld_tube_end(vertices, target_ring, sides=12, blend_rings=6):
    """Match the authored cut surfaces, tapering the correction into the tube.

    Coincident ring centers do not imply a closed surface: the source's proximal
    and distal rings have different planes. Preserve cyclic vertex order while
    matching the last ring exactly to the fruit-side boundary. No source file is
    modified; callers apply this small rest-mesh repair in the runtime stage.
    """
    vertices, target_ring = np.asarray(vertices), np.asarray(target_ring)
    _, first, inverse = np.unique(np.round(vertices,7),axis=0,return_index=True,return_inverse=True)
    ordered = np.argsort(first)
    unique = vertices[first[ordered]]
    if len(unique)%sides or len(unique)<2*sides or target_ring.shape!=(sides,3):
        raise ValueError('Cannot weld changed source tube topology')
    cap = unique[-sides:]
    candidates = [np.roll(ring,shift,axis=0) for ring in (target_ring,target_ring[::-1])
                  for shift in range(sides)]
    matched = min(candidates,key=lambda ring: np.square(ring-cap).sum())
    remap = np.empty(len(ordered),dtype=int); remap[ordered]=np.arange(len(ordered))
    ids = remap[inverse]
    last = len(unique)//sides-1
    span = min(blend_rings,last)
    if span<1:
        raise ValueError('Weld blend must span at least one tube ring')
    u=np.clip((ids//sides-(last-span))/span,0.,1.)
    blend=u*u*(3.-2.*u)
    return vertices+blend[:,None]*(matched-cap)[ids%sides]


def vertex_normals(points, face_counts, face_indices):
    """Recompute vertex normals after the local rest-shape repair."""
    triangles=[]; start=0
    for count in face_counts:
        face=face_indices[start:start+count]; start+=count
        triangles.extend((face[0],face[i],face[i+1]) for i in range(1,count-1))
    triangles=np.asarray(triangles,dtype=int)
    points=np.asarray(points)
    a,b,c=points[triangles].transpose(1,0,2)
    normals=np.zeros_like(points,dtype=float)
    area=np.cross(b-a,c-a)
    for column in triangles.T:
        np.add.at(normals,column,area)
    return normals/np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
