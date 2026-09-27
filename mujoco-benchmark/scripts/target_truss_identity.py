"""Exact identities in the existing single/multiple-GLB truss namespaces.

Names are checked against body ancestry before a rachis contact is authorized.
A same-numbered rachis on another truss, main stem, or mounting hardware is not
included. This module never changes a collision mask, shape, or material.
"""
from __future__ import annotations
import re


def target_names(target, anchor):
    match = re.fullmatch(r'(?P<prefix>truss_[0-9]+__)?Tomato_(?P<index>[0-9]+)', target)
    if not match:
        raise ValueError('Unsupported target namespace; require explicit mapping')
    prefix, index = match['prefix'] or '', match['index']
    anchor_name = anchor.rsplit('/', 1)[-1]
    if not re.fullmatch(re.escape(prefix)+r'Attachment_[0-9]+', anchor_name):
        raise ValueError('Target and attachment namespaces differ')
    return dict(prefix=prefix, fruit=prefix+'glb_col_Tomato_'+index,
                pedicels=(prefix+'glb_col_'+anchor_name[len(prefix):],
                          prefix+'glb_col_TRUSS_Pedicel_proximal_'+index+'_02'),
                truss_root=prefix+'TRUSS_Truss_01_Peduncle_00')


def ancestors(model, body):
    result = set()
    while body:
        if body in result:
            raise ValueError('Cyclic body ancestry')
        result.add(int(body))
        body = int(model.body_parentid[body])
    return result


def target_rachis_geoms(engine):
    """Select ONLY rachis colliders owned by the target's actual truss root."""
    names = target_names(engine.target, engine.target_spec['anchor'])
    model = engine.model
    root = model.body(names['truss_root']).id
    if root not in ancestors(model, engine.fruit):
        raise ValueError('Target is not a descendant of its declared truss root')
    pattern = re.compile(re.escape(names['prefix'])+r'glb_col_TRUSS_Rachis_[0-9]+')
    found = []
    for gid in range(model.ngeom):
        name = model.geom(gid).name or ''
        if not pattern.fullmatch(name):
            continue
        if root not in ancestors(model, int(model.geom_bodyid[gid])):
            raise ValueError('Rachis name matches but body ownership differs')
        if int(model.geom_contype[gid]) or int(model.geom_conaffinity[gid]):
            found.append(name)
    if not found:
        raise ValueError('No active rachis colliders for the target truss; review mapping')
    return tuple(sorted(found)), names['truss_root']
