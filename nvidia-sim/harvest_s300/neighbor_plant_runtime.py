"""Additional exact plant views/preloads for bounded, one-world A/B trials."""
import numpy as np


def gravity_preload(jac, masses, link_names, plant):
    """The existing fixed authored-pose preload, including external fruit mass."""
    preload = np.zeros(jac.shape[-1])
    for i, mass in enumerate(masses):
        if i > 0:
            preload -= jac[i-1, :3].T @ np.array([0., 0., -9.81*mass])
    rows = {b['path']: b for b in plant['bodies']}
    for fruit in plant['fruits']:
        i = link_names.index(fruit['anchor'].rsplit('/', 1)[1])
        if i == 0:
            raise ValueError('Fruit anchored to fixed root is not supported by existing preload')
        force = np.array([0., 0., -9.81*fruit['mass']])
        offset = np.array(fruit['center']) - rows[fruit['anchor']]['rest_position']
        preload -= jac[i-1, :3].T @ force + jac[i-1, 3:].T @ np.cross(offset, force)
    if not np.isfinite(preload).all():
        raise ValueError('Nonfinite neighbor preload')
    return preload


def initialize(view, manifest, *, arr, tensor, indices):
    from compliant_foliage import armature_values
    handles=[]; records=[]
    for row in manifest.get('elastic_neighbors', []):
        plant = view.create_articulation_view([row['root']])
        if plant.count != 1:
            raise ValueError('Neighbor articulation identity mismatch: '+row['root'])
        names=list(plant.shared_metatype.dof_names)
        expected=armature_values(row, names)
        if not np.allclose(arr(plant.get_dof_armatures()), expected[None], rtol=0., atol=1e-10):
            raise ValueError('Neighbor native armature mismatch')
        preload=gravity_preload(arr(plant.get_jacobians())[0], arr(plant.get_masses())[0],
            list(plant.shared_metatype.link_names), row['plant'])
        native=tensor(preload[None].copy())
        plant.set_dof_actuation_forces(native,indices)
        handles.append((plant,native))
        records.append(dict(root=row['root'],source_plant=row['source_plant'],
            dof_count=plant.max_dofs,dof_names=names,preload=preload.tolist(),
            initial_q=arr(plant.get_dof_positions())[0].tolist(),
            initial_qvel=arr(plant.get_dof_velocities())[0].tolist(),
            method='Same fixed authored-pose gravity preload as active target; no continuous gravity cancellation'))
    return handles,records
