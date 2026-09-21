import numpy as np
from scipy.spatial.transform import Rotation
from display_skin import DisplaySkin
from elastic_geometry import skin_points


def test_cached_skin_preserves_multi_bone_seams_positions_and_normals():
    rng=np.random.default_rng(42)
    points=rng.normal(size=(37,3));rest=rng.normal(size=(8,3))
    ids=rng.integers(0,8,size=(37,4));weights=rng.uniform(size=ids.shape)
    weights[:10,1:]=0  # Fully attached end rings, including repeated bone IDs.
    weights/=weights.sum(axis=1,keepdims=True)
    normals=rng.normal(size=points.shape)
    kernel=DisplaySkin(points,rest,ids,weights,normals)
    for _ in range(3):
        rotations=Rotation.random(8,random_state=rng).as_matrix();positions=rng.normal(size=rest.shape)
        actual,n=kernel.evaluate(DisplaySkin.transforms(rotations,positions))
        expected=skin_points(points,rest,rotations,positions,ids,weights)
        expected_n=(np.einsum('nkij,nj->nki',rotations[ids],normals)*weights[:,:,None]).sum(axis=1)
        np.testing.assert_allclose(actual,expected,atol=1e-12)
        np.testing.assert_allclose(n,expected_n,atol=1e-12)
