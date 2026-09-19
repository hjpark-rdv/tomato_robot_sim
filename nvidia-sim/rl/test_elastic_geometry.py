import numpy as np
import pytest
from elastic_geometry import (tube_centerline, resample_rod, point_segment_distances,
                              bind_terminal_frame, skin_points, weld_tube_end, vertex_normals)


def test_original_style_split_vertices_recover_centerline():
    angles=np.arange(12)*2*np.pi/12
    centers=np.array([[0.,0.,0.],[.0002,0.,.006],[.0005,0.,.012]])
    ring=np.column_stack([.001*np.cos(angles),.001*np.sin(angles),np.zeros(12)])
    vertices=np.repeat((centers[:,None]+ring).reshape(-1,3),3,axis=0)
    recovered,radii=tube_centerline(vertices,12)
    np.testing.assert_allclose(recovered,centers,atol=1e-12)
    np.testing.assert_allclose(radii,.001)


def test_topology_change_fails_instead_of_building_wrong_rods():
    with pytest.raises(ValueError,match='topology'):
        tube_centerline(np.arange(75).reshape(-1,3),12)


def test_arc_length_resampling_preserves_endpoints():
    p,r=resample_rod([[0,0,0],[0,0,.01],[.01,0,.01]],[.002,.0015,.001],4)
    np.testing.assert_allclose(p,[[0,0,0],[0,0,.005],[0,0,.01],[.005,0,.01],[.01,0,.01]])
    np.testing.assert_allclose(r[[0,-1]],[.002,.001])


def test_distance_clamps_to_capsule_end_and_checks_side():
    d=point_segment_distances([[0,0,.03],[.01,0,.01]],[[0,0,0]],[[0,0,.02]])
    np.testing.assert_allclose(d,[[.01],[.01]])


def test_degenerate_rod_rejected():
    with pytest.raises(ValueError):
        resample_rod([[0,0,0],[0,0,0]],[.001,.001],3)


def test_terminal_surface_follows_attachment_even_with_tilted_cap():
    # A curved source tube's end ring can lie off the coarse segment's plane.
    points=np.array([[.001,0,.0198],[-.001,0,.0202],[0,0,.01]])
    ids=np.tile([0,1],(3,1)); weights=np.tile([.2,.8],(3,1))
    ids,weights=bind_terminal_frame(points,ids,weights,np.array([0,0,.015]),
                                   np.array([0,0,.02]),2,cap_ids=[0,1])
    np.testing.assert_allclose(weights.sum(axis=1),1.)
    np.testing.assert_allclose(weights[:2],[[0,0,1],[0,0,1]])
    np.testing.assert_allclose(weights[2],[.2,.8,0])
    # Rotate and translate the attachment independently of the preceding rods.
    from scipy.spatial.transform import Rotation
    rotations=Rotation.from_euler('y',[0,10,30],degrees=True).as_matrix()
    rest=np.array([[0,0,.005],[0,0,.015],[0,0,.02]])
    positions=rest+np.array([[0,0,0],[.0002,0,0],[.0005,0,0]])
    actual=skin_points(points,rest,rotations,positions,ids,weights)
    expected=(points[:2]-rest[2])@rotations[2].T+positions[2]
    np.testing.assert_allclose(actual[:2],expected,atol=1e-12)


def test_skinning_preserves_authored_geometry_at_rest():
    points=np.array([[.001,0,.0198],[-.001,0,.0202]])
    rest=np.array([[0,0,.005],[0,0,.015],[0,0,.02]])
    ids,weights=bind_terminal_frame(points,np.tile([0,1],(2,1)),
        np.tile([.2,.8],(2,1)),rest[1],rest[2],2)
    actual=skin_points(points,rest,np.tile(np.eye(3),(3,1,1)),rest,ids,weights)
    np.testing.assert_allclose(actual,points,atol=1e-12)


def test_coincident_centers_need_surface_welding_for_tilted_cut_planes():
    from scipy.spatial.transform import Rotation
    theta=np.arange(12)*2*np.pi/12
    circle=np.column_stack([.0006*np.cos(theta),.0006*np.sin(theta),np.zeros(12)])
    rings=circle[None]+np.column_stack([np.zeros(8),np.zeros(8),np.linspace(0,.02,8)])[:,None]
    target=Rotation.from_euler('x',70,degrees=True).apply(circle*.96)+[0,0,.02]
    # This regression must fail a center-only seam check: centers already match.
    np.testing.assert_allclose(rings[-1].mean(0),target.mean(0),atol=1e-12)
    assert np.linalg.norm(rings[-1][:,None]-target[None],axis=2).min(1).max()>.0005
    original=np.repeat(rings.reshape(-1,3),3,axis=0)
    result=weld_tube_end(original,target)
    cap=result[-36::3]
    np.testing.assert_allclose(np.linalg.norm(cap[:,None]-target[None],axis=2).min(1),0,atol=1e-12)
    np.testing.assert_allclose(result[:36],original[:36])
    np.testing.assert_allclose(result[::3],result[1::3])


def test_normals_follow_repaired_surface():
    points=np.array([[0.,0.,0.],[1.,0.,0.],[1.,1.,1.],[0.,1.,1.]])
    normals=vertex_normals(points,[4],[0,1,2,3])
    np.testing.assert_allclose(normals,np.tile([0,-1/np.sqrt(2),1/np.sqrt(2)],(4,1)))
