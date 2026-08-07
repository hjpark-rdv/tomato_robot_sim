import numpy as np

from rbpodo_tomato_harvest.tomato_tf_generator import (
    camera_target_id,
    clustered_height_order,
    descending_height_order,
    detection_cluster_id,
    harvest_tf_frame_id,
    parent_frame_tomato_rotation,
    quaternion_from_rotation,
    rotation_from_quaternion,
)


def test_detection_ids_are_assigned_from_highest_z_down():
    points = [
        (0.30, 0.10, 0.45),
        (-0.20, 0.05, 0.92),
        (0.10, -0.10, 0.70),
    ]

    assert descending_height_order(points) == [1, 2, 0]


def test_equal_height_detection_order_uses_xy_as_stable_tie_breaker():
    points = [
        (0.20, 0.10, 0.80),
        (-0.10, 0.20, 0.80),
        (-0.10, -0.20, 0.80),
    ]

    assert descending_height_order(points) == [2, 1, 0]


def test_detection_cluster_id_reads_vision_path_component():
    identifier = "20260806_180654_902_0001/cluster_17/tomato_3"

    assert detection_cluster_id(identifier) == "cluster_17"
    assert detection_cluster_id("tomato_3") == "tomato_3"


def test_camera_target_suffix_defines_harvest_tf_and_cluster():
    identifier = "20260807/session/cluster-result-C0:T7"

    assert camera_target_id(identifier) == "C0:T7"
    assert harvest_tf_frame_id(identifier, 3) == "C0:T7"
    assert detection_cluster_id(identifier) == "c0"
    assert harvest_tf_frame_id("legacy_tomato", 3) == (
        "detected_tomato_3_tf"
    )


def test_clusters_use_summed_height_before_internal_height_order():
    points = [
        (0.0, 0.0, 0.90),  # cluster 1 total = 1.00
        (0.0, 0.0, 0.10),
        (0.2, 0.0, 0.70),  # cluster 2 total = 1.30
        (0.2, 0.0, 0.60),
    ]
    detection_ids = [
        "capture/cluster_1/tomato_0",
        "capture/cluster_1/tomato_1",
        "capture/cluster_2/tomato_0",
        "capture/cluster_2/tomato_1",
    ]

    assert clustered_height_order(points, detection_ids) == [2, 3, 0, 1]


def test_camera_cluster_targets_stay_together_then_sort_by_height():
    points = [
        (0.0, 0.0, 0.95),  # C0
        (0.0, 0.0, 0.40),  # C1
        (0.0, 0.0, 0.90),  # C1
        (0.0, 0.0, 0.30),  # C0
    ]
    detection_ids = [
        "capture-C0:T7",
        "capture-C1:T2",
        "capture-C1:T3",
        "capture-C0:T8",
    ]

    assert clustered_height_order(points, detection_ids) == [2, 1, 0, 3]


def test_unclustered_detections_keep_global_height_order():
    points = [
        (0.0, 0.0, 0.50),
        (0.0, 0.0, 0.90),
        (0.0, 0.0, 0.70),
    ]

    assert clustered_height_order(
        points,
        ["tomato_0", "tomato_1", "tomato_2"],
    ) == [1, 2, 0]


def test_z_points_to_sky_and_x_points_to_horizontal_stem():
    rotation = parent_frame_tomato_rotation(
        sky_axis_in_parent=[0.0, 0.0, 1.0],
        stem_direction_in_parent=[2.0, 1.0, 3.0],
    )

    expected_x = np.array([2.0, 1.0, 0.0])
    expected_x /= np.linalg.norm(expected_x)
    assert np.allclose(rotation[:, 0], expected_x)
    assert np.allclose(rotation[:, 2], [0.0, 0.0, 1.0])
    assert np.allclose(rotation.T @ rotation, np.eye(3))
    assert np.linalg.det(rotation) > 0.999999


def test_camera_rotation_does_not_change_world_sky_constraint():
    sky_in_camera = np.array([0.0, -1.0, 0.0])
    stem_in_camera = np.array([1.0, 0.5, 1.0])
    rotation = parent_frame_tomato_rotation(sky_in_camera, stem_in_camera)

    assert np.allclose(rotation[:, 2], sky_in_camera)
    assert abs(np.dot(rotation[:, 0], sky_in_camera)) < 1e-9
    assert np.dot(rotation[:, 0], stem_in_camera) > 0.0


def test_rotation_quaternion_round_trip():
    rotation = parent_frame_tomato_rotation(
        sky_axis_in_parent=[0.2, -0.8, 0.55],
        stem_direction_in_parent=[0.7, 0.4, 0.1],
    )
    reconstructed = rotation_from_quaternion(quaternion_from_rotation(rotation))
    assert np.allclose(reconstructed, rotation)
