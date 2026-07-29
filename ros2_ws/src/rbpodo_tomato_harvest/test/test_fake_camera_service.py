import pytest

from rbpodo_tomato_harvest.fake_camera_service import vertical_harvest_order


def test_vertical_order_finishes_first_xy_column_before_higher_other_column():
    points = [
        (0.00, 0.00, 1.00),
        (0.01, 0.00, 0.75),
        (0.02, 0.01, 0.45),
        (0.30, 0.00, 0.90),
        (0.31, 0.01, 0.55),
    ]

    order = vertical_harvest_order(points, xy_tolerance=0.05)

    assert order == [0, 1, 2, 3, 4]


def test_vertical_order_always_starts_at_global_highest_tomato():
    points = [
        (-0.20, 0.00, 0.80),
        (0.20, 0.00, 1.20),
        (0.21, 0.01, 0.60),
    ]

    order = vertical_harvest_order(points, xy_tolerance=0.05)

    assert order == [1, 2, 0]


def test_vertical_order_chooses_nearest_next_xy_column():
    points = [
        (0.00, 0.00, 1.00),
        (0.00, 0.00, 0.50),
        (0.40, 0.00, 0.90),
        (-0.20, 0.00, 0.70),
    ]

    order = vertical_harvest_order(points, xy_tolerance=0.05)

    assert order == [0, 1, 3, 2]


def test_vertical_order_is_independent_of_camera_input_order():
    points = [
        (0.30, 0.00, 0.55),
        (0.00, 0.00, 0.45),
        (0.30, 0.00, 0.90),
        (0.00, 0.00, 1.00),
    ]

    order = vertical_harvest_order(points, xy_tolerance=0.05)

    assert order == [3, 1, 2, 0]


def test_vertical_order_rejects_invalid_tolerance():
    with pytest.raises(ValueError, match="xy_tolerance"):
        vertical_harvest_order([(0.0, 0.0, 1.0)], xy_tolerance=-0.01)
