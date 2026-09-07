import numpy as np
import pytest
from PIL import Image

from rbpodo_tomato_harvest.laboro_mask_worker import render_laboro_masks


def test_render_laboro_masks_overlays_only_accepted_cherry_instances():
    source = Image.new("RGB", (100, 80), color=(100, 100, 100))
    masks = np.zeros((2, 80, 100), dtype=bool)
    masks[0, 35:55, 20:40] = True
    masks[1, 40:60, 65:85] = True

    output, summary = render_laboro_masks(
        source,
        masks,
        labels=[0, 2],
        scores=[0.95, 0.20],
        score_threshold=0.50,
    )

    assert output.size == source.size
    assert output.getpixel((30, 45)) != source.getpixel((30, 45))
    assert output.getpixel((75, 50)) == source.getpixel((75, 50))
    assert summary["count"] == 1
    assert summary["class_counts"] == {
        "l_fully_ripened": 1,
        "l_half_ripened": 0,
        "l_green": 0,
    }
    assert summary["maximum_score"] == pytest.approx(0.95)


def test_render_laboro_masks_rejects_wrong_mask_geometry():
    with pytest.raises(ValueError, match="do not match"):
        render_laboro_masks(
            Image.new("RGB", (20, 12)),
            np.zeros((1, 10, 10), dtype=bool),
            labels=[0],
            scores=[0.9],
        )
