"""The biggest colored blob is not always the block.

Caught live 2026-07-18: the wood house out-browns the brown block, so a
furniture contour won largest-blob every frame and the block never
published. The detector must offer every above-threshold blob, biggest
first, and let the size-distance gate pick. Needs cv2/numpy but no ROS.
"""

from __future__ import annotations

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from perception_node.color_block_detector import blob_candidates  # noqa: E402


def _mask_with_boxes(boxes):
    mask = np.zeros((480, 640), dtype=np.uint8)
    for x, y, w, h in boxes:
        mask[y:y + h, x:x + w] = 255
    return mask


def test_all_qualifying_blobs_returned_biggest_first():
    # scenery-sized smear plus a block-sized square
    mask = _mask_with_boxes([(50, 0, 130, 45), (300, 220, 40, 40)])
    cands = blob_candidates(mask, min_area=400)
    assert len(cands) == 2
    areas = [c[2] for c in cands]
    assert areas[0] > areas[1]
    # runner-up centroid sits inside the small square
    u, v, _ = cands[1]
    assert 300 <= u <= 340 and 220 <= v <= 260


def test_tiny_speckle_excluded():
    mask = _mask_with_boxes([(300, 220, 40, 40), (10, 10, 8, 8)])
    cands = blob_candidates(mask, min_area=400)
    assert len(cands) == 1


def _in_band(bands, hsv_px):
    px = np.uint8([[list(hsv_px)]])
    return any(cv2.inRange(px, np.array(lo), np.array(hi))[0, 0] == 255
               for lo, hi in bands)


def test_orange_brown_separate_on_saturation():
    """Orange and brown share hue; only saturation splits them.

    Pixel values below are masked-pixel medians measured in the live sim
    (2026-07-18) after a 'brown' sighting sent the robot to the orange
    block. Any band edit that breaks these is reintroducing that bug.
    """
    from perception_node.color_block_detector import COLOR_BANDS

    orange_lit = (15, 255, 255)     # orange block, any face
    brown_lit = (13, 197, 255)      # brown block, top face in light
    brown_shaded = (13, 199, 127)   # brown block, shaded face
    wood_floor = (20, 86, 250)

    assert _in_band(COLOR_BANDS["orange"], orange_lit)
    assert not _in_band(COLOR_BANDS["brown"], orange_lit)
    for px in (brown_lit, brown_shaded):
        assert _in_band(COLOR_BANDS["brown"], px)
        assert not _in_band(COLOR_BANDS["orange"], px)
    assert not _in_band(COLOR_BANDS["brown"], wood_floor)
    assert not _in_band(COLOR_BANDS["orange"], wood_floor)
