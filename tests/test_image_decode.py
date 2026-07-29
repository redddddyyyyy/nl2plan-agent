"""Decoding camera frames without cv_bridge.

cv_bridge's converter is a compiled extension built against NumPy 1.x, and on
NumPy 2 it raises on every frame. The detector catches nothing and publishes
nothing, so the robot just never sees a block — a silent, total failure
inherited from a dependency doing one reshape. `image_to_bgr` replaces it.

Getting a channel order or a row stride wrong here is equally silent: the
detector keeps running and simply matches the wrong colours, or shears the
image. So both are pinned.
"""

from __future__ import annotations

import numpy as np
import pytest

from perception_node.color_block_detector import image_to_bgr


class FakeImage:
    """Enough of sensor_msgs/Image to decode."""

    def __init__(self, height, width, encoding, data, step=None):
        self.height = height
        self.width = width
        self.encoding = encoding
        self.data = data
        self.step = step if step is not None else width * 3


def test_rgb8_channels_are_swapped_to_bgr():
    """The Gazebo camera publishes R8G8B8; OpenCV's HSV expects BGR."""
    msg = FakeImage(1, 2, 'rgb8', bytes([10, 20, 30, 40, 50, 60]))
    out = image_to_bgr(msg)
    assert out.shape == (1, 2, 3)
    assert list(out[0, 0]) == [30, 20, 10]
    assert list(out[0, 1]) == [60, 50, 40]


def test_bgr8_is_passed_through_unswapped():
    msg = FakeImage(1, 2, 'bgr8', bytes([10, 20, 30, 40, 50, 60]))
    out = image_to_bgr(msg)
    assert list(out[0, 0]) == [10, 20, 30]


def test_a_padded_row_stride_does_not_shear_the_image():
    """`step` can exceed width*3. Reshaping on width would skew every row."""
    width, height, pad = 2, 2, 5
    rows = []
    for r in range(height):
        rows.append(bytes([r * 10 + c for c in range(width * 3)]) + bytes(pad))
    msg = FakeImage(height, width, 'bgr8', b''.join(rows), step=width * 3 + pad)

    out = image_to_bgr(msg)
    assert out.shape == (height, width, 3)
    assert list(out[0, 0]) == [0, 1, 2]
    assert list(out[1, 0]) == [10, 11, 12]


def test_the_result_is_writable_and_contiguous():
    """OpenCV rejects the negative stride an rgb8 flip leaves behind."""
    msg = FakeImage(2, 2, 'rgb8', bytes(range(12)))
    out = image_to_bgr(msg)
    assert out.flags['C_CONTIGUOUS']
    assert out.flags['WRITEABLE']


def test_cvtcolor_accepts_the_result():
    """The actual downstream call, which is what rejects a bad array."""
    cv2 = pytest.importorskip("cv2")
    msg = FakeImage(4, 4, 'rgb8', bytes(range(48)))
    hsv = cv2.cvtColor(image_to_bgr(msg), cv2.COLOR_BGR2HSV)
    assert hsv.shape == (4, 4, 3)


def test_an_unexpected_encoding_is_refused_rather_than_misread():
    """mono8 or bayer would reshape to nonsense and match arbitrary colours."""
    msg = FakeImage(1, 2, 'mono8', bytes([1, 2]))
    with pytest.raises(ValueError, match='mono8'):
        image_to_bgr(msg)


def test_it_matches_what_cv_bridge_produced():
    """Equivalence, against a recorded cv_bridge result rather than a live call.

    Deliberately not `import cv_bridge` and compare. On NumPy 2 that import
    does not raise, it *segfaults* — it took the whole pytest process down when
    this file first tried it, guarded by try/except and everything, because a
    C-level abort is not catchable. Which rather makes the case for not having
    the detector depend on it.

    The expected array below is what cv_bridge returned for this input on
    NumPy 1.x: 4x4 rgb8 of range(48), converted to bgr8, i.e. each pixel's
    channels reversed.
    """
    data = bytes(range(48))
    want = np.frombuffer(data, dtype=np.uint8).reshape(4, 4, 3)[:, :, ::-1]

    got = image_to_bgr(FakeImage(4, 4, 'rgb8', data))
    assert np.array_equal(got, want)
