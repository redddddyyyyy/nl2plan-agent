"""The block's size is load-bearing, so the constraints that set it are asserted.

Three independent limits pin the block, and every one of them is invisible in
the code that depends on it:

  * the arm cannot reach a short block at all,
  * the gripper cannot open wide enough for a fat one,
  * and the lidar turns a tall one into an obstacle Nav2 will not approach.

The middle of that window is narrow. Someone tuning the block for a nicer
camera image would find each of these the hard way — a mission that fails at
the grasp, a gripper that closes through the block, or a robot that plans a
path around the thing it was sent to fetch. So they are written down here.
"""

from __future__ import annotations

import math

import pytest

from nl2plan_agent.ros_backend import kinematics as kin
from nl2plan_agent.ros_backend.kinematics import ARM
from perception_node.block_spec import BLOCK_D, BLOCK_H, BLOCK_W
from perception_node.color_block_detector import DIST_MIN, GROUND_Z

# From mobile_arm.urdf.xacro. The finger boxes are 0.008 wide and hang off
# origins at -+0.018, so their inner faces start 0.028 m apart, and
# node.gripper() drives them symmetrically by +-half_opening.
FINGER_REST_GAP = 0.028
FINGER_TRAVEL = 0.005      # symmetric limit, capped by the +0.005 joint stop

LIDAR_PLANE_Z = 0.200      # single-plane scan, base_link 0.13 + 0.05 + 0.02
INFLATION_RADIUS = 0.55    # nav2_params.yaml, both costmaps

GRASP_Z = BLOCK_H / 2


def test_the_arm_can_reach_the_block_with_joint_margin():
    """The reason the block is tall. A 50 mm cube is unreachable at any range."""
    solved = None
    for i in range(120):
        x = 0.15 + i * 0.002
        for pitch_deg in range(-90, 10):
            q = kin.inverse(x, 0.0, GRASP_Z, math.radians(pitch_deg))
            if q is None:
                continue
            margin = min(ARM.lift_limit - abs(q[1]),
                         ARM.elbow_limit - abs(q[2]),
                         ARM.wrist_limit - abs(q[3]))
            if margin >= 0.20:
                solved = (x, q, margin)
    assert solved is not None, (
        f"nothing can grasp a block at z={GRASP_Z} with joint margin to spare")
    x, q, margin = solved
    assert x >= 0.25, "wanted at least 0.25 m of standoff to grasp from"


def test_a_five_centimetre_cube_would_still_be_unreachable():
    """Guards the reason the block changed, so it cannot quietly change back."""
    for i in range(100):
        x = 0.05 + i * 0.004
        for pitch_deg in range(-90, 91, 5):
            assert not kin.reachable(x, 0.0, 0.025, math.radians(pitch_deg))


def test_the_gripper_can_open_wider_than_the_block():
    """The reason the block is narrow. Symmetric travel is capped at +-5 mm."""
    widest = FINGER_REST_GAP + 2 * FINGER_TRAVEL
    assert widest == pytest.approx(0.038, abs=1e-9)
    assert BLOCK_W < widest, "the fingers cannot get around the block"
    assert (widest - BLOCK_W) / 2 >= 0.003, "wanted at least 3 mm either side"


def test_the_gripper_closes_onto_the_block_rather_than_through_it():
    """Closed has to squeeze, but not so far the fingers pass through."""
    manipulation = pytest.importorskip(
        "nl2plan_agent.ros_backend.manipulation", reason="needs rclpy")

    open_gap = FINGER_REST_GAP + 2 * manipulation.GRIPPER_OPEN
    closed_gap = FINGER_REST_GAP + 2 * manipulation.GRIPPER_CLOSED

    assert open_gap > BLOCK_W, "GRIPPER_OPEN does not clear the block"
    assert closed_gap < BLOCK_W, "GRIPPER_CLOSED never touches the block"
    assert BLOCK_W - closed_gap <= 0.006, "closing that far drives through it"


def test_the_block_stays_below_the_lidar_plane():
    """A block the lidar can see is an obstacle with a 0.55 m inflation skirt.

    The robot has to stand about 0.25 m from the block to grasp it, which is
    deep inside that skirt, so a visible block is one Nav2 refuses to approach.
    """
    assert BLOCK_H < LIDAR_PLANE_Z
    assert LIDAR_PLANE_Z - BLOCK_H >= 0.03, "wanted 30 mm of clearance"
    assert INFLATION_RADIUS > 0.25, "the premise of this test, stated"


def test_the_block_is_square_in_plan():
    """Orientation is never commanded, so it must not matter which face is on."""
    assert BLOCK_W == BLOCK_D


def test_the_projection_plane_tracks_the_block():
    """GROUND_Z is the block's mid-height; a stale value biases every sighting."""
    assert GROUND_Z == pytest.approx(BLOCK_H / 2, abs=1e-12)


def test_the_camera_sees_the_block_before_the_arm_needs_it():
    """What removes most of the blind creep.

    A block enters the frame when its TOP does, so a tall one appears closer.
    The detector's near gate has to sit outside the true optical edge, and the
    grasp distance has to sit inside it, or the base is creeping blind again.
    """
    cam_x, cam_z, cam_pitch = 0.20, 0.23, 0.349
    hfov, width, height = 1.2, 640, 480
    vfov = 2 * math.atan(math.tan(hfov / 2) * height / width)
    lower_edge = cam_pitch + vfov / 2

    first_seen = cam_x + (cam_z - BLOCK_H) / math.tan(lower_edge)
    assert first_seen < 0.30, "the block is not visible until too close"
    assert DIST_MIN < first_seen, "the near gate cuts off live sightings"

    manipulation = pytest.importorskip(
        "nl2plan_agent.ros_backend.manipulation", reason="needs rclpy")
    blind = first_seen - manipulation.GRASP_REACH
    assert 0.0 <= blind <= 0.06, f"blind creep is {blind:.3f} m"


def test_grasp_reach_is_where_the_fixed_pose_actually_goes():
    """GRASP_REACH has to name a real position, which it did not before today."""
    manipulation = pytest.importorskip(
        "nl2plan_agent.ros_backend.manipulation", reason="needs rclpy")

    x, _, z = kin.forward(tuple(manipulation.GRASP))
    assert manipulation.GRASP_REACH == pytest.approx(x, abs=0.006)

    # And the pads have to overlap the block they are closing on.
    overlap = min(z + 0.025, BLOCK_H) - max(z - 0.025, 0.0)
    assert overlap >= 0.02, f"only {overlap * 1000:.0f} mm of pad on the block"
