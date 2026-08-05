"""pick()'s final approach must never grasp a block it can't re-confirm.

Live failure mode (Rajeev, 2026-07-20/21): a search-scan sighting can land
several tenths of a meter off truth (0.55 m measured at the lounge,
09:02 run). When the standoff re-confirm then fails, the old code fell
back to the stale map coordinate, crept to empty floor, and fired the
magic grasp anyway — on video the block visibly teleports into the
gripper. A refused pick recovers in one find_object; a teleport ruins
the demo take.
"""

from __future__ import annotations

import pytest

# `manipulation` imports geometry_msgs, so this whole module needs a ROS
# environment. Skip rather than let the ImportError escape: an unhandled one
# here is a *collection* error, which aborts the entire run with exit 2 and
# reports no tests at all — so a machine without ROS sees the suite as broken
# rather than partly skipped. requirements.txt promises the agent side runs
# without ROS, and every other ROS-dependent module already skips cleanly.
pytest.importorskip(
    "geometry_msgs",
    reason="needs ROS2 message packages; source the ROS environment to run this")

from nl2plan_agent.ros_backend import backend as backend_mod
from nl2plan_agent.ros_backend import manipulation, nav, perception
from nl2plan_agent.ros_backend.backend import RosBackend


class FakeNode:
    def __init__(self, robot=(0.0, 0.0, 0.0)):
        self.robot = robot
        self.odom = robot


def make_backend(robot=(0.0, 0.0, 0.0), det_xy=(1.0, 0.0)):
    b = RosBackend.__new__(RosBackend)
    b._node = FakeNode(robot)
    b._nav_checked = True
    b._holding = None
    b._last_detection = {
        "color": "magenta",
        "entity": "distractor_magenta",
        "object_id": "magenta_block",
        "x": det_xy[0],
        "y": det_xy[1],
    }
    b._poses = {}
    return b


@pytest.fixture
def grasped(monkeypatch):
    """Record whether grasp_sequence fired; it must never fire on a refusal."""
    calls = []
    monkeypatch.setattr(manipulation, "grasp_sequence",
                        lambda node, entity: calls.append(entity))
    return calls


def _nav_teleports_robot(backend):
    """Stand-in for Nav2: report success and move the fake robot to the goal."""
    def fake_navigate(node, x, y, theta):
        node.robot = (x, y, theta)
        node.odom = (x, y, theta)
        return {"success": True, "final_pose": {"x": x, "y": y, "theta": theta},
                "duration_s": 1.0}
    return fake_navigate


def test_pick_refuses_when_standoff_reconfirm_fails(monkeypatch, grasped):
    b = make_backend()
    monkeypatch.setattr(nav, "navigate", _nav_teleports_robot(b))
    monkeypatch.setattr(perception, "confirm_here", lambda node, color: None)

    out = b.pick("magenta_block")

    assert out["success"] is False
    assert "find_object" in out["error"]
    assert grasped == []
    # The sighting is spent — the model must re-scan, not re-pick blind.
    assert b._last_detection is None


def test_pick_refuses_when_creep_stalls(monkeypatch, grasped):
    """A genuine stall (pressed against furniture, odometry not advancing)
    must refuse the grasp and back off for a clear view — the old code
    treated the stall as docked and teleport-grasped across the gap."""
    b = make_backend()
    monkeypatch.setattr(nav, "navigate", _nav_teleports_robot(b))
    monkeypatch.setattr(perception, "confirm_here",
                        lambda node, color: {"x": 1.0, "y": 0.0, "rel": None})

    def align_stalls(node, tx, ty, reach, timeout=0, stall_is_error=False):
        assert stall_is_error, "pick must ask align to treat stalls as errors"
        return "Pressed against something 0.42 m before reaching the target."

    monkeypatch.setattr(manipulation, "align", align_stalls)
    backed = []
    monkeypatch.setattr(manipulation, "back_away",
                        lambda node: backed.append(True))

    out = b.pick("magenta_block")

    assert out["success"] is False
    assert "find_object" in out["error"]
    assert grasped == []
    assert backed == [True]


def test_pick_tolerates_amcl_drift_during_the_creep(monkeypatch, grasped):
    """AMCL correcting itself mid-creep must NOT fail the pick.

    Live 2026-07-21 ~12:16-12:29: the approach completed cleanly (11-12 s,
    no stall), but the map estimate shifted during the creep, so a
    robot-vs-target distance check re-measured AFTER the creep read
    0.52-0.58 m and refused a pick whose robot was standing at the block.
    On video: drive up to the block, back off, stare, repeat. The shortfall
    check belongs inside align (odometry), never across two AMCL fixes."""
    b = make_backend()
    monkeypatch.setattr(nav, "navigate", _nav_teleports_robot(b))
    monkeypatch.setattr(perception, "confirm_here",
                        lambda node, color: {"x": 1.0, "y": 0.0, "rel": None})

    def align_arrives_with_drift(node, tx, ty, reach, timeout=0,
                                 stall_is_error=False):
        # Physically at the block, but AMCL's belief jumped 0.2 m mid-creep:
        # believed distance to the target reads 0.55 m.
        node.robot = (tx - reach - 0.2, ty, 0.0)
        node.odom = node.robot
        return None

    monkeypatch.setattr(manipulation, "align", align_arrives_with_drift)

    out = b.pick("magenta_block")

    assert out["success"] is True
    assert grasped == ["distractor_magenta"]


def test_pick_grasps_after_confirm_and_full_approach(monkeypatch, grasped):
    b = make_backend()
    monkeypatch.setattr(nav, "navigate", _nav_teleports_robot(b))
    monkeypatch.setattr(perception, "confirm_here",
                        lambda node, color: {"x": 1.0, "y": 0.0, "rel": None})

    def align_arrives(node, tx, ty, reach, timeout=0, stall_is_error=False):
        node.robot = (tx - reach, ty, 0.0)
        node.odom = node.robot
        return None

    monkeypatch.setattr(manipulation, "align", align_arrives)

    out = b.pick("magenta_block")

    assert out["success"] is True
    assert grasped == ["distractor_magenta"]
