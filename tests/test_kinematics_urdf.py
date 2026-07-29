"""`kinematics.ARM` must keep matching the robot the simulator actually loads.

The constants in `kinematics.py` are transcribed from the URDF's joint origins.
Transcription rots: someone lengthens a link in the model, the solver keeps
solving, every test that only checks the solver against itself stays green, and
the arm quietly aims 2 cm off. So this file goes back to the URDF, rebuilds the
chain generically from joint origins and axes, and compares.

It also pins the two structural assumptions the closed form is built on — that
the arm joints are pure translations with no rotated mounts, and that their axes
are exactly +z then +y, +y, +y. If either stops holding, the planar reduction in
`kinematics.py` is invalid and no amount of constant-fixing will save it.

Skips cleanly where there is no ROS: requirements.txt promises the agent side
runs without it, and this is the one test that needs the simulator's model.
"""

from __future__ import annotations

import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from nl2plan_agent.ros_backend import kinematics as kin
from nl2plan_agent.ros_backend.kinematics import ARM

URDF_CANDIDATES = (
    Path.home() / "ros2_ws/src/mobile_arm_sim/urdf/mobile_arm.urdf.xacro",
    Path.home() / "ros2_ws/install/mobile_arm_sim/share/mobile_arm_sim/urdf/mobile_arm.urdf.xacro",
)

# base_footprint -> the centre of the finger pads, in order.
CHAIN = (
    "base_footprint_joint",
    "arm_base_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_joint",
    "gripper_base_joint",
)
ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint",
              "elbow_joint", "wrist_joint")


@pytest.fixture(scope="module")
def urdf():
    """The expanded model, or a skip if this machine has no simulator."""
    path = next((p for p in URDF_CANDIDATES if p.exists()), None)
    if path is None:
        pytest.skip("mobile_arm_sim URDF not on this machine")
    if shutil.which("xacro") is None:
        pytest.skip("xacro not on PATH; source the ROS environment to run this")

    out = subprocess.run(["xacro", str(path)], capture_output=True, text=True)
    if out.returncode != 0:
        pytest.skip(f"xacro could not expand the model: {out.stderr[:200]}")

    root = ET.fromstring(out.stdout)
    joints = {}
    for el in root.findall("joint"):
        origin = el.find("origin")
        limit = el.find("limit")
        axis = el.find("axis")
        joints[el.get("name")] = {
            "xyz": _triple(origin.get("xyz") if origin is not None else None),
            "rpy": _triple(origin.get("rpy") if origin is not None else None),
            "axis": _triple(axis.get("xyz") if axis is not None else None),
            "lower": float(limit.get("lower")) if limit is not None else None,
            "upper": float(limit.get("upper")) if limit is not None else None,
        }
    return joints


def _triple(text):
    if text is None:
        return (0.0, 0.0, 0.0)
    a, b, c = (float(v) for v in text.split())
    return (a, b, c)


# --------------------------------------------------------------------------
# The assumptions the planar reduction rests on
# --------------------------------------------------------------------------

def test_arm_joints_are_not_mounted_at_an_angle(urdf):
    """Any non-zero rpy on the chain breaks the planar reduction outright."""
    for name in CHAIN:
        assert urdf[name]["rpy"] == (0.0, 0.0, 0.0), f"{name} has a rotated mount"


def test_arm_joint_axes_are_a_turntable_carrying_a_planar_chain(urdf):
    assert urdf["shoulder_pan_joint"]["axis"] == (0.0, 0.0, 1.0)
    for name in ("shoulder_lift_joint", "elbow_joint", "wrist_joint"):
        assert urdf[name]["axis"] == (0.0, 1.0, 0.0), f"{name} is not parallel"


def test_links_run_along_their_own_z(urdf):
    """The reason sine and cosine come out swapped from the textbook form."""
    for name in ("shoulder_lift_joint", "elbow_joint", "wrist_joint",
                 "gripper_base_joint"):
        x, y, z = urdf[name]["xyz"]
        assert (x, y) == (0.0, 0.0), f"{name} is offset sideways"
        assert z > 0.0


# --------------------------------------------------------------------------
# The constants
# --------------------------------------------------------------------------

def test_link_lengths_match_the_urdf(urdf):
    assert ARM.l1 == pytest.approx(urdf["elbow_joint"]["xyz"][2], abs=1e-9)
    assert ARM.l2 == pytest.approx(urdf["wrist_joint"]["xyz"][2], abs=1e-9)


def test_pad_offset_matches_the_urdf(urdf):
    """L3 runs to the middle of the finger pads, not the fingertips."""
    to_gripper = urdf["gripper_base_joint"]["xyz"][2]
    to_finger = urdf["left_finger_joint"]["xyz"][2]
    pad_half_height = 0.05 / 2          # the finger box is 0.05 m tall
    assert ARM.l3 == pytest.approx(to_gripper + to_finger + pad_half_height,
                                   abs=1e-9)


def test_pivot_position_matches_the_urdf(urdf):
    x = sum(urdf[n]["xyz"][0] for n in CHAIN[:4])
    z = sum(urdf[n]["xyz"][2] for n in CHAIN[:4])
    assert ARM.pivot_x == pytest.approx(x, abs=1e-9)
    assert ARM.pivot_z == pytest.approx(z, abs=1e-9)


@pytest.mark.parametrize("joint,attr", [
    ("shoulder_pan_joint", "pan_limit"),
    ("shoulder_lift_joint", "lift_limit"),
    ("elbow_joint", "elbow_limit"),
    ("wrist_joint", "wrist_limit"),
])
def test_joint_stops_match_the_urdf(urdf, joint, attr):
    """Stops are what make a target unreachable well inside the annulus.

    `lift_limit` in particular is the constraint that stops this arm touching
    the floor at all, so a silent change here would silently change the
    workspace.
    """
    lower, upper = urdf[joint]["lower"], urdf[joint]["upper"]
    assert lower == pytest.approx(-upper, abs=1e-9), "expected symmetric stops"
    assert getattr(ARM, attr) == pytest.approx(upper, abs=1e-9)


# --------------------------------------------------------------------------
# Forward kinematics, rebuilt from the model rather than from the closed form
# --------------------------------------------------------------------------

def _mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(4))
                       for j in range(4)) for i in range(4))


def _translate(x, y, z):
    return ((1.0, 0.0, 0.0, x),
            (0.0, 1.0, 0.0, y),
            (0.0, 0.0, 1.0, z),
            (0.0, 0.0, 0.0, 1.0))


def _rot(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    if axis == (0.0, 0.0, 1.0):
        return ((c, -s, 0.0, 0.0), (s, c, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    if axis == (0.0, 1.0, 0.0):
        return ((c, 0.0, s, 0.0), (0.0, 1.0, 0.0, 0.0),
                (-s, 0.0, c, 0.0), (0.0, 0.0, 0.0, 1.0))
    raise AssertionError(f"unexpected axis {axis}")


def _urdf_forward(urdf, q):
    """Pads in base_footprint, composed joint by joint straight from the model.

    Shares no algebra with kinematics.forward — that one collapses the chain
    into a planar sum, this one multiplies the transforms the URDF declares.
    """
    angles = dict(zip(ARM_JOINTS, q))
    T = _translate(0.0, 0.0, 0.0)
    for name in CHAIN:
        j = urdf[name]
        T = _mul(T, _translate(*j["xyz"]))
        if name in angles:
            T = _mul(T, _rot(j["axis"], angles[name]))
    # gripper_base -> the middle of the pads, along the gripper's own z
    pad = urdf["left_finger_joint"]["xyz"][2] + 0.05 / 2
    T = _mul(T, _translate(0.0, 0.0, pad))
    return (T[0][3], T[1][3], T[2][3])


@pytest.mark.parametrize("q", [
    (0.0, 0.0, 0.0, 0.0),
    (0.0, -0.5, 1.2, 0.3),        # REST
    (0.0, 0.6, 1.4, 0.5),         # PRE_GRASP
    (0.0, 0.9, 1.6, 0.5),         # GRASP
    (0.0, 0.0, 1.0, 0.3),         # LIFT
    (0.0, 1.1, 0.6, 0.1),         # DROP
    (0.8, 1.57, 1.39, 0.0),       # at the lift stop, panned off the centre line
    (-1.2, -1.4, -2.2, 1.5),
    (2.9, 0.35, 2.4, -1.5),
])
def test_closed_form_agrees_with_the_urdf_chain(urdf, q):
    got = kin.forward(q)
    want = _urdf_forward(urdf, q)
    for g, w, axis in zip(got, want, "xyz"):
        assert g == pytest.approx(w, abs=1e-12), f"{axis} disagrees for {q}"


def test_off_centre_pan_is_where_the_offset_bug_would_show(urdf):
    """Panned off the centre line, the pre-rotation offset stops cancelling.

    Applying the 0.050 m arm-base offset after the pan instead of before it
    passes every on-axis check and fails here.
    """
    q = (1.1, 0.9, 1.6, 0.5)
    got, want = kin.forward(q), _urdf_forward(urdf, q)
    assert got == pytest.approx(want, abs=1e-12)

    naive_x = math.cos(q[0]) * (ARM.pivot_x + kin._planar_radius(q))
    assert abs(naive_x - want[0]) > 0.01, "this case is supposed to discriminate"
