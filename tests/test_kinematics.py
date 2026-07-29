"""The arm's closed form has to be right before anything is allowed to trust it.

`inverse()` is the one place in this codebase where being wrong is silent: a
sign error returns four plausible-looking floats, the arm swings somewhere
confident and wrong, and the teleport pin attaches the block anyway so the
mission still reports success. None of the live tests can catch that. These
can, because they never start a simulator.
"""

from __future__ import annotations

import itertools
import math

import pytest

from nl2plan_agent.ros_backend import kinematics as kin
from nl2plan_agent.ros_backend.kinematics import ARM

# The five poses replayed by manipulation.py, in (pan, lift, elbow, wrist).
FIXED_POSES = {
    "REST":      (0.0, -0.5, 1.2, 0.3),
    "PRE_GRASP": (0.0,  0.6, 1.4, 0.5),
    "GRASP":     (0.0,  0.9, 1.6, 0.5),
    "LIFT":      (0.0,  0.0, 1.0, 0.3),
    "DROP":      (0.0,  1.1, 0.6, 0.1),
}


def _joint_grid():
    """Configurations spanning the joint box, all inside the URDF stops."""
    for pan in (-1.2, 0.0, 0.7):
        for lift in (-1.4, -0.6, 0.0, 0.6, 1.4):
            for elbow in (-2.2, -0.9, 0.4, 1.6, 2.4):
                for wrist in (-1.4, -0.3, 0.5, 1.5):
                    yield (pan, lift, elbow, wrist)


def _requires_canonical_form_in_range(q):
    """Skip configurations `inverse` is not contractually able to return.

    `inverse` always spells its answer with the arm facing the target. A
    configuration that puts the pads *behind* the pan axis has to be respelled
    with the pan turned 180 degrees, and the model's pan stop is 3.14 rather
    than pi — so a target dead behind the axis needs 3.14159 and is genuinely
    out of range by five hundredths of a degree. That is a property of the
    URDF, not of the solver, and refusing is the correct answer.
    """
    pan = q[0]
    if kin._planar_radius(q) < 0.0:
        pan = kin._wrap(pan + math.pi)
    if abs(pan) > ARM.pan_limit:
        pytest.skip("mirrored pan falls outside the URDF's 3.14 stop")


# --------------------------------------------------------------------------
# The round trip
# --------------------------------------------------------------------------

@pytest.mark.parametrize("q", list(_joint_grid()))
def test_inverse_lands_on_the_point_forward_produced(q):
    """forward(inverse(forward(q))) == forward(q), to well under a millimetre.

    Deliberately not `inverse(forward(q)) == q`. The elbow sign gives two arms
    that reach the same point, and `inverse` is free to hand back the other
    one; what it is not free to do is miss the point.
    """
    _requires_canonical_form_in_range(q)
    target = kin.forward(q)
    pitch = kin.pitch_of(q)

    solved = kin.inverse(*target, pitch)
    assert solved is not None, f"{q} is a real configuration but was refused"

    landed = kin.forward(solved)
    for got, want, axis in zip(landed, target, "xyz"):
        assert got == pytest.approx(want, abs=1e-9), f"{axis} drifted"
    assert kin.pitch_of(solved) == pytest.approx(pitch, abs=1e-9)


@pytest.mark.parametrize("q", list(_joint_grid()))
def test_every_solution_respects_the_joint_stops(q):
    _requires_canonical_form_in_range(q)
    solved = kin.inverse(*kin.forward(q), kin.pitch_of(q))
    assert solved is not None
    assert ARM.within_limits(solved), f"{solved} is past a stop"


def test_elbow_up_is_preferred_when_both_branches_fit():
    """Both elbow signs reach this point; the returned one bends upward."""
    target = kin.forward((0.0, 0.35, 1.0, -0.2))
    solved = kin.inverse(*target, kin.pitch_of((0.0, 0.35, 1.0, -0.2)))
    assert solved is not None
    assert solved[2] > 0, "expected the elbow-up branch"


# --------------------------------------------------------------------------
# Refusing, rather than clamping
# --------------------------------------------------------------------------

def test_target_beyond_full_extension_is_refused():
    reach = ARM.pivot_x + ARM.max_extension
    assert kin.inverse(reach + 0.01, 0.0, ARM.pivot_z, 0.0) is None


def test_target_inside_the_dead_zone_is_refused():
    """Closer than |L1-L2| to the pivot the two proximal links cannot fold up."""
    assert kin.inverse(ARM.pivot_x, 0.0, ARM.pivot_z - 0.01, -math.pi / 2) is None


def test_refusal_is_sharp_rather_than_clamped():
    """A millimetre either side of the limit gives a solution or a None.

    The failure this guards against is `inverse` quietly solving for the
    nearest point it *can* reach. That would return angles here too, and the
    arm would confidently miss.
    """
    z, pitch = 0.10, -math.pi / 4
    edge = kin.max_forward_reach(pitch, z)

    assert kin.inverse(edge - 0.002, 0.0, z, pitch) is not None
    assert kin.inverse(edge + 0.002, 0.0, z, pitch) is None


def test_reachable_agrees_with_inverse():
    for x in (0.10, 0.20, 0.28, 0.34, 0.40, 0.50):
        for pitch in (0.0, -math.pi / 4, -math.pi / 2):
            assert kin.reachable(x, 0.0, 0.025, pitch) is (
                kin.inverse(x, 0.0, 0.025, pitch) is not None)


# --------------------------------------------------------------------------
# The numbers the docs quote
# --------------------------------------------------------------------------

def test_grasp_pose_does_not_reach_a_floor_block():
    """docs/physics.md section 5 claims GRASP lands at x=0.254, z=0.153.

    That is the claim the whole IK task rests on: the fixed poses never arrive,
    so the pin performs the entire grasp. If this test ever goes green-to-red
    because someone retuned GRASP, the docs need editing, not the test.
    """
    x, y, z = kin.forward(FIXED_POSES["GRASP"])
    assert (x, y, z) == pytest.approx((0.254, 0.0, 0.153), abs=0.001)

    block_z = 0.025
    assert z - block_z > 0.12, "GRASP is supposed to stop well above the block"


def test_pre_grasp_reaches_further_forward_than_grasp():
    """The 'grasp' motion is a retract-and-lower, not a reach."""
    assert kin.forward(FIXED_POSES["PRE_GRASP"])[0] > kin.forward(FIXED_POSES["GRASP"])[0]


@pytest.mark.parametrize("pose,x,z", [
    ("REST",      0.144, 0.540),
    ("PRE_GRASP", 0.307, 0.250),
    ("GRASP",     0.254, 0.153),
    ("LIFT",      0.252, 0.503),
    ("DROP",      0.405, 0.289),
])
def test_fixed_pose_table_in_the_docs(pose, x, z):
    got = kin.forward(FIXED_POSES[pose])
    assert got[0] == pytest.approx(x, abs=0.001)
    assert got[2] == pytest.approx(z, abs=0.001)


def test_horizontal_reach_at_pivot_height_is_full_extension():
    """The one reach figure nobody disputes, as a check on the solver itself.

    Arm straight out level with its own shoulder: 0.050 + 0.375 = 0.425 m. If
    this drifts, the solver is wrong and every other number here is worthless.
    """
    assert kin.max_forward_reach(0.0, ARM.pivot_z) == pytest.approx(0.425, abs=0.002)


def test_a_block_on_the_floor_cannot_be_reached_at_all():
    """The finding. A floor block's centre is out of the workspace, full stop.

    Not "GRASP_REACH is a bit optimistic" — there is no distance and no
    approach pitch at which the pads can be put at z = 0.025 m. `shoulder_lift`
    stops at 1.57, so the upper arm can never point below horizontal, and the
    remaining two links cannot make up the difference.
    """
    for i in range(100):
        x = 0.05 + i * 0.004
        for pitch_deg in range(-90, 91, 2):
            assert not kin.reachable(x, 0.0, 0.025, math.radians(pitch_deg)), (
                f"x={x:.3f} pitch={pitch_deg} was expected to be unreachable")


def test_the_pads_bottom_out_just_above_the_floor():
    """How close the arm gets: about 35 mm, against a block centre at 25 mm."""
    assert _any_reachable_at(0.034) is None
    assert _any_reachable_at(0.036) is not None


def _any_reachable_at(z):
    for i in range(100):
        x = 0.05 + i * 0.004
        for pitch_deg in range(-90, 91):
            if kin.reachable(x, 0.0, z, math.radians(pitch_deg)):
                return (x, pitch_deg)
    return None


def test_the_only_purchase_on_a_block_is_its_top_corner():
    """A block is a 0.05 m cube, so the pads have z = 0 to 0.05 to aim at.

    They can only get into the top fifth of that, and only close in. This is
    the number a replacement GRASP_REACH has to respect.
    """
    best = max((kin.max_forward_reach(math.radians(p), 0.05), p)
               for p in range(-90, 1))
    reach, pitch_deg = best
    assert reach == pytest.approx(0.280, abs=0.003)
    assert -75 <= pitch_deg <= -60


def test_every_floor_level_solution_sits_on_the_lift_stop():
    """There is no margin anywhere near the floor — lift is always at 1.57.

    Worth asserting because it means a real controller would be fighting a
    joint limit for the whole grasp, which is not somewhere to operate.
    """
    x = kin.max_forward_reach(math.radians(-69), 0.05)
    q = kin.inverse(x, 0.0, 0.05, math.radians(-69))
    assert q is not None
    assert q[1] == pytest.approx(ARM.lift_limit, abs=0.02)


def test_no_reach_constant_could_have_served_a_floor_block():
    """Why the block changed rather than the constant.

    Lowering GRASP_REACH was the plan's answer while the block was a 50 mm
    cube. It was not an answer: there is no value it could have taken. The
    live constant is checked against the live block in test_block_geometry.py;
    this is only the record of why 0.35 could not simply be reduced.
    """
    assert math.isnan(kin.max_forward_reach(math.radians(-69), 0.025))
    assert kin.max_forward_reach(math.radians(-69), 0.05) < 0.350


def test_the_plans_proposed_thirty_centimetre_reach_is_still_too_far():
    """0.28-0.30 m came from the annulus alone and does not survive the stops."""
    assert not kin.reachable(0.30, 0.0, 0.05, math.radians(-69))
    assert not kin.reachable(0.30, 0.0, 0.05, math.radians(-45))
