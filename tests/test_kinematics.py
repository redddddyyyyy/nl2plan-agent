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
import random

import pytest

from nl2plan_agent.ros_backend import kinematics as kin
from nl2plan_agent.ros_backend.kinematics import ARM, ArmGeometry

# The poses docs/physics.md section 5 tabulates, in (pan, lift, elbow, wrist).
#
# Four of these are still what manipulation.py replays. DROP is not: that entry
# is the *measured* pose, and manipulation.py now carries a solved one. Both are
# kept, because they pin different claims — this table pins the historical
# figures in the docs, and LIVE_DROP below pins the constant actually in use.
# Labelling this set "what manipulation.py replays" is what made the drift
# invisible in the first place.
FIXED_POSES = {
    "REST":      (0.0, -0.5, 1.2, 0.3),
    "PRE_GRASP": (0.0,  0.6, 1.4, 0.5),
    "GRASP":     (0.0,  0.9, 1.6, 0.5),
    "LIFT":      (0.0,  0.0, 1.0, 0.3),
    "DROP":      (0.0,  1.1, 0.6, 0.1),      # superseded — see LIVE_DROP
}

# What manipulation.py replays today: solved by this module, not measured.
LIVE_DROP = (0.0, 0.814, 0.733, 0.547)


def _joint_grid():
    """Configurations spanning the joint box, all inside the URDF stops."""
    for pan in (-1.2, 0.0, 0.7):
        for lift in (-1.4, -0.6, 0.0, 0.6, 1.4):
            for elbow in (-2.2, -0.9, 0.4, 1.6, 2.4):
                for wrist in (-1.4, -0.3, 0.5, 1.5):
                    yield (pan, lift, elbow, wrist)


def _canonical_pan(q):
    """The pan angle `inverse` will report for this configuration's pose.

    `inverse` always spells its answer with the arm facing the target, so a
    configuration putting the pads *behind* the pan axis is respelled with the
    pan turned 180 degrees.
    """
    pan = q[0]
    if kin._planar_radius(q) < 0.0:
        pan = kin._wrap(pan + math.pi)
    return pan


def _requires_canonical_form_in_range(q):
    """Skip configurations `inverse` is not contractually able to return.

    The model's pan stop is 3.14 rather than pi, so a target dead behind the
    axis needs 3.14159 and is genuinely out of range by five hundredths of a
    degree. That is a property of the URDF, not of the solver, and refusing is
    the correct answer.
    """
    if abs(_canonical_pan(q)) > ARM.pan_limit:
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
# Completeness: a refusal has to mean unreachable, not unsearched
# --------------------------------------------------------------------------
#
# `inverse` tries two elbow branches at one pan angle and gives up. But there
# are up to *four* joint vectors realising any (target, pitch): two elbow
# branches, each in two pan spellings related by the mirror map
#
#     M(pan, lift, elbow, wrist) = (pan +/- pi, -lift, -elbow, -wrist)
#
# which negates the planar radius and therefore lands the pads in exactly the
# same place. The two spellings the solver never looks at are not hypothetical.
#
# Testing only two is exhaustive anyway — but *only because the stops are
# symmetric*. M negates the three pitch joints, and each of their ranges is
# symmetric about zero, so M preserves admissibility on them; the pan is the
# only coordinate where the spellings differ in legality, and for a target
# ahead of the axis the mirror always carries the larger |pan| of the two.
#
# That hypothesis is invisible in the code, because `ArmGeometry` stores one
# limit per joint and tests `abs(angle) <= limit` — the datatype cannot express
# its own violation. The last test in this section makes it executable, because
# a real arm's shoulder is not symmetric and this is what will bite on hardware.


def test_the_mirror_map_lands_the_pads_in_the_same_place():
    """The lemma the completeness argument turns on."""
    for q in _joint_grid():
        if abs(kin._planar_radius(q)) < 1e-9:
            continue        # rho == 0 is the one pose with no facing spelling
        mirrored = (kin._wrap(q[0] + math.pi), -q[1], -q[2], -q[3])
        assert kin.forward(mirrored) == pytest.approx(kin.forward(q), abs=1e-12)
        assert kin.pitch_of(mirrored) == pytest.approx(kin.pitch_of(q), abs=1e-12)


def test_a_refusal_means_unreachable_rather_than_unsearched():
    """The property that makes `reachable()` a decision procedure.

    Every configuration in the joint box strikes some pose. Feed that pose back
    and the solver must return *something* — not necessarily the same
    configuration, since the elbow sign is its choice, but some configuration
    reaching the same point at the same pitch. A refusal here would mean
    `reachable()` can answer "no" about a target the arm is standing in, and a
    caller trusting it would decline work the robot can do.
    """
    rng = random.Random(20260803)
    checked = 0
    for _ in range(20000):
        q = (rng.uniform(-ARM.pan_limit, ARM.pan_limit),
             rng.uniform(-ARM.lift_limit, ARM.lift_limit),
             rng.uniform(-ARM.elbow_limit, ARM.elbow_limit),
             rng.uniform(-ARM.wrist_limit, ARM.wrist_limit))
        if abs(_canonical_pan(q)) > ARM.pan_limit:
            continue        # the 3.14-vs-pi artefact, not an incompleteness
        checked += 1
        target = kin.forward(q)
        solved = kin.inverse(*target, kin.pitch_of(q))
        assert solved is not None, f"{q} is a real configuration but was refused"
        assert kin.forward(solved) == pytest.approx(target, abs=1e-9)
    assert checked > 19900, "the pan filter is supposed to reject a handful"


class _AsymmetricArm(ArmGeometry):
    """An arm whose shoulder range is not symmetric — i.e. a real one.

    `ArmGeometry` cannot represent this: it holds one limit per joint and tests
    `abs(angle) <= limit`. That is not a modelling shortcut, it is the
    hypothesis completeness rests on, and the datatype conceals it by making
    the violation inexpressible. Overriding `within_limits` puts it on record.
    """

    lift_lower = -0.3

    def within_limits(self, q):
        pan, lift, elbow, wrist = q
        return (abs(pan) <= self.pan_limit
                and self.lift_lower <= lift <= self.lift_limit
                and abs(elbow) <= self.elbow_limit
                and abs(wrist) <= self.wrist_limit)


def test_completeness_fails_the_moment_the_stops_go_asymmetric():
    """The caveat that will bite on hardware, as an executable example.

    `q` is a pose this asymmetric arm can physically strike: its lift sits at
    +1.2, well inside a [-0.3, 1.57] range. But the pads end up *behind* the
    pan axis, so the only spelling `inverse` ever returns is the mirrored one,
    which needs lift = -1.2 and is refused. The solver reports the target
    unreachable while the arm is standing in it.

    Nothing is broken today — the URDF's stops really are symmetric, which
    test_kinematics_urdf.py checks. This exists so that a hardware URDF with a
    realistic shoulder trips a test, instead of quietly shrinking the reported
    workspace and refusing perfectly good grasps.

    The fix, if that day comes, is to try the mirrored pan as a third and
    fourth branch in `inverse`.
    """
    arm = _AsymmetricArm(
        pivot_x=ARM.pivot_x, pivot_z=ARM.pivot_z,
        l1=ARM.l1, l2=ARM.l2, l3=ARM.l3,
        pan_limit=ARM.pan_limit, lift_limit=ARM.lift_limit,
        elbow_limit=ARM.elbow_limit, wrist_limit=ARM.wrist_limit)
    q = (1.0, 1.2, -2.5, -0.27)

    assert arm.within_limits(q), "the arm is supposed to be able to strike this"
    assert kin._planar_radius(q, arm) < 0, "the pads must sit behind the pan axis"

    target = kin.forward(q, arm)
    pitch = kin.pitch_of(q, arm)
    assert kin.inverse(*target, pitch, arm) is None, "expected a refusal"

    # The symmetric arm serves the identical target, by the mirrored spelling.
    solved = kin.inverse(*target, pitch, ARM)
    assert solved is not None
    assert solved[0] == pytest.approx(kin._wrap(q[0] + math.pi), abs=1e-9)
    assert solved[1] == pytest.approx(-q[1], abs=1e-9)
    assert solved[2] == pytest.approx(-q[2], abs=1e-9)
    assert solved[3] == pytest.approx(-q[3], abs=1e-9)
    assert kin.forward(solved) == pytest.approx(target, abs=1e-12)


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


def test_the_live_drop_pose_is_where_manipulation_says_it_is():
    """The solved DROP, against docs/physics.md: pads at (0.370, 0.313).

    The table above deliberately keeps the superseded measured pose, because it
    is what the docs' section 5 figures were computed from. This pins the
    constant `manipulation.py` actually replays, which nothing else did — the
    table's label claimed to cover it and quietly did not.
    """
    x, y, z = kin.forward(LIVE_DROP)
    assert (x, z) == pytest.approx((0.370, 0.313), abs=0.001)
    assert kin.pitch_of(LIVE_DROP) == pytest.approx(math.radians(-30), abs=0.01)

    # It is a solved pose, so the solver has to reproduce it — this is the
    # round trip the pose was generated by, run backwards.
    assert kin.inverse(x, y, z, kin.pitch_of(LIVE_DROP)) is not None

    # Whether the held block then clears the table top is a question about
    # CARRY_HOLD_BELOW_PADS and the block's height, not about the arm, and it
    # lives in test_block_geometry.py. Importing that constant here would drag
    # ROS into the one test file that is guaranteed to run without it.

    # Every joint well inside its stop, which is the property the measured
    # pose it replaced did not have.
    assert min(ARM.lift_limit - abs(LIVE_DROP[1]),
               ARM.elbow_limit - abs(LIVE_DROP[2]),
               ARM.wrist_limit - abs(LIVE_DROP[3])) > 0.75


def test_horizontal_reach_at_pivot_height_is_full_extension():
    """The one reach figure nobody disputes, as a check on the solver itself.

    Arm straight out level with its own shoulder: 0.050 + 0.375 = 0.425 m. If
    this drifts, the solver is wrong and every other number here is worthless.
    """
    assert kin.max_forward_reach(0.0, ARM.pivot_z) == pytest.approx(0.425, abs=0.002)


# --------------------------------------------------------------------------
# The floor bound, as a theorem rather than a scan
# --------------------------------------------------------------------------
#
# This block used to be a 100-by-91 sweep asserting that no (x, pitch) pair put
# the pads at floor height. That sweep was evidence, not proof: a finite probe
# of a continuum, which is the same epistemic object the rest of this project
# declines to accept from a simulator. The claim does not need one.
#
# Writing a = lift, b = lift + elbow, c = lift + elbow + wrist for the three
# links' angles from vertical, forward kinematics gives
#
#     z = pivot_z + L1 cos(a) + L2 cos(b) + L3 cos(c)
#
# The lift stop holds |a| <= lift_limit < pi/2. Cosine is even and decreasing
# on [0, pi/2], so cos(a) >= cos(lift_limit) > 0, while cos(b) and cos(c) are
# bounded below by -1. The coefficients are positive, so summing gives
#
#     z >= pivot_z + L1 cos(lift_limit) - L2 - L3
#
# and the bound is attained at a = lift_limit, b = c = pi. It holds uniformly
# over pan, over position, and over approach pitch — none of which appear in
# it. That last part is the whole finding, and a sweep cannot express it.


def floor_bound(arm=ARM):
    """Greatest lower bound on pad height, over the entire joint box."""
    return arm.pivot_z + arm.l1 * math.cos(arm.lift_limit) - arm.l2 - arm.l3


def floor_bound_witness(arm=ARM, pan=0.0):
    """The configuration attaining `floor_bound`: a = lift_limit, b = c = pi."""
    return (pan, arm.lift_limit, math.pi - arm.lift_limit, 0.0)


def test_the_bounds_hypothesis_still_holds():
    """cos(a) > 0 needs the lift stop strictly inside a quarter turn.

    This is the single line of the derivation a URDF change could invalidate,
    and it is why the bound is stated against `lift_limit` rather than as a
    number. Open shoulder_lift past pi/2 and the bound is not merely wrong, it
    is unproven — the arm could then point its upper link below horizontal and
    every conclusion here would need redoing. The sweep this replaced would
    have gone quietly green instead.
    """
    assert ARM.lift_limit < math.pi / 2


def test_the_floor_bound_is_attained():
    """Tightness. Without a witness this is an inequality, not the answer."""
    q = floor_bound_witness()
    assert ARM.within_limits(q), f"{q} is supposed to be a legal configuration"
    assert kin.forward(q)[2] == pytest.approx(floor_bound(), abs=1e-12)

    # Where it bottoms out, for the record: straight down, 0.200 m ahead.
    x, y, z = kin.forward(q)
    assert (x, y) == pytest.approx((0.200, 0.0), abs=0.001)
    assert kin.pitch_of(q) == pytest.approx(-math.pi / 2, abs=1e-6)


def test_the_bound_is_arithmetically_what_the_docs_quote():
    """35 mm. The odd 0.119 mm is L1*cos(1.57) — the URDF rounding pi/2."""
    assert floor_bound() == pytest.approx(0.0351194, abs=1e-6)


def test_no_configuration_in_the_joint_box_beats_the_bound():
    """A falsification attempt, not the establishment. The proof is above.

    A grid this coarse could never have *found* the bound — it is attained at a
    single point of the joint box. This is here to catch an arithmetic slip in
    `floor_bound`, and nothing more is claimed for it.
    """
    bound = floor_bound()
    steps = 24
    for i in range(steps + 1):
        lift = -ARM.lift_limit + 2 * ARM.lift_limit * i / steps
        for j in range(steps + 1):
            elbow = -ARM.elbow_limit + 2 * ARM.elbow_limit * j / steps
            for k in range(steps + 1):
                wrist = -ARM.wrist_limit + 2 * ARM.wrist_limit * k / steps
                q = (0.0, lift, elbow, wrist)
                assert kin.forward(q)[2] >= bound - 1e-12, f"{q} beat the bound"


def test_a_block_on_the_floor_cannot_be_reached_at_all():
    """The finding, now a consequence rather than a 9,100-point scan.

    A floor block's centre sits at z = 0.025 m, below the bound by 10 mm. Not
    "GRASP_REACH is a bit optimistic" — there is no distance and no approach
    pitch at which the pads can be put there, because neither distance nor
    pitch appears in the bound.
    """
    block_centre_z = 0.025
    assert floor_bound() > block_centre_z
    assert floor_bound() - block_centre_z > 0.010


def test_the_solver_draws_the_line_where_the_bound_does():
    """`inverse` and the bound have to agree, or one of them is wrong.

    The only sweep left in this section, and it sweeps to *check* a bound
    proved independently of it rather than to discover where the bound is.
    Sharp to a tenth of a millimetre on this grid.
    """
    assert _any_reachable_at(floor_bound() - 0.0001) is None
    assert _any_reachable_at(floor_bound() + 0.0001) is not None


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
