"""The redundancy resolution, and the claim that it is certified rather than sampled.

Two things are under test and they fail in opposite directions.

`Phi` must not be too *small*. If a certified-feasible interval contains a pitch
the solver refuses, the search has promised a pose the arm cannot strike. Every
test of the inner bound therefore hands its pitches back to `kinematics.inverse`
and insists it produces something.

`Phi` must not be too *large*, in the sense that the outer bound must contain
every feasible pitch there is. That is the direction sampling can attack:
a dense independent sweep must never find a feasible pitch outside the outer
bound, nor a margin better than the one `resolve` certified as optimal. Sampling
cannot confirm the enclosure, but it can refute it, and refutation is what a
test is for.

The suite also pins the finding that motivated writing this at all. The tables
in the theory writeup were computed on a one-degree grid over [-90, 0] degrees,
and every one of them reported the lower end of `Phi` as exactly -90 degrees.
That is what a sweep clipped by its own bounds looks like, and it reads as a
physical fact about the arm — that it cannot tilt the gripper past straight
down. It can: at the live grasp the feasible band runs to -113 degrees.
`test_the_old_ninety_degree_floor_was_the_sweeps_edge` is that regression.
"""

from __future__ import annotations

import math

import pytest

from nl2plan_agent.ros_backend import kinematics as kin
from nl2plan_agent.ros_backend import redundancy as red
from nl2plan_agent.ros_backend.kinematics import ARM

# The live grasp: the 30x30x150 mm bar, pads closing at the height section 3.7
# of the writeup uses.
BAR_Z = 0.075
BAR_TARGETS = (0.240, 0.262, 0.280, 0.300, 0.320)

# The 50 mm cube this replaced, gripped at its own centre height.
CUBE_Z = 0.050

# `resolve` certifies to a tolerance, and certifying costs O(1/sqrt(tolerance)).
# The default is fine for a grasp; tests that call it dozens of times loosen it,
# since nothing here turns on the sixth decimal of a margin.
LOOSE = 1e-4


def _sweep(x, y, z, lo=-math.pi, hi=math.pi, n=20000):
    """An independent dense sweep. Deliberately the naive thing this replaces."""
    feasible = []
    best = None
    for i in range(n + 1):
        phi = lo + (hi - lo) * i / n
        for q in kin.branch_solutions(x, y, z, phi):
            if ARM.within_limits(q):
                feasible.append(phi)
                m = ARM.stop_margin(q)
                if best is None or m > best[0]:
                    best = (m, phi)
                break
    return feasible, best


# --------------------------------------------------------------------------
# The stop margin itself
# --------------------------------------------------------------------------

def test_margin_is_non_negative_exactly_when_the_pose_is_legal():
    """It has to be a graded version of `within_limits`, not a separate opinion."""
    for lift in (-1.8, -1.57, -1.2, 0.0, 1.2, 1.57, 1.8):
        for elbow in (-2.6, -2.5, -1.0, 0.0, 2.5, 2.6):
            for wrist in (-1.6, -1.57, 0.0, 1.57, 1.6):
                q = (0.0, lift, elbow, wrist)
                assert (ARM.stop_margin(q) >= 0.0) == ARM.within_limits(q), q


def test_margin_reports_the_tightest_joint_not_the_average():
    q = (0.0, 1.5, 0.0, 0.0)
    assert ARM.stop_margin(q) == pytest.approx(ARM.lift_limit - 1.5)


def test_the_pan_is_excluded_from_the_margin_and_reported_separately():
    """Including it would flatten the objective with a pitch-independent constant.

    The pan is fixed by the target. If it entered the margin, every pitch would
    share the same cap whenever the pan was the tightest joint, and the argmax
    would stop discriminating between them for no reason connected to the arm.
    """
    near_pan_stop = (3.0, 0.0, 0.0, 0.0)
    assert ARM.stop_margin(near_pan_stop) == pytest.approx(ARM.wrist_limit)
    assert ARM.pan_margin(near_pan_stop) == pytest.approx(ARM.pan_limit - 3.0)


# --------------------------------------------------------------------------
# Phi: the inner bound must contain nothing the solver refuses
# --------------------------------------------------------------------------

@pytest.mark.parametrize("x", BAR_TARGETS)
def test_every_certified_pitch_is_one_the_solver_actually_serves(x):
    phi = red.feasible_pitches(x, 0.0, BAR_Z)
    assert phi.inner
    for lo, hi in phi.inner:
        for i in range(41):
            t = lo + (hi - lo) * i / 40
            assert kin.inverse(x, 0.0, BAR_Z, t) is not None, (x, math.degrees(t))


@pytest.mark.parametrize("x", BAR_TARGETS)
def test_a_dense_sweep_finds_no_feasible_pitch_outside_the_outer_bound(x):
    outer = red.feasible_pitches(x, 0.0, BAR_Z).outer
    feasible, _ = _sweep(x, 0.0, BAR_Z)
    assert feasible
    for t in feasible:
        assert any(lo - 1e-9 <= t <= hi + 1e-9 for lo, hi in outer), \
            (x, math.degrees(t))


@pytest.mark.parametrize("x", BAR_TARGETS)
def test_the_two_bounds_close_to_within_the_resolution_asked_for(x):
    """`inner <= Phi <= outer`, and the gap is where the boundary is known to be."""
    phi = red.feasible_pitches(x, 0.0, BAR_Z, resolution=1e-6)
    assert len(phi.inner) == len(phi.outer) == 1
    (i_lo, i_hi), (o_lo, o_hi) = phi.inner[0], phi.outer[0]
    assert o_lo <= i_lo and i_hi <= o_hi
    assert i_lo - o_lo < 1e-5 and o_hi - i_hi < 1e-5


def test_an_empty_phi_is_a_proof_of_unreachability():
    """Theorem 3 says a floor block is out of reach at every pitch. `Phi` agrees,
    and agrees by exclusion over a continuum rather than by failing to find."""
    phi = red.feasible_pitches(0.240, 0.0, 0.025)
    assert phi.is_empty and not phi.inner and not phi.outer
    feasible, _ = _sweep(0.240, 0.0, 0.025)
    assert feasible == []


def test_phi_is_empty_beyond_full_extension_and_inside_the_dead_zone():
    assert red.feasible_pitches(0.900, 0.0, 0.200).is_empty
    assert red.feasible_pitches(ARM.pivot_x, 0.0, ARM.pivot_z).is_empty


def test_a_target_behind_the_pan_stop_is_refused_before_any_search():
    """The one test that does not depend on the pitch: the pan cannot help."""
    narrow = kin.ArmGeometry(**{**ARM.__dict__, "pan_limit": 0.2})
    assert red.feasible_pitches(-0.2, 0.05, 0.20, arm=narrow).is_empty
    assert red.resolve(-0.2, 0.05, 0.20, arm=narrow) is None


# --------------------------------------------------------------------------
# The regression that motivated the whole module
# --------------------------------------------------------------------------

def test_the_old_ninety_degree_floor_was_the_sweeps_edge_not_the_arms():
    """A grid reported `Phi` as [-90, -43] degrees at x = 0.240. Half of that
    was an artefact.

    The upper endpoint was real: -42.59 degrees, which a one-degree grid rounds
    to -43. The lower endpoint was the edge of the sweep. Searching the whole
    turn finds the band continues to -113.02 degrees, so the sampled table
    understated the arm's pitch freedom by 23 degrees and did it in a way that
    looked like a fact about the joint stops.
    """
    phi = red.feasible_pitches(0.240, 0.0, BAR_Z)
    (lo, hi), = phi.inner
    assert math.degrees(hi) == pytest.approx(-42.59, abs=0.05)
    assert math.degrees(lo) == pytest.approx(-113.02, abs=0.05)
    # Not a rounding quibble: pitches past straight down are genuinely served.
    for t in (-95.0, -105.0, -112.0):
        assert kin.inverse(0.240, 0.0, BAR_Z, math.radians(t)) is not None


def test_restricting_to_approaches_from_above_is_a_choice_not_a_fact():
    """`FROM_ABOVE` reproduces the old table exactly — which is the point.

    The band the writeup swept is a legitimate preference: a grasp usually wants
    to come down onto the object rather than tilt back under it. It is just not
    a property of the arm, so it is a parameter with a name rather than a loop
    bound.
    """
    wide = red.feasible_pitches(0.240, 0.0, BAR_Z, domain=red.FULL_TURN)
    above = red.feasible_pitches(0.240, 0.0, BAR_Z, domain=red.FROM_ABOVE)
    (a_lo, a_hi), = above.inner
    assert math.degrees(a_lo) == pytest.approx(-90.0, abs=1e-3)
    assert a_hi == pytest.approx(wide.inner[0][1], abs=1e-6)
    assert above.measure < wide.measure


# --------------------------------------------------------------------------
# phi*: the criterion, and the certificate attached to it
# --------------------------------------------------------------------------

@pytest.mark.parametrize("x", BAR_TARGETS)
def test_no_sampled_pitch_beats_the_certified_optimum(x):
    """The claim `resolve` makes that a grid scan cannot: nothing does better."""
    got = red.resolve(x, 0.0, BAR_Z, tolerance=LOOSE)
    _, best = _sweep(x, 0.0, BAR_Z)
    assert best is not None
    assert best[0] <= got.margin + 1e-12, (x, best[0], got.margin)
    assert got.margin <= got.margin_bound + 1e-12
    assert got.optimality_gap <= LOOSE + 1e-12


@pytest.mark.parametrize("x", BAR_TARGETS)
def test_the_chosen_pose_lands_on_the_target_and_inside_the_stops(x):
    """A criterion that returns an unreachable pose would be worse than a constant."""
    got = red.resolve(x, 0.0, BAR_Z, tolerance=LOOSE)
    assert ARM.within_limits(got.joints)
    assert got.margin == pytest.approx(ARM.stop_margin(got.joints), abs=1e-12)
    px, py, pz = kin.forward(got.joints)
    assert (px, py, pz) == pytest.approx((x, 0.0, BAR_Z), abs=1e-9)
    assert kin.inverse(x, 0.0, BAR_Z, got.pitch) is not None


@pytest.mark.parametrize("x", BAR_TARGETS)
def test_the_chosen_pitch_is_certified_feasible(x):
    got = red.resolve(x, 0.0, BAR_Z, tolerance=LOOSE)
    assert red.feasible_pitches(x, 0.0, BAR_Z).contains(got.pitch)


def test_the_optimum_is_reported_with_the_joint_that_rations_it():
    """`shoulder_lift` binds at every grasp — the same stop as Theorem 3.

    Worth asserting rather than remarking, because it is the load-bearing
    observation: one model change relieves both the floor bound and the grasp
    margin, and if that ever stopped being true the argument would need redoing.
    """
    for x in BAR_TARGETS:
        got = red.resolve(x, 0.0, BAR_Z, tolerance=LOOSE)
        assert got.binding_joint == "shoulder_lift", (x, got.binding_joint)


def test_an_unreachable_target_resolves_to_nothing_rather_than_to_a_guess():
    assert red.resolve(0.240, 0.0, 0.025) is None
    assert red.resolve(0.900, 0.0, 0.200) is None
    assert red.approach_pitch(0.240, 0.0, 0.025) is None


def test_restricting_the_domain_can_only_lose_margin():
    for x in BAR_TARGETS:
        wide = red.resolve(x, 0.0, BAR_Z, tolerance=LOOSE)
        above = red.resolve(x, 0.0, BAR_Z, domain=red.FROM_ABOVE, tolerance=LOOSE)
        assert above.margin <= wide.margin + LOOSE


# --------------------------------------------------------------------------
# What the numbers say about the object change
# --------------------------------------------------------------------------

def test_the_bar_buys_about_five_times_the_margin_of_the_cube():
    """Quantifies a claim the earlier draft made qualitatively.

    Changing the object from a 50 mm cube to a 150 mm bar did not merely make
    the block reachable. At the nominal grasp it multiplied the room to the
    nearest stop by roughly five, and it widened the feasible pitch band from a
    sliver to something an approach error can be absorbed in.
    """
    bar = red.resolve(0.262, 0.0, BAR_Z, tolerance=LOOSE)
    cube = red.resolve(0.262, 0.0, CUBE_Z, tolerance=LOOSE)
    assert bar.margin / cube.margin == pytest.approx(5.0, rel=0.15)


def test_the_cube_at_the_far_grasp_sat_on_the_stop():
    """0.001 rad of room, which is no room. This is the pose that was in use."""
    cube = red.resolve(0.280, 0.0, CUBE_Z, tolerance=1e-7)
    assert cube.margin < 0.005
    assert cube.binding_joint == "shoulder_lift"
    band = red.feasible_pitches(0.280, 0.0, CUBE_Z)
    assert math.degrees(band.measure) < 6.0


def test_margin_falls_off_as_the_target_moves_out():
    """Monotone across the reach band, which is what makes phi* worth having:
    the further the target, the less the hand-picked constant could afford."""
    margins = [red.resolve(x, 0.0, BAR_Z, tolerance=LOOSE).margin for x in BAR_TARGETS]
    assert margins == sorted(margins, reverse=True)


# --------------------------------------------------------------------------
# Agreement with the solver's own branch preference
# --------------------------------------------------------------------------

def test_the_branch_split_did_not_change_what_inverse_returns():
    """`inverse` was refactored onto `branch_solutions`. It must still prefer
    elbow-up, and still refuse rather than clamp."""
    for x in BAR_TARGETS:
        for deg in range(-110, -40, 3):
            t = math.radians(deg)
            branches = kin.branch_solutions(x, 0.0, BAR_Z, t)
            got = kin.inverse(x, 0.0, BAR_Z, t)
            legal = [q for q in branches if ARM.within_limits(q)]
            assert got == (legal[0] if legal else None)


def test_resolve_agrees_with_inverse_at_the_grasp():
    """They can differ — `inverse` takes elbow-up whenever it fits, `resolve`
    takes the roomier branch — but at the live grasp they do not, and the
    accessor that reports the difference should say so."""
    for x in BAR_TARGETS:
        got = red.resolve(x, 0.0, BAR_Z, tolerance=LOOSE)
        assert got.agrees_with_inverse
        assert got.elbow_up
