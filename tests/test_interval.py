"""The interval arithmetic has to be an over-approximation, or it proves nothing.

Everything `redundancy.py` claims rests on one property: an interval operation
returns a set containing every value the real operation takes on the inputs. If
that fails anywhere, the branch-and-bound discards a region that did contain a
solution and reports a target unreachable that the arm can in fact serve — and
it reports it with a proof attached, which is worse than reporting it with
nothing attached.

So these tests are almost all the same shape. Take random inputs, sample points
inside them, apply the real function, and insist the result lies in the
enclosure. Soundness is checked by sampling; that is not circular, because
sampling can only ever *refute* an over-approximation. A missed sample proves
the module wrong. No number of hits proves it right, which is why the case
analysis in the module is written to be read.

Two regressions are pinned by name at the bottom. Both were real, both produced
an interval whose lower bound exceeded its upper bound, and both came from the
same place: the branch cut of `atan2`.
"""

from __future__ import annotations

import math
import random

import pytest

from nl2plan_agent.ros_backend import interval as iv
from nl2plan_agent.ros_backend.interval import Interval

SAMPLES = 40
CASES = 300


@pytest.fixture(autouse=True)
def _fixed_seed():
    random.seed(20260804)


def _random_interval(lo=-4.0, hi=4.0, max_width=3.0):
    a = random.uniform(lo, hi)
    b = a + random.uniform(0.0, max_width)
    return Interval(a, b)


def _points(box: Interval, n=SAMPLES):
    yield box.lo
    yield box.hi
    yield box.mid
    for _ in range(n):
        yield random.uniform(box.lo, box.hi)


def _encloses(box: Interval, value: float, slack=1e-12):
    return box.lo - slack <= value <= box.hi + slack


# --------------------------------------------------------------------------
# Arithmetic
# --------------------------------------------------------------------------

@pytest.mark.parametrize("op", ["add", "sub", "mul"])
def test_arithmetic_encloses_every_pair_of_points(op):
    fn = {"add": lambda a, b: a + b,
          "sub": lambda a, b: a - b,
          "mul": lambda a, b: a * b}[op]
    for _ in range(CASES):
        u, v = _random_interval(), _random_interval()
        got = fn(u, v)
        for a in _points(u, 8):
            for b in _points(v, 8):
                assert _encloses(got, fn(a, b)), (op, u, v, a, b, got)


def test_squaring_is_tighter_than_multiplying_by_itself():
    """`x*x` cannot know the two factors move together; `sqr` can.

    On an interval straddling zero the product form admits negative values that
    no square ever takes. Tightness is not cosmetic here — a looser enclosure
    means more subdivision, and on the wrong function it means a search that
    never terminates.
    """
    u = Interval(-1.0, 2.0)
    assert u.sqr().lo == 0.0
    assert (u * u).lo < 0.0
    for _ in range(CASES):
        u = _random_interval()
        got = u.sqr()
        assert got.lo <= (u * u).lo + 1e-12 or got.lo >= (u * u).lo
        for a in _points(u):
            assert _encloses(got, a * a)


def test_division_by_a_straddling_interval_is_refused():
    with pytest.raises(ZeroDivisionError):
        Interval(1.0, 2.0) / Interval(-1.0, 1.0)


# --------------------------------------------------------------------------
# Transcendentals
# --------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["sin", "cos"])
def test_trig_encloses_and_finds_the_extrema(name):
    real = getattr(math, name)
    for _ in range(CASES):
        u = _random_interval(-8.0, 8.0, max_width=5.0)
        got = getattr(u, name)()
        assert -1.0 <= got.lo <= got.hi <= 1.0
        for a in _points(u):
            assert _encloses(got, real(a)), (name, u, a, got)


@pytest.mark.parametrize("name", ["sin", "cos"])
def test_trig_over_a_full_turn_is_the_whole_range(name):
    got = getattr(Interval(0.3, 0.3 + 2 * math.pi), name)()
    assert (got.lo, got.hi) == (-1.0, 1.0)


def test_trig_is_tight_when_no_extremum_is_inside():
    """Away from a turning point the enclosure should be the endpoints, near
    enough. A padded-but-tight result is the whole reason this is usable."""
    u = Interval(0.2, 0.9)
    got = u.cos()
    assert got.hi - got.lo == pytest.approx(math.cos(0.2) - math.cos(0.9), abs=1e-12)


def test_acos_encloses_and_clamps_like_the_solver():
    for _ in range(CASES):
        u = _random_interval(-1.2, 1.2, max_width=0.6)
        got = u.acos()
        # The upper end may sit a couple of ulps above `math.pi` on purpose:
        # `math.pi` is below the real pi, so `acos(-1)` under-reports and the
        # pad is what keeps the enclosure honest.
        assert 0.0 <= got.lo <= got.hi <= math.pi + 1e-15
        for a in _points(u):
            if -1.0 <= a <= 1.0:
                assert _encloses(got, math.acos(a)), (u, a, got)
    # Wholly outside the unit interval: clamped, exactly as `inverse` clamps the
    # same cosine rather than treating float noise as out of reach.
    assert Interval(1.0 + 1e-15, 1.0 + 1e-12).acos() .lo == pytest.approx(0.0, abs=1e-9)


# --------------------------------------------------------------------------
# wrapped_abs — the quantity every joint stop is tested against
# --------------------------------------------------------------------------

def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def test_wrapped_abs_encloses_the_solvers_own_wrap():
    for _ in range(CASES):
        u = _random_interval(-10.0, 10.0, max_width=4.0)
        got = u.wrapped_abs()
        assert 0.0 <= got.lo <= got.hi <= math.pi + 1e-12
        for a in _points(u):
            assert _encloses(got, abs(_wrap(a))), (u, a, got)


def test_wrapped_abs_reaches_zero_only_across_a_multiple_of_a_turn():
    assert Interval(-0.2, 0.3).wrapped_abs().lo == 0.0
    assert Interval(2 * math.pi - 0.2, 2 * math.pi + 0.3).wrapped_abs().lo == 0.0
    assert Interval(0.4, 0.9).wrapped_abs().lo == pytest.approx(0.4, abs=1e-12)


def test_wrapped_abs_is_blind_to_whole_turns():
    """The property that makes `atan2_mod`'s shifted representative sound.

    Every angle in this module reaches a joint stop through `wrapped_abs`, so an
    enclosure that is a turn out from the principal value is still the right
    answer about the stop. If this ever stopped holding, `atan2_mod` would have
    to be replaced rather than adjusted.
    """
    for _ in range(CASES):
        u = _random_interval(-3.0, 3.0, max_width=2.0)
        turns = random.choice([-2, -1, 1, 2])
        shifted = Interval(u.lo + turns * iv.TWO_PI, u.hi + turns * iv.TWO_PI)
        a, b = u.wrapped_abs(), shifted.wrapped_abs()
        assert a.lo == pytest.approx(b.lo, abs=1e-9)
        assert a.hi == pytest.approx(b.hi, abs=1e-9)


# --------------------------------------------------------------------------
# atan2 — where both of the real bugs were
# --------------------------------------------------------------------------

def _congruent_to_something_in(box: Interval, value: float, slack=1e-12):
    """`atan2_mod` promises an enclosure modulo a turn, so test it that way."""
    for turns in (-1, 0, 1):
        if _encloses(box, value + turns * iv.TWO_PI, slack):
            return True
    return False


def test_atan2_encloses_every_point_of_the_box_modulo_a_turn():
    for _ in range(CASES):
        y, x = _random_interval(-2.0, 2.0), _random_interval(-2.0, 2.0)
        got = iv.atan2_mod(y, x)
        assert got.lo <= got.hi
        for b in _points(y, 6):
            for a in _points(x, 6):
                if a == 0.0 and b == 0.0:
                    continue                      # atan2(0,0) is a convention
                assert _congruent_to_something_in(got, math.atan2(b, a)), \
                    (y, x, b, a, got)


def test_atan2_is_exact_where_the_sign_of_y_is_fixed():
    """No cut crossed, so the enclosure should be the true range, not a hull."""
    y, x = Interval(0.5, 1.5), Interval(-2.0, -0.5)
    got = iv.atan2_mod(y, x)
    lo = min(math.atan2(b, a) for b in (0.5, 1.5) for a in (-2.0, -0.5))
    hi = max(math.atan2(b, a) for b in (0.5, 1.5) for a in (-2.0, -0.5))
    assert got.lo == pytest.approx(lo, abs=1e-12)
    assert got.hi == pytest.approx(hi, abs=1e-12)


def test_a_box_on_the_branch_cut_stays_narrow_instead_of_giving_up():
    """The regression that motivated `atan2_mod` rather than a principal value.

    A box straddling the negative x-axis has `atan2` values clustered at both
    ends of [-pi, pi]. Reporting [-pi, pi] would be sound and useless: down near
    the pan axis the wrist point sits almost directly below the shoulder, that
    is exactly this case, and an enclosure that wide admits every joint value,
    so no box can ever be decided and the search subdivides until it runs out
    of budget. Reporting the values below the axis as `atan2 + 2*pi` keeps the
    interval tight around pi.
    """
    got = iv.atan2_mod(Interval(-0.01, 0.01), Interval(-1.0, -0.9))
    assert got.width < 0.05, got
    assert got.lo < math.pi < got.hi


def test_negative_zero_cannot_flip_a_corner_across_the_cut():
    """The second regression, and the subtler one.

    Negating an interval whose endpoint is zero produces -0.0. It compares equal
    to 0.0, so every test in the module sees no difference — but `atan2` reads
    its sign, and `atan2(-0.0, -1)` is -pi where `atan2(0.0, -1)` is +pi. One
    corner landing on the wrong side of the cut does not widen the enclosure, it
    inverts it, and the constructor raises on an interval whose bounds are the
    wrong way round. That is a good failure; it was found because the search
    crashed rather than because it lied.
    """
    assert math.copysign(1.0, Interval(-0.0, -0.0).lo) > 0.0
    assert math.copysign(1.0, (-Interval(0.0, 1.0)).hi) > 0.0
    # The shape of box that crashed: y touching zero from below, x negative.
    got = iv.atan2_mod(Interval(-0.5, 0.0), Interval(-1.0, -0.5))
    assert got.lo <= got.hi
    for b in (-0.5, -0.25, 0.0):
        for a in (-1.0, -0.75, -0.5):
            assert _congruent_to_something_in(got, math.atan2(b, a))


def test_the_right_half_plane_needs_no_shift():
    got = iv.atan2_mod(Interval(-1.0, 1.0), Interval(0.5, 2.0))
    assert -math.pi / 2 - 1e-9 <= got.lo <= got.hi <= math.pi / 2 + 1e-9


# --------------------------------------------------------------------------
# Bookkeeping
# --------------------------------------------------------------------------

def test_an_inverted_interval_is_refused_rather_than_silently_ordered():
    """Sorting the bounds would turn a logic error into a wrong answer."""
    with pytest.raises(ValueError):
        Interval(1.0, 0.0)


def test_splitting_terminates_on_adjacent_floats():
    """Bisection has to stop somewhere, and it must not stop by looping."""
    tiny = Interval(1.0, math.nextafter(1.0, math.inf))
    left, right = tiny.split()
    assert left.width == 0.0 and right.width == 0.0


def test_hull_and_intersect_agree_with_set_operations():
    for _ in range(CASES):
        u, v = _random_interval(), _random_interval()
        h = u.hull(v)
        assert h.lo <= min(u.lo, v.lo) and h.hi >= max(u.hi, v.hi)
        both = u.intersect(v)
        if both is None:
            assert u.hi < v.lo or v.hi < u.lo
        else:
            for a in _points(both, 5):
                assert u.contains(a) and v.contains(a)
