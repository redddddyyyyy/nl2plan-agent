"""Interval arithmetic, just enough of it to certify a search instead of sampling it.

This exists because of a specific criticism the project has to survive. A grid
scan and a simulator run are the same epistemic object: both evaluate finitely
many points and report that nothing went wrong at any of them. Neither
establishes anything about the points in between, and the whole stance of this
work is that establishing something is the job. A scan that misses a one-degree
window is indistinguishable from a scan that proves the window is not there.

An interval evaluation is different in kind. Give the functions below an
interval and they return an interval *guaranteed to contain every value the
real function takes on it*. So `f(X).lo > 0` is a proof that `f` is positive
everywhere on `X` — over a continuum, not a sample of it. Branch-and-bound
turns that into a decision procedure: subdivide only the boxes that are still
undecided, and stop when the undecided part is thinner than the tolerance you
were asked for. What comes out is an inner and an outer bound with the answer
provably sandwiched between them.

**On soundness, honestly.** CPython exposes no directed rounding, so results
are padded outward by a fixed number of ulps after every operation instead.
That is sound provided the platform's libm is faithful to within a couple of
ulps on `sin`, `cos`, `acos`, `atan2`, `sqrt` and `hypot`, which is true of
every libm anyone runs this on but is not machine-checked here. The padding is
the one place where this module asks to be believed rather than checked; it is
recorded because the difference between a proof and an argument is exactly the
kind of thing this codebase is supposed to be careful about. Everything else
below is exact case analysis.

Written for `redundancy.py`, which searches over one variable. Kept general
because the same machinery is what replaces the remaining grid scans over the
full joint box.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple, Union

__all__ = ["Interval", "atan2_mod", "hypot", "imin", "imax", "UNIT", "TWO_PI"]

TWO_PI = 2.0 * math.pi

# Ulps of outward padding applied after every operation. Two is one for the
# operation's own rounding and one of slack for a libm that is faithful rather
# than correctly rounded.
_PAD = 2

# Lattice-membership tests below are biased *towards* answering "yes". Saying a
# maximum of cosine is present when it is only nearly present widens the result,
# which stays sound; saying it is absent when it is present would narrow it,
# which would not.
_LATTICE_SLOP = 1e-14


_NEG_INF = -math.inf
_POS_INF = math.inf
_nextafter = math.nextafter


# Unrolled rather than looped over `_PAD`. These two run a quarter of a million
# times per call to `resolve`, and at that count the loop and the global lookups
# are a measurable fraction of the search.
def _down(v: float) -> float:
    return _nextafter(_nextafter(v, _NEG_INF), _NEG_INF)


def _up(v: float) -> float:
    return _nextafter(_nextafter(v, _POS_INF), _POS_INF)


def _holds_lattice_point(lo: float, hi: float, offset: float,
                         period: float = TWO_PI) -> bool:
    """Whether some `offset + k*period`, k integral, lies in `[lo, hi]`."""
    k = math.ceil((lo - offset) / period - _LATTICE_SLOP)
    return offset + k * period <= hi + _LATTICE_SLOP * (1.0 + abs(hi))


@dataclass(frozen=True, slots=True)
class Interval:
    """A closed real interval. Every operation returns a guaranteed enclosure."""

    lo: float
    hi: float

    def __post_init__(self) -> None:
        if not (self.lo <= self.hi):
            raise ValueError(f"degenerate interval [{self.lo}, {self.hi}]")
        # Normalise negative zero. It compares equal to zero, so it is invisible
        # to every test here, but `atan2` reads its sign: `atan2(-0.0, -1)` is
        # -pi where `atan2(0.0, -1)` is +pi. Negating an interval whose endpoint
        # is zero is enough to produce one, and a corner that lands on the wrong
        # side of the branch cut inverts the enclosure rather than widening it.
        if self.lo == 0.0:
            object.__setattr__(self, "lo", 0.0)
        if self.hi == 0.0:
            object.__setattr__(self, "hi", 0.0)

    # -- basics ------------------------------------------------------------

    @property
    def width(self) -> float:
        return self.hi - self.lo

    @property
    def mid(self) -> float:
        return 0.5 * (self.lo + self.hi)

    def contains(self, v: float) -> bool:
        return self.lo <= v <= self.hi

    def hull(self, other: "Interval") -> "Interval":
        return Interval(min(self.lo, other.lo), max(self.hi, other.hi))

    def intersect(self, other: "Interval") -> Optional["Interval"]:
        lo, hi = max(self.lo, other.lo), min(self.hi, other.hi)
        return Interval(lo, hi) if lo <= hi else None

    def split(self) -> Tuple["Interval", "Interval"]:
        m = self.mid
        # A midpoint that rounds onto an endpoint means the interval is down to
        # adjacent floats; halving it further would loop forever.
        if not (self.lo < m < self.hi):
            return Interval(self.lo, self.lo), Interval(self.hi, self.hi)
        return Interval(self.lo, m), Interval(m, self.hi)

    # -- arithmetic --------------------------------------------------------

    def __neg__(self) -> "Interval":
        return Interval(-self.hi, -self.lo)

    def __add__(self, other: Union[float, "Interval"]) -> "Interval":
        o = _as_interval(other)
        return Interval(_down(self.lo + o.lo), _up(self.hi + o.hi))

    __radd__ = __add__

    def __sub__(self, other: Union[float, "Interval"]) -> "Interval":
        o = _as_interval(other)
        return Interval(_down(self.lo - o.hi), _up(self.hi - o.lo))

    def __rsub__(self, other: Union[float, "Interval"]) -> "Interval":
        return _as_interval(other) - self

    def __mul__(self, other: Union[float, "Interval"]) -> "Interval":
        o = _as_interval(other)
        corners = (self.lo * o.lo, self.lo * o.hi, self.hi * o.lo, self.hi * o.hi)
        return Interval(_down(min(corners)), _up(max(corners)))

    __rmul__ = __mul__

    def __truediv__(self, other: Union[float, "Interval"]) -> "Interval":
        o = _as_interval(other)
        if o.lo <= 0.0 <= o.hi:
            raise ZeroDivisionError("interval divisor straddles zero")
        corners = (self.lo / o.lo, self.lo / o.hi, self.hi / o.lo, self.hi / o.hi)
        return Interval(_down(min(corners)), _up(max(corners)))

    def sqr(self) -> "Interval":
        """Tighter than `self * self`, which forgets the two factors are equal."""
        a, b = self.lo * self.lo, self.hi * self.hi
        if self.lo >= 0.0:
            return Interval(_down(a), _up(b))
        if self.hi <= 0.0:
            return Interval(_down(b), _up(a))
        return Interval(0.0, _up(max(a, b)))

    def sqrt(self) -> "Interval":
        if self.hi < 0.0:
            raise ValueError("sqrt of a wholly negative interval")
        return Interval(_down(math.sqrt(max(self.lo, 0.0))), _up(math.sqrt(self.hi)))

    # -- transcendentals ---------------------------------------------------

    def cos(self) -> "Interval":
        if self.width >= TWO_PI:
            return Interval(-1.0, 1.0)
        ends = (math.cos(self.lo), math.cos(self.hi))
        hi = 1.0 if _holds_lattice_point(self.lo, self.hi, 0.0) else max(ends)
        lo = -1.0 if _holds_lattice_point(self.lo, self.hi, math.pi) else min(ends)
        # Clipped back to the true range of cosine after padding. Widening past
        # +-1 is sound but pointless, and it costs precision downstream where
        # this feeds a cosine rule and then an acos.
        return Interval(max(-1.0, _down(lo)), min(1.0, _up(hi)))

    def sin(self) -> "Interval":
        if self.width >= TWO_PI:
            return Interval(-1.0, 1.0)
        ends = (math.sin(self.lo), math.sin(self.hi))
        hi = 1.0 if _holds_lattice_point(self.lo, self.hi, math.pi / 2) else max(ends)
        lo = -1.0 if _holds_lattice_point(self.lo, self.hi, -math.pi / 2) else min(ends)
        return Interval(max(-1.0, _down(lo)), min(1.0, _up(hi)))

    def acos(self) -> "Interval":
        """Decreasing on [-1, 1]. The argument is clamped, not rejected.

        Clamping mirrors what `kinematics.inverse` does with the same quantity:
        a cosine a few ulps outside the unit interval is float noise on a target
        at full extension, not a target beyond it. Callers that need to know
        whether the excursion was real test the unclamped interval themselves.
        """
        lo = math.acos(min(max(self.hi, -1.0), 1.0))
        hi = math.acos(min(max(self.lo, -1.0), 1.0))
        # `acos` is never negative. The upper end is left padded: `math.pi` is
        # a shade below the real pi, so `acos(-1)` genuinely under-reports.
        return Interval(max(0.0, _down(lo)), _up(hi))

    # -- the one non-obvious primitive -------------------------------------

    def wrapped_abs(self) -> "Interval":
        """Encloses `abs(kinematics._wrap(t))` for every `t` in this interval.

        `_wrap` folds an angle into [-pi, pi], so `abs(_wrap(t))` is the
        distance from `t` to the nearest multiple of 2*pi: a sawtooth, zero on
        the even multiples of pi and pi on the odd ones. Piecewise linear with
        known extrema, so the range is exact rather than estimated.

        This is the quantity every joint stop is tested against, which is why it
        gets a primitive of its own. It also makes an interval evaluation
        immune to the 2*pi jumps in `atan2_mod` below: a representative that is
        one turn out lands on the same tooth.
        """
        if self.width >= TWO_PI:
            return Interval(0.0, math.pi)
        d_lo = abs((self.lo + math.pi) % TWO_PI - math.pi)
        d_hi = abs((self.hi + math.pi) % TWO_PI - math.pi)
        lo = 0.0 if _holds_lattice_point(self.lo, self.hi, 0.0) else min(d_lo, d_hi)
        hi = math.pi if _holds_lattice_point(self.lo, self.hi, math.pi) else max(d_lo, d_hi)
        # A distance is never negative; the far end still needs its pad, since
        # `math.pi` sits just below the real pi.
        return Interval(max(0.0, _down(lo)), _up(hi))

    def __repr__(self) -> str:      # pragma: no cover - debugging aid
        return f"[{self.lo:.9g}, {self.hi:.9g}]"


UNIT = Interval(-1.0, 1.0)


def _as_interval(v: Union[float, int, Interval]) -> Interval:
    if type(v) is Interval:
        return v
    f = float(v)
    return Interval(f, f)


def hypot(x: float, y: float) -> Interval:
    h = math.hypot(x, y)
    return Interval(_down(h), _up(h))


def imin(*xs: Interval) -> Interval:
    return Interval(min(x.lo for x in xs), min(x.hi for x in xs))


def imax(*xs: Interval) -> Interval:
    return Interval(max(x.lo for x in xs), max(x.hi for x in xs))


def _atan2_upper(y: Interval, x: Interval) -> Interval:
    """`atan2` over a box in the closed upper half-plane. Requires `y.lo >= 0`.

    There the function is continuous with values in [0, pi] — the negative
    x-axis is the *edge* of this half-plane, and `atan2(0, negative)` returns
    `+pi`, which is the correct limit from above rather than a jump. It is
    monotone in each argument once the sign of the other is fixed:
    `d/dy = x/r**2` and `d/dx = -y/r**2`. Splitting `x` at zero fixes both
    signs, after which the extrema sit at corners and the enclosure is exact.
    """
    pieces = []
    for xp in _split_at_zero(x):
        # Increasing in y where x >= 0, decreasing where x <= 0.
        y_at_lo, y_at_hi = (y.lo, y.hi) if xp.lo >= 0.0 else (y.hi, y.lo)
        # Decreasing in x throughout, since y >= 0 here.
        x_at_lo, x_at_hi = xp.hi, xp.lo
        pieces.append(Interval(_down(math.atan2(y_at_lo, x_at_lo)),
                               _up(math.atan2(y_at_hi, x_at_hi))))
    out = pieces[0]
    for p in pieces[1:]:
        out = out.hull(p)
    return out


def _atan2_signed_y(y: Interval, x: Interval) -> Interval:
    """`atan2` over a box on which the sign of `y` does not change.

    The lower half-plane is done by reflecting the upper one, rather than by
    the same corner analysis with the signs turned round. The reason is that
    the closed lower half-plane *touches* the branch cut along its own edge:
    `atan2(0, negative)` is `+pi` by definition, while every nearby point with
    `y < 0` is close to `-pi`. Corner analysis on that piece produces a lower
    bound above its upper bound and the enclosure is not merely loose but
    ill-formed. Reflecting instead reports the edge as `-pi`, which is the
    limit from below, and so is continuous on the piece.

    That makes the result a representative modulo 2*pi rather than the
    principal value — see `atan2_mod`, which is the only intended caller.
    """
    if y.lo >= 0.0:
        return _atan2_upper(y, x)
    return -_atan2_upper(-y, x)


def _split_at_zero(v: Interval):
    if v.lo >= 0.0 or v.hi <= 0.0:
        return (v,)
    return (Interval(v.lo, 0.0), Interval(0.0, v.hi))


def atan2_mod(y: Interval, x: Interval) -> Interval:
    """Encloses `atan2(y, x)` **modulo 2*pi** over the box.

    The qualifier is the whole subtlety. `atan2` jumps by 2*pi across the
    negative x-axis, so on a box straddling that axis no interval in [-pi, pi]
    can contain the values — they cluster at both ends. The honest options are
    to give up and return [-pi, pi], or to return a tight interval around pi by
    reporting the values below the axis as `atan2 + 2*pi`. This does the second.

    Giving up is not free. Down near the pan axis the wrist point sits almost
    directly below the shoulder, which is exactly the straddling case, and a
    [-pi, pi] answer there swamps the joint-limit test: the enclosure admits
    every margin, nothing can be decided, and branch-and-bound subdivides
    forever without converging. The shifted representative keeps those boxes
    decidable.

    It is sound for this caller because every consumer of an angle here goes
    through `wrapped_abs`, or through a sum that itself goes through
    `wrapped_abs`, and both are invariant under whole turns. It would not be
    sound for a caller that wanted the principal value; hence the name.
    """
    if y.lo >= 0.0 or y.hi <= 0.0:
        return _atan2_signed_y(y, x)

    upper = _atan2_signed_y(Interval(0.0, y.hi), x)
    lower = _atan2_signed_y(Interval(y.lo, 0.0), x)
    if x.lo >= 0.0:
        # Right half-plane: the cut is nowhere near, the two halves already join
        # continuously through zero.
        return upper.hull(lower)
    # The cut is inside the box. Lift the lower half by a turn so the two halves
    # meet at pi instead of tearing apart at -pi and +pi.
    return upper.hull(lower + TWO_PI)
