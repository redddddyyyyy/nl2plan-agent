"""Resolving the arm's one redundant degree of freedom by a stated criterion.

Three joints act in the vertical plane; a position target constrains two
degrees of freedom. The arm is redundant by one, and fixing the approach pitch
`phi` is what removes the redundancy and makes `kinematics.inverse` closed-form.

Nothing in the system chose `phi`. Every call site supplied a hand-picked
constant, which meant the arm's last free parameter was set by whoever typed
the pose table, on no recorded basis, and could not be defended or audited. A
constant that happens to work is indistinguishable from one that happens to
work *today*, and the difference shows up when the target moves.

This module replaces the constants with two definitions.

**The feasible pitch set.**

    Phi(x, y, z) = { phi : inverse(x, y, z, phi) is not None }

Non-empty exactly when the target is reachable at all, so `Phi` is the honest
reachability predicate; the pitch-conditional one is a slice through it.

**The margin-optimal pitch.** With the stop margin
`m(q) = min_i (limit_i - |q_i|)` over the three pitch joints,

    phi* = argmax_{phi in Phi} m(q(phi))

Maximising the distance to the nearest stop is not the only defensible
criterion — minimum joint travel and manipulability are the other obvious ones
— but it is the one that answers the question this arm keeps failing. The
binding joint at every grasp is `shoulder_lift`, the same stop that puts the
floor out of reach; margin is the quantity that was silently at zero when the
old 50 mm cube was in use, and a pose sitting on a stop has no room to absorb
the approach error that hardware will bring.

**Why this is not a grid scan.** Section 3.8 of the theory writeup lists the
1-degree sweep used to compute the tables as one of three places where sampling
was standing in for proof, and a grid can miss a narrow feasible window
entirely — the difference between "no window here" and "no window at the
points I looked" is the whole argument of the project. So both computations
below run as interval branch-and-bound over `phi`, using `interval.py`:
subdivide, discard any sub-interval that provably contains no solution, keep
any that provably contains only solutions, and stop when what is left is
thinner than the tolerance asked for. `Phi` comes back as an inner and an outer
bound with the true set provably between them, and `phi*` comes back with a
bound on how much better any other pitch could possibly be. This is the
technique the writeup prescribes for the joint box; `phi` is one-dimensional,
so it is cheap here, and the machinery is then in hand for the harder case.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from . import interval as iv
from . import kinematics as kin
from .interval import Interval
from .kinematics import ARM, ArmGeometry, Joints

__all__ = [
    "FeasiblePitchSet", "Approach",
    "feasible_pitches", "resolve", "approach_pitch",
]

# Enough to place the gripper a good deal finer than any servo can hold, and
# far finer than the URDF's own numbers are known.
_PITCH_RESOLUTION = 1e-6        # rad, for the boundary of Phi

# The certified optimality gap for phi*, in radians of margin. Asking for less
# gets expensive fast and buys nothing: the objective is smooth and curved at
# its maximum, so its enclosure over a box of width w is only about w tall
# however close the box is, and certifying a gap of t therefore takes O(1/sqrt(t))
# boxes. 1e-6 rad is four orders below what the servos resolve and further still
# below the confidence anyone should have in the URDF's link lengths, which were
# read off a model rather than measured. 1e-9 costs ten seconds a target for a
# number with no physical referent.
_MARGIN_TOLERANCE = 1e-6

_MAX_BOXES = 200_000

# The whole turn, which is the complete domain: the pose depends on the pitch
# only through its sine and cosine and through joint values that are wrapped,
# so a pitch and that pitch plus a turn are the same command.
FULL_TURN = (-math.pi, math.pi)

# Approaches from above, between horizontal and straight down. Not a property
# of the arm — the arm reaches well past straight down — but the band a grasp
# usually wants, and worth naming so that restricting to it is a decision on
# the record rather than a scan that quietly stopped at its own edge. The
# tables in the writeup were computed over exactly this band and reported as
# though it were all of Phi; the lower endpoint came out at -90 degrees for
# every target, which is what a clipped sweep looks like.
FROM_ABOVE = (-math.pi / 2, 0.0)


@dataclass(frozen=True)
class FeasiblePitchSet:
    """Certified enclosure of `Phi(x, y, z)`.

    `inner` is a union of intervals every one of whose pitches is *proved*
    servable; `outer` contains `inner` and every pitch that might be. The truth
    satisfies `inner <= Phi <= outer`, and they differ only inside boxes
    narrower than `resolution`, so the boundary of `Phi` is located to within
    that. An empty `outer` is a proof the target is unreachable at any approach
    pitch whatsoever — not a report that nothing was found.
    """

    inner: Tuple[Tuple[float, float], ...]
    outer: Tuple[Tuple[float, float], ...]
    resolution: float

    def __bool__(self) -> bool:
        return bool(self.outer)

    @property
    def is_empty(self) -> bool:
        """True only when unreachability is proved, not merely unwitnessed."""
        return not self.outer

    @property
    def measure(self) -> float:
        """Total width of the inner bound, in radians: how much pitch freedom."""
        return sum(hi - lo for lo, hi in self.inner)

    def contains(self, pitch: float) -> bool:
        """Whether this pitch is *proved* feasible. Undecided reads as False."""
        return any(lo <= pitch <= hi for lo, hi in self.inner)

    def degrees(self) -> Tuple[Tuple[float, float], ...]:
        """`inner`, in degrees, for tables and log lines."""
        return tuple((math.degrees(lo), math.degrees(hi)) for lo, hi in self.inner)


@dataclass(frozen=True)
class Approach:
    """A target with its redundancy resolved, and the evidence for the choice."""

    pitch: float                # phi*
    joints: Joints              # the configuration to command
    margin: float               # m(q) at phi*, radians to the nearest stop
    elbow_up: bool              # which branch won
    margin_bound: float         # no pitch anywhere does better than this
    binding_joint: str          # which stop rations the margin
    pan_margin: float           # the pitch-independent one, reported separately

    @property
    def optimality_gap(self) -> float:
        """Certified: `margin` is within this of the best any pitch achieves."""
        return self.margin_bound - self.margin

    @property
    def agrees_with_inverse(self) -> bool:
        """Whether `inverse(target, phi*)` returns this same configuration.

        Usually yes. It can be no: `inverse` takes elbow-up whenever elbow-up
        fits, while this module compares the branches and takes the one with
        more room. When they disagree, `joints` is the answer with the argument
        behind it, so command `joints` rather than re-deriving it.
        """
        x, y, z = kin.forward(self.joints)
        got = kin.inverse(x, y, z, self.pitch)
        return got is not None and all(
            abs(a - b) <= 1e-9 for a, b in zip(got, self.joints))


@dataclass(frozen=True)
class _Target:
    """A target reduced to the plane the pitch search happens in."""

    x: float
    y: float
    z: float
    rho: float                  # pads' distance from the pan axis
    zeta: float                 # pads' height above the lift pivot
    pan: float
    d: Interval                 # |(rho, zeta)|, from the pivot
    psi: Interval               # its bearing, so the wrist circle needs one cos
    arm: ArmGeometry


def _prepare(x: float, y: float, z: float, arm: ArmGeometry) -> Optional[_Target]:
    """Fold the target into the plane, or refuse it on the one pitch-free test.

    The pan angle does not depend on the approach pitch, so if the target sits
    behind the pan stop no pitch can help and the search is over before it
    starts. Everything after this point is genuinely one-dimensional.
    """
    dx = x - arm.pivot_x
    rho = math.hypot(dx, y)
    zeta = z - arm.pivot_z
    pan = kin._wrap(math.atan2(y, dx))
    if abs(pan) > arm.pan_limit:
        return None
    return _Target(
        x=x, y=y, z=z, rho=rho, zeta=zeta, pan=pan,
        d=iv.hypot(rho, zeta),
        psi=iv.atan2_mod(Interval(zeta, zeta), Interval(rho, rho)),
        arm=arm,
    )


def _enclose(box: Interval, tgt: _Target,
             elbow_up: bool) -> Optional[Tuple[Interval, bool]]:
    """Enclose the stop margin of one elbow branch over a whole range of pitches.

    Returns `(margin, branch_exists_everywhere)`, or None when the branch
    provably exists at no pitch in the box. `margin.lo >= 0` together with the
    flag is a proof that every pitch in the box is servable; `margin.hi < 0` is
    a proof that none is.

    Mirrors `kinematics.branch_solutions` term for term, with one substitution.
    Where the solver squares the wrist offsets, this uses the identity

        rho_w**2 + zeta_w**2  =  d**2 + L3**2 - 2*L3*d*cos(phi - psi)

    which is the same number — the wrist point rides a circle of radius L3 about
    the target — but mentions `phi` once instead of twice. Interval arithmetic
    charges for every repeated mention of a variable, because it has no way to
    know two occurrences move together; a form with one occurrence is tight
    where the literal transcription would be loose enough to stall the search.
    The two are asserted equal in the tests.
    """
    arm = tgt.arm
    l1, l2, l3 = arm.l1, arm.l2, arm.l3

    r_sq = tgt.d.sqr() + l3 * l3 - (2.0 * l3) * tgt.d * (box - tgt.psi).cos()
    c3 = (r_sq - (l1 * l1 + l2 * l2)) / (2.0 * l1 * l2)

    # The annulus test, exactly as the solver spells it, including the tolerance
    # that keeps float noise at full extension from reading as out of reach.
    reach = Interval(-1.0 - kin._COS_EPS, 1.0 + kin._COS_EPS)
    if c3.intersect(reach) is None:
        return None
    exists = reach.lo <= c3.lo and c3.hi <= reach.hi

    elbow_abs = c3.acos()                       # in [0, pi]; clamps as the solver does
    elbow = elbow_abs if elbow_up else -elbow_abs

    rho_w = tgt.rho - l3 * box.cos()
    zeta_w = tgt.zeta - l3 * box.sin()
    base = iv.atan2_mod(rho_w, zeta_w)
    gamma = iv.atan2_mod(l2 * elbow.sin(), l1 + l2 * elbow.cos())

    lift = base - gamma
    wrist = (math.pi / 2 - box) - lift - elbow

    margin = iv.imin(
        arm.lift_limit - lift.wrapped_abs(),
        # elbow_abs is already the wrapped absolute value: acos lands in [0, pi].
        arm.elbow_limit - elbow_abs,
        arm.wrist_limit - wrist.wrapped_abs(),
    )
    return margin, exists


def _evaluate(pitch: float, tgt: _Target,
              elbow_up: bool) -> Optional[Tuple[Joints, float]]:
    """The margin at a single pitch, straight out of the production solver.

    Deliberately not an interval evaluation. Every candidate the search reports
    is a pose `kinematics` itself produced and passed its own limit test, so no
    approximation in this module can promote an unreachable target into a
    reachable one. The interval machinery is used only to *exclude*.
    """
    branches = kin.branch_solutions(tgt.x, tgt.y, tgt.z, pitch, tgt.arm)
    if not branches:
        return None
    q = branches[0 if elbow_up else 1]
    if not tgt.arm.within_limits(q):
        return None
    return q, tgt.arm.stop_margin(q)


def _merge(boxes: Sequence[Interval], slop: float) -> Tuple[Tuple[float, float], ...]:
    """Coalesce a pile of sub-intervals into the intervals they actually form."""
    if not boxes:
        return ()
    ordered = sorted(boxes, key=lambda b: b.lo)
    out: List[List[float]] = [[ordered[0].lo, ordered[0].hi]]
    for b in ordered[1:]:
        if b.lo <= out[-1][1] + slop:
            out[-1][1] = max(out[-1][1], b.hi)
        else:
            out.append([b.lo, b.hi])
    return tuple((lo, hi) for lo, hi in out)


def _domain(bounds: Tuple[float, float]) -> Interval:
    lo, hi = bounds
    if hi - lo > iv.TWO_PI:
        raise ValueError("a pitch domain wider than a turn repeats itself")
    return Interval(lo, hi)


def feasible_pitches(x: float, y: float, z: float,
                     arm: ArmGeometry = ARM,
                     domain: Tuple[float, float] = FULL_TURN,
                     resolution: float = _PITCH_RESOLUTION,
                     max_boxes: int = _MAX_BOXES) -> FeasiblePitchSet:
    """Compute `Phi(x, y, z)` as a certified pair of bounds.

    Searches the whole turn by default, which is complete. Narrow `domain` to
    impose a preference — `FROM_ABOVE` for grasps that must come down onto the
    object — and the result is then `Phi` intersected with that band, still
    certified. Restricting is a stated choice; the point of the default is that
    nothing is excluded by accident.

    Cost is proportional to the number of boundary points of `Phi`, not to
    `1/resolution` — the interior and the exterior are settled in a handful of
    boxes each and only the edges get subdivided.
    """
    empty = FeasiblePitchSet((), (), resolution)
    tgt = _prepare(x, y, z, arm)
    if tgt is None:
        return empty

    inside: List[Interval] = []
    undecided: List[Interval] = []
    stack = [_domain(domain)]
    budget = max_boxes

    while stack:
        box = stack.pop()
        if budget <= 0:
            undecided.append(box)
            continue
        budget -= 1

        proved_in = False
        possible = False
        for elbow_up in (True, False):
            found = _enclose(box, tgt, elbow_up)
            if found is None:
                continue
            margin, exists = found
            if exists and margin.lo >= 0.0:
                proved_in = True
                break
            if margin.hi >= 0.0:
                possible = True

        if proved_in:
            inside.append(box)
        elif not possible:
            continue                        # proved empty, over a continuum
        elif box.width <= resolution:
            undecided.append(box)
        else:
            stack.extend(box.split())

    return FeasiblePitchSet(
        inner=_merge(inside, resolution),
        outer=_merge(list(inside) + undecided, resolution),
        resolution=resolution,
    )


def resolve(x: float, y: float, z: float,
            arm: ArmGeometry = ARM,
            domain: Tuple[float, float] = FULL_TURN,
            tolerance: float = _MARGIN_TOLERANCE,
            max_boxes: int = _MAX_BOXES) -> Optional[Approach]:
    """Serve this target at the approach pitch that keeps the most room to a stop.

    This is the entry point a caller should use instead of picking a pitch:
    hand it a point and it returns the configuration to command, the pitch it
    chose, and a certificate that no other pitch beats it by more than
    `tolerance`. None means no pitch in `domain` serves the target — a proof of
    unreachability, since the search excludes over continua rather than failing
    to find. With the default domain that is unreachability outright; with a
    narrowed one it is unreachability under the stated preference, and the two
    should not be reported as though they were the same claim.

    The branch-and-bound is a best-first maximisation. Every candidate is
    evaluated through `kinematics.branch_solutions` and its own limit test, so
    the answer is always a pose the solver stands behind; intervals only decide
    which regions of pitch cannot possibly hold anything better and can be
    dropped unexamined. Both elbow branches compete, so the criterion settles
    the discrete freedom as well as the continuous one — `inverse`'s preference
    for elbow-up is a tie-break, not a reason.
    """
    tgt = _prepare(x, y, z, arm)
    if tgt is None:
        return None

    best: Optional[Tuple[float, float, Joints, bool]] = None    # margin, pitch, q, up

    def offer(pitch: float) -> None:
        nonlocal best
        for elbow_up in (True, False):
            got = _evaluate(pitch, tgt, elbow_up)
            if got is None:
                continue
            q, m = got
            if best is None or m > best[0]:
                best = (m, pitch, q, elbow_up)

    def bound(box: Interval) -> Optional[float]:
        """The most margin any pitch in this box could have, on either branch."""
        ubs = [found[0].hi for found in
               (_enclose(box, tgt, up) for up in (True, False)) if found is not None]
        return max(ubs) if ubs else None

    whole = _domain(domain)
    for seed in (whole.lo, whole.mid, whole.hi):
        offer(seed)

    top = bound(whole)
    if top is None:
        return None
    heap: List[Tuple[float, int, Interval]] = [(-top, 0, whole)]
    tick = 1
    budget = max_boxes
    frontier = top

    while heap:
        neg_ub, _, box = heapq.heappop(heap)
        ub = -neg_ub
        frontier = ub
        # Best-first, so the head of the heap bounds everything still queued.
        if best is not None and ub <= best[0] + tolerance:
            break
        if budget <= 0 or box.width <= 0.0:
            break
        budget -= 1

        for child in box.split():
            offer(child.mid)
            child_ub = bound(child)
            # A child that cannot beat the incumbent is discarded, and with it
            # every pitch inside it — this is where the continuum gets covered.
            if child_ub is None or child_ub < 0.0:
                continue
            if best is not None and child_ub <= best[0]:
                continue
            heapq.heappush(heap, (-child_ub, tick, child))
            tick += 1
    else:
        frontier = best[0] if best is not None else 0.0

    if best is None:
        return None

    margin, pitch, q, elbow_up = best
    _, lift, elbow, wrist = q
    binding = min(
        (("shoulder_lift", arm.lift_limit - abs(lift)),
         ("elbow", arm.elbow_limit - abs(elbow)),
         ("wrist", arm.wrist_limit - abs(wrist))),
        key=lambda pair: pair[1],
    )[0]

    return Approach(
        pitch=pitch,
        joints=q,
        margin=margin,
        elbow_up=elbow_up,
        margin_bound=max(frontier, margin),
        binding_joint=binding,
        pan_margin=arm.pan_margin(q),
    )


def approach_pitch(x: float, y: float, z: float,
                   arm: ArmGeometry = ARM,
                   domain: Tuple[float, float] = FULL_TURN) -> Optional[float]:
    """Just `phi*`, for callers that only want the number for a table."""
    got = resolve(x, y, z, arm, domain=domain)
    return None if got is None else got.pitch
