"""Closed-form kinematics for the arm, with no ROS import.

Same reason `logic.py` has no ROS in it: this is where a sign error quietly
aims the gripper at empty floor instead of failing, so it has to be testable
without starting a simulator.

Two conventions in the URDF are the opposite of the textbook planar arm, and
both are easy to write backwards:

* The 0.050 m arm-base offset is applied *before* the pan joint. The pan axis
  is the vertical line through x = 0.050 in `base_footprint`, so the pan angle
  is `atan2(y, x - 0.050)` — not `atan2(y, x)` with 0.050 taken off the radius.
  The two agree only on the robot's centre line, which is exactly where every
  approach puts the block, so getting it wrong stays invisible until something
  reaches off-axis.
* Every arm joint turns about +y and every link runs along its own +z, so at
  all-zero the arm points straight up rather than straight ahead. Sine and
  cosine come out swapped relative to the usual form, and the gripper's pitch
  is `pi/2 - (lift + elbow + wrist)`.

Lengths are quoted to the centre of the finger pads, not the fingertips,
because the pads are where a block is actually held. Tips are 0.130 m out.

Joint order is `(pan, lift, elbow, wrist)` throughout, matching the arrays in
`manipulation.py`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

Joints = Tuple[float, float, float, float]
Point = Tuple[float, float, float]

# acos's argument can land a few ulps outside [-1, 1] for a target that sits
# exactly at full extension. That is float noise on a reachable target, not an
# unreachable one, so it is absorbed. Anything further out returns None; the
# target itself is never clamped inward.
_COS_EPS = 1e-9


@dataclass(frozen=True)
class ArmGeometry:
    """Link lengths and joint stops, all read off the URDF joint origins."""

    pivot_x: float          # shoulder-lift pivot, forward of base centre
    pivot_z: float          # shoulder-lift pivot, above the floor
    l1: float               # shoulder-lift to elbow
    l2: float               # elbow to wrist
    l3: float               # wrist to the centre of the finger pads
    pan_limit: float        # all four stops are symmetric in this model
    lift_limit: float
    elbow_limit: float
    wrist_limit: float

    @property
    def max_extension(self) -> float:
        return self.l1 + self.l2 + self.l3

    def within_limits(self, q: Joints) -> bool:
        pan, lift, elbow, wrist = q
        return (abs(pan) <= self.pan_limit
                and abs(lift) <= self.lift_limit
                and abs(elbow) <= self.elbow_limit
                and abs(wrist) <= self.wrist_limit)

    def stop_margin(self, q: Joints) -> float:
        """How much room the tightest of the three pitch joints has left.

        Non-negative exactly when `within_limits` holds, so it is a graded
        version of the same test: zero means a joint is sitting on its stop,
        and larger is further from any of them.

        The pan is excluded deliberately. It does not depend on the approach
        pitch, so including it could only cap the objective in
        `redundancy.py` with a constant and flatten the very thing that
        search is trying to distinguish. `pan_margin` reports it separately.
        """
        _, lift, elbow, wrist = q
        return min(self.lift_limit - abs(lift),
                   self.elbow_limit - abs(elbow),
                   self.wrist_limit - abs(wrist))

    def pan_margin(self, q: Joints) -> float:
        """The pan's room to its stop. Fixed by the target, not by the pitch."""
        return self.pan_limit - abs(q[0])


ARM = ArmGeometry(
    pivot_x=0.050, pivot_z=0.260,
    l1=0.150, l2=0.120, l3=0.105,
    pan_limit=3.14, lift_limit=1.57, elbow_limit=2.5, wrist_limit=1.57,
)


def _wrap(angle: float) -> float:
    """Fold an angle into [-pi, pi]. Same pose, representable joint value."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _planar_radius(q: Joints, arm: ArmGeometry = ARM) -> float:
    """Distance of the pads from the pan axis, signed: negative is behind it."""
    _, lift, elbow, wrist = q
    return (arm.l1 * math.sin(lift)
            + arm.l2 * math.sin(lift + elbow)
            + arm.l3 * math.sin(lift + elbow + wrist))


def forward(q: Joints, arm: ArmGeometry = ARM) -> Point:
    """Where the centre of the finger pads sits, in `base_footprint`."""
    pan, lift, elbow, wrist = q
    rho = _planar_radius(q, arm)
    zeta = (arm.l1 * math.cos(lift)
            + arm.l2 * math.cos(lift + elbow)
            + arm.l3 * math.cos(lift + elbow + wrist))

    return (arm.pivot_x + rho * math.cos(pan),
            rho * math.sin(pan),
            arm.pivot_z + zeta)


def pitch_of(q: Joints, arm: ArmGeometry = ARM) -> float:
    """The gripper's approach pitch: 0 is horizontal, -pi/2 points straight down.

    Reported in the frame `inverse` works in, which is the one where the arm
    faces its target. A configuration can put the gripper *behind* the pan axis
    — the planar radius goes negative — and the identical gripper pose is then
    also spelled with the pan turned 180 degrees and the three pitch joints
    negated. `inverse` always returns that second spelling, so this reports the
    pitch of that spelling too. Without the flip the two disagree by more than
    a radian on exactly the poses nobody checks by eye.
    """
    total = q[1] + q[2] + q[3]
    if _planar_radius(q, arm) < 0.0:
        total = -total
    return _wrap(math.pi / 2 - total)


def branch_solutions(x: float, y: float, z: float, pitch: float,
                     arm: ArmGeometry = ARM) -> Tuple[Joints, ...]:
    """Both elbow spellings of this target, *before* the stops are applied.

    Elbow-up first, elbow-down second. Empty when the wrist point falls outside
    the annulus the two proximal links can span, which is the one failure that
    is about geometry rather than about stops.

    Split out of `inverse` so that `redundancy.py` can weigh the two branches
    against each other rather than re-deriving them. Two implementations of the
    same closed form is how the search and the solver end up disagreeing about
    which targets exist, and the disagreement would show up as an arm refusing
    a pose the search had just promised.
    """
    dx = x - arm.pivot_x
    rho = math.hypot(dx, y)
    zeta = z - arm.pivot_z
    pan = math.atan2(y, dx)

    # Back off along the approach direction to find where the wrist must be.
    rho_w = rho - arm.l3 * math.cos(pitch)
    zeta_w = zeta - arm.l3 * math.sin(pitch)

    c3 = ((rho_w * rho_w + zeta_w * zeta_w - arm.l1 * arm.l1 - arm.l2 * arm.l2)
          / (2 * arm.l1 * arm.l2))
    if abs(c3) > 1.0 + _COS_EPS:
        return ()
    c3 = max(-1.0, min(1.0, c3))

    base = math.atan2(rho_w, zeta_w)
    out = []
    for elbow in (math.acos(c3), -math.acos(c3)):     # elbow-up first
        lift = base - math.atan2(arm.l2 * math.sin(elbow),
                                 arm.l1 + arm.l2 * math.cos(elbow))
        wrist = (math.pi / 2 - pitch) - lift - elbow
        out.append((_wrap(pan), _wrap(lift), _wrap(elbow), _wrap(wrist)))
    return tuple(out)


def inverse(x: float, y: float, z: float, pitch: float,
            arm: ArmGeometry = ARM) -> Optional[Joints]:
    """Joint angles putting the finger pads at (x, y, z) at `pitch`, or None.

    None means the target cannot be served — either the wrist point falls
    outside the annulus the two proximal links can span, or every elbow branch
    needs a joint past its stop. It is deliberately not clamped to the nearest
    achievable pose: a caller that gets angles back is entitled to assume the
    gripper will arrive, and silently substituting a different target is how an
    arm reaches confidently for somewhere it cannot get to.

    The elbow sign gives two solutions. Elbow-up is preferred and elbow-down is
    the fallback, so a target only fails once both are out of limits.

    The preference is a convention, not a criterion: it takes the first branch
    that fits rather than the better one. `redundancy.resolve` is the entry
    point that chooses on a stated basis instead.
    """
    for q in branch_solutions(x, y, z, pitch, arm):
        if arm.within_limits(q):
            return q
    return None


def reachable(x: float, y: float, z: float, pitch: float,
              arm: ArmGeometry = ARM) -> bool:
    """Whether `inverse` can serve this target.

    Defined as "inverse returns something" rather than as an independent
    annulus test, so the two can never disagree. The annulus alone is not the
    answer anyway: a wrist point inside it can still demand a joint past a stop.
    """
    return inverse(x, y, z, pitch, arm) is not None


def max_forward_reach(pitch: float, z: float, arm: ArmGeometry = ARM,
                      resolution: float = 0.0005) -> float:
    """Furthest x on the centre line the pads can reach at this height and pitch.

    Scan then bisect, rather than a formula, so the number carries the joint
    limits with it. This is what a reach constant should be checked against —
    the horizontal full-extension figure flatters the arm badly for anything
    near the floor.

    The scan is not laziness. Near the floor the reachable band does not start
    at the pan axis: the arm cannot fold far enough to point straight down at
    its own feet, so `x = pivot_x` is unreachable and bisecting down from the
    far edge would need a bracket that does not exist. Returns NaN when nothing
    on the centre line is reachable at this height and pitch.
    """
    span = arm.max_extension
    steps = max(int(span / resolution), 1)
    best = None
    for i in range(steps + 1):
        x = arm.pivot_x + span * i / steps
        if reachable(x, 0.0, z, pitch, arm):
            best = x
    if best is None:
        return float("nan")

    lo, hi = best, min(best + span / steps, arm.pivot_x + span)
    while hi - lo > resolution / 10:
        mid = (lo + hi) / 2
        if reachable(mid, 0.0, z, pitch, arm):
            lo = mid
        else:
            hi = mid
    return lo
