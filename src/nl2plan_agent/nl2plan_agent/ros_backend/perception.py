"""find_object's eyes: stop, spin in place, confirm while stationary.

The AMCL-lag guard lives here, not in the prompt: a sighting made while the
robot moves can land over a metre off, so nothing counts until four
sightings taken standing still cluster within 0.2 m.
"""

from __future__ import annotations

import math
import time
from typing import Optional

from geometry_msgs.msg import Twist
from rclpy.time import Time

from .logic import cluster_spread, mean_xy

SPIN_SPEED = 0.4                                   # rad/s while scanning
SPIN_TIMEOUT_S = 2 * math.pi / SPIN_SPEED * 1.2    # one revolution + slack
FRESH_S = 1.0        # a sighting older than this isn't "in view"
SETTLE_S = 1.0       # let AMCL settle after stopping before trusting anything
CONFIRM_WINDOW_S = 5.0
MIN_SAMPLES = 4
MAX_SPREAD_M = 0.2
# Two blocks cannot share a spot. Orange announces itself with a huge fully
# saturated blob at its true position, while its antialiased rim sheds
# pixels into every brown band worth trying — three rounds of band tuning
# all lost to it. A "brown" cluster sitting on a fresh orange detection IS
# the orange block; reject it and keep scanning.
CROSS_VETO_M = 0.35
CROSS_VETO_FRESH_S = 3.0
# Plausibility band for a confirmed cluster's distance from the robot. The
# camera can't see the floor inside 0.45 m, and every search pose puts a
# real block 0.9 m out at most. Elevated red decals (the bedroom bin's
# stars) break the ground-plane projection and confirm outside this band —
# measured live 2026-07-18 before this gate existed.
CONFIRM_DIST_MIN = 0.45
CONFIRM_DIST_MAX = 0.95


def _fresh(node, color: str) -> bool:
    age = node.block_age(color)
    return age is not None and age < FRESH_S


def _confirm(node, color: str) -> Optional[dict]:
    """Collect stationary sightings; mean (x, y) if they hold up, else None."""
    node.stop_base()
    start_stamp = node.get_clock().now()
    time.sleep(SETTLE_S)
    samples: list = []
    last_stamp = None
    deadline = time.monotonic() + CONFIRM_WINDOW_S
    while time.monotonic() < deadline:
        msg = node.blocks.get(color)
        if msg is not None:
            stamp = Time.from_msg(msg.header.stamp)
            if stamp > start_stamp and (last_stamp is None or stamp != last_stamp):
                last_stamp = stamp
                p = msg.pose.position
                samples.append((p.x, p.y))
                if len(samples) >= MIN_SAMPLES:
                    if cluster_spread(samples) < MAX_SPREAD_M:
                        x, y = mean_xy(samples)
                        if node.robot is not None:
                            d = math.hypot(x - node.robot[0], y - node.robot[1])
                            if not (CONFIRM_DIST_MIN <= d <= CONFIRM_DIST_MAX):
                                # Off the plausible floor band: a decal or
                                # reflection, not a block. Keep scanning.
                                return None
                        if _stolen_by_orange(node, color, x, y):
                            return None
                        return {"x": x, "y": y}
                    # Four sightings scattered past 20 cm are not a parked
                    # 5 cm cube — a phantom, not the block.
                    return None
        time.sleep(0.1)
    return None


def _stolen_by_orange(node, color: str, x: float, y: float) -> bool:
    """True when a brown cluster coincides with a live orange sighting."""
    if color != "brown":
        return False
    om = node.blocks.get("orange")
    age = node.block_age("orange")
    if om is None or age is None or age > CROSS_VETO_FRESH_S:
        return False
    p = om.pose.position
    return math.hypot(x - p.x, y - p.y) < CROSS_VETO_M


def scan_for(node, color: str) -> Optional[dict]:
    """One full stop-and-spin scan for `color`. {'x','y'} or None."""
    node.stop_base()
    time.sleep(SETTLE_S)
    if _fresh(node, color):
        hit = _confirm(node, color)
        if hit is not None:
            return hit

    spin_deadline = time.monotonic() + SPIN_TIMEOUT_S
    cmd = Twist()
    cmd.angular.z = SPIN_SPEED
    while time.monotonic() < spin_deadline:
        node.cmd_pub.publish(cmd)
        time.sleep(0.1)
        if _fresh(node, color):
            hit = _confirm(node, color)   # stops the base itself
            if hit is not None:
                return hit
            # Sighting didn't hold up — keep spinning out the revolution.
    node.stop_base()
    return None


def confirm_here(node, color):
    """Stationary re-confirm from the current pose; refined {'x','y'} or None.

    Used by pick at its standoff: a sighting made during the search scan can
    carry ~0.2 m of oblique-angle projection error, which the blind creep
    would faithfully reproduce. Dead-ahead at ~0.6 m the error collapses.
    """
    return _confirm(node, color)
