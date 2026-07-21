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

from .logic import cluster_spread, mean_xy, to_robot_frame

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
# camera can't see the floor inside 0.45 m; elevated red decals (the
# bedroom bin's stars) break the ground-plane projection and confirm
# outside this band — measured live 2026-07-18 before this gate existed.
# The ceiling is set by where Nav2 actually parks, not by the nominal
# search poses: blocks sit 0.7-0.8 m from those, but goal tolerance plus
# AMCL error routinely leaves the robot ~1.0-1.2 m out, and a 0.95 ceiling
# rejected TRUE sightings there (measured live 2026-07-21: real clusters
# at 0.954-1.185 m across gym, lounge and sofa — the "robot stares at the
# block then walks away" bug). Phantom risk at the wider ceiling is thin:
# the detector's own 0.45-1.4 m projection band and size-distance gate now
# kill the bin-decal/wall-trim phantoms upstream (three clean red scans at
# the bedroom pose, 2026-07-21, zero phantom clusters).
CONFIRM_DIST_MIN = 0.45
CONFIRM_DIST_MAX = 1.30

# Post-mortem of the most recent scan_for/confirm_here call. The tool result
# the LLM sees stays a bare "not visible" on purpose (a 7B model chases
# numbers), but "not visible" has at least three distinct causes — detector
# silent for the whole spin, sightings too stale, or the confirm gate
# rejecting live ones — and the trace couldn't tell them apart. Live runs
# 2026-07-20 15:05-15:29 failed at EVERY pose (orange at lounge included)
# and this is what pins which layer went blind.
LAST_SCAN: dict = {}


def _fresh(node, color: str) -> bool:
    age = node.block_age(color)
    return age is not None and age < FRESH_S


def _note(why: str, **numbers):
    LAST_SCAN.setdefault("confirms", []).append(
        {"why": why, **{k: round(v, 3) if isinstance(v, float) else v
                        for k, v in numbers.items()}})


def _confirm(node, color: str) -> Optional[dict]:
    """Collect stationary sightings; mean (x, y) if they hold up, else None."""
    node.stop_base()
    start_stamp = node.get_clock().now()
    time.sleep(SETTLE_S)
    samples: list = []
    rel_samples: list = []
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
                # Cluster in the robot's own frame: AMCL's heading can be
                # tens of degrees out (measured 24 deg at bedroom_window),
                # which smears map-frame samples of a parked block right
                # past MAX_SPREAD_M and reports "not visible" on a block in
                # plain view. The relative geometry is steady regardless.
                rel_samples.append(to_robot_frame(p.x, p.y, node.robot)
                                   if node.robot is not None else (p.x, p.y))
                if len(samples) >= MIN_SAMPLES:
                    if cluster_spread(rel_samples) < MAX_SPREAD_M:
                        x, y = mean_xy(samples)
                        rel = mean_xy(rel_samples)
                        if node.robot is not None:
                            d = math.hypot(x - node.robot[0], y - node.robot[1])
                            if not (CONFIRM_DIST_MIN <= d <= CONFIRM_DIST_MAX):
                                # Off the plausible floor band: a decal or
                                # reflection, not a block. Keep scanning.
                                _note("dist_band", dist=d, x=x, y=y)
                                return None
                        if _stolen_by_orange(node, color, x, y):
                            _note("orange_veto", x=x, y=y)
                            return None
                        _note("ok")
                        # 'rel' is the offset as measured off the robot's own
                        # nose. pick re-anchors it through the robot pose at
                        # approach time, so a map estimate that shifts in
                        # between cannot steer the creep into empty floor.
                        return {"x": x, "y": y, "rel": rel}
                    # Four sightings scattered past 20 cm are not a parked
                    # 5 cm cube — a phantom, not the block.
                    _note("spread", spread=cluster_spread(rel_samples))
                    return None
        time.sleep(0.1)
    _note("window_starved", got=len(samples))
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
    global LAST_SCAN
    LAST_SCAN = {"call": "scan_for", "color": color,
                 "initial_age": node.block_age(color),
                 "fresh_ticks": 0, "confirms": []}
    node.stop_base()
    time.sleep(SETTLE_S)
    if _fresh(node, color):
        LAST_SCAN["fresh_ticks"] += 1
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
            LAST_SCAN["fresh_ticks"] += 1
            hit = _confirm(node, color)   # stops the base itself
            if hit is not None:
                return hit
            # Sighting didn't hold up — keep spinning out the revolution.
    node.stop_base()
    LAST_SCAN["final_age"] = node.block_age(color)
    node.get_logger().warning(f"scan_for failed: {LAST_SCAN}")
    return None


def confirm_here(node, color):
    """Stationary re-confirm from the current pose; refined {'x','y'} or None.

    Used by pick at its standoff: a sighting made during the search scan can
    carry ~0.2 m of oblique-angle projection error, which the blind creep
    would faithfully reproduce. Dead-ahead at ~0.6 m the error collapses.
    """
    global LAST_SCAN
    LAST_SCAN = {"call": "confirm_here", "color": color,
                 "initial_age": node.block_age(color), "confirms": []}
    hit = _confirm(node, color)
    if hit is None:
        node.get_logger().warning(f"confirm_here failed: {LAST_SCAN}")
    return hit
