"""Decision logic for the ROS2 backend that needs no ROS to test.

Color parsing, sighting-cluster math, and Nav2 status mapping live here as
plain functions so pytest covers them without a sim.
"""

from __future__ import annotations

import math
import re
from typing import Optional, Sequence

# Color -> Gazebo entity name. Red is the original mission target; the other
# three were spawned as distractors, hence the different naming scheme.
COLOR_ENTITIES = {
    "red": "target_block",
    "orange": "distractor_orange",
    "magenta": "distractor_magenta",
    "brown": "distractor_brown",
}

KNOWN_COLORS = tuple(COLOR_ENTITIES)


def parse_color(description: str) -> Optional[str]:
    """Pull a known block color out of free text, or None.

    Whole-word match, case-insensitive: "the MAGENTA block" -> "magenta",
    but "browning pan" is not brown.
    """
    words = re.findall(r"[a-z]+", description.lower())
    for color in KNOWN_COLORS:
        if color in words:
            return color
    return None


def cluster_spread(samples: Sequence[tuple[float, float]]) -> float:
    """Diagonal of the bounding box around (x, y) samples, in meters."""
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def mean_xy(samples: Sequence[tuple[float, float]]) -> tuple[float, float]:
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    return sum(xs) / len(xs), sum(ys) / len(ys)


# action_msgs/GoalStatus terminal codes; imported nowhere so this module
# stays rclpy-free.
_STATUS_NAMES = {4: "succeeded", 5: "canceled", 6: "aborted"}


def nav_result(status: int, duration_s: float, final_pose: dict) -> dict:
    """Map a Nav2 terminal status to the navigate_to tool result."""
    if status == 4:
        return {
            "success": True,
            "final_pose": final_pose,
            "duration_s": round(duration_s, 1),
        }
    name = _STATUS_NAMES.get(status, f"status {status}")
    return {"success": False,
            "error": f"Navigation {name} before reaching the goal."}


def standoff_pose(rx: float, ry: float, tx: float, ty: float,
                  standoff: float) -> tuple[float, float, float]:
    """Nav goal `standoff` meters from target (tx, ty), approached from the
    robot's side, facing the target. Approach legs belong to Nav2 — a blind
    creep from the search pose rammed a stool that sat between the robot
    and the block."""
    ang = math.atan2(ry - ty, rx - tx)
    gx = tx + standoff * math.cos(ang)
    gy = ty + standoff * math.sin(ang)
    return gx, gy, math.atan2(ty - gy, tx - gx)
