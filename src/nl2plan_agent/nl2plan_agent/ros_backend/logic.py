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


def pick_error(object_id: str, available: Optional[str]) -> str:
    """Why a pick was refused, naming the id that would have worked.

    A small model that fumbles the id (a placeholder, or the colour alone)
    can only recover if the error tells it what to say instead; without
    that it re-runs find_object and re-fumbles until the step cap.
    """
    if available:
        return (f"No confirmed object '{object_id}' in reach. "
                f"The object you just found is '{available}' — "
                f"call pick with that exact object_id.")
    return (f"No confirmed object '{object_id}' in reach; "
            "run find_object first.")


def to_robot_frame(x: float, y: float,
                   robot: tuple[float, float, float]) -> tuple[float, float]:
    """Map-frame (x, y) expressed relative to a robot at (rx, ry, yaw).

    Detections are back-projected through the robot's believed pose, so a
    localization error lands in BOTH the robot pose and the projection.
    Subtracting one from the other cancels it, which is what makes a
    stationary block look stationary even while AMCL's heading wanders.
    """
    rx, ry, yaw = robot
    dx, dy = x - rx, y - ry
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    return (dx * cos_y - dy * sin_y, dx * sin_y + dy * cos_y)


def from_robot_frame(rel_x: float, rel_y: float,
                     robot: tuple[float, float, float]) -> tuple[float, float]:
    """Inverse of to_robot_frame: a robot-relative offset back into the map.

    Re-anchoring a sighting through the robot's CURRENT pose is what keeps
    the final approach honest — the offset was measured against the robot,
    so it stays true even if the map estimate has shifted underneath it.
    """
    rx, ry, yaw = robot
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return (rx + rel_x * cos_y - rel_y * sin_y,
            ry + rel_x * sin_y + rel_y * cos_y)


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
