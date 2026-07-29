"""RosBackend: the four blocking tool methods behind Ros2Backend.

State that spans tool calls lives here: which entity is being held and the
last confirmed detection that pick acts on.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

from ..named_poses import load_named_poses
from . import manipulation, nav, perception
from .logic import (COLOR_ENTITIES, from_robot_frame, parse_color, pick_error,
                    standoff_pose)
from .node import get_node

REFINE_STANDOFF = 0.6   # m: outside the 0.30 m blind zone, prime viewing

_DEFAULT_POSES = Path(__file__).resolve().parents[4] / "config" / "named_poses.yaml"


class RosBackend:

    def __init__(self):
        self._node = get_node()
        self._nav_checked = False
        self._holding: Optional[str] = None      # pinned Gazebo entity name
        self._last_detection: Optional[dict] = None
        poses_file = os.environ.get("NL2PLAN_POSES_FILE", str(_DEFAULT_POSES))
        self._poses = load_named_poses(poses_file)

    # ---------- tools ----------

    def navigate_to(self, target_name: Optional[str], pose: Optional[dict]) -> dict:
        if target_name is None and pose is None:
            return {"success": False,
                    "error": "Either target_name or pose must be provided."}
        if target_name is not None:
            resolved = self._poses.get(target_name)
            if resolved is None:
                known = ", ".join(sorted(self._poses))
                return {"success": False,
                        "error": f"Unknown named pose '{target_name}'. Known poses: {known}."}
        else:
            resolved = pose

        if not self._nav_checked:
            err = nav.wait_nav_active(self._node)
            if err is not None:
                return {"success": False, "error": err}
            self._nav_checked = True

        return nav.navigate(self._node,
                            float(resolved["x"]), float(resolved["y"]),
                            float(resolved.get("theta", 0.0)))

    def find_object(self, description: str) -> dict:
        color = parse_color(description)
        if color is None:
            known = ", ".join(COLOR_ENTITIES)
            return {"found": False,
                    "error": f"Can't recognize a color in '{description}'. "
                             f"I can find blocks in: {known}."}
        hit = perception.scan_for(self._node, color)
        if hit is None:
            return {"found": False,
                    "error": f"No {color} block visible from here; "
                             "try navigating elsewhere."}
        self._last_detection = {
            "color": color,
            "entity": COLOR_ENTITIES[color],
            "object_id": f"{color}_block",
            "x": hit["x"],
            "y": hit["y"],
        }
        return {
            "found": True,
            "object_id": f"{color}_block",
            "pose": {"x": hit["x"], "y": hit["y"], "theta": 0.0},
            "confidence": 0.9,
        }

    def pick(self, object_id: str) -> dict:
        if self._holding is not None:
            return {"success": False,
                    "error": f"Already holding {self._holding}. Place it first."}
        det = self._last_detection
        if det is None or det["object_id"] != object_id:
            return {"success": False,
                    "error": pick_error(object_id,
                                        det["object_id"] if det else None)}
        node = self._node
        if node.robot is not None:
            dist = ((node.robot[0] - det["x"]) ** 2 +
                    (node.robot[1] - det["y"]) ** 2) ** 0.5
            if dist > manipulation.PICK_RANGE:
                self._last_detection = None
                return {"success": False,
                        "error": "The robot has moved away since that sighting; "
                                 "run find_object again from here."}
        # Approach in three stages: Nav2 drives to a standoff (it plans
        # around furniture — a blind creep from the search pose rammed a
        # stool), the block is re-confirmed dead-ahead (search-scan
        # sightings carry ~0.2 m of oblique-angle error), and only the
        # last fraction of a meter is creeped on odometry.
        if node.robot is None:
            return {"success": False,
                    "error": "No localization yet. Is the sim running?"}
        rx, ry, _ = node.robot
        far = ((rx - det["x"]) ** 2 + (ry - det["y"]) ** 2) ** 0.5
        if far > REFINE_STANDOFF + 0.15:
            gx, gy, yaw = standoff_pose(rx, ry, det["x"], det["y"],
                                        REFINE_STANDOFF)
            r = nav.navigate(node, gx, gy, yaw)
            if not r.get("success"):
                return {"success": False,
                        "error": f"Could not approach the {det['color']} "
                                 f"block: {r['error']}"}
        else:
            err = manipulation.align(node, det["x"], det["y"], far)
            if err is not None:
                return {"success": False, "error": err}
        fresh = perception.confirm_here(node, det["color"])
        if fresh is None:
            # The standoff is prime viewing: 0.6 m out, dead ahead. A block
            # that can't be re-confirmed from here was a bad sighting (0.55 m
            # off truth, measured at the lounge 2026-07-21) or is gone —
            # grasping on the stale coordinate teleports it across visible
            # floor. A refused pick costs one find_object; a teleport costs
            # the demo take.
            self._last_detection = None
            return {"success": False,
                    "error": f"Lost the {det['color']} block on final "
                             "approach; run find_object again from here."}
        det = {**det, "x": fresh["x"], "y": fresh["y"]}
        self._last_detection = det
        # Drive the final stretch on the offset measured off the robot's own
        # nose, re-anchored to where the robot believes it is NOW. The map
        # coordinate is derived from the same sighting, but replaying it
        # after the estimate shifts (24 deg of AMCL yaw, measured at
        # bedroom_window) aims the creep at floor beside the block and lets
        # the magic grasp fake the pick from there.
        gx, gy = det["x"], det["y"]
        if fresh.get("rel") is not None and node.robot is not None:
            gx, gy = from_robot_frame(fresh["rel"][0], fresh["rel"][1],
                                      node.robot)
        # stall_is_error: a creep that presses into an obstacle short of
        # reach must refuse, not grasp — the pin would teleport the block
        # across the gap. The shortfall is odometry-measured inside align;
        # re-measuring it here against node.robot double-counts whatever
        # AMCL corrected mid-creep and refuses picks whose robot is
        # standing at the block (live 2026-07-21, the back-off/stare loop).
        err = manipulation.align(node, gx, gy, manipulation.GRASP_REACH,
                                 stall_is_error=True)
        if err is not None:
            # Back off before refusing: stalled ~0.5 m out, the block sits
            # in the camera's near blind zone and the re-scan this error
            # asks for would fail from here.
            manipulation.back_away(node)
            self._last_detection = None
            return {"success": False,
                    "error": err + " Backed off for a clear view; "
                             "run find_object again from here."}
        manipulation.grasp_sequence(node, det["entity"])
        self._holding = det["entity"]
        return {"success": True}

    def place(self, pose: Optional[dict]) -> dict:
        if self._holding is None:
            return {"success": False,
                    "error": "Nothing to place; not holding anything."}
        node = self._node
        m = manipulation
        tx, ty = m.TABLE_XY
        if pose is not None:
            tx, ty = float(pose["x"]), float(pose["y"])
        # The LLM handles coarse travel (navigate_to the table standoff);
        # this drives the last stretch by touch — the stall guard docking
        # against the table IS the arrival signal.
        err = m.align(node, tx, ty, m.TABLE_REACH)
        if err is not None:
            return {"success": False,
                    "error": err + " Still holding the block."}
        entity = self._holding
        m.run_stages(node, [
            (lambda: node.arm(m.DROP), 3.0),
            (lambda: node.gripper(m.GRIPPER_OPEN), 0.4),
            (lambda: m.detach(node, entity), 1.5),
            (lambda: node.arm(m.LIFT), 2.0),
            (lambda: node.arm(m.REST), 2.0),
        ])
        m.back_away(node)
        self._holding = None
        self._last_detection = None
        return {"success": True}
