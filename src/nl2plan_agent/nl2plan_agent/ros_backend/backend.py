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
from .logic import COLOR_ENTITIES, parse_color
from .node import get_node

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
                    "error": f"No confirmed object '{object_id}' in reach; "
                             "run find_object first."}
        node = self._node
        if node.robot is not None:
            dist = ((node.robot[0] - det["x"]) ** 2 +
                    (node.robot[1] - det["y"]) ** 2) ** 0.5
            if dist > manipulation.PICK_RANGE:
                self._last_detection = None
                return {"success": False,
                        "error": "The robot has moved away since that sighting; "
                                 "run find_object again from here."}
        err = manipulation.align(node, det["x"], det["y"],
                                 manipulation.GRASP_REACH)
        if err is not None:
            return {"success": False, "error": err}
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
