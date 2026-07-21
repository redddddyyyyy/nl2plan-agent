"""Tool implementations.

Two backends:
  * Ros2Backend  - production. Calls ROS2 services. Requires rclpy (Linux only).
  * MockBackend  - testing + demo dry runs. Pure Python, no ROS2.

The dispatch layer (`ToolDispatcher`) does schema validation, JSON repair on
malformed input, and structured trace logging to logs/agent_trace.jsonl.
"""

from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Protocol

from jsonschema import Draft202012Validator, ValidationError

try:
    from json_repair import repair_json  # type: ignore
except ImportError:  # pragma: no cover
    def repair_json(s: str) -> str:
        return s

from .ros_backend.logic import parse_color
from .tool_schemas import TOOLS_BY_NAME


class ToolBackend(Protocol):
    def navigate_to(self, target_name: Optional[str], pose: Optional[dict]) -> dict: ...
    def find_object(self, description: str) -> dict: ...
    def pick(self, object_id: str) -> dict: ...
    def place(self, pose: Optional[dict]) -> dict: ...


@dataclass
class MockWorld:
    """A toy world mirroring the live aws-small-house scene."""

    robot_pose: dict = field(default_factory=lambda: {"x": 4.5, "y": -1.5, "theta": 3.1416})
    holding: Optional[str] = None
    named_poses: dict = field(default_factory=lambda: {
        "home": {"x": 4.3, "y": -1.5, "theta": 3.1416},
        "gym": {"x": -5.77, "y": -3.17, "theta": -2.36},
        "bedroom": {"x": -6.9, "y": -0.1, "theta": 3.1416},
        "bedroom_window": {"x": -4.5, "y": 1.4, "theta": 1.5708},
        "sofa": {"x": -1.8, "y": -1.9, "theta": 0.0},
        "lounge": {"x": 1.8, "y": 4.2, "theta": 0.0},
        "table": {"x": 4.0, "y": -1.75, "theta": -1.5708},
    })
    objects: dict = field(default_factory=lambda: {
        "red block": {"x": -7.6, "y": -0.1, "theta": 0.0},
        "orange block": {"x": 2.6, "y": 4.2, "theta": 0.0},
        "magenta block": {"x": -6.3, "y": -3.7, "theta": 0.0},
        "brown block": {"x": -1.1, "y": -1.9, "theta": 0.0},
    })
    blocked_paths: set = field(default_factory=set)


class MockBackend:
    """In-memory backend. Useful for testing the agent without ROS2."""

    def __init__(self, world: Optional[MockWorld] = None) -> None:
        self.world = world or MockWorld()

    def navigate_to(self, target_name: Optional[str], pose: Optional[dict]) -> dict:
        if target_name is None and pose is None:
            return {"success": False, "error": "Either target_name or pose must be provided."}
        if target_name is not None:
            if target_name in self.world.blocked_paths:
                return {"success": False, "error": f"Path to '{target_name}' is blocked."}
            resolved = self.world.named_poses.get(target_name)
            if resolved is None:
                return {"success": False, "error": f"Unknown named pose '{target_name}'."}
        else:
            resolved = pose
        self.world.robot_pose = dict(resolved)
        return {"success": True, "final_pose": dict(resolved), "duration_s": 1.0}

    def find_object(self, description: str) -> dict:
        color = parse_color(description)
        if color is None:
            return {"found": False,
                    "error": f"Can't recognize a color in '{description}'. "
                             "I can find blocks in: red, orange, magenta, brown."}
        match = self.world.objects.get(f"{color} block")
        rp = self.world.robot_pose
        dist = None
        if match is not None:
            dist = ((match["x"] - rp["x"]) ** 2 + (match["y"] - rp["y"]) ** 2) ** 0.5
        if match is not None:
            self._last_found_key = f"{color} block"
        if match is None or dist > 2.0:
            # Same wording as the live backend: steer the model to another
            # room, never into inching toward an out-of-range block.
            return {"found": False,
                    "error": f"No {color} block visible from here; "
                             "try navigating elsewhere."}
        return {
            "found": True,
            "object_id": f"obj_{abs(hash(description)) % 10_000:04d}",
            "pose": dict(match),
            "confidence": 0.93,
        }

    def pick(self, object_id: str) -> dict:
        if self.world.holding is not None:
            return {"success": False, "error": f"Already holding {self.world.holding}."}
        if not object_id.startswith("obj_"):
            return {"success": False, "error": f"Invalid object_id '{object_id}'."}
        self.world.holding = object_id
        self._held_key = getattr(self, "_last_found_key", None)
        return {"success": True}

    def place(self, pose: Optional[dict]) -> dict:
        if self.world.holding is None:
            return {"success": False, "error": "Nothing to place; not holding anything."}
        # The block lands where the robot stands - the mock table is only
        # ever as honest as this line.
        key = getattr(self, "_held_key", None)
        if key in self.world.objects:
            self.world.objects[key] = dict(self.world.robot_pose)
        self.world.holding = None
        self._held_key = None
        return {"success": True}


class Ros2Backend:
    """Real backend targeting the mobile_arm_sim robot + Nav2.

    Thin delegate: the actual node, Nav2 client, scan, and arm sequences
    live in the ros_backend package. Imports are deferred so this module
    stays importable (and the test suite green) without rclpy.
    """

    def __init__(self) -> None:
        try:
            from .ros_backend.backend import RosBackend
        except ImportError as e:
            raise RuntimeError(
                "Ros2Backend requires rclpy. Run on Linux with ROS2 Humble sourced."
            ) from e
        self._impl = RosBackend()

    def navigate_to(self, target_name: Optional[str], pose: Optional[dict]) -> dict:
        return self._impl.navigate_to(target_name, pose)

    def find_object(self, description: str) -> dict:
        return self._impl.find_object(description)

    def pick(self, object_id: str) -> dict:
        return self._impl.pick(object_id)

    def place(self, pose: Optional[dict]) -> dict:
        return self._impl.place(pose)


@dataclass
class ToolCallResult:
    name: str
    args: dict
    output: dict
    duration_s: float
    valid_args: bool


class ToolDispatcher:
    """Validates LLM-supplied tool calls, executes them, logs every call."""

    def __init__(
        self,
        backend: ToolBackend,
        trace_path: Optional[Path] = None,
    ) -> None:
        self.backend = backend
        self.trace_path = trace_path
        self._validators = {
            name: Draft202012Validator(spec["function"]["parameters"])
            for name, spec in TOOLS_BY_NAME.items()
        }
        self._fns: Dict[str, Callable[[dict], dict]] = {
            "navigate_to": lambda a: backend.navigate_to(a.get("target_name"), a.get("pose")),
            "find_object": lambda a: backend.find_object(a["description"]),
            "pick":        lambda a: backend.pick(a["object_id"]),
            "place":       lambda a: backend.place(a.get("pose")),
        }

    def call(self, name: str, raw_args: Any) -> ToolCallResult:
        args, valid = self._coerce_args(name, raw_args)
        start = time.time()
        if name not in self._fns:
            output = {"success": False, "error": f"Unknown tool '{name}'."}
        elif not valid:
            output = {"success": False, "error": "Arguments failed schema validation."}
        else:
            try:
                output = self._fns[name](args)
            except Exception as exc:  # pragma: no cover  (defensive; real backend may raise)
                output = {"success": False, "error": f"Tool raised: {exc!r}"}
        duration = time.time() - start
        result = ToolCallResult(name=name, args=args, output=output, duration_s=duration, valid_args=valid)
        self._log(result)
        return result

    def _coerce_args(self, name: str, raw: Any) -> tuple[dict, bool]:
        if isinstance(raw, str):
            try:
                args = json.loads(raw)
            except json.JSONDecodeError:
                try:
                    args = json.loads(repair_json(raw))
                except Exception:
                    return {}, False
        elif isinstance(raw, dict):
            args = raw
        else:
            return {}, False
        validator = self._validators.get(name)
        if validator is None:
            return args, False
        try:
            validator.validate(args)
            return args, True
        except ValidationError:
            return args, False

    def _log(self, result: ToolCallResult) -> None:
        if self.trace_path is None:
            return
        entry = {
            "ts": time.time(),
            "id": uuid.uuid4().hex[:8],
            "kind": "tool_call",
            "tool": result.name,
            "args": result.args,
            "output": result.output,
            "duration_s": result.duration_s,
            "valid_args": result.valid_args,
        }
        self.trace_path.parent.mkdir(parents=True, exist_ok=True)
        with self.trace_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(entry) + "\n")
