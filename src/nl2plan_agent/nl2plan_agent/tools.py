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

from .tool_schemas import TOOLS_BY_NAME


class ToolBackend(Protocol):
    def navigate_to(self, target_name: Optional[str], pose: Optional[dict]) -> dict: ...
    def find_object(self, description: str) -> dict: ...
    def pick(self, object_id: str) -> dict: ...
    def place(self, pose: Optional[dict]) -> dict: ...


@dataclass
class MockWorld:
    """A toy world used by MockBackend for tests and offline dry runs."""

    robot_pose: dict = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "theta": 0.0})
    holding: Optional[str] = None
    named_poses: dict = field(default_factory=lambda: {
        "start": {"x": 0.0, "y": 0.0, "theta": 0.0},
        "kitchen": {"x": 5.0, "y": 2.0, "theta": 0.0},
        "table": {"x": 5.5, "y": 2.5, "theta": 0.0},
    })
    objects: dict = field(default_factory=lambda: {
        "red mug": {"x": 5.5, "y": 2.7, "theta": 0.0},
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
        match = self.world.objects.get(description.lower().strip())
        if match is None:
            return {"found": False, "error": f"No object matching '{description}' visible."}
        rp = self.world.robot_pose
        dist = ((match["x"] - rp["x"]) ** 2 + (match["y"] - rp["y"]) ** 2) ** 0.5
        if dist > 2.0:
            return {"found": False, "error": "Object too far; drive closer first."}
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
        return {"success": True}

    def place(self, pose: Optional[dict]) -> dict:
        if self.world.holding is None:
            return {"success": False, "error": "Nothing to place; not holding anything."}
        self.world.holding = None
        return {"success": True}


class Ros2Backend:
    """Real backend targeting the mobile_arm_sim robot + Nav2.

    Wiring targets (filled in during Phase 2):
      * navigate_to -> Nav2 NavigateToPose action client on /navigate_to_pose
      * find_object -> subscribe to /target_block_pose (block_detector.py output);
        upgrade to GroundingDINO publishing the same topic later
      * pick / place -> JointGroupPositionController commands on /arm_controller
        and /gripper_controller, mirroring mobile_arm_sim/scripts/pick_and_place.py

    Fails fast off-Linux so tests stay deterministic.
    """

    def __init__(self) -> None:
        try:
            import rclpy  # noqa: F401  (lazy import; raises if not installed)
        except ImportError as e:
            raise RuntimeError(
                "Ros2Backend requires rclpy. Run on Linux with ROS2 Humble sourced."
            ) from e

    def navigate_to(self, target_name: Optional[str], pose: Optional[dict]) -> dict:
        raise NotImplementedError("Wire to Nav2 NavigateToPose action on /navigate_to_pose.")

    def find_object(self, description: str) -> dict:
        raise NotImplementedError("Wire to /target_block_pose from block_detector (later GroundingDINO).")

    def pick(self, object_id: str) -> dict:
        raise NotImplementedError("Wire to arm_controller + gripper_controller (see pick_and_place.py).")

    def place(self, pose: Optional[dict]) -> dict:
        raise NotImplementedError("Wire to arm_controller + gripper_controller (see pick_and_place.py).")


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
