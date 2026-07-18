"""JSON schemas for the four tools the LLM can call.

Format matches Ollama's tool-calling API (OpenAI-compatible function schemas).
The same schemas are used to (a) advertise tools to the model and (b) validate
the model's tool-call arguments before execution.
"""

from __future__ import annotations

POSE_SCHEMA = {
    "type": "object",
    "properties": {
        "x": {"type": "number", "description": "World-frame x in meters"},
        "y": {"type": "number", "description": "World-frame y in meters"},
        "theta": {"type": "number", "description": "Heading in radians"},
    },
    "required": ["x", "y", "theta"],
    "additionalProperties": False,
}

NAVIGATE_TO = {
    "type": "function",
    "function": {
        "name": "navigate_to",
        "description": (
            "Drive the robot base to a named location or absolute pose. "
            "Use a named target ('home', 'gym', 'bedroom', 'bedroom_window', 'lounge', 'table') when one is appropriate; "
            "otherwise provide an explicit pose. Returns success and the final pose."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "target_name": {
                    "type": "string",
                    "description": (
                        "Named location from the world map. "
                        "One of: home, gym, bedroom, bedroom_window, lounge, table. Mutually exclusive with pose."
                    ),
                },
                "pose": {
                    **POSE_SCHEMA,
                    "description": "Explicit goal pose. Mutually exclusive with target_name.",
                },
            },
            "additionalProperties": False,
        },
    },
}

FIND_OBJECT = {
    "type": "function",
    "function": {
        "name": "find_object",
        "description": (
            "Look for an object matching a natural-language description using "
            "the robot's camera. Returns whether the object was found and, "
            "if so, a stable object_id plus its world-frame pose. "
            "Call this before pick()."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "description": {
                    "type": "string",
                    "description": (
                        "Natural-language description, e.g. 'red block', 'magenta block'. "
                        "Name the block color — one of: red, orange, magenta, brown."
                    ),
                },
            },
            "required": ["description"],
            "additionalProperties": False,
        },
    },
}

PICK = {
    "type": "function",
    "function": {
        "name": "pick",
        "description": (
            "Grasp an object that has already been located by find_object. "
            "Performs a top-down grasp. Requires the robot to be near the object."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "object_id": {
                    "type": "string",
                    "description": "object_id returned by a previous find_object call.",
                },
            },
            "required": ["object_id"],
            "additionalProperties": False,
        },
    },
}

PLACE = {
    "type": "function",
    "function": {
        "name": "place",
        "description": (
            "Releases the held object onto the drop table, driving the final approach itself. "
            "Navigate to 'table' first."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "pose": POSE_SCHEMA,
            },
            "additionalProperties": False,
        },
    },
}

ALL_TOOLS = [NAVIGATE_TO, FIND_OBJECT, PICK, PLACE]

TOOLS_BY_NAME = {t["function"]["name"]: t for t in ALL_TOOLS}
