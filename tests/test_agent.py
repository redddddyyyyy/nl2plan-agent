"""Agent + dispatcher tests using a scripted fake chat function and MockBackend.

These run on Windows without ROS2. They exercise:
  - happy path: a 3-step plan completes
  - JSON repair: malformed tool args get repaired and validated
  - schema validation: bad args produce a structured tool error
  - tool failure recovery: a failing tool prompts a different next step
  - step cap: a runaway loop terminates with stopped_reason="step_cap"
"""

from __future__ import annotations

from typing import Iterator, List

import pytest

from nl2plan_agent.agent import Agent, AgentConfig
from nl2plan_agent.tools import MockBackend, MockWorld, ToolDispatcher


def make_fake_chat(scripted: List[dict]) -> "callable":
    """Build a chat function that returns the next scripted response each call.

    Each scripted entry is the value of the `message` key of a normal Ollama response.
    """
    it: Iterator[dict] = iter(scripted)

    def chat(**kwargs):
        try:
            msg = next(it)
        except StopIteration:
            return {"message": {"role": "assistant", "content": "(out of scripted responses)"}}
        return {"message": msg}

    return chat


# ---------- happy path ----------

def test_happy_path_full_pick_and_place():
    scripted = [
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "navigate_to",
                                       "arguments": {"target_name": "bedroom"}}}]},
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "find_object",
                                       "arguments": {"description": "red block"}}}]},
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "pick",
                                       "arguments": {"object_id": "obj_0001"}}}]},
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "navigate_to",
                                       "arguments": {"target_name": "table"}}}]},
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "place",
                                       "arguments": {}}}]},
        {"role": "assistant", "content": "Put the red block on the table."},
    ]
    dispatcher = ToolDispatcher(MockBackend())
    agent = Agent(dispatcher, AgentConfig(model="test"), chat_fn=make_fake_chat(scripted))
    result = agent.run("pick up the red block and put it on the table")

    assert result.stopped_reason == "completed"
    assert "red block" in result.final_message.lower()
    assert result.steps_taken == 6


# ---------- JSON repair ----------

def test_malformed_json_args_are_repaired():
    # Single tool call with broken JSON (missing closing brace) as a string.
    scripted = [
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "navigate_to",
                                       "arguments": '{"target_name": "gym"'}}]},
        {"role": "assistant", "content": "Done."},
    ]
    dispatcher = ToolDispatcher(MockBackend())
    agent = Agent(dispatcher, AgentConfig(model="test"), chat_fn=make_fake_chat(scripted))
    result = agent.run("go to the gym")

    assert result.stopped_reason == "completed"
    # The tool call ran successfully despite malformed args.
    import json as _json
    tool_msg = [m for m in result.messages if m["role"] == "tool"][0]
    assert _json.loads(tool_msg["content"])["success"] is True


# ---------- schema validation rejects unknown args ----------

def test_invalid_args_produce_structured_error():
    scripted = [
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "navigate_to",
                                       "arguments": {"target_name": "hallway",
                                                     "rocket_fuel": True}}}]},
        {"role": "assistant", "content": "Reported error."},
    ]
    dispatcher = ToolDispatcher(MockBackend())
    agent = Agent(dispatcher, AgentConfig(model="test"), chat_fn=make_fake_chat(scripted))
    result = agent.run("go to the gym")

    tool_msg = [m for m in result.messages if m["role"] == "tool"][0]
    assert "schema" in tool_msg["content"].lower()


# ---------- tool failure recovery ----------

def test_tool_failure_then_recovery():
    world = MockWorld()
    world.blocked_paths.add("bedroom")

    scripted = [
        # First attempt: west_room path blocked
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "navigate_to",
                                       "arguments": {"target_name": "bedroom"}}}]},
        # Recovery: try the hallway instead
        {"role": "assistant", "content": "Bedroom blocked, trying the gym.",
         "tool_calls": [{"function": {"name": "navigate_to",
                                       "arguments": {"target_name": "gym"}}}]},
        {"role": "assistant", "content": "Arrived in the hallway."},
    ]
    dispatcher = ToolDispatcher(MockBackend(world))
    agent = Agent(dispatcher, AgentConfig(model="test"), chat_fn=make_fake_chat(scripted))
    result = agent.run("go to the bedroom")

    assert result.stopped_reason == "completed"
    import json as _json
    tool_msgs = [m for m in result.messages if m["role"] == "tool"]
    assert len(tool_msgs) == 2
    assert "blocked" in tool_msgs[0]["content"].lower()
    assert _json.loads(tool_msgs[1]["content"])["success"] is True


# ---------- step cap ----------

def test_step_cap_terminates_runaway_loop():
    looping_response = {
        "role": "assistant", "content": "",
        "tool_calls": [{"function": {"name": "find_object",
                                      "arguments": {"description": "nonexistent"}}}],
    }
    scripted = [looping_response] * 50  # way more than the step cap
    dispatcher = ToolDispatcher(MockBackend())
    agent = Agent(dispatcher,
                  AgentConfig(model="test", max_steps=5),
                  chat_fn=make_fake_chat(scripted))
    result = agent.run("find a thing that does not exist")

    assert result.stopped_reason == "step_cap"
    assert result.steps_taken == 5


# ---------- backend smoke ----------

def test_mock_backend_navigate_and_find_and_pick():
    backend = MockBackend()
    # find_object before driving close should fail (distance > 2.0)
    assert backend.find_object("red block")["found"] is False
    # An unknown color is a structured miss, not a crash
    assert backend.find_object("blue block")["found"] is False
    # Drive to the west room, then the red block is in range
    assert backend.navigate_to("bedroom", None)["success"] is True
    res = backend.find_object("the red block")
    assert res["found"] is True
    assert res["confidence"] > 0.5
    assert backend.pick(res["object_id"])["success"] is True
    assert backend.pick(res["object_id"])["success"] is False
    assert backend.place(None)["success"] is True


# ---------- empty-reply stall guard ----------

def test_empty_reply_mid_mission_gets_nudged():
    scripted = [
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "navigate_to",
                                       "arguments": {"target_name": "bedroom"}}}]},
        {"role": "assistant", "content": ""},   # mid-mission stall: no text, no tools
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "find_object",
                                       "arguments": {"description": "red block"}}}]},
        {"role": "assistant", "content": "Found it and stopped there."},
    ]
    dispatcher = ToolDispatcher(MockBackend())
    agent = Agent(dispatcher, AgentConfig(model="test"), chat_fn=make_fake_chat(scripted))
    result = agent.run("find the red block")

    assert result.stopped_reason == "completed"
    assert result.final_message == "Found it and stopped there."
    nudges = [m for m in result.messages
              if m["role"] == "user" and "continue" in m["content"].lower()]
    assert len(nudges) == 1


def test_intent_reply_without_tool_call_gets_nudged():
    # Live failure 2026-07-20: mid-search qwen replied "I will navigate to
    # the lounge to find the orange block." with no tool call, and the run
    # finalized "completed" while the model was still mid-plan.
    scripted = [
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "navigate_to",
                                       "arguments": {"target_name": "lounge"}}}]},
        {"role": "assistant",
         "content": "I will navigate to the bedroom to find the red block.\n\n"},
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "navigate_to",
                                       "arguments": {"target_name": "bedroom"}}}]},
        {"role": "assistant", "content": "",
         "tool_calls": [{"function": {"name": "find_object",
                                       "arguments": {"description": "red block"}}}]},
        {"role": "assistant", "content": "Found the red block in the bedroom."},
    ]
    dispatcher = ToolDispatcher(MockBackend())
    agent = Agent(dispatcher, AgentConfig(model="test"), chat_fn=make_fake_chat(scripted))
    result = agent.run("find the red block")

    assert result.stopped_reason == "completed"
    assert result.final_message == "Found the red block in the bedroom."
    nudges = [m for m in result.messages
              if m["role"] == "user" and "call the tool" in m["content"].lower()]
    assert len(nudges) == 1


def test_persistent_empty_replies_end_the_run():
    scripted = [{"role": "assistant", "content": ""}] * 5
    dispatcher = ToolDispatcher(MockBackend())
    agent = Agent(dispatcher, AgentConfig(model="test"), chat_fn=make_fake_chat(scripted))
    result = agent.run("do nothing")

    assert result.stopped_reason == "completed"
    assert result.final_message == "(no response)"


# ---------- session history threading (interactive mode) ----------

def test_history_carries_between_runs():
    dispatcher = ToolDispatcher(MockBackend())
    agent = Agent(dispatcher, AgentConfig(model="test"),
                  chat_fn=make_fake_chat([
                      {"role": "assistant", "content": "Orange is at the bedroom window."}]))
    r1 = agent.run("scout for the orange block")
    assert r1.stopped_reason == "completed"

    agent2 = Agent(dispatcher, AgentConfig(model="test"),
                   chat_fn=make_fake_chat([{"role": "assistant", "content": "On my way."}]))
    r2 = agent2.run("now fetch it", history=r1.messages)

    # exactly one system prompt, and the first exchange rides along
    assert sum(1 for m in r2.messages if m["role"] == "system") == 1
    assert any("scout for the orange block" in m.get("content", "")
               for m in r2.messages if m["role"] == "user")
    assert any("bedroom window" in m.get("content", "")
               for m in r2.messages if m["role"] == "assistant")
    assert r2.final_message == "On my way."


def test_mock_place_moves_the_block():
    backend = MockBackend()
    assert backend.navigate_to("bedroom", None)["success"] is True
    res = backend.find_object("red block")
    assert res["found"] is True
    assert backend.pick(res["object_id"])["success"] is True
    assert backend.navigate_to("table", None)["success"] is True
    assert backend.place(None)["success"] is True
    # the block is now findable at the table, not at its old spot
    res = backend.find_object("red block")
    assert res["found"] is True
    table = backend.world.named_poses["table"]
    assert res["pose"]["x"] == table["x"] and res["pose"]["y"] == table["y"]


# ---------- block locations are in the prompt, not guesswork ----------

def test_prompt_names_the_room_for_every_block():
    """The model must not have to hunt room by room.

    Without this the search order is pure guesswork: a 2026-07-20 run spent
    all 16 steps sweeping rooms for the orange block and hit the step cap
    without ever reaching pick, while find_object at the right pose works
    first try. Rooms below come from perception_node/scene_setup.py.
    """
    from nl2plan_agent.prompt import SYSTEM_PROMPT

    colors = ("brown", "orange", "red", "magenta")
    lowered = SYSTEM_PROMPT.lower()
    for color, room in (("orange", "lounge"), ("brown", "sofa"),
                        ("red", "bedroom"), ("magenta", "gym")):
        others = [c for c in colors if c != color]
        # The line must tie THIS color to its room - a line naming every
        # colour and every room (as the old prompt did) teaches nothing.
        line = next((l for l in lowered.splitlines()
                     if color in l and room in l
                     and not any(o in l for o in others)), None)
        assert line is not None, f"prompt never puts {color} in {room} on its own"


def test_prompt_gives_a_fixed_sweep_order():
    """Search order is brown -> orange -> red -> magenta when the room is
    unknown, so a miss never turns into a random walk."""
    from nl2plan_agent.prompt import SYSTEM_PROMPT

    lowered = SYSTEM_PROMPT.lower()
    order = ["lounge", "sofa", "bedroom", "gym"]

    def lists_in_order(line):
        cursor = 0
        for room in order:
            idx = line.find(room, cursor)
            if idx < 0:
                return False
            cursor = idx + len(room)
        return True

    # Some other line (e.g. the location inventory) may name the same rooms
    # in another order; at least one line must give them as a sweep.
    assert any(lists_in_order(l) for l in lowered.splitlines()), \
        "no line lists the vantage points in sweep order"
