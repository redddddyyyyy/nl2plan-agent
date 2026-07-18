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
