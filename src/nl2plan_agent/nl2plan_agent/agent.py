"""NL2Plan agent orchestrator.

Drives a local LLM (via Ollama) through a multi-turn tool-use loop, dispatching
tool calls to the ToolDispatcher and feeding results back into the conversation
until the model produces a final text response, the step cap is hit, or the
wall-clock cap is hit.

Designed to be robust against the kinds of failures small local models make:
- Malformed JSON in tool-call arguments (repaired + retried)
- Hallucinated tool names (returned to model as a structured error)
- Infinite tool loops (hard step cap)
"""

from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, List, Optional

from .prompt import SYSTEM_PROMPT
from .tool_schemas import ALL_TOOLS
from .tools import ToolDispatcher


@dataclass
class AgentConfig:
    model: str = "qwen2.5:7b-instruct"
    max_steps: int = 16
    max_wall_clock_s: float = 600.0
    temperature: float = 0.2


@dataclass
class AgentResult:
    final_message: str
    steps_taken: int
    duration_s: float
    stopped_reason: str  # "completed" | "step_cap" | "time_cap" | "error"
    messages: List[dict] = field(default_factory=list)


# Ollama client is imported lazily so this module is importable without Ollama installed.
ChatFn = Callable[..., dict]


def _default_chat() -> ChatFn:  # pragma: no cover - thin wrapper around external client
    import ollama
    return ollama.chat


class Agent:
    def __init__(
        self,
        dispatcher: ToolDispatcher,
        config: Optional[AgentConfig] = None,
        chat_fn: Optional[ChatFn] = None,
        trace_path: Optional[Path] = None,
    ) -> None:
        self.dispatcher = dispatcher
        self.config = config or AgentConfig()
        self._chat = chat_fn or _default_chat()
        self.trace_path = trace_path

    def run(self, user_command: str) -> AgentResult:
        messages: List[dict] = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_command},
        ]
        self._log({"kind": "user", "content": user_command})

        start = time.time()
        for step in range(self.config.max_steps):
            if time.time() - start > self.config.max_wall_clock_s:
                return self._finalize(messages, step, start, "time_cap",
                                      "Time cap reached before command finished.")

            try:
                response = self._chat(
                    model=self.config.model,
                    messages=messages,
                    tools=ALL_TOOLS,
                    options={"temperature": self.config.temperature},
                    keep_alive="30m",
                )
            except Exception as exc:
                return self._finalize(messages, step, start, "error", f"Chat call failed: {exc!r}")

            assistant_msg = response["message"]
            if hasattr(assistant_msg, "model_dump"):
                assistant_msg = assistant_msg.model_dump()
            messages.append(assistant_msg)
            self._log({"kind": "assistant", "content": assistant_msg.get("content", ""),
                       "tool_calls": assistant_msg.get("tool_calls", [])})

            tool_calls = assistant_msg.get("tool_calls") or []
            if not tool_calls:
                final = assistant_msg.get("content", "") or "(no response)"
                return self._finalize(messages, step + 1, start, "completed", final)

            for tc in tool_calls:
                fn = tc.get("function", {})
                name = fn.get("name", "")
                raw_args = fn.get("arguments", {})
                result = self.dispatcher.call(name, raw_args)
                messages.append({
                    "role": "tool",
                    "name": name,
                    "content": json.dumps(result.output),
                })

        return self._finalize(messages, self.config.max_steps, start, "step_cap",
                              "Step cap reached before command finished.")

    def _finalize(self, messages, steps, start, reason, final_text) -> AgentResult:
        duration = time.time() - start
        self._log({"kind": "result", "stopped_reason": reason,
                   "steps": steps, "duration_s": duration, "final": final_text})
        return AgentResult(
            final_message=final_text,
            steps_taken=steps,
            duration_s=duration,
            stopped_reason=reason,
            messages=messages,
        )

    def _log(self, entry: dict) -> None:
        if self.trace_path is None:
            return
        entry = {"ts": time.time(), "id": uuid.uuid4().hex[:8], **entry}
        self.trace_path.parent.mkdir(parents=True, exist_ok=True)
        with self.trace_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(entry) + "\n")


def main() -> None:  # pragma: no cover  - thin CLI wrapper
    """Tiny CLI: `python -m nl2plan_agent.agent "your command"`."""
    import argparse
    from .tools import MockBackend

    parser = argparse.ArgumentParser()
    parser.add_argument("command", help="Natural-language command for the robot.")
    parser.add_argument("--mock", action="store_true", help="Use MockBackend instead of ROS2.")
    parser.add_argument("--model", default="qwen2.5:7b-instruct")
    parser.add_argument("--trace", default="logs/agent_trace.jsonl")
    args = parser.parse_args()

    trace_path = Path(args.trace)
    if args.mock:
        backend = MockBackend()
    else:
        from .tools import Ros2Backend
        backend = Ros2Backend()
    dispatcher = ToolDispatcher(backend, trace_path=trace_path)
    agent = Agent(dispatcher, AgentConfig(model=args.model), trace_path=trace_path)
    result = agent.run(args.command)
    print(f"\n=== Result ({result.stopped_reason}, {result.steps_taken} steps, "
          f"{result.duration_s:.1f}s) ===")
    print(result.final_message)


if __name__ == "__main__":  # pragma: no cover
    main()
