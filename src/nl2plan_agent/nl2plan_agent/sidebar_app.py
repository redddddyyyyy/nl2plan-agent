"""Streamlit sidebar that visualizes the agent's reasoning live.

Reads logs/agent_trace.jsonl (append-only JSONL written by Agent + ToolDispatcher)
and renders three columns:
  1. The user command and the model's final response.
  2. The step-by-step trace: assistant messages and tool calls.
  3. The latest Gazebo camera frame, if present at logs/latest_frame.png.

Run with:
    streamlit run src/nl2plan_agent/nl2plan_agent/sidebar_app.py
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import streamlit as st

TRACE_PATH = Path("logs/agent_trace.jsonl")
FRAME_PATH = Path("logs/latest_frame.png")


def load_trace() -> list[dict]:
    if not TRACE_PATH.exists():
        return []
    entries: list[dict] = []
    with TRACE_PATH.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return entries


def render_step(entry: dict) -> None:
    kind = entry.get("kind", "?")
    ts = entry.get("ts", 0.0)
    when = time.strftime("%H:%M:%S", time.localtime(ts))
    if kind == "user":
        st.markdown(f"**[{when}] user**")
        st.code(entry.get("content", ""), language="text")
    elif kind == "assistant":
        st.markdown(f"**[{when}] assistant**")
        content = entry.get("content", "")
        if content:
            st.write(content)
        for tc in entry.get("tool_calls") or []:
            fn = tc.get("function", {})
            st.markdown(f"  -> `{fn.get('name', '?')}({fn.get('arguments', {})})`")
    elif kind == "tool_call":
        st.markdown(f"**[{when}] tool: `{entry.get('tool')}`**")
        st.json({"args": entry.get("args"), "output": entry.get("output"),
                 "duration_s": round(entry.get("duration_s", 0.0), 3),
                 "valid_args": entry.get("valid_args")})
    elif kind == "result":
        st.success(
            f"**[{when}] done** - {entry.get('stopped_reason')} "
            f"in {entry.get('steps')} steps, {entry.get('duration_s', 0):.1f}s"
        )
        st.write(entry.get("final", ""))


def main() -> None:
    st.set_page_config(page_title="NL2Plan agent", layout="wide")
    st.title("NL2Plan agent - live trace")

    col_trace, col_sim = st.columns([3, 2])

    with col_sim:
        st.subheader("Simulator view")
        if FRAME_PATH.exists():
            st.image(str(FRAME_PATH))
        else:
            st.info(f"Waiting for a frame at {FRAME_PATH}.")

    with col_trace:
        st.subheader("Agent trace")
        entries = load_trace()
        if not entries:
            st.info(f"No trace yet. Run the agent and write to {TRACE_PATH}.")
        else:
            for e in entries:
                render_step(e)

    if st.button("Refresh"):
        st.rerun()


if __name__ == "__main__":
    main()
