# Resume context for NL2Plan agent

> This file is a session-handoff doc. If you (Claude Code, or a human picking this up) are starting cold on this repo — read this first, then `README.md`. Everything below is decided context, not up for re-debate unless explicitly reopened.

Last updated: **2026-06-25**.

---

## What this project is (one paragraph)

A local LLM (Qwen2.5-7B via Ollama) takes a natural-language command like *"go to the kitchen, pick up the red mug, bring it back, avoid the chairs"*, decomposes it into a short sequence of tool calls, and drives a real Hybrid A* planner, a GroundingDINO perception node, and a MoveIt2 manipulation stack inside Gazebo. A recovery loop re-plans when a tool call fails. The pattern is the SayCan / PaLM-E lineage: LLM does task-level reasoning; classical / verified components do low-level control.

**Target outcome:** a 90-second demo video + a README with quantitative metrics, used as a portfolio piece for agentic-AI, robotics, CV, and AV roles.

---

## Locked-in decisions — do NOT re-litigate

| Decision | Choice |
|---|---|
| Planner | Reuse the user's existing repo `github.com/redddddyyyyy/hybrid-astar-planner` as-is. Already publishes `/planned_path` (nav_msgs/Path). Bicycle kinematics. |
| Simulator + OS | Native Linux + ROS2 Humble + Gazebo Classic. |
| Manipulation | Full pick-and-place via MoveIt2. Not mocked. Scope justified by overlap with user's professor research. |
| LLM | Local via Ollama. Default `qwen2.5:7b-instruct`. Fallbacks: `llama3.1:8b-instruct`, `hermes3:8b`. RTX 4070 has headroom for 13B if 7B prompt-tuning stalls. |
| Robot platform | Ackermann mobile manipulator (F1Tenth-style chassis + Open Manipulator-X arm). Bicycle kinematics is non-negotiable because Hybrid A* assumes it. |
| Framing | SayCan / PaLM-E lineage. **Never** compare to Tesla Optimus in code, README, or interview prep — Optimus is end-to-end neural, wrong analogue. |
| Timeline | 4–6 weeks realistic. Not the 2 weeks from the original pitch. |

---

## Environment

- **Windows side (current workspace, code authored here):** `C:\Users\rajee\claude\NL2Plan agent\`. Pure-Python agent + tests run here without ROS2.
- **Linux side:** dual-boot on the same machine. NVIDIA RTX 4070 with CUDA enabled. ROS2 Humble + Gazebo + Ollama all run here. GPU is plenty for GroundingDINO (~100ms/frame) and Ollama 7B–13B models.
- **No file-sync needed** — code lives on GitHub now. Linux side will `git clone` rather than mounting the Windows partition.

---

## Implementation status

### Done (Windows side, runnable without ROS2)
- Agent core, **6/6 pytest passing**:
  - `src/nl2plan_agent/nl2plan_agent/agent.py` — Ollama loop, 12-step cap, 300s wall-clock cap, JSON repair, schema validation, structured trace logging to `logs/agent_trace.jsonl`
  - `tools.py` — `ToolDispatcher` + working `MockBackend` + `Ros2Backend` stub (raises on Windows by design)
  - `tool_schemas.py` — four tools (`navigate_to`, `find_object`, `pick`, `place`) in Ollama tool-use format
  - `prompt.py` — system prompt with find-before-pick rule + few-shot
  - `sidebar_app.py` — Streamlit live trace viewer
  - `named_poses.py` — YAML loader
- ROS2 package scaffolds with `TODO(linux)` recipes in docstrings:
  - `controller_node` (Pure Pursuit)
  - `perception_node` (GroundingDINO)
  - `manipulation_node` (MoveIt2)
  - `kitchen_world` (world + URDF + launch)
- `scripts/setup_linux.sh`, `README.md`, `requirements.txt`, `.gitignore`, `config/named_poses.yaml`
- `tests/test_agent.py` — happy path, JSON repair, schema validation, recovery, step cap, mock backend

### Pending (needs Linux box)
- **Stage 1 — Foundation:** bring up Gazebo + the user's Hybrid A* planner + Pure Pursuit controller. Verify point-to-point nav in RViz.
- **Stage 2 — Tool layer:** fill in every `TODO(linux)` marker in `controller.py`, `perception.py`, `manipulation.py`. Build the kitchen world SDF and the Ackermann mobile-manipulator URDF.
- **Stage 3 — LLM + recovery:** tune Qwen2.5-7B prompt against real ROS2 tools. Run the three demo commands × 10 each. Record metrics (success rate, replan latency, recovery rate).
- **Stage 4 — Polish:** 90s demo video, fill metrics table in README, post to GitHub/LinkedIn/YouTube.

### Open questions still parked
1. Constraints from the user's professor's research (specific robot, specific objects)? May change Stage 1 robot-platform choice.

---

## Portfolio sequencing (decided 2026-06-25)

User is targeting **agentic AI (primary), CV, autonomous driving, and robotics (secondary)** roles.

- **Ship NL2Plan first.** Justification: it's a rarer kind of agent project — one that actually executes against the real world (real planner, real perception, real sim) rather than another text-only LLM demo. That story translates across all four target bands.
- **Nav2 debugging agent comes next**, after NL2Plan ships. Reuses ROS2 + Linux setup, broadens the agentic-AI portfolio, complements rather than competes. Don't start it before NL2Plan ships — context-switching mid-build kills both.

---

## First action when you (or Linux Claude Code) resume on Linux

```bash
# 1. Clone fresh on Linux
git clone https://github.com/redddddyyyyy/nl2plan-agent.git
cd nl2plan-agent

# 2. Run the Linux setup script (installs ROS2 Humble deps, MoveIt2, clones planner, builds)
bash scripts/setup_linux.sh

# 3. Source the workspace
source install/setup.bash

# 4. Verify Stage 1 — bring up the sim
ros2 launch kitchen_world kitchen.launch.py

# 5. Start working through the TODO(linux) markers in:
#    src/controller_node/controller_node/controller.py     (Pure Pursuit)
#    src/perception_node/perception_node/perception.py     (GroundingDINO)
#    src/manipulation_node/manipulation_node/manipulation.py  (MoveIt2)
```

If `setup_linux.sh` fails partway, read the script — it's idempotent and documents each step. Don't paper over errors; investigate.

To smoke-test the agent loop without ROS2 (works on Linux or Windows):

```bash
pip install -r requirements.txt
ollama pull qwen2.5:7b-instruct
python -m nl2plan_agent.agent --mock "pick up the red mug and bring it to the start"
```

---

## Key files map

```
src/nl2plan_agent/nl2plan_agent/
  agent.py           ← Ollama loop, recovery, tracing. Main entry point.
  tools.py           ← ToolDispatcher + MockBackend + Ros2Backend stub.
  tool_schemas.py    ← The four tools' JSON schemas.
  prompt.py          ← System prompt + few-shot.
  sidebar_app.py     ← Streamlit trace viewer (run separately).

src/controller_node/controller_node/controller.py     ← TODO(linux): Pure Pursuit.
src/perception_node/perception_node/perception.py     ← TODO(linux): GroundingDINO + RGB-D + TF.
src/manipulation_node/manipulation_node/manipulation.py ← TODO(linux): MoveIt2 wrapper.
src/kitchen_world/                                    ← Gazebo world + URDF + launch.

scripts/setup_linux.sh   ← Linux bootstrap. Read before running.
config/named_poses.yaml  ← Named pose registry.
tests/test_agent.py      ← 6 tests, all passing on Windows.
```

---

## Things to flag if they come up

- **"Let's switch the planner / use Nav2 instead of Hybrid A*"** → no. Reusing the user's existing planner is a locked decision.
- **"Let's use a diff-drive robot"** → no. Bicycle kinematics is required by Hybrid A*.
- **"Should we compare to Tesla Optimus in the README?"** → no, wrong analogue.
- **"Let's mock the manipulation"** → no, full MoveIt2 is in scope.
- **"Should we use a hosted LLM (Claude, GPT-4) instead of Ollama?"** → no, local Ollama is the choice. The interesting tension is *7B reliability under tool use*, which is part of the story.
