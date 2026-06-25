# NL2Plan Agent

Natural-language commands -> structured plan -> verified low-level control in simulation.

A local LLM (Qwen2.5-7B via Ollama) takes a command like *"go to the kitchen, pick up the red mug, bring it back, avoid the chairs"*, decomposes it into a short sequence of tool calls, and drives a real Hybrid A* planner, an open-vocabulary perception node (GroundingDINO), and a MoveIt2 manipulation stack inside Gazebo. A recovery loop re-plans when a tool call fails.

The pattern is the SayCan / PaLM-E lineage: the LLM does task-level reasoning, classical / verified components do low-level control. The point is to show LLM portfolio work that has physical grounding, not just prompt engineering.

## Architecture

```
[CLI / Streamlit]
       |
       v
[Agent orchestrator]  <----- failure feedback -----+
       |                                            |
       | tool calls (JSON via Ollama tool-use API)  |
       v                                            |
[Tool dispatcher]                                   |
  navigate_to    --+                                |
  find_object    --|--> [ROS2 services] --> [Gazebo]+
  pick           --|
  place          --+
       |
       v
[ROS2 nodes]
  hybrid_astar_planner   (https://github.com/redddddyyyyy/hybrid-astar-planner, as-is)
  perception_node        (GroundingDINO + RGB-D + TF)
  manipulation_node      (MoveIt2 wrapper for pick/place)
  controller_node        (Pure Pursuit path follower)
       |
       v
[Gazebo]  Ackermann mobile manipulator in a kitchen world
```

## Repo layout

```
NL2Plan agent/
  src/
    nl2plan_agent/       agent orchestrator + tool dispatcher + Streamlit sidebar
    controller_node/     Pure Pursuit path follower
    perception_node/     GroundingDINO wrapper
    manipulation_node/   MoveIt2 wrapper
    kitchen_world/       Gazebo world + URDF
    hybrid_astar_planner/  git symlink to the user's planner repo (created by setup_linux.sh)
  config/named_poses.yaml
  scripts/setup_linux.sh
  tests/test_agent.py    Windows/Linux-runnable agent tests (mocked ROS2)
  docs/
  logs/                  agent_trace.jsonl is written here at runtime
```

## Quickstart

The pure-Python agent + tests run on Windows (`pip install -r requirements.txt`); everything that talks to ROS2 needs Linux.

### Try the agent dry (Windows or Linux, no ROS2 needed)

```bash
pip install -r requirements.txt
python -m nl2plan_agent.agent --mock "pick up the red mug and bring it back"
```

The mock backend has a toy world with a "red mug" on a "table" near a "kitchen" pose. You'll see the LLM decompose the command and call the tools against the mock world.

### Full sim (Linux + ROS2 Humble + Ollama)

```bash
bash scripts/setup_linux.sh         # installs ROS2 deps, MoveIt2, clones the planner, builds
source install/setup.bash
ros2 launch kitchen_world kitchen.launch.py    # Gazebo + nodes
python -m nl2plan_agent.agent "go to the kitchen and bring back the red mug"
streamlit run src/nl2plan_agent/nl2plan_agent/sidebar_app.py    # in a second terminal
```

## Stages (matches the implementation plan)

| Stage | What | Verification |
| --- | --- | --- |
| 1 | Workspace + planner + sim foundation | Robot drives point-to-point in Gazebo with the planner's path visible in RViz |
| 2 | Tool layer + perception + manipulation | Each of the four tools demonstrable from a Python REPL against live Gazebo |
| 3 | LLM agent + recovery loop | Three NL commands each run 10x; >=7/10 on commands 1-2, >=5/10 on command 3 |
| 4 | Polish + 90s demo video + README metrics | One-take 90s recording of the three commands |
| 5 (optional) | Research handoff | Perception + manipulation nodes documented for reuse |

## Design trade-offs

These are honest and meant to be visible in the README, not hidden.

- **Bicycle kinematics over diff-drive mobile manipulator.** Hybrid A* is built for non-holonomic Ackermann vehicles. Using an Ackermann base instead of a Fetch / Tiago keeps the planner's assumptions honest at the cost of a less common robot form factor.
- **Local 7B model tool-use reliability.** Models this size fail JSON schema validation 5-15% of the time. The dispatcher does schema validation, JSON repair, and a single retry before reporting a structured error to the model. Measured failure rate is in the metrics table below.
- **Framing.** This is the SayCan / PaLM-E lineage (LLM task planning over verified controllers). Not Tesla Optimus, which is moving toward end-to-end neural policies.

## Metrics

(Filled in after Stage 3.)

| Command | Success rate | Avg plan time | Avg execution time | Recovery rate |
| --- | --- | --- | --- | --- |
| `Go to the kitchen.` | TBD | TBD | TBD | n/a |
| `Pick up the red mug and bring it to the start.` | TBD | TBD | TBD | n/a |
| `Bring me the red mug.` (with mid-run obstacle) | TBD | TBD | TBD | TBD |

## Acknowledgements

Builds on:
- [Hybrid A* path planner](https://github.com/redddddyyyyy/hybrid-astar-planner)
- [GroundingDINO](https://github.com/IDEA-Research/GroundingDINO)
- [MoveIt2](https://moveit.ai/)
- [Ollama](https://ollama.com/)

## License

MIT.
