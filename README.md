# NL2Plan Agent

Natural-language commands → structured plan → verified low-level control in simulation.

A local LLM (Qwen2.5-7B via Ollama) takes a command like *"go to the red block, pick it up, and bring it back to the start"*, decomposes it into a short sequence of tool calls, and drives a Nav2 navigation stack, a perception node, and an arm controller inside Gazebo. A recovery loop re-plans when a tool call fails.

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
  find_object    --|--> [ROS2 actions/topics] -----+
  pick           --|       |
  place          --+       v
                       [Robot stack]
                         Nav2 (smac_planner + AMCL + controller_server)
                         block_detector  (HSV; upgrade target: GroundingDINO)
                         arm + gripper controllers (ros2_control)
                              |
                              v
                       [Gazebo] mobile_arm_sim — mecanum mobile manipulator
```

## Repo layout

```
NL2Plan agent/
  src/
    nl2plan_agent/       agent orchestrator + tool dispatcher + Streamlit sidebar
    perception_node/     wraps the perception topic the agent's find_object reads
    manipulation_node/   wraps arm + gripper controllers for pick/place
  config/named_poses.yaml
  scripts/setup_linux.sh
  tests/test_agent.py    runs unchanged on Windows or Linux (MockBackend, no ROS2)
  docs/
  logs/                  agent_trace.jsonl is written here at runtime
```

The robot itself lives in a sibling workspace: `/home/reddy/ros2_ws/src/mobile_arm_sim/` (the prof's project). NL2Plan depends on `mobile_arm_sim` being launched and its topics/actions live.

## Quickstart

The pure-Python agent + tests run on Windows (`pip install -r requirements.txt`); everything that talks to ROS2 needs Linux.

### Try the agent dry (Windows or Linux, no ROS2 needed)

```bash
pip install -r requirements.txt
ollama pull qwen2.5:7b-instruct
python -m nl2plan_agent.agent --mock "pick up the red mug and bring it back"
```

The mock backend has a toy world with a "red mug" near a "kitchen" pose. You'll see the LLM decompose the command and call the tools against the mock world.

### Full sim (Linux + ROS2 Humble + Ollama + mobile_arm_sim)

```bash
bash scripts/setup_linux.sh                                   # ROS2 deps + Python deps + colcon build
source install/setup.bash
source /home/reddy/ros2_ws/install/setup.bash                 # mobile_arm_sim
ros2 launch mobile_arm_sim autonomous.launch.py               # Gazebo + Nav2 + perception (once Day 4 lands)
python -m nl2plan_agent.agent "go to the red block and bring it back"
streamlit run src/nl2plan_agent/nl2plan_agent/sidebar_app.py  # in a second terminal
```

## Stages (matches the implementation plan)

| Stage | What | Verification |
| --- | --- | --- |
| 0 | Linux env + Ollama + agent smoke test | 6/6 pytest pass, end-to-end mock-mode run completes with real LLM |
| 1 | mobile_arm_sim infrastructure (sensors + scene + map + Nav2 + perception) | "2D Nav Goal" in RViz drives the robot around obstacles; HSV detector publishes pose only on the target color |
| 2 | Tool layer — wire Ros2Backend to Nav2 + perception + arm | Each of the four tools demonstrable from a Python REPL against live Gazebo |
| 3 | LLM agent + recovery loop on the real sim | Three NL commands each run 10×; ≥7/10 on commands 1–2, ≥5/10 on command 3 |
| 4 | Polish + 90s demo video + README metrics | One-take 90s recording of the three commands |

## Design trade-offs

These are honest and meant to be visible in the README, not hidden.

- **Consolidated with the prof's `mobile_arm_sim`.** The robot, sim, sensors, and Nav2 stack are shared with the user's professor's research project. Two deliverables ship from one infrastructure: a Python state-machine version (the prof's checkpoint) and this LLM-driven version. Earlier iteration committed to a custom Ackermann chassis + Hybrid A*; switched after the prof robot's holonomic kinematics + Nav2 plan came into view. Hybrid A* may return as a future enhancement.
- **Local 7B model tool-use reliability.** Models this size fail JSON schema validation 5-15% of the time. The dispatcher does schema validation, JSON repair, and a single retry before reporting a structured error to the model. Measured failure rate is in the metrics table below.
- **Framing.** This is the SayCan / PaLM-E lineage (LLM task planning over verified controllers). Not Tesla Optimus, which is moving toward end-to-end neural policies.

## Metrics

(Filled in after Stage 3.)

| Command | Success rate | Avg plan time | Avg execution time | Recovery rate |
| --- | --- | --- | --- | --- |
| `Go to the red block.` | TBD | TBD | TBD | n/a |
| `Pick up the red block and bring it to the start.` | TBD | TBD | TBD | n/a |
| `Bring me the red block.` (with mid-run obstacle) | TBD | TBD | TBD | TBD |

## Acknowledgements

Builds on:
- [Nav2](https://navigation.ros.org/)
- [Ollama](https://ollama.com/)
- The `mobile_arm_sim` robot from ongoing professor-led research (private)

Future enhancements:
- [Hybrid A* path planner](https://github.com/redddddyyyyy/hybrid-astar-planner) (parked)
- [GroundingDINO](https://github.com/IDEA-Research/GroundingDINO) (planned upgrade for `find_object`)
- [MoveIt2](https://moveit.ai/) (planned upgrade for `pick` / `place`)

## License

MIT.
