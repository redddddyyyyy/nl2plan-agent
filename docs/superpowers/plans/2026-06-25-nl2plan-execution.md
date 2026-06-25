# NL2Plan Agent Execution Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive the existing NL2Plan scaffold from "Python-only agent passes tests" to a 90s demo of a Qwen2.5-7B agent commanding Hybrid A* + GroundingDINO + MoveIt2 in Gazebo.

**Architecture:** Already locked in RESUME.md. LLM does task-level reasoning via Ollama tool-use; classical/verified ROS2 nodes (planner, controller, perception, manipulation) do low-level control. Recovery loop re-plans on tool failures. SayCan / PaLM-E lineage.

**Tech Stack:** ROS2 Humble, Gazebo Classic 11, MoveIt2, Ollama (Qwen2.5-7B), GroundingDINO, hybrid-astar-planner (user's repo, reused as-is), Ackermann mobile manipulator (F1Tenth + Open Manipulator-X), rclpy, Python 3.10.

## Global Constraints

- **Do not re-litigate any locked-in decision from RESUME.md** (planner choice, simulator, manipulation scope, LLM, robot platform, framing, timeline).
- **Bicycle kinematics is non-negotiable** (Hybrid A* assumes it).
- **Never compare to Tesla Optimus** anywhere in code, README, or notes.
- **Local Ollama only**, no hosted LLMs.
- Realistic horizon: **4–6 weeks of evening work**, not one session. This plan stages the work; not all phases land today.
- Commit and push to `redddddyyyyy/nl2plan-agent` frequently; future sessions resume from GitHub, not from this machine's disk.
- Author code on Linux only from here on out — the Windows path in RESUME.md is historical.

---

## Environment Audit (2026-06-25)

| Component | Status | Notes |
|---|---|---|
| Ubuntu 22.04.5 | ✅ | jammy |
| ROS2 Humble | ✅ | sourced in `~/.bashrc` |
| Gazebo Classic 11.10.2 | ✅ | |
| MoveIt2 (Humble) | ✅ | `ros-humble-moveit-*` present |
| colcon, rosdep | ✅ | |
| Python 3.10.12 | ✅ | |
| Disk (94 GB free) | ✅ | model weights + builds will fit |
| RAM (31 GB) | ✅ | |
| **Ollama** | ❌ | not installed |
| **NVIDIA driver** | ❌ | `nvidia-smi` reports "couldn't communicate with the NVIDIA driver"; no `nvidia-driver-*` package installed |
| **hybrid-astar-planner repo** | ❌ | exists on GitHub but not cloned locally; `setup_linux.sh` will clone it |

Two blockers (NVIDIA driver, Ollama) require sudo and the NVIDIA install needs a reboot. Phase 0 cannot finish until the user OKs both.

---

## Phase 0 — Environment Prep (this session)

### Task 0.1: Install Ollama and pull the default model

**Files:** none (system install)

**Interfaces:**
- Produces: `ollama` CLI on PATH, model `qwen2.5:7b-instruct` cached locally.

- [ ] **Step 1: Install Ollama via the official script (requires sudo)**

```bash
curl -fsSL https://ollama.com/install.sh | sh
```

Expected: installer reports "Ollama is now running" and starts a systemd service `ollama.service`.

- [ ] **Step 2: Verify the daemon is up**

```bash
systemctl status ollama --no-pager | head -5
ollama --version
```

- [ ] **Step 3: Pull the default model**

```bash
ollama pull qwen2.5:7b-instruct
```

Expected: ~4.4 GB download. Without GPU, this will still work but inference will be slow until Phase 0.2 lands.

- [ ] **Step 4: Smoke-test**

```bash
ollama run qwen2.5:7b-instruct "Reply with the single word: ok"
```

Expected: model returns `ok` (or close to it).

### Task 0.2: Resolve NVIDIA driver (gated by user approval)

**Files:** none (system install + reboot)

**Interfaces:**
- Produces: `nvidia-smi` reports the RTX 4070 and a driver version; CUDA is loadable.

This is a **destructive system change** — sudo + reboot. Do not run without explicit user confirmation.

- [ ] **Step 1: Identify the recommended driver**

```bash
ubuntu-drivers devices
```

Expected: a list of recommended `nvidia-driver-XXX` packages (likely 535 or 550 on 22.04).

- [ ] **Step 2: Install recommended driver**

```bash
sudo ubuntu-drivers autoinstall
```

- [ ] **Step 3: Reboot**

```bash
sudo reboot
```

- [ ] **Step 4: After reboot, verify**

```bash
nvidia-smi
```

Expected: table with the RTX 4070 listed, driver version printed, no error.

- [ ] **Step 5: Verify Ollama uses GPU**

```bash
ollama run qwen2.5:7b-instruct "say hi" &
sleep 5
nvidia-smi | grep -i ollama
```

Expected: Ollama process appears in the GPU memory column.

### Task 0.3: Run `setup_linux.sh`

**Files:**
- Modify (transitive, via colcon): `build/`, `install/`, `log/`
- Create (transitive): `src/_hybrid_astar_repo/`, symlink `src/hybrid_astar_planner`

- [ ] **Step 1: Source ROS2**

```bash
source /opt/ros/humble/setup.bash
```

- [ ] **Step 2: Run the script**

```bash
cd "/home/reddy/NL2plan agent"
bash scripts/setup_linux.sh
```

Expected: each `==>` banner runs cleanly. If apt fails, read the error before retrying.

- [ ] **Step 3: Source the new overlay**

```bash
source install/setup.bash
```

- [ ] **Step 4: Confirm the planner package is discoverable**

```bash
ros2 pkg list | grep -E "hybrid_astar|controller_node|perception_node|manipulation_node|kitchen_world|nl2plan_agent"
```

Expected: all six packages listed.

### Task 0.4: Verify the Python agent still passes on Linux

**Files:**
- Test: `tests/test_agent.py` (no changes)

- [ ] **Step 1: Install Python deps for the agent**

```bash
pip install --user -r requirements.txt
```

- [ ] **Step 2: Run the existing test suite**

```bash
cd "/home/reddy/NL2plan agent"
python -m pytest tests/test_agent.py -v
```

Expected: **6/6 passing** (the RESUME.md guarantee).

- [ ] **Step 3: Smoke-test the agent loop against the mock backend**

```bash
python -m nl2plan_agent.agent --mock "pick up the red mug and bring it to the start"
```

Expected: agent emits a sequence of tool calls and the mock backend reports success. `logs/agent_trace.jsonl` gains entries.

- [ ] **Step 4: Commit any minor Linux-vs-Windows fixups**

If any test failed because of a path separator or shebang, fix and commit:

```bash
git status
git diff
git add -p
git commit -m "fix: linux-compat tweaks surfaced by first run"
git push
```

---

## Phase 1 — Stage 1 Foundation (this session if 0.x finishes)

**Acceptance gate (RESUME.md):** "Robot drives point-to-point in Gazebo with the planner's path visible in RViz."

### Task 1.1: Author the kitchen world SDF

**Files:**
- Modify: `src/kitchen_world/worlds/kitchen.world`

Build a Gazebo world with: a floor, walls forming a kitchen alcove, a table mesh, a red mug `model://red_mug` on the table, two `chair` models as obstacles in the navigable area, a "start" pose region marked with a visual-only floor decal so the operator can see it in RViz.

- [ ] **Step 1: Read the current `kitchen.world` to see what's scaffolded**
- [ ] **Step 2: Add `<world>` children: ground plane, sun, kitchen walls, table, mug, two chairs**
- [ ] **Step 3: Launch alone to verify it loads**

```bash
gazebo "/home/reddy/NL2plan agent/src/kitchen_world/worlds/kitchen.world"
```

Expected: a Gazebo window opens with the kitchen, mug, table, and chairs visible. Close it.

- [ ] **Step 4: Commit**

```bash
git add src/kitchen_world/worlds/kitchen.world
git commit -m "feat(kitchen_world): author kitchen SDF with table, mug, chairs"
```

### Task 1.2: Author the Ackermann mobile-manipulator URDF/xacro

**Files:**
- Modify: `src/kitchen_world/urdf/robot.urdf.xacro`

Compose: F1Tenth-style chassis (rectangular base, 4 wheels with Ackermann steering joints), Open Manipulator-X arm mounted on top, RGB-D camera on the front, IMU + odometry plugin, ackermann_drive plugin parameters that match the planner's bicycle assumption.

Concrete must-haves so later tasks line up:
- `base_link` is the planner reference frame.
- `front_camera_optical` is the camera frame perception_node reads.
- `wheel_base` xacro property exposed; controller_node will read it as a ROS2 param defaulting to the same value.

- [ ] **Step 1: Read existing scaffold**
- [ ] **Step 2: Write the xacro**
- [ ] **Step 3: Validate**

```bash
xacro "/home/reddy/NL2plan agent/src/kitchen_world/urdf/robot.urdf.xacro" > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

Expected: `Successfully Parsed XML`, no missing-link errors.

- [ ] **Step 4: Visualize in RViz briefly via `robot_state_publisher`** (sanity check)
- [ ] **Step 5: Commit**

### Task 1.3: Implement Pure Pursuit `controller_node`

**Files:**
- Modify: `src/controller_node/controller_node/controller.py` (currently a 30-line stub)

**Interfaces:**
- Consumes: `/planned_path` (`nav_msgs/Path`) from `hybrid_astar_planner`; `/odom` (`nav_msgs/Odometry`) from Gazebo.
- Produces: `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`).

Recipe is in the existing module docstring. The implementation must:
1. Maintain a buffered latest `Path`.
2. On each `/odom` callback, find the first path point with distance ≥ `Ld` from the robot.
3. Compute `alpha = heading_to_lookahead - yaw`.
4. `delta = atan2(2 * wheelbase * sin(alpha), Ld)`.
5. Publish `AckermannDriveStamped(steering_angle=delta, speed=target_speed)`.
6. Stop (speed=0) within `goal_tolerance` of the final point.

Params (declare with `node.declare_parameter`):
- `lookahead_distance` (float, default 0.6 m)
- `wheelbase` (float, default 0.3 m — matches URDF)
- `target_speed` (float, default 0.5 m/s)
- `goal_tolerance` (float, default 0.15 m)

- [ ] **Step 1: Write a unit test that doesn't need rclpy**

Pull the math into a pure function `pure_pursuit_command(robot_pose, path, params) -> (steering_angle, speed)` and test it in `tests/test_controller.py`:

```python
import math
from controller_node.controller import pure_pursuit_command, ControllerParams

def test_straight_ahead_steers_zero():
    path = [(x, 0.0) for x in [0.0, 0.5, 1.0, 1.5, 2.0]]
    pose = (0.0, 0.0, 0.0)  # at origin facing +x
    params = ControllerParams(lookahead_distance=0.6, wheelbase=0.3,
                              target_speed=0.5, goal_tolerance=0.15)
    delta, speed = pure_pursuit_command(pose, path, params)
    assert abs(delta) < 1e-3
    assert speed == 0.5

def test_left_turn_steers_positive():
    path = [(0.5*math.cos(t), 0.5*math.sin(t)) for t in [0.0, 0.5, 1.0]]
    pose = (0.0, 0.0, 0.0)
    params = ControllerParams(0.4, 0.3, 0.5, 0.15)
    delta, _ = pure_pursuit_command(pose, path, params)
    assert delta > 0.0

def test_within_tolerance_stops():
    path = [(0.0, 0.0), (0.05, 0.0)]
    pose = (0.04, 0.0, 0.0)
    params = ControllerParams(0.6, 0.3, 0.5, 0.15)
    _, speed = pure_pursuit_command(pose, path, params)
    assert speed == 0.0
```

- [ ] **Step 2: Run test, watch it fail**

```bash
python -m pytest tests/test_controller.py -v
```

Expected: ImportError or AttributeError.

- [ ] **Step 3: Implement the pure function + the rclpy node wrapper**

In `controller.py`:

```python
import math
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped


@dataclass
class ControllerParams:
    lookahead_distance: float
    wheelbase: float
    target_speed: float
    goal_tolerance: float


def _yaw_from_quat(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def pure_pursuit_command(robot_pose, path: List[Tuple[float, float]],
                         params: ControllerParams):
    if not path:
        return 0.0, 0.0
    rx, ry, ryaw = robot_pose
    # Goal-reached check uses the final point
    gx, gy = path[-1]
    if math.hypot(gx - rx, gy - ry) <= params.goal_tolerance:
        return 0.0, 0.0
    # Lookahead: first point at distance >= Ld; fall back to last
    lookahead = path[-1]
    for px, py in path:
        if math.hypot(px - rx, py - ry) >= params.lookahead_distance:
            lookahead = (px, py)
            break
    lx, ly = lookahead
    alpha = math.atan2(ly - ry, lx - rx) - ryaw
    # Normalize alpha to [-pi, pi]
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    delta = math.atan2(2.0 * params.wheelbase * math.sin(alpha),
                       params.lookahead_distance)
    return delta, params.target_speed


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.declare_parameter("lookahead_distance", 0.6)
        self.declare_parameter("wheelbase", 0.3)
        self.declare_parameter("target_speed", 0.5)
        self.declare_parameter("goal_tolerance", 0.15)
        self._path: List[Tuple[float, float]] = []
        self.create_subscription(Path, "/planned_path", self._on_path, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self._pub = self.create_publisher(AckermannDriveStamped,
                                          "/ackermann_cmd", 10)

    def _on_path(self, msg: Path):
        self._path = [(p.pose.position.x, p.pose.position.y)
                      for p in msg.poses]

    def _on_odom(self, msg: Odometry):
        if not self._path:
            return
        p = msg.pose.pose
        yaw = _yaw_from_quat(p.orientation.x, p.orientation.y,
                             p.orientation.z, p.orientation.w)
        params = ControllerParams(
            self.get_parameter("lookahead_distance").value,
            self.get_parameter("wheelbase").value,
            self.get_parameter("target_speed").value,
            self.get_parameter("goal_tolerance").value,
        )
        delta, speed = pure_pursuit_command(
            (p.position.x, p.position.y, yaw), self._path, params
        )
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.steering_angle = delta
        cmd.drive.speed = speed
        self._pub.publish(cmd)


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run unit tests, expect green**

```bash
python -m pytest tests/test_controller.py -v
```

- [ ] **Step 5: colcon build, source, smoke-test the node**

```bash
colcon build --packages-select controller_node --symlink-install
source install/setup.bash
ros2 run controller_node controller_node &
ros2 topic info /planned_path
ros2 topic info /ackermann_cmd
# publish a fake path
ros2 topic pub --once /planned_path nav_msgs/msg/Path '...'
ros2 topic echo /ackermann_cmd --once
```

Expected: an `AckermannDriveStamped` lands on the wire.

- [ ] **Step 6: Commit**

```bash
git add src/controller_node tests/test_controller.py
git commit -m "feat(controller_node): pure pursuit implementation + unit tests"
git push
```

### Task 1.4: Wire the launch file + Stage-1 verification

**Files:**
- Modify: `src/kitchen_world/launch/kitchen.launch.py`

The launch must start: Gazebo with `kitchen.world`, `robot_state_publisher` with the URDF, the user's `hybrid_astar_planner` node (whatever its node name is — read its README), the `controller_node`, RViz with a config that shows `/planned_path` and the robot model.

- [ ] **Step 1: Read the planner repo's README + launch files to see how to start it**

```bash
ls "/home/reddy/NL2plan agent/src/_hybrid_astar_repo/ros2_ws/src/hybrid_astar_planner/"
```

- [ ] **Step 2: Update `kitchen.launch.py` to compose the full Stage-1 graph**
- [ ] **Step 3: Launch it**

```bash
ros2 launch kitchen_world kitchen.launch.py
```

- [ ] **Step 4: From a second terminal, send a goal pose**

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '...'
```

Expected: planner publishes `/planned_path`, controller publishes `/ackermann_cmd`, robot drives the path in Gazebo, path visible as a strip in RViz.

- [ ] **Step 5: Commit launch + RViz config**

```bash
git add src/kitchen_world/launch src/kitchen_world/rviz
git commit -m "feat(kitchen_world): stage-1 launch wires planner + controller + sim"
git push
```

---

## Phase 2 — Stage 2 Tool Layer (later session)

Detailed plan to be written when Phase 1 lands. Sketch:

### Task 2.1: `perception_node` — GroundingDINO + RGB-D + TF

Download SwinT weights once (~700 MB) to `~/.cache/grounding_dino/`. Subscribe `/camera/color/image_raw` + `/camera/depth/image_rect_raw`. Expose ROS2 service `/find_object` with request `{query: string}` and response `{success: bool, pose: PoseStamped, confidence: float}`. Pipeline: image → DINO inference → top box → depth lookup at box centroid → camera-frame point → tf2 transform to `map` frame.

### Task 2.2: `manipulation_node` — MoveIt2 pick/place

Use `pymoveit2` or `moveit_py`. Services `/pick` and `/place`, both taking `PoseStamped`. Pick = approach 10cm above target → open gripper → descend → close → lift. Place = approach above destination → descend → open → retreat.

### Task 2.3: Replace `Ros2Backend` stub with real rclpy clients

Each method in `tools.py:Ros2Backend` calls the corresponding ROS2 service or action. Map failures to the structured error format the agent already understands. Add one integration test that runs the full agent against live nodes and asserts a `pick the red mug` plan completes.

---

## Phase 3 — Stage 3 LLM + Recovery (later session)

### Task 3.1: Run 3 demo commands × 10 each, log results

Commands from README:
1. "Go to the kitchen."
2. "Pick up the red mug and bring it to the start."
3. "Bring me the red mug." (with mid-run obstacle dropped in)

Gates: ≥7/10 on 1 & 2, ≥5/10 on 3.

### Task 3.2: If gates fail — prompt tuning, then larger model

In order: tighten few-shot in `prompt.py`, add more named poses, switch to `llama3.1:8b-instruct`, then `hermes3:8b`, then bump to a 13B variant (RTX 4070 has headroom).

### Task 3.3: Fill the metrics table in `README.md`

---

## Phase 4 — Stage 4 Polish (later session)

### Task 4.1: 90-second one-take recording

OBS or `ffmpeg` screen-record. Split screen: terminal + RViz + Gazebo. Caption each command on screen.

### Task 4.2: Final README pass, post to GitHub/LinkedIn/YouTube

---

## Self-Review Notes

- **Spec coverage:** every RESUME.md stage maps to a Phase here; the four `TODO(linux)` files each have a task; the kitchen world + URDF are split out so they don't get smuggled into a single 6-hour task.
- **Phase 0 blockers are gated on user approval** because they touch system state. No automatic sudo.
- **Phase 1 is the only phase with full step-level code in this plan.** Phases 2–4 are sketched because they depend on the URDF/world choices we lock in Phase 1, and writing them in full now invites churn. Re-open this doc and detail them when Phase 1 lands.
- **DRY/YAGNI check:** the pure-Python `pure_pursuit_command` lets us test the math without rclpy — important because rclpy can't be installed cleanly into the repo's pip env. No premature abstraction beyond that split.
