# Architecture

## Why this document

The research agenda from 22 July asks for a *modular robotic simulation
architecture* — one that transfers to another environment, and one whose safety
properties can eventually be proved rather than asserted. This document states
what the modules in this repository actually are, what contract each one
publishes, and what it would cost to replace each one.

I want to be straight about the order things happened in. I built this
working-first: get an LLM to drive a real Nav2 stack, then fix what broke. The
structure below is read out of the code that resulted, not a plan I drew up
front and executed. Some of the boundaries are clean because I needed them to be
— I could not test an LLM loop against a Gazebo sim on every commit, so a seam
had to exist. Others are cleaner in the diagram than in the source, and section
6 says which.

## 1. The layers

```
  natural-language command
           |
           v
  +--------------------------------+
  |  Orchestrator      agent.py    |   LLM loop, step + wall-clock caps,
  |                                |   narration/empty-reply nudges, JSONL trace
  +--------------------------------+
           |  chat_fn (injected)        -> Ollama, or any callable, or a stub
           |  tool calls
           v
  +--------------------------------+
  |  Tool contract     tool_schemas |  4 tools, JSON Schema per tool
  |  Dispatcher        tools.py     |  validate -> repair -> execute -> log
  +--------------------------------+
           |  ToolBackend Protocol: navigate_to / find_object / pick / place
           |
     +-----+------------------+
     v                        v
  MockBackend            Ros2Backend  -> ros_backend/
  pure Python                          backend.py   tool bodies, cross-call state
  no ROS                               nav.py       Nav2 action client
                                       perception.py  scan / confirm gates
                                       manipulation.py  arm, creep, grasp pin
                                       node.py      the one long-lived ROS node
                                       logic.py     pure math, imports no rclpy
                                            |
                                            |  ROS topics / actions / services
                                            v
                              +---------------------------------+
                              |  perception_node  (own package) |
                              |  HSV bands -> /block_pose/<col> |
                              +---------------------------------+
                              |  Nav2, ros2_control, Gazebo     |
                              |  (mobile_arm_sim, external)     |
                              +---------------------------------+
```

## 2. The seams that earn the word "modular"

Three boundaries carry the whole design. Each is checkable, not just claimed.

**The `ToolBackend` protocol** (`tools.py:32`). Four methods. `MockBackend` and
`Ros2Backend` both satisfy it, and the dispatcher holds one by structural type
rather than by inheritance. The consequence is the one that matters day to day:
the entire language side of the system — prompt, tool loop, schema validation,
recovery behaviour — runs and is tested with no ROS installed at all. The test
suite is 36 tests and finishes in about a second:

```
$ python3 -m pytest tests -p no:launch_testing -p no:launch_ros -q
....................................                                     [100%]
36 passed in 1.11s
```

**Injected `chat_fn`** (`agent.py:68`). The orchestrator takes the chat callable
as a constructor argument and only falls back to importing Ollama if none is
given. So the reverse cut also works: the robot-facing code can be driven by a
scripted sequence with no model in the loop. This is how the agent's failure
handling — malformed JSON, hallucinated tool names, step caps, the narration
nudge — is tested deterministically, which is not possible against a live 7B
model.

**`logic.py` imports no `rclpy`, deliberately.** Frame transforms, sighting
cluster math, Nav2 status mapping, standoff geometry, and the pick error strings
all live there as plain functions. That is the part of the robot code where a
sign error silently aims the robot at empty floor, and it is exactly the part
that is unit-testable without a simulator. The module even maps Nav2's
`GoalStatus` codes with a local integer table rather than importing
`action_msgs`, to keep the import graph clean.

There is a fourth boundary that is clean almost by accident: `perception_node`
is a separate ROS package in a separate process, and the only thing crossing
between it and the agent is a `PoseStamped` on `/block_pose/<color>`. No shared
objects, no shared Python. It is the most replaceable component in the repo, and
section 7 leans on that.

## 3. Module by module

| Module | What it does | Contract it publishes | Depends on | Cost to replace |
| --- | --- | --- | --- | --- |
| `agent.py` | Runs the multi-turn tool loop; enforces step and wall-clock caps; nudges the model when it narrates instead of acting or returns an empty message; writes a JSONL trace of every turn | `Agent.run(command, history) -> AgentResult` | `ChatFn`, `ToolDispatcher`, `SYSTEM_PROMPT`, `ALL_TOOLS` | **Low.** Swapping the model is a constructor argument. Swapping to a different agent framework means rewriting this file only; nothing below it knows an LLM exists |
| `tool_schemas.py` | The four tool definitions as JSON Schema, in the shape Ollama's tool-use API expects | `ALL_TOOLS`, `TOOLS_BY_NAME` | nothing | **Low**, but it is the interface definition — changing a schema means changing the prompt, both backends, and the dispatcher together |
| `tools.py` — `ToolDispatcher` | Validates every LLM-supplied argument against its schema, attempts JSON repair on malformed input, executes, catches exceptions, logs the call | `call(name, raw_args) -> ToolCallResult` | `jsonschema`, `json_repair`, a `ToolBackend` | **Low.** This is the safety-relevant chokepoint; see section 4 |
| `tools.py` — `MockBackend` | An in-memory house with the same rooms, blocks and error strings as the live one | `ToolBackend` | nothing | **Low**, but it is hand-maintained; see section 6 |
| `ros_backend/backend.py` | The four tool bodies, and the only state that spans tool calls: what is held, and the last confirmed detection | `ToolBackend` | everything below it | **High.** This is where the pick protocol lives — standoff, re-confirm, creep, refuse. Replacing it means re-deriving that protocol |
| `ros_backend/node.py` | The single long-lived ROS node: publishers, cached AMCL/odom/detection state, TF, the Gazebo entity-state client, and the 20 Hz grasp pin, spinning on a daemon executor thread | `get_node() -> BackendNode` | `rclpy`, `nav2_msgs`, `gazebo_msgs`, `tf2_ros` | **High.** Every ROS assumption in the project is concentrated here, which is the point — it is also therefore the single file a port to another middleware would rewrite |
| `ros_backend/nav.py` | Blocking Nav2 client: waits for `bt_navigator` to reach lifecycle *active*, sends one goal, blocks to a terminal result | `wait_nav_active`, `navigate` | `nav2_msgs`, `lifecycle_msgs` | **Low.** Roughly 80 lines against a standard Nav2 interface. A different planner that speaks `NavigateToPose` drops in unchanged |
| `ros_backend/perception.py` | `find_object`'s eyes: stop, spin, and accept nothing until four stationary sightings cluster within 0.2 m in the robot's frame, inside a plausible distance band, with a cross-colour veto | `scan_for`, `confirm_here` | `node`, `logic` | **Medium.** The gating policy is general; its constants are not |
| `ros_backend/manipulation.py` | Arm pose sequences, the align-and-creep dock with its stall guard, and the magic-grasp pin | `align`, `grasp_sequence`, `back_away`, `detach` | `node`, `rclpy`, `tf2_ros` | **High.** Joint targets and reach constants were measured on one robot |
| `ros_backend/logic.py` | Pure decision math, no ROS | plain functions | `math`, `re` | **Low**, and it is the piece most worth keeping |
| `perception_node` | Separate process. HSV bands per colour, size-versus-distance gate, ground-plane back-projection, one pose topic per colour | `/block_pose/<color>` (`PoseStamped`) | `cv2`, `cv_bridge`, camera topics | **Low structurally, medium in practice.** Any detector publishing that topic works; the bands themselves are scene-specific |
| `config/named_poses.yaml` | Room name to map pose, overridable by `NL2PLAN_POSES_FILE` | YAML | — | **Trivial.** The only part of the environment that is genuinely config today |

## 4. The software safety contract

This section covers *software* safety: what the tool layer refuses to do. It is
not the physical-safety question — whether the robot obeys real dynamics and
would stay upright on hardware — which is section 5 and is a much harder
problem. The two are separate, and I originally conflated them.

The system already refuses to do a set of specific unsafe things. Until now
those refusals existed only as scattered conditionals with comments explaining
which live run motivated them. Writing them down as a numbered contract is the
point of this section, and it is the step that has to come before reasoning
formally about any of them: nothing can be proved about a system whose
properties have never been stated.

I want to be exact about the epistemic status here. **These are established by
code inspection and by live runs that exercised some of them. They are not
proved, and the test suite does not yet try to violate them.** Every "how it
holds" entry below should be read as "this is where the check is written", not
"this has been shown to be unbypassable".

| # | Invariant | Enforced at | What goes wrong without it |
| --- | --- | --- | --- |
| I1 | No tool executes on arguments that fail their JSON Schema | `tools.py:208` `_coerce_args`, checked in `call` before dispatch | A 7B model emits malformed or invented arguments regularly; unchecked, they reach a motion command |
| I2 | An exception inside a tool becomes a structured error to the model, never a crash | `tools.py:201` | One bad call ends the mission and leaves the robot wherever it stood |
| I3 | `pick` is refused while already holding something | `backend.py:86` | The pin is overwritten; the first block is dropped silently in place |
| I4 | `pick` is refused unless `object_id` matches the last confirmed detection | `backend.py:89`, message from `logic.pick_error` | The model invents an id and the grasp fires on a stale or imaginary target |
| I5 | `pick` is refused if the robot has moved beyond `PICK_RANGE` (1.3 m) since the sighting | `backend.py:96` | Grasping on a coordinate measured from a pose the robot has left |
| I6 | The final approach is planned by Nav2, not creeped blind, whenever the block is beyond the 0.6 m standoff | `backend.py:113`, `logic.standoff_pose` | A blind creep drove into a stool that sat between robot and block |
| I7 | No grasp fires without a fresh dead-ahead re-confirmation at the standoff | `backend.py:125`, `perception.confirm_here` | Search-scan sightings carry roughly 0.2 m of oblique-angle error; one was 0.55 m off truth. The pin would teleport the block across visible floor |
| I8 | No grasp fires if the creep stalled against something short of `GRASP_REACH` | `backend.py:155`, `manipulation.align(stall_is_error=True)` | Same teleport, this time across the obstacle the robot is pressed against |
| I9 | The final creep is aimed at the sighting re-anchored through the robot's *current* pose, not at the stored map coordinate | `backend.py:146`, `logic.from_robot_frame` | A localization correction arriving mid-approach (24 degrees of AMCL yaw error was measured at one pose) aims the creep at floor beside the block |
| I10 | A sighting is confirmed only from four stationary samples clustering within 0.2 m **in the robot's frame** | `perception.py:94` | Heading error smears map-frame samples of a parked block past the gate, reporting "not visible" on a block in plain view |
| I11 | A confirmed cluster must sit within 0.45–1.30 m of the robot | `perception.py:100` | Elevated red decals break the ground-plane projection and confirm as blocks |
| I12 | A brown cluster coinciding with a live orange sighting is rejected | `perception.py:105` `_stolen_by_orange` | Orange's antialiased rim sheds pixels into every workable brown band; the robot drives to the wrong block |
| I13 | `place` is refused when not holding anything | `backend.py:171` | An empty release sequence reported as success |
| I14 | The grasp pin teleports the block relative to the **robot model**, never the map frame | `node.py:100`, `set_entity_rel` | AMCL error bakes into the block's true world position, corrupting the scene |
| I15 | The pin stops before the release teleport, not after | `manipulation.py:145` `detach` | A racing 20 Hz pin tick overwrites the release and the block returns to the gripper |
| I16 | Every mission terminates: hard step cap and wall-clock cap | `agent.py:93`, `agent.py:94` | A model that loops on a failing tool never stops |

I3, I4, I5, I13 and I16 are properties of the tool state machine alone. They
depend on no geometry, no sensor, and no simulator, so a model checker could
take them without needing a physics model — though it would then be proving
things about the *abstraction*, not about the robot. I6 through I12, I14 and
I15 all quantify over sensor error and are harder. None of them say anything
about whether the machine stays upright.

## 5. Physical validity — the harder question

Everything above is about software behaviour. A separate question, and the one
the research agenda actually asks, is whether the simulation reproduces real
dynamics well enough that code validated here would behave the same on physical
hardware — and specifically whether the robot would stay upright.

The current answer is no, and for a structural reason rather than a tuning one.

**The base cannot tip over in this simulation, because tipping is excluded from
the physics.** The chassis is driven by `libgazebo_ros_planar_move.so`
(`mobile_arm.urdf.xacro:420`), a plugin that *sets* the base's planar velocity
directly rather than applying wheel torques. Reinforcing that, the four wheels
are `type="fixed"` joints (`:96`) and so never rotate, and their contact
friction is explicitly zeroed with `mu1=0, mu2=0` (`:104`) — the model comments
record that friction was otherwise braking the plugin, costing 65–90% of the
commanded speed. The consequence is that there are no wheel normal forces, no
lateral load transfer, and no traction limit anywhere in the model, and the base
is constrained to the ground plane by construction. Running this simulation any
number of times therefore yields no evidence at all about tip-over. Absence of
falling here is not a result; it is an assumption of the model.

**The mass properties are synthetic.** Every inertia comes from the
`box_inertia` and `cylinder_inertia` macros (`:29`, `:38`), which compute
uniform-density solid inertia from the link's bounding geometry. The chassis is
a 5 kg uniform box. Nothing in the model represents motors, battery, compute, or
wiring — which on a real platform dominate both total mass and centre-of-mass
placement. The joint `effort="20"` limits are identical on shoulder, elbow and
wrist, which is a placeholder rather than an actuator specification.

**The mass is not where the shape is.** None of the fifteen links gives its
`<inertial>` block an `<origin>`, so by the URDF spec each link's centre of mass
sits at its link frame. The geometry does not: `base_link`'s box is drawn
centred 0.04 m above its frame, and every arm link's shape is centred partway
along its length. Gazebo therefore simulates mass concentrated at the joints
rather than distributed along the limbs, and the inertia tensors compound it —
each is computed about its own shape's centroid and then applied at the link
origin with no parallel-axis shift. The simulated mass distribution is
internally inconsistent with the drawn robot, and it biases toward stability.

The static analysis below is computed both ways: mass at the link frame (what
Gazebo simulates) and mass at the shape centroid (what the robot appears to be).
Centre of mass is taken against the wheel support polygon, ±0.14 m fore-aft by
±0.165 m lateral, across the five arm poses in `manipulation.py` while carrying
the 50 g block. `a_tip` is the horizontal acceleration that would tip the robot;
the last column is how many times Nav2's commanded limit that is.

| Arm pose | Margin (m) | CoM height (m) | a_tip (m/s²) | × DWB limit |
| --- | --- | --- | --- | --- |
| | *Gazebo / geometric* | *Gazebo / geometric* | *Gazebo / geometric* | *geometric* |
| REST | 0.131 / 0.131 | 0.087 / 0.119 | 14.8 / 10.8 | 4.3 |
| PRE_GRASP | 0.123 / 0.120 | 0.080 / 0.110 | 15.0 / 10.7 | 4.3 |
| GRASP | 0.123 / 0.120 | 0.077 / 0.106 | 15.7 / 11.2 | 4.5 |
| LIFT | 0.127 / 0.125 | 0.087 / 0.119 | 14.3 / 10.3 | 4.1 |
| DROP | 0.119 / 0.116 | 0.079 / 0.109 | 14.8 / 10.4 | 4.2 |
| **worst of all configurations** | 0.122 / 0.119 | 0.086 / 0.118 | **14.0 / 9.9** | **4.0** |

The last row sweeps all four arm joints across their full limits rather than
only the five commanded poses. It lands at `shoulder_lift = +0.78 rad` with the
rest at zero, and is only 4% worse than DROP — the arm is roughly one seventh of
the robot's 7.39 kg, so arm posture barely moves the centre of mass. The chassis
dominates. Nav2's DWB is configured at `acc_lim_x/y: 2.5` m/s²
(`nav2_params.yaml:160`), so the worst reachable configuration still has four
times the headroom it needs.

Two caveats on that comfort. First, it is a statement about a 5 kg uniform box
carrying a 1 kg arm, not about hardware: the components that would move a real
centre of mass are absent from the model, and a real 4-DOF arm with servos over
a 0.35 m reach could plausibly be three times the modelled arm mass. Second, and
because the chassis dominates, the result is *insensitive* to the thing that
varies during a mission and *highly sensitive* to the thing that is synthetic.
The analysis method is sound; the inputs are not yet real.

The figures come from `analysis/envelope.py` in the `mobile-arm-dynamics`
repository, which reads this model's URDF directly. An earlier draft of this
section quoted roughly 19 m/s² by measuring centre-of-mass height from
`base_link` rather than from the ground plane one wheel radius below it; the
tooling is what caught it.

Worth noting separately that the arm *is* dynamically simulated — revolute
joints with damping and friction under `gazebo_ros2_control` — so it is only the
base that is kinematic. And because the grasp is a teleport pin rather than a
contact grasp, the payload never loads the arm, so the one event where payload
dynamics matter is faked.

What would have to change, in dependency order, before any claim about physical
robots could be supported:

1. **A simulation that can tip.** Continuous wheel joints, a drive plugin that
   applies wheel torques, realistic contact friction. Nothing downstream means
   anything until this holds, and it will disturb the Nav2 tuning and the
   odometry that the current results depend on.
2. **Measured mass properties.** From CAD, or by weighing the platform and
   finding the centre of mass on a tilt table. This is the gate on sim-to-real:
   step 1 without step 2 simulates a robot that does not exist.
3. **A contact grasp**, so the payload enters the dynamics.
4. **The stability analysis proper.** Static first — centre-of-mass projection
   inside the support polygon over the whole commanded arm configuration space,
   which is analytically provable for a rigid-body model. Then dynamic, via a
   force-angle stability measure or zero-moment-point criterion evaluated over
   the acceleration envelope. The provable statement has the form *for all base
   accelerations within a bound and all arm configurations in a set, the
   stability measure stays positive* — which converts directly into a runtime
   constraint on acceleration and arm extension.
5. **Formal reachability tooling** over the closed loop, if wanted, and only
   after the above.

One structural note. None of this belongs in this repository. NL2Plan is the
task-planning layer; it emits `navigate_to`, `pick` and `place`. Tip-over
stability is a property of the chassis and the low-level controllers, which live
in `mobile_arm_sim`. What NL2Plan would do is *consume* the result: a conclusion
of the form "the arm must be stowed while base speed exceeds v" becomes another
row in the section 4 contract, enforced in the tool layer. The proof work sits
one layer below this repository, and the contract above is the interface through
which its conclusions would arrive.

## 6. Where the modularity does not hold yet

A four-method seam is a syntactic boundary. It says nothing about whether the
two sides share assumptions, and here they share a lot.

**The environment is described in five places, only one of which is config.**
`config/named_poses.yaml` holds the room poses. But `prompt.py` separately
hardcodes the room names *and* a block-to-room map as a hard rule;
`logic.COLOR_ENTITIES` hardcodes the four Gazebo entity names;
`manipulation.TABLE_XY` hardcodes the drop table's coordinates; and
`MockBackend.MockWorld` restates rooms, poses and block positions again in
Python. Adding a room means editing four files, and nothing detects
disagreement between them.

**`MockBackend` is a hand-written twin.** Its value depends entirely on it
behaving like the live backend, and that correspondence is maintained by me
noticing. The error strings are duplicated by hand with a comment asking that
they be kept identical. It has already diverged in one visible way: it accepts
any `object_id` beginning with `obj_`, where the live backend requires a match
against the last confirmed detection. The mock therefore enforces a weaker I4
than the system it stands in for, and a test written against it would pass
without exercising the real check.

**The perception constants are measurements, not parameters.** The HSV bands are
masked-pixel percentiles taken under one renderer's lighting, and the comments
in `color_block_detector.py` record how narrowly orange and brown separate:
hue ~13–17 versus ~13, distinguished only by saturation, with wooden furniture
and the wood floor sitting immediately below the brown floor value. That
calibration is not a parameter that transfers; it is a fact about one scene. The
same is true of `CONFIRM_DIST_MAX`, which was widened to 1.30 m specifically
because Nav2's goal tolerance plus AMCL error routinely parks the robot 0.95 to
1.19 m from blocks that are nominally 0.7 m from the search pose.

**The manipulation constants are facts about one robot.** The four joint targets
were measured on `mobile_arm_sim` and carry a comment telling me not to retune
them here. `GRASP_REACH` is where those fixed poses happen to reach. There is no
kinematic model, so a different arm invalidates all of it — this is what the
planned MoveIt2 upgrade would fix.

**The known bugs are boundary bugs.** This is the part a modularity claim should
be judged on, and it is not flattering. Of the six issues in
[known_issues.md](known_issues.md), three — #1, #3 and #4 — are the same species:
a coordinate produced under one pose estimate being consumed under another,
across the perception/manipulation boundary. #1 is the sharpest: `align`'s final
heading trim re-reads AMCL and recomputes a bearing to a map coordinate that was
derived from an *earlier* AMCL fix (`manipulation.py:120`), which is precisely
the mistake I9 exists to prevent, surviving in the one place that was not
audited. The frame in which a position is expressed is load-bearing information
that the interfaces between these modules do not carry. A typed observation —
value plus frame plus timestamp — instead of a bare `(x, y)` tuple would make
all three of those bugs unrepresentable.

**The agent holds no world model.** Cross-call state is two fields on
`RosBackend`: what is held, and the last detection. Everything else the system
"knows" lives in the LLM's conversation history. This is why known issue #5
exists — after `place` moves a block, the prompt still asserts its original
room. It works, and it keeps the code small, but it means there is no queryable
state to write invariants against beyond the two fields above.

## 7. What transfer to another environment would cost

Splitting section 6 by what actually changes gives two packs. Nothing in the
current code has this shape; describing it is the proposal.

**An environment pack** — everything that is a fact about the building and the
objects in it:

| Item | Where it lives now | Should be |
| --- | --- | --- |
| Room name to map pose | `config/named_poses.yaml` | already config |
| Room names in the prompt | hardcoded in `prompt.py` | generated from the pose file |
| Object-to-room semantic map | hardcoded in `prompt.py` | environment pack, marked as *initial* locations (see known issue #5) |
| Object name to simulator entity | `logic.COLOR_ENTITIES` | environment pack |
| Detector colour calibration | `COLOR_BANDS` in `color_block_detector.py` | environment pack, with the measurement procedure recorded |
| Drop location | `manipulation.TABLE_XY` | environment pack |
| Mock world | `MockWorld` defaults in `tools.py` | generated from the same pack, so the twin cannot drift |

**A robot pack** — everything that is a fact about the machine: the four joint
targets, `GRIPPER_OPEN`/`GRIPPER_CLOSED`, `GRASP_REACH`, `TABLE_REACH`, the
creep and rotation speeds, and the stall-guard thresholds. All of these are
currently module constants in `manipulation.py`.

What is *not* in either pack is the interesting part, because it is what would
actually transfer unchanged: the agent loop and its recovery behaviour, the tool
schemas, the dispatcher, `logic.py` in full, the Nav2 client, the perception
gating *policy* (stationary sampling, robot-frame clustering, distance band,
cross-colour veto), the pick protocol of standoff–reconfirm–creep–refuse, and
the safety contract in section 4. That is the reusable core, and it is most of
the design work.

The honest estimate for a port, then: the structure is right and the seams hold,
but the environment and robot facts are currently spread across six source files
rather than gathered into two packs. Doing that gathering is a bounded, mostly
mechanical piece of work, and it is the concrete next step I would propose
toward the transferability the agenda asks for. Recalibrating a colour detector
for a new scene is the one part that is genuinely empirical and cannot be made
cheap by refactoring.

## 8. Status

What is measured: eight missions across four blocks completed end to end on
2026-07-21 with no failed tool calls, plus one continuous session fetching all
four; the numbers are in the [README](../README.md). The 36-test suite passes
without ROS.

What is claimed but not measured: that the invariants in section 4 hold under
adversarial input. Nothing currently tries to violate them. I1, I2, I3, I13 and
I16 could be tested through `MockBackend` today and are not. I4 is enforced in
the mock only in a weaker form — any `object_id` starting with `obj_` passes —
so a test written against the mock would not exercise the real check. I5, I7 and
I8 have no mock equivalent at all and need either a sim run or a new fake at the
`ros_backend` level. Writing tests that attempt each violation is the obvious
next piece of work on this strand, and it is a precondition for treating any of
this as proved.

What is not attempted: any formal proof. Section 4 states the properties, which
is the step that has to come first, and section 6 states the coupling that a
proof would have to contend with.
