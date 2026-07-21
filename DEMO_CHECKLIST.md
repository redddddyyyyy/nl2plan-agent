# Demo checklist — Cotter meeting, Wed 2026-07-22

Code is FROZEN as of 2026-07-21 evening (35/35 tests). Demo format: **one
mission per take, reset the scene between takes.** Back-to-back missions
without a reset eject previously-placed blocks from the table drop zone —
that is the one known way to scramble a take.

## Bring-up (10 min before)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/NL2plan\ agent/install/setup.bash

ros2 launch perception_node nl2plan_sim.launch.py
# in a second terminal (same sources):
ros2 lifecycle get /bt_navigator     # wait for: active [3]
```

Known launch flake: if bt_navigator never goes active or the detector spams
map-TF warnings, Ctrl-C and relaunch. Don't debug it.

## Per take

```bash
timeout 10 ros2 run perception_node scene_setup   # blocks -> home spots
nl2plan --interactive
```

One command on camera, cut after "Task completed". Then quit / Ctrl-C the
session and reset again before the next take. Never ask for a block that is
already on the table without resetting first.

## Known-good commands (all passed from a clean scene on 2026-07-21)

- "pick up the red block and place it on table"       (bedroom, ~4 steps)
- "pick up the magenta block and place it on table"   (gym, ~3 steps)
- "pick up the orange block and place it on table"    (lounge, ~3 steps)
- "pick up the brown block and place it on table"     (behind the sofa, ~8 steps)
- follow-up in the same session: "where did you find each block?" — answered
  from conversation memory, no driving. Good closer.

## If a take goes sideways

Quit the session, run the reset, relaunch `nl2plan --interactive` (fresh
conversation), go again. A mission that wanders and recovers is honest
behavior — keep or cut in editing, your call. Never run two agent sessions
or any background script at the same time: one robot, one Nav2 stack.

## If asked how it knows where blocks are

Task planner is given a semantic map (block -> room, in the system prompt);
perception does the real local search: stop-and-spin scan, HSV detection with
a size-distance gate, four sightings clustered under 0.2 m in the robot frame
before anything counts, re-confirm at 0.6 m before grasping — and the pick
REFUSES rather than grasping anything it can't re-confirm. Random-spawn +
sweep-search mode is the planned next step (scene_setup is the seed).
