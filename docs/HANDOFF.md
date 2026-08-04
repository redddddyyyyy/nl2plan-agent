# Mobile manipulator — IK / theorem work: handoff

Written 2026-08-04, to allow a cold resume in a different environment.
Everything an assistant needs is in this file; no prior conversation required.

---

## 1. The project

Working under a professor on a **mobile manipulator** — a Hiwonder JetRover
(four-wheel mecanum base plus an arm) doing pick-and-place in a simulated
hospital-like environment. The research agenda calls for **safety proofs
grounded in real physics rather than simulation**; the phrase "digital
simulation contracts" appears, so this is assurance-case work, not a demo.

**Repos**
- `https://github.com/redddddyyyyy/nl2plan-agent` — LLM agent + ROS backend.
  Kinematics at `src/nl2plan_agent/nl2plan_agent/ros_backend/kinematics.py`.
- `https://github.com/redddddyyyyy/mobile-arm-dynamics` — analysis and proofs
  (`analysis/`, `docs/theorem.md`), plus `urdf/mobile_arm_baseline.urdf`.
- `mobile_arm_sim` — the shared simulator repo. **URL not yet provided.**
  Expected at `~/ros2_ws/src/mobile_arm_sim`. Not solely ours, so changes there
  are constrained.

**Best entry point:** `physics_and_geometry_briefing.docx` (also at
`mobile-arm-dynamics/docs/`). It states section by section what is proved, what
is implemented, and what is neither. Read it before the code.

**Context:** the original working machine was lost to a coffee spill. The repos
plus that briefing are all that survived.

---

## 2. The single most important correction

**The inverse kinematics is already written.** It was remembered as "not
implemented completely"; that is out of date. `kinematics.py` has
`forward` / `inverse` / `reachable` / `max_forward_reach`, with joint stops
enforced, refusing rather than clamping. The briefing itself says so (§3, "The
solver is now written").

Do not spend a session writing IK. The real gaps are wiring, rigour, and
redundancy resolution.

---

## 3. Geometry, and the two traps

A turntable (`shoulder_pan`, about +z) carrying a planar three-link chain
(`shoulder_lift`, `elbow`, `wrist`, all about +y).

| Symbol | Meaning | Value |
| --- | --- | --- |
| `p_x`, `p_z` | pan axis fwd of base centre; lift pivot above floor | 0.050, 0.260 m |
| `L1`, `L2`, `L3` | lift→elbow, elbow→wrist, wrist→**pad centres** | 0.150, 0.120, 0.105 m |
| stops | pan, lift, elbow, wrist — **all symmetric** | 3.14, 1.57, 2.5, 1.57 rad |

Confirmed against `mobile_arm_baseline.urdf` joint origins.

**Trap 1.** The 0.050 m arm-base offset applies **before** the pan joint:
`pan = atan2(y, x - 0.050)`, not `atan2(y, x)` with 0.050 off the radius. They
agree only on the centre line — which is where every approach puts the block,
so the error stays invisible until something reaches off-axis.

**Trap 2.** Links run along their own **+z**, so at all-zero the arm points
straight **up**. Sine and cosine come out swapped from the textbook planar form,
and gripper pitch is `phi = pi/2 - (lift + elbow + wrist)`.

Both were wrong in the first draft of the briefing.

Using cumulative angles `a = lift`, `b = lift+elbow`, `c = lift+elbow+wrist`
(the three links' angles from vertical) makes the algebra short:

    rho  = L1 sin a + L2 sin b + L3 sin c        (radius from pan axis)
    zeta = L1 cos a + L2 cos b + L3 cos c        (height above pivot)
    pads = (p_x + rho cos pan, rho sin pan, p_z + zeta)

---

## 4. Results proved and tested (2026-08-03/04)

### Theorem 3 — the floor bound (tight)

For **every** configuration in the joint box, uniformly in x, y and approach
pitch:

    z >= p_z + L1*cos(lift_limit) - L2 - L3 = 0.0351194 m

*Proof.* `|a| <= lift_limit = 1.57 < pi/2`, so `cos a >= cos(1.57) > 0`;
`cos b >= -1`, `cos c >= -1`; sum with positive coefficients. **Attained** at
`(pan, 1.57, pi-1.57, 0)` — pads at x = 0.200, pointing straight down.

A floor block centre sits at 0.025 m, so it misses by **10.1 mm**.

*Corollary:* no value of `GRASP_REACH` could ever have worked. The bound is a
statement about `z`; `GRASP_REACH` is an `x`. The object had to change, or the
model. The obstruction is the **lift stop**, not link length.

This replaced a 100 × 91 grid scan.

### Theorem 2 — completeness (conditional)

`inverse` tries two elbow branches at one pan angle. There are up to **four**
spellings, related by the mirror map

    M(pan, lift, elbow, wrist) = (pan ± pi, -lift, -elbow, -wrist)

which negates the planar radius and so lands the pads in the same place.
Testing two is exhaustive anyway — **but only because the stops are symmetric.**
M negates the three pitch joints, and symmetric ranges are closed under
negation; the pan is the only coordinate where legality differs, and for a
front target the mirror always carries the larger `|pan|`.

**This will break on hardware.** A real shoulder is not symmetric. Concrete
witness, with a `[-0.3, 1.57]` shoulder: `q = (1.0, 1.2, -2.5, -0.27)` is
strikeable, but the pads land *behind* the pan axis, so the only spelling
returned needs `lift = -1.2` and is refused. The solver reports a target
unreachable while the arm is standing in it. Fix when that day comes: add the
mirrored pan as a third and fourth branch.

Verified: 300k random configurations, 79 refusals, **all** of them the known
artefact that the URDF's pan stop is 3.14 rather than pi (out of range by five
hundredths of a degree). Zero genuine incompleteness.

### Redundancy — implemented 2026-08-04, and certified rather than sampled

Three planar joints, two positional DOF; fixing the approach pitch removes the
redundancy. Nothing used to *choose* it — every call site hand-picked a value.
Now `ros_backend/redundancy.py` does.

`Phi(x,y,z) = { phi : inverse(...) is not None }`, stop margin
`m(q) = min_i (limit_i - |q_i|)` over the three pitch joints, and
`phi* = argmax_{phi in Phi} m(q(phi))` over **both elbow branches**, so the
criterion settles the discrete freedom too.

Computed by **interval branch-and-bound over phi**, not by sweeping it: a
sub-interval whose margin enclosure is wholly negative is discarded along with
every pitch inside it. Output is `Phi_inner ⊆ Phi ⊆ Phi_outer` plus a certified
bound on how much better any other pitch could be. Every *candidate* still goes
through `kinematics.inverse` and its own limit test, so the search can only
exclude, never invent.

At the live grasp target (30×30×150 mm bar, pads gripping at z = 0.075), over
the **whole turn** of pitch:

| Target x | `Phi` (certified) | `phi*` | margin |
| --- | --- | --- | --- |
| 0.240 | [−113.02°, −42.59°] | −78.65° | 0.2388 rad |
| 0.262 | [−103.56°, −39.41°] | −73.18° | **0.2031 rad** |
| 0.280 | [−94.91°, −38.35°] | −68.62° | 0.1634 rad |
| 0.300 | [−83.87°, −39.37°] | −63.37° | 0.1069 rad |
| 0.320 | [−69.18°, −44.92°] | −57.74° | 0.0343 rad |

Old 50 mm cube at z = 0.050: 0.0409 rad at x = 0.262, and 0.0012 rad at 0.280 —
pinned against the stop, with a 4.7° pitch window. So changing the object bought
roughly **5× the joint margin** and widened the window from a sliver to ~57°.
`shoulder_lift` binds in every case — the same stop that makes the floor
unreachable.

**The old table was wrong by 23°, and the branch-and-bound is what found it.**
The 1° sweep ran over [−90°, 0°] and reported `−90°` as the lower end of `Phi`
for every row. That was the loop bound, not the arm: the arm reaches well past
straight down. The upper endpoints were genuine, and so were all the `phi*`
values — the optimum already sat inside the swept band. That combination is the
nastiest possible: the number anyone would have checked against the robot was
correct, so the error had nowhere to surface.

Restricting to approaches from above is still available as
`domain=red.FROM_ABOVE`, and reproduces the old endpoints exactly. The point is
that it is now a named argument rather than a `range()`.

---

## 5. The rigour question, and what was decided

Asked what "theorem proving" should mean here. Position taken:

- **Not** Coq/Lean/Isabelle. Real-trigonometry goals are the most painful class
  to mechanise, for a claim a reviewer accepts from four lines of algebra — and
  it would prove things about a URDF modelling an unmeasured robot.
- **Model checking is cheap, real, and orthogonal.** The briefing's §6 already
  isolates software guards 3, 4, 5, 13, 16 as pure state-machine properties.
  Separate workstream; proves nothing about the arm.
- **The actual gap is sampling presented as proof.** Grid scans are the same
  epistemic object as a simulator run, and the document's whole stance is that a
  simulator cannot *establish* anything. That is the criticism to expect.
- **So:** prove analytically where possible; elsewhere replace grid sampling
  with **interval branch-and-bound over the joint box** — exhaustive by
  construction, ~60 lines, days not months.

---

## 6. Deliverables

Everything now lives in the **repo working trees** under
`C:\Users\Sadha\robot-work\repos\`, not loose in a folder. New code:

- **`ros_backend/interval.py`** — interval arithmetic: enclosures for `+ - * /`,
  `sqr`, `sqrt`, `sin`, `cos`, `acos`, `atan2`, and `wrapped_abs` (the
  distance-to-a-multiple-of-2π that every joint stop is tested against).
  Written for `redundancy.py`, kept general because it is what step 5 needs.
- **`ros_backend/redundancy.py`** — `feasible_pitches()` (`Phi`), `resolve()`
  (`phi*`, returning the configuration to command), `approach_pitch()`,
  and the `FULL_TURN` / `FROM_ABOVE` domains.
- **`kinematics.py`** — two small additions: `branch_solutions()` exposes both
  elbow spellings before the stops are applied, and `inverse()` is now a filter
  over it, so the search and the solver cannot drift apart. Plus
  `ArmGeometry.stop_margin()` / `.pan_margin()`.
- **`tests/test_interval.py`** (22 tests) — soundness by randomised containment,
  plus the two `atan2` branch-cut regressions by name.
- **`tests/test_redundancy.py`** (46 tests) — `Phi` inner bound checked against
  the solver, outer bound against a dense sweep, `phi*` against sampled margins,
  and the −90° artefact pinned as a regression.
- **`docs/ik_theory_section.md`** — the writeup, now **in both repos'** `docs/`.
  §3.7 rewritten around the certified computation and the 23° correction; §3.8
  moves `Phi` out of the sampled list and states the libm assumption; §3.9's
  status table brought up to date.
- **`HANDOFF.md`** — this file.

> **Still not committed and still not pushed.** There is no git on this machine;
> the repos are unzipped zipballs. Verified 2026-08-04: GitHub
> `redddddyyyyy/nl2plan-agent` is still at `797b35f` (2026-07-30) and its
> `tests/test_kinematics.py` is still the old 9,832-byte version. Getting all of
> the above upstream remains the highest-value next action, and it is now a
> larger diff than it was.

---

## 7. Environment (Windows 11 box)

- Python 3.12.10, installed via winget, at
  `%LOCALAPPDATA%\Programs\Python\Python312\python.exe`. **Not on PATH** —
  invoke by full path, or add `...\Python312` and `...\Python312\Scripts`.
- Installed: pytest, numpy, pyyaml, jsonschema, opencv-python-headless, and the
  repo's `requirements.txt`.
- **No git. No ROS.** Repos fetched from `codeload.github.com/<owner>/<repo>/zip/refs/heads/main`.
- **Everything lives in `C:\Users\Sadha\robot-work\`** — repos, the writeup, the
  briefing, the zipballs. It was previously in a per-session temp scratchpad
  under `%LOCALAPPDATA%\Temp\claude\...`, which is a directory both Windows and
  the tooling treat as disposable; a cleanup would have taken the only copy of
  the work with it. Moved 2026-08-04. Do not work out of the scratchpad again.
- Test status: `nl2plan-agent` **641 passed / 129 skipped** (excluding
  `test_pick_approach.py`), up from 573 — the 68 new tests, nothing broken;
  `mobile-arm-dynamics` 44 passed.
- `resolve()` costs ~300 ms at its default 1e-6 rad tolerance, ~5 ms at 1e-4.
  Certifying a smooth maximum to a gap of `t` needs `O(1/sqrt(t))` boxes, so
  asking for more precision gets expensive fast and buys nothing physical.

---

## 8. Next steps, in order

1. ~~Replace the grid-scan floor test with the theorem.~~ **Done.**
2. ~~Add a completeness test pinning the symmetric-stops hypothesis.~~ **Done.**
3. ~~Implement `Phi` and margin-optimal `phi*`.~~ **Done 2026-08-04**, by
   interval branch-and-bound rather than by grid — which is what caught the 23°
   error in §4 above.
4. **Wire `redundancy.resolve()` into `pick()`** — today `pick()` calls
   `manipulation.grasp_sequence()`, which replays fixed PRE_GRASP/GRASP poses,
   so the arm has no reachability check at all. Only `DROP` is IK-derived, baked
   in offline. Needs a simulator to test properly.
   **Command `resolve().joints` directly.** Do not take `resolve().pitch` and
   feed it back through `inverse()`: `inverse` prefers elbow-up whenever
   elbow-up fits, which would silently discard the branch the criterion chose.
   `Approach.agrees_with_inverse` reports whether they happen to coincide (at
   the live grasp targets they do).
5. **Replace the remaining scans with interval branch-and-bound** over the joint
   box — `max_forward_reach()`'s bisection, which assumes without proof that the
   reachable set is an interval in `x`, and the reach-versus-height table's 1°
   pitch sweep. `interval.py` is written and tested, so this is now mostly
   plumbing.

### Defects found by running the suite, not yet fixed

- `tests/test_pick_approach.py` hard-fails at *collection* without ROS
  (`geometry_msgs`), so pytest exits 2 and runs **nothing**. On a clean machine
  the whole suite looks broken. Needs `pytest.importorskip("geometry_msgs")`.
- `cv2` missing from `requirements.txt` though two test modules need it.

### Cheap win not yet taken

Point `test_kinematics_urdf.py` at the checked-in
`mobile-arm-dynamics/urdf/mobile_arm_baseline.urdf`. Today it skips wherever the
simulator is absent, so "the constants match the URDF" is never actually checked.

### On building the simulator

Advised **against** doing it before the meeting: `mobile_arm_sim` is not in
hand, the machine is Windows (WSL2/Docker plus display and GPU passthrough), and
it adds nothing to a theory deliverable — the project's own thesis is that the
simulator cannot establish these results, and Theorem 3 holds against the URDF
Gazebo loads. The sim worth building is the one the briefing lists as missing:
*"a simulation that can tip"*, with real wheel joints, a torque-applying drive
plugin and contact friction, built to **attempt falsification** of the tip-over
argument. A week's research task, and a better thing to propose than half-build.

---

## 9. Open question

**The `mobile_arm_sim` repo URL was asked for and never supplied.** It would let
the URDF cross-check run against the real model rather than the baseline copy.

---

## 10. To resume in a fresh session

Paste this:

> I'm working under a professor on a mobile manipulator (JetRover, mecanum base
> + arm) doing pick-and-place, with safety proofs grounded in physics rather
> than simulation. Read `C:\Users\Sadha\robot-work\HANDOFF.md` — it has the full
> state, the geometry, the theorems proved, and the ordered next steps.
> Then pick up at step 4.

Everything is under `C:\Users\Sadha\robot-work\`: `HANDOFF.md`,
`ik_theory_section.md`, `physics_and_geometry_briefing.docx`, and `repos\`.

Two things that have each cost a session:

- **The IK is already written**, in `kinematics.py`. So is the redundancy
  resolution, in `redundancy.py`. Don't rewrite either.
- Python is at `%LOCALAPPDATA%\Programs\Python\Python312\python.exe` and is
  **not on PATH**. No git, no ROS. Run the suite from
  `repos\nl2plan-agent-main\` with `-m pytest tests/ --ignore=tests/test_pick_approach.py`.
