# Arm kinematics: the closed form, and what it proves

Rajeev Reddy — draft replacement for §3 of *Physics and geometry of the mobile
manipulator*

This section states the arm's forward and inverse kinematics as results with
proofs, rather than as code that happens to work. The motivation is narrow and
specific: the previous draft established its central claim — that the arm
cannot reach a block on the floor — by scanning a grid of 100 positions by 91
approach angles. That is a finite probe of a continuum. It is the same
epistemic object as a simulation run, and this document elsewhere declines to
accept simulation runs as establishment. The claim deserves better, and as it
turns out it falls to four lines of algebra.

Where algebra does not reach, the replacement is interval branch-and-bound
rather than a finer grid — exhaustive over a continuum by construction. §3.7 is
the first result computed that way, and doing so immediately corrected a 23°
error that the grid it replaced had concealed for as long as the table existed.

Everything numerical below has been reproduced from an independent
reimplementation built from the URDF joint origins, sharing no code with
`ros_backend/kinematics.py`.

---

## 3.1 Model and conventions

The arm is a turntable carrying a planar three-link chain. Reading the joint
tree out of the URDF: `shoulder_pan` rotates about **+z**; `shoulder_lift`,
`elbow` and `wrist` all rotate about **+y** and are therefore mutually
parallel. Every link runs along its own **+z**. No joint on the chain carries a
rotated mount (all `rpy` are zero).

Those three facts are what license the planar reduction, and all three are
pinned by tests against the URDF rather than assumed.

**Notation.** Joint vector `q = (θ₁, θ₂, θ₃, θ₄)` = (pan, lift, elbow, wrist).
Throughout, work with the *cumulative* angles

    a = θ₂,    b = θ₂ + θ₃,    c = θ₂ + θ₃ + θ₄

which is the substitution that makes the proofs short. Geometrically `a`, `b`,
`c` are the angles of the three links **from vertical**.

**Constants**, transcribed from the URDF joint origins and checked by test:

| Symbol | Meaning | Value |
| --- | --- | --- |
| `p_x` | pan axis, forward of base centre | 0.050 m |
| `p_z` | lift pivot, above the floor | 0.260 m |
| `L₁` | lift pivot → elbow | 0.150 m |
| `L₂` | elbow → wrist | 0.120 m |
| `L₃` | wrist → centre of the finger pads | 0.105 m |
| `Λ₁` | pan stop (symmetric) | 3.14 rad |
| `Λ₂` | lift stop | 1.57 rad |
| `Λ₃` | elbow stop | 2.5 rad |
| `Λ₄` | wrist stop | 1.57 rad |

Write `Q = [−Λ₁,Λ₁] × [−Λ₂,Λ₂] × [−Λ₃,Λ₃] × [−Λ₄,Λ₄]` for the joint box.

`L₃` runs to the **centre of the finger pads**, not the fingertips, because the
pads are where a block is actually held. Fingertips are 0.130 m out.

**Two conventions that invert the textbook form.** Both were wrong in the first
draft of this work, and both fail silently — they return four plausible floats
and aim the arm at empty floor.

1. The 0.050 m arm-base offset is applied **before** the pan joint. The pan axis
   is the vertical line through `x = 0.050`, so `θ₁ = atan2(y, x − 0.050)`, not
   `atan2(y, x)` with 0.050 subtracted from the radius. The two agree only on
   the robot's centre line — which is exactly where every approach puts the
   block, so the error stays invisible until something reaches off-axis.
2. Links run along their own **+z**, so at `q = 0` the arm points straight
   **up**, not straight ahead. Sine and cosine therefore appear swapped
   relative to the standard planar-arm derivation, and gripper pitch is
   `φ = π/2 − c`, not `c`.

---

## 3.2 Forward kinematics

**Proposition 1.** *The centre of the finger pads, in `base_footprint`, is*

    ρ(q) = L₁ sin a + L₂ sin b + L₃ sin c        (radius from the pan axis)
    ζ(q) = L₁ cos a + L₂ cos b + L₃ cos c        (height above the pivot)

    p(q) = ( p_x + ρ cos θ₁ ,  ρ sin θ₁ ,  p_z + ζ )

*and the gripper's approach pitch is `φ = π/2 − c`, measuring 0 as horizontal
and −π/2 as pointing straight down.*

*Proof.* Compose the chain. Each of the three parallel joints contributes a
segment of length `Lᵢ` at cumulative angle from vertical equal to the sum of
the joint angles up to and including it; projecting onto the vertical and the
in-plane horizontal gives the two sums. The pan rotation acts on the resulting
planar point about the axis through `p_x`, which contributes the offset before
the rotation. The gripper's axis is the third link's axis, at angle `c` from
vertical, hence at `π/2 − c` from horizontal. ∎

Sanity check: arm straight out level (`θ₂ = π/2`, `θ₃ = θ₄ = 0`) gives `c = π/2`,
so `φ = 0` and `ρ = L₁+L₂+L₃ = 0.375`, putting the pads at `x = 0.425 m`. This
is the one reach figure nobody disputes and it recovers exactly.

**Sign convention on ρ.** `ρ` is signed: negative means the pads are behind the
pan axis. Such a pose has a second spelling with the pan turned 180° and the
three pitch joints negated (see Lemma 1). The pitch reported for a negative-`ρ`
configuration is that of the second spelling, so that forward and inverse agree
on every pose rather than disagreeing by more than a radian on the ones nobody
checks by eye.

### Where the five fixed poses actually put the gripper

Computed from Proposition 1:

| Pose | `(θ₁,θ₂,θ₃,θ₄)` | x | z | pitch |
| --- | --- | --- | --- | --- |
| REST | (0, −0.5, 1.2, 0.3) | 0.1437 | 0.5402 | +32.7° |
| PRE_GRASP | (0, 0.6, 1.4, 0.5) | 0.3067 | 0.2497 | −53.2° |
| GRASP | (0, 0.9, 1.6, 0.5) | **0.2541** | **0.1532** | −81.9° |
| LIFT | (0, 0, 1.0, 0.3) | 0.2522 | 0.5029 | +15.5° |
| DROP (current) | (0, 0.814, 0.733, 0.547) | 0.3700 | 0.3134 | −30.0° |

Two consequences, both stated in the previous draft and both confirmed here.
A floor block sits at `z = 0.025`; GRASP stops **0.128 m above it** and 0.10 m
short horizontally, so the fixed-pose grasp never arrives and the teleport pin
performs the entire pick. And PRE_GRASP reaches *further forward* than GRASP
does, which makes the "grasp" motion a retract-and-lower rather than a reach.

---

## 3.3 Inverse kinematics: the closed form

Fix a target `(x, y, z)` and an approach pitch `φ`. Then:

    θ₁  = atan2(y, x − p_x)
    ρ   = √((x − p_x)² + y²)
    ζ   = z − p_z

    ρ_w = ρ − L₃ cos φ                 (back off along the approach to the wrist)
    ζ_w = ζ − L₃ sin φ

    cos θ₃ = (ρ_w² + ζ_w² − L₁² − L₂²) / (2 L₁ L₂)
    θ₃  = ± arccos(·)                  (elbow-down / elbow-up)
    θ₂  = atan2(ρ_w, ζ_w) − atan2(L₂ sin θ₃, L₁ + L₂ cos θ₃)
    θ₄  = (π/2 − φ) − θ₂ − θ₃

*Derivation of the middle two lines.* The wrist point must satisfy
`L₁ sin a + L₂ sin b = ρ_w` and `L₁ cos a + L₂ cos b = ζ_w`. Summing squares and
using `cos(b − a) = cos θ₃` gives the cosine rule line directly. For `θ₂`, put
`k₁ = L₁ + L₂ cos θ₃` and `k₂ = L₂ sin θ₃`; expanding the two equations gives

    ρ_w = k₁ sin a + k₂ cos a = r sin(a + ψ)
    ζ_w = k₁ cos a − k₂ sin a = r cos(a + ψ)

with `r = √(k₁² + k₂²)` and `ψ = atan2(k₂, k₁)`. Hence `a + ψ = atan2(ρ_w, ζ_w)`,
which is the stated line. The `atan2` arguments are in the order
`(radial, vertical)` — the swap noted in §3.1. ∎

Note `L₃` and `φ` enter **only** through the wrist point. That is what makes the
problem closed-form: fixing the pitch decouples the wrist from the two proximal
links entirely.

---

## 3.4 Solution structure and completeness

**Lemma 1 (the mirror map).** *Let `M(θ₁,θ₂,θ₃,θ₄) = (θ₁ ± π, −θ₂, −θ₃, −θ₄)`.
Then `M` preserves the pad position and the reported approach pitch.*

*Proof.* Negating all three pitch joints negates `a`, `b`, `c`, hence `ρ ↦ −ρ`
(sine is odd) and `ζ ↦ ζ` (cosine is even). The height is unchanged. In the
plane, `p_x + (−ρ)cos(θ₁ ± π) = p_x + ρ cos θ₁` and likewise for `y`, so the
position is unchanged. The pitch is preserved by the negative-`ρ` convention of
§3.2. ∎

**Theorem 1 (solution count).** *For a given target and pitch there are at most
four joint vectors realising it: two elbow branches, each in two pan spellings
related by `M`.*

*Proof.* `θ₁` is determined up to the `M`-flip. Given the pan spelling, the
wrist point is determined by the target and `φ`. `cos θ₃` is then determined, so
`θ₃` takes at most the two values `±arccos(·)` (one, if the argument is ±1). Each
value of `θ₃` determines `θ₂` uniquely by the `atan2` expression, and then `θ₄`
uniquely. ∎

The solver tests only the two elbow branches at the canonical pan. That is
nevertheless exhaustive:

**Theorem 2 (completeness).** *Suppose every pitch-joint stop is symmetric about
zero. If a target and pitch are realisable anywhere in `Q`, they are realisable
by one of the two branches the solver tests. Hence a refusal is a proof of
unreachability at that pitch, not a failure to search.*

*Proof.* By Theorem 1 the only alternatives to the two tested branches are their
images under `M`. `M` negates `θ₂, θ₃, θ₄`, and each of their admissible ranges
`[−Λᵢ, Λᵢ]` is symmetric about zero, so `M` preserves membership on those three
coordinates. It remains to check the pan. For a target ahead of the pan axis
(`x > p_x`) the canonical `θ₁` lies in `(−π/2, π/2)`, while its mirror lies
outside it in magnitude; so if the mirror satisfies the pan stop, so does the
canonical spelling. Therefore the mirror is admissible only when the canonical
one already is, and testing the canonical two suffices. ∎

**The hypothesis is doing real work, and it will not survive contact with
hardware.** Completeness here is a *consequence of the URDF's symmetric stops*,
not an intrinsic property of the algorithm. A real JetRover with, say, a
shoulder travelling +90°/−30° breaks the lemma immediately, and the solver
would then silently refuse targets it can actually reach. Adding the mirrored-pan
branch is a five-line change and should be made **before** any hardware trial,
not after one produces a confusing refusal.

*Verification.* 300,000 random configurations drawn from `Q` were mapped through
forward kinematics and fed back to the solver. 79 were refused. All 79 were the
known artefact that the URDF's pan stop is 3.14 rather than π, so a target
directly behind the axis needs 3.14159 and is genuinely out of range by five
hundredths of a degree. Zero refusals came from anything else.

**Corollary (refusal is meaningful).** Because the solver is complete and does
not clamp, `reachable()` is a decision procedure at fixed pitch. This is what
would let the arm decline an impossible target instead of reaching confidently
for it — a capability the robot does not have today, since `pick()` never calls
the solver.

---

## 3.5 The main result: the arm cannot reach the floor

The previous draft established this by scanning a grid. It does not need one.

**Theorem 3 (floor bound).** *For every `q ∈ Q` — every pan, every approach
pitch, every position in the workspace —*

    z(q)  ≥  p_z + L₁ cos Λ₂ − L₂ − L₃  =  0.0351194 m

*and the bound is attained.*

*Proof.* By Proposition 1, `z = p_z + L₁ cos a + L₂ cos b + L₃ cos c`. The lift
stop gives `|a| = |θ₂| ≤ Λ₂ = 1.57 < π/2`; cosine is even and decreasing on
`[0, π/2]`, so `cos a ≥ cos Λ₂ > 0`. Trivially `cos b ≥ −1` and `cos c ≥ −1`.
The coefficients `L₁, L₂, L₃` are positive, so the bound follows by summing.

For attainment, take `a = Λ₂ = 1.57`, `b = π`, `c = π`. This requires
`θ₂ = 1.57 ≤ Λ₂` ✓, `θ₃ = b − a = π − 1.57 ≈ 1.5716 ≤ Λ₃ = 2.5` ✓, and
`θ₄ = c − b = 0 ≤ Λ₄` ✓. So `q* = (θ₁, 1.57, π − 1.57, 0)` lies in `Q` and
achieves the bound, for any pan. ∎

**Corollary 3.1.** A block centre at `z = 0.025 m` is outside the workspace, by
**10.1 mm**.

**Corollary 3.2 (no reach constant could have worked).** The bound is a
statement about `z` alone, holding *uniformly in x, y and φ*. Lowering
`GRASP_REACH`, which is an `x`, cannot affect it. There is no value that
constant could have taken. The object had to change, or the model.

Three remarks, because the theorem is easy to over- or under-read.

*On tightness.* The proof relaxes the coupling between the links — it minimises
each cosine independently, ignoring that `b` and `c` are constrained relative to
`a`. Such a relaxation normally gives a bound strictly below the true minimum.
Here it does not, because the relaxed optimum happens to be feasible. That is a
piece of luck worth pointing at rather than hiding: it is why the bound is exact
and not merely valid.

*On the odd numeral.* Were the lift stop exactly `π/2`, the bound would be
`0.260 − 0.225 = 0.035 m` exactly. The extra 0.119 mm is `L₁ cos(1.57)` — pure
artefact of the URDF rounding `π/2` to 1.57. The physically meaningful statement
is "the pads bottom out 35 mm above the floor, at `x = 0.200 m`, pointing
straight down."

*On what binds.* The obstruction is the **lift stop**, not link length. The
kinematic maximum `L₁+L₂+L₃ = 0.375 m` is nowhere near binding. `shoulder_lift`
stops at horizontal, and the elbow and wrist together cannot make up the
remaining drop. The remedies are therefore a taller object, a lift stop past
1.57, or a longer forearm — all model changes, none of them software.

*Numerical check.* 200,000 random configurations from `Q`; lowest `z` observed
was 0.036079 m, never below the bound. The solver refuses `z = 0.035` at every
pitch and admits `z = 0.036`, bracketing the bound to within a millimetre.

---

## 3.6 Reachability: the annulus is necessary, not sufficient

The wrist point must lie in the annulus the two proximal links span:

    |L₁ − L₂|  ≤  √(ρ_w² + ζ_w²)  ≤  L₁ + L₂
       0.030 m ≤        ·          ≤  0.270 m

**This is exactly the condition `|cos θ₃| ≤ 1`**, so it is necessary and it is
free. It is **not sufficient**, and the gap is the whole of §3.5: every
floor-level target sits comfortably inside this annulus and is still
unreachable, because the solution the annulus admits needs `θ₂` past its stop.

An annulus test alone waves through targets the arm cannot bend to. It looks
rigorous and is not. It is the check this project originally used, and it is
the reason the reach constant went unchallenged for so long. **A reachability
check must solve for the angles and test them against the stops** — which is
what `reachable()` now does, and why it is defined as "`inverse` returned
something" rather than as an independent test that could drift out of agreement
with it.

---

## 3.7 The redundancy, resolved

Three joints act in the plane; a position target constrains two degrees of
freedom. The arm is therefore **redundant by one**, and fixing the approach
pitch `φ` is what removes the redundancy and makes the solution closed-form.

Nothing in the system used to *choose* `φ`: every call site supplied a
hand-picked value, so the arm's last free parameter was set on no recorded
basis and could not be defended or audited. It is now chosen, by the criterion
below, in `ros_backend/redundancy.py`.

**Definition.** The *feasible pitch set* of a target is

    Φ(x,y,z) = { φ : inverse(x,y,z,φ) ≠ ⊥ }

By Theorem 2, `Φ ≠ ∅` exactly when the target is reachable at all. So
`Φ` is the honest reachability predicate; the pitch-conditional one is a slice
of it.

**Definition.** The *stop margin* of a configuration is
`m(q) = minᵢ (Λᵢ − |θᵢ|)` over the three pitch joints — how much room the
tightest joint has left. Choosing

    φ* = argmax_{φ ∈ Φ} m(q(φ))

resolves the redundancy by a stated criterion instead of by hand. Maximising
distance to the nearest stop is not the only defensible criterion — minimum
joint travel and manipulability are the obvious alternatives — but it is the
one that answers the question this arm keeps failing, since `shoulder_lift` is
the binding joint at every grasp *and* the obstruction in Theorem 3.

The argmax ranges over both elbow branches, so the criterion settles the
discrete freedom as well as the continuous one. `inverse`'s preference for
elbow-up is a convention — it returns the first branch that fits, not the
better one — and `resolve` therefore returns the configuration to command
rather than a pitch to feed back into `inverse`.

### How it is computed, and why not on a grid

Both `Φ` and `φ*` are computed by **interval branch-and-bound over `φ`**, not by
sweeping it. Sub-intervals of pitch are evaluated in interval arithmetic
(`ros_backend/interval.py`); one whose margin enclosure is entirely negative is
discarded, *with every pitch inside it*; one whose enclosure is entirely
non-negative is certified feasible wholesale; only undecided intervals are
subdivided. What comes back is a pair of bounds with `Φ_inner ⊆ Φ ⊆ Φ_outer`
differing only inside intervals narrower than the requested resolution, and a
`φ*` carrying a certified bound on how much better any other pitch could
possibly be (10⁻⁶ rad by default).

Every *candidate* is still evaluated through `kinematics.inverse` and its own
limit test, so nothing the search reports is a pose the solver would refuse.
The interval machinery is used only to **exclude** — which is the direction in
which a grid cannot help.

This matters more than it sounds, because the grid was wrong.

### The tables, and a 23° error the grid concealed

Over the whole turn of pitch, for the live grasp geometry — the 30×30×150 mm
bar, pads gripping at `z = 0.075 m`:

| Target x | `Φ` (certified) | width | `φ*` | margin at `φ*` |
| --- | --- | --- | --- | --- |
| 0.240 | [−113.02°, −42.59°] | 70.4° | −78.65° | 0.2388 rad |
| 0.262 | [−103.56°, −39.41°] | 64.2° | −73.18° | **0.2031 rad** |
| 0.280 | [−94.91°, −38.35°] | 56.6° | −68.62° | 0.1634 rad |
| 0.300 | [−83.87°, −39.37°] | 44.5° | −63.37° | 0.1069 rad |
| 0.320 | [−69.18°, −44.92°] | 24.3° | −57.74° | 0.0343 rad |

Against the **old** 50 mm cube, gripped at `z = 0.050 m`:

| Target x | `Φ` (certified) | width | `φ*` | margin at `φ*` |
| --- | --- | --- | --- | --- |
| 0.262 | [−87.66°, −59.46°] | 28.2° | −73.97° | 0.0409 rad |
| 0.280 | [−71.51°, −66.80°] | 4.7° | −69.17° | 0.0012 rad — on the stop |

The previous draft of this section reported `Φ` at x = 0.240 as [−90°, −43°],
from a 1° sweep. The upper endpoint was right: the true boundary is −42.59°,
which a 1° grid rounds to −43°. **The lower endpoint was the edge of the
sweep.** The scan ran over [−90°, 0°] and reported its own loop bound as a
property of the arm — and it did so for every row in the table, which is what
made it look like a fact rather than an artefact. The arm reaches pitches well
past straight down; at x = 0.240 the band continues to −113.02°, so the
feasible window was understated by 23°.

That is precisely the failure mode §3.8 warns about, found in this document's
own numbers. A grid that stops at the answer's edge cannot tell you it stopped.

Restricting to approaches from above is a perfectly reasonable *preference* —
a grasp usually wants to come down onto the object rather than tilt back under
it — and it is available as `domain=FROM_ABOVE`, which reproduces the old
table's endpoints exactly. The distinction the code now makes is that the
restriction is a named argument rather than a loop bound.

| Target x | `Φ ∩ [−90°, 0°]` | width |
| --- | --- | --- |
| 0.240 | [−90.00°, −42.59°] | 47.4° |
| 0.262 | [−90.00°, −39.41°] | 50.6° |
| 0.280 | [−90.00°, −38.35°] | 51.6° |
| 0.300 | [−83.87°, −39.37°] | 44.5° |
| 0.320 | [−69.18°, −44.92°] | 24.3° |

Note that **`φ*` and the margins are unchanged** by the restriction: the
optimum already lay inside the band. The old table's `φ*` column was therefore
right for the wrong reason, and only its `Φ` column was wrong — which is the
most dangerous shape an error can take, because the number anyone would have
checked against the robot was correct.

### What the numbers say

Changing the object from a 50 mm cube to a 150 mm bar did not merely make the
block reachable. At the nominal grasp it multiplied the joint margin by
**roughly five** (0.2031 against 0.0409 rad), and at x = 0.280 it took the arm
off a hard stop entirely: 0.0012 rad of room and a 4.7° pitch window became
0.1634 rad and 56.6°. The old geometry had no room to absorb approach error.
The new geometry has some.

**In every single case the binding joint is `shoulder_lift`.** The same stop
that makes the floor unreachable is the one that rations margin at the grasp.
One constraint explains both findings, which is worth saying aloud because it
means one model change — raising that stop — would relieve both at once.

---

## 3.8 What is proved, what is computed, and what is still sampled

Stated separately because the distinction is the point of this document.

**Proved analytically** — hold for all configurations, no enumeration:
Proposition 1 (forward kinematics); the closed form of §3.3; Theorem 1
(at most four solutions); Theorem 2 (completeness, *conditional on symmetric
stops*); Theorem 3 and its corollaries (the floor bound, tight); §3.6 (the
annulus is necessary and insufficient).

**Computed exactly** — finite evaluations of a closed form, not approximations:
the fixed-pose table in §3.2; the individual solutions in §3.7.

**Certified by interval branch-and-bound** — exhaustive over a continuum, not a
sample of it, and so proof-shaped rather than evidence-shaped:

- `Φ(x,y,z)` and `φ*` in §3.7. The bounds are enclosures: `Φ_inner ⊆ Φ ⊆
  Φ_outer` with the boundary located to the stated resolution, and `φ*` with a
  bound on how much any other pitch could improve on it. An empty `Φ` is a
  *proof* of unreachability at every approach pitch, not a failure to find one.

There is one assumption underneath this and it should be stated rather than
buried. CPython exposes no directed rounding, so `interval.py` pads every
result outward by two ulps instead. That is sound provided the platform's libm
is faithful to within a couple of ulps on `sin`, `cos`, `acos`, `atan2`,
`sqrt` and `hypot` — true of every libm this will run on, and not
machine-checked here. It is the one place in the chain that asks to be believed
rather than checked.

**Still established by sampling — the remaining honest gap:**

- `max_forward_reach()` scans in `x` and bisects. The bisection assumes the
  reachable set is an interval in `x` at fixed pitch. That is plausible and
  unproven; if the set were disconnected the function would silently return the
  edge of the wrong component.
- The reach-versus-height table sweeps pitch on a 1° grid.

**The remedy is not a proof assistant.** It is interval branch-and-bound over
the joint box: evaluate the forward map on interval arithmetic, subdivide any
box whose image straddles the target, discard boxes that provably cannot
contain a solution. This terminates with a certificate covering a continuum
rather than a grid. §3.7 is that method applied to the one-dimensional case,
and the arithmetic it needed is now written and tested, so extending it to the
remaining two items is a smaller job than it was.

**This list has already earned its keep.** `Φ` was on it in the previous draft.
Recomputing it by branch-and-bound rather than by sweep is what exposed the 23°
error in §3.7 — an error introduced by a scan reporting its own loop bound as a
property of the arm. The criticism this document anticipates from a reviewer,
that a grid scan and a simulator run are the same epistemic object, turned out
to be correct about this document's own numbers.

---

## 3.9 Status in the code

| Result | Where it lives | State |
| --- | --- | --- |
| Forward kinematics | `ros_backend/kinematics.py: forward()` | Implemented; cross-checked against an independently composed URDF chain |
| Inverse kinematics | `kinematics.py: inverse()` | Implemented, complete, refuses rather than clamps |
| Reachability decision | `kinematics.py: reachable()` | Implemented |
| Both elbow branches | `kinematics.py: branch_solutions()` | Implemented; `inverse` is now a filter over it, so the search and the solver cannot drift apart |
| Stop margin `m(q)` | `kinematics.py: ArmGeometry.stop_margin()` | Implemented; non-negative exactly when `within_limits` holds |
| Floor bound (Thm 3) | `tests/test_kinematics.py` | Proved in §3.5; asserted as a theorem with a witness |
| Completeness (Thm 2) | `tests/test_kinematics.py` | Proved in §3.4; asserted, with an asymmetric-stop counterexample pinning the hypothesis |
| Interval arithmetic | `ros_backend/interval.py` | Implemented and tested — enclosures for the search, reusable for the joint box |
| Feasible pitch set `Φ` | `ros_backend/redundancy.py: feasible_pitches()` | Implemented, certified inner and outer bounds |
| Margin-optimal `φ*` | `ros_backend/redundancy.py: resolve()` | Implemented, with a certified optimality gap |
| Calling any of it from `pick()` | `ros_backend/backend.py` | **Not done — `pick()` calls `grasp_sequence()`, which replays PRE_GRASP and GRASP verbatim** |

The ordered next steps, smallest first:

1. ~~Replace the grid-scan floor test with an assertion of Theorem 3.~~ Done.
2. ~~Add a completeness test that pins the symmetric-stops hypothesis.~~ Done.
3. ~~Implement `Φ` and `φ*`.~~ Done, and certified rather than sampled.
4. **Wire `redundancy.resolve()` into `pick()`**, so the arm can refuse a target
   it cannot serve and choose its own approach pitch when it can. This is the
   change that gives the robot a reachability check at all. `resolve` returns
   the configuration to command directly; it should not be round-tripped
   through `inverse`, which would silently re-impose the elbow-up convention.
5. Replace the remaining scans in §3.8 with interval branch-and-bound over the
   joint box, reusing `interval.py`.

Steps 1–3 needed no simulator and no hardware, and are complete. Step 4 needs
one. Step 5 does not.

A note on cost, since it affects where `resolve` can be called. `feasible_pitches`
takes about 10 ms; `resolve` takes about 300 ms at the default 10⁻⁶ rad
tolerance, because certifying a smooth maximum to a gap of `t` needs `O(1/√t)`
sub-intervals. That is fine once per grasp and fine for building tables, and it
is adjustable: 10⁻⁴ rad costs about 5 ms and is still four orders finer than
anything the servos resolve.

**A correctness note found while writing this.** `tests/test_kinematics.py`
declares `FIXED_POSES["DROP"] = (0.0, 1.1, 0.6, 0.1)` under a docstring reading
"the five poses replayed by `manipulation.py`". `manipulation.py` now holds
`DROP = [0.0, 0.814, 0.733, 0.547]`. The test still passes — it checks the old
pose against the old documented figures (0.405, 0.289), both of which reproduce
— but it is no longer testing the live constant. This is precisely the
transcription rot that `test_kinematics_urdf.py` was written to prevent, in the
file that prevents it for everything else.

**Also worth doing:** `test_kinematics_urdf.py` skips whenever the simulator's
URDF is absent, which is most machines. The `mobile-arm-dynamics` repo has a
checked-in expanded `urdf/mobile_arm_baseline.urdf` whose arm joint origins and
stops match `ARM` exactly. Pointing the test's candidate list at it would make
the URDF cross-check run everywhere instead of nowhere.
