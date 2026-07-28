# The geometry behind the tuned constants

Several numbers in this repository were arrived at by watching the robot fail and
adjusting until it stopped: the 0.6 m standoff before a grasp, the 0.35 m reach
the base drives to, the decision to cluster detections in the robot's frame
rather than the map's. They worked, but "it worked after I changed it" is a weak
thing to stand on.

This document derives them. Each section states the geometry, gives the number
it predicts, and compares that against the constant already in the code. In
three cases the derivation and the tuned value agree, which is the useful
outcome — it means the fix was right for a reason rather than by luck.

Everything here is about code that exists in this repository. The separate
question of whether the physical robot would stay upright is not answered here,
because this repository cannot answer it; see the note at the end.

## 1. Where the detector's numbers come from

`perception_node/color_block_detector.py` finds a coloured blob, takes its
centroid in pixels, and puts that point on the floor. The projection is exact
geometry. Given camera intrinsics `K`, and the camera's rotation `R` and
position `t` in the map frame, the ray through pixel `(u, v)` is scaled until it
meets the ground plane:

```
ray_cam = K⁻¹ [u, v, 1]ᵀ
ray_map = R · ray_cam
s       = (z_ground − t_z) / ray_map_z
P_map   = t + s · ray_map
```

The guard that earns its place is the sign check on `ray_map_z`. A pixel above
the horizon gives a ray that never descends, `s` comes out negative, and the
detector confidently reports a block behind the camera. Rejecting rays that do
not point downward is the difference between a geometric method and one that
merely looks like one.

## 2. Why the grasp re-confirms at 0.6 m

`pick` does not grasp on the sighting it found during the search scan. It drives
to a 0.6 m standoff, faces the block, and looks again. That standoff was picked
by trial.

The geometry says what it buys. A camera at height `h` looking down at
depression angle `θ` sees ground range `d = h / tan θ`. Differentiating, an error
in the camera's pitch estimate produces a range error of

```
∂d/∂θ = h / sin²θ
```

The camera sits 0.230 m above the floor. Evaluating at the ranges that matter:

| Ground range | Depression angle | Range error per degree of pitch error |
| --- | --- | --- |
| 0.6 m | 21.0° | **3.1 cm** |
| 1.0 m | 13.0° | 8.0 cm |
| 1.1 m | 11.8° | 9.6 cm |

Search-scan sightings land at roughly 0.95–1.19 m, because Nav2's goal tolerance
plus localisation error leaves the robot about a metre from a block nominally
0.7 m from the search pose. So re-confirming at 0.6 m instead of grasping on the
original sighting is worth a factor of three in range error. The tuned number
was the right one.

## 3. Why detections are clustered in the robot's frame

`find_object` accepts a sighting only when four consecutive samples fall within
0.2 m of each other, measured **in the robot's frame** rather than the map's.
That choice looks arbitrary until the error is written down.

A detection is projected through the robot's believed pose. A heading error `ψ`
at range `d` therefore displaces the projected point laterally by

```
lateral error ≈ d · ψ
```

A 24° AMCL yaw error was measured at one search pose. At 1.1 m that predicts
0.46 m of displacement. The error measured against ground truth at that pose was
0.55 m — the same magnitude, with the remainder accounted for by the pitch term
in section 2. After the localisation problem was fixed the same pose showed 3.5°
of yaw error, predicting 0.07 m, comfortably inside the 0.2 m gate. That is
exactly when the failure stopped.

The reason robot-frame clustering works follows directly. The same heading error
enters the robot's pose *and* the projection of the detection. Differencing the
two cancels it as a common-mode term, so a stationary block looks stationary
even while the heading estimate wanders. In the map frame it does not: the block
smears across the gate and the tool reports "not visible" for something in plain
view.

## 4. The arm's reach, checked against the constant

`manipulation.py` drives the base until the block sits `GRASP_REACH = 0.35 m`
ahead of base centre. That number was measured on the simulator months before
any of this analysis existed.

The arm is a shoulder pan about the vertical axis followed by three joints
rotating about parallel axes — a planar three-link chain on a turntable. Reading
the link lengths out of the URDF joint origins:

| Segment | Length |
| --- | --- |
| L1, shoulder-lift to elbow | 0.150 m |
| L2, elbow to wrist | 0.120 m |
| L3, wrist to gripper tip | 0.105 m |
| kinematic maximum, L1+L2+L3 | 0.375 m |
| shoulder pivot, forward of base centre | 0.050 m |
| **maximum forward tip reach** | **0.425 m** |

`GRASP_REACH` sits at 82% of the kinematic maximum. The empirical constant and
the geometry agree, and the margin is sensible rather than accidental — reaching
at full extension would leave no room for the approach error the standoff exists
to absorb.

## 5. What the arm does not do

The robot does not solve inverse kinematics. It never computes joint angles for
a target position. `pick` and `place` replay five fixed joint-angle sets — REST,
PRE_GRASP, GRASP, LIFT, DROP — measured on the simulator.

This is worth being plain about, because it explains the shape of the rest of
the code. Since the arm cannot adapt to where the block is, the base has to put
the block where the arm already goes, and that is why the approach protocol is
as elaborate as it is: navigate to a standoff, re-confirm, creep the last
fraction of a metre, refuse if the creep stalls.

The closed form for this arm is short, and implementing it is the next
manipulation task. For a target `(x, y, z)` in the base frame with a chosen
gripper approach pitch `φ`:

```
θ₁ = atan2(y, x)                                  pan
ρ  = √(x² + y²) − 0.05                            radial distance from pivot
ζ  = z − 0.26                                     height above pivot
ρ_w = ρ − L3·cos φ                                wrist position
ζ_w = ζ − L3·sin φ
c₃ = (ρ_w² + ζ_w² − L1² − L2²) / (2·L1·L2)
θ₃ = ±acos(c₃)                                    elbow, two solutions
θ₂ = atan2(ζ_w, ρ_w) − atan2(L2·sin θ₃, L1 + L2·cos θ₃)
θ₄ = φ − θ₂ − θ₃                                  wrist
```

The part worth having is the reachability condition, because it would let the
arm refuse an impossible target instead of reaching for it. A solution exists
exactly when the wrist point lies in the annulus the two proximal links can
span:

```
|L1 − L2| ≤ √(ρ_w² + ζ_w²) ≤ L1 + L2
  0.030 m ≤        reach      ≤ 0.270 m
```

Three joints in the plane against two positional degrees of freedom makes the
arm redundant by one; fixing the approach pitch removes the redundancy and makes
the solution closed-form. The elbow sign gives the usual elbow-up and elbow-down
pair, and the one to take is whichever keeps every joint inside its limits.

## 6. What obstacle avoidance actually guarantees

Navigation is Nav2's, and the guarantee is narrower than it looks. A global
planner (NavFn) runs over a static costmap; a local controller (DWB) samples
velocity commands, rolls each forward for a fixed horizon, and scores them.

What that provides is clearance: obstacles are inflated by the robot's radius
plus a margin, and the planner will not route the robot's centre through an
inflated cell. That is a checkable geometric property.

What it does not provide is a guarantee. DWB samples a finite set of
trajectories. It has no completeness property — if none of the sampled
trajectories is collision-free the controller fails and recovery behaviours run,
which is error handling rather than a proof.

Two limits are worth stating because they are structural rather than tuning:

- **The scan plane sits 0.20 m above the floor.** The blocks are 0.05 m cubes,
  four times below it. They are invisible to navigation, which is why an early
  scene placed a block where the robot drove straight over it. Anything shorter
  than 0.20 m, and anything overhanging above it, is not an obstacle as far as
  Nav2 is concerned.
- **The costmap footprint must cover the robot.** Computing the circumscribed
  radius from the URDF — chassis corners and wheel outer edges — gives 0.265 m.
  A costmap configured with a smaller radius spends the difference out of the
  obstacle clearance margin.

## What this document does not cover

Whether the physical robot would stay upright. That question depends on mass,
centre-of-mass placement, wheel geometry and floor friction, none of which this
repository contains — the simulated base is driven kinematically and cannot tip
over, by construction rather than by result. The argument for it, and the
measurements it needs, are separate work and live outside this repository. See
[architecture.md](architecture.md) section 5 for why the simulation cannot
answer it.
