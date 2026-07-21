# Known issues

Findings from an independent review (2026-07-21) of the approach and perception
code, kept here rather than fixed in a hurry: each one needs a live sim run to
verify, and the build described in the checkpoint report is the one that was
measured. Ordered by severity.

### 1. The final heading trim still trusts the map frame

`ros_backend/manipulation.py`, end of `align()`

After the creep, `align` re-reads the AMCL pose and computes a fresh bearing to
the target's *map* coordinate — but that coordinate was derived from an AMCL fix
taken before the creep started. If the estimate corrects itself in between, the
trim rotates by the correction instead of toward the block, and the fixed arm
poses then reach beside it. At 0.35 m of reach, a 10 deg error is already wider
than the 5 cm block.

This is the same mistake that caused the false pick refusals fixed earlier the
same day, in the one place that was not audited. It never fired across twelve
verified missions, because a straight 0.25 m creep rarely spans a large AMCL
correction, but the exposure is real.

**Fix:** remember the odometry yaw goal set by the initial rotation and trim back
to that, instead of recomputing a bearing from the map frame. Odometry is smooth
over the two or three seconds a creep lasts, which is exactly the interval the
trim is meant to correct.

### 2. A refused approach always reverses, whatever went wrong

`ros_backend/backend.py`, the `align` error branch in `pick()`

`pick` treats every `align` failure as "pressed against an obstacle" and backs
away 0.3 m. But `align` also fails on a rotation timeout, a creep timeout, and
missing localization. Reversing blindly after a partial rotation moves the robot
somewhere it has not just driven through, and when odometry is unavailable
`back_away` does nothing at all while the returned message still claims the robot
backed off.

**Fix:** distinguish the contact stall from the other failure modes and reverse
only for that one; make the error text match what actually happened.

### 3. A confirmed sighting returns an averaged map coordinate

`ros_backend/perception.py`, `_confirm()`

The gate deliberately accepts clusters that are tight in the robot's frame even
when their map projections disagree, then returns the mean of those disagreeing
map coordinates. `find_object` caches that mean and `pick` uses it for the range
check and the Nav2 standoff goal, so the standoff can be aimed up to ~0.2 m off.
The re-confirm at the standoff hides this in practice.

**Fix:** derive the returned coordinate by transforming the averaged
robot-relative observation through one current robot pose, rather than averaging
map coordinates that were computed under different pose estimates.

### 4. Detections are converted using an unsynchronized robot pose

`ros_backend/perception.py`, `_confirm()`

A detection is projected by the detector using TF at the image timestamp, but the
backend converts it to the robot frame using its latest cached AMCL pose. The two
run in separate executors, so the common-mode cancellation that makes robot-frame
clustering work is only approximate. The confirm window runs with the robot
stationary, which is when AMCL is quietest, so the practical effect is small.

**Fix:** publish the observation in a robot-relative frame with its own stamp, or
look up the robot pose at the detection timestamp.

### 5. The prompt's block locations are spawn locations

`prompt.py`

The room list is stated as a hard rule, but `place` moves blocks and an
interactive session remembers earlier commands. After a block has been placed on
the table, asking for it again sends the robot to its original room and then
through the fallback sweep, which does not include the table.

**Fix:** describe the list as initial locations and tell the model to prefer a
location it has observed during the session.

### 6. Scan diagnostics are module-global

`ros_backend/perception.py`, `LAST_SCAN`

Fine for one agent session, which is the only supported mode today, but two
concurrent scans would interleave their diagnostics.
