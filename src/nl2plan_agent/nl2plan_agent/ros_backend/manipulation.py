"""Arm sequences, align-creep docking, and the magic-grasp pin.

Ported from mobile_arm_sim's autonomous_pick_place.py as blocking calls (the
executor daemon thread keeps the pin timer and subscriptions alive while
these loops sleep). The joint targets were measured there, not guessed —
don't retune them here.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist

# Arm poses (shoulder_pan, shoulder_lift, elbow, wrist).
REST      = [0.0, -0.5, 1.2, 0.3]
PRE_GRASP = [0.0,  0.6, 1.4, 0.5]
GRASP     = [0.0,  0.9, 1.6, 0.5]
LIFT      = [0.0,  0.0, 1.0, 0.3]
# DROP puts the gripper 0.35 m ahead of base centre — over the table middle
# when the base is touch-docked. The old release pose dropped blocks into
# the robot/table gap.
DROP      = [0.0,  1.1, 0.6, 0.1]

GRIPPER_OPEN = -0.015
GRIPPER_CLOSED = -0.005

GRASP_REACH = 0.35   # the fixed arm poses reach ~0.35 m ahead of base centre
PICK_RANGE = 1.3     # detection range is ~1.1 m; farther means stale detection
TABLE_XY = (4.0, -2.5)
TABLE_REACH = 0.20   # aims PAST the contact point: the stall guard is the stop

ALIGN_TIMEOUT_S = 40.0


def _wait_poses(node, timeout: float = 5.0) -> Optional[str]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if node.robot is not None and node.odom is not None:
            return None
        time.sleep(0.1)
    return "No localization yet (AMCL/odom silent). Is the sim running?"


def _ang_err(target: float, current: float) -> float:
    return (target - current + math.pi) % (2 * math.pi) - math.pi


def _rotate_to(node, goal_yaw: float, deadline: float) -> bool:
    while time.monotonic() < deadline:
        if abs(_ang_err(goal_yaw, node.odom[2])) <= 0.03:
            node.stop_base()
            return True
        cmd = Twist()
        cmd.angular.z = 0.3 if _ang_err(goal_yaw, node.odom[2]) > 0 else -0.3
        node.cmd_pub.publish(cmd)
        time.sleep(0.05)
    node.stop_base()
    return False


def align(node, tx: float, ty: float, reach: float,
          timeout: float = ALIGN_TIMEOUT_S,
          stall_is_error: bool = False) -> Optional[str]:
    """Face (tx, ty), creep until it sits `reach` m ahead; error or None.

    Rotation error is measured once against AMCL, then executed on odometry
    — odom is smooth while AMCL updates arrive in chunky steps that make a
    feedback loop hunt. The creep's stall guard treats "commanded forward
    but not moving" as contact; for the table, contact IS arrival
    (touch-docking; the distance-based table stop missed half the time).
    A pick approach passes stall_is_error=True: contact short of reach
    means an obstacle, and grasping from there teleports the block. The
    shortfall is judged HERE, on odometry, because comparing the robot's
    AMCL pose to the target after the creep double-counts whatever AMCL
    corrected mid-creep — that misread 0.52-0.58 m on picks whose robot
    was standing at the block (live 2026-07-21, the back-off/stare loop).
    """
    err = _wait_poses(node)
    if err is not None:
        return err
    deadline = time.monotonic() + timeout

    rx, ry, ryaw = node.robot
    bearing = math.atan2(ty - ry, tx - rx)
    drive = max(math.hypot(tx - rx, ty - ry) - reach, 0.0)
    if not _rotate_to(node, node.odom[2] + _ang_err(bearing, ryaw), deadline):
        return "Lineup timed out while rotating."

    start = node.odom
    stall_ref, stall_t = -1.0, time.monotonic()
    while True:
        if time.monotonic() > deadline:
            node.stop_base()
            return "Lineup timed out while creeping."
        moved = math.hypot(node.odom[0] - start[0], node.odom[1] - start[1])
        if moved >= drive:
            break
        if moved > stall_ref + 0.02:
            stall_ref, stall_t = moved, time.monotonic()
        elif time.monotonic() - stall_t > 2.5:
            node.stop_base()   # pressed against something static
            if stall_is_error:
                return (f"Pressed against something "
                        f"{max(drive - moved, 0.0):.2f} m before reaching "
                        "the target.")
            return None   # docked (the table's arrival signal)
        cmd = Twist()
        cmd.linear.x = 0.1
        node.cmd_pub.publish(cmd)
        time.sleep(0.05)

    # Heading drift over the creep once dropped a block on the table edge;
    # face the target one more time.
    node.stop_base()
    rx, ry, ryaw = node.robot
    bearing = math.atan2(ty - ry, tx - rx)
    if not _rotate_to(node, node.odom[2] + _ang_err(bearing, ryaw), deadline):
        return "Lineup timed out while trimming heading."
    return None


def run_stages(node, stages):
    """Fire each (callable, dwell_s) in order, sleeping out the dwell."""
    for fn, dwell in stages:
        fn()
        time.sleep(dwell)


def attach(node, entity: str):
    node.pinned_entity = entity


def detach(node, entity: str):
    """Stop the pin, then park the block in clear air 8 cm below the gripper.

    Cut loose at the gripper origin the block sits inside the palm's
    collision box and the next arm swing bats it off the table. Pin stops
    FIRST so a racing 20 Hz tick can't overwrite the release teleport.
    """
    node.pinned_entity = None
    try:
        t = node.tf_buffer.lookup_transform('base_footprint', 'gripper_base',
                                            rclpy.time.Time())
        node.set_entity_rel(entity,
                            t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z - 0.08)
    except tf2_ros.TransformException:
        pass


def grasp_sequence(node, entity: str):
    """Open, reach, close, pin, lift — same stages and dwells as the sim's."""
    run_stages(node, [
        (lambda: node.gripper(GRIPPER_OPEN), 0.5),
        (lambda: node.arm(PRE_GRASP), 2.5),
        (lambda: node.arm(GRASP), 2.0),
        (lambda: node.gripper(GRIPPER_CLOSED), 0.6),
        (lambda: attach(node, entity), 0.5),
        (lambda: node.arm(LIFT), 2.0),
    ])


def back_away(node, distance: float = 0.3, timeout: float = 8.0):
    """Reverse a short straight line on odometry; best-effort."""
    if node.odom is None:
        return
    start = node.odom
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        moved = math.hypot(node.odom[0] - start[0], node.odom[1] - start[1])
        if moved >= distance:
            break
        cmd = Twist()
        cmd.linear.x = -0.08
        node.cmd_pub.publish(cmd)
        time.sleep(0.05)
    node.stop_base()
