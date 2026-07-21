"""Pure-logic tests for the ROS2 backend: no rclpy, no sim."""

from __future__ import annotations

import math

from nl2plan_agent.ros_backend.logic import (
    COLOR_ENTITIES,
    cluster_spread,
    mean_xy,
    nav_result,
    parse_color,
)


# ---------- color parsing ----------

def test_parse_color_finds_each_known_color():
    assert parse_color("the red block") == "red"
    assert parse_color("pick up the ORANGE cube") == "orange"
    assert parse_color("a magenta block please") == "magenta"
    assert parse_color("brown block") == "brown"


def test_parse_color_needs_a_whole_word():
    # "browning" contains "brown" but is not a color word
    assert parse_color("the browning pan") is None


def test_parse_color_unknown_returns_none():
    assert parse_color("the blue block") is None
    assert parse_color("nonexistent") is None


def test_color_entity_map_matches_gazebo_spawn_names():
    assert COLOR_ENTITIES["red"] == "target_block"
    assert COLOR_ENTITIES["magenta"] == "distractor_magenta"
    assert COLOR_ENTITIES["orange"] == "distractor_orange"
    assert COLOR_ENTITIES["brown"] == "distractor_brown"


# ---------- sighting cluster ----------

def test_cluster_spread_tight_sightings():
    samples = [(1.00, 2.00), (1.05, 2.02), (0.98, 1.97), (1.02, 2.01)]
    assert cluster_spread(samples) < 0.2


def test_cluster_spread_scattered_sightings():
    # two sightings 0.6 m apart — the phantom-target case from the sim work
    samples = [(1.0, 2.0), (1.6, 2.0), (1.0, 2.1), (1.3, 2.0)]
    assert cluster_spread(samples) > 0.2


def test_mean_xy_averages():
    x, y = mean_xy([(1.0, 2.0), (3.0, 4.0)])
    assert x == 2.0 and y == 3.0


# ---------- nav status mapping ----------

def test_nav_result_succeeded():
    out = nav_result(4, 12.34, {"x": 1.0, "y": 2.0, "theta": 0.0})
    assert out["success"] is True
    assert out["final_pose"] == {"x": 1.0, "y": 2.0, "theta": 0.0}
    assert out["duration_s"] == 12.3


def test_nav_result_aborted_and_canceled():
    assert nav_result(6, 5.0, {})["success"] is False
    assert "aborted" in nav_result(6, 5.0, {})["error"]
    assert "canceled" in nav_result(5, 5.0, {})["error"]


def test_nav_result_unknown_status():
    out = nav_result(99, 5.0, {})
    assert out["success"] is False
    assert "99" in out["error"]


def test_standoff_pose_between_robot_and_target():
    from nl2plan_agent.ros_backend.logic import standoff_pose
    import math
    # robot due south of target: goal sits 0.6 m south of target, facing north
    gx, gy, yaw = standoff_pose(0.0, 0.0, 0.0, 2.0, 0.6)
    assert abs(gx) < 1e-9 and abs(gy - 1.4) < 1e-9
    assert abs(yaw - math.pi / 2) < 1e-6
    # goal is always `standoff` from the target, on the robot's side
    gx, gy, yaw = standoff_pose(3.0, -1.0, 1.0, 1.0, 0.5)
    assert abs(math.hypot(gx - 1.0, gy - 1.0) - 0.5) < 1e-9


def test_to_robot_frame_cancels_localization_error():
    """Clustering must survive AMCL heading jitter.

    Measured live 2026-07-20 at the bedroom_window pose: ground-truth yaw
    1.428 vs AMCL 1.85 (24 deg out). The block's map projection swings with
    that error, so map-frame samples of a STATIONARY block scatter past the
    0.2 m spread gate and find_object reports "not visible" while staring
    right at it. Relative to the robot the same sightings are steady,
    because the error is common to the robot pose and the projection.
    """
    from nl2plan_agent.ros_backend.logic import cluster_spread, to_robot_frame

    # One block 0.9 m dead ahead, seen while AMCL's yaw drifts 1.43 -> 1.85.
    truth = (-4.5, 2.1)
    samples_map, samples_rel = [], []
    for yaw in (1.428, 1.55, 1.70, 1.85):
        robot = (-4.44, 1.19, yaw)
        # what the detector computes when it trusts this (wrong) yaw
        bearing = math.atan2(truth[1] - robot[1], truth[0] - robot[0])
        rng = math.hypot(truth[0] - robot[0], truth[1] - robot[1])
        skew = yaw - 1.428
        seen = (robot[0] + rng * math.cos(bearing + skew),
                robot[1] + rng * math.sin(bearing + skew))
        samples_map.append(seen)
        samples_rel.append(to_robot_frame(seen[0], seen[1], robot))

    assert cluster_spread(samples_map) > 0.2      # map frame: gate rejects
    assert cluster_spread(samples_rel) < 0.05     # robot frame: rock steady


def test_pick_error_names_the_id_that_would_work():
    """A rejected pick must say which id is valid, not just 'find first'.

    Live 2026-07-20: qwen called pick with the literal placeholder
    '<object_id_from_find_object_response>' straight after a successful
    find_object. The old error only said "run find_object first", so the
    model re-scanned instead of correcting the id and burned the step cap.
    """
    from nl2plan_agent.ros_backend.logic import pick_error

    msg = pick_error("<object_id_from_find_object_response>", "orange_block")
    assert "orange_block" in msg

    # Nothing confirmed yet: fall back to telling it to scan.
    assert "find_object" in pick_error("orange_block", None)


def test_from_robot_frame_round_trips():
    from nl2plan_agent.ros_backend.logic import from_robot_frame, to_robot_frame

    robot = (-4.44, 1.19, 1.428)
    rel = to_robot_frame(-4.5, 2.1, robot)
    back = from_robot_frame(rel[0], rel[1], robot)
    assert math.isclose(back[0], -4.5, abs_tol=1e-9)
    assert math.isclose(back[1], 2.1, abs_tol=1e-9)


def test_relative_target_survives_a_localization_jump():
    """Re-anchoring a sighting beats replaying its map coordinates.

    pick() sights the block, then drives the last stretch. If AMCL jumps in
    between (it did: 24 deg at bedroom_window), map coordinates recorded
    before the jump point somewhere else entirely, so the robot creeps to
    empty floor and the magic grasp teleports the block in from there.
    The robot-relative offset stays true through the jump.
    """
    from nl2plan_agent.ros_backend.logic import from_robot_frame, to_robot_frame

    truth = (-4.5, 2.1)
    at_sighting = (-4.44, 1.19, 1.428)
    rel = to_robot_frame(truth[0], truth[1], at_sighting)

    # AMCL yaw jumps 24 deg while the robot physically stands still.
    after_jump = (-4.44, 1.19, 1.85)
    re_anchored = from_robot_frame(rel[0], rel[1], after_jump)

    def bearing_off_the_nose(target, robot):
        b = math.atan2(target[1] - robot[1], target[0] - robot[0])
        return (b - robot[2] + math.pi) % (2 * math.pi) - math.pi

    want = bearing_off_the_nose(truth, at_sighting)
    # Replaying the stale map coordinate: the robot now thinks the block
    # sits 24 deg off where it did, and steers into empty floor.
    stale = bearing_off_the_nose(truth, after_jump)
    assert abs(stale - want) > 0.4
    # Re-anchoring the relative offset keeps it dead ahead as measured.
    fixed = bearing_off_the_nose(re_anchored, after_jump)
    assert math.isclose(fixed, want, abs_tol=1e-9)
