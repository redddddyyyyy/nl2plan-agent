"""Pure-logic tests for the ROS2 backend: no rclpy, no sim."""

from __future__ import annotations

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
