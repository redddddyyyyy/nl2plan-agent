"""Pure Pursuit path follower for an Ackermann-steered base.

Subscribes:
  /planned_path     nav_msgs/Path           (from hybrid_astar_planner)
  /odom             nav_msgs/Odometry       (from Gazebo plugin)

Publishes:
  /ackermann_cmd    ackermann_msgs/AckermannDriveStamped

This is a stub. Fill in the lookahead computation and the steering-angle
formula (delta = atan2(2 * L * sin(alpha), Ld)) on the Linux side.
"""

# TODO(linux): implement on Linux with rclpy.
# Reference implementation pattern:
#   1. Buffer the latest /planned_path. Index = 0 on new path.
#   2. On each /odom callback:
#      a. Find lookahead point: first path point at distance >= Ld from robot.
#      b. Compute alpha = angle from heading to lookahead point.
#      c. Steering = atan2(2 * wheelbase * sin(alpha), Ld).
#      d. Publish AckermannDriveStamped with steering + target speed.
#   3. Stop when within goal_tolerance of final path point.


def main():
    raise NotImplementedError("Implement on Linux. See module docstring for the recipe.")


if __name__ == "__main__":
    main()
