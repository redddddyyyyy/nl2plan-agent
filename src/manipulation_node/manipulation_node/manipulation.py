"""Pick and place via MoveIt2.

Exposes:
  /nl2plan/pick   (string object_id -> bool success, string error)
  /nl2plan/place  (geometry_msgs/Pose target -> bool success, string error)

Pick recipe (top-down only):
  1. Look up object pose from the perception node's object table by object_id.
  2. Plan to approach pose = object pose + (0, 0, +0.10) with gripper open,
     gripper facing -z.
  3. Plan Cartesian descent to object pose + (0, 0, +0.02).
  4. Close gripper.
  5. Plan Cartesian lift to approach pose.
  6. On any planning/execution failure: return success=False with a structured
     error string. The agent will decide whether to retry from a different
     base pose.

Place recipe is the reverse with the target pose instead of the object pose.
"""

# TODO(linux): implement on Linux against MoveIt2 Python bindings (pymoveit2
# or the moveit_py package). The robot's planning group name and end-effector
# link name must match the URDF in kitchen_world.


def main():
    raise NotImplementedError("Implement on Linux. See module docstring.")


if __name__ == "__main__":
    main()
