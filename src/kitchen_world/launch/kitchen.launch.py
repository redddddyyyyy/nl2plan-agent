"""Launch Gazebo with the kitchen world + spawn the mobile manipulator + bring up the
hybrid_astar_planner, controller_node, perception_node, and manipulation_node."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare("kitchen_world")
    world = PathJoinSubstitution([pkg_share, "worlds", "kitchen.world"])

    return LaunchDescription([
        ExecuteProcess(
            cmd=["gazebo", "--verbose", world,
                 "-s", "libgazebo_ros_init.so",
                 "-s", "libgazebo_ros_factory.so"],
            output="screen",
        ),
        # TODO(linux): add robot_state_publisher with the URDF, spawn_entity for the robot,
        # and bring up controller_node, perception_node, manipulation_node, hybrid_astar_planner.
    ])
