"""One-command bringup: mobile_arm_sim's full stack plus the multi-color
detector. The sim's own red-only detector keeps publishing
/target_block_pose; the agent ignores it."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('mobile_arm_sim'),
        'launch', 'autonomous.launch.py')))

    # PYTHONNOUSERSITE keeps pip-user numpy 2.x away from cv_bridge — the
    # same clash the sim-side nodes hit when run by hand.
    detector = Node(
        package='perception_node',
        executable='color_block_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
        additional_env={'PYTHONNOUSERSITE': '1'},
    )
    mover = Node(
        package='perception_node',
        executable='scene_setup',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([sim, detector, mover])
