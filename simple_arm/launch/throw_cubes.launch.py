import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    pkg_name = "simple_arm"
    pkg_share = get_package_share_directory(pkg_name)

    run_move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'run_move_group.launch.py'),
        ),)

    collision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'collision.launch.py'),
        ),)

    moveit_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'throw_moveit_controller.launch.py'),
        ),)

    timer_1 = IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription([  # hack since you can't have recursive timer
        moveit_controller,
        TimerAction(
            period=10.,
            actions=[
                collision,
            ])
    ])))

    return LaunchDescription([
        run_move_group_node,
        TimerAction(
            period=5.,
            actions=[
                timer_1
            ]
        )
    ])