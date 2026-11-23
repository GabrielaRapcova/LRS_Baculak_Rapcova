"""Start the localisation estimator, optionally with the truth-only evaluator.

    ros2 launch drone_localization localization.launch.py
    ros2 launch drone_localization localization.launch.py evaluator:=true

The evaluator is a separate process: it subscribes to /evaluation/ground_truth
and the estimate, and publishes only /evaluation/* topics that nothing in the
estimator or controller subscribes to.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('drone_localization')
    default_params = os.path.join(share, 'config', 'estimator.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params,
                              description='estimator parameter file'),
        DeclareLaunchArgument('evaluator', default_value='false',
                              description='also start the truth-only evaluator'),
        DeclareLaunchArgument('evaluation_dir', default_value='evaluation',
                              description='where the evaluator writes CSV and plots'),
        Node(
            package='drone_localization',
            executable='estimator_node',
            name='localization_estimator',
            output='screen',
            parameters=[LaunchConfiguration('params'), {'use_sim_time': True}],
        ),
        Node(
            package='drone_localization',
            executable='estimate_evaluator.py',
            name='estimate_evaluator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('evaluator')),
            # Writing CSV + plot on Ctrl-C can take a few seconds; don't SIGTERM it at 5 s.
            sigterm_timeout='30',
            parameters=[{'use_sim_time': True,
                         'output_dir': LaunchConfiguration('evaluation_dir')}],
        ),
    ])
