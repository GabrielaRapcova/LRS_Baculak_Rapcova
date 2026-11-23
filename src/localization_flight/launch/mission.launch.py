"""Estimator + estimate-driven flight (Assignment 2 §4).

    ros2 launch localization_flight mission.launch.py                  # fly
    ros2 launch localization_flight mission.launch.py evaluator:=true  # + truth evaluator
    ros2 launch localization_flight mission.launch.py fly:=false       # estimator only

Start Gazebo, SITL, MAVROS and the sensor simulator first. The flight node
starts the mission by itself as soon as the FCU is connected and the estimate
is valid: no manual input after launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    loc_share = get_package_share_directory('drone_localization')
    flight_share = get_package_share_directory('localization_flight')

    return LaunchDescription([
        DeclareLaunchArgument('evaluator', default_value='false'),
        DeclareLaunchArgument('fly', default_value='true'),
        DeclareLaunchArgument('mission_params',
                              default_value=os.path.join(flight_share, 'config', 'mission.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, 'launch', 'localization.launch.py')),
            launch_arguments={'evaluator': LaunchConfiguration('evaluator')}.items(),
        ),
        Node(
            package='localization_flight',
            executable='estimated_mission_node',
            name='estimated_mission',
            output='screen',
            condition=IfCondition(LaunchConfiguration('fly')),
            parameters=[LaunchConfiguration('mission_params'), {'use_sim_time': True}],
        ),
    ])
