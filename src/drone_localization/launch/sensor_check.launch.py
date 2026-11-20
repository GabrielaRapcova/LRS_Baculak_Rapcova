"""Start the §1 sensor-check node on simulation time."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('report_period_s', default_value='5.0'),
        Node(
            package='drone_localization',
            executable='sensor_check_node',
            name='sensor_check_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'report_period_s': LaunchConfiguration('report_period_s'),
            }],
        ),
    ])
