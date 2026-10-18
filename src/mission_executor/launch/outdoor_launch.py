"""Launch outdoor diagnostics or the takeoff/return flight check."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Keep flight disabled unless explicitly requested."""
    return LaunchDescription([
        DeclareLaunchArgument('enable_flight', default_value='false'),
        DeclareLaunchArgument('execute_square', default_value='true'),
        DeclareLaunchArgument('execute_figure_eight', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission_executor'),
                'config', 'outdoor.yaml'])),
        Node(
            package='mission_executor', executable='outdoor_mission_node',
            name='outdoor_mission_node',
            parameters=[LaunchConfiguration('params_file'), {
                'enable_flight': ParameterValue(
                    LaunchConfiguration('enable_flight'), value_type=bool),
                'execute_square': ParameterValue(
                    LaunchConfiguration('execute_square'), value_type=bool),
                'execute_figure_eight': ParameterValue(
                    LaunchConfiguration('execute_figure_eight'),
                    value_type=bool),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)}],
            output='screen'),
    ])
