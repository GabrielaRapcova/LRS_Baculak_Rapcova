"""Prepare the real-flight entry point; hardware setup remains pending."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Reuse the common mission launch and keep flight disabled by default."""
    return LaunchDescription([
        DeclareLaunchArgument('enable_flight', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission_executor'),
                'config', 'outdoor.yaml'])),
        # TODO: Configure MAVROS transport, system IDs and telemetry rates.
        # TODO: Start MAVROS with use_sim_time=false and hardware settings.
        # TODO: Validate frame alignment, flight limits and onboard failsafes.
        LogInfo(msg='Real-run placeholder: MAVROS hardware setup is pending.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mission_executor'),
                'launch', 'outdoor_launch.py'])),
            launch_arguments={
                'enable_flight': LaunchConfiguration('enable_flight'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false',
            }.items()),
    ])
