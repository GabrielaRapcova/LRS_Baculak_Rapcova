"""Launch the outdoor mission with a Gazebo-to-ROS clock bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Reuse the common mission launch and keep flight disabled by default."""
    return LaunchDescription([
        DeclareLaunchArgument('enable_flight', default_value='false'),
        DeclareLaunchArgument('execute_square', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission_executor'),
                'config', 'outdoor.yaml'])),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='outdoor_clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mission_executor'),
                'launch', 'outdoor_launch.py'])),
            launch_arguments={
                'enable_flight': LaunchConfiguration('enable_flight'),
                'execute_square': LaunchConfiguration('execute_square'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'true',
            }.items()),
    ])
