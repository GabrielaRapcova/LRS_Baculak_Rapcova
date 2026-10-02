from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file', description='Path to the static PCD map'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('mapper'), 'config', 'params.yaml'])),
        Node(package='mapper', executable='static_mapper', name='static_mapper',
             parameters=[LaunchConfiguration('params_file'),
                         {'map_file': LaunchConfiguration('map_file')}], output='screen'),
    ])
