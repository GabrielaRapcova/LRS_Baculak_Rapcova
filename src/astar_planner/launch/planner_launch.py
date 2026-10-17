from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('astar_planner'), 'config', 'astar.yaml'])),
        Node(package='astar_planner', executable='astar_planner_node', name='astar_planner',
             parameters=[LaunchConfiguration('params_file')], output='screen'),
    ])
