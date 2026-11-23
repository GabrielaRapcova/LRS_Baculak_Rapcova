from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('voxel_costmap'), 'config', 'costmap.yaml'])),
        Node(package='voxel_costmap', executable='voxel_costmap_node',
             name='voxel_costmap', parameters=[LaunchConfiguration('params_file')],
             output='screen'),
    ])
