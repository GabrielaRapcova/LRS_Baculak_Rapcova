from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapper',
            executable='mapper_node',
            namespace='pepsi',
            name='mapper',
            parameters=[PathJoinSubstitution([
                FindPackageShare('mapper'), 'config', 'params.yaml'])
            ],
        ),
    ])