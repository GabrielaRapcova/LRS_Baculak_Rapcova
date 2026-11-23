"""Run the indoor mission and publish its frame alignment for RViz."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Matches the executor's map-to-local conversion: (-y, x, z).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='indoor_map_to_mavros_local',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '-1.5707963267948966', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'mavros_local',
            ],
            output='screen',
        ),
        Node(
            package='mission_executor',
            executable='mission_executor_node',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('mission_executor'),
                    'config', 'mission_executor.yaml',
                ]),
                {
                    'mission_file': '/home/user/LRS-URK/missions/exam_example.csv',
                    'mavros_frame': 'mavros_local',
                    'enable_task_sequence_check': False,
                    'enable_task_flight': True,
                    'task_flight_confirmed': True,
                    'task_slow_sitl_telemetry': True,
                    'enable_task_planned_flight': True,
                    'task_planner_frame_confirmed': True,
                    'task_planned_max_z_m': 4.0,
                    'task_path_point_tolerance_m': 0.50,
                    'enable_dynamic_start': True,
                    'dynamic_start_clearance_confirmed': True,
                },
            ],
            output='screen',
        ),
    ])
