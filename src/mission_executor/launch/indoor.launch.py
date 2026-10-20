"""Configure existing MAVROS, then launch indoor alignment and optional flight."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as ParameterNode
from rclpy.parameter_client import AsyncParameterClient


def configure_mavros(launch_context):
    """Apply the local frame settings before starting the mission executor."""
    parameter_file = PathJoinSubstitution([
        FindPackageShare('mission_executor'), 'config', 'indoor_mavros.yaml'
    ]).perform(launch_context)
    ros_context = Context()
    rclpy.init(context=ros_context)
    node = ParameterNode('configure_indoor_mavros', context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    try:
        client = AsyncParameterClient(node, '/mavros/local_position')
        if not client.wait_for_services(timeout_sec=10.0):
            raise RuntimeError('MAVROS unavailable. Start scripts/run_mavros.sh first.')
        future = client.load_parameter_file_atomically(parameter_file, use_wildcard=True)
        rclpy.spin_until_future_complete(node, future, executor=executor, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError('Timed out configuring MAVROS local-position frames.')
        result = future.result().result
        if not result.successful:
            raise RuntimeError('MAVROS rejected frame parameters: ' + result.reason)
        node.get_logger().info('MAVROS local-position frame configured as mavros_local.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ros_context)


def generate_launch_description():
    """Keep the existing map-to-local rotation and require explicit flight enablement."""
    enable_flight = LaunchConfiguration('enable_flight')
    sequence_only = PythonExpression(["'", enable_flight, "'.lower() != 'true'"])
    return LaunchDescription([
        DeclareLaunchArgument('mission_file', description='Absolute path to indoor CSV mission'),
        DeclareLaunchArgument('enable_flight', default_value='false'),
        DeclareLaunchArgument('task_flight_confirmed', default_value='false'),
        DeclareLaunchArgument('task_planner_frame_confirmed', default_value='false'),
        DeclareLaunchArgument('task_slow_sitl_telemetry', default_value='false'),
        OpaqueFunction(function=configure_mavros),
        # Inverse of the executor's map-to-local conversion: (-y, x, z).
        # Both origins must coincide; this launch is specific to the current world.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='indoor_map_to_mavros_local',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '-1.5707963267948966', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'mavros_local'],
            output='screen'),
        Node(
            package='mission_executor', executable='mission_executor_node',
            name='mission_executor_node',
            parameters=[{
                'mission_file': ParameterValue(
                    LaunchConfiguration('mission_file'), value_type=str),
                'mavros_frame': 'mavros_local',
                'enable_task_sequence_check': ParameterValue(sequence_only, value_type=bool),
                'enable_task_flight': ParameterValue(enable_flight, value_type=bool),
                'enable_task_planned_flight': ParameterValue(enable_flight, value_type=bool),
                'task_flight_confirmed': ParameterValue(
                    LaunchConfiguration('task_flight_confirmed'), value_type=bool),
                'task_planner_frame_confirmed': ParameterValue(
                    LaunchConfiguration('task_planner_frame_confirmed'), value_type=bool),
                'task_slow_sitl_telemetry': ParameterValue(
                    LaunchConfiguration('task_slow_sitl_telemetry'), value_type=bool),
            }],
            output='screen'),
    ])
