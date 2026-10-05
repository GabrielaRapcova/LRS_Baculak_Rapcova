"""ROS action integration test; source install/setup.bash before running."""
import os
import signal
import subprocess
import time
os.environ['ROS_DOMAIN_ID'] = '175'
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from uav_navigation_msgs.action import ComputePath3D
from uav_navigation_msgs.msg import VoxelBlock, VoxelCostmap

process = subprocess.Popen(['ros2', 'run', 'astar_planner', 'astar_planner_node'], start_new_session=True)
rclpy.init()
node = rclpy.create_node('planner_smoke_test')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
publisher = node.create_publisher(VoxelCostmap, '/costmap', qos)
paths = []
subscription = node.create_subscription(Path, '/plan', paths.append, qos)
client = ActionClient(node, ComputePath3D, '/compute_path_3d')

def wait(condition):
    deadline = time.monotonic() + 15
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert condition(), 'ROS operation timed out'

def request(start, goal, frame='map'):
    message = ComputePath3D.Goal()
    for pose, point in [(message.start, start), (message.goal, goal)]:
        pose.header.frame_id = frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
        pose.pose.orientation.w = 1.0
    return message

def send(message):
    future = client.send_goal_async(message)
    wait(future.done)
    handle = future.result()
    assert handle.accepted
    result = handle.get_result_async()
    wait(result.done)
    return result.result()

try:
    assert client.wait_for_server(timeout_sec=10)
    assert send(request((1.5,1.5,1.5), (6.5,5.5,4.5))).result.status == ComputePath3D.Result.NO_MAP
    wait(lambda: publisher.get_subscription_count() > 0)
    snapshot = VoxelCostmap()
    snapshot.header.frame_id = 'map'
    snapshot.resolution = 1.0
    snapshot.map_id = 1
    block = VoxelBlock()
    block.occupancy = [0] * 512
    snapshot.blocks = [block]
    publisher.publish(snapshot)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    assert not paths, 'Costmap reception triggered planning'
    for start, goal in [((1.6,1.7,1.8), (6.6,5.7,4.8)), ((1.5,6.5,1.5), (6.5,1.5,6.5))]:
        outcome = send(request(start, goal))
        result = outcome.result
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert result.status == ComputePath3D.Result.SUCCESS and len(result.path.poses) == 2
        assert result.path.poses[0].pose.position.x == start[0]
        assert result.path.poses[-1].pose.position.z == goal[2]
        assert result.raw_point_count > result.refined_point_count
        print(f'Route: {result.raw_point_count} -> {result.refined_point_count} points; {result.planning_time:.4f} s')
    wait(lambda: len(paths) == 2)
    count = len(paths)
    snapshot.sequence += 1
    block.occupancy = [254] * 512
    publisher.publish(snapshot)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    assert len(paths) == count, 'Costmap update caused planning or path publication'
    assert send(request((1.5,1.5,1.5), (6.5,5.5,4.5))).result.status == ComputePath3D.Result.INVALID_START
    assert send(request((1.5,1.5,1.5), (6.5,5.5,4.5), 'odom')).result.status == ComputePath3D.Result.INVALID_START
    # Larger disconnected search supplies feedback and exercises asynchronous cancel.
    large = VoxelCostmap()
    large.header.frame_id = 'map'
    large.resolution = 1.0
    large.map_id = 2
    for z in range(8):
        for y in range(8):
            for x in range(8):
                b = VoxelBlock()
                b.x, b.y, b.z = x,y,z
                b.occupancy = [0] * 512
                if x == 4:
                    for iz in range(8):
                        for iy in range(8):
                            b.occupancy[8*(iy+8*iz)] = 254
                large.blocks.append(b)
    publisher.publish(large)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    feedback = []
    future = client.send_goal_async(request((1.5,1.5,1.5),(62.5,62.5,62.5)), feedback_callback=feedback.append)
    wait(future.done)
    handle = future.result()
    assert handle.accepted
    wait(lambda: bool(feedback))
    cancellation = handle.cancel_goal_async()
    wait(cancellation.done)
    assert cancellation.result().goals_canceling
    result = handle.get_result_async()
    wait(result.done)
    assert result.result().status == GoalStatus.STATUS_CANCELED
    assert result.result().result.status == ComputePath3D.Result.CANCELLED
    print('PASS: requests, action results, snapshot-only input, no automatic planning, feedback and cancellation')
finally:
    node.destroy_node()
    rclpy.shutdown()
    os.killpg(process.pid, signal.SIGINT)
    process.wait(timeout=10)
