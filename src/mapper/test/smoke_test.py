"""Run after building and sourcing install/setup.bash; uses an isolated ROS domain."""
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '171'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from uav_navigation_msgs.msg import VoxelMap
from uav_navigation_msgs.srv import CheckCollision

with tempfile.TemporaryDirectory() as directory:
    fixture = Path(directory) / 'test.pcd'
    fixture.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 4\nHEIGHT 1\nPOINTS 4\nDATA ascii\n-0.1 0.1 0.1\n0.1 0.1 0.1\n0.2 0.2 0.2\nnan 0 0\n')
    process = subprocess.Popen(['ros2', 'run', 'mapper', 'static_mapper', '--ros-args',
        '-p', f'map_file:={fixture}', '-p', 'resolution:=1.0',
        '-p', 'known_free_min:=[-1.0, -1.0, -1.0]',
        '-p', 'known_free_max:=[2.0, 2.0, 2.0]',
        '-p', 'publish_frequency:=0.01'], start_new_session=True)
    node = None
    try:
        time.sleep(2)  # Subscribe after initial publication to verify retained snapshot.
        rclpy.init()
        node = rclpy.create_node('static_mapper_smoke_test')
        received = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        subscription = node.create_subscription(VoxelMap, '/map', received.append, qos)
        deadline = time.monotonic() + 10
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert received, 'Late subscriber did not receive snapshot'
        snapshot = received[0]
        assert snapshot.resolution == 1.0 and snapshot.header.frame_id == 'map'
        assert sum(list(b.occupancy).count(254) for b in snapshot.blocks) == 2
        assert sum(list(b.occupancy).count(0) for b in snapshot.blocks) == 25
        client = node.create_client(CheckCollision, '/query_occupancy')
        assert client.wait_for_service(timeout_sec=5)
        for point, cost in [((-0.1, 0.1, 0.1), 254), ((1.1, 1.1, 1.1), 0),
                            ((2.1, 1.1, 1.1), 255), ((99., 0., 0.), 255)]:
            request = CheckCollision.Request()
            request.point.x, request.point.y, request.point.z = point
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done(), 'Query timed out'
            response = future.result()
            assert response.cost == cost and response.collision == (cost != 0)
        print('PASS: voxelization, finite filtering, free bounds, queries, late subscriber')
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
