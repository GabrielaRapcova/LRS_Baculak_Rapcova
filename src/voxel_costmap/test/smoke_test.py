"""Integration test: run after sourcing the built workspace."""
import os
import signal
import subprocess
import time
os.environ['ROS_DOMAIN_ID'] = '173'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from uav_navigation_msgs.msg import VoxelBlock, VoxelMap, VoxelMapUpdate, VoxelCostmap
from uav_navigation_msgs.srv import CheckCollision

inflation_width = float(os.environ.get('COSTMAP_TEST_INFLATION_WIDTH', '2.0'))
assert inflation_width in (0.0, 2.0)

process = subprocess.Popen(['ros2', 'run', 'voxel_costmap', 'voxel_costmap_node', '--ros-args',
    '-p', 'vehicle_radius:=1.0', '-p', 'position_tolerance:=0.0', '-p', 'safety_margin:=0.0',
    '-p', f'inflation_radius:={inflation_width}'], start_new_session=True)
rclpy.init()
node = rclpy.create_node('costmap_smoke_test')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
publisher = node.create_publisher(VoxelMap, '/map', qos)
updates = node.create_publisher(VoxelMapUpdate, '/map_updates', 10)
received = []
subscription = node.create_subscription(VoxelCostmap, '/costmap', received.append, qos)
client = node.create_client(CheckCollision, '/check_collision')

def wait(condition):
    deadline = time.monotonic() + 10
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert condition(), 'Timed out waiting for costmap'

def block(x, occupied=False):
    result = VoxelBlock()
    result.x = x
    result.occupancy = [0] * 512
    if occupied:
        result.occupancy[7 + 8 * (3 + 8 * 3)] = 254
    return result

def query(point):
    request = CheckCollision.Request()
    request.point.x, request.point.y, request.point.z = point
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    assert future.done()
    return future.result()

def at(snapshot, x, y, z):
    for b in snapshot.blocks:
        if (b.x, b.y, b.z) == (x // 8, y // 8, z // 8):
            return b.occupancy[x % 8 + 8 * (y % 8 + 8 * (z % 8))]
    return 255

try:
    assert client.wait_for_service(timeout_sec=10)
    assert query((0., 0., 0.)).collision  # no map yet
    wait(lambda: publisher.get_subscription_count() > 0)
    snapshot = VoxelMap()
    snapshot.header.frame_id = 'map'
    snapshot.resolution = 1.0
    snapshot.origin.x = 10.0
    snapshot.map_id = 42
    snapshot.blocks = [block(0, True), block(1)]
    # Keep a specific adjacent voxel unknown.
    snapshot.blocks[1].occupancy[8 * (3 + 8 * 3)] = 255
    publisher.publish(snapshot)
    wait(lambda: len(received) > 0)
    costmap = received[-1]
    # Identical upstream republication must not emit another costmap.
    count = len(received)
    for _ in range(3):
        publisher.publish(snapshot)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
    assert len(received) == count, 'Duplicate map caused costmap republication'
    assert costmap.origin.x == 10.0 and costmap.resolution == 1.0 and costmap.map_id == 42
    assert at(costmap, 7, 3, 3) == 254
    assert at(costmap, 7, 3, 4) == 253  # vertical clearance
    if inflation_width:
        assert at(costmap, 8, 4, 3) in range(1, 253)  # block edge, diagonal
        assert at(costmap, 9, 3, 3) in range(1, 253)  # inside the soft zone
        assert at(costmap, 10, 3, 3) in range(1, 253)  # 1 m clearance + 2 m width
        assert at(costmap, 11, 3, 3) == 0  # beyond the combined radius
    else:
        assert at(costmap, 8, 4, 3) == 0
        assert at(costmap, 9, 3, 3) == 0
        assert all(v in (0, 253, 254, 255) for b in costmap.blocks for v in b.occupancy)
    assert at(costmap, 8, 3, 3) == 255  # preserved unknown
    assert query((17.5, 3.5, 4.5)).collision
    assert query((19.5, 3.5, 3.5)).collision is False
    assert query((100., 100., 100.)).cost == 255
    # Remove obstacle and verify both its cell and neighboring inflation clear.
    update = VoxelMapUpdate()
    update.header.frame_id = 'map'
    update.map_id = 42
    update.sequence = 1
    update.blocks = [block(0)]
    updates.publish(update)
    wait(lambda: received[-1].sequence == 1)
    assert at(received[-1], 7, 3, 3) == 0 and at(received[-1], 9, 3, 3) == 0
    # Stale update ignored.
    update.blocks = [block(0, True)]
    updates.publish(update)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    assert query((17.5, 3.5, 3.5)).cost == 0
    # Lost sequence invalidates costs, then a full snapshot recovers.
    update.sequence = 3
    updates.publish(update)
    wait(lambda: received[-1].sequence == 2)
    assert not received[-1].blocks
    assert query((17.5, 3.5, 3.5)).cost == 255
    snapshot.sequence = 3
    publisher.publish(snapshot)
    wait(lambda: received[-1].sequence == 3)
    assert at(received[-1], 7, 3, 3) == 254
    # All-unknown replacement removes a block and emits an explicit clear.
    update.sequence = 4
    cleared = block(0)
    cleared.occupancy = [255] * 512
    update.blocks = [cleared]
    updates.publish(update)
    wait(lambda: received[-1].sequence == 4)
    assert at(received[-1], 7, 3, 3) == 255
    assert at(received[-1], 9, 3, 3) == 0
    # New epoch inherits new resolution and resets output sequencing.
    snapshot.map_id = 43
    snapshot.sequence = 0
    snapshot.resolution = 0.5
    publisher.publish(snapshot)
    wait(lambda: received[-1].map_id == 43)
    assert received[-1].sequence == 0 and received[-1].resolution == 0.5
    # Changing geometry within an epoch is rejected and invalidates the old costs.
    snapshot.resolution = 1.0
    publisher.publish(snapshot)
    wait(lambda: received[-1].map_id == 43 and not received[-1].blocks)
    assert query((17.5, 3.5, 3.5)).cost == 255
    snapshot.resolution = 0.5
    publisher.publish(snapshot)
    wait(lambda: bool(received[-1].blocks))
    # A malformed update cannot leave a usable retained costmap.
    update.map_id = 43
    update.sequence = 1
    invalid = block(0)
    invalid.occupancy[0] = 42
    update.blocks = [invalid]
    updates.publish(update)
    wait(lambda: not received[-1].blocks)
    publisher.publish(snapshot)
    wait(lambda: bool(received[-1].blocks))
    late = []
    late_subscription = node.create_subscription(VoxelCostmap, '/costmap', late.append, qos)
    wait(lambda: bool(late))
    assert late[-1].map_id == 43 and late[-1].blocks
    assert node.count_publishers('/costmap_updates') == 0
    # Wait longer than the previous default 5-second republication interval.
    count = len(received)
    deadline = time.monotonic() + 5.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert len(received) == count, 'Unchanged costmap was periodically republished'
    print('PASS: 3D inflation, block edges, unknown policy, removal, snapshot transport, stale/gapped updates, recovery, geometry reset')
finally:
    node.destroy_node()
    rclpy.shutdown()
    os.killpg(process.pid, signal.SIGINT)
    process.wait(timeout=10)
