"""Check ROS-time readiness and wall-time stall detection with a fake clock."""

import os
import signal
import subprocess
import time

from geometry_msgs.msg import PoseStamped, TwistStamped

from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import MessageInterval

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from rosgraph_msgs.msg import Clock

from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Float64, String, UInt32

os.environ['ROS_DOMAIN_ID'] = '184'
rclpy.init()
node = rclpy.create_node('outdoor_clock_test')


def accept_stream(request, response):
    """Accept stream configuration without changing the fake telemetry."""
    response.success = True
    return response


stream_service = node.create_service(
    MessageInterval, '/mavros/set_message_interval', accept_stream)
clock_pub = node.create_publisher(Clock, '/clock', 10)
state = State(connected=True)
extended = ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
fix = NavSatFix(latitude=48.15, longitude=17.07, altitude=150.0)
fix.status.status = 0
pose = PoseStamped()
pose.header.frame_id = 'map'
pose.pose.orientation.w = 1.0
fixtures = [
    ('state', State, state),
    ('extended_state', ExtendedState, extended),
    ('global_position/global', NavSatFix, fix),
    ('global_position/raw/satellites', UInt32, UInt32(data=12)),
    ('global_position/compass_hdg', Float64, Float64(data=90.0)),
    ('global_position/rel_alt', Float64, Float64()),
    ('local_position/pose', PoseStamped, pose),
    ('local_position/velocity_local', TwistStamped, TwistStamped()),
]
publishers = [(node.create_publisher(kind, '/mavros/' + topic, 10), msg)
              for topic, kind, msg in fixtures]
states = []
retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)
subscription = node.create_subscription(
    String, '/outdoor_mission/state',
    lambda msg: states.append(msg.data), retained)
process = subprocess.Popen([
    'ros2', 'run', 'mission_executor', 'outdoor_mission_node', '--ros-args',
    '-p', 'coordinate_mode:=enu',
    '-p', 'use_sim_time:=true', '-p', 'gps_ready_duration:=0.6',
    '-p', 'telemetry_timeout:=0.4', '-p', 'clock_stall_timeout:=1.0'],
    start_new_session=True)
sim_time = 10.0


def run_for(duration, speed=0.0, telemetry=True):
    """Drive ROS time independently of telemetry."""
    global sim_time
    previous = time.monotonic()
    deadline = previous + duration
    while time.monotonic() < deadline:
        current = time.monotonic()
        sim_time += (current - previous) * speed
        previous = current
        clock = Clock()
        clock.clock.sec = int(sim_time)
        clock.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
        clock_pub.publish(clock)
        rclpy.spin_once(node, timeout_sec=0.01)
        if telemetry:
            for publisher, message in publishers:
                publisher.publish(message)
        time.sleep(0.02)


try:
    deadline = time.monotonic() + 15.0
    while not all(pub.get_subscription_count() for pub, _ in publishers):
        assert time.monotonic() < deadline, 'Subscriptions missing'
        rclpy.spin_once(node, timeout_sec=0.05)
    run_for(0.3, speed=0.2)
    run_for(0.5)  # Pausing must not complete the readiness dwell.
    assert 'REFERENCE_READY' not in states, states
    run_for(3.5, speed=0.2)
    assert states[-1] == 'REFERENCE_READY', states
    run_for(0.5, telemetry=False)  # Wall time alone must not age telemetry.
    assert states[-1] == 'REFERENCE_READY', states
    run_for(0.8, telemetry=False)  # A prolonged clock stall is still detected.
    assert states[-1] == 'TELEMETRY_LOST', states
    run_for(3.5, speed=0.2)
    assert states[-1] == 'REFERENCE_READY', states
    run_for(0.6, speed=1.0, telemetry=False)
    assert states[-1] == 'TELEMETRY_LOST', states
    sim_time = 1.0  # A reset invalidates samples from the old timeline.
    run_for(0.2, speed=0.2, telemetry=False)
    assert states[-1] == 'TELEMETRY_LOST', states
    run_for(3.5, speed=0.2)
    assert states[-1] == 'REFERENCE_READY', states
    print('PASS: slow ROS clock, paused dwell, freshness; '
          'stall watchdog and reset recovery')
finally:
    node.destroy_node()
    rclpy.shutdown()
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    process.wait(timeout=10)
