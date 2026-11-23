"""Test outdoor diagnostics using fake telemetry."""

import argparse
import math
import os
import signal
import subprocess
import time

from diagnostic_msgs.msg import DiagnosticArray

from geometry_msgs.msg import PoseStamped, TwistStamped

from mavros_msgs.msg import ExtendedState, HomePosition, State
from mavros_msgs.srv import MessageInterval

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Float64, String, UInt32

os.environ['ROS_DOMAIN_ID'] = '181'

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--no-home', action='store_true')
parser.add_argument('--mavros-prefix', default='/mavros')
parser.add_argument('--coordinate-mode', choices=['enu', 'gps'], default='enu')
args = parser.parse_args()
use_home = not args.no_home
prefix = '/' + args.mavros_prefix.strip('/')
gps_parameters = []
if args.coordinate_mode == 'gps':
    # Same 5 m square and POI as the ENU fixture, at its GPS reference.
    for index, coordinates in enumerate([
            [48.15004496570, 17.07000000000],
            [48.15004496568, 17.07006719486],
            [48.14999999998, 17.07006719480],
            [48.15000000000, 17.07000000000]], start=1):
        gps_parameters.extend(['-p', f'square_corner_{index}_gps:={coordinates}'])
    gps_parameters.extend(['-p', 'poi_gps:=[48.15002248285, 17.07003359742]'])
process = subprocess.Popen(
    ['ros2', 'run', 'mission_executor', 'outdoor_mission_node', '--ros-args',
     '-p', f'coordinate_mode:={args.coordinate_mode}',
     '-p', f'mavros_prefix:={args.mavros_prefix}',
     '-p', 'gps_ready_duration:=0.2', '-p', 'telemetry_timeout:=0.5'] + gps_parameters,
    start_new_session=True)
rclpy.init()
node = rclpy.create_node('outdoor_diagnostics_test')
interval_requests = []


def request_interval(request, response):
    """Accept the one-shot telemetry request."""
    interval_requests.append((request.message_id, request.message_rate))
    response.success = True
    return response


interval_service = node.create_service(
    MessageInterval, prefix + '/set_message_interval', request_interval)
retained = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)
states = []
diagnostics = []
state_subscription = node.create_subscription(
    String, '/outdoor_mission/state',
    lambda msg: states.append(msg.data), retained)
diagnostic_subscription = node.create_subscription(
    DiagnosticArray, '/outdoor_mission/diagnostics', diagnostics.append, 10)

state = State()
state.connected = True
state.mode = 'STABILIZE'
extended = ExtendedState()
extended.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
fix = NavSatFix()
fix.latitude = 48.15
fix.longitude = 17.07
fix.altitude = 150.0
fix.status.status = 0
pose = PoseStamped()
pose.header.frame_id = 'map'
pose.pose.position.x = 10.0
pose.pose.position.y = 20.0
pose.pose.position.z = 3.0
pose.pose.orientation.w = 1.0
velocity = TwistStamped()
heading = Float64(data=90.0)  # East, matching identity quaternion yaw.
relative_altitude = Float64(data=0.0)
satellites = UInt32(data=255)
home = HomePosition()
home.geo.latitude = fix.latitude
home.geo.longitude = fix.longitude
home.geo.altitude = fix.altitude
home.position = pose.pose.position

telemetry = [
    ('state', State, state),
    ('extended_state', ExtendedState, extended),
    ('global_position/global', NavSatFix, fix),
    ('global_position/raw/satellites', UInt32, satellites),
    ('global_position/compass_hdg', Float64, heading),
    ('global_position/rel_alt', Float64, relative_altitude),
    ('local_position/pose', PoseStamped, pose),
    ('local_position/velocity_local', TwistStamped, velocity),
]
publishers = []
for topic, message_type, message in telemetry:
    publisher = node.create_publisher(message_type, prefix + '/' + topic, 10)
    publishers.append((publisher, message))
if use_home:
    home_publisher = node.create_publisher(
        HomePosition, prefix + '/home_position/home', retained)
    publishers.append((home_publisher, home))


def spin_for(duration, publish=True):
    """Publish fixtures and receive diagnostics for a bounded duration."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if publish:
            for publisher, message in publishers:
                publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.03)
        time.sleep(0.02)


def latest_values():
    """Return named values from the latest diagnostic report."""
    assert diagnostics, 'No diagnostics received'
    entries = diagnostics[-1].status[0].values
    return {entry.key: entry.value for entry in entries}


try:
    deadline = time.monotonic() + 10.0
    while not all(pub.get_subscription_count() for pub, _ in publishers):
        assert time.monotonic() < deadline, 'Telemetry subscriptions missing'
        rclpy.spin_once(node, timeout_sec=0.05)
    spin_for(1.5)
    assert 'REFERENCE_READY' not in states, 'Unknown satellites accepted'
    assert 'satellites' in diagnostics[-1].status[0].message
    satellites.data = 4
    spin_for(0.5)
    assert 'REFERENCE_READY' not in states, 'Too few satellites accepted'
    satellites.data = 12
    fix.status.status = -1
    spin_for(0.5)
    assert 'REFERENCE_READY' not in states, 'Invalid GPS fix accepted'
    fix.status.status = 0
    spin_for(1.5)
    assert states[-1] == 'REFERENCE_READY', states
    values = latest_values()
    expected_source = 'fcu_home' if use_home else 'qualified_fix'
    assert values['origin_source'] == expected_source, values
    assert math.isclose(float(values['flight_local_z']), 8.0)
    assert abs(float(values['yaw_alignment_error_degrees'])) < 1e-6
    assert math.isclose(
        float(values['horizontal_bound_with_margin']),
        math.sqrt(50) + 0.5, abs_tol=1e-6)
    assert values['coordinate_mode'] == args.coordinate_mode
    original_latitude = values['origin_latitude']
    assert node.count_publishers(prefix + '/setpoint_raw/local') == 0
    assert node.count_publishers(prefix + '/setpoint_position/local') == 0
    clients = node.get_client_names_and_types_by_node(
        'outdoor_mission_node', '/')
    assert clients == [(prefix + '/set_message_interval',
                        ['mavros_msgs/srv/MessageInterval'])], clients
    assert interval_requests == [(245, 2.0)], interval_requests
    state.connected = False
    spin_for(0.4)
    state.connected = True
    spin_for(1.2)
    assert interval_requests == [(245, 2.0)], interval_requests
    # Motion is observed only; it is never commanded by the node.
    pose.pose.position.z = 8.0
    relative_altitude.data = 5.0
    spin_for(1.2)
    values = latest_values()
    assert math.isclose(float(values['height_above_launch_local']), 5.0)
    assert math.isclose(float(values['height_above_launch_relative']), 5.0)
    # Invalid attitude invalidates readiness, then valid data can recover.
    pose.pose.orientation.w = 0.0
    spin_for(0.3)
    assert states[-1] == 'TELEMETRY_LOST'
    pose.pose.orientation.w = 1.0
    heading.data = 0.0
    spin_for(1.2)
    assert states[-1] == 'REFERENCE_READY'
    assert math.isclose(
        float(latest_values()['yaw_alignment_error_degrees']), -90.0)
    # Lost streams invalidate readiness but never recapture a shifted origin.
    spin_for(1.2, publish=False)
    assert states[-1] == 'TELEMETRY_LOST'
    fix.latitude += 0.00001
    spin_for(1.5)
    assert states[-1] == 'REFERENCE_READY'
    assert latest_values()['origin_latitude'] == original_latitude
    print(f'PASS: {expected_source}, GPS gating, heading/altitude reports; '
          'stale recovery, frozen origin, no flight interfaces')
finally:
    node.destroy_node()
    rclpy.shutdown()
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    process.wait(timeout=10)
