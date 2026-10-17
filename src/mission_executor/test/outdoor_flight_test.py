"""Exercise takeoff/return and failures against a fake MAVROS vehicle."""

import math
import os
import signal
import statistics
import subprocess
import sys
import time

from geometry_msgs.msg import PoseStamped, TwistStamped

from mavros_msgs.msg import ExtendedState, HomePosition, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Float64, String, UInt32

from std_srvs.srv import Trigger

os.environ['ROS_DOMAIN_ID'] = '183'


class FakeVehicle(Node):
    """Provide telemetry and command acknowledgements without hardware."""

    def __init__(self, scenario):
        """Create the fake vehicle and ROS interfaces."""
        super().__init__('fake_outdoor_vehicle')
        self.scenario = scenario
        self.mode = 'STABILIZE'
        self.armed = False
        self.connected = True
        self.gps_valid = True
        self.local_frame = 'map'
        self.home_shift = 0.0
        self.x = 10.0
        self.y = 20.0
        self.z = 3.0
        self.yaw = 0.0
        self.heading_bias = 90.0 if scenario == 'heading_disagreement' else 0.0
        self.desired_z = 3.0
        self.target = None
        self.calls = []
        self.samples = []
        self.return_samples = []
        self.square_samples = []
        self.curve_samples = []
        self.freeze_position = False
        self.states = []
        self.violations = []
        self.last_tick = time.monotonic()
        self.suppress_local = False
        self.mode_service = self.create_service(
            SetMode, '/mavros/set_mode', self.set_mode)
        self.arm_service = self.create_service(
            CommandBool, '/mavros/cmd/arming', self.arm)
        self.takeoff_service = self.create_service(
            CommandTOL, '/mavros/cmd/takeoff', self.takeoff)
        self.target_subscription = self.create_subscription(
            PositionTarget, '/mavros/setpoint_raw/local', self.setpoint, 100)
        retained = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.state_subscription = self.create_subscription(
            String, '/outdoor_mission/state',
            lambda msg: self.states.append(msg.data), retained)
        self.abort_client = self.create_client(
            Trigger, '/outdoor_mission/abort')
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.extended_pub = self.create_publisher(
            ExtendedState, '/mavros/extended_state', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/local_position/pose', 10)
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/mavros/local_position/velocity_local', 10)
        self.fix_pub = self.create_publisher(
            NavSatFix, '/mavros/global_position/global', 10)
        self.heading_pub = self.create_publisher(
            Float64, '/mavros/global_position/compass_hdg', 10)
        self.altitude_pub = self.create_publisher(
            Float64, '/mavros/global_position/rel_alt', 10)
        self.satellites_pub = self.create_publisher(
            UInt32, '/mavros/global_position/raw/satellites', 10)
        self.home_pub = self.create_publisher(
            HomePosition, '/mavros/home_position/home', retained)
        self.timer = self.create_timer(0.02, self.tick)

    def set_mode(self, request, response):
        """Acknowledge mode requests; optionally leave telemetry unchanged."""
        self.calls.append(request.custom_mode)
        response.mode_sent = True
        if request.custom_mode == 'LAND' and self.scenario == 'reject_land':
            response.mode_sent = False
            return response
        if self.scenario != 'unconfirmed_mode':
            self.mode = request.custom_mode
        return response

    def arm(self, request, response):
        """Refuse unsafe disarm and model rejected/unconfirmed arming."""
        self.calls.append('ARM' if request.value else 'DISARM')
        if not request.value and self.z > 3.01:
            self.violations.append('Disarm requested while airborne')
        response.success = not (
            request.value and self.scenario == 'reject_arm')
        if response.success and self.scenario != 'unconfirmed_arm':
            self.armed = request.value
        return response

    def takeoff(self, request, response):
        """Check home-relative altitude and optionally refuse takeoff."""
        self.calls.append('TAKEOFF')
        if not self.armed or self.mode != 'GUIDED':
            self.violations.append('Takeoff before GUIDED/armed confirmation')
        if not math.isclose(request.altitude, 6.2, abs_tol=1e-5):
            self.violations.append('Takeoff altitude ignored home offset')
        response.success = self.scenario != 'reject_takeoff'
        if response.success:
            self.desired_z = 8.0
        return response

    def setpoint(self, message):
        """Record timing and follow position/yaw targets in the fake model."""
        self.samples.append((time.monotonic(), message))
        self.target = message
        if self.states and self.states[-1] == 'RETURN_HOME':
            self.return_samples.append((time.monotonic(), message))
        if self.states and self.states[-1] == 'SQUARE':
            self.square_samples.append((
                time.monotonic(), message, self.x, self.y, self.yaw))
        if self.states and self.states[-1] == 'FIGURE_EIGHT':
            self.curve_samples.append(message)

    def tick(self):
        """Publish coherent telemetry and integrate a simple vertical model."""
        current_time = time.monotonic()
        dt = min(current_time - self.last_tick, 0.1)
        self.last_tick = current_time
        previous_x = self.x
        previous_y = self.y
        previous_z = self.z
        if self.mode == 'LAND':
            if self.scenario != 'landing_timeout':
                self.z = max(3.0, self.z - 5.0 * dt)
        elif self.mode == 'GUIDED' and self.armed:
            destination = self.desired_z
            if self.target is not None:
                if not self.freeze_position:
                    self.x = self.target.position.x
                    self.y = self.target.position.y
                destination = self.target.position.z
                difference = self.target.yaw - self.yaw
                self.yaw += math.atan2(
                    math.sin(difference), math.cos(difference))
            step = max(-5.0 * dt, min(5.0 * dt, destination - self.z))
            self.z += step
        if (self.scenario == 'return_drift' and self.states and
                self.states[-1] == 'STABILIZE'):
            self.x = 10.2  # Small disturbance inside arrival tolerance.
        state = State()
        state.connected = self.connected
        state.armed = self.armed
        state.mode = self.mode
        self.state_pub.publish(state)
        extended = ExtendedState()
        extended.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        if self.z > 3.01:
            extended.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        self.extended_pub.publish(extended)
        pose = PoseStamped()
        pose.header.frame_id = self.local_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        if not self.suppress_local:
            self.pose_pub.publish(pose)
        velocity = TwistStamped()
        velocity.twist.linear.x = (self.x - previous_x) / max(dt, 1e-6)
        velocity.twist.linear.y = (self.y - previous_y) / max(dt, 1e-6)
        velocity.twist.linear.z = (self.z - previous_z) / max(dt, 1e-6)
        self.velocity_pub.publish(velocity)
        fix = NavSatFix()
        fix.status.status = 0 if self.gps_valid else -1
        fix.latitude = 48.15 + (self.y - 20.0) / 111190.0
        fix.longitude = 17.07 + (self.x - 10.0) / (
            111320.0 * math.cos(math.radians(48.15)))
        fix.altitude = 150.0 + self.z - 3.0
        self.fix_pub.publish(fix)
        self.heading_pub.publish(Float64(
            data=(90.0 - self.yaw * 180.0 / math.pi +
                  self.heading_bias) % 360.0))
        self.altitude_pub.publish(Float64(data=1.2 + self.z - 3.0))
        self.satellites_pub.publish(UInt32(data=12))
        if self.scenario != 'no_home':
            home = HomePosition()
            home.geo.latitude = 48.15
            home.geo.longitude = 17.07 + self.home_shift
            home.geo.altitude = 148.8
            home.position.x = 10.0
            home.position.y = 20.0
            home.position.z = 1.8
            self.home_pub.publish(home)


def run_scenario(scenario):
    """Check one mission outcome and its command history."""
    vehicle = FakeVehicle(scenario)
    settle_duration = 0.4
    if scenario == 'stale_pose':
        settle_duration = 1.2
    full_mission = scenario.startswith('full')
    run_square = scenario.startswith('square') or full_mission
    phase_timeout = 45.0 if full_mission else 6.0
    command = [
        'ros2', 'run', 'mission_executor', 'outdoor_mission_node',
        '--ros-args',
        '-p', 'enable_flight:=true', '-p', 'gps_ready_duration:=0.2',
        '-p', f'execute_square:={str(run_square).lower()}',
        '-p', f'execute_figure_eight:={str(full_mission).lower()}',
        '-p', f'settle_duration:={settle_duration}',
        '-p', 'command_timeout:=1.5',
        '-p', 'takeoff_timeout:=2.5', '-p', f'phase_timeout:={phase_timeout}',
        '-p', 'landing_timeout:=6.0', '-p', 'telemetry_timeout:=0.8']
    process = subprocess.Popen(command, start_new_session=True)
    abort_future = None
    event_triggered = False
    expected = 'ABORTED'
    if scenario in ('complete', 'return_drift', 'square',
                    'heading_disagreement', 'full'):
        expected = 'DONE'
    if scenario in ('reject_land', 'landing_timeout'):
        expected = 'FAILED'
    if scenario in ('pilot_override', 'connection_loss', 'full_override'):
        expected = 'RELEASE_CONTROL'
    deadline = time.monotonic() + 110.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(vehicle, timeout_sec=0.01)
            trigger_state = 'STABILIZE'
            if scenario.startswith('square'):
                trigger_state = 'SQUARE'
            if full_mission:
                trigger_state = 'FIGURE_EIGHT'
            if (trigger_state in vehicle.states and not event_triggered):
                event_triggered = True
                if scenario in ('abort', 'square_abort', 'full_abort'):
                    assert vehicle.abort_client.service_is_ready()
                    abort_future = vehicle.abort_client.call_async(
                        Trigger.Request())
                elif scenario in ('pilot_override', 'full_override'):
                    vehicle.mode = 'LOITER'
                elif scenario == 'full_tracking':
                    vehicle.freeze_position = True
                elif scenario == 'connection_loss':
                    vehicle.connected = False
                elif scenario == 'bad_gps':
                    vehicle.gps_valid = False
                elif scenario == 'stale_pose':
                    vehicle.suppress_local = True
                elif scenario == 'home_change':
                    vehicle.home_shift = 0.00001
                elif scenario == 'frame_change':
                    vehicle.local_frame = 'reset_map'
            if vehicle.states and vehicle.states[-1] in (
                    'DONE', 'ABORTED', 'RELEASE_CONTROL', 'FAILED'):
                break
            assert process.poll() is None, 'Controller exited unexpectedly'
        assert vehicle.states and vehicle.states[-1] == expected, (
            scenario, vehicle.states, vehicle.calls)
        assert not vehicle.violations, vehicle.violations
        assert vehicle.calls.count('ARM') <= 1
        assert vehicle.calls.count('TAKEOFF') <= 1
        if scenario in ('reject_arm', 'unconfirmed_arm',
                        'unconfirmed_mode', 'no_home'):
            assert 'TAKEOFF' not in vehicle.calls
            assert not vehicle.samples
        if scenario == 'unconfirmed_mode':
            assert 'ARM' not in vehicle.calls
        if scenario in ('pilot_override', 'connection_loss', 'full_override'):
            assert 'LAND' not in vehicle.calls
            count = len(vehicle.samples)
            until = time.monotonic() + 0.4
            while time.monotonic() < until:
                rclpy.spin_once(vehicle, timeout_sec=0.01)
            assert len(vehicle.samples) <= count + 1
        elif expected == 'FAILED':
            assert 'DISARM' not in vehicle.calls
            assert vehicle.armed
        else:
            assert not vehicle.armed
        if scenario in ('complete', 'return_drift', 'square',
                        'heading_disagreement', 'full'):
            assert vehicle.calls == ['GUIDED', 'ARM', 'TAKEOFF', 'LAND',
                                     'DISARM'], vehicle.calls
            assert len(vehicle.samples) >= 15
            intervals = [b[0] - a[0] for a, b in zip(
                vehicle.samples, vehicle.samples[1:])]
            assert 0.015 < statistics.median(intervals) < 0.06
            for _, message in vehicle.samples:
                assert math.isclose(message.position.z, 8.0)
                assert not (message.type_mask & (
                    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                    PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_YAW))
                assert message.type_mask & PositionTarget.IGNORE_YAW_RATE
        if scenario == 'return_drift':
            assert len(vehicle.return_samples) >= 3
            assert any(sample.position.x > 10.1 for _, sample in
                       vehicle.return_samples)
            assert abs(vehicle.return_samples[-1][1].position.x - 10.0) < 0.01
            for previous, following in zip(
                    vehicle.return_samples, vehicle.return_samples[1:]):
                elapsed = following[0] - previous[0]
                step = abs(following[1].position.x - previous[1].position.x)
                assert step <= elapsed + 0.04, (step, elapsed)
        if scenario == 'square':
            assert vehicle.states.count('SQUARE') == 5, vehicle.states
            corners = [(10.0, 25.0), (15.0, 25.0), (15.0, 20.0),
                       (10.0, 20.0), (10.0, 25.0)]
            corner_index = 0
            hold_started = None
            for stamp, message, x, y, yaw in vehicle.square_samples:
                expected_yaw = math.atan2(22.5 - y, 12.5 - x)
                error = math.atan2(math.sin(message.yaw - expected_yaw),
                                   math.cos(message.yaw - expected_yaw))
                assert abs(error) < 0.04, error
                destination = corners[corner_index]
                if math.hypot(x - destination[0], y - destination[1]) < 0.01:
                    if hold_started is None:
                        hold_started = stamp
                    if stamp - hold_started >= 0.3:
                        corner_index += 1
                        hold_started = None
                        if corner_index == len(corners):
                            break
                else:
                    hold_started = None
            assert corner_index == 5, corner_index
            for previous, following in zip(
                    vehicle.square_samples, vehicle.square_samples[1:]):
                elapsed = following[0] - previous[0]
                step = math.hypot(
                    following[1].position.x - previous[1].position.x,
                    following[1].position.y - previous[1].position.y)
                assert step <= elapsed + 0.04, (step, elapsed)
        if scenario == 'full':
            phases = ['SQUARE', 'MOVE_TO_CENTER', 'ALIGN_TANGENT',
                      'FIGURE_EIGHT', 'SETTLE_CENTER', 'RETURN_HOME', 'LAND']
            indices = [vehicle.states.index(phase) for phase in phases]
            assert indices == sorted(indices), vehicle.states
            assert vehicle.states.count('FIGURE_EIGHT') == 1
            samples = vehicle.curve_samples
            assert len(samples) > 500, len(samples)
            # A queued alignment hold can arrive after the phase notification.
            while abs(samples[0].position.x - 12.5) < 0.02:
                samples = samples[1:]
            initial_phase = math.asin((samples[0].position.x - 12.5) / 3.0)
            first_stamp = samples[0].header.stamp
            first_time = first_stamp.sec + first_stamp.nanosec * 1e-9
            for message in samples:
                stamp = message.header.stamp
                elapsed = stamp.sec + stamp.nanosec * 1e-9 - first_time
                phase = min(initial_phase + 0.2 * elapsed, 2.0 * math.pi)
                assert abs(message.position.x - (
                    12.5 + 3.0 * math.sin(phase))) < 0.02, (phase, message)
                assert abs(message.position.y - (
                    22.5 + 3.0 * math.sin(phase) * math.cos(phase))) < 0.02
                yaw = math.atan2(math.cos(2.0 * phase), math.cos(phase))
                assert abs(math.atan2(math.sin(message.yaw - yaw),
                                      math.cos(message.yaw - yaw))) < 0.02
            assert 31.2 < elapsed < 31.6, elapsed
            assert abs(samples[-1].position.x - 12.5) < 0.1
            assert abs(samples[-1].position.y - 22.5) < 0.1
            for previous, following in zip(samples, samples[1:]):
                previous_time = (previous.header.stamp.sec +
                                 previous.header.stamp.nanosec * 1e-9)
                following_time = (following.header.stamp.sec +
                                  following.header.stamp.nanosec * 1e-9)
                step = math.hypot(following.position.x - previous.position.x,
                                  following.position.y - previous.position.y)
                assert step <= 0.85 * (following_time - previous_time) + 0.02
                assert abs(following.yaw - previous.yaw) < 0.1
        if abort_future is not None:
            assert abort_future.done() and abort_future.result().success
        print(f'PASS: {scenario}: {vehicle.calls}', flush=True)
    finally:
        vehicle.destroy_node()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)


def main():
    """Run the requested scenarios in a separate ROS domain."""
    rclpy.init()
    scenarios = sys.argv[1:] or [
        'complete', 'return_drift', 'abort', 'pilot_override',
        'connection_loss', 'bad_gps', 'stale_pose', 'home_change',
        'frame_change',
        'reject_arm', 'unconfirmed_mode', 'unconfirmed_arm',
        'reject_takeoff', 'no_home', 'reject_land', 'landing_timeout',
        'square', 'square_abort', 'heading_disagreement',
        'full', 'full_abort', 'full_override', 'full_tracking']
    try:
        for scenario in scenarios:
            run_scenario(scenario)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
