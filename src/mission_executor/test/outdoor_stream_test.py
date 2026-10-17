"""Check one-shot stream setup and terminal startup failures."""

import os
import signal
import subprocess
import time

from outdoor_flight_test import FakeVehicle

import rclpy

os.environ['ROS_DOMAIN_ID'] = '185'


class StreamVehicle(FakeVehicle):
    """Provide selectable stream responses with fake telemetry."""

    def __init__(self, outcome):
        """Choose the response before creating the inherited stream service."""
        self.outcome = outcome
        self.requests = []
        super().__init__('stream_setup')
        if outcome == 'missing_service':
            self.destroy_service(self.stream_service)
        if outcome == 'missing_landed':
            self.suppress_landed = True

    def accept_stream(self, request, response):
        """Reject the request or respond after the startup deadline."""
        self.requests.append((request.message_id, request.message_rate))
        if self.outcome == 'late':
            time.sleep(1.0)  # Only blocks the fake FCU process.
        response.success = self.outcome != 'reject'
        return response


def run_case(outcome, flight):
    """Require failure to remain terminal in both mission modes."""
    vehicle = StreamVehicle(outcome)
    process = subprocess.Popen([
        'ros2', 'run', 'mission_executor', 'outdoor_mission_node',
        '--ros-args',
        '-p', f'enable_flight:={str(flight).lower()}',
        '-p', 'gps_ready_duration:=0.2',
        '-p', 'command_timeout:=0.3'],
        start_new_session=True)
    try:
        deadline = time.monotonic() + 12.0
        failed_case = outcome in ('reject', 'late')
        while time.monotonic() < deadline:
            rclpy.spin_once(vehicle, timeout_sec=0.02)
            if failed_case and 'FAILED' in vehicle.states:
                break
            if not failed_case and vehicle.states:
                # Wait beyond the startup response timeout.
                until = time.monotonic() + 1.0
                while time.monotonic() < until:
                    rclpy.spin_once(vehicle, timeout_sec=0.02)
                break
        assert vehicle.states, 'No mission state received'
        expected = 'FAILED' if failed_case else 'WAIT_READY'
        assert vehicle.states[-1] == expected, vehicle.states
        # Late success or continued healthy telemetry must not recover FAILED.
        until = time.monotonic() + 0.6
        while time.monotonic() < until:
            rclpy.spin_once(vehicle, timeout_sec=0.02)
        assert vehicle.states[-1] == expected, vehicle.states
        assert not vehicle.calls, vehicle.calls
        assert 'REFERENCE_READY' not in vehicle.states, vehicle.states
        assert process.poll() is None, 'Node exited unexpectedly'
        expected_requests = [(245, 2.0)]
        if outcome == 'missing_service':
            expected_requests = []
        assert vehicle.requests == expected_requests, vehicle.requests
        print(f'PASS: stream {outcome}, flight={flight}', flush=True)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
        vehicle.destroy_node()


def main():
    """Run each startup outcome with flight enabled and disabled."""
    rclpy.init()
    try:
        for flight in (False, True):
            outcomes = ('reject', 'late', 'missing_landed', 'missing_service')
            for outcome in outcomes:
                run_case(outcome, flight)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
