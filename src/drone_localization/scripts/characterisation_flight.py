#!/usr/bin/env python3
"""Open-loop characterisation flight for §1 (not part of the submitted controller).

Records nothing itself — run ``record_characterisation.sh`` alongside it.
It produces the motion the characterisation needs, without any position
feedback at all (no MAVROS pose, no truth, no estimate):

    1. ground_s   stationary on the ground  -> gyro bias, VO drift at rest
    2. takeoff    MAVROS takeoff service to `takeoff_alt` m
    3. hover_s    stationary in the air     -> GPS noise while hovering
    4. square     timed body-agnostic ENU velocity legs at `speed` m/s
    5. yaw turns  +yaw_rate then -yaw_rate  -> gyro vs VO yaw while turning
    6. land       LAND mode, wait for disarm

Velocities are sent as `/mavros/setpoint_raw/local` velocity setpoints (ENU,
MAVROS converts to NED). Only MAVROS state and command services are used.
"""

import math

import rclpy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node

VELOCITY_YAW_RATE_MASK = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
    | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW)


class CharacterisationFlight(Node):

    def __init__(self):
        super().__init__('characterisation_flight')
        ns = self.declare_parameter('mavros_ns', '/mavros').value.rstrip('/')
        self.ground_s = self.declare_parameter('ground_s', 20.0).value
        self.takeoff_alt = self.declare_parameter('takeoff_alt', 2.0).value
        self.takeoff_wait_s = self.declare_parameter('takeoff_wait_s', 10.0).value
        self.hover_s = self.declare_parameter('hover_s', 15.0).value
        speed = self.declare_parameter('speed', 0.5).value
        leg_s = self.declare_parameter('leg_s', 6.0).value
        yaw_rate = self.declare_parameter('yaw_rate', 0.4).value
        yaw_s = self.declare_parameter('yaw_s', 8.0).value

        # (label, duration, vx, vy, vz, yaw_rate) — ENU, open loop.
        self.legs = []
        for label, vx, vy in (('east', speed, 0.0), ('north', 0.0, speed),
                              ('west', -speed, 0.0), ('south', 0.0, -speed)):
            self.legs.append((label, leg_s, vx, vy, 0.0, 0.0))
            self.legs.append(('stop', 2.0, 0.0, 0.0, 0.0, 0.0))
        self.legs.append(('yaw +', yaw_s, 0.0, 0.0, 0.0, yaw_rate))
        self.legs.append(('stop', 3.0, 0.0, 0.0, 0.0, 0.0))
        self.legs.append(('yaw -', yaw_s, 0.0, 0.0, 0.0, -yaw_rate))
        self.legs.append(('hover', 5.0, 0.0, 0.0, 0.0, 0.0))

        self.state = State()
        self.create_subscription(State, f'{ns}/state', self._on_state, 10)
        self.setpoint_pub = self.create_publisher(
            PositionTarget, f'{ns}/setpoint_raw/local', 10)
        self.mode_cli = self.create_client(SetMode, f'{ns}/set_mode')
        self.arm_cli = self.create_client(CommandBool, f'{ns}/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, f'{ns}/cmd/takeoff')

        self.phase = 'wait_connection'
        self.phase_start = self.now()
        self.last_request = None
        self.leg_index = 0
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'characterisation flight: {self.ground_s:.0f} s ground, takeoff to '
            f'{self.takeoff_alt:.1f} m, {self.hover_s:.0f} s hover, square at '
            f'{speed:.2f} m/s, yaw turns at {math.degrees(yaw_rate):.0f} deg/s')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg):
        self.state = msg

    def time_in_phase(self):
        return self.now() - self.phase_start

    def enter(self, phase):
        self.get_logger().info(f'phase: {self.phase} -> {phase}')
        self.phase = phase
        self.phase_start = self.now()
        self.last_request = None

    def request(self, client, request, period):
        """Send a service request, repeated every `period` s until the phase changes.

        The outcome is read from /mavros/state (mode, armed), so the response
        itself is not awaited.
        """
        if not client.service_is_ready():
            return
        if self.last_request is not None and self.now() - self.last_request < period:
            return
        self.last_request = self.now()
        client.call_async(request)

    def send_velocity(self, vx, vy, vz, yaw_rate):
        setpoint = PositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # ENU in, MAVROS converts
        setpoint.type_mask = VELOCITY_YAW_RATE_MASK
        setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z = vx, vy, vz
        setpoint.yaw_rate = yaw_rate
        self.setpoint_pub.publish(setpoint)

    def known_disarmed(self):
        """MAVROS reports connected=False, mode='' on a missed heartbeat: unknown, not disarmed."""
        return self.state.connected and self.state.mode != '' and not self.state.armed

    def tick(self):
        phase = self.phase
        if phase == 'wait_connection':
            if self.state.connected:
                self.enter('ground')
        elif phase == 'ground':
            if self.time_in_phase() >= self.ground_s:
                self.enter('guided')
        elif phase == 'guided':
            if self.state.mode == 'GUIDED':
                self.enter('arm')
            else:
                self.request(self.mode_cli, SetMode.Request(custom_mode='GUIDED'), 2.0)
        elif phase == 'arm':
            if self.state.armed:
                self.enter('takeoff')
            else:
                self.request(self.arm_cli, CommandBool.Request(value=True), 3.0)
        elif phase == 'takeoff':
            if self.last_request is None:
                self.request(self.takeoff_cli,
                             CommandTOL.Request(altitude=float(self.takeoff_alt)), 0.0)
            if self.time_in_phase() >= self.takeoff_wait_s:
                self.enter('hover')
        elif phase == 'hover':
            self.send_velocity(0.0, 0.0, 0.0, 0.0)
            if self.time_in_phase() >= self.hover_s:
                self.enter('legs')
        elif phase == 'legs':
            label, duration, vx, vy, vz, wz = self.legs[self.leg_index]
            self.send_velocity(vx, vy, vz, wz)
            if self.time_in_phase() >= duration:
                self.leg_index += 1
                if self.leg_index == len(self.legs):
                    self.enter('land')
                else:
                    self.phase_start = self.now()
                    self.get_logger().info(f'leg: {self.legs[self.leg_index][0]}')
        elif phase == 'land':
            if self.state.mode == 'LAND':
                self.enter('wait_disarm')
            else:
                self.request(self.mode_cli, SetMode.Request(custom_mode='LAND'), 2.0)
        elif phase == 'wait_disarm':
            if self.known_disarmed():
                self.get_logger().info('landed and disarmed: stop the recording now')
                self.enter('done')


def main():
    rclpy.init()
    node = CharacterisationFlight()
    try:
        while rclpy.ok() and node.phase != 'done':
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
