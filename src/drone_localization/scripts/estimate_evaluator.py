#!/usr/bin/env python3
"""Live, truth-only evaluation of the estimate (a separate process).

Subscribes to /evaluation/ground_truth and /localization/estimate_state, and
  * logs the current and RMS errors every `report_period_s` of sim time;
  * publishes /evaluation/estimate_error (Float64MultiArray:
    [t, ex, ey, ez, evx, evy, evz, eyaw]) for rqt_plot / PlotJuggler;
  * on shutdown (Ctrl-C) writes <output_dir>/estimate.csv, truth.csv and errors.png.

Nothing the estimator or the controller subscribes to is published here.
"""

import math
import os
import signal

import numpy as np
import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from drone_localization_tools import analysis
from drone_localization_tools import replay
from drone_localization_tools.bag_io import stamp_seconds, yaw_from_quaternion

TRUTH_COLUMNS = 't,x,y,z,yaw,vx,vy,vz,wz'


class EstimateEvaluator(Node):

    def __init__(self):
        super().__init__('estimate_evaluator')
        self.output_dir = self.declare_parameter('output_dir', 'evaluation').value
        self.report_period = self.declare_parameter('report_period_s', 5.0).value
        self.truth = []      # rows: t, x, y, z, yaw, vx, vy, vz, wz
        self.estimate = []   # estimate_state rows: t, 8 states, 8 sigmas
        self.next_report = None
        self.create_subscription(Odometry, '/evaluation/ground_truth', self.on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/localization/estimate_state',
                                 self.on_estimate, 50)
        self.error_pub = self.create_publisher(
            Float64MultiArray, '/evaluation/estimate_error', 10)
        self.get_logger().info(f'evaluating against truth; results -> {self.output_dir}/')

    def on_truth(self, msg):
        t = stamp_seconds(msg.header)
        if self.truth and t <= self.truth[-1][0]:
            return
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.truth.append([t, p.x, p.y, p.z, yaw_from_quaternion(msg.pose.pose.orientation),
                           v.x, v.y, v.z, msg.twist.twist.angular.z])

    def on_estimate(self, msg):
        row = list(msg.data[:17])
        if len(row) < 17 or len(self.truth) < 2:
            return
        self.estimate.append(row)
        t = row[0]
        # Interpolate the last few truth samples at the estimate stamp.
        recent = np.array(self.truth[-50:])
        if not recent[0, 0] <= t <= recent[-1, 0]:
            return
        position, yaw, velocity, _ = analysis.interp_truth(recent, np.array([t]))
        error = [t, *(np.array(row[1:4]) - position[0]), *(np.array(row[4:7]) - velocity[0]),
                 float(analysis.wrap(row[7] - yaw[0]))]
        self.error_pub.publish(Float64MultiArray(data=[float(e) for e in error]))

        if self.next_report is None:
            self.next_report = t + self.report_period
        if t >= self.next_report:
            self.next_report = t + self.report_period
            self.report(error)

    def score(self):
        """Errors of everything received so far, or None if there is too little data."""
        if len(self.estimate) < 10 or len(self.truth) < 10:
            return None, None, None
        estimate, truth = np.array(self.estimate), np.array(self.truth)
        inside = (estimate[:, 0] > truth[0, 0]) & (estimate[:, 0] < truth[-1, 0])
        if inside.sum() < 10:
            return None, None, None
        return replay.score(estimate[inside], truth), estimate[inside], truth

    def report(self, error):
        s, _, _ = self.score()
        if s is None:
            return
        within = (s['within_2sigma']['x'] + s['within_2sigma']['y']) / 2
        self.get_logger().info(
            f"t={error[0]:.1f} now: horiz {math.hypot(error[1], error[2]):.2f} m, "
            f"z {error[3]:+.2f} m, yaw {math.degrees(error[7]):+.1f} deg | so far: "
            f"horiz RMS {s['rms_horizontal_m']:.2f} (max {s['max_horizontal_m']:.2f}) m, "
            f"yaw RMS {s['rms_yaw_deg']:.1f} deg, within 2sigma xy {within:.2f}")

    def save(self):
        s, estimate, truth = self.score()
        if s is None:
            self.get_logger().warn('not enough data to save an evaluation')
            return
        os.makedirs(self.output_dir, exist_ok=True)
        np.savetxt(os.path.join(self.output_dir, 'estimate.csv'), np.array(self.estimate),
                   delimiter=',', header=','.join(replay.EST_COLUMNS), comments='')
        np.savetxt(os.path.join(self.output_dir, 'truth.csv'), np.array(self.truth),
                   delimiter=',', header=TRUTH_COLUMNS, comments='')
        replay.plot(estimate, truth, s, os.path.join(self.output_dir, 'errors.png'),
                    title='live run')
        print(f"[estimate_evaluator] horizontal RMS {s['rms_horizontal_m']:.2f} m "
              f"(max {s['max_horizontal_m']:.2f}), z RMS {s['rms_z_m']:.2f} m, "
              f"yaw RMS {s['rms_yaw_deg']:.1f} deg -> {self.output_dir}/", flush=True)


def main():
    # Import matplotlib up front so saving on Ctrl-C is quick.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot  # noqa: F401

    rclpy.init()
    node = EstimateEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # a second Ctrl-C must not abort saving
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
