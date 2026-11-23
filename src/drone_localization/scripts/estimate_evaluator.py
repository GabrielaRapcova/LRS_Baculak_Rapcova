#!/usr/bin/env python3
"""Live, truth-only evaluation of the estimate (separate process).

Subscribes to /evaluation/ground_truth and /localization/estimate_state, and:
  * logs the current and RMS errors every `report_period_s` of sim time;
  * publishes /evaluation/estimate_error (Float64MultiArray:
    [t, ex, ey, ez, evx, evy, evz, eyaw]) for rqt_plot / PlotJuggler;
  * on shutdown (Ctrl-C) writes <output_dir>/estimate.csv, truth.csv and
    errors.png (same plot as evaluate_replay.py).

It never publishes anything the estimator or controller subscribes to.
"""

import bisect
import math
import os

import numpy as np
import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from drone_localization_tools import analysis as an
from drone_localization_tools import replay


def _stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EstimateEvaluator(Node):

    def __init__(self):
        super().__init__('estimate_evaluator')
        self.output_dir = self.declare_parameter('output_dir', 'evaluation').value
        self.report_period = self.declare_parameter('report_period_s', 5.0).value
        self.truth = []        # rows t, x, y, z, yaw, vx, vy, vz, wz
        self.truth_t = []
        self.est = []          # estimate_state rows (t, 8 states, 8 sigmas)
        self.next_report = None
        self.create_subscription(Odometry, '/evaluation/ground_truth', self.on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/localization/estimate_state',
                                 self.on_state, 50)
        self.err_pub = self.create_publisher(Float64MultiArray, '/evaluation/estimate_error', 10)
        self.get_logger().info(f'evaluating estimate against truth; results -> {self.output_dir}/')

    def on_truth(self, m):
        t = _stamp(m.header)
        if self.truth_t and t <= self.truth_t[-1]:
            return
        p, v = m.pose.pose.position, m.twist.twist.linear
        self.truth.append([t, p.x, p.y, p.z, _yaw(m.pose.pose.orientation),
                           v.x, v.y, v.z, m.twist.twist.angular.z])
        self.truth_t.append(t)

    def _truth_at(self, t):
        i = bisect.bisect_left(self.truth_t, t)
        if i == 0 or i >= len(self.truth_t):
            return None
        a, b = np.array(self.truth[i - 1]), np.array(self.truth[i])
        w = (t - a[0]) / (b[0] - a[0])
        row = a + w * (b - a)
        row[4] = a[4] + w * an.wrap(b[4] - a[4])
        return row

    def on_state(self, m):
        if len(m.data) < 17:
            return
        row = list(m.data[:17])
        self.est.append(row)
        tr = self._truth_at(row[0])
        if tr is None:
            return
        e = [row[0], row[1] - tr[1], row[2] - tr[2], row[3] - tr[3],
             row[4] - tr[5], row[5] - tr[6], row[6] - tr[7], float(an.wrap(row[7] - tr[4]))]
        self.err_pub.publish(Float64MultiArray(data=[float(v) for v in e]))

        if self.next_report is None:
            self.next_report = row[0] + self.report_period
        if row[0] >= self.next_report:
            self.next_report = row[0] + self.report_period
            self.report(e)

    def report(self, e):
        s = self._score()
        if s is None:
            return
        self.get_logger().info(
            f"t={e[0]:.1f} now: horiz {math.hypot(e[1], e[2]):.2f} m, z {e[3]:+.2f} m, "
            f"yaw {math.degrees(e[7]):+.1f} deg | so far: horiz RMS {s['rms_horizontal_m']:.2f} "
            f"(max {s['max_horizontal_m']:.2f}) m, yaw RMS {s['rms_yaw_deg']:.1f} deg, "
            f"within 2sigma xy {(s['within_2sigma']['x'] + s['within_2sigma']['y']) / 2:.2f}")

    def _score(self):
        if len(self.est) < 10 or len(self.truth) < 10:
            return None
        est = np.array(self.est)
        truth = np.array(self.truth)
        est = est[(est[:, 0] > truth[0, 0]) & (est[:, 0] < truth[-1, 0])]
        if len(est) < 10:
            return None
        return replay.score(est, truth)

    def save(self):
        s = self._score()
        if s is None:
            self.get_logger().warn('not enough data to save an evaluation')
            return
        os.makedirs(self.output_dir, exist_ok=True)
        est = np.array(self.est)
        truth = np.array(self.truth)
        np.savetxt(os.path.join(self.output_dir, 'estimate.csv'), est, delimiter=',',
                   header=','.join(replay.EST_COLUMNS), comments='')
        np.savetxt(os.path.join(self.output_dir, 'truth.csv'), truth, delimiter=',',
                   header='t,x,y,z,yaw,vx,vy,vz,wz', comments='')
        est = est[(est[:, 0] > truth[0, 0]) & (est[:, 0] < truth[-1, 0])]
        replay.plot(est, truth, s, os.path.join(self.output_dir, 'errors.png'), title='live run')
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
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
