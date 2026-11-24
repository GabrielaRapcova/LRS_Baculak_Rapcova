"""Offline sensor simulation without Gazebo or ROS.

Drives the course-supplied ``localization_sensor_sim.sensor_model.SensorCorruptor``
(the exact GPS/VO corruption used in Gazebo) with a synthetic 100 Hz truth
trajectory, and adds a Gazebo-like biased gyro. Output arrays use the
:mod:`bag_io` column layout, so the same analysis and estimator replay code runs
on simulated data and on real bags.

Used for developing and regression-testing the estimator; never at runtime.
"""

import math
import os
import sys

import numpy as np


def _import_sensor_model():
    """The course sensor model: installed with colcon, or from $LOCALIZATION_SENSOR_SIM_DIR."""
    try:
        from localization_sensor_sim import sensor_model
    except ImportError:
        source = os.environ.get('LOCALIZATION_SENSOR_SIM_DIR')
        if not source:
            raise ImportError('localization_sensor_sim not found: build it with colcon or set '
                              'LOCALIZATION_SENSOR_SIM_DIR to <repo>/localization_sensor_sim')
        sys.path.insert(0, source)
        from localization_sensor_sim import sensor_model
    return sensor_model


DEFAULT_CONFIG = {
    'gps_position_stddev': [0.6, 0.6, 0.9],
    'gps_outlier_min_m': 5.0,
    'gps_outlier_max_m': 8.0,
    'vo_scale_stddev': 0.01,
    'vo_initial_velocity_bias_stddev': 0.01,
    'vo_velocity_bias_random_walk_stddev': 0.0005,
    'vo_velocity_noise_stddev': 0.03,
    'vo_position_step_noise_stddev': 0.003,
    'vo_initial_yaw_rate_bias_stddev': 0.0005,
    'vo_yaw_bias_random_walk_stddev': 0.00003,
    'vo_yaw_step_noise_stddev': math.radians(0.03),
    'vo_pose_position_noise_stddev': 0.01,
    'vo_pose_yaw_noise_stddev': math.radians(0.3),
}

# Gazebo world IMU (fei_lrs_gazebo_localization.world): z noise 0.004, bias mean
# 0.02 with stddev 0.004, drawn once per run. Gazebo flips the bias sign at
# random (a real seed-42 run measured about -0.017 rad/s).
GYRO_NOISE_STD = 0.004
GYRO_BIAS_MEAN = 0.02
GYRO_BIAS_STD = 0.004

START_ENU = (-7.0, 13.0, 0.0)
START_YAW = math.pi / 2.0


def characterisation_profile(ground_s=20.0, hover_s=15.0, speed=0.5, leg_s=6.0,
                             yaw_rate=0.4, yaw_s=8.0):
    """Velocity segments mirroring scripts/characterisation_flight.py."""
    segs = [(ground_s, (0, 0, 0), 0.0), (5.0, (0, 0, 0.4), 0.0), (hover_s, (0, 0, 0), 0.0)]
    for vx, vy in ((speed, 0), (0, speed), (-speed, 0), (0, -speed)):
        segs += [(leg_s, (vx, vy, 0), 0.0), (2.0, (0, 0, 0), 0.0)]
    segs += [(yaw_s, (0, 0, 0), yaw_rate), (3.0, (0, 0, 0), 0.0),
             (yaw_s, (0, 0, 0), -yaw_rate), (5.0, (0, 0, 0), 0.0),
             (5.0, (0, 0, -0.4), 0.0), (3.0, (0, 0, 0), 0.0)]
    return segs


def mission_profile(speed=0.5, laps=2, ground_s=5.0, altitude=2.0, yaw_follow=False):
    """The §4 route: (0,0)->(0,-4)->(2.5,-4)->(2.5,0)->(0,0), twice, at 2 m.

    With ``yaw_follow`` the vehicle turns to face each leg (exercises yaw wrap).
    """
    segs = [(ground_s, (0, 0, 0), 0.0), (altitude / 0.4, (0, 0, 0.4), 0.0), (3.0, (0, 0, 0), 0.0)]
    route = [(0, 0), (0, -4), (2.5, -4), (2.5, 0), (0, 0)]
    yaw = START_YAW
    for _ in range(laps):
        for (x0, y0), (x1, y1) in zip(route[:-1], route[1:]):
            dx, dy = x1 - x0, y1 - y0
            dist = math.hypot(dx, dy)
            if yaw_follow:
                target = math.atan2(dy, dx)
                dyaw = (target - yaw + math.pi) % (2 * math.pi) - math.pi
                if abs(dyaw) > 1e-3:
                    segs.append((abs(dyaw) / 0.5, (0, 0, 0), math.copysign(0.5, dyaw)))
                yaw = target
            segs.append((dist / speed, (speed * dx / dist, speed * dy / dist, 0), 0.0))
            segs.append((1.5, (0, 0, 0), 0.0))
    segs += [(altitude / 0.4, (0, 0, -0.4), 0.0), (3.0, (0, 0, 0), 0.0)]
    return segs


def truth_trajectory(segments, rate=100.0, tau=0.4, t0=100.0):
    """Integrate first-order-smoothed velocity segments into a truth array.

    Returns rows ``t, x, y, z, yaw, vx, vy, vz, wz`` (ENU, yaw unwrapped->wrapped).
    """
    dt = 1.0 / rate
    pos = np.array(START_ENU, dtype=float)
    yaw = START_YAW
    vel = np.zeros(3)
    wz = 0.0
    alpha = dt / (tau + dt)
    rows = []
    t = t0
    for duration, v_cmd, wz_cmd in segments:
        v_cmd = np.asarray(v_cmd, dtype=float)
        for _ in range(int(round(duration * rate))):
            vel += alpha * (v_cmd - vel)
            wz += alpha * (wz_cmd - wz)
            pos = pos + vel * dt
            pos[2] = max(pos[2], 0.0)
            yaw = yaw + wz * dt
            rows.append([t, *pos, (yaw + math.pi) % (2 * math.pi) - math.pi, *vel, wz])
            t += dt
    return np.asarray(rows)


def simulate(truth, seed=42, gyro_seed=None, config=None, events=()):
    """Run the course corruption model over ``truth``.

    ``events`` is a list of ``(time, action)`` with actions
    ``'gps_off'``, ``'gps_on'``, ``'vo_off'``, ``'vo_on'``, ``'gps_outlier'`` —
    the same controls the teacher's services provide.
    """
    sm = _import_sensor_model()
    cfg = dict(DEFAULT_CONFIG, **(config or {}))
    corruptor = sm.SensorCorruptor(seed, cfg)
    grng = np.random.default_rng(seed + 1000 if gyro_seed is None else gyro_seed)
    gyro_bias = GYRO_BIAS_MEAN + grng.normal(0.0, GYRO_BIAS_STD)
    if grng.uniform() < 0.5:
        gyro_bias = -gyro_bias

    events = sorted(events)
    ev_i = 0
    gps_on = vo_on = True
    outlier = False
    next_gps = next_vo = None
    gps, vo, imu = [], [], []
    for row in truth:
        t = row[0]
        while ev_i < len(events) and events[ev_i][0] <= t:
            action = events[ev_i][1]
            gps_on = {'gps_off': False, 'gps_on': True}.get(action, gps_on)
            vo_on = {'vo_off': False, 'vo_on': True}.get(action, vo_on)
            outlier = outlier or action == 'gps_outlier'
            ev_i += 1
        position, yaw, wz = row[1:4], row[4], row[8]
        corruptor.advance(position, yaw, t)
        due, next_gps = sm.advance_schedule(next_gps, 0.2, t)
        if due and gps_on:
            g = corruptor.gps_measurement(position, outlier=outlier)
            outlier = False
            gps.append([t, *g, t])
        due, next_vo = sm.advance_schedule(next_vo, 1.0 / 30.0, t)
        if due and vo_on:
            s = corruptor.noisy_vo_sample()
            vo.append([t, *s.position, s.yaw, *s.body_velocity, t])
        imu.append([t, wz + gyro_bias + grng.normal(0.0, GYRO_NOISE_STD), t])

    truth_out = np.column_stack([truth, truth[:, 0]])
    hidden = {'gyro_bias': gyro_bias, 'vo_scale': corruptor.vo_scale,
              'vo_velocity_bias': corruptor.vo_velocity_bias.tolist(),
              'vo_yaw_rate_bias': corruptor.vo_yaw_rate_bias}
    return {'gps': np.asarray(gps), 'vo': np.asarray(vo), 'imu': np.asarray(imu),
            'truth': truth_out}, hidden
