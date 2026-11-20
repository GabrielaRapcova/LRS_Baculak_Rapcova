#!/usr/bin/env python3
"""Run the estimator offline and score it against truth.

Two data sources:
    evaluate_replay.py --bag BAG              recorded run (needs /evaluation/ground_truth)
    evaluate_replay.py --sim 42 [--sim 73 …]  course sensor model on a synthetic mission

The sensor stream goes through the C++ ``ekf_replay`` binary, i.e. the same
Localizer code the ROS node runs. Truth is only used afterwards for scoring.

Examples:
    evaluate_replay.py --sim 42 --sim 73 --sim 7
    evaluate_replay.py --sim 42 --yaw-follow accel_xy=0.8
    evaluate_replay.py --bag rosbags/char_seed42 --sensors imu,gps
"""

import argparse
import os

from drone_localization_tools import replay


def describe(name, s):
    w = s['within_2sigma']
    bias = f"  bias err {s['final_bias_err']:+.4f} rad/s" if 'final_bias_err' in s else ''
    print(f"{name}: horizontal RMS {s['rms_horizontal_m']:.2f} m (max {s['max_horizontal_m']:.2f})"
          f"  z RMS {s['rms_z_m']:.2f}  yaw RMS {s['rms_yaw_deg']:.1f} deg "
          f"(max {s['max_yaw_deg']:.1f})  |v| RMS {s['rms_speed_err']:.3f} m/s{bias}")
    print(f"    within 2 sigma: x {w['x']:.2f}  y {w['y']:.2f}  z {w['z']:.2f}  yaw {w['yaw']:.2f}"
          '   (about 0.95 = consistent covariance)')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bag', action='append', default=[])
    p.add_argument('--sim', type=int, action='append', default=[], metavar='SEED')
    p.add_argument('--profile', choices=('mission', 'characterisation'), default='mission')
    p.add_argument('--yaw-follow', action='store_true', help='simulated drone faces each leg')
    p.add_argument('--sensors', default='imu,gps')
    p.add_argument('--binary', help='path to ekf_replay')
    p.add_argument('--out', default='replay_plots')
    p.add_argument('overrides', nargs='*', help='filter parameters as key=value')
    args = p.parse_args()

    overrides = dict(o.split('=', 1) for o in args.overrides)
    sensors = tuple(args.sensors.split(','))
    os.makedirs(args.out, exist_ok=True)

    runs = []
    for bag in args.bag:
        from drone_localization_tools.bag_io import read_bag
        runs.append((os.path.basename(os.path.normpath(bag)), read_bag(bag), None))
    if args.sim:
        from drone_localization_tools import sim
        profile = (sim.mission_profile(yaw_follow=args.yaw_follow)
                   if args.profile == 'mission' else sim.characterisation_profile())
        truth = sim.truth_trajectory(profile)
        for seed in args.sim:
            data, hidden = sim.simulate(truth, seed=seed)
            runs.append((f'sim_seed{seed}', data, hidden['gyro_bias']))

    if not runs:
        p.error('give --bag and/or --sim')
    for name, data, gyro_bias in runs:
        if 'truth' not in data:
            print(f'{name}: no ground truth in bag, cannot score')
            continue
        est, log = replay.run(data, sensors=sensors, overrides=overrides, binary=args.binary)
        s = replay.score(est, data['truth'], gyro_bias)
        describe(name, s)
        for line in log.strip().splitlines():
            if line.startswith(('IMU', 'GPS', 'VO', '[WARN', '[ERROR')):
                print('    ' + line)
        path = os.path.join(args.out, f'{name}.png')
        replay.plot(est, data['truth'], s, path, title=name)
    print(f'plots in {args.out}/')


if __name__ == '__main__':
    main()
