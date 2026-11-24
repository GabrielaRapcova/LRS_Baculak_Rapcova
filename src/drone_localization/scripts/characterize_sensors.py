#!/usr/bin/env python3
"""§1 characterisation: plots and a summary from one or two recorded bags.

Usage:
    characterize_sensors.py BAG [--compare BAG2] [--out DIR]

Produces in DIR (default: ./characterisation_<bagname>):
    rates.png        stamp intervals of GPS / VO / IMU
    gps_noise.png    GPS error vs truth per axis + horizontal scatter
    vo_drift.png     VO position / yaw drift and body-velocity error vs truth
    gyro_bias.png    gyro Z error vs truth, stationary samples highlighted
    trajectory.png   truth, GPS and VO in the ENU plane
    summary.json     all numbers printed to the terminal

With --compare, both runs are summarised side by side: same seed should give the
same hidden biases (gyro bias, VO velocity bias), a different seed should not.

Ground truth is read from the bag for offline evaluation only.
"""

import argparse
import json
import os
import sys

import numpy as np

from drone_localization_tools import analysis as an
from drone_localization_tools.bag_io import read_bag


def _plt():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt


def plot_rates(data, out):
    plt = _plt()
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.5))
    for ax, kind, nominal in zip(axes, ('gps', 'vo', 'imu'), (5.0, 30.0, 100.0)):
        if kind not in data:
            ax.set_title(f'{kind}: no data')
            continue
        t = data[kind][:, 0]
        dt = np.diff(t) * 1000.0
        ax.plot(t[1:] - t[0], dt, '.', ms=2)
        ax.axhline(1000.0 / nominal, color='k', lw=0.8, ls='--', label=f'nominal {nominal:g} Hz')
        st = an.rate_stats(t)
        ax.set_title(f"{kind.upper()}: {st['rate_hz']:.2f} Hz, max gap {st['max_gap']*1000:.0f} ms")
        ax.set_xlabel('sim time [s]')
        ax.set_ylabel('stamp interval [ms]')
        ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'rates.png'), dpi=130)
    plt.close(fig)


def plot_gps(data, out):
    plt = _plt()
    g = an.gps_error(data['gps'], data['truth'])
    t = g['t'] - g['t'][0]
    fig, axes = plt.subplots(1, 2, figsize=(13, 4), gridspec_kw={'width_ratios': [2, 1]})
    for i, name in enumerate('xyz'):
        axes[0].plot(t, g['err'][:, i], lw=0.8,
                     label=f"{name}: mean {g['mean'][i]:+.2f}, std {g['std'][i]:.2f} m")
    axes[0].set_xlabel('time [s]')
    axes[0].set_ylabel('GPS - truth [m]')
    axes[0].set_title('GPS noise')
    axes[0].legend(fontsize=8)
    axes[1].scatter(g['err'][:, 0], g['err'][:, 1], s=4)
    axes[1].set_aspect('equal')
    axes[1].set_xlabel('east error [m]')
    axes[1].set_ylabel('north error [m]')
    axes[1].set_title(f"horizontal RMS {g['rms_horizontal']:.2f} m")
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'gps_noise.png'), dpi=130)
    plt.close(fig)


def plot_vo(data, out):
    plt = _plt()
    v = an.vo_drift(data['vo'], data['truth'])
    t = v['t'] - v['t'][0]
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for i, name in enumerate('xyz'):
        axes[0].plot(t, v['drift'][:, i], lw=0.9, label=name)
    axes[0].plot(t, np.linalg.norm(v['drift'], axis=1), 'k', lw=1.2, label='|drift|')
    axes[0].set_ylabel('VO position drift [m]')
    axes[0].set_title(f"VO drift: {v['final_drift_m']:.2f} m after {t[-1]:.0f} s "
                      f"({v['drift_rate_m_per_min']:.2f} m/min)")
    axes[0].legend(fontsize=8)
    axes[1].plot(t, np.degrees(v['yaw_drift']), lw=0.9)
    axes[1].set_ylabel('VO yaw drift [deg]')
    for i, name in enumerate(('bx', 'by', 'bz')):
        scale, bias = v['vb_fit'][i]
        axes[2].plot(t, v['vb_err'][:, i], lw=0.5,
                     label=f'{name}: scale {scale:.3f}, bias {bias:+.3f} m/s')
    axes[2].set_ylabel('VO body vel - truth [m/s]')
    axes[2].set_xlabel('time [s]')
    axes[2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'vo_drift.png'), dpi=130)
    plt.close(fig)


def plot_gyro(data, out):
    plt = _plt()
    b = an.gyro_bias(data['imu'], data['truth'])
    t = b['t'] - b['t'][0]
    fig, ax = plt.subplots(figsize=(11, 3.8))
    ax.plot(t, b['err'], lw=0.4, color='0.6', label='gyro_z - truth yaw rate')
    ax.plot(t[b['still']], b['err'][b['still']], '.', ms=1.5, label='stationary samples')
    ax.axhline(b['bias_stationary'], color='r', lw=1.2,
               label=f"stationary bias {b['bias_stationary']:+.4f} rad/s "
                     f"({np.degrees(b['bias_stationary']):+.2f} deg/s)")
    ax.axhline(0.0, color='k', lw=0.6)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('[rad/s]')
    ax.set_title(f"gyro Z bias and noise (std {b['noise_std']:.4f} rad/s)")
    ax.legend(fontsize=8, loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'gyro_bias.png'), dpi=130)
    plt.close(fig)


def plot_trajectory(data, out):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(7, 7))
    tr = data['truth']
    ax.plot(tr[:, 1], tr[:, 2], 'k', lw=1.5, label='truth')
    if 'gps' in data:
        ax.plot(data['gps'][:, 1], data['gps'][:, 2], '.', ms=3, alpha=0.6, label='GPS')
    if 'vo' in data:
        ax.plot(data['vo'][:, 1], data['vo'][:, 2], lw=1.0, label='VO')
    if 'est' in data:
        ax.plot(data['est'][:, 1], data['est'][:, 2], lw=1.2, label='estimate (recorded)')
    ax.set_aspect('equal')
    ax.set_xlabel('x ENU [m]')
    ax.set_ylabel('y ENU [m]')
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'trajectory.png'), dpi=130)
    plt.close(fig)


def print_summary(name, summary):
    print(f'\n=== {name} ===')
    for kind, st in summary['rates'].items():
        print(f"  {kind:5s} {st['n']:6d} msgs  {st['rate_hz']:7.2f} Hz  "
              f"max gap {st['max_gap']*1000:6.0f} ms  non-monotonic {st['non_monotonic']}")
    if 'gps' in summary:
        g = summary['gps']
        print('  GPS   std  [m]  ' + ' '.join(f'{v:.3f}' for v in g['std_m'])
              + f"   horizontal RMS {g['rms_horizontal_m']:.3f} m")
    if 'vo' in summary:
        v = summary['vo']
        print(f"  VO    drift {v['final_drift_m']:.3f} m in {v['duration_s']:.0f} s "
              f"({v['drift_rate_m_per_min']:.3f} m/min), "
              f"yaw drift {v['final_yaw_drift_deg']:+.2f} deg")
        print('        body-vel (scale, bias): '
              + ', '.join(f'({s:.3f}, {b:+.4f})' for s, b in v['body_velocity_scale_bias']))
    if 'gyro' in summary:
        b = summary['gyro']
        print(f"  gyro  stationary bias {b['bias_stationary_rad_s']:+.5f} rad/s "
              f"({b['bias_stationary_deg_s']:+.3f} deg/s), noise std {b['noise_std_rad_s']:.4f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag')
    parser.add_argument('--compare', help='second bag (other or same seed)')
    parser.add_argument('--out', help='output directory')
    args = parser.parse_args()

    name = os.path.basename(os.path.normpath(args.bag))
    out = args.out or f'characterisation_{name}'
    os.makedirs(out, exist_ok=True)

    data = read_bag(args.bag)
    missing = [k for k in ('gps', 'vo', 'imu', 'truth') if k not in data]
    if missing:
        print(f'warning: bag has no {missing} messages', file=sys.stderr)

    summary = an.summarize(data)
    print_summary(name, summary)
    plot_rates(data, out)
    if 'truth' in data:
        for kind, fn in (('gps', plot_gps), ('vo', plot_vo), ('imu', plot_gyro)):
            if kind in data:
                fn(data, out)
        plot_trajectory(data, out)

    result = {name: summary}
    if args.compare:
        name_b = os.path.basename(os.path.normpath(args.compare))
        data_b = read_bag(args.compare)
        summary_b = an.summarize(data_b)
        print_summary(name_b, summary_b)
        if 'truth' in data and 'truth' in data_b:
            rep = an.reproducibility(data, data_b)
            print(f"\n  GPS error sequence, first {rep['samples']} samples: "
                  f"correlation {rep['correlation']:+.3f}, "
                  f"max |diff| {rep['max_abs_diff_m']:.3f} m")
            print('  (same seed -> correlation ~1 and identical hidden biases; '
                  'different seed -> ~0 and different biases)')
            result['reproducibility'] = rep
        result[name_b] = summary_b

    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(result, f, indent=2)
    print(f'\nplots and summary.json written to {out}/')


if __name__ == '__main__':
    main()
