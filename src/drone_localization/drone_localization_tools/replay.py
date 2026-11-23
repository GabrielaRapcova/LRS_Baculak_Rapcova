"""Feed recorded or simulated sensor data through the C++ ``ekf_replay`` binary.

The estimator code that runs here is the same ``Localizer`` the ROS node uses;
only the transport differs. Truth is used afterwards to score the output.
"""

import os
import shutil
import subprocess
import tempfile

import numpy as np

from drone_localization_tools import analysis as an

# Same order as the EKF state: position, velocity, yaw, bias, then sigmas.
EST_COLUMNS = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'yaw', 'bias',
               'sx', 'sy', 'sz', 'svx', 'svy', 'svz', 'syaw', 'sbias']
GPS_VAR = (0.36, 0.36, 0.81)
VO_VEL_VAR = 0.03 ** 2  # what the sensor simulator puts in the twist covariance


def find_binary(explicit=None):
    """Locate ekf_replay: explicit path, $EKF_REPLAY, PATH, or the colcon install."""
    for c in (explicit, os.environ.get('EKF_REPLAY'), shutil.which('ekf_replay')):
        if c and os.path.isfile(c):
            return c
    try:
        from ament_index_python.packages import get_package_prefix
        c = os.path.join(get_package_prefix('drone_localization'),
                         'lib', 'drone_localization', 'ekf_replay')
        if os.path.isfile(c):
            return c
    except Exception:  # noqa: BLE001 - ament not available offline
        pass
    raise FileNotFoundError('ekf_replay not found; build the package or set EKF_REPLAY')


def to_events(data, sensors=('imu', 'gps', 'vo'), jitter=0.0, seed=0):
    """Return event lines sorted by arrival (``recv`` column, plus optional jitter)."""
    rng = np.random.default_rng(seed)
    events = []
    if 'imu' in sensors and 'imu' in data:
        for t, wz, recv in data['imu']:
            events.append((recv, f'imu,{t:.6f},{wz:.8f}'))
    if 'gps' in sensors and 'gps' in data:
        for row in data['gps']:
            t, x, y, z, recv = row[:5]
            recv = recv + abs(rng.normal(0.0, jitter)) if jitter > 0 else recv
            events.append((recv, f'gps,{t:.6f},{x:.6f},{y:.6f},{z:.6f},'
                                 f'{GPS_VAR[0]},{GPS_VAR[1]},{GPS_VAR[2]}'))
    if 'vo' in sensors and 'vo' in data:
        for row in data['vo']:
            t, yaw, vb, recv = row[0], row[4], row[5:8], row[8]
            recv = recv + abs(rng.normal(0.0, jitter)) if jitter > 0 else recv
            events.append((recv, f'vo,{t:.6f},{yaw:.8f},{vb[0]:.6f},{vb[1]:.6f},{vb[2]:.6f},'
                                 f'{VO_VEL_VAR},{VO_VEL_VAR},{VO_VEL_VAR}'))
    events.sort(key=lambda e: e[0])
    return [e[1] for e in events]


def run(data, sensors=('imu', 'gps', 'vo'), overrides=None, binary=None, every=1, jitter=0.0):
    """Run the replay and return (estimate array, stderr text)."""
    binary = find_binary(binary)
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
        f.write('\n'.join(to_events(data, sensors, jitter)) + '\n')
        path = f.name
    try:
        args = [binary, path, '--every', str(every)]
        args += [f'{k}={v}' for k, v in (overrides or {}).items()]
        proc = subprocess.run(args, capture_output=True, text=True, check=True)
    finally:
        os.unlink(path)
    lines = proc.stdout.strip().splitlines()[1:]
    est = np.array([[float(v) for v in ln.split(',')] for ln in lines]) if lines else \
        np.zeros((0, len(EST_COLUMNS)))
    return est, proc.stderr


def score(est, truth, gyro_bias=None, skip_s=0.0):
    """Errors of the estimate against truth, plus 2-sigma consistency fractions."""
    t = est[:, 0]
    keep = t >= t[0] + skip_s
    est, t = est[keep], t[keep]
    pos, yaw, vel, _ = an.interp_truth(truth, t)
    e_pos = est[:, 1:4] - pos
    e_yaw = an.wrap(est[:, 7] - yaw)
    e_vel = est[:, 4:7] - vel
    out = {
        't': t,
        'e_pos': e_pos,
        'e_yaw': e_yaw,
        'e_vel': e_vel,
        'rms_horizontal_m': float(np.sqrt(np.mean(np.sum(e_pos[:, :2] ** 2, axis=1)))),
        'max_horizontal_m': float(np.max(np.linalg.norm(e_pos[:, :2], axis=1))),
        'rms_z_m': float(np.sqrt(np.mean(e_pos[:, 2] ** 2))),
        'rms_yaw_deg': float(np.degrees(np.sqrt(np.mean(e_yaw ** 2)))),
        'max_yaw_deg': float(np.degrees(np.max(np.abs(e_yaw)))),
        'rms_speed_err': float(np.sqrt(np.mean(np.sum(e_vel ** 2, axis=1)))),
        # Fraction of samples whose error lies within 2 sigma (~0.95 if consistent).
        'within_2sigma': {
            'x': float(np.mean(np.abs(e_pos[:, 0]) < 2 * est[:, 9])),
            'y': float(np.mean(np.abs(e_pos[:, 1]) < 2 * est[:, 10])),
            'z': float(np.mean(np.abs(e_pos[:, 2]) < 2 * est[:, 11])),
            'yaw': float(np.mean(np.abs(e_yaw) < 2 * est[:, 15])),
        },
    }
    if gyro_bias is not None:
        out['e_bias'] = est[:, 8] - gyro_bias
        out['final_bias_err'] = float(est[-1, 8] - gyro_bias)
    return out


def plot(est, truth, s, path, title='', spans=(), marks=()):
    """Error-vs-time figure with the filter's own 2-sigma bounds."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    keep = est[:, 0] >= s['t'][0]
    est = est[keep]
    t = s['t'] - s['t'][0]
    fig, axes = plt.subplots(5 if 'e_bias' in s else 4, 1, figsize=(11, 12), sharex=True)
    for i, name in enumerate('xyz'):
        ax = axes[0] if i < 2 else axes[1]
        line, = ax.plot(t, s['e_pos'][:, i], lw=0.8, label=f'{name} error')
        ax.fill_between(t, -2 * est[:, 9 + i], 2 * est[:, 9 + i],
                        color=line.get_color(), alpha=0.12)
    axes[0].set_ylabel('horizontal [m]')
    axes[0].set_title(f"{title}  RMS horizontal {s['rms_horizontal_m']:.2f} m, "
                      f"max {s['max_horizontal_m']:.2f} m")
    axes[1].set_ylabel('z [m]')
    for i, name in enumerate(('vx', 'vy', 'vz')):
        axes[2].plot(t, s['e_vel'][:, i], lw=0.6, label=name)
    axes[2].set_ylabel('velocity err [m/s]')
    axes[3].plot(t, np.degrees(s['e_yaw']), lw=0.8)
    axes[3].fill_between(t, -2 * np.degrees(est[:, 15]), 2 * np.degrees(est[:, 15]),
                         alpha=0.15)
    axes[3].set_ylabel('yaw err [deg]')
    axes[3].set_title(f"yaw RMS {s['rms_yaw_deg']:.1f} deg, max {s['max_yaw_deg']:.1f} deg")
    if 'e_bias' in s:
        axes[4].plot(t, s['e_bias'], lw=0.8)
        axes[4].fill_between(t, -2 * est[:, 16], 2 * est[:, 16], alpha=0.15)
        axes[4].set_ylabel('bias err [rad/s]')
    t0 = s['t'][0]
    for ax in axes:
        for a, b, label in spans:  # e.g. sensor outages
            ax.axvspan(a - t0, b - t0, color='0.85', zorder=0)
        for m, label in marks:  # e.g. injected outliers
            ax.axvline(m - t0, color='r', lw=0.8, ls=':')
        ax.axhline(0, color='k', lw=0.5)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=8, loc='upper right')
    for a, b, label in spans:
        axes[0].text(0.5 * (a + b) - t0, axes[0].get_ylim()[1] * 0.85, label,
                     ha='center', fontsize=8)
    axes[-1].set_xlabel('time since initialisation [s]')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
