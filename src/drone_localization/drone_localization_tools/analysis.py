"""Sensor characterisation on arrays produced by :mod:`bag_io`.

Every function here is pure NumPy so it can be tested without ROS. Ground truth
is used only to *measure* sensor errors offline; it never reaches the estimator.
"""

import numpy as np

# Column indices (see bag_io docstring).
GPS_T, GPS_P = 0, slice(1, 4)
VO_T, VO_P, VO_YAW, VO_VB = 0, slice(1, 4), 4, slice(5, 8)
IMU_T, IMU_WZ = 0, 1
TR_T, TR_P, TR_YAW, TR_V, TR_WZ = 0, slice(1, 4), 4, slice(5, 8), 8


def wrap(angle):
    """Wrap angles to [-pi, pi)."""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def rate_stats(t):
    """Rate and gap statistics of a stamp series (seconds)."""
    t = np.asarray(t, dtype=float)
    if t.size < 2:
        return {'n': int(t.size), 'rate_hz': 0.0, 'median_dt': float('nan'),
                'max_gap': float('nan'), 'non_monotonic': 0, 'duplicates': 0}
    dt = np.diff(t)
    span = t[-1] - t[0]
    return {
        'n': int(t.size),
        'rate_hz': float((t.size - 1) / span) if span > 0 else 0.0,
        'median_dt': float(np.median(dt)),
        'max_gap': float(dt.max()),
        'non_monotonic': int(np.sum(dt < 0)),
        'duplicates': int(np.sum(dt == 0)),
    }


def interp_truth(truth, t):
    """Interpolate truth (position, yaw, velocity, yaw rate) at times ``t``."""
    tt = truth[:, TR_T]
    pos = np.column_stack([np.interp(t, tt, truth[:, c]) for c in (1, 2, 3)])
    yaw = wrap(np.interp(t, tt, np.unwrap(truth[:, TR_YAW])))
    vel = np.column_stack([np.interp(t, tt, truth[:, c]) for c in (5, 6, 7)])
    wz = np.interp(t, tt, truth[:, TR_WZ])
    return pos, yaw, vel, wz


def stationary_mask(truth, t, max_speed=0.05, max_yaw_rate=0.02):
    """True where the vehicle is neither translating nor turning."""
    _, _, vel, wz = interp_truth(truth, t)
    return (np.linalg.norm(vel, axis=1) < max_speed) & (np.abs(wz) < max_yaw_rate)


def gps_error(gps, truth):
    """GPS position error against truth: per-sample error and statistics."""
    t = gps[:, GPS_T]
    pos, _, _, _ = interp_truth(truth, t)
    err = gps[:, GPS_P] - pos
    return {
        't': t,
        'err': err,
        'mean': err.mean(axis=0),
        'std': err.std(axis=0, ddof=1) if len(err) > 1 else np.zeros(3),
        'rms_horizontal': float(np.sqrt(np.mean(np.sum(err[:, :2] ** 2, axis=1)))),
    }


def body_from_enu(vel_enu, yaw):
    """Rotate ENU velocity rows into the yaw-only body frame."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack([
        c * vel_enu[:, 0] + s * vel_enu[:, 1],
        -s * vel_enu[:, 0] + c * vel_enu[:, 1],
        vel_enu[:, 2],
    ])


def vo_drift(vo, truth):
    """VO pose drift and body-velocity error against truth.

    Pose drift is reported relative to the first VO sample's offset from truth,
    so it measures accumulated drift rather than a constant frame offset.
    Body velocity is fitted per axis as ``v_vo = scale * v_true + bias``.
    """
    t = vo[:, VO_T]
    pos, yaw, vel, _ = interp_truth(truth, t)
    pos_err = vo[:, VO_P] - pos
    yaw_err = wrap(vo[:, VO_YAW] - yaw)
    drift = pos_err - pos_err[0]
    yaw_drift = wrap(yaw_err - yaw_err[0])
    duration = t[-1] - t[0] if len(t) > 1 else 0.0

    vb_true = body_from_enu(vel, yaw)
    vb_vo = vo[:, VO_VB]
    fits = []
    for axis in range(3):
        a = np.column_stack([vb_true[:, axis], np.ones(len(t))])
        moving = np.abs(vb_true[:, axis]).max() > 0.1
        if moving:
            (scale, bias), *_ = np.linalg.lstsq(a, vb_vo[:, axis], rcond=None)
        else:  # Scale unobservable without motion on this axis.
            scale, bias = float('nan'), float(np.mean(vb_vo[:, axis] - vb_true[:, axis]))
        fits.append((float(scale), float(bias)))

    return {
        't': t,
        'pos_err': pos_err,
        'yaw_err': yaw_err,
        'drift': drift,
        'yaw_drift': yaw_drift,
        'final_drift_m': float(np.linalg.norm(drift[-1])) if len(t) else 0.0,
        'final_yaw_drift_deg': float(np.degrees(yaw_drift[-1])) if len(t) else 0.0,
        'drift_rate_m_per_min': (float(np.linalg.norm(drift[-1]) / duration * 60.0)
                                 if duration > 0 else 0.0),
        'vb_true': vb_true,
        'vb_err': vb_vo - vb_true,
        'vb_fit': fits,
    }


def gyro_bias(imu, truth):
    """Gyro Z error against truth, overall and while stationary."""
    t = imu[:, IMU_T]
    _, _, _, wz = interp_truth(truth, t)
    err = imu[:, IMU_WZ] - wz
    still = stationary_mask(truth, t)
    out = {
        't': t,
        'err': err,
        'still': still,
        'bias_all': float(err.mean()),
        'noise_std': float(err.std(ddof=1)) if len(err) > 1 else 0.0,
    }
    out['bias_stationary'] = float(err[still].mean()) if still.any() else float('nan')
    out['stationary_samples'] = int(still.sum())
    return out


def gyro_bias_truth_free(imu, t0, t1):
    """Mean gyro Z between t0 and t1 — the bias if the drone did not turn."""
    m = (imu[:, IMU_T] >= t0) & (imu[:, IMU_T] <= t1)
    return float(imu[m, IMU_WZ].mean()) if m.any() else float('nan')


def summarize(data):
    """Return a JSON-friendly summary of everything measurable in ``data``."""
    out = {'rates': {k: rate_stats(v[:, 0]) for k, v in data.items()}}
    truth = data.get('truth')
    if truth is None:
        return out
    if 'gps' in data:
        g = gps_error(data['gps'], truth)
        out['gps'] = {'mean_err_m': g['mean'].tolist(), 'std_m': g['std'].tolist(),
                      'rms_horizontal_m': g['rms_horizontal']}
    if 'vo' in data:
        v = vo_drift(data['vo'], truth)
        out['vo'] = {
            'duration_s': float(v['t'][-1] - v['t'][0]),
            'final_drift_m': v['final_drift_m'],
            'drift_rate_m_per_min': v['drift_rate_m_per_min'],
            'final_yaw_drift_deg': v['final_yaw_drift_deg'],
            'body_velocity_scale_bias': v['vb_fit'],
            'body_velocity_err_std': v['vb_err'].std(axis=0).tolist(),
        }
    if 'imu' in data:
        b = gyro_bias(data['imu'], truth)
        out['gyro'] = {
            'bias_all_rad_s': b['bias_all'],
            'bias_stationary_rad_s': b['bias_stationary'],
            'bias_stationary_deg_s': float(np.degrees(b['bias_stationary'])),
            'noise_std_rad_s': b['noise_std'],
            'stationary_samples': b['stationary_samples'],
        }
    return out


def reproducibility(data_a, data_b, n=25):
    """Compare GPS error sequences of two runs.

    The GPS noise draws come from the seeded RNG in a fixed order, so the first
    ``n`` GPS errors of two runs with the same seed coincide (up to the tiny
    truth-interpolation difference), while different seeds are uncorrelated.
    """
    ea = gps_error(data_a['gps'], data_a['truth'])['err'][:n]
    eb = gps_error(data_b['gps'], data_b['truth'])['err'][:n]
    k = min(len(ea), len(eb))
    ea, eb = ea[:k], eb[:k]
    corr = float(np.corrcoef(ea.ravel(), eb.ravel())[0, 1]) if k > 2 else float('nan')
    return {'samples': k, 'max_abs_diff_m': float(np.abs(ea - eb).max()) if k else 0.0,
            'correlation': corr}
