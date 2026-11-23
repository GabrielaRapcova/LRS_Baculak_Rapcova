"""Read localisation bags into plain NumPy arrays.

The arrays use one row per message and these columns:

    gps   : t, x, y, z, recv
    vo    : t, x, y, z, yaw, vbx, vby, vbz, recv
    imu   : t, wz, recv
    truth : t, x, y, z, yaw, vx, vy, vz, wz, recv
    est   : t, x, y, z, vx, vy, vz, yaw, bias, 8 sigmas
            (from /localization/estimate_state, same layout as ekf_replay output)

``t`` is the header stamp (simulation time) and ``recv`` the bag receive time,
both in seconds. Only this module depends on ROS, so the analysis functions can
be exercised on synthetic data without a ROS installation.
"""

import math

import numpy as np

TOPICS = {
    'gps': '/localization/gps',
    'vo': '/localization/vo',
    'imu': '/localization/imu',
    'truth': '/evaluation/ground_truth',
    'est': '/localization/estimate_state',
}


def _stamp(header):
    return header.stamp.sec + 1e-9 * header.stamp.nanosec


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _row(kind, msg, recv):
    if kind == 'gps':
        p = msg.pose.pose.position
        return [_stamp(msg.header), p.x, p.y, p.z, recv]
    if kind == 'imu':
        return [_stamp(msg.header), msg.angular_velocity.z, recv]
    if kind == 'est':
        return list(msg.data[:17]) if len(msg.data) >= 17 else None
    # Odometry-based streams.
    p = msg.pose.pose.position
    v = msg.twist.twist.linear
    row = [_stamp(msg.header), p.x, p.y, p.z, _yaw(msg.pose.pose.orientation),
           v.x, v.y, v.z]
    if kind == 'truth':
        row.append(msg.twist.twist.angular.z)
    row.append(recv)
    return row


def read_bag(path, topics=None):
    """Return {kind: ndarray} for the localisation topics present in a bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    topics = topics or TOPICS
    wanted = {name: kind for kind, name in topics.items()}

    reader = rosbag2_py.SequentialReader()
    storage_id = 'mcap' if str(path).endswith('.mcap') else ''
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(types[name])
                 for name in wanted if name in types}

    rows = {kind: [] for kind in topics}
    while reader.has_next():
        name, data, recv_ns = reader.read_next()
        if name not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[name])
        row = _row(wanted[name], msg, recv_ns * 1e-9)
        if row is not None:
            rows[wanted[name]].append(row)

    return {kind: np.asarray(r, dtype=float) for kind, r in rows.items() if r}
