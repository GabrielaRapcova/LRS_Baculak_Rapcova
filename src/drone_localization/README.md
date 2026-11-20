# drone_localization — Assignment 2, drone localisation

An eight-state EKF that fuses noisy GPS, drifting visual odometry and a biased
yaw-rate gyro from `localization_sensor_sim`. This README grows with each
increment; the full design document follows at the end.

| Increment | Status |
|---|---|
| 1. Sensor acquisition and characterisation (§1) | **done** |
| 2. EKF core: predict + GPS | – |
| 3. Yaw, gyro bias, VO updates | – |
| 4. ROS estimator node + evaluator | – |
| 5. Outages, outlier gating (§3) | – |
| 6. Flight on the estimate (§4), separate package | – |
| 7. Documentation | – |

## Build

From the workspace root (`ros2_ws`):

```bash
colcon build --packages-select drone_localization
source install/setup.bash
```

`localization_sensor_sim` lives at the course repository root. Build it once
into the same workspace so `ros2 launch` and the offline simulator can find it:

```bash
colcon build --base-paths src ../localization_sensor_sim \
  --packages-select localization_sensor_sim drone_localization
```

## §1 Sensor acquisition and characterisation

### Bring-up (repository root, separate terminals)

```bash
GZ_SIM_RESOURCE_PATH="$PWD/models:${GZ_SIM_RESOURCE_PATH:-}" \
  gz sim -r -v 4 --seed 42 "$PWD/worlds/fei_lrs_gazebo_localization.world"
scripts/run_sitl.sh
scripts/run_mavros.sh
ros2 launch localization_sensor_sim localization_sensors.launch.py seed:=42
```

### Live check: `sensor_check_node`

```bash
ros2 launch drone_localization sensor_check.launch.py
```

It subscribes to `/localization/{gps,vo,imu}` with `SensorDataQoS` and every
5 s prints:

- the rate from **header stamps** (simulation time) and from arrival time;
- frame checks: GPS `map`, VO `vo_odom` → `base_link`, IMU `base_link`;
- duplicate or out-of-order stamps, and the largest gap;
- truth-free corruption evidence while the drone is still: GPS std (noise),
  mean gyro Z (bias), VO displacement over the window (drift).

It never subscribes to ground truth or MAVROS localisation.

Quick manual checks:

```bash
ros2 topic hz /localization/imu
ros2 topic echo --once /localization/vo --field child_frame_id
```

### Recording and plots

1. Start recording in `ros2_ws/rosbags/`, which is git-ignored:
   `cd rosbags && ros2 run drone_localization record_characterisation.sh char_seed42`
2. Fly the open-loop characterisation profile:
   `ros2 run drone_localization characterisation_flight.py --ros-args -p use_sim_time:=true`.
   This is 20 s on the ground, takeoff to 2 m, 15 s hover, a 3 m square at
   0.5 m/s, then ±180° yaw turns and landing. It uses timed velocity setpoints
   only, with no position feedback. It is a characterisation tool, not part of
   the submitted controller.
3. Stop the recording after `landed and disarmed`.
4. Analyse:

```bash
ros2 run drone_localization characterize_sensors.py char_seed42
```

This writes `rates.png`, `gps_noise.png`, `vo_drift.png`, `gyro_bias.png`,
`trajectory.png` and `summary.json`. Ground truth is read from the bag
**offline only**, to measure the errors.

### Seed comparison

Repeat with a second seed (Gazebo `--seed 73` **and** `seed:=73` for the sensor
launch), and once more with seed 42. Then:

```bash
ros2 run drone_localization characterize_sensors.py char_seed42 --compare char_seed73
ros2 run drone_localization characterize_sensors.py char_seed42 --compare char_seed42b
```

- **Same seed:** identical hidden biases (gyro bias, VO body-velocity bias and
  scale) and a GPS error sequence with correlation ≈ 1.
- **Different seed:** a different gyro bias and VO bias, and GPS error
  correlation ≈ 0.

The GPS sequence only matches if no truth message was dropped between runs.
The hidden biases are drawn first, so they match regardless.

### Offline simulator (no Gazebo)

`drone_localization_tools.sim` drives the course's own
`sensor_model.SensorCorruptor` with a synthetic trajectory, plus a Gazebo-like
biased gyro. It's used to develop and regression-test the estimator. Expected
characterisation from simulated seed 42 / 73:

| | seed 42 | seed 73 |
|---|---|---|
| GPS std x/y/z [m] | 0.60 / 0.60 / 0.91 | 0.58 / 0.58 / 0.88 |
| VO drift | 1.10 m in 104 s, yaw −4.5° | 0.91 m in 104 s, yaw +2.8° |
| gyro Z bias | +0.028 rad/s (1.6°/s) | +0.018 rad/s (1.0°/s) |

(The simulator's gyro RNG is not Gazebo's, so real-run biases differ.)
