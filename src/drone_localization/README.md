# drone_localization — Assignment 2, drone localisation

An eight-state EKF that fuses noisy GPS, drifting visual odometry and a biased
yaw-rate gyro from `localization_sensor_sim`. This README grows with each
increment; the full design document follows at the end.

| Increment | Status |
|---|---|
| 1. Sensor acquisition and characterisation (§1) | **done** |
| 2. EKF core: predict + GPS | **done** |
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

It subscribes to `/localization/{gps,vo,imu}` with `SensorDataQoS` and prints
a report every `window_s` (default 10 s) of **simulation** time. The window is
measured on IMU stamps, so it holds the same amount of data at any real-time
factor. Our Gazebo runs at about 0.09× real time, so one report arrives about
every 2 minutes of wall time. Each report shows:

- the rate from **header stamps** (simulation time) and from arrival time;
- frame checks: GPS `map`, VO `vo_odom` → `base_link`, IMU `base_link`;
- duplicate or out-of-order stamps, and the largest gap;
- truth-free corruption evidence while the drone is still: GPS std (noise),
  mean gyro Z (bias), VO displacement over the window (drift).

It never subscribes to ground truth or MAVROS localisation.

- the real-time factor of that window.

A first live run on seed 42 showed every stream at its nominal stamp rate
(5 / 30 / 100 Hz), no frame errors and no out-of-order stamps. The gyro Z mean
at rest was **−0.017 rad/s** (−1.0°/s). Gazebo draws the bias sign at random,
so it is negative here although the world's `bias_mean` is +0.02.

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
| gyro Z bias | −0.028 rad/s (−1.6°/s) | +0.018 rad/s (1.0°/s) |

(The simulator's gyro RNG is not Gazebo's, so real-run biases differ.)

## Estimator core (increment 2)

```
include/drone_localization/ekf.hpp        8-state EKF: predict, generic gated Joseph update
include/drone_localization/localizer.hpp  initialisation, timing, per-sensor stats, logging
src/ekf_replay.cpp                        CSV replay of a sensor stream through Localizer
test/test_ekf.cpp                         gtests
```

The core is plain C++17/Eigen with no ROS, so the ROS node, the offline replay
and the unit tests all run the same code.

**State** (ENU `map`): `x = [px py pz vx vy vz yaw b]`, where `b` is the gyro Z
bias (true yaw rate = `gyro_z − b`).

**Prediction** runs on every IMU message, using the dt between header stamps:

| | model | process noise |
|---|---|---|
| p, v | constant velocity | white acceleration, `accel_xy` = 0.5, `accel_z` = 0.3 m²/s³ |
| yaw | `yaw += (gyro_z − b)·dt`, wrapped | gyro σ = 0.004 rad/s per 100 Hz sample |
| b | random walk | `bias_walk` = 1e-4 rad/s/√s |

The yaw/bias Jacobian `∂yaw/∂b = −dt` is what lets yaw measurements correct
the bias later.

**GPS update:** `h(x) = p`, with R taken from the message covariance. It uses
the Joseph form and a χ² gate: 16.27 = 3 DOF at 99.9 %.

**Timing.** Every measurement is applied at its own header stamp. When a stamp
is ahead of the filter, the filter is predicted to that stamp with the last
gyro rate, then updated. A stamp up to `max_measurement_lag` (0.05 s) behind
the filter is applied at the filter time and counted as `late`. Anything older
is dropped and counted as `stale`. An IMU sample at or before the filter time
is never integrated twice.

**Initialisation** uses data only, never truth:

- position is the component-wise **median** of the first 10 GPS fixes, which is
  robust to an injected outlier, with variance (π/2)·σ²/n;
- yaw is the configured initial ENU yaw (π/2), σ = 10°;
- v = 0, σ = 0.2 m/s;
- bias is the **mean gyro Z collected on the ground** before the 10th fix, with
  σ = its standard error (floored at 0.003 rad/s). This assumes the drone is
  at rest when the estimator starts; the filter keeps refining the bias later.

### Tests and offline evaluation

```bash
colcon test --packages-select drone_localization --event-handlers console_direct+
ros2 run drone_localization evaluate_replay.py --sim 42 --sim 73 --sim 7
ros2 run drone_localization evaluate_replay.py --bag rosbags/char_seed42   # real run
```

`evaluate_replay.py` sends the sensor stream through the `ekf_replay` binary
(the same `Localizer`) and scores it against truth afterwards. It writes
error-vs-time plots with the filter's own ±2σ bands. Filter parameters can be
overridden as `key=value`, for example `accel_xy=0.8`.

GPS + gyro only, simulated §4 mission, five seeds:

| metric | result |
|---|---|
| horizontal RMS / max | 0.48–0.54 m / 1.2–1.6 m |
| z RMS | 0.41–0.50 m |
| yaw RMS | 0.5–2.3° (rests on the ground bias estimate only) |
| velocity RMS | 0.55–0.63 m/s, poor: GPS alone barely observes velocity |
| errors within 2σ | 0.95–1.00, so the covariance is consistent |

Yaw and bias are **unobservable** with GPS alone, so their uncertainty keeps
growing. Increment 3 adds the VO yaw and body-velocity updates.
