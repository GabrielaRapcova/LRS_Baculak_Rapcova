# localization_flight — Assignment 2 §4, flight on the estimate

`estimated_mission_node` flies the two-lap route **using only
`/localization/estimate`** (from `drone_localization`) as position and yaw
feedback. It is a standalone node; the A1 `mission_executor` is not used.

```
 /localization/gps ─┐
 /localization/vo  ─┼─► localization_estimator ──► /localization/estimate ──► estimated_mission ──► /mavros/setpoint_raw/local ──► MAVROS ──► ArduPilot
 /localization/imu ─┘   (8-state EKF)               (Odometry, map ENU)       (state machine +      (velocity + yaw rate, ENU)
                                                                                 P controller)  ◄── /mavros/state, set_mode / arming / takeoff
 /evaluation/ground_truth ──► estimate_evaluator   (separate; publishes only /evaluation/*, never read by the system)
```

## What the node uses

| uses | does **not** use |
|---|---|
| `/localization/estimate`: position, yaw, σ_xy | `/mavros/local_position/*`, `/mavros/global_position/*`, `/mavros/odometry/*`, `/mavros/imu/*` |
| `/mavros/state`: connected, armed, mode | `/evaluation/ground_truth` or any Gazebo pose |
| `/mavros/set_mode`, `/mavros/cmd/arming`, `/mavros/cmd/takeoff` | |
| publishes `/mavros/setpoint_raw/local`: **velocity + yaw rate** | position setpoints |

**Why velocity setpoints?** With position setpoints, ArduPilot would close the
position loop on *its own* EKF, and our estimate would only decide when a
waypoint counts as reached. With velocity setpoints, the position loop is
closed here, on our estimate:

```
v_xy  = (target − p̂)_xy / |…| · min(0.5 m/s, 0.8·|…|)    straight line at 0.5 m/s, slowing inside 0.6 m
v_z   = clamp(0.8·(z_target − ẑ), ±0.4 m/s)
ψ̇     = clamp(1.0·wrap(ψ_origin − ψ̂), ±0.3 rad/s)          holds the origin heading
```

ArduPilot only executes velocity and yaw rate, and those are not position
feedback.

## Sequence (unattended after launch)

1. **WAIT_READY**: FCU connected, and an estimate fresher than 0.5 s with
   σ_xy < 0.3 m. The node then **settles on the ground** for at least 10 s of sim time (about 2 min of wall time at a 0.09 real-time factor; progress is logged every 2 s),
   until the estimator's gyro-bias σ is below 2.5e-4 rad/s. That estimate
   becomes the **mission origin** (x, y, z, yaw).
2. **SET_GUIDED → ARM**: retried until accepted (ArduPilot rejects arming
   while its EKF settles).
3. **TAKEOFF**: MAVROS takeoff service to 2 m. The phase ends when the
   **estimated** height reaches 70 % of 2 m. No setpoints are streamed during
   the takeoff command (same as A1).
4. **CLIMB**: estimate-driven velocity control to origin + (0, 0, 2 m).
5. **ROUTE**: (0,0) → (0,−4) → (2.5,−4) → (2.5,0) → (0,0), twice. A waypoint
   is reached when the estimate is within 0.3 m horizontally and 0.35 m
   vertically.
6. **FINAL_HOLD**: 3 s above the origin, then **LAND** mode, then wait for
   the auto-disarm.

**Watchdogs.**
- Estimate older than 0.5 s: zero-velocity hover.
- Estimate older than 3 s, or σ_xy > 2 m: LAND.
- If the FCU leaves GUIDED during flight (pilot takeover), the node stops
  sending setpoints. That means a *real* other mode held for 1.5 s. An empty
  mode or a lost link (MAVROS emits those on missed heartbeats, common at low
  real-time factor) is treated as unknown, and the node keeps flying. The
  first live flight aborted on exactly that before this fix.

**The GPS outage** needs nothing special here. The estimator carries the
position on VO, and the controller keeps flying the estimate.

## Run

```bash
# repository root, separate terminals (any seed):
GZ_SIM_RESOURCE_PATH="$PWD/models:${GZ_SIM_RESOURCE_PATH:-}" \
  gz sim -r -v 4 --seed 42 "$PWD/worlds/fei_lrs_gazebo_localization.world"
scripts/run_sitl.sh
scripts/run_mavros.sh
ros2 launch localization_sensor_sim localization_sensors.launch.py seed:=42

# ros2_ws:
ros2 launch localization_flight mission.launch.py evaluator:=true
# teacher's outage test during a middle leg:
ros2 service call /localization/set_gps_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /localization/set_gps_enabled std_srvs/srv/SetBool "{data: true}"
```

To prove the feedback path:

```bash
ros2 node info /estimated_mission    # subscriptions: /localization/estimate, /mavros/state only
```

## Closed-loop test (no ROS or Gazebo)

`test/test_closed_loop.cpp` runs the real `Localizer` and this controller on a
vehicle with a lagged velocity loop. Its sensors reproduce the characterised
corruption: GPS noise, VO scale/bias/noise and yaw drift, and a gyro bias of
about ±0.02 rad/s. Each run includes **a 15 s GPS outage mid-route and an
outlier on the first fix afterwards**. The controller sees only the estimate.

| seed | route time | max cross-track (truth) | max altitude error | final offset from origin | outlier rejected |
|---|---|---|---|---|---|
| 1–6 | 52–54 s | 0.23–0.29 m (0.70 m worst, during the outage) | 0.19–0.54 m | 0.04–0.25 m | yes |

Full mission time (takeoff + 2 laps + hold + landing) is therefore about
70–80 s, inside the expected 60–90 s.

```bash
colcon test --packages-select localization_flight --event-handlers console_direct+
```
