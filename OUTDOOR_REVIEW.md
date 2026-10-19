# Outdoor mission review

Living list: append topics; mark resolved only when the decision/evidence is recorded.
“Resolved” below describes the software contract, not real-vehicle validation.

| ID | Topic | Status | Core answer / next step |
|---|---|---|---|
| 01 | Position and odometry source | Resolved | Control uses MAVROS `local_position/pose` and `velocity_local` in ENU. We do not subscribe to `local_position/odom`. MAVROS converts FCU data; it does not fuse sensors. |
| 02 | Sensor fusion | Open: confirm live configuration | Saved SITL parameters (`~/ardupilot/ArduCopter/mav.parm`, modified 2026-10-18) show `AHRS_EKF_TYPE=10` (Sim), despite `EK3_ENABLE=1`: EKF3 enabled does not mean selected. SRC1 config is GPS XY position/XY+Z velocity, barometer height, compass yaw. Confirm live estimator/source set; real-vehicle fusion remains unverified. |
| 03 | Measurement noise / positioning accuracy | Open: measure | FCU fusion rejects inconsistent measurements; our node adds finite/fresh-input checks, bounded targets and continuous arrival dwell. No additional filter, covariance gate or estimator-health subscription. A 0.25 m arrival tolerance is relative to the estimate, not proof of 0.25 m physical accuracy. |
| 04 | Compass precision and yaw | Open: verify vehicle | Control uses fused local attitude, not raw magnetometer values. `compass_hdg` is the FCU-reported global heading. Check calibration, configured yaw source and frames; compare using bags/diagnostics. Heading freshness/validity is preflight-only; mismatch never commands LAND. A biased fused yaw can still bias the flight. |
| 05 | Coordinate consistency | Resolved: diagnostics | GPS/local XY and local/relative altitude differences are reported, with flags at configured thresholds. They do not abort. Home/reference changes and local-frame changes remain flight guards. |
| 06 | Fault responses | Resolved: current policy | See table below. Review against estimator/failsafe evidence before real flight. |
| 07 | Readiness vs flight health | Resolved | Preflight additionally requires usable heading, relative altitude, on-ground/disarmed capture and readiness dwell. In flight, require navigation telemetry; relative altitude is additionally required during TAKEOFF. |
| 08 | Clock and reference timing | Open: review | Central clock reset/jump/stall supervision plus per-target timestep guard. Reference pairing still compares arrival times, not synchronized sensor timestamps. Separate heartbeat timeout remains unimplemented. |
| 09 | Estimator and onboard failsafes | Open | Inspect EKF health/reset reporting and vehicle GPS/EKF/link failsafe settings; mission node currently does not subscribe to estimator-health messages. |
| 10 | Flight evidence | Open | Record two complete runs, abort/override, commanded vs actual track and yaw error. Measure noise before changing tolerances. |
| 11 | Transient GPS faults | Open | Runtime GPS validity/satellite faults still act immediately. Review persistence against FCU estimator health rather than adding arbitrary delays. |
| 12 | Executor and callback timing | Resolved: serial; timing open | Single-threaded executor, default callback group, no shared mutex. Services remain asynchronous. Diagnostics likely costs most per regular invocation; measure callback duration and setpoint jitter under load. |
| 13 | Vehicle topic/service prefix | Deferred | Add a configurable MAVROS interface prefix: `/mavros` in simulation, `/droneXX` on the real vehicle. Apply consistently to telemetry, setpoints and services; expose in launch/YAML. Keep mission-owned topics separate. |
| 14 | Landed-state stream setup | Resolved: one-shot | Wait for connected FCU/service; request 245 at 2 Hz once. Success + fresh data gates readiness. Rejection/response timeout → terminal FAILED in either mode; node stays alive; restart required. No retries/reconnection setup. |

## Current fault responses

| Condition | Response |
|---|---|
| Invalid configuration | Exit at startup. |
| Startup landed-state request rejected or response timed out | FAILED, no arming; restart required. |
| Incomplete preflight data, poor GPS/satellites or invalid envelope | Wait; do not arm. |
| Operator abort; stale/invalid GPS, satellites, pose, velocity or landed-state telemetry; invalid attitude/frame; clock stall/forward jump; changed home/frame; flight-bound or tracking-error violation; takeoff relative-altitude loss; unexpected disarm; navigation command rejection/timeout | If still owning connected GUIDED, stop mission setpoints and request LAND. Confirm ground/disarm → ABORTED. Otherwise abort while disarmed or release control. |
| FCU disconnected/stale, pilot/autopilot mode override, unexpected arming before mission arming | Stop setpoints → RELEASE_CONTROL; do not reclaim control. |
| Backward ROS clock reset during an active mission | Invalidate old samples → RELEASE_CONTROL; restart required. |
| LAND/disarm rejected or timed out | FAILED; operator recovery; no forced in-air disarm. |
| Compass mismatch/loss after takeoff; GPS/local or altitude disagreement alone | Diagnostics only; no abort. |

## Simplification decisions

Separate preflight and runtime checks; remove duplicate satellite/POI-yaw/timeout logic;
reuse local yaw conversion; centralize clock supervision. Keep publish-time authority,
service acknowledgement + observed state, stale-response generations and ground-only disarm guards.

## Sources

- [ArduPilot EKF](https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html)
- [EKF source selection](https://ardupilot.org/copter/docs/common-ekf-sources.html)
- [MAVROS local position](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/local_position.cpp)
- [MAVROS global position](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/global_position.cpp)
