> **Archived development document.** Progress notes, examples and TODOs below
> describe earlier revisions and are not the current acceptance checklist.
> See [DOCUMENTATION_AND_DEFENCE.md](../../DOCUMENTATION_AND_DEFENCE.md).

# Outdoor mission review

Living list: append topics; mark resolved only when the decision/evidence is recorded.
“Resolved” below describes the software contract, not real-vehicle validation.

| ID | Topic | Status | Core answer / next step |
|---|---|---|---|
| 01 | Position and odometry source | Resolved | Control uses MAVROS `local_position/pose` and `velocity_local` in ENU. We do not subscribe to `local_position/odom`. MAVROS converts FCU data; it does not fuse sensors. |
| 02 | Sensor fusion | Open: confirm live configuration | Latest saved SITL parameters (`~/ardupilot/ArduCopter/mav.parm`, modified 2026-10-20) show `AHRS_EKF_TYPE=3`, `EK3_ENABLE=1`; earlier snapshot used Sim (10). Saved configuration does not establish which estimator was active in past recordings. SRC1 config is GPS XY position/XY+Z velocity, barometer height, compass yaw. Real-vehicle fusion remains unverified. |
| 03 | Measurement noise / positioning accuracy | Open: measure | FCU fusion rejects inconsistent measurements; our node adds finite/fresh-input checks, bounded targets and continuous arrival dwell. No additional filter, covariance gate or estimator-health subscription. A 0.25 m arrival tolerance is relative to the estimate, not proof of 0.25 m physical accuracy. |
| 04 | Compass precision and yaw | Open: verify vehicle | Control uses fused local attitude, not raw magnetometer values. `compass_hdg` is the FCU-reported global heading. Check calibration, configured yaw source and frames; compare using bags/diagnostics. Heading freshness/validity is preflight-only; mismatch never commands LAND. A biased fused yaw can still bias the flight. |
| 05 | Coordinate consistency | Resolved: diagnostics | GPS/local XY and local/relative altitude differences are reported, with flags at configured thresholds. They do not abort. Home/reference changes and local-frame changes remain flight guards. |
| 06 | Fault responses | Resolved: current policy | See table below. Review against estimator/failsafe evidence before real flight. |
| 07 | Readiness vs flight health | Resolved | Preflight additionally requires usable heading, relative altitude, on-ground/disarmed capture and readiness dwell. In flight, require navigation telemetry; relative altitude is additionally required during TAKEOFF. |
| 08 | Clock and reference timing | Open: review | Central clock reset/jump/stall supervision plus per-target timestep guard. Reference pairing still compares arrival times, not synchronized sensor timestamps. Separate heartbeat timeout remains unimplemented. |
| 09 | Estimator and onboard failsafes | Partial: configuration inspected | Node checks telemetry/flight state, not EKF health or resets directly; onboard mode changes release mission control. Latest saved SITL: EKF failsafe LAND in position modes (`FS_EKF_ACTION=1`, threshold 0.8); GCS failsafe disabled (`FS_GCS_ENABLE=0`, timeout 5 s). Confirm live values and test separately: estimator fault, MAVROS/link loss, mission-process loss. A running MAVROS/GCS can keep heartbeat alive after mission exit. No failsafe parameters changed. |
| 10 | Flight evidence | Partial: recordings linked | Developer recorded outdoor runs: [recordings](https://drive.google.com/drive/folders/18p4L7dQqtNPm-RtJtOg_cNxpsdhLNMUm?usp=sharing). Contents not yet reviewed here; confirm complete-run count and abort/override coverage, then quantify commanded vs actual track and yaw error. GPS and magnetometer noise tests remain pending. |
| 11 | Transient GPS faults | Resolved: current policy; satellite test passed | Developer reduced simulated satellite count below the configured minimum and observed landing. Keep the current runtime GPS/satellite abort policy; no diagnostic-only change or debounce added. Other GPS fault cases and estimator interaction remain unverified. |
| 12 | Executor and callback timing | Resolved: cadence checked in SITL | Single-threaded executor, default callback group, no shared mutex; asynchronous services. Developer checked setpoint cadence during the simulation run and accepted it. Exact rate/gaps and callback durations were not recorded here; no general load/timing guarantee claimed. |
| 13 | Vehicle topic/service prefix | Implemented and tested: outdoor | `mavros_prefix` defaults to `/mavros`; YAML and all outdoor launches expose it for e.g. `/droneXX`. Applies to all MAVROS subscriptions, setpoint publication and service clients; mission-owned interfaces remain separate. Default/custom-prefix fake-telemetry tests and custom flight-interface graph check passed. Does not rename/start MAVROS or change the indoor executor. |
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

## Control-loss observations

- MAVROS stopped during outdoor flight: mission reported `RELEASE_CONTROL: FCU
  connection lost; defer to onboard failsafes`; developer observed the vehicle
  remain airborne. Accepted for the current SITL setup.
- Mission node stopped: developer observed the vehicle stop and hold in the air.
  Accepted; automatic landing on mission-process exit is not required for this setup.
- These observations establish behaviour in the tested configuration, not a
  guarantee under complete link loss or estimator failure. Onboard EKF-failsafe
  behaviour remains untested; no heartbeat watchdog or failsafe changes added.

## GPS satellite-count test

Developer reduced the GPS satellite count below the configured minimum during
flight and observed landing. This confirms the existing satellite guard's LAND
response in the tested SITL configuration; the current policy is retained.

## Noise-test observations

- GPS altitude: `SIM_GPS1_NOISE=2` produced no visible change in flight behaviour.
- Barometer noise: developer also observed no visible change; applied amplitudes
  and affected barometer instances were not recorded here.
- These are visual observations, not measured robustness results. Active estimator,
  raw sensor changes and ground-truth errors were not confirmed for these runs.
  Quantitative testing and magnetometer disturbance testing remain open; move on
  without changing mission tolerances or fault policy on this evidence alone.

## Sources

- [ArduPilot EKF](https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html)
- [EKF source selection](https://ardupilot.org/copter/docs/common-ekf-sources.html)
- [MAVROS local position](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/local_position.cpp)
- [MAVROS global position](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/global_position.cpp)
- [ArduPilot EKF failsafe](https://ardupilot.org/copter/docs/common-ekf-inav-failsafe.html)
- [ArduPilot GCS failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html)
