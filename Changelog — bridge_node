# Changelog — bridge_node

All notable changes to the Autoware ↔ CarMaker bridge. Dates are 2026. Format follows Keep-a-Changelog style; versions follow the project's milestone tags.

## [v1.0-integration-freeze] — Jul 15 · commit 10a69e8
**M4: full autonomous loop at 40 km/h on the Al Marjan network.**
### Added
- PI velocity controller (Kp 0.3, Ki 0.05, anti-windup ±2.0, standstill hold brake 0.3) replacing the open-loop accel→gas map, which saturated on grades and produced brake spikes.
- Measured-dt integration (real elapsed time per control tick, not a hardcoded constant).
- Steering ratio ×6.7 on the forward path (Autoware tire angle → CarMaker steering-wheel angle, per the vehicle's Rack2StWhl table).
- Acceleration low-pass filter (0.7 old + 0.3 raw) on the differentiated velocity feed.
- IMU relay to /sensing/imu/imu_data; heading_rate wired into VelocityReport.
- MultiThreadedExecutor so callbacks don't queue behind the control timer.
### Changed
- ZOH timer 1 ms → 20 ms (50 Hz). The 1 ms Python timer consumed 94% of a core and starved the pose/state callbacks (kinematic_state fell to 8.7 Hz); 50 Hz restored 100 Hz feedback.
### Fixed
- Velocity cap at 15 km/h: raised common.param.yaml max_vel 4.17 → 11.1 m/s (Autoware-side config, recorded here because the bridge run procedure depends on it).

## Week 6 (Jul 6–10) — "Layer 2" commits
**True closed loop achieved: fake sim retired, Autoware drives CarMaker's real physics.**
### Added
- Pose converter: /carmaker/pose → nav_msgs/Odometry on /localization/kinematic_state (yaw→quaternion, qz/qw) with twist populated — empty twist makes Autoware's smoother believe the car is parked.
- TF broadcaster map→base_link (the fake sim used to provide it).
- /localization/acceleration publisher (differentiated velocity with first-callback and dt>0 guards) — root cause of the engage block: 0 publishers, 11 subscribers.
- Return-path fan-out: VelocityReport, SteeringReport, ControlModeReport from /carmaker/vehicle_state.
### Changed
- Operates with simple_planning_simulator disabled (launch_dummy_vehicle=false, planning_simulator.launch.xml line 96).

## Week 5 — return path foundations
### Added
- Subscription to CMNode's /carmaker/vehicle_state (Car.ConBdy1.v, Car.Susp[0].SteerAngle @ 100 Hz).
- First closed-loop drive at 25 km/h via AUTO engage (M3 milestone; simple_planning_simulator still co-running at that point — true closed loop landed in Week 6).

## Weeks 3–4 — [Part 1 baseline] — see docs/HANDOVER_PART1
### Added
- Forward path: /control/command/control_cmd → gas/brake/steer conversion → /carmaker/VehicleControl with use_vc, selector_ctrl.
- Original ZOH design; ament_python packaging.

## Pending (designed, not merged)
- Staleness guard: if CarMaker feedback age > 100 ms → one clear ERROR, zero gas/steer, freeze integrator. Motivated by the Jul 16 mid-route freeze incident (CarMaker sim halted; bridge fought 16-second-old velocity at full brake).
