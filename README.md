# Autoware ↔ IPG CarMaker ROS 2 Bridge (SP1)

**Closed-loop autonomous driving simulation: Autoware Universe drives a physics-accurate IPG CarMaker vehicle on a real HD map of Al Marjan Island.**

`ROS 2 Humble` · `Python 3 / rclpy` · `IPG CarMaker 15.0 + CMRosIF` · `Ubuntu 22.04` · `Autoware Universe (59468ee)`

---

## What This Is

RAKTA (Ras Al Khaimah Transport Authority) operates Level-4 autonomous vehicles whose vendor software is closed and encrypted — field anomalies cannot be inspected at the code level. This project builds the open-source digital twin that makes that inspection possible: **Autoware Universe** (the open autonomous-driving stack) drives a vehicle simulated by **IPG CarMaker** (industry-standard vehicle physics), connected by this bridge.

This repository is the bridge — the translator that closes the loop:

```
AUTOWARE UNIVERSE                  bridge_node.py                    IPG CARMAKER 15.0
(localization, planning,     (this repository)                 (vehicle physics, road,
 control — 45 ROS 2 nodes)                                      sensors — via CMRosIF)

/control/command/control_cmd ──▶  PI velocity controller  ──▶  /carmaker/VehicleControl
                                  steer ×6.7 (tire→wheel)
                                  ZOH republish @ 50 Hz

/vehicle/status/*            ◀──  state fan-out            ◀──  /carmaker/vehicle_state (100 Hz)
/localization/kinematic_state ◀── pose → Odometry + TF     ◀──  /carmaker/pose (100 Hz)
/localization/acceleration   ◀──  d(v)/dt + low-pass       ◀──  (differentiated)
/sensing/imu/imu_data        ◀──  IMU relay                ◀──  /carmaker/imu (100 Hz)
```

## Verified Result

**Full autonomous loop on the real Al Marjan Island road network at 40 km/h** — Autoware planning and control, CarMaker physics, no simulated vehicle anywhere in the chain. Velocity tracking error ≤ 0.43 m/s in steady state; zero minimum-risk-maneuver events on the final runs. (Tag: `v1.0-integration-freeze`.)

## Key Engineering Decisions

| Decision | Why |
|---|---|
| **PI controller on velocity** (Kp 0.3, Ki 0.05, anti-windup ±2.0) | Open-loop acceleration→gas mapping saturated on real road grades and produced brake spikes. Closed-loop velocity control tracks the planner's target through slope and drag. |
| **Steering ×6.7 on the forward path only** | Autoware commands *tire* angle; CarMaker's `VehicleControl.Steering.Ang` expects *steering-wheel* angle. Ratio measured from the vehicle's `Rack2StWhl` table. The return path is already tire angle — asymmetric on purpose. |
| **50 Hz zero-order hold** (not 1000 Hz) | A 1 ms Python timer starved every other callback (94% of a core). 50 Hz + measured-dt integration restored 100 Hz feedback with zero loss of control quality. |
| **Ground-truth localization feed** (current stage) | Control was proven on clean pose data before introducing sensor noise. The LiDAR→PCD→NDT stage replaces this next; the readiness gates are documented in `docs/`. |
| **Odometry twist always populated** | Autoware's velocity smoother treats an empty twist as a permanently parked car. |

## Repository Layout

```
bridge_node/
├── bridge_node/bridge_node.py   # the node (single file, ~230 lines, heavily commented)
├── package.xml / setup.py       # ament_python packaging
├── docs/
│   ├── HANDOVER_PART1.pdf       # bare machine → forward path (install, CMRosIF, license)
│   └── HANDOVER_PART2.md        # closed loop, maps, control tuning, 24-bug troubleshooting matrix
├── CHANGELOG.md
└── README.md
```

## Quick Start

Full rebuild-from-bare-machine instructions live in `docs/HANDOVER_PART1` (§1–6) and `docs/HANDOVER_PART2` (§16 runbook). Summary for a machine already set up:

```bash
# 1. CarMaker first (CMNode must exist before anyone subscribes)
cd ~/CM_Projects/FS_autonomous && bash CMStart.sh     # confirm: "hellocm: I am Alive"

# 2. Bridge second (subscriber ready before Autoware publishes)
cd ~/ros2_ws && source install/setup.bash && ros2 run bridge_node bridge_node

# 3. Autoware with the Al Marjan map
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/almarjan_demo \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

# 4. Initialize pose programmatically, set goal in RViz, engage AUTO.
```

Every step's verification command — and what to do when it fails — is in the runbook (`docs/HANDOVER_PART2.md` §16) and the troubleshooting matrix (§15, bugs #7–#30, each root-caused on this system).

## Interface Reference

| Direction | Topic | Type | Rate |
|---|---|---|---|
| ⬅ subscribe | `/control/command/control_cmd` | `autoware_control_msgs/Control` | ~10 Hz |
| ➡ publish | `/carmaker/VehicleControl` | `vehiclecontrol_msgs/VehicleControl` | 50 Hz (ZOH) |
| ⬅ subscribe | `/carmaker/vehicle_state` | `Float64MultiArray` [v, steer] | 100 Hz |
| ⬅ subscribe | `/carmaker/pose` | `Float64MultiArray` [x, y, z, yaw] | 100 Hz |
| ⬅ subscribe | `/carmaker/imu` | `sensor_msgs/Imu` | 100 Hz |
| ➡ publish | `/vehicle/status/velocity_status` · `steering_status` · `control_mode` | Autoware vehicle msgs | 100 Hz |
| ➡ publish | `/localization/kinematic_state` (+ TF `map→base_link`) | `nav_msgs/Odometry` | 100 Hz |
| ➡ publish | `/localization/acceleration` | `AccelWithCovarianceStamped` | 100 Hz |
| ➡ publish | `/sensing/imu/imu_data` | `sensor_msgs/Imu` | 100 Hz |

## Known Limitations / Roadmap

- **Staleness guard** (zero-command + integrator freeze when CarMaker feedback ages > 100 ms) — designed, not yet merged.
- **NDT localization** — gated on LiDAR point verification and the PointCloud→PointCloud2 conversion; six readiness gates documented in `docs/HANDOVER_PART2.md` §18.
- **Vehicle swap to the Yutong robobus** — requires re-reading the steering ratio and re-tuning PI for bus mass.

## Author & Context

**Ali Alsaid** — Systems Integration Engineer (intern), RAKTA Autonomous Office · EEE, American University of Ras Al Khaimah.
Built June–July 2026 as SP1 of RAKTA's *Virtual Safety Assessment for Autonomous Vehicles in Al Marjan* program.
Supervision: **Dr. Vima Rau** (RAKTA). Road network & vehicle data: **Mahra** (SP3). Scenario testing (downstream): **Amal** (SP2).

## License

Internship work product for RAKTA — **all rights reserved; no license granted**. Do not reuse without written permission from RAKTA. (CarMaker-side project files are IPG-licensed and are intentionally **not** in this repository.)
