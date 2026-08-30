# RoverTech Nav2 — ROS 2 Autonomous Navigation for a Skid-Steer Rover

ROS 2 Humble autonomous navigation stack for a 6-wheel skid-steer rover with real-time EtherCAT motor control (Wittenstein Simco drives) and ZED 2i stereo perception.

## Features

- **Full Nav2 stack**: global planning (Navfn), local planning (DWB), costmaps (voxel/obstacle/inflation), BT-based behaviors (spin, backup, drive-on-heading, wait), velocity smoothing
- **Real-time EtherCAT motor control**: IgH EtherCAT master via `ros2_control` plugin with CiA 402 drive profile — velocity mode, auto fault reset, auto enable
- **ZED 2i → SLAM pipeline**: point cloud → laser scan → `slam_toolbox` (Ceres solver, loop closure, 0.05m resolution)
- **Dual-mode operation**: SLAM mapping or map-based AMCL localization, selected at launch
- **Two launch systems**: simple single-purpose launches (`my_robot_bringup`) or advanced dual-mode orchestrator (`rover_nav2`) with timed lifecycle staging and auto map saving
- **ArUco absolute yaw correction**: ZED 2i marker detection → 1-D Kalman filter (`yaw_kalman.py`) → `/aruco/attitude` fused by `robot_localization` EKF as drift-correcting heading
- **Marker-based approach**: `marker_navigator` drives to a target ArUco id via Nav2 actions (search/approach/stop) with `aruco_approach.launch.py`
- **Triple-enforced safety limits**: velocity clamped to ±0.25 m/s linear, ±0.50 rad/s angular with command timeout fallback

## Hardware Requirements

- **Rover**: 6-wheel skid-steer (3 per side), 0.865m × 0.645m chassis
- **Drives**: 6× Wittenstein Simco drives (CiA 402 over EtherCAT)
- **Camera**: Stereolabs ZED 2i on mast (front-center)
- **Computer**: NVIDIA Jetson or x86, Ubuntu 22.04, ROS 2 Humble
- **Network**: EtherCAT bus (IgH master)

## Architecture

```
                        ┌──────────────────────────┐
                        │  RViz2 / Teleop / UI     │
                        └──────────┬───────────────┘
                                   │ /cmd_vel (Twist)
                                   ▼
                     ┌─────────────────────────┐
                     │  nav2_cmd_vel_relay     │
                     │  clamps ±0.25/±0.50     │
                     │  timeout → zero velocity│
                     └──────────┬──────────────┘
                                │ /diff_drive_controller/cmd_vel (TwistStamped)
                                ▼
┌──────────────┐   ┌─────────────────────┐   ┌────────────────────────┐
│  slam_toolbox│   │  diff_drive_controller│   │  Nav2 Stack           │
│  (mapping)   │   │  odom, cmd_vel       │   │  planner, controller, │
│  or          │◄──┤  wheel_sep_mult: 1.3 │◄──┤  costmaps, BT,        │
│  map_server  │   │  open_loop: false    │   │  AMCL, smoother, etc. │
│  + AMCL      │   └──────────┬──────────┘   └────────────────────────┘
└──────┬───────┘              │
       │                      │ ros2_control
       │                      ▼
       │           ┌─────────────────────┐
       │           │  ethercat_driver    │
       │           │  EthercatDriver     │
       │           │  (SystemInterface)  │
       │           └──────────┬──────────┘
       │                      │ IgH EtherCAT Master (ecrt)
       │                      ▼
       │           ┌─────────────────────┐
       └──────────►│  6× Simco Drives    │
                   │  CiA 402, vel mode  │
                   │  auto fault reset   │
                   └─────────────────────┘
```

### Control Pipeline

```
Nav2 → /cmd_vel (Twist)
  → nav2_cmd_vel_relay [safety clamp, stamp, timeout]
    → /diff_drive_controller/cmd_vel (TwistStamped)
      → diff_drive_controller [skid-steer odometry]
        → ros2_control → EthercatDriver
          → IgH Master → 6× Simco drives
```

## Package Overview

| Package | Description |
|---------|-------------|
| `ethercat_driver_ros2/` | IgH EtherCAT master wrapper, ros2_control `SystemInterface` plugin, CiA 402 drive profile, generic slave plugin, SDO service definitions |
| `my_robot_description/` | URDF/XACRO model: 6-wheel chassis, ZED camera mast, mock & real ros2_control hardware configs |
| `aruco_detection/` | ZED 2i ArUco detection (`detection_node`), attitude yaw correction (`attitude_node` + `yaw_kalman.py` 1-D KF, `yaw_correction.launch.py`) |
| `my_robot_bringup/` | Launch files, Nav2 params, SLAM/EKF configs, pointcloud→laserscan, safety relay, map saver watchdog, `marker_navigator` + `brake_release` nodes |
| `rover_nav2/` | Advanced dual-mode launch orchestrator with timed lifecycle staging, plus `aruco_approach.launch.py` (Nav2 + marker approach) |
| `rover_nav2_sim/` | Separate colcon workspace for simulation (mock hardware) |

## Quick Start

### Build

```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/local_setup.bash
```

### Real-Rover Mapping

```bash
ros2 launch my_robot_bringup nav2_mapping.launch.py
```

Teleoperate the rover slowly. Verify map exists:
```bash
ros2 topic echo --qos-durability transient_local --once /map
```

Save the map:
```bash
mkdir -p ~/nav2_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/nav2_ws/maps/rover_real_map \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

### Navigation (Saved Map)

```bash
ros2 launch my_robot_bringup nav2_navigation.launch.py \
  map:=$HOME/nav2_ws/maps/rover_real_map.yaml
```

In RViz: set `Fixed Frame` → `map`, use `2D Pose Estimate`, then send a `Nav2 Goal`.

### Advanced Launch (Dual-Mode Orchestrator)

**Mapping mode** (SLAM + Nav2 + auto map saving):
```bash
ros2 launch rover_nav2 navigation.launch.py generate_new_map:=true
```

**Localization mode** (map server + AMCL + Nav2):
```bash
ros2 launch rover_nav2 navigation.launch.py \
  generate_new_map:=false \
  map:=$HOME/nav2_ws/maps/rover_real_map.yaml
```

## Configuration Reference

| File | Purpose |
|------|---------|
| `config/nav2_params_rover.yaml` | Full Nav2 stack tuning: AMCL, DWB planners, costmaps, BT, smoother, velocity smoother, behaviors |
| `config/skid_steer_controller.yaml` | `diff_drive_controller` with wheel_separation_multiplier: 1.5, open_loop: false |
| `config/my_robot_controller.yaml` | Alternative controller config (mult: 1.3, use_stamped_vel: true) |
| `config/slam_toolbox_nav2.yaml` | Ceres solver, 0.05m resolution, loop closure enabled |
| `config/SIM2015D_slave_config_gem.yaml` | EtherCAT slave PDO mapping, SDO init sequence, velocity factors |
| `config/pointcloud_to_laserscan_nav2.yaml` | ZED point cloud → laser scan conversion (angle range ±1.20 rad) |
| `config/ekf_params.yaml` | `robot_localization` EKF: fuses wheel odom + ZED IMU + ArUco yaw (`/aruco/attitude`) → `/odometry/filtered` |
| `config/attitude_params.yaml` (`aruco_detection`) | ArUco yaw KF tuning: process/measurement noise, gating, loss timeout (see below) |
| `config/marker_navigator.yaml` | Marker approach: target id, stop distance/hysteresis, explore spin/step, timeouts |

### Key Tuning Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max linear velocity | 0.25 m/s | Enforced at relay, Nav2 params, controller |
| Max angular velocity | 0.50 rad/s | Enforced at relay, Nav2 params, controller |
| Wheel separation multiplier | 1.3 – 1.5 | Critical skid-steer tuning for odometry accuracy |
| DWB sim_time | 1.7 s | Trajectory prediction horizon |
| Costmap resolution | 0.05 m | Both local and global |
| Inflation radius | 0.50 m (local) / 0.55 m (global) | |
| SLAM resolution | 0.05 m | |
| Controller frequency | 20 Hz | DWB local planner |
| EtherCAT control frequency | 100 Hz | |

## Safety

Velocity limits are enforced at **three independent layers**:

1. **`nav2_cmd_vel_relay.py`** — clamps incoming `/cmd_vel`, publishes zero on 0.5s timeout
2. **`nav2_params_rover.yaml`** — DWB planner velocity/accel limits
3. **`skid_steer_controller.yaml`** — diff_drive_controller acceleration limits

## ArUco Yaw Correction & EKF Fusion

`aruco_attitude_node` turns ZED marker detections into an **absolute yaw** fused by the `robot_localization` EKF as a drift-correcting heading (`/aruco/attitude`, `PoseWithCovarianceStamped`). See `src/aruco_detection/Readme.production.md` for the full reference.

**Pipeline:**
```
ZED rgb/image_rect_color + camera_info
  → detection_node (/aruco/markers, /aruco/marker_ids, /aruco/detection_status)
    → attitude_node: per-marker wall-normal → base_footprint → psi=atan2(n.y,n.x)
      → relative yaw vs first-seen psi_ref → 1-D Kalman filter (yaw_kalman.py) → /aruco/attitude
        → EKF (/odometry/filtered, pose0) → Nav2 (odom_topic: /odometry/filtered)
```

**Kalman filter (`aruco_detection/yaw_kalman.py`):**
- `predict(dt, Q)` grows `P` by process noise (larger `kf_loss_process_noise` while coasting during marker loss).
- `update(yaw_obs, R)` with `R = (base_sigma + dist_coeff·distance/√count)²` from `/aruco/detection_status`; innovations gated by `gate_sigma`; `P` clamped to `[min_cov, max_cov]`.
- Always-on 20 Hz predict timer: keeps publishing a coasting estimate during blackouts with growing covariance so the EKF de-weights it; after `loss_timeout` (2.5 s) hard re-latches/resets and goes silent until markers return — survives indefinite loss.

**Key params (`config/attitude_params.yaml`):**

| Parameter | Default | Notes |
|---|---|---|
| `kf_process_noise` | 0.01 rad²/s | Uncertainty growth while tracking |
| `kf_loss_process_noise` | 0.05 rad²/s | Growth while coasting (no markers) |
| `kf_meas_base_sigma_deg` | 1.0 deg | Floor measurement noise |
| `kf_meas_dist_coeff` | 2.0 | Noise per metre of marker distance |
| `kf_gate_sigma` | 3.0 | Innovation gate (std-devs) |
| `kf_max_cov_deg` | 20.0 deg | Covariance ceiling (published cov saturates here while coasting) |
| `kf_min_cov_deg` | 0.5 deg | Covariance floor |
| `loss_timeout` | 2.5 s | Blackout → reset and silence |
| `yaw_threshold` | 10.0 deg | Inter-marker agreement reject |

**Launch:** `ros2 launch aruco_detection yaw_correction.launch.py` standalone, or via the full stack (`ros2 launch my_robot_bringup nav2_navigation.launch.py` / `ros2 launch rover_nav2 navigation.launch.py` which now include detection + attitude + EKF). The EKF is launched automatically in navigation (see `config/ekf_params.yaml`: `odom0` diff-drive odom, `imu0` ZED IMU, `pose0` ArUco yaw).

**Marker approach:** `ros2 launch rover_nav2 aruco_approach.launch.py marker_id:=42` — starts the dual-mode Nav2 stack then (after ~20 s staging) `marker_navigator_node` (via `my_robot_bringup`) which state-machines SEARCH (spin full turn + step forward) / APPROACH (refreshing `NavigateToPose` pulled back `stop_distance` from marker) / STOP / IDLE entirely through Nav2 actions, so costmaps and the velocity relay stay intact.

## EtherCAT Driver Stack

```
ethercat_interface (C++ lib, IgH ecrt wrapper)
  └── ethercat_driver (ros2_control SystemInterface plugin)
       └── ethercat_generic_plugins/
            ├── ethercat_generic_slave (generic YAML-configurable slave)
            └── ethercat_generic_cia402_drive (CiA 402 drive profile)
```

Drive configuration per-joint in `simco_ros2_control.xacro`:
- Plugin: `EcCiA402Drive`
- Mode: Profile Velocity (mode 3)
- Auto fault reset + auto enable
- RPDO: control word (0x6040) + target velocity (0x60FF, factor 3.99287e5)
- TPDO: status word (0x6041) + actual velocity (0x606C, factor 2.50446e-6)

## Troubleshooting

Check the active navigation chain:
```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 topic echo --once /cmd_vel
ros2 topic echo --once /diff_drive_controller/cmd_vel
ros2 run tf2_ros tf2_echo map odom
```

If Nav2 plans but rover does not move, check the relay:
```bash
ros2 node info /nav2_cmd_vel_relay
ros2 topic echo --once /cmd_vel
ros2 topic echo --once /diff_drive_controller/cmd_vel
```

If `/zed/scan` does not publish:
```bash
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 node info /pointcloud_to_laserscan
```

If frame `map` does not exist during navigation:
```bash
ros2 topic echo --qos-durability transient_local --once /map
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

## License

See `src/ethercat_driver_ros2/LICENSE`.
