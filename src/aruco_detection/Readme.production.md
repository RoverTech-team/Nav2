# ArUco Marker Detection — ZED 2i

Real-time ArUco marker detection pipeline for the **ZED 2i** stereo camera. Uses OpenCV's ArUco module to detect markers in the rectified RGB stream and publishes their 6-DoF poses. Designed for production deployment on RoverTech's navigation stack.

## System Overview

```
ZED 2i RGB stream ──► ArUco detection ──► /aruco/markers (PoseArray)
                          │                   /aruco/marker_ids (Int32MultiArray)
                          │                   /aruco/debug_image (Image, optional)
                          │
                     RViz 2 (live view / debugging)
```

The node subscribes to the ZED's rectified RGB image and camera info, runs marker detection across one or more ArUco dictionaries, estimates pose via `estimatePoseSingleMarkers`, and publishes results at a configurable rate.

## Hardware Requirements

- **ZED 2i** camera (USB 3.0)
- CUDA-capable GPU (recommended for ZED SDK)
- Display for RViz (non-headless)

## Dependencies

| Package | Purpose |
|---|---|
| `zed_wrapper` (stereolabs) | ZED 2i ROS 2 driver |
| `cv_bridge` | ROS ↔ OpenCV image conversion |
| `python3-opencv` (≥ 4.5) | ArUco detection |
| `rclpy` | ROS 2 Python client |
| `geometry_msgs` | PoseArray message type |
| `sensor_msgs` | Image / CameraInfo message types |
| `rviz2` | Live visualization |

OpenCV 4.5.x and 4.7+ are both supported — the node detects the API version and selects the appropriate ArUco API (`DetectorParameters_create` vs `ArucoDetector`).

## Installation

```bash
# Build the workspace
cd ~/nav2_ws_new
colcon build --packages-select aruco_detection
source install/setup.bash
```

## Running

Launch the full pipeline (ZED driver + detection + RViz):

```bash
ros2 launch aruco_detection zed_detection.launch.py
```

Launch components individually (e.g., on a headless robot, omit rviz2):

```bash
ros2 run aruco_detection detection_node --ros-args --params-file src/aruco_detection/config/zed_params.yaml
```

## Nodes

### `aruco_detection_node`

**Subscribed topics:**

| Topic | Type | Description |
|---|---|---|
| `image_topic` (param) | `sensor_msgs/Image` | Rectified RGB image from ZED |
| `camera_info_topic` (param) | `sensor_msgs/CameraInfo` | Camera intrinsics |

**Published topics:**

| Topic | Type | Description |
|---|---|---|
| `/aruco/markers` | `geometry_msgs/PoseArray` | Poses of all detected markers (frame: `marker_frame`) |
| `/aruco/marker_ids` | `std_msgs/Int32MultiArray` | IDs corresponding to each pose |
| `/aruco/debug_image` | `sensor_msgs/Image` | Annotated image with markers/axes drawn (if enabled) |

**Logging:** Each detected marker is logged at INFO with its translation vector and distance.

## Parameters (`zed_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `/zed/zed_node/rgb/image_rect_color` | Input image topic |
| `camera_info_topic` | `/zed/zed_node/rgb/camera_info` | Camera info topic |
| `marker_size` | `0.15` | Physical marker side length (meters) |
| `aruco_dictionaries` | `["DICT_6X6_250", "DICT_7X7_250"]` | Dictionaries to search (tried in order) |
| `marker_frame` | `zed_camera_link` | Output frame_id for pose messages |
| `publish_rate` | `10.0` | Max publish frequency (Hz) |
| `publish_debug_image` | `true` | Enable `/aruco/debug_image` output |

**Supported dictionaries:** `DICT_4X4_50`, `DICT_4X4_100`, `DICT_4X4_250`, `DICT_4X4_1000`, `DICT_5X5_50`, `DICT_5X5_100`, `DICT_5X5_250`, `DICT_5X5_1000`, `DICT_6X6_50`, `DICT_6X6_100`, `DICT_6X6_250`, `DICT_6X6_1000`, `DICT_7X7_50`, `DICT_7X7_100`, `DICT_7X7_250`, `DICT_7X7_1000`, `DICT_ARUCO_ORIGINAL`.

## Launch File

`launch/zed_detection.launch.py` starts three nodes:

1. **ZED wrapper** — via `IncludeLaunchDescription` with `camera_model:=zed2i`
2. **aruco_detection_node** — with parameters from `config/zed_params.yaml`
3. **rviz2** — with the config from `config/zed_detection.rviz`

The RViz config displays the camera feed, TF axes, and the `/aruco/markers` PoseArray overlay.

## Attitude Estimation (Heading / Yaw Correction)

`aruco_attitude_node` converts detected markers into an **absolute yaw estimate** that is fused by the `robot_localization` EKF (see `ekf_params.yaml`) as a drift-correcting heading measurement (`/aruco/attitude`, `PoseWithCovarianceStamped`, frame `odom`).

**Pipeline:** `/aruco/markers` + `/aruco/marker_ids` + `/aruco/detection_status` → per-marker wall-normal projected into `base_footprint` → `psi = atan2(n.y, n.x)` → relative yaw vs. the first-seen reference (`psi_ref`) → **1-D Kalman filter** → `/aruco/attitude`.

**Kalman filter (`aruco_detection/yaw_kalman.py`):**
- `predict(dt)` grows covariance `P` by process noise `Q` (coasting during marker loss).
- `update(yaw_obs, R)` fuses each frame. Measurement noise `R = (base_sigma + dist_coeff·distance/√count)²` is derived from `/aruco/detection_status` (`distance` = max marker distance, `count` = markers detected).
- Innovations are **gated** (`|innovation| > gate_sigma·√(P+R)` → rejected) and the filter **resets** if `P` exceeds `kf_max_cov_deg` (long loss → re-latch on next detection).

**Key parameters (`config/attitude_params.yaml`):**

| Parameter | Default | Description |
|---|---|---|
| `kf_process_noise` | `0.01` | Yaw uncertainty growth during tracking (rad²/s) |
| `kf_loss_process_noise` | `0.05` | Yaw uncertainty growth while coasting (no markers) (rad²/s) |
| `kf_meas_base_sigma_deg` | `1.0` | Floor measurement noise (deg) |
| `kf_meas_dist_coeff` | `2.0` | Noise growth per metre of marker distance |
| `kf_default_distance` | `2.0` | Distance used when status unavailable (m) |
| `kf_gate_sigma` | `3.0` | Innovation gate (std-devs) |
| `kf_max_cov_deg` | `20.0` | Covariance clamp ceiling (deg) — published cov saturates here while coasting |
| `kf_min_cov_deg` | `0.5` | Output covariance floor (deg) |
| `loss_timeout` | `2.5` | s — after this blackout the estimate hard re-latches/reset and goes silent |
| `yaw_threshold` | `10.0` | Inter-marker agreement reject (deg) |

**Long marker loss (survival):** `attitude_node` is always-on (20 Hz predict timer). While markers are visible it publishes a smoothed yaw; during a blackout it keeps publishing a **coasting** estimate whose covariance grows (using `kf_loss_process_noise`) and clamps at `kf_max_cov_deg`, so the downstream EKF de-weights it. After `loss_timeout` (2.5 s) it hard re-latches/resets and goes silent, letting the EKF coast on wheel/IMU odometry. The node survives arbitrarily long marker absence and cleanly re-latches on marker return (never gets stuck).

**Launch:** `ros2 launch aruco_detection yaw_correction.launch.py` (standalone), or via the full stack (`my_robot_bringup nav2_navigation.launch.py` / `rover_nav2 navigation.launch.py`, which now include detection + attitude + EKF).

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| No image in RViz | ZED TF not publishing | Ensure `publish_tf` is not set to `false` in ZED params |
| Blank/gray RViz image | QoS mismatch | RViz topic settings must use `Reliable`, `Keep Last`, `Volatile` |
| Node starts but no detections | Camera info not received | Wait for ZED to publish `camera_info` (~1 s after startup) |
| Node crashes on startup | OpenCV API mismatch | Falls back to old API automatically on 4.5.x |
| No markers detected | Wrong dictionary / marker size | Verify `aruco_dictionaries` and `marker_size` in `zed_params.yaml` |
| `/aruco/markers` empty logs | No markers in frame | Populated logs only appear when markers are detected |

## File Layout

```
src/aruco_detection/
├── Readme.production.md
├── package.xml
├── setup.py
├── aruco_detection/
│   ├── __init__.py
│   └── detection_node.py
├── config/
│   ├── zed_detection.rviz
│   └── zed_params.yaml
├── launch/
│   └── zed_detection.launch.py
└── scripts/
    ├── detection_node
    └── debug_viewer
```
