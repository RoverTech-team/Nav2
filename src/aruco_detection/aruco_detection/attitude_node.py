import os as _os
import sys as _sys
# ── numpy compat shim ───────────────────────────────────────────────
_numpy_user = _os.path.expanduser("~/.local/lib/python3.10/site-packages")
_numpy_sys = "/usr/lib/python3/dist-packages"
_numpy_roshub = "/opt/ros/humble/lib/python3/dist-packages"
_clean = [p for p in _sys.path if p != _numpy_user]
_safe = [_numpy_roshub, _numpy_sys] + _clean
_sys.path = _safe
_os.environ["PYTHONPATH"] = ":".join(_safe)
# ── end shim ─────────────────────────────────────────────────────────

import itertools
import json
import math
import numpy as np
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray, Float64MultiArray
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from aruco_detection.yaw_kalman import YawKalmanFilter


_DEFAULT_PC_RATE_MAP = {0: 10.0, 1: 10.0, 2: 7.0, 3: 5.0, 4: 3.0}
_PREDICT_HZ = 20.0


def _quat_to_normal_camera(px, py, pz, pw):
    return np.array([
        2.0 * (px * pz + pw * py),
        2.0 * (py * pz - pw * px),
        1.0 - 2.0 * (px * px + py * py),
    ])


def _rotation_matrix_from_quat(qx, qy, qz, qw):
    xx = qx * qx; xy = qx * qy; xz = qx * qz; xw = qx * qw
    yy = qy * qy; yz = qy * qz; yw = qy * qw
    zz = qz * qz; zw = qz * qw
    return np.array([
        [1 - 2*(yy + zz), 2*(xy - zw), 2*(xz + yw)],
        [2*(xy + zw), 1 - 2*(xx + zz), 2*(yz - xw)],
        [2*(xz - yw), 2*(yz + xw), 1 - 2*(xx + yy)],
    ])


def _normalize_angle_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


def _circular_mean_deg(angles):
    s = sum(math.sin(math.radians(a)) for a in angles)
    c = sum(math.cos(math.radians(a)) for a in angles)
    return math.degrees(math.atan2(s, c))


class AttitudeNode(Node):
    def __init__(self):
        super().__init__("aruco_attitude_node")

        self.declare_parameter("marker_frame", "zed_camera_link")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("yaw_threshold", 10.0)
        self.declare_parameter("loss_timeout", 2.5)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("valid_marker_ids", [0, 1, 42])
        self.declare_parameter("pointcloud_rate_map",
            '{"0": 10.0, "1": 10.0, "2": 7.0, "3": 5.0, "4": 3.0}')
        self.declare_parameter("pointcloud_update_interval", 3.0)
        # Kalman filter parameters
        self.declare_parameter("kf_process_noise", 0.01)        # rad^2 / s (tracking)
        self.declare_parameter("kf_loss_process_noise", 0.05)   # rad^2 / s (coasting)
        self.declare_parameter("kf_meas_base_sigma_deg", 1.0)   # floor measurement noise
        self.declare_parameter("kf_meas_dist_coeff", 2.0)       # deg per metre
        self.declare_parameter("kf_gate_sigma", 3.0)            # innovation gate (std-devs)
        self.declare_parameter("kf_max_cov_deg", 20.0)          # reset threshold
        self.declare_parameter("kf_min_cov_deg", 0.5)           # covariance floor
        self.declare_parameter("kf_default_distance", 2.0)      # m, used when unknown

        self._marker_frame = self.get_parameter("marker_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._yaw_threshold = self.get_parameter("yaw_threshold").value
        self._loss_timeout = rclpy.duration.Duration(
            seconds=self.get_parameter("loss_timeout").value)
        self._queue_size = self.get_parameter("queue_size").value
        self._loss_Q = self.get_parameter("kf_loss_process_noise").value
        raw_valid = self.get_parameter("valid_marker_ids").value
        self._valid_ids = set(int(v) for v in raw_valid)
        raw_rate_map = self.get_parameter("pointcloud_rate_map").value
        try:
            self._pc_rate_map = {int(k): float(v) for k, v in json.loads(raw_rate_map).items()}
        except (json.JSONDecodeError, ValueError, TypeError):
            self.get_logger().warn("Invalid pointcloud_rate_map, using defaults")
            self._pc_rate_map = dict(_DEFAULT_PC_RATE_MAP)
        self._pc_update_interval = self.get_parameter("pointcloud_update_interval").value

        # Kalman filter over the visual yaw
        self._kf = YawKalmanFilter(
            process_noise=self.get_parameter("kf_process_noise").value,
            gate_sigma=self.get_parameter("kf_gate_sigma").value,
            max_cov_deg=self.get_parameter("kf_max_cov_deg").value,
            min_cov_deg=self.get_parameter("kf_min_cov_deg").value,
        )
        self._meas_base_sigma = self.get_parameter("kf_meas_base_sigma_deg").value
        self._meas_dist_coeff = self.get_parameter("kf_meas_dist_coeff").value
        self._default_distance = self.get_parameter("kf_default_distance").value

        self._latched = False
        self._psi_ref = 0.0
        self._last_detection_time = self.get_clock().now()
        self._last_predict_time = self.get_clock().now()
        self._latest_ids = None
        self._latest_distance = 0.0
        self._latest_count = 0
        self._current_adaptive_mode = 0
        self._fallback_logged = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/aruco/attitude", self._queue_size)

        self._markers_sub = self.create_subscription(
            PoseArray, "/aruco/markers", self._markers_callback, self._queue_size)
        self._ids_sub = self.create_subscription(
            Int32MultiArray, "/aruco/marker_ids", self._ids_callback, self._queue_size)
        self._status_sub = self.create_subscription(
            Float64MultiArray, "/aruco/detection_status", self._status_callback, self._queue_size)

        self._pc_client = self.create_client(SetParameters, "/zed/zed_node/set_parameters")
        self._pc_timer = self.create_timer(self._pc_update_interval, self._pc_rate_callback)
        self._predict_timer = self.create_timer(1.0 / _PREDICT_HZ, self._predict_callback)

        valid_str = ", ".join(str(v) for v in sorted(self._valid_ids))
        self.get_logger().info(
            f"Attitude node started (Kalman-filtered) — base={self._base_frame}, "
            f"valid_ids=[{valid_str}], yaw_threshold={self._yaw_threshold}deg, "
            f"kf_process_noise={self.get_parameter('kf_process_noise').value}rad^2/s, "
            f"kf_gate_sigma={self._kf._gate_sigma}"
        )

    def _ids_callback(self, msg):
        self._latest_ids = msg.data

    def _status_callback(self, msg):
        if len(msg.data) < 6:
            return
        self._current_adaptive_mode = int(msg.data[2])
        self._latest_count = int(msg.data[4])
        self._latest_distance = float(msg.data[5])

    def _get_tf(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, self._marker_frame, rclpy.time.Time())
            return t.transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if not self._fallback_logged:
                self.get_logger().warn(
                    f"TF lookup failed ({self._marker_frame} → {self._base_frame}): {e}")
                self.get_logger().warn("Using identity transform as fallback")
                self._fallback_logged = True
            return None

    def _publish_attitude(self, now):
        msg_out = PoseWithCovarianceStamped()
        msg_out.header.stamp = now.to_msg()
        msg_out.header.frame_id = "odom"
        yaw_rad = self._kf.state
        msg_out.pose.pose.orientation.x = 0.0
        msg_out.pose.pose.orientation.y = 0.0
        msg_out.pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
        msg_out.pose.pose.orientation.w = math.cos(yaw_rad * 0.5)
        msg_out.pose.covariance[35] = self._kf.publishable_covariance()
        self._pub.publish(msg_out)

    def _predict_callback(self):
        now = self.get_clock().now()
        dt = (now - self._last_predict_time).nanoseconds * 1e-9
        self._last_predict_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / _PREDICT_HZ

        time_since = (now - self._last_detection_time).nanoseconds * 1e-9
        coasting = time_since > 0.15

        # Grow uncertainty faster while coasting (no fresh markers).
        self._kf.predict(dt, self._loss_Q if coasting else None)

        # Keep publishing a coasting estimate during a marker blackout so the
        # downstream EKF can de-weight it via the (growing) covariance.
        if self._kf.initialized and 0.15 < time_since <= self._loss_timeout.nanoseconds * 1e-9:
            self._publish_attitude(now)

        # Hard re-latch once the loss timeout elapses: drop the stale estimate
        # and wait silently for markers to return.
        if time_since > self._loss_timeout.nanoseconds * 1e-9 and self._latched:
            self._latched = False
            self._psi_ref = 0.0
            self._kf.reset()
            self.get_logger().info(
                f"Long marker loss — re-latch/reset after {time_since:.1f}s")

    def _markers_callback(self, msg):
        if self._latest_ids is None:
            return

        ids = self._latest_ids
        poses = msg.poses
        if not poses or not ids:
            return

        if len(poses) != len(ids):
            self.get_logger().warn(
                f"Mismatch: {len(poses)} poses vs {len(ids)} ids, skipping frame")
            return

        now = self.get_clock().now()

        tf = self._get_tf()
        if tf is None:
            R_tf = np.eye(3)
        else:
            R_tf = _rotation_matrix_from_quat(
                tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)

        yaws = []
        for i in range(len(poses)):
            mid = int(ids[i])
            if mid not in self._valid_ids:
                continue

            p = poses[i]
            n_camera = _quat_to_normal_camera(
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            n_base = R_tf @ n_camera
            psi = math.degrees(math.atan2(n_base[1], n_base[0]))
            yaws.append(psi)

        if not yaws:
            # No usable markers this frame; the predict timer owns the
            # coasting/reset logic, so just return here.
            return

        # We have at least one valid marker this frame
        self._last_detection_time = now

        if len(yaws) > 1:
            max_delta = max(abs(_normalize_angle_deg(a - b))
                            for a, b in itertools.combinations(yaws, 2))
            if max_delta > self._yaw_threshold:
                self.get_logger().warn(
                    f"Marker disagreement {max_delta:.1f}° > {self._yaw_threshold}° — skipping")
                return

        psi_mean = _circular_mean_deg(yaws)

        if not self._latched:
            self._psi_ref = psi_mean
            self._latched = True
            self.get_logger().info(
                f"Reference established — psi_ref={self._psi_ref:.1f}°")
            yaw_obs_deg = 0.0
        else:
            yaw_obs_deg = _normalize_angle_deg(psi_mean - self._psi_ref)

        # Measurement noise grows with marker distance, shrinks with count.
        dist = self._latest_distance if self._latest_distance > 0.0 else self._default_distance
        count = self._latest_count if self._latest_count > 0 else 1
        sigma_deg = self._meas_base_sigma + self._meas_dist_coeff * dist / math.sqrt(count)
        R = math.radians(sigma_deg) ** 2

        accepted = self._kf.update(math.radians(yaw_obs_deg), R)
        if not accepted:
            self.get_logger().warn(
                f"Kalman gate rejected innovation (sigma={sigma_deg:.1f}°)")
            return

        self._publish_attitude(now)

    def _pc_rate_callback(self):
        rate = self._pc_rate_map.get(self._current_adaptive_mode, 10.0)
        if not self._pc_client.wait_for_service(timeout_sec=0.2):
            return
        req = SetParameters.Request()
        from rcl_interfaces.msg import Parameter, ParameterType
        param = Parameter()
        param.name = "depth.point_cloud_freq"
        # Use ParameterType constant (Humble: no Parameter.Type enum)
        try:
            param.value.type = ParameterType.PARAMETER_DOUBLE
        except AttributeError:
            param.value.type = 3  # PARAMETER_DOUBLE
        param.value.double_value = rate
        req.parameters = [param]
        future = self._pc_client.call_async(req)
        future.add_done_callback(lambda f: None)

    def destroy_node(self):
        if hasattr(self, '_pc_timer'):
            self._pc_timer.cancel()
        if hasattr(self, '_predict_timer'):
            self._predict_timer.cancel()
        super().destroy_node()


def main():
    rclpy.init()
    node = AttitudeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
