import itertools
import json
import math
import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray, Float64MultiArray
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


_DEFAULT_PC_RATE_MAP = {0: 10.0, 1: 10.0, 2: 7.0, 3: 5.0, 4: 3.0}


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
        self.declare_parameter("jump_threshold", 15.0)
        self.declare_parameter("loss_timeout", 0.5)
        self.declare_parameter("temporal_window", 20)
        self.declare_parameter("min_yaw_stddev", 0.5)
        self.declare_parameter("max_yaw_stddev", 20.0)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("valid_marker_ids", [0, 1, 42])
        self.declare_parameter("pointcloud_rate_map",
            '{"0": 10.0, "1": 10.0, "2": 7.0, "3": 5.0, "4": 3.0}')
        self.declare_parameter("pointcloud_update_interval", 3.0)

        self._marker_frame = self.get_parameter("marker_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._yaw_threshold = self.get_parameter("yaw_threshold").value
        self._jump_threshold = self.get_parameter("jump_threshold").value
        self._loss_timeout = rclpy.duration.Duration(
            seconds=self.get_parameter("loss_timeout").value)
        self._temporal_window = self.get_parameter("temporal_window").value
        self._min_yaw_stddev = self.get_parameter("min_yaw_stddev").value
        self._max_yaw_stddev = self.get_parameter("max_yaw_stddev").value
        self._queue_size = self.get_parameter("queue_size").value
        raw_valid = self.get_parameter("valid_marker_ids").value
        self._valid_ids = set(int(v) for v in raw_valid)
        raw_rate_map = self.get_parameter("pointcloud_rate_map").value
        try:
            self._pc_rate_map = {int(k): float(v) for k, v in json.loads(raw_rate_map).items()}
        except (json.JSONDecodeError, ValueError, TypeError):
            self.get_logger().warn("Invalid pointcloud_rate_map, using defaults")
            self._pc_rate_map = dict(_DEFAULT_PC_RATE_MAP)
        self._pc_update_interval = self.get_parameter("pointcloud_update_interval").value

        self._latched = False
        self._psi_ref = 0.0
        self._yaw_ref = 0.0
        self._last_published_yaw = None
        self._last_detection_time = self.get_clock().now()
        self._yaw_history = []
        self._current_adaptive_mode = 0

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

        self._latest_ids = None
        self._fallback_logged = False

        self._pc_client = self.create_client(SetParameters, "/zed/zed_node/set_parameters")
        self._pc_timer = self.create_timer(self._pc_update_interval, self._pc_rate_callback)

        valid_str = ", ".join(str(v) for v in sorted(self._valid_ids))
        rates_str = ", ".join(f"mode{k}={v}Hz" for k, v in sorted(self._pc_rate_map.items()))
        self.get_logger().info(
            f"Attitude node started — base={self._base_frame}, "
            f"valid_ids=[{valid_str}], yaw_threshold={self._yaw_threshold}deg, "
            f"jump_threshold={self._jump_threshold}deg, "
            f"loss_timeout={self._loss_timeout.nanoseconds*1e-9:.1f}s, "
            f"pc_rate_map={{{rates_str}}}"
        )

    def _ids_callback(self, msg):
        self._latest_ids = msg.data

    def _status_callback(self, msg):
        self._current_adaptive_mode = int(msg.data[2])

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
            time_since_detection = (now - self._last_detection_time).nanoseconds * 1e-9
            if time_since_detection > self._loss_timeout.nanoseconds * 1e-9 and self._latched:
                self._latched = False
                self._yaw_history.clear()
                self.get_logger().info(
                    f"Latch released — no markers for {time_since_detection:.1f}s")
            return

        if len(yaws) > 1:
            max_delta = max(abs(_normalize_angle_deg(a - b))
                            for a, b in itertools.combinations(yaws, 2))
            if max_delta > self._yaw_threshold:
                self.get_logger().warn(
                    f"Marker disagreement {max_delta:.1f}° > {self._yaw_threshold}° — skipping")
                return

        if not self._latched:
            self._psi_ref = _circular_mean_deg(yaws)
            self._yaw_ref = 0.0
            self._latched = True
            self._last_detection_time = now
            self.get_logger().info(
                f"Latch established — psi_ref={self._psi_ref:.1f}°, yaw_ref=0.0°")
            return

        psi_mean = _circular_mean_deg(yaws)
        yaw_obs = self._yaw_ref + _normalize_angle_deg(psi_mean - self._psi_ref)

        if self._last_published_yaw is not None:
            jump = abs(_normalize_angle_deg(yaw_obs - self._last_published_yaw))
            if jump > self._jump_threshold:
                self.get_logger().warn(
                    f"Jump rejected {jump:.1f}° > {self._jump_threshold}°")
                return

        self._last_detection_time = now
        self._yaw_history.append(yaw_obs)
        if len(self._yaw_history) > self._temporal_window:
            self._yaw_history.pop(0)
        self._last_published_yaw = yaw_obs

        stddev = float(np.std(self._yaw_history)) if len(self._yaw_history) > 1 else self._max_yaw_stddev
        stddev = max(self._min_yaw_stddev, min(self._max_yaw_stddev, stddev))
        cov = (stddev * math.pi / 180.0) ** 2

        msg_out = PoseWithCovarianceStamped()
        msg_out.header.stamp = now.to_msg()
        msg_out.header.frame_id = "odom"
        yaw_rad = math.radians(yaw_obs)
        msg_out.pose.pose.orientation.x = 0.0
        msg_out.pose.pose.orientation.y = 0.0
        msg_out.pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
        msg_out.pose.pose.orientation.w = math.cos(yaw_rad * 0.5)
        msg_out.pose.covariance[35] = cov

        self._pub.publish(msg_out)

    def _pc_rate_callback(self):
        rate = self._pc_rate_map.get(self._current_adaptive_mode, 10.0)
        if not self._pc_client.wait_for_service(timeout_sec=0.2):
            return
        req = SetParameters.Request()
        param = Parameter()
        param.name = "depth.point_cloud_freq"
        param.value.type = Parameter.Type.DOUBLE
        param.value.double_value = rate
        req.parameters = [param]
        future = self._pc_client.call_async(req)
        future.add_done_callback(lambda f: None)

    def destroy_node(self):
        if hasattr(self, '_pc_timer'):
            self._pc_timer.cancel()
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
