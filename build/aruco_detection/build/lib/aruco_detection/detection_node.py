import json
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import math

_OPENCV_NEW_API = hasattr(cv2.aruco, "ArucoDetector")


_ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


_RESOLUTIONS = {
    "HD2K": (2208, 1242),
    "HD1080": (1920, 1080),
    "HD720": (1280, 720),
    "VGA": (640, 480),
}


def _rvec_to_quaternion(rvec):
    angle = np.linalg.norm(rvec)
    if angle < 1e-8:
        return (0.0, 0.0, 0.0, 1.0)
    axis = rvec.flatten() / angle
    s = math.sin(angle / 2.0)
    x = axis[0] * s
    y = axis[1] * s
    z = axis[2] * s
    w = math.cos(angle / 2.0)
    return (x, y, z, w)


def _rvec_to_yaw(rvec):
    angle = np.linalg.norm(rvec)
    if angle < 1e-8:
        return 0.0
    axis = rvec.flatten() / angle
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__("aruco_detection_node")

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("marker_size", 0.15)
        self.declare_parameter("aruco_dictionaries", ["DICT_6X6_250", "DICT_7X7_250"])
        self.declare_parameter("marker_frame", "zed_camera_link")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("adaptive_resolution", True)
        self.declare_parameter("resolution_chain", ["HD2K", "HD1080", "HD720", "VGA"])
        self.declare_parameter("scan_timeout", 3.0)
        self.declare_parameter("stepdown_frames", 10)
        self.declare_parameter("stepup_frames", 1)
        self.declare_parameter("stepdown_distance_tolerance", 0.3)
        self.declare_parameter("stepdown_angle_tolerance", 10.0)
        self.declare_parameter("resolution_to_rate",
            '{"2208x1242": 5.0, "1920x1080": 10.0, "1280x720": 15.0, "640x480": 20.0}')
        self.declare_parameter("failure_forget_timeout", 10.0)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        marker_size = self.get_parameter("marker_size").value
        dict_names = self.get_parameter("aruco_dictionaries").value
        if isinstance(dict_names, str):
            dict_names = [dict_names]
        self._marker_frame = self.get_parameter("marker_frame").value
        publish_rate = self.get_parameter("publish_rate").value
        publish_debug = self.get_parameter("publish_debug_image").value

        self._adaptive = self.get_parameter("adaptive_resolution").value
        chain_strs = self.get_parameter("resolution_chain").value
        self._resolution_chain = []
        for s in chain_strs:
            if s in _RESOLUTIONS:
                self._resolution_chain.append(_RESOLUTIONS[s])
        if not self._resolution_chain:
            self.get_logger().warn("No valid resolutions in chain, using HD720")
            self._resolution_chain = [_RESOLUTIONS["HD720"]]
        self._scan_timeout = self.get_parameter("scan_timeout").value
        self._stepdown_frames = self.get_parameter("stepdown_frames").value
        self._stepup_frames = self.get_parameter("stepup_frames").value
        self._current_res_idx = 0
        self._consecutive_detections = 0
        self._consecutive_misses = 0
        self._last_detection_time = self.get_clock().now()
        self._stepdown_distance_tolerance = self.get_parameter("stepdown_distance_tolerance").value
        self._stepdown_angle_tolerance = self.get_parameter("stepdown_angle_tolerance").value
        raw_rate_map = self.get_parameter("resolution_to_rate").value
        self._rate_map = {}
        for k, v in json.loads(raw_rate_map).items():
            self._rate_map[k] = float(v)
        self._failure_forget_timeout = self.get_parameter("failure_forget_timeout").value
        self._last_detection_distance = 0.0
        self._last_detection_yaw = 0.0
        self._failed_res_idx = -1
        self._failed_res_distance = 0.0
        self._failed_res_yaw = 0.0

        active = []
        for name in dict_names:
            if name not in _ARUCO_DICT:
                self.get_logger().warn(f"Unknown dictionary '{name}', skipping")
                continue
            did = _ARUCO_DICT[name]
            d = cv2.aruco.getPredefinedDictionary(did)
            if _OPENCV_NEW_API:
                p = cv2.aruco.DetectorParameters()
                active.append((name, cv2.aruco.ArucoDetector(d, p)))
            else:
                p = cv2.aruco.DetectorParameters_create()
                active.append((name, (d, p)))
            self.get_logger().info(f"  Loaded dictionary: {name}")

        if not active:
            self.get_logger().error("No valid dictionaries configured, using DICT_6X6_250")
            d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            if _OPENCV_NEW_API:
                active.append(("DICT_6X6_250", cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())))
            else:
                active.append(("DICT_6X6_250", (d, cv2.aruco.DetectorParameters_create())))

        self._detectors = active
        self._marker_size = marker_size
        self._publish_debug = publish_debug
        self._camera_matrix = None
        self._dist_coeffs = None
        self._bridge = CvBridge()

        self._pub = self.create_publisher(PoseArray, "/aruco/markers", 10)
        self._id_pub = self.create_publisher(Int32MultiArray, "/aruco/marker_ids", 10)
        self._debug_pub = self.create_publisher(Image, "/aruco/debug_image", 1)
        self._status_pub = self.create_publisher(Float64MultiArray, "/aruco/detection_status", 10)

        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, 1
        )

        if self._adaptive and self._resolution_chain:
            start_w, start_h = self._resolution_chain[0]
            start_key = f"{start_w}x{start_h}"
            initial_rate = self._rate_map.get(start_key, publish_rate)
        else:
            initial_rate = publish_rate
        self._last_pub_time = self.get_clock().now()
        self._pub_interval = rclpy.duration.Duration(seconds=1.0 / initial_rate)

        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, 1
        )

        dict_list = ", ".join(n for n, _ in active)
        adaptive_str = f"adaptive={self._adaptive}"
        if self._adaptive and self._resolution_chain:
            chain_names = self.get_parameter("resolution_chain").value
            adaptive_str += f" chain=[{' → '.join(chain_names)}]"
            adaptive_str += f" stepdown={self._stepdown_frames} stepup={self._stepup_frames}"
            adaptive_str += f" dist_tol={self._stepdown_distance_tolerance}m angle_tol={self._stepdown_angle_tolerance}deg"
            adaptive_str += f" timeout={self._scan_timeout}s"
            adaptive_str += f" forget={self._failure_forget_timeout}s"
            rates_str = ", ".join(f"{k}:{v}Hz" for k, v in self._rate_map.items())
            adaptive_str += f" rates=[{rates_str}]"
        self.get_logger().info(
            f"ArUco detection node started — image_topic={image_topic}, "
            f"dictionaries=[{dict_list}], marker_size={marker_size:.3f}m, "
            f"initial_rate={initial_rate:.0f}Hz, {adaptive_str}"
        )

    def _camera_info_callback(self, msg):
        if self._camera_matrix is not None:
            return
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.get_logger().info(
            f"Camera info received — "
            f"fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, "
            f"cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}"
        )

    def _image_callback(self, msg):
        if self._camera_matrix is None:
            return

        rate_ok = (self.get_clock().now() - self._last_pub_time) >= self._pub_interval
        if not rate_ok:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # Adaptive resolution: downscale when tracking, use full when scanning
        now = self.get_clock().now()
        cam_matrix = self._camera_matrix
        dist_coeffs = self._dist_coeffs
        adaptive_mode = 0  # 0=disabled, 1+=chain index

        if self._adaptive and self._resolution_chain:
            adaptive_mode = self._current_res_idx + 1
            h, w = frame.shape[:2]
            target_w, target_h = self._resolution_chain[self._current_res_idx]
            if (w, h) != (target_w, target_h):
                frame = cv2.resize(frame, (target_w, target_h))
                sx = target_w / w
                sy = target_h / h
                cam_matrix = self._camera_matrix.copy()
                cam_matrix[0, 0] *= sx
                cam_matrix[0, 2] *= sx
                cam_matrix[1, 1] *= sy
                cam_matrix[1, 2] *= sy

        t_start = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        all_corners = []
        all_ids = []
        all_src = []

        for dict_name, detector in self._detectors:
            if _OPENCV_NEW_API:
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                d, p = detector
                corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
            if ids is not None and len(ids) > 0:
                all_corners.extend(corners)
                all_ids.append(ids.flatten())
                all_src.extend([dict_name] * len(ids))

        msg_out = PoseArray()
        msg_out.header.stamp = now.to_msg()
        msg_out.header.frame_id = self._marker_frame

        max_distance = 0.0
        if all_ids:
            all_ids = np.concatenate(all_ids)
            for i in range(len(all_ids)):
                c = all_corners[i]
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [c], self._marker_size, cam_matrix, dist_coeffs
                )
                tvec = tvecs[0].flatten()
                rvec = rvecs[0].flatten()
                dist = float(np.linalg.norm(tvec))
                yaw = _rvec_to_yaw(rvec)
                max_distance = max(max_distance, dist)

                self.get_logger().info(
                    f"Marker {all_ids[i]} ({all_src[i]}) — "
                    f"tvec: [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}], "
                    f"distance: {dist:.2f}m, yaw: {_rvec_to_yaw(rvec):.1f}deg"
                )

                qx, qy, qz, qw = _rvec_to_quaternion(rvec)

                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw
                msg_out.poses.append(pose)

            self._last_detection_distance = max_distance
            self._last_detection_yaw = yaw
            self._last_detection_time = now

        if self._adaptive and self._resolution_chain:
            old_idx = self._current_res_idx
            if len(all_ids) > 0:
                self._consecutive_detections += 1
                self._consecutive_misses = 0
                if (self._consecutive_detections >= self._stepdown_frames
                        and self._current_res_idx < len(self._resolution_chain) - 1):
                    next_idx = self._current_res_idx + 1
                    if next_idx == self._failed_res_idx:
                        d_close = abs(self._last_detection_distance - self._failed_res_distance) < self._stepdown_distance_tolerance
                        y_close = abs(self._last_detection_yaw - self._failed_res_yaw) < self._stepdown_angle_tolerance
                        if d_close and y_close:
                            self._consecutive_detections = 0
                        else:
                            self._consecutive_detections = 0
                            self._current_res_idx += 1
                    else:
                        self._consecutive_detections = 0
                        self._current_res_idx += 1
            else:
                self._consecutive_detections = 0
                self._consecutive_misses += 1
                time_since_detection = (now - self._last_detection_time).nanoseconds * 1e-9
                if time_since_detection > self._failure_forget_timeout:
                    self._failed_res_idx = -1
                    self.get_logger().info("Failure forget timeout — retrying higher resolution")
                if self._consecutive_misses >= self._stepup_frames and self._current_res_idx > 0:
                    self._consecutive_misses = 0
                    self._failed_res_idx = self._current_res_idx
                    self._failed_res_distance = self._last_detection_distance
                    self._failed_res_yaw = self._last_detection_yaw
                    self._current_res_idx -= 1
                if time_since_detection >= self._scan_timeout and self._current_res_idx != 0:
                    self._current_res_idx = 0
                    self._consecutive_detections = 0
                    self._consecutive_misses = 0

            if old_idx != self._current_res_idx:
                old_w, old_h = self._resolution_chain[old_idx]
                new_w, new_h = self._resolution_chain[self._current_res_idx]
                key = f"{new_w}x{new_h}"
                rate = self._rate_map.get(key, 10.0)
                self._pub_interval = rclpy.duration.Duration(seconds=1.0 / rate)
                self.get_logger().info(
                    f"Adaptive step: {old_w}x{old_h} → {new_w}x{new_h} "
                    f"(mode {old_idx + 1} → {self._current_res_idx + 1}, "
                    f"rate: {rate:.0f} Hz)"
                )

        t_elapsed = (time.perf_counter() - t_start) * 1000.0

        self._pub.publish(msg_out)
        if len(all_ids) > 0:
            id_msg = Int32MultiArray()
            id_msg.data = all_ids.tolist()
            self._id_pub.publish(id_msg)
        self._last_pub_time = now

        # Publish detection status
        status = Float64MultiArray()
        status.data = [
            float(frame.shape[1]),  # width
            float(frame.shape[0]),  # height
            float(adaptive_mode),
            t_elapsed,
            float(len(all_ids)) if all_ids is not None else 0.0,
            max_distance,
        ]
        self._status_pub.publish(status)

        if self._publish_debug and all_ids:
            cv2.aruco.drawDetectedMarkers(frame, all_corners, all_ids)
            for i in range(len(all_ids)):
                rvecs_single, tvecs_single, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [all_corners[i]], self._marker_size, cam_matrix, dist_coeffs
                )
                cv2.drawFrameAxes(
                    frame, cam_matrix, dist_coeffs,
                    rvecs_single[0], tvecs_single[0], self._marker_size * 0.5
                )
            try:
                debug_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                debug_msg.header.stamp = msg_out.header.stamp
                debug_msg.header.frame_id = self._marker_frame
                self._debug_pub.publish(debug_msg)
            except CvBridgeError:
                pass

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = ArucoDetectionNode()
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
