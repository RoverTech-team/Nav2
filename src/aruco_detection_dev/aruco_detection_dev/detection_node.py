import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32MultiArray
import math


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


class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__("aruco_detection_node")

        self.declare_parameter("camera_id", 0)
        self.declare_parameter("marker_size", 0.15)
        self.declare_parameter("aruco_dictionaries", ["DICT_6X6_250", "DICT_7X7_250"])
        self.declare_parameter("marker_frame", "camera_link")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("show_preview", True)

        camera_id = self.get_parameter("camera_id").value
        marker_size = self.get_parameter("marker_size").value
        dict_names = self.get_parameter("aruco_dictionaries").value
        if isinstance(dict_names, str):
            dict_names = [dict_names]
        self._marker_frame = self.get_parameter("marker_frame").value
        publish_rate = self.get_parameter("publish_rate").value
        show_preview = self.get_parameter("show_preview").value

        active = []
        for name in dict_names:
            if name not in _ARUCO_DICT:
                self.get_logger().warn(f"Unknown dictionary '{name}', skipping")
                continue
            did = _ARUCO_DICT[name]
            d = cv2.aruco.getPredefinedDictionary(did)
            p = cv2.aruco.DetectorParameters()
            active.append((name, cv2.aruco.ArucoDetector(d, p)))
            self.get_logger().info(f"  Loaded dictionary: {name}")

        if not active:
            self.get_logger().error("No valid dictionaries configured, using DICT_6X6_250")
            d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            active.append(("DICT_6X6_250", cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())))

        self._detectors = active
        self._marker_size = marker_size
        self._show_preview = show_preview
        self._camera_matrix = None
        self._dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        self._pub = self.create_publisher(PoseArray, "/aruco_dev/markers", 10)
        self._id_pub = self.create_publisher(Int32MultiArray, "/aruco_dev/marker_ids", 10)

        interval = 1.0 / max(publish_rate, 1.0)
        self._timer = self.create_timer(interval, self._tick)
        self._cap = None
        self._open_camera(camera_id)

        dict_list = ", ".join(n for n, _ in active)
        self.get_logger().info(
            f"ArUco detection node started — camera={camera_id}, "
            f"dictionaries=[{dict_list}], marker_size={marker_size:.3f}m, "
            f"publish_rate={publish_rate:.1f}Hz"
        )

    def _open_camera(self, camera_id):
        backends = [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]
        for backend in backends:
            cap = cv2.VideoCapture(camera_id, backend)
            if cap.isOpened():
                self._cap = cap
                ret, frame = cap.read()
                if ret:
                    h, w = frame.shape[:2]
                    self._camera_matrix = np.array([
                        [w, 0, w / 2.0],
                        [0, w, h / 2.0],
                        [0, 0, 1],
                    ], dtype=np.float64)
                    self.get_logger().info(
                        f"Camera opened (backend={backend}), frame={w}x{h}, "
                        f"estimated camera_matrix focal={w}"
                    )
                    return
                cap.release()
        self.get_logger().error("Failed to open any camera. Check camera_id and permissions.")

    def _tick(self):
        if self._cap is None or not self._cap.isOpened():
            return

        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn("Camera read failed")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        all_corners = []
        all_ids = []
        all_src = []

        for dict_name, detector in self._detectors:
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                all_corners.extend(corners)
                all_ids.append(ids.flatten())
                all_src.extend([dict_name] * len(ids))

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._marker_frame

        if all_ids:
            all_ids = np.concatenate(all_ids)
            for i in range(len(all_ids)):
                c = all_corners[i]
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [c], self._marker_size, self._camera_matrix, self._dist_coeffs
                )
                tvec = tvecs[0].flatten()
                rvec = rvecs[0].flatten()
                dist = float(np.linalg.norm(tvec))

                self.get_logger().info(
                    f"Marker {all_ids[i]} ({all_src[i]}) — "
                    f"tvec: [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}], "
                    f"distance: {dist:.2f}m"
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
                msg.poses.append(pose)

            if self._show_preview:
                cv2.aruco.drawDetectedMarkers(frame, all_corners, all_ids)
                for i in range(len(all_ids)):
                    rvecs_single, tvecs_single, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [all_corners[i]], self._marker_size, self._camera_matrix, self._dist_coeffs
                    )
                    cv2.drawFrameAxes(
                        frame, self._camera_matrix, self._dist_coeffs,
                        rvecs_single[0], tvecs_single[0], self._marker_size * 0.5
                    )
                cv2.putText(
                    frame, f"Markers: {len(all_ids)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
        else:
            if self._show_preview:
                cv2.putText(
                    frame, "No markers", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
                )

        self._pub.publish(msg)
        if all_ids.size > 0:
            id_msg = Int32MultiArray()
            id_msg.data = all_ids.tolist()
            self._id_pub.publish(id_msg)

        if self._show_preview:
            cv2.imshow("ArUco Detection (DEV)", frame)
            cv2.pollKey()

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        if self._show_preview:
            cv2.destroyAllWindows()
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
