import json
import math
import os
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from std_msgs.msg import Int32MultiArray, Float64MultiArray, Float64
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


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


class YawTestNode(Node):
    def __init__(self):
        super().__init__("aruco_yaw_test_node")

        self.declare_parameter("marker_azimuths", '{"0": 90.0, "1": 180.0, "42": 270.0}')
        self.declare_parameter("marker_frame", "zed_camera_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("test_duration", 120.0)
        self.declare_parameter("output_dir", "~/aruco_test_results")
        self.declare_parameter("yaw_threshold", 10.0)
        self.declare_parameter("jump_threshold", 15.0)
        self.declare_parameter("use_identity_tf_fallback", True)

        raw_azimuths = self.get_parameter("marker_azimuths").value
        self._azimuths = {int(k): float(v) for k, v in json.loads(raw_azimuths).items()}
        self._marker_frame = self.get_parameter("marker_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._use_fallback = self.get_parameter("use_identity_tf_fallback").value
        test_duration = self.get_parameter("test_duration").value
        output_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        self._yaw_threshold = self.get_parameter("yaw_threshold").value
        self._jump_threshold = self.get_parameter("jump_threshold").value

        os.makedirs(output_dir, exist_ok=True)
        timestamp = self.get_clock().now().to_msg().sec
        from datetime import datetime
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self._csv_path = os.path.join(output_dir, f"yaw_test_{ts}.csv")
        self._summary_path = os.path.join(output_dir, f"summary_{ts}.json")
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "timestamp_sec", "frame_stamp_sec",
            "marker_id", "psi_deg", "wall_azimuth_deg", "yaw_deg",
            "distance_m",
            "n_markers_visible",
            "img_width", "img_height", "adaptive_mode",
            "processing_time_ms",
        ])
        self._csv_file.flush()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(Float64, "/aruco/yaw_test", 10)

        self._markers_sub = self.create_subscription(
            PoseArray, "/aruco/markers", self._markers_callback, 10
        )
        self._ids_sub = self.create_subscription(
            Int32MultiArray, "/aruco/marker_ids", self._ids_callback, 10
        )
        self._status_sub = self.create_subscription(
            Float64MultiArray, "/aruco/detection_status", self._status_callback, 10
        )

        self._latest_ids = None
        self._frame_count = 0
        self._frame_with_markers = 0
        self._markers_seen = set()
        self._all_yaws = []
        self._all_distances = []
        self._prev_yaw = None
        self._jumps_rejected = 0

        self._latest_status = None
        self._prev_w = -1
        self._prev_h = -1
        self._resolution_changes = []
        self._status_count = 0
        self._processing_times = []
        self._status_by_res = {}
        self._distances_by_res = {}

        self._shutdown_timer = self.create_timer(test_duration, self._on_timeout)
        self._test_duration = test_duration

        dict_str = ", ".join(f"{k}: {v}°" for k, v in self._azimuths.items())
        self.get_logger().info(
            f"Yaw test node started — duration={test_duration:.0f}s, "
            f"marker_azimuths={{{dict_str}}}, "
            f"output={self._csv_path}"
        )

    def _ids_callback(self, msg):
        self._latest_ids = msg.data

    def _status_callback(self, msg):
        if len(msg.data) < 6:
            return
        self._latest_status = msg.data
        self._status_count += 1
        w = int(msg.data[0])
        h = int(msg.data[1])
        mode = int(msg.data[2])
        proc_ms = msg.data[3]
        n_detected = int(msg.data[4])
        max_dist = msg.data[5]

        if proc_ms > 0:
            self._processing_times.append(proc_ms)

        res_key = f"{w}x{h}"
        if res_key not in self._status_by_res:
            self._status_by_res[res_key] = {"frames": 0, "frames_with_markers": 0}
        self._status_by_res[res_key]["frames"] += 1
        if n_detected > 0:
            self._status_by_res[res_key]["frames_with_markers"] += 1

        if w != self._prev_w or h != self._prev_h:
            now_t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
            self._resolution_changes.append({
                "time_sec": now_t,
                "from_w": self._prev_w,
                "from_h": self._prev_h,
                "to_w": w,
                "to_h": h,
                "mode": mode,
            })
            self._prev_w = w
            self._prev_h = h

    def _get_tf(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, self._marker_frame, rclpy.time.Time()
            )
            return t.transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if self._use_fallback:
                if not hasattr(self, "_fallback_logged"):
                    self.get_logger().warn(
                        f"TF lookup failed ({self._marker_frame} → {self._base_frame}): {e}"
                    )
                    self.get_logger().warn(
                        "Using identity transform as fallback — yaw is in camera frame"
                    )
                    self._fallback_logged = True
                t = TransformStamped()
                t.transform.rotation.w = 1.0
                return t.transform
            self.get_logger().warn(f"TF lookup failed: {e}")
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
                f"Mismatch: {len(poses)} poses vs {len(ids)} ids, skipping frame"
            )
            return

        tf = self._get_tf()
        if tf is None:
            return

        R_tf = _rotation_matrix_from_quat(
            tf.rotation.x, tf.rotation.y,
            tf.rotation.z, tf.rotation.w
        )

        self._frame_count += 1
        frame_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        yaws = []
        dists = []
        for i in range(len(poses)):
            mid = int(ids[i])
            if mid not in self._azimuths:
                continue

            p = poses[i]
            tvec = np.array([p.position.x, p.position.y, p.position.z])
            dist = float(np.linalg.norm(tvec))
            dists.append(dist)

            n_camera = _quat_to_normal_camera(
                p.orientation.x, p.orientation.y,
                p.orientation.z, p.orientation.w
            )
            n_base = R_tf @ n_camera
            psi = math.degrees(math.atan2(n_base[1], n_base[0]))
            azimuth = self._azimuths[mid]
            yaw = _normalize_angle_deg(azimuth - psi)

            img_w = int(self._latest_status[0]) if self._latest_status is not None else -1
            img_h = int(self._latest_status[1]) if self._latest_status is not None else -1
            mode  = int(self._latest_status[2]) if self._latest_status is not None else -1
            proc  = self._latest_status[3]      if self._latest_status is not None else -1.0

            self._csv_writer.writerow([
                f"{now_t:.3f}", f"{frame_t:.3f}",
                mid, f"{psi:.2f}", azimuth, f"{yaw:.2f}",
                f"{dist:.3f}",
                len(poses),
                img_w, img_h, mode,
                f"{proc:.1f}",
            ])
            if self._latest_status is not None:
                res_key = f"{img_w}x{img_h}"
                if res_key not in self._distances_by_res:
                    self._distances_by_res[res_key] = []
                self._distances_by_res[res_key].append(dist)
            yaws.append(yaw)
            self._markers_seen.add(mid)

        if yaws:
            self._frame_with_markers += 1
            mean_yaw = _circular_mean_deg(yaws)

            max_delta = 0.0
            if len(yaws) > 1:
                max_delta = max(abs(_normalize_angle_deg(a - b))
                                for a in yaws for b in yaws if yaws.index(a) < yaws.index(b))
            if max_delta > self._yaw_threshold and len(yaws) > 1:
                yaw_str = ", ".join(f"ID{ids[j]}={yaws[j]:.1f}" for j in range(len(poses))
                                    if int(ids[j]) in self._azimuths)
                self.get_logger().warn(
                    f"Marker disagreement {max_delta:.1f}° > {self._yaw_threshold}° — {yaw_str}"
                )
                return

            jump = abs(_normalize_angle_deg(mean_yaw - self._prev_yaw)) if self._prev_yaw is not None else 0.0
            if jump > self._jump_threshold:
                self._jumps_rejected += 1
                yaw_str = ", ".join(f"ID{ids[j]}={yaws[j]:.1f}" for j in range(len(poses))
                                    if int(ids[j]) in self._azimuths)
                self.get_logger().warn(
                    f"Jump rejected {jump:.1f}° > {self._jump_threshold}° — {yaw_str}"
                )
                return

            self._all_yaws.append(mean_yaw)
            self._all_distances.append(sum(dists) / len(dists))
            self._prev_yaw = mean_yaw

            yaw_str = ", ".join(f"ID{ids[j]}={yaws[j]:.1f}" for j in range(len(poses))
                                if int(ids[j]) in self._azimuths)
            self.get_logger().info(
                f"Yaw={mean_yaw:.1f}° ({len(yaws)} markers) — {yaw_str}"
            )

            msg_out = Float64()
            msg_out.data = math.radians(mean_yaw)
            self._pub.publish(msg_out)

        self._csv_file.flush()

    def _on_timeout(self):
        self._shutdown_timer.cancel()
        self._csv_file.close()

        summary = {
            "test_duration_sec": self._test_duration,
            "total_frames": self._frame_count,
            "frames_with_markers": self._frame_with_markers,
            "jumps_rejected": self._jumps_rejected,
            "markers_seen": sorted(self._markers_seen),
            "n_markers_visible_avg": (
                self._frame_with_markers / max(self._frame_count, 1)
                if self._frame_count > 0 else 0.0
            ),
        }

        # Processing time stats
        if self._processing_times:
            pt_arr = np.array(self._processing_times)
            summary["processing_time_stats"] = {
                "mean_ms": float(np.mean(pt_arr)),
                "stddev_ms": float(np.std(pt_arr)),
                "min_ms": float(np.min(pt_arr)),
                "max_ms": float(np.max(pt_arr)),
                "median_ms": float(np.median(pt_arr)),
            }
        else:
            summary["processing_time_stats"] = None

        # Resolution tracking
        if self._resolution_changes:
            summary["resolution_changes"] = self._resolution_changes

        if self._status_by_res:
            time_by_res = {}
            total_frames_all = sum(v["frames"] for v in self._status_by_res.values())
            for res_key, data in self._status_by_res.items():
                detection_rate = (
                    data["frames_with_markers"] / max(data["frames"], 1)
                )
                entry = {
                    "frames": data["frames"],
                    "frames_with_markers": data["frames_with_markers"],
                    "detection_rate": round(detection_rate, 4),
                    "time_pct": round(data["frames"] / max(total_frames_all, 1) * 100.0, 1),
                }
                if res_key in self._distances_by_res and self._distances_by_res[res_key]:
                    d_arr = np.array(self._distances_by_res[res_key])
                    entry["distance_stats"] = {
                        "mean_m": float(np.mean(d_arr)),
                        "stddev_m": float(np.std(d_arr)),
                        "min_m": float(np.min(d_arr)),
                        "max_m": float(np.max(d_arr)),
                        "count": len(d_arr),
                    }
                time_by_res[res_key] = entry
            summary["detection_by_resolution"] = time_by_res

        # Yaw stats
        if self._all_yaws:
            yaw_arr = np.array(self._all_yaws)
            summary["yaw_stats"] = {
                "mean": float(np.mean(yaw_arr)),
                "stddev": float(np.std(yaw_arr)),
                "min": float(np.min(yaw_arr)),
                "max": float(np.max(yaw_arr)),
                "median": float(np.median(yaw_arr)),
            }
        else:
            summary["yaw_stats"] = None
            self.get_logger().error("No yaw estimates recorded — check marker visibility")

        # Distance stats
        if self._all_distances:
            dist_arr = np.array(self._all_distances)
            summary["distance_stats"] = {
                "mean": float(np.mean(dist_arr)),
                "stddev": float(np.std(dist_arr)),
                "min": float(np.min(dist_arr)),
                "max": float(np.max(dist_arr)),
            }

        with open(self._summary_path, "w") as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info(
            f"Test complete — results saved to {self._summary_path}"
        )
        self.get_logger().info(
            f"Summary: {self._frame_with_markers}/{self._frame_count} frames with markers, "
            f"markers seen: {sorted(self._markers_seen)}"
        )
        if self._processing_times:
            pt_arr = np.array(self._processing_times)
            self.get_logger().info(
                f"Processing time: mean={np.mean(pt_arr):.1f}ms, "
                f"max={np.max(pt_arr):.1f}ms"
            )
        if summary["yaw_stats"]:
            self.get_logger().info(
                f"Yaw stats: mean={summary['yaw_stats']['mean']:.1f}°, "
                f"stddev={summary['yaw_stats']['stddev']:.1f}°"
            )

        rclpy.shutdown()

    def destroy_node(self):
        if hasattr(self, "_csv_file") and not self._csv_file.closed:
            self._csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = YawTestNode()
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
