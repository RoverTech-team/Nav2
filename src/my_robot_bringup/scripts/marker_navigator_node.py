#!/usr/bin/env python3
"""Marker navigator: drive the rover toward a target ArUco marker.

State machine (10 Hz tick):
  SEARCH   — marker not in sight: spin a full turn, then step forward,
             repeat until the marker is reacquired or explore_timeout.
  APPROACH — marker visible beyond stop_distance: send/refresh a
             NavigateToPose goal pulled back stop_distance from the marker.
  STOP     — measured distance <= stop_distance: cancel goals and hold.
  IDLE     — explore_timeout reached: hold and wait for the marker to
             reappear (re-engages APPROACH if it does).

The node never publishes cmd_vel directly; all motion goes through the
Nav2 servers (bt_navigator + behavior_server), so costmap/velocity limits
and the cmd_vel relay safety path stay intact.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from builtin_interfaces.msg import Duration as RosDuration
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Int32MultiArray, String
from nav2_msgs.action import NavigateToPose, Spin, DriveOnHeading
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

IDLE, SEARCH, APPROACH, STOP = "IDLE", "SEARCH", "APPROACH", "STOP"

_ACTION_SERVER_WAIT = 30.0


class MarkerNavigatorNode(Node):
    def __init__(self):
        super().__init__("marker_navigator_node")

        self.declare_parameter("target_marker_id", 42)
        self.declare_parameter("stop_distance", 0.5)
        self.declare_parameter("footprint_margin", 0.0)
        self.declare_parameter("stop_hysteresis", 0.6)
        self.declare_parameter("goal_refresh_period", 1.0)
        self.declare_parameter("marker_stale_time", 0.5)
        self.declare_parameter("loss_timeout", 1.0)
        self.declare_parameter("min_markers_seen", 1)
        self.declare_parameter("rotate_speed", 0.3)
        self.declare_parameter("explore_step", 0.5)
        self.declare_parameter("explore_timeout", 120.0)
        self.declare_parameter("spin_full_turn", 6.283)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("camera_frame", "zed_camera_link")
        self.declare_parameter("global_frame", "map")

        self._target_id = int(self.get_parameter("target_marker_id").value)
        self._stop_distance = float(self.get_parameter("stop_distance").value)
        self._footprint_margin = float(self.get_parameter("footprint_margin").value)
        self._stop_hysteresis = float(self.get_parameter("stop_hysteresis").value)
        self._goal_refresh_period = float(self.get_parameter("goal_refresh_period").value)
        self._marker_stale_time = float(self.get_parameter("marker_stale_time").value)
        self._loss_timeout = float(self.get_parameter("loss_timeout").value)
        self._min_markers_seen = int(self.get_parameter("min_markers_seen").value)
        self._rotate_speed = float(self.get_parameter("rotate_speed").value)
        self._explore_step = float(self.get_parameter("explore_step").value)
        self._explore_timeout = float(self.get_parameter("explore_timeout").value)
        self._spin_full_turn = float(self.get_parameter("spin_full_turn").value)
        self._base_frame = self.get_parameter("base_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._global_frame = self.get_parameter("global_frame").value

        if self._stop_hysteresis <= self._stop_distance:
            self.get_logger().warn(
                f"stop_hysteresis ({self._stop_hysteresis}) <= stop_distance "
                f"({self._stop_distance}) — forcing hysteresis above stop distance")
            self._stop_hysteresis = self._stop_distance + 0.1

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Latest detection frame: /aruco/markers and /aruco/marker_ids are
        # published back-to-back with index-paired entries (detection_node.py),
        # same pairing convention as attitude_node.py.
        self._latest_poses = None
        self._latest_ids = None
        self._poses_arrival = None
        self._ids_arrival = None

        self._markers_sub = self.create_subscription(
            PoseArray, "/aruco/markers", self._markers_callback, 10)
        self._ids_sub = self.create_subscription(
            Int32MultiArray, "/aruco/marker_ids", self._ids_callback, 10)
        self._status_pub = self.create_publisher(String, "/marker_nav/status", 10)

        # key → {"client", "goal", "handle", "result", "status", "done"}
        self._actions = {
            "nav": self._make_action("nav", ActionClient(self, NavigateToPose, "/navigate_to_pose")),
            "spin": self._make_action("spin", ActionClient(self, Spin, "/spin")),
            "step": self._make_action("step", ActionClient(self, DriveOnHeading, "/drive_on_heading")),
        }

        self._state = SEARCH
        self._last_marker_time = None       # last fresh target observation
        self._last_goal_time = None         # last NavigateToPose (re)send
        self._explore_start = None          # wall time SEARCH was entered
        self._search_stage = "spin"
        self._stage_done_time = None
        self._last_status_pub = None
        self._tf_warn_time = None
        self._consecutive_seen = 0

        self._publish_status("startup")
        self._tick_timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"Marker navigator started — target_id={self._target_id}, "
            f"stop_distance={self._stop_distance} m (+ margin {self._footprint_margin} m), "
            f"explore: spin {self._spin_full_turn:.2f} rad / step {self._explore_step} m, "
            f"timeout {self._explore_timeout} s")

    # ── detection input ────────────────────────────────────────────────

    def _markers_callback(self, msg):
        self._latest_poses = msg.poses
        self._poses_arrival = self.get_clock().now()

    def _ids_callback(self, msg):
        self._latest_ids = msg.data
        self._ids_arrival = self.get_clock().now()

    def _target_observation(self):
        """Return (distance_m, marker_pos_camera) for the target marker, or None.

        Distance is the horizontal range in base_frame (accounts for the
        camera being offset/elevated); falls back to the camera-frame 3D
        norm — which overestimates range off-axis, i.e. stops early, never
        late — when TF is unavailable.
        """
        now = self.get_clock().now()
        if (self._latest_poses is None or self._latest_ids is None
                or not self._latest_poses or not self._latest_ids):
            return None
        if len(self._latest_poses) != len(self._latest_ids):
            return None
        if self._poses_arrival is None or self._ids_arrival is None:
            return None
        if abs((self._poses_arrival - self._ids_arrival).nanoseconds) > 0.3e9:
            return None
        if (now - self._poses_arrival).nanoseconds * 1e-9 > self._marker_stale_time:
            return None

        found = None
        for i, mid in enumerate(self._latest_ids):
            if int(mid) != self._target_id:
                continue
            p = self._latest_poses[i].position
            p_cam = (p.x, p.y, p.z)
            t = self._lookup(self._base_frame, self._camera_frame)
            if t is not None:
                b = self._apply_transform(t, p_cam)
                if b is not None:
                    found = (math.hypot(b[0], b[1]), p_cam)
                    break
            found = (math.sqrt(p_cam[0] ** 2 + p_cam[1] ** 2 + p_cam[2] ** 2), p_cam)
            break
        if found is None:
            self._consecutive_seen = 0
            return None
        # Require min_markers_seen consecutive frames to reject false positives
        self._consecutive_seen += 1
        if self._consecutive_seen < max(1, self._min_markers_seen):
            return None
        return found

    def _apply_transform(self, t, point):
        tr = t.transform.translation
        r = t.transform.rotation
        m = self._rot_matrix(r.x, r.y, r.z, r.w)
        return (
            m[0][0] * point[0] + m[0][1] * point[1] + m[0][2] * point[2] + tr.x,
            m[1][0] * point[0] + m[1][1] * point[1] + m[1][2] * point[2] + tr.y,
            m[2][0] * point[0] + m[2][1] * point[1] + m[2][2] * point[2] + tr.z,
        )

    @staticmethod
    def _rot_matrix(qx, qy, qz, qw):
        return [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]

    def _lookup(self, target, source):
        try:
            return self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _lookup_point(self, target, source, point):
        """Transform a source-frame point into target frame; None on failure."""
        t = self._lookup(target, source)
        if t is None:
            return None
        return self._apply_transform(t, point)

    # ── action management ──────────────────────────────────────────────

    def _make_action(self, key, client):
        return {"client": client, "handle": None, "result": None,
                "status": None, "done": True, "key": key}

    def _send_action(self, key, goal):
        a = self._actions[key]
        if not a["done"]:
            return False  # still active (e.g. cancellation in flight) — retry next tick
        if not a["client"].wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(f"Action server for '{key}' not available yet", throttle_duration_sec=5.0)
            return False
        a["done"] = False
        a["status"] = None
        fut = a["client"].send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_goal_response(f, key))
        return True

    def _on_goal_response(self, future, key):
        a = self._actions[key]
        try:
            handle = future.result()
        except Exception as e:  # noqa: BLE001 — action client transport errors
            self.get_logger().error(f"'{key}' goal request failed: {e}")
            a["done"] = True
            return
        if not handle.accepted:
            self.get_logger().warn(f"'{key}' goal rejected")
            a["done"] = True
            return
        a["handle"] = handle
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(lambda f: self._on_result(f, key))

    def _on_result(self, future, key):
        a = self._actions[key]
        a["done"] = True
        a["handle"] = None
        try:
            a["result"] = future.result()
            a["status"] = a["result"].status
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"'{key}' result retrieval failed: {e}")
            a["status"] = None

    def _action_busy(self, key):
        a = self._actions[key]
        return not a["done"]

    def _cancel_active(self):
        """Cancel every in-flight action (goal handles only — the request
        futures resolve on their own)."""
        for key, a in self._actions.items():
            if a["handle"] is not None:
                try:
                    a["handle"].cancel_goal_async()
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn(f"Failed to cancel '{key}': {e}")

    # ── state machine ──────────────────────────────────────────────────

    def _set_state(self, state):
        if state == self._state:
            return
        self.get_logger().info(f"State: {self._state} → {state}")
        self._state = state
        self._publish_status(state)

    def _publish_status(self, state):
        msg = String()
        msg.data = state
        self._status_pub.publish(msg)

    def _effective_stop(self):
        return self._stop_distance + self._footprint_margin

    def _tick(self):
        now = self.get_clock().now()
        obs = self._target_observation()

        if obs is not None:
            self._last_marker_time = now

        if self._state == SEARCH:
            self._tick_search(now, obs)
        elif self._state == APPROACH:
            self._tick_approach(now, obs)
        elif self._state == STOP:
            self._tick_stop(obs)
        elif self._state == IDLE:
            self._tick_idle(obs)

    # SEARCH ─ rotate a full turn, then step forward, repeat ────────────
    def _tick_search(self, now, obs):
        if self._explore_start is None:
            self._explore_start = now
            self._search_stage = "spin"
            self._stage_done_time = None
        if (now - self._explore_start).nanoseconds * 1e-9 > self._explore_timeout:
            self.get_logger().error(
                f"Explore timeout ({self._explore_timeout:.0f} s) — target marker "
                f"{self._target_id} not found, going IDLE")
            self._cancel_active()
            self._set_state(IDLE)
            return

        # Reacquired → drop the explore pattern and approach.
        if obs is not None:
            self._cancel_active()
            self._explore_start = None
            self._set_state(APPROACH)
            return

        # Wait for the in-flight explore action before advancing stages.
        key = "spin" if self._search_stage == "spin" else "step"
        if self._action_busy(key):
            return
        a = self._actions[key]
        if a["result"] is not None:
            if self._stage_done_time is None:
                self._stage_done_time = now
            if (now - self._stage_done_time).nanoseconds * 1e-9 < 0.5:
                return  # settle window: let fresh detections flow in
            self._actions[key]["result"] = None
            self._stage_done_time = None
            self._search_stage = "step" if key == "spin" else "spin"

        if self._search_stage == "spin":
            goal = Spin.Goal()
            goal.target_yaw = self._spin_full_turn
            secs = self._spin_full_turn / max(self._rotate_speed, 0.05) + 10.0
            goal.time_allowance = RosDuration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
            self.get_logger().info("SEARCH: spinning a full turn")
            self._send_action("spin", goal)
        else:
            goal = DriveOnHeading.Goal()
            goal.target_distance = self._explore_step
            goal.speed = self._explore_step / 4.0 if self._explore_step > 0 else 0.1
            goal.time_allowance = RosDuration(sec=15, nanosec=0)
            self.get_logger().info(f"SEARCH: stepping forward {self._explore_step} m")
            self._send_action("step", goal)

    # APPROACH ─ send/refresh a marker-relative NavigateToPose goal ─────
    def _tick_approach(self, now, obs):
        effective_stop = self._effective_stop()

        # Reached: cancel everything and hold.
        if obs is not None and obs[0] <= effective_stop:
            self._cancel_active()
            self._set_state(STOP)
            return

        # Lost: after loss_timeout give up the approach and search.
        if (self._last_marker_time is None
                or (now - self._last_marker_time).nanoseconds * 1e-9 > self._loss_timeout):
            self.get_logger().warn("Marker lost during approach — switching to SEARCH")
            self._cancel_active()
            self._explore_start = None
            self._set_state(SEARCH)
            return

        # Live safety distance is handled by obs above; navigation goal
        # refresh happens on the (slower) refresh cadence.
        if self._last_goal_time is not None:
            if (now - self._last_goal_time).nanoseconds * 1e-9 < self._goal_refresh_period * 1e9:
                return
        if self._action_busy("nav"):
            return

        goal_pose = self._compute_goal_pose(obs[1])
        if goal_pose is None:
            # Cannot express the marker in map frame (AMCL/SLAM TF down):
            # hold position rather than navigate blind.
            self._warn_tf_throttled(now)
            return

        self._cancel_active()
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        if self._send_action("nav", goal):
            self._last_goal_time = now

    def _compute_goal_pose(self, p_cam):
        marker_map = self._lookup_point(self._global_frame, self._camera_frame, p_cam)
        base_map = self._lookup_point(self._global_frame, self._base_frame, (0.0, 0.0, 0.0))
        if marker_map is None or base_map is None:
            return None

        dx = marker_map[0] - base_map[0]
        dy = marker_map[1] - base_map[1]
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return None
        pullback = min(self._effective_stop(), max(d - 0.05, 0.0))
        gx = marker_map[0] - dx / d * pullback
        gy = marker_map[1] - dy / d * pullback
        yaw = math.atan2(dy, dx)

        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _warn_tf_throttled(self, now):
        if self._tf_warn_time is not None and \
                (now - self._tf_warn_time).nanoseconds * 1e-9 < 5.0:
            return
        self._tf_warn_time = now
        self.get_logger().warn(
            f"TF {self._global_frame}→{self._camera_frame}/{self._base_frame} "
            "unavailable — holding position (localization not ready?)")

    # STOP ─ hold until the marker moves away beyond hysteresis ─────────
    def _tick_stop(self, obs):
        if obs is not None and obs[0] > self._stop_hysteresis:
            self.get_logger().info(
                f"Distance {obs[0]:.2f} m > hysteresis {self._stop_hysteresis} m — resuming APPROACH")
            self._set_state(APPROACH)

    # IDLE ─ explore gave up; re-engage if the marker shows up ──────────
    def _tick_idle(self, obs):
        if obs is not None:
            self.get_logger().info("Marker back in sight from IDLE — resuming APPROACH")
            self._set_state(APPROACH)

    def destroy_node(self):
        self._cancel_active()
        for a in self._actions.values():
            a["client"].destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = MarkerNavigatorNode()
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
