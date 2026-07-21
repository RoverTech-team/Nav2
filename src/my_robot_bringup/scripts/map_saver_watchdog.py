#!/usr/bin/env python3
"""Periodically save the live SLAM map while mapping is running."""

import os
import subprocess

import rclpy
from rclpy.node import Node


class MapSaverWatchdog(Node):

    def __init__(self):
        super().__init__("map_saver_watchdog")

        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("initial_delay", 30.0)
        self.declare_parameter("save_period", 30.0)
        self.declare_parameter("save_timeout", 30.0)

        self._map_yaml = self.get_parameter("map_yaml_path").value
        self._initial_delay = float(self.get_parameter("initial_delay").value)
        self._save_period = float(self.get_parameter("save_period").value)
        self._save_timeout = float(self.get_parameter("save_timeout").value)
        self._save_count = 0

        if not self._map_yaml:
            self.get_logger().error("map_yaml_path parameter is empty; map saver will exit.")
            rclpy.shutdown()
            return

        self._map_yaml = os.path.abspath(os.path.expanduser(self._map_yaml))
        self._map_dir = os.path.dirname(self._map_yaml)
        self._map_stem = os.path.splitext(os.path.basename(self._map_yaml))[0]
        os.makedirs(self._map_dir, exist_ok=True)

        self._save_period = max(1.0, self._save_period)
        self._save_timeout = max(1.0, self._save_timeout)
        first_delay = max(0.1, self._initial_delay)

        self.get_logger().info(
            "Periodic map saver started: "
            f"{os.path.join(self._map_dir, self._map_stem)} "
            f"(first save in {first_delay:.1f}s, then every {self._save_period:.1f}s)."
        )

        self._initial_timer = self.create_timer(first_delay, self._on_initial_timer)
        self._periodic_timer = None

    def _on_initial_timer(self):
        self._initial_timer.cancel()
        self._save_map()
        self._periodic_timer = self.create_timer(self._save_period, self._save_map)

    def _save_map(self):
        self._save_count += 1
        map_prefix = os.path.join(self._map_dir, self._map_stem)
        self.get_logger().info(f"Saving map #{self._save_count}: {map_prefix}")

        try:
            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", self._map_stem],
                cwd=self._map_dir,
                capture_output=True,
                text=True,
                timeout=self._save_timeout,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                f"map_saver_cli timed out after {self._save_timeout:.1f}s."
            )
            return
        except FileNotFoundError:
            self.get_logger().error("ros2 binary not found. Is your ROS 2 workspace sourced?")
            return

        if result.returncode == 0:
            self.get_logger().info("Map save completed.")
            return

        stderr = result.stderr.strip()
        stdout = result.stdout.strip()
        details = stderr or stdout or "no output"
        self.get_logger().warn(
            f"map_saver_cli failed with rc={result.returncode}; will retry. {details}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverWatchdog()
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
