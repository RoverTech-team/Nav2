#!/usr/bin/env python3
"""Brake release helper for SIM2015D CIA402 drives.
Claims reset_fault command_interfaces and pulses them to clear Fault/STO.
Holds reset_fault=1.0 for pulse_duration (1.0s) until OP_ENABLED, then 0.0.
Watchdog repeats every repeat_interval while auto_repeat=True.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.msg import ControllerState
from std_msgs.msg import String as _Unused  # keep import order stable

class BrakeReleaseNode(Node):
    def __init__(self):
        super().__init__('brake_release_node')
        self.declare_parameter('joints', [
            'base_front_left_wheel_joint',
            'base_front_right_wheel_joint',
            'base_middle_left_wheel_joint',
            'base_middle_right_wheel_joint',
            'base_back_left_wheel_joint',
            'base_back_right_wheel_joint',
        ])
        self.declare_parameter('auto_repeat', True)
        self.declare_parameter('pulse_duration', 1.0)
        self.declare_parameter('repeat_interval', 5.0)
        self.declare_parameter('activate_controller', True)
        self.declare_parameter('controller_name', 'brake_release_controller')

        self.joints = self.get_parameter('joints').value
        self.auto_repeat = self.get_parameter('auto_repeat').value
        self.pulse_duration = float(self.get_parameter('pulse_duration').value)
        self.repeat_interval = float(self.get_parameter('repeat_interval').value)
        self.activate_controller = bool(self.get_parameter('activate_controller').value)
        self.controller_name = str(self.get_parameter('controller_name').value)

        self.cli = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self._init_timer = None
        self._repeat_timer = None
        self._clear_timer = None
        self._activated = False
        self.get_logger().info(
            f'Brake release node ready for {len(self.joints)} joints '
            f'(pulse={self.pulse_duration}s, repeat={self.repeat_interval}s, auto_repeat={self.auto_repeat}). '
            'Will pulse reset_fault until OP_ENABLED (watchdog).')

        self.pub = self.create_publisher(Float64MultiArray, '/brake_release_controller/commands', 10)
        # Poll every 0.5s until controller_manager ready, then start 1s pulse and watchdog
        self._init_timer = self.create_timer(0.5, self.try_activate)

    def try_activate(self):
        if not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('controller_manager not ready, retrying...', throttle_duration_sec=5.0)
            return
        # Activate brake_release_controller if requested (once)
        if self.activate_controller and not self._activated:
            req = SwitchController.Request()
            req.activate_controllers = [self.controller_name]
            req.deactivate_controllers = []
            req.strictness = SwitchController.Request.STRICT
            req.activate_asap = False
            req.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
            future = self.cli.call_async(req)
            future.add_done_callback(self._on_activate_done)
            self.get_logger().info(f'Activating {self.controller_name} until OP_ENABLED...')
            # cancel init timer after first attempt; pulse will be triggered on service response
            if self._init_timer is not None:
                self._init_timer.cancel()
                self._init_timer = None
            return
        # Already activated — fallback direct pulse if service path not used
        self._start_pulsing()

    def _on_activate_done(self, future):
        try:
            result = future.result()
            if result.ok:
                self.get_logger().info(f'{self.controller_name} activated.')
                self._activated = True
                self._start_pulsing()
                return
            else:
                self.get_logger().warn(
                    f'Failed to activate {self.controller_name} (controller not loaded yet), retrying in {self.repeat_interval}s...')
        except Exception as e:
            self.get_logger().warn(f'SwitchController call failed: {e}, retrying...')
        # Retry: keep init timer alive until success
        self._activated = False
        if self._init_timer is None:
            self._init_timer = self.create_timer(self.repeat_interval, self.try_activate)

    def _start_pulsing(self):
        if self._init_timer is not None:
            self._init_timer.cancel()
            self._init_timer = None
        self.pulse()
        if self.auto_repeat and self._repeat_timer is None:
            self._repeat_timer = self.create_timer(self.repeat_interval, self.pulse)

    def pulse(self):
        msg = Float64MultiArray()
        msg.data = [1.0] * len(self.joints)
        self.pub.publish(msg)
        self.get_logger().info(f'Pulsed reset_fault=1.0 for {self.pulse_duration}s (watchdog until OP_ENABLED)')
        # Ensure previous clear timer is cancelled before creating new one
        if self._clear_timer is not None:
            self._clear_timer.cancel()
        self._clear_timer = self.create_timer(self.pulse_duration, self.clear_once)

    def clear_once(self):
        if self._clear_timer is not None:
            self._clear_timer.cancel()
            self._clear_timer = None
        msg = Float64MultiArray()
        msg.data = [0.0] * len(self.joints)
        self.pub.publish(msg)
        self.get_logger().info('Cleared reset_fault=0.0')

    def destroy_node(self):
        # Ensure we leave reset_fault at 0.0 on shutdown
        try:
            if hasattr(self, 'pub'):
                msg = Float64MultiArray()
                msg.data = [0.0] * len(self.joints)
                self.pub.publish(msg)
        except Exception:
            pass
        if self._init_timer is not None:
            self._init_timer.cancel()
        if self._repeat_timer is not None:
            self._repeat_timer.cancel()
        if self._clear_timer is not None:
            self._clear_timer.cancel()
        super().destroy_node()

def main():
    rclpy.init()
    node = BrakeReleaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
