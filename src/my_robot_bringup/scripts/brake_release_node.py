#!/usr/bin/env python3
"""Brake release helper for SIM2015D CIA402 drives.
Claims reset_fault command_interfaces and pulses them to clear Fault state.
Use when auto_fault_reset fails due to timing or STO. Publishes 1.0 for 0.5s then 0.0.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController

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
        self.declare_parameter('pulse_duration', 0.5)
        self.declare_parameter('repeat_interval', 5.0)

        self.joints = self.get_parameter('joints').value
        self.auto_repeat = self.get_parameter('auto_repeat').value
        self.pulse_duration = float(self.get_parameter('pulse_duration').value)
        self.repeat_interval = float(self.get_parameter('repeat_interval').value)

        # Try to use forward_command_controller if available, otherwise direct service
        self.cli = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.timer = None
        self.get_logger().info(f'Brake release node ready for {len(self.joints)} joints. Will pulse reset_fault via controller_manager if available.')

        # Publish to forward_command_controller's commands topic
        self.pub = self.create_publisher(Float64MultiArray, '/brake_release_controller/commands', 10)
        self.create_timer(1.0, self.try_activate)

    def try_activate(self):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('controller_manager not ready, retrying...')
            return
        self.get_logger().info('Attempting to pulse reset_fault interfaces...')
        self.pulse()
        if self.auto_repeat:
            self.create_timer(self.repeat_interval, self.pulse)
        self.destroy_timer(self.timer) if hasattr(self, 'timer') and self.timer else None

    def pulse(self):
        msg = Float64MultiArray()
        msg.data = [1.0] * len(self.joints)
        self.pub.publish(msg)
        self.get_logger().info('Pulsed reset_fault=1.0')
        # reset to 0 after pulse_duration
        self.create_timer(self.pulse_duration, self.clear)

    def clear(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * len(self.joints)
        self.pub.publish(msg)
        self.get_logger().info('Cleared reset_fault=0.0')

def main():
    rclpy.init()
    node = BrakeReleaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
