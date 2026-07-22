import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DebugViewerNode(Node):
    def __init__(self):
        super().__init__("aruco_debug_viewer")
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
        topic = self.get_parameter("image_topic").value
        self._bridge = CvBridge()
        self.create_subscription(Image, topic, self._image_callback, 1)
        self.get_logger().info(f"Debug viewer subscribed to {topic}")

    def _image_callback(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("ArUco Detection", frame)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = DebugViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
