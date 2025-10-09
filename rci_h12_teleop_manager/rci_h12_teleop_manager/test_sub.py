import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


from copy import deepcopy
class TestSub(Node):
    def __init__(self):
        super().__init__("unity_wrist_publisher")
        self.subscription = self.create_subscription(Float32, '/rci_h12_manager/l_wrist_pose', self.test_cb, 10)

        cal_timer_period = 0.01  # seconds
        self.cal_timer = self.create_timer(cal_timer_period, self.cal_timer_callback)
    def test_cb(self, msg):
        self.get_logger().info(f"Here : {msg.data}")


def main():
    rclpy.init()
    node = TestSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

