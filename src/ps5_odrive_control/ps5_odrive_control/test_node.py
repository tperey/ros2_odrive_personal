import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestNode(Node):
    def __init__(self):
        super().__init__('ps5_odrive_node')
        # Publisher on the 'chatter' topic
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # Timer to publish every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from PS5 ODrive!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
