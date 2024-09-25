import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class rover_teleop_node(Node):
    
    def __init__(self):
        super().__init__('rover_teleop_node')
        self.subscriptions = self.create_subscription(String, '/motion', self.DoMotion, 10)
        self.subscriptions
    
    def DoMotion(self, msg):
        self.get_logger().info