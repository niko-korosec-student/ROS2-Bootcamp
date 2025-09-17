"""
OOP Example 4: ROS2 Nodes

In robotics, OOP is especially useful in ROS2:
- Node attributes store publishers, subscribers, and timers
- Methods define behavior (callbacks, actions)
- Encapsulation keeps everything neatly inside one class
- Inheritance allows us to build more complex nodes from simple ones

Here we define a MinimalNode:
- Publishes a String message every second
- Uses a counter (self.i) as part of its state
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hi, counting: {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
