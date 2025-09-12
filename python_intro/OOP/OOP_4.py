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
        self.get_logger().info(f"sent: {msg.data}")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
Zakaj OOP v ROSu?

    Omogoča lepo strukturiranje node-ov, da so spremenljivke in funkcije združene v objekt.

    Lažje upravljanje z več funkcionalnostmi (več publisherjev, subscriberjev, servisov).

    Enostavno podedovanje in prilagoditev obnašanja node-ov.

    Povečuje berljivost in vzdrževanje kode.
"""