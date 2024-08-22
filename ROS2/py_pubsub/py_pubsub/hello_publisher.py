#!/usr/bin/env python3

import rclpy
from rclpy.node import Node  # node class in rclpy
from std_msgs.msg import String

class HelloPublisher(Node):

    def __init__(self):
        super().__init__("hello_node")
        self.publisher_ = self.create_publisher(String, 'hello', 10)  # topic named "hello"
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello,'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)  # writes logs


def main(args=None):
    rclpy.init(args=args)  # initialize rclpy
    node = HelloPublisher()
    rclpy.spin(node)  # continously run node until ctrl+c'ed
    rclpy.shutdown()

if __name__ == "__main__":
    main()


