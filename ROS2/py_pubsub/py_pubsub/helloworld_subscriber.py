#!/usr/bin/env python3

import rclpy
from rclpy.node import Node  # node class in rclpy
from std_msgs.msg import String

class HelloWorldSubscriber(Node):

    def __init__(self):
        super().__init__("helloworld_subscriber")
        self.hello_subscriber_ = self.create_subscription(String, 'hello', self.hello_callback, 10)  # subscribed to topic named "hello"
        self.world_subscriber_ = self.create_subscription(String, 'world', self.world_callback, 10)
        self.publisher_ = self.create_publisher(String, 'helloworld', 10)
        self.hello_msg = ""
        self.world_msg = ""

    def hello_callback(self, msg):
        self.hello_msg = msg.data

    def world_callback(self, msg):
        self.world_msg = msg.data

        combined_msg = String()
        combined_msg.data = self.hello_msg + " " + self.world_msg
        self.publisher_.publish(combined_msg)
        self.get_logger().info('Publishing: "%s"' % combined_msg.data)


def main(args=None):
    rclpy.init(args=args)  # initialize rclpy
    node = HelloWorldSubscriber()
    rclpy.spin(node)  # continously run node until ctrl+c'ed
    rclpy.shutdown()

if __name__ == "__main__":
    main()