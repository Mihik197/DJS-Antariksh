#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class RadiusPublisher(Node):
    def __init__(self):
        super().__init__('radius_publisher')
        self.publisher_ = self.create_publisher(Float32, 'radius', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.radius = 1.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.radius
        self.publisher_.publish(msg)
        self.get_logger().info(f'radius: {self.radius}')

def main(args=None):
    rclpy.init(args=args)
    radius_publisher = RadiusPublisher()
    rclpy.spin(radius_publisher)
    radius_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()