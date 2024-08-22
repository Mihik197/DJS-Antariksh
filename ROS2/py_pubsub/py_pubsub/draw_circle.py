#!/usr/bin/env python3

# for learning purpose
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)  # third arg is queue size
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")  # writes logs
        self.start_time = time.time()
        self.t = 0.0


    def send_velocity_command(self):
        msg = Twist()
        
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        
        self.cmd_vel_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)  # initialize rclpy
    node = DrawCircleNode()
    rclpy.spin(node)  # continously run node until ctrl+c'ed
    rclpy.shutdown()

if __name__ == "__main__":
    main()