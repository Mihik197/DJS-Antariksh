#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class FigureEightPublisher(Node):

    def __init__(self):
        super().__init__('figure_eight_publisher')

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)  # current coordinates

        # spawn coordinates for pose message (just for initialization)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.timer = self.create_timer(0.1, self.move_infinity)        

        self.duration = 40  # duration to complete half loop
        self.start_time = self.get_clock().now().to_msg().sec

    def pose_callback(self, pose):
        self.x = pose.x
        self.y = pose.y
        self.theta = pose.theta

    def move_infinity(self):
        current_time = self.get_clock().now().to_msg().sec
        elapsed_time = current_time - self.start_time
        move_cmd = Twist()
        move_cmd.linear.x = 0.1

        if elapsed_time < self.duration:
            move_cmd.angular.z = 2 * math.pi / self.duration
        elif elapsed_time < 2 * self.duration:
            move_cmd.angular.z = -2 * math.pi / self.duration
        else:
            self.start_time = current_time  # so that we don't stop after just one loop

        self.publisher_.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    figure_eight_publisher = FigureEightPublisher()
    rclpy.spin(figure_eight_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()