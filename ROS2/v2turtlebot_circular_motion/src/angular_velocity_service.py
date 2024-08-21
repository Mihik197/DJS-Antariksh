#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from v2turtlebot_circular_motion.srv import ComputeAngVel

class AngularVelocityService(Node):
    def __init__(self):
        super().__init__('angular_velocity_service')
        self.srv = self.create_service(ComputeAngVel, 'compute_ang_vel', self.compute_ang_vel_callback)

    def compute_ang_vel_callback(self, request, response):
        linear_velocity = 0.1  # default value
        radius = request.radius
        if radius != 0:  # to avoid / by 0 error
            angular_velocity = linear_velocity / radius
        else:
            angular_velocity = 0.0
        response.angular_velocity = angular_velocity
        self.get_logger().info(f'radius: {radius}, angular velocity: {angular_velocity}')
        return response

def main(args=None):
    rclpy.init(args=args)
    angular_velocity_service = AngularVelocityService()
    rclpy.spin(angular_velocity_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()