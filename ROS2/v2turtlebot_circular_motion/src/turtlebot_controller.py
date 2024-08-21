#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # for sending a single float32 value
from geometry_msgs.msg import Twist
from v2turtlebot_circular_motion.srv import ComputeAngVel

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.subscription = self.create_subscription(Float32, 'radius', self.radius_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.client = self.create_client(ComputeAngVel, 'compute_ang_vel')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ComputeAngVel.Request()

    def radius_callback(self, msg):
        self.req.radius = msg.data
        future = self.client.call_async(self.req)
        future.add_done_callback(self.angular_velocity_callback)
        self.get_logger().info(f'Subscribed radius: {self.req.radius}')

    def angular_velocity_callback(self, future):
        try:
            response = future.result()
            twist = Twist()
            twist.linear.x = 0.1  # default value
            twist.angular.z = response.angular_velocity
            self.publisher_.publish(twist)
            self.get_logger().info(f'Published: Linear Velocity: 0.1, Angular Velocity: {response.angular_velocity}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()