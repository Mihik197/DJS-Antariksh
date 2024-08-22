#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from task_2v2.srv import TurtleCommand

class TurtleServer(Node):

    def __init__(self):
        super().__init__('turtle_server')

        self.vx = 0.0
        self.vy = 0.0

        self.srv = self.create_service(TurtleCommand, 'turtle_command', self.spawn_turtle_callback)
        self.spawn_client = self.create_client(Spawn, '/spawn')  # client for spawning
        self.get_logger().info('Turtle Server is ready.')
        self.cmd_vel_pub = self.create_publisher(Twist, '/mihik/cmd_vel', 10)  # publisher for velocity
        self.timer = self.create_timer(0.1, self.publish_velocity)

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')

        # to create a spawn object to send to /spawn service
        self.spawn_request = Spawn.Request()

    def spawn_turtle_callback(self, request, response):
        self.spawn_request.x = request.x
        self.spawn_request.y = request.y
        self.spawn_request.theta = 0.0
        self.spawn_request.name = 'mihik'

        self.vx = request.vx
        self.vy = request.vy

        spawn_future = self.spawn_client.call_async(self.spawn_request)
        self.get_logger().info(f'spawned turtle at ({request.x}, {request.y}).')
        response.success = True
        return response

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = float(self.vx)
        twist.linear.y = self.vy
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'velocity: linear.x={twist.linear.x}, linear.y={twist.linear.y}')

def main(args=None):
    rclpy.init(args=args)
    turtle_server = TurtleServer()
    rclpy.spin(turtle_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()