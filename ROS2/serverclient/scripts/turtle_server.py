#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from serverclient.srv import TurtleCommand

class TurtleServer(Node):

    def __init__(self):
        super().__init__('turtle_server')

        # default values (initial state)
        self.default_x = 5.0
        self.default_y = 5.0
        self.default_vx = 0.0
        self.default_vy = 0.0

        self.srv = self.create_service(TurtleCommand, 'turtle_command', self.turtle_command_callback)
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.get_logger().info('Turtle Server is ready.')

    def spawn_turtle_callback(self, request, response):
        # to create a spawn object to send to /spawn service
        spawn_request = Spawn.Request()

        # default values if not provided in the request
        spawn_request.x = request.x if request.x != 0.0 else self.default_x
        spawn_request.y = request.y if request.y != 0.0 else self.default_y
        spawn_request.theta = 0.0

        spawn_future = self.spawn_client.call_async(spawn_request)
        rclpy.spin_until_future_complete(self, spawn_future)

        if spawn_future.result() is not None:
            self.get_logger().info(f'Spawned turtle at ({request.x}, {request.x}).')

            # only if it is spawned properly, we should move the turtle
            twist = Twist()
            twist.linear.x = request.vx if request.vx != 0.0 else self.default_vx
            twist.linear.y = request.vy if request.vy != 0.0 else self.default_vy
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'Moving turtle with velocity ({request.vx}, {request.vy}).')

            response.success = True  # return response true
            return response

def main(args=None):
    rclpy.init(args=args)
    turtle_server = TurtleServer()
    rclpy.spin(turtle_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()