#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from task_2v2.srv import TurtleCommand

class TurtleClient(Node):

    def __init__(self):
        super().__init__('turtle_client')

        self.client = self.create_client(TurtleCommand, 'turtle_command')

        # a timeout for the service requests
        while not self.client.wait_for_service(timeout_sec=1.0):  # if service doesn't respond in given time
            self.get_logger().info('service not available, waiting again...')

        # to create a TurtleCommand object for preparing to request
        self.request = TurtleCommand.Request()

    def send_request(self, x, y, vx, vy):
        self.request.x = x
        self.request.y = y
        self.request.vx = vx
        self.request.vy = vy

        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            return self.future.result()
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    turtle_client = TurtleClient()

    response = turtle_client.send_request(3.0, 3.0, 1.0, 1.0)  # the actual request
    turtle_client.get_logger().info(f'Success: {response.success}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()