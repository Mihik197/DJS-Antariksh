#!/usr/bin/env python3

# for learning purpose
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)  # last arg is queue size

    def pose_callback(self, msg: Pose):
        self.get_logger().info("(" + str(msg.x) + ", " + str(msg.y) + ")")  # writes logs        

def main(args=None):
    rclpy.init(args=args)  # initialize rclpy
    node = PoseSubscriberNode()
    rclpy.spin(node)  # continously run node until ctrl+c'ed
    rclpy.shutdown()

if __name__ == "__main__":
    main()
