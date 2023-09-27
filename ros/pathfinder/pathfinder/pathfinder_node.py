#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class PathfinderNode(Node):

    def __init__(self):
        super().__init__("pathfinder")
        self.get_logger().info("Hello from pathfinder")


def main(args=None):
    rclpy.init(args=args)

    node = PathfinderNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()