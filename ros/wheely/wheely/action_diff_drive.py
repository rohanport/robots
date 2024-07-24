#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_messages.msg import ActionDiffDrive
from geometry_msgs.msg import Twist

class ActionDiffDriveNode(Node):

    def __init__(self):
        super().__init__('action_diff_drive')

        self.action_subscription = self.create_subscription(
            ActionDiffDrive,
            'rxinfer/wheely/action/diff_drive',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        

    def listener_callback(self, msg):
        ang_vel = max(-1.0, min(msg.ang_vel, 1.0))
        trans_vel = max(-0.5, min(msg.trans_vel, 0.5))
        twist = Twist()
        twist.linear.x = trans_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = ang_vel
        
        self.publisher.publish(twist)

        


def main(args=None):
    rclpy.init(args=args)

    action_diff_drive = ActionDiffDriveNode()

    rclpy.spin(action_diff_drive)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
