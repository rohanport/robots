#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped


class FoodManagerNode(Node):

    def __init__(self):
        super().__init__('food_manager')

        self.wheely_position = self.create_subscription(
            TransformStamped,
            'model/wheely/pose',
            self.listener_callback,
            10)
        self.j = 0

        # self.wheely_position  # prevent unused variable warning
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def listener_callback(self, msg):
        if(msg.child_frame_id == 'wheely'):
            if (self.j % 100 == 0):
                self.get_logger().info('wheely position: "%s"' % msg.transform.translation)
            self.j += 1


def main(args=None):
    rclpy.init(args=args)

    food_manager = FoodManagerNode()

    rclpy.spin(food_manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
