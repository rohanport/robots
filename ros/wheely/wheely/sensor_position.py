#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from custom_messages.msg import SensoryStatePosition


class SensorPositionNode(Node):

    def __init__(self):
        super().__init__('sensor_position')

        self.wheely_position = self.create_subscription(
            TransformStamped,
            'model/wheely/pose',
            self.listener_callback,
            10)
        self.position = SensoryStatePosition()
        self.position.x = 0.0
        self.position.y = 0.0

        self.publisher = self.create_publisher(SensoryStatePosition, 'sensory_state_position', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_position)

    def publish_position(self):
        self.publisher.publish(self.position)

    def listener_callback(self, msg):
        if(msg.child_frame_id == 'wheely'):
            self.position.x = msg.transform.translation.x
            self.position.y = msg.transform.translation.y


def main(args=None):
    rclpy.init(args=args)

    sensor_position = SensorPositionNode()

    rclpy.spin(sensor_position)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
