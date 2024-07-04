#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_messages.msg import FoodPosition
from custom_messages.msg import SensoryStatePosition
from random import random
from math import dist

class FoodManagerNode(Node):

    def __init__(self):
        super().__init__('food_manager')

        self.spawn_food()
        self.wheely_position_sub = self.create_subscription(
            SensoryStatePosition,
            'ros/wheely/sensory_states/position',
            self.wheely_position_callback,
            10)
        
        self.publisher = self.create_publisher(FoodPosition, 'ros/wheely/food/position', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.publisher.publish(self.food_position)
        
    def spawn_food(self):
        self.food_position = FoodPosition()
        self.food_position.x = 15 * (random() - 0.5)
        self.food_position.y = 15 * (random() - 0.5)
        self.get_logger().info(f'food_position={self.food_position}')

    def wheely_position_callback(self, msg):
        dist_to_food = dist([msg.x, msg.y], [self.food_position.x, self.food_position.y])
        if (dist_to_food < 1.0):
            self.spawn_food()


def main(args=None):
    rclpy.init(args=args)

    food_manager = FoodManagerNode()

    rclpy.spin(food_manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
