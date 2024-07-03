#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from custom_messages.msg import SensoryStatePosition
from tf_transformations import euler_from_quaternion

class SensorPositionNode(Node):

    def __init__(self):
        super().__init__('sensor_position')

        self.wheely_position = self.create_subscription(
            TransformStamped,
            'model/wheely/pose',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(SensoryStatePosition, 'ros/wheely/sensory_states/position', 10)
        
    def listener_callback(self, msg):
        if(msg.child_frame_id == 'wheely'):
            
            orientation_list = [
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w
            ]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

            sensory_state = SensoryStatePosition()
            sensory_state.x = msg.transform.translation.x
            sensory_state.y = msg.transform.translation.y
            sensory_state.yaw = yaw
            
            self.publisher.publish(sensory_state)


def main(args=None):
    rclpy.init(args=args)

    sensor_position = SensorPositionNode()

    rclpy.spin(sensor_position)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
