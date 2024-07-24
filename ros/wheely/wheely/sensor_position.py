#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from custom_messages.msg import SensoryStatePosition
from custom_messages.msg import Pause
from tf_transformations import euler_from_quaternion

class SensorPositionNode(Node):

    def __init__(self):
        super().__init__('sensor_position')

        self.pause_sub = self.create_subscription(
            Pause,
            'dashboard/wheely/pause',
            self.pause_callback,
            10)
        
        self.publisher = self.create_publisher(SensoryStatePosition, 'ros/wheely/sensory_states/position', 10)

        self.is_paused = True
        self.set_paused(False)

    def pause_callback(self, msg):
        self.set_paused(msg.pause)

    def set_paused(self, pause):
        if (pause and not self.is_paused):
            self.destroy_subscription(self.pose_sub)
            self.is_paused = True
        elif (not pause and self.is_paused):
            self.pose_sub = self.create_subscription(
                TransformStamped,
                'model/wheely/pose',
                self.pose_callback,
                10)
            self.is_paused = False
        
        self.get_logger().info(f'paused={self.is_paused}')
        
    def pose_callback(self, msg):
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
