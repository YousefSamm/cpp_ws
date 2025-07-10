#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from custom_interfaces.action import Navigate
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster


class SpeedSubscriber(Node):
    def __init__(self):
        super().__init__('speed_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            'speed',
            self.listener_callback,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received speed: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    speed_subscriber = SpeedSubscriber()
    rclpy.spin(speed_subscriber)
    speed_subscriber.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()