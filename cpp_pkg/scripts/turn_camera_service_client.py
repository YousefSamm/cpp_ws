#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import TurnCamera
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np


class TurnCameraClientPy(Node):
    def __init__(self):
        super().__init__('turn_camera_client_python')
        self.cli = self.create_client(TurnCamera, 'turn_camera')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = TurnCamera.Request()
        self.bridge = cv_bridge.CvBridge()
        self.image = None
        self.angle = 0.0
        self.image_received = False

    def send_request(self):
        self.angle = input('Enter the angle to turn the camera: ')
        self.angle = float(self.angle)
        self.get_logger().info(f'Camera will turn {self.angle} degrees')
        self.req.degree_turn = self.angle
        future = self.cli.call_async(self.req)
        self.get_logger().info('Request sent')
        
        # Wait for the service response
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                if response.camera_image is not None:
                    self.get_logger().info('Response received')
                    cv_image = self.bridge.imgmsg_to_cv2(response.camera_image, 'bgr8')
                    cv2.imshow("Camera Image", cv_image)
                    cv2.waitKey(0)
                else:
                    self.get_logger().error('Received response, but camera_image is None')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        else:
            self.get_logger().error('Service call did not complete')

def main(args=None):
    rclpy.init(args=args)
    turn_camera_client = TurnCameraClientPy()
    turn_camera_client.send_request()
    turn_camera_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()