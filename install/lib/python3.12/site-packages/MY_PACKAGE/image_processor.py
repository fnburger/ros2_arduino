#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Int32, '/dominant_color_code', 10)
        self.bridge = CvBridge()
        self.get_logger().info('ImageProcessor started. Subscribing to /camera/image_raw')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Split channels: B, G, R (OpenCV is BGR)
        b, g, r = cv2.split(cv_image)
        # Sum pixel values per channel (ignores alpha if any)
        sum_b = np.sum(b)
        sum_g = np.sum(g)
        sum_r = np.sum(r)
        # Determine dominant: 0=Red, 1=Green, 2=Blue
        sums = [sum_r, sum_g, sum_b]
        dominant_idx = np.argmax(sums)
        color_code = Int32()
        color_code.data = int(dominant_idx)  # Cast to Python int!
        self.publisher_.publish(color_code)
        self.get_logger().info(f'Sums - R:{int(sum_r)}, G:{int(sum_g)}, B:{int(sum_b)} | Dominant: {int(dominant_idx)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()