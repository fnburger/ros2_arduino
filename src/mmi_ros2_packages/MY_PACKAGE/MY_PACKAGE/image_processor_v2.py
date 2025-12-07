#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ImageProcessorV2(Node):
    def __init__(self):
        super().__init__('image_processor_v2')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Int32, '/dominant_color_code', 10)
        self.bridge = CvBridge()
        
        # Full paths to Haar cascades (Ubuntu standard)
        cascades_dir = '/usr/share/opencv4/haarcascades/'
        face_path = os.path.join(cascades_dir, 'haarcascade_frontalface_default.xml')
        smile_path = os.path.join(cascades_dir, 'haarcascade_smile.xml')
        
        # Load with path checks
        if not os.path.exists(face_path):
            self.get_logger().error(f'Face cascade not found at {face_path}. Install libopencv-dev.')
            return
        if not os.path.exists(smile_path):
            self.get_logger().error(f'Smile cascade not found at {smile_path}. Install libopencv-contrib-dev.')
            return
        
        face_cascade = cv2.CascadeClassifier(face_path)
        smile_cascade = cv2.CascadeClassifier(smile_path)
        if face_cascade.empty() or smile_cascade.empty():
            self.get_logger().error('Failed to load Haar cascades (empty classifiers).')
            return
        self.face_cascade = face_cascade
        self.smile_cascade = smile_cascade
        
        self.get_logger().info('ImageProcessorV2 started with face/smile detection.')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        
        color_code = Int32()
        if len(faces) == 0:
            # No face: Red (0)
            color_code.data = 0
            self.get_logger().info('No face detected → Red (0)')
        else:
            # Face detected: Check for smile in each face ROI
            smile_detected = False
            for (x, y, w, h) in faces:
                # Draw rectangle for viz
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                # ROI for smile (whole face ROI, not just lower half—better detection)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = cv_image[y:y+h, x:x+w]
                smiles = self.smile_cascade.detectMultiScale(
                    roi_gray, scaleFactor=1.7, minNeighbors=20, minSize=(25, 25))
                if len(smiles) > 0:
                    smile_detected = True
                    # Draw smile rect
                    for (sx, sy, sw, sh) in smiles:
                        cv2.rectangle(roi_color, (sx, sy), (sx+sw, sy+sh), (0, 255, 0), 2)
                    break  # One smile is enough
            
            if smile_detected:
                color_code.data = 2  # Blue (2)
                self.get_logger().info('Smile detected → Blue (2)')
            else:
                color_code.data = 1  # Green (1)
                self.get_logger().info('Face detected (no smile) → Green (1)')
        
        self.publisher_.publish(color_code)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()