#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BlueObjectDetector(Node):
    def __init__(self):
        super().__init__('blue_object_detector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Publisher for the processed image
        self.publisher = self.create_publisher(
            Image,
            '/detected_blue_objects',
            10)
        
        self.get_logger().info('Blue Object Detector Node has been started')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            return
        
        # Process the image to detect blue objects
        processed_image = self.detect_blue_objects(cv_image)
        
        # Convert back to ROS Image message and publish
        try:
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            self.publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {str(e)}')
    
    def detect_blue_objects(self, image):
        # Convert BGR to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for blue color in HSV
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Bitwise-AND mask and original image
        blue_objects = cv2.bitwise_and(image, image, mask=mask)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw bounding boxes around detected blue objects
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter small detections
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(image, 'Blue Object', (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return image

def main(args=None):
    rclpy.init(args=args)
    blue_object_detector = BlueObjectDetector()
    rclpy.spin(blue_object_detector)
    blue_object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()