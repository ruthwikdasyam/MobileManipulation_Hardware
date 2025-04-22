#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthPrinter:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('depth_printer', anonymous=True)

        # Bridge to convert ROS messages to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the depth image topic
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)

        # The pixel coordinates to check (change these values for different pixels)
        self.pixel_x = 320  # Horizontal pixel (center of the image, assuming 640x480)
        self.pixel_y = 240  # Vertical pixel (center of the image, assuming 480p)

        # Start the ROS loop
        rospy.spin()

    def depth_callback(self, msg):
        # Convert the ROS depth image to OpenCV format (16-bit single-channel image)
        try:
            depth_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")
            return

        # Print the shape of the depth image (optional, for debugging)
        rospy.loginfo(f"Depth image shape: {depth_cv_image.shape}")

        # Access the pixel at the specified location
        depth_value = depth_cv_image[self.pixel_y, self.pixel_x]

        # Convert depth to meters (RealSense depth is typically in millimeters)
        depth_in_meters = depth_value / 1000.0

        # Print the depth at the specified pixel
        rospy.loginfo(f"Depth at pixel ({self.pixel_x}, {self.pixel_y}) = {depth_in_meters:.3f} meters")

if __name__ == '__main__':
    try:
        # Create an instance of the DepthPrinter class
        DepthPrinter()
    except rospy.ROSInterruptException:
        pass
