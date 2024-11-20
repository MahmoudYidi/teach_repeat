#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
import glob
import time

class ImageDeviationCalculator(Node):
    def __init__(self):
        super().__init__('image_deviation_calculator')

        # Declare parameters
        self.declare_parameter('image_folder', 'rosbag_data')  # Folder where images are saved
        self.declare_parameter('image_interval', 3.0)  # Time interval in seconds

        # Retrieve parameters
        self.image_folder = self.get_parameter('image_folder').value
        self.image_interval = self.get_parameter('image_interval').value

        # Create the CvBridge object for ROS image conversion
        self.bridge = CvBridge()

        # Create subscription to the latest image (current ROS image)
        self.create_subscription(Image, '/front_camera/image_raw', self.image_callback, 10)

        # Timer to check for the saved image and calculate deviation at the specified interval
        self.timer = self.create_timer(self.image_interval, self.check_for_saved_image)

        # Initialize variables
        self.last_saved_image_path = None
        self.last_ros_image = None
        self.processed_images = set()  # Track processed images
        self.image_files = []
        self.image_index = 0

        self.get_logger().info(f"Started image deviation calculator with {self.image_interval} seconds interval")

    def image_callback(self, msg):
        """Callback to handle the incoming ROS image."""
        try:
            # Convert ROS image message to OpenCV image
            ros_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_ros_image = ros_image
            #self.get_logger().info("Updated current ROS image.")  # Debug log
        except Exception as e:
            self.get_logger().error(f"Error converting ROS image: {e}")

    def check_for_saved_image(self):
        """Check for the next saved image and calculate deviation."""
        try:
            # Get all image files in the image folder
            self.image_files = sorted(glob.glob(os.path.join(self.image_folder, 'image_*.png')))

            if not self.image_files:
                self.get_logger().warn("No saved images found!")
                return

            # Only process images in order and at specified intervals
            if self.image_index < len(self.image_files):
                saved_image_path = self.image_files[self.image_index]
                self.image_index += 1  # Move to the next image for next interval

                self.get_logger().info(f"Checking saved image: {saved_image_path}")  # Debug log

                # Read the saved image
                saved_image = cv2.imread(saved_image_path)

                if self.last_ros_image is not None:
                    # Perform ORB feature extraction and matching
                    deviation = self.calculate_deviation(saved_image, self.last_ros_image)
                    self.get_logger().info(f"Deviation distance: {deviation:.2f} pixels")
                else:
                    self.get_logger().warn("No current ROS image to compare with.")
        except Exception as e:
            self.get_logger().error(f"Error checking saved images: {e}")

    def calculate_deviation(self, image1, image2):
        """Calculate the deviation distance between two images using ORB."""
        # Initialize ORB detector
        orb = cv2.ORB_create()

        # Find the keypoints and descriptors with ORB
        kp1, des1 = orb.detectAndCompute(image1, None)
        kp2, des2 = orb.detectAndCompute(image2, None)

        # Create a Brute Force Matcher and match descriptors
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Match descriptors between the two images
        matches = bf.match(des1, des2)

        # Sort them in ascending order of distance
        matches = sorted(matches, key=lambda x: x.distance)

        # Calculate the deviation distance (average distance between matched keypoints)
        total_distance = 0
        num_matches = len(matches)

        for match in matches:
            total_distance += match.distance

        if num_matches > 0:
            average_distance = total_distance / num_matches
            return average_distance
        else:
            return 0

def main(args=None):
    rclpy.init(args=args)
    image_deviation_calculator = ImageDeviationCalculator()
    rclpy.spin(image_deviation_calculator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
