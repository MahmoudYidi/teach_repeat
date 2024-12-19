#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry  # For robot position
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import os
import cv2
import glob
import time
from deviation import calculate_laser_deviation, calculate_image_deviation
import numpy as np
import json
import math
import csv  # Import the CSV module

class DeviationCalculator(Node):
    def __init__(self):
        super().__init__('deviation_calculator')

        # Declare parameters for folders, interval, and topics
        self.declare_parameter('scan_folder', '')
        self.declare_parameter('image_folder', '')
        self.declare_parameter('deviation_interval', 0.0)
        self.declare_parameter('laser_scan_topic', '')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('depth_image_topic', '')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('laser_deviation_topic', '')
        self.declare_parameter('image_deviation_topic', '')

        # Retrieve parameters
        self.scan_folder = self.get_parameter('scan_folder').value
        self.image_folder = self.get_parameter('image_folder').value
        self.deviation_interval = self.get_parameter('deviation_interval').value
        self.laser_scan_topic = self.get_parameter('laser_scan_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_image_topic = self.get_parameter('depth_image_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.laser_deviation_topic = self.get_parameter('laser_deviation_topic').value
        self.image_deviation_topic = self.get_parameter('image_deviation_topic').value

        # Create CvBridge object for converting ROS image messages
        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(LaserScan, self.laser_scan_topic, self.scan_callback, 1)
        self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.create_subscription(Image, self.depth_image_topic, self.depth_callback, 1)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Publishers
        self.laser_deviation_publisher = self.create_publisher(Float32, self.laser_deviation_topic, 1)
        self.image_deviation_publisher = self.create_publisher(Float32, self.image_deviation_topic, 1)

        # Timer to trigger deviation calculation based on interval
        self.timer = self.create_timer(self.deviation_interval, self.calculate_and_publish_deviations)

        # Initialize variables
        self.last_ros_scan = None
        self.last_ros_image = None
        self.depth_image = None
        self.last_position = (0.0, 0.0)  # Initialize robot position (x, y)
        self.scan_files = sorted(glob.glob(os.path.join(self.scan_folder, 'lidar_scan_*.json')))
        self.image_files = sorted(glob.glob(os.path.join(self.image_folder, 'image_*.png')))
        self.scan_index = 0
        self.image_index = 0

        # Open the CSV file in append mode to store odometry data
        self.odom_output_file = os.path.join(self.scan_folder, "odom_data2.csv")
        with open(self.odom_output_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            # Write header only if the file is empty
            if file.tell() == 0:
                writer.writerow(['Timestamp', 'X', 'Y'])  # CSV header row

        self.get_logger().info(f"Started deviation calculator with {self.deviation_interval} seconds interval")

    def scan_callback(self, msg):
        """Callback to handle the incoming laser scan."""
        try:
            # Convert LaserScan message to numpy array
            ranges = np.array(msg.ranges)
            ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]  # Filter invalid data
            self.last_ros_scan = ranges
        except Exception as e:
            self.get_logger().error(f"Error processing laser scan: {e}")

    def image_callback(self, msg):
        """Callback to handle the incoming image."""
        try:
            # Convert ROS image message to OpenCV image
            ros_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_ros_image = ros_image
        except Exception as e:
            self.get_logger().error(f"Error converting ROS image: {e}")

    def depth_callback(self, msg):
        """Process the depth image and store it for deviation calculation."""
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
        self.depth_image = depth_image

    def odom_callback(self, msg):
        """Callback to handle the incoming odometry data and extract the robot's position (x, y)."""
        try:
            position = msg.pose.pose.position
            x, y = position.x, position.y
            self.last_position = (x, y)
            with open(self.odom_output_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time.time(), x, y])
        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {e}")

    def calculate_and_publish_deviations(self):
        """Perform deviation calculations only when the interval is reached."""
        if self.last_ros_scan is not None and self.last_ros_image is not None and self.depth_image is not None:
            try:
                if self.scan_index < len(self.scan_files) and self.image_index < len(self.image_files):
                    saved_scan_path = self.scan_files[self.scan_index]
                    self.scan_index += 1
                    with open(saved_scan_path, 'r') as f:
                        saved_scan = json.load(f)
                        saved_ranges = np.array(saved_scan.get('ranges', []))
                        saved_ranges = saved_ranges[~np.isnan(saved_ranges) & ~np.isinf(saved_ranges)]

                    laser_deviation = calculate_laser_deviation(saved_ranges, self.last_ros_scan, self.last_position)
                    self.laser_deviation_publisher.publish(Float32(data=laser_deviation))
                    self.get_logger().info(f"Published laser deviation: {laser_deviation:.2f} meters")

                    saved_image_path = self.image_files[self.image_index]
                    self.image_index += 1
                    saved_image = cv2.imread(saved_image_path)

                    image_deviation = calculate_image_deviation(saved_image, self.last_ros_image, self.depth_image, self.last_position)
                    self.image_deviation_publisher.publish(Float32(data=image_deviation))
                    self.get_logger().info(f"Published image deviation: {image_deviation:.2f} meters")
            except Exception as e:
                self.get_logger().error(f"Error during deviation calculation: {e}")

def main(args=None):
    rclpy.init(args=args)
    deviation_calculator = DeviationCalculator()
    rclpy.spin(deviation_calculator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
