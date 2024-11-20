#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry  # For robot orientation
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

class DeviationCalculator(Node):
    def __init__(self):
        super().__init__('deviation_calculator')

        # Declare parameters
        self.declare_parameter('scan_folder', 'rosbag_data')
        self.declare_parameter('image_folder', 'rosbag_data')
        self.declare_parameter('deviation_interval', 3.0)

        # Retrieve parameters
        self.scan_folder = self.get_parameter('scan_folder').value
        self.image_folder = self.get_parameter('image_folder').value
        self.deviation_interval = self.get_parameter('deviation_interval').value

        # Create CvBridge object for converting ROS image messages
        self.bridge = CvBridge()

        # Create subscription to the latest laser scan, image, and odometry (for orientation)
        self.create_subscription(LaserScan, '/front_lidar_scan', self.scan_callback, 1)
        self.create_subscription(Image, '/front_camera/image_raw', self.image_callback, 1)
        self.create_subscription(Image, '/front_camera/depth/image_raw', self.depth_callback, 1)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)  # Subscribe to Odometry for robot orientation

        # Create publishers for deviation topics
        self.laser_deviation_publisher = self.create_publisher(Float32, '/laser_deviation', 1)
        self.image_deviation_publisher = self.create_publisher(Float32, '/image_deviation', 1)

        # Timer to trigger deviation calculation based on interval
        self.timer = self.create_timer(self.deviation_interval, self.calculate_and_publish_deviations)

        # Initialize variables
        self.last_ros_scan = None
        self.last_ros_image = None
        self.depth_image = None
        self.last_orientation = 0.0  # Initialize robot orientation (yaw angle)
        self.scan_files = sorted(glob.glob(os.path.join(self.scan_folder, 'lidar_scan_*.json')))
        self.image_files = sorted(glob.glob(os.path.join(self.image_folder, 'image_*.png')))
        self.scan_index = 0
        self.image_index = 0

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
        # Convert depth message to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')  # Assuming depth image is 32-bit float
        depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
        self.depth_image = depth_image
    
    def odom_callback(self, msg):
        """Callback to handle the incoming odometry data and extract the robot's orientation (yaw)."""
        try:
            # Extract quaternion orientation from odometry message
            quat = msg.pose.pose.orientation
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            euler = self.quaternion_to_euler([quat.x, quat.y, quat.z, quat.w])
            # Store the yaw (orientation) angle
            self.last_orientation = euler[2]  # yaw in radians
        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {e}")

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles."""
        # Roll, pitch, yaw
        x, y, z, w = quat
        roll_x = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_y = math.asin(2.0 * (w * y - z * x))
        yaw_z = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return [roll_x, pitch_y, yaw_z]  # Return the Euler angles (roll, pitch, yaw)

    def calculate_and_publish_deviations(self):
        """Perform deviation calculations only when the interval is reached."""
        # Check if we have data to compare
        if self.last_ros_scan is not None and self.last_ros_image is not None and self.depth_image is not None:
            try:
                # Only process the next scan and image if they are available
                if self.scan_index < len(self.scan_files) and self.image_index < len(self.image_files):
                    # Process laser scan comparison
                    saved_scan_path = self.scan_files[self.scan_index]
                    self.scan_index += 1
                    with open(saved_scan_path, 'r') as f:
                        saved_scan = json.load(f)
                        saved_ranges = np.array(saved_scan.get('ranges', []))
                        saved_ranges = saved_ranges[~np.isnan(saved_ranges) & ~np.isinf(saved_ranges)]

                    # Pass the robot's orientation (yaw) to the deviation calculation
                    laser_deviation = calculate_laser_deviation(saved_ranges, self.last_ros_scan, self.last_orientation)
                    self.laser_deviation_publisher.publish(Float32(data=laser_deviation))
                    self.get_logger().info(f"Published laser deviation: {laser_deviation:.2f} meters")

                    # Process image comparison
                    saved_image_path = self.image_files[self.image_index]
                    self.image_index += 1
                    saved_image = cv2.imread(saved_image_path)

                    image_deviation = calculate_image_deviation(saved_image, self.last_ros_image, self.depth_image, self.last_orientation)
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
