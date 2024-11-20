#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import json
import numpy as np
import os
import glob
from sklearn.neighbors import NearestNeighbors

class LaserScanDeviationCalculator(Node):
    def __init__(self):
        super().__init__('laser_scan_deviation_calculator')

        # Declare parameters
        self.declare_parameter('scan_folder', 'rosbag_data')  # Folder where saved scans are stored
        self.declare_parameter('scan_interval', 3.0)  # Time interval in seconds

        # Retrieve parameters
        self.scan_folder = self.get_parameter('scan_folder').value
        self.scan_interval = self.get_parameter('scan_interval').value

        # Create subscription to the latest laser scan
        self.create_subscription(LaserScan, '/front_lidar_scan', self.scan_callback, 10)

        # Timer to check for the saved scan and calculate deviation at the specified interval
        self.timer = self.create_timer(self.scan_interval, self.check_for_saved_scan)

        # Initialize variables
        self.last_saved_scan_path = None
        self.last_ros_scan = None
        self.processed_scans = set()  # Track processed scans
        self.scan_files = []
        self.scan_index = 0

        self.get_logger().info(f"Started laser scan deviation calculator with {self.scan_interval} seconds interval")

    def scan_callback(self, msg):
        """Callback to handle the incoming laser scan."""
        try:
            # Convert the LaserScan message to a numpy array (range values)
            ranges = np.array(msg.ranges)
            # Filter out NaN and Infinity values (invalid ranges)
            ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]

            # Store the latest scan data
            self.last_ros_scan = ranges
            #self.get_logger().info("Updated current laser scan.")  # Debug log
        except Exception as e:
            self.get_logger().error(f"Error processing laser scan: {e}")

    def check_for_saved_scan(self):
        """Check for the next saved scan and calculate deviation."""
        try:
            # Get all scan files in the scan folder (e.g., lidar_scan_1.json, lidar_scan_2.json, etc.)
            self.scan_files = sorted(glob.glob(os.path.join(self.scan_folder, 'lidar_scan_*.json')))

            if not self.scan_files:
                self.get_logger().warn("No saved scans found!")
                return

            # Only process scans in order and at specified intervals
            if self.scan_index < len(self.scan_files):
                saved_scan_path = self.scan_files[self.scan_index]
                self.scan_index += 1  # Move to the next scan for next interval

                self.get_logger().info(f"Checking saved scan: {saved_scan_path}")  # Debug log

                # Load the saved scan (assuming saved scans are stored as a dictionary with ranges, header, etc.)
                with open(saved_scan_path, 'r') as f:
                    try:
                        saved_scan = json.load(f)

                        # Extract the relevant fields from the JSON (e.g., ranges, header, etc.)
                        saved_ranges = np.array(saved_scan.get('ranges', []))
                        # Ensure ranges are numeric and filter out invalid data
                        saved_ranges = np.array(saved_ranges, dtype=np.float64)  # Ensure the correct data type
                        saved_ranges = saved_ranges[~np.isnan(saved_ranges) & ~np.isinf(saved_ranges)]  # Clean invalid data

                    except json.JSONDecodeError as e:
                        self.get_logger().error(f"Error loading JSON file {saved_scan_path}: {e}")
                        return

                if self.last_ros_scan is not None:
                    # Calculate deviation between current scan and saved scan
                    deviation = self.calculate_deviation(saved_ranges, self.last_ros_scan)
                    self.get_logger().info(f"Deviation distance: {deviation:.2f} meters")
                else:
                    self.get_logger().warn("No current laser scan to compare with.")
            else:
                self.get_logger().info("All saved scans have been processed.")
        except Exception as e:
            self.get_logger().error(f"Error checking saved scans: {e}")
    
    def calculate_deviation(self, saved_scan, current_scan):
        """Calculate the deviation between two laser scans using a fitting algorithm."""
        # Ensure both scans have valid data
        if saved_scan.size == 0 or current_scan.size == 0:
            self.get_logger().warn("One or both of the scans have no data.")
            return 0.0
        
        # Reshape both scans into points in polar coordinates (r, theta) -> (x, y)
        saved_points = self.polar_to_cartesian(saved_scan)
        current_points = self.polar_to_cartesian(current_scan)

        # Use NearestNeighbors for finding closest points between the two sets
        nn = NearestNeighbors(n_neighbors=1)
        nn.fit(saved_points)
        distances, _ = nn.kneighbors(current_points)

        # Return the average deviation (distance) between corresponding points
        deviation = np.mean(distances)
        return deviation

    def polar_to_cartesian(self, scan):
        """Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)."""
        # Ensure scan data is valid
        if len(scan) == 0:
            return np.array([])

        angles = np.linspace(0, 2 * np.pi, len(scan), endpoint=False)
        x = scan * np.cos(angles)
        y = scan * np.sin(angles)
        points = np.column_stack((x, y))
        return points

def main(args=None):
    rclpy.init(args=args)
    laser_scan_deviation_calculator = LaserScanDeviationCalculator()
    rclpy.spin(laser_scan_deviation_calculator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
