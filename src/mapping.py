#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.serialization import serialize_message
import json
import os
from cv_bridge import CvBridge
import numpy as np
import cv2

class RosbagTimeRecorder(Node):
    def __init__(self):
        super().__init__('rosbag_time_recorder')

        # Declare parameters
        self.declare_parameter('output_folder', 'rosbag_data')  # Folder for the rosbag
        self.declare_parameter('record_interval', 3.0)  # Time interval in seconds

        # Retrieve parameters
        self.output_folder = self.get_parameter('output_folder').value
        self.record_interval = self.get_parameter('record_interval').value

        # Create rosbag writer for cmd_vel (velocity) data only
        self.bag_writer = self.create_bag_writer(self.output_folder)

        # Define topics to record (cmd_vel for rosbag, others for raw data)
        self.topics_to_record = {
            '/front_camera/image_raw': Image,
            '/front_lidar_scan': LaserScan,
            '/diff_drive_controller/cmd_vel_unstamped': Twist,
            '/diff_drive_controller/odom': Odometry
        }

        # Data buffer to store latest messages
        self.data_buffer = {topic: None for topic in self.topics_to_record}

        # Initialize CVBridge for image conversion
        self.bridge = CvBridge()

        # Create subscriptions and add topics to rosbag writer
        for topic, msg_type in self.topics_to_record.items():
            if topic == '/diff_drive_controller/cmd_vel_unstamped':
                # Save velocity data to rosbag (cmd_vel)
                self.create_subscription(msg_type, topic, self.create_callback(topic), 10)
            else:
                # Subscribe to non-rosbag topics
                self.create_subscription(msg_type, topic, self.create_callback(topic), 10)

        # Initialize counters for filenames
        self.image_counter = 1
        self.lidar_counter = 1

        # Timer to save messages at the specified interval
        self.timer = self.create_timer(self.record_interval, self.save_data)

        self.get_logger().info(f"Time-based file recording started with {self.record_interval} seconds interval")

    def create_bag_writer(self, filename):
        # Create a rosbag writer to record data
        storage_options = StorageOptions(uri=filename, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')  # Default options
        bag_writer = SequentialWriter()
        bag_writer.open(storage_options, converter_options)

        # Define the metadata for the topics
        cmd_vel_metadata = TopicMetadata(
            name='/diff_drive_controller/cmd_vel_unstamped',
            type='geometry_msgs/msg/Twist', 
            serialization_format='cdr'
        )

        # Create the topic in the bag
        bag_writer.create_topic(cmd_vel_metadata)

        return bag_writer

    def create_callback(self, topic):
        def callback(msg):
            # Buffer the latest message for each topic
            self.data_buffer[topic] = (msg, self.get_clock().now())
            if topic == '/diff_drive_controller/cmd_vel_unstamped':
                # Save cmd_vel data to rosbag continuously
                self.save_velocity_data(msg)

        return callback

    def save_data(self):
        try:
            # Save the images and lidar data based on the specified interval
            if self.data_buffer.get('/front_camera/image_raw') is not None:
                self.save_image(self.data_buffer['/front_camera/image_raw'][0])

            if self.data_buffer.get('/front_lidar_scan') is not None:
                self.save_lidar_scan(self.data_buffer['/front_lidar_scan'][0])

            self.get_logger().info(f"Saved image and lidar data at {self.record_interval} seconds interval")
        except Exception as e:
            self.get_logger().error(f"Error saving data: {str(e)}")

    def save_velocity_data(self, msg):
        try:
            # Save cmd_vel data to rosbag (cmd_vel)
            timestamp = self.get_clock().now()
            serialized_msg = serialize_message(msg)  # Correct use of serialize_message
            if serialized_msg is not None:
                # Write to rosbag
                self.bag_writer.write('/diff_drive_controller/cmd_vel_unstamped', serialized_msg, timestamp.nanoseconds)

            self.get_logger().info("Saved cmd_vel data to ros_bag.bag")
        except Exception as e:
            self.get_logger().error(f"Error saving velocity data: {str(e)}")


    def save_image(self, msg):
        try:
            # Convert the ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Save the image as a PNG file with a sequential name
            image_filename = os.path.join(self.output_folder, f"image_{self.image_counter}.png")
            cv2.imwrite(image_filename, cv_image)
            self.image_counter += 1
            self.get_logger().info(f"Saved image to {image_filename}")

        except Exception as e:
            self.get_logger().error(f"Error saving image: {str(e)}")

    def save_lidar_scan(self, msg):
        try:
            # Convert LaserScan data to JSON format
            lidar_data = {
                'header': msg.header,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges': list(msg.ranges),
                'intensities': list(msg.intensities)
            }

            # Save the lidar data as a JSON file with a sequential name
            lidar_filename = os.path.join(self.output_folder, f"lidar_scan_{self.lidar_counter}.json")
            with open(lidar_filename, 'w') as f:
                json.dump(lidar_data, f, default=str)
            self.lidar_counter += 1
            self.get_logger().info(f"Saved lidar scan to {lidar_filename}")

        except Exception as e:
            self.get_logger().error(f"Error saving lidar scan: {str(e)}")

    def stop_recording(self):
        self.get_logger().info("Stopping rosbag recording")
        self.bag_writer.reset()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = RosbagTimeRecorder()
    rclpy.spin(recorder)
    rclpy.shutdown()


if __name__ == '__main__':
    main()