#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.serialization import serialize_message
from cv_bridge import CvBridge
import json
import os
import csv
import cv2
from teachrepeat.action import MapRecord  


class MapRecorder(Node):
    def __init__(self):
        super().__init__('map_recorder')

        # Action server setup
        self._action_server = ActionServer(
            self,
            MapRecord,
            'record_map',
            self.execute_callback
        )

        # Data buffer and default values
        self.data_buffer = {}
        self.image_counter = 1
        self.lidar_counter = 1
        # Declare parameters
        self.declare_parameter('camera_topic', '')
        self.declare_parameter('lidar_topic', '')
        self.declare_parameter('cmd_vel_topic', '')
        self.declare_parameter('odom_topic', '')

        # Retrieve parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.bridge = CvBridge()

        # Initialize the bag writer (
        self.bag_writer = None

        # CSV for odometry data
        self.odom_csv_file = None
        self.csv_writer = None

        # Default velocity handling
        self.default_velocity = Twist()
        self.is_default_velocity = True
        self.recording_active = False  
        self.get_logger().info("Mapping Node succesfully initialised")

    def execute_callback(self, goal_handle):
        # Extract parameters from goal
        output_folder = goal_handle.request.output_folder
        record_interval = goal_handle.request.record_interval
        start_map = goal_handle.request.start_map

        if not start_map:
            self.stop_recording()
            goal_handle.succeed()
            result = MapRecord.Result()
            result.success = True
            result.message = 'Recording stopped successfully.'
            return result

        self.setup_recording(output_folder, record_interval)
        goal_handle.succeed()
        result = MapRecord.Result()
        result.success = True
        result.message = 'Recording started successfully.'
        return result

    def setup_recording(self, output_folder, record_interval):
        try:
            self.output_folder = output_folder
            self.record_interval = record_interval

            self.bag_writer = self.create_bag_writer(self.output_folder)

            self.odom_csv_file = os.path.join(self.output_folder, "odom_data.csv")
            self.csv_file = open(self.odom_csv_file, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Timestamp', 'X', 'Y'])

            self.is_default_velocity = True

            # Immediately start recording default velocity
            self.save_velocity_data(self.default_velocity)

           
            self.topics_to_record = {
                self.camera_topic: Image,
                self.lidar_topic: LaserScan,
                self.cmd_vel_topic: Twist,
                self.odom_topic: Odometry
            }
            for topic, msg_type in self.topics_to_record.items():
                self.create_subscription(msg_type, topic, self.create_callback(topic), 10)

            self.recording_active = True
            self.timer = self.create_timer(self.record_interval, self.save_data)

            
            self.get_logger().info(f"Recording started: saving data every {self.record_interval} seconds")
        except Exception as e:
            self.get_logger().error(f"Error starting recording: {e}")

    def stop_recording(self):
        if self.recording_active:
            try:
                # Close CSV and rosbag
                if self.csv_file:
                    self.csv_file.close()

                if self.bag_writer:
                    #self.bag_writer.close()
                    self.bag_writer = None 

                self.recording_active = False
                self.get_logger().info("Recording stopped.")
            except Exception as e:
                self.get_logger().error(f"Error stopping recording: {str(e)}")
        else:
            self.get_logger().info("Recording is already stopped.")

    def create_bag_writer(self, filename):
        storage_options = StorageOptions(uri=filename, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        bag_writer = SequentialWriter()
        bag_writer.open(storage_options, converter_options)

        cmd_vel_metadata = TopicMetadata(
            name=self.cmd_vel_topic,
            type='geometry_msgs/msg/Twist',
            serialization_format='cdr'
        )
        bag_writer.create_topic(cmd_vel_metadata)
        return bag_writer

    def create_callback(self, topic):
        def callback(msg):
            self.data_buffer[topic] = (msg, self.get_clock().now())
            if topic == self.cmd_vel_topic:
                self.handle_velocity_message(msg)
            elif topic == self.odom_topic:
                self.save_odometry(msg)
        return callback

    def handle_velocity_message(self, msg):
        if self.is_default_velocity:
            self.is_default_velocity = False
        self.save_velocity_data(msg)

    def save_velocity_data(self, msg):
        try:
            timestamp = self.get_clock().now()
            serialized_msg = serialize_message(msg)
            if serialized_msg is not None:
                self.bag_writer.write(self.cmd_vel_topic, serialized_msg, timestamp.nanoseconds)
            self.get_logger().info("Saved velocity data to rosbag")
        except Exception as e:
            self.get_logger().error(f"Error saving velocity data: {str(e)}")

    def save_data(self):
        if self.recording_active:
            try:
                if self.data_buffer.get(self.camera_topic) is not None:
                    self.save_image(self.data_buffer[self.camera_topic][0])

                if self.data_buffer.get(self.lidar_topic) is not None:
                    self.save_lidar_scan(self.data_buffer[self.lidar_topic][0])

                self.get_logger().info(f"Saved image and scan data")
            except Exception as e:
                self.get_logger().error(f"Error saving data: {str(e)}") 

    def save_odometry(self, msg):
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            timestamp = self.get_clock().now().to_msg().sec
            self.csv_writer.writerow([timestamp, x, y])
        except Exception as e:
            #self.get_logger().error(f"Error saving odometry data: {str(e)}")
            pass

    def save_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            image_filename = os.path.join(self.output_folder, f"image_{self.image_counter}.png")
            cv2.imwrite(image_filename, cv_image)
            self.image_counter += 1
        except Exception as e:
            self.get_logger().error(f"Error saving image: {str(e)}")

    def save_lidar_scan(self, msg):
        try:
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
            lidar_filename = os.path.join(self.output_folder, f"lidar_scan_{self.lidar_counter}.json")
            with open(lidar_filename, 'w') as f:
                json.dump(lidar_data, f, default=str)
            self.lidar_counter += 1
        except Exception as e:
            self.get_logger().error(f"Error saving lidar scan: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    recorder = MapRecorder()
    rclpy.spin(recorder)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
