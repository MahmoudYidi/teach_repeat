import math
import os
import time
import numpy as np
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import Twist
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rclpy.time import Time

class DeviationProcessor:
    def __init__(self, fx=185.90483944879418, fy=185.90483944879418, cx=320.5, cy=240.5, camera_distance=1.0):
        """
        Initialize the processor using full camera parameters.
        :param fx: Focal length in the x-direction (in pixels).
        :param fy: Focal length in the y-direction (in pixels).
        :param cx: Principal point in the x-direction (in pixels).
        :param cy: Principal point in the y-direction (in pixels).
        :param camera_distance: Distance from the camera to the observed plane in meters (Z).
        """
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.camera_distance = camera_distance
        
        # Calculate the pixel-to-meter ratio based on the focal length and distance (Z)
        #self.image_to_meter_ratio = self.camera_distance / self.fx  # Use fx for horizontal scaling

    #def normalize_deviations(self, laser_deviation, image_deviation):
   
        # Convert image deviation from pixels to meters using the calculated ratio
        #normalized_image_deviation = image_deviation * self.image_to_meter_ratio
        #return laser_deviation, normalized_image_deviation

    def combine_deviations_rmse(self, laser_deviation, image_deviation):
        print('laser', laser_deviation)
        print('image', image_deviation)
        combined_sign = np.sign(laser_deviation + image_deviation)
        rmse = math.sqrt((laser_deviation**2 + image_deviation**2) / 2)
        combined_deviation = combined_sign * rmse
        print('combined',combined_deviation)
        return combined_deviation
        

    def compute_velocity_correction(self, combined_deviation, kp=0.001):
        gained = -kp * combined_deviation
        print('gained',gained)
 
        return gained
    def create_bag_reader(self, directory):
        db_files = [f for f in os.listdir(directory) if f.endswith('.db3')]
        if not db_files:
            raise FileNotFoundError("No .db3 files found in the specified directory")

        db_file = db_files[0]
        file_path = os.path.join(directory, db_file)

        storage_options = StorageOptions(uri=file_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        bag_reader = SequentialReader()
        bag_reader.open(storage_options, converter_options)
        return bag_reader

    def process_rosbag(self, bag_reader, topic, callback):
        previous_timestamp = None

        while True:
            try:
                topic_name, serialized_msg, timestamp = bag_reader.read_next()
                if topic_name == topic:
                    msg = deserialize_message(serialized_msg, Twist)

                    if previous_timestamp is not None:
                        current_time = Time(seconds=timestamp / 1e9)
                        prev_time = Time(seconds=previous_timestamp / 1e9)
                        delay = (current_time - prev_time).nanoseconds / 1e9
                        if delay > 0:
                            time.sleep(delay)

                    callback(msg)
                    previous_timestamp = timestamp
            except StopIteration:
                break

