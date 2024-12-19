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
    def __init__(self):
       pass

    def combine_deviations(self, laser_deviation, image_deviation):
        #print('laser', laser_deviation)
        #print('image', image_deviation)
        #combined_sign = np.sign(laser_deviation + image_deviation)
        combined_deviation = ((0.5 * laser_deviation) + (0.5 * image_deviation)) / 1 #Weighted equal 
        #combined_deviation = combined_sign * weighted_average
        #print('combined',combined_deviation)
        return combined_deviation
        

    def compute_velocity_correction(self, combined_deviation, kp=0.000025):
        gained = kp * combined_deviation
        #print('gained',gained)
 
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
                print("Finished processing rosbag.")
                del bag_reader
                break
            except Exception as e:
                print(f"Error reading rosbag: {e}")
                break

