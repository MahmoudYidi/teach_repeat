#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32  # For receiving laser and image deviations
from process import DeviationProcessor  # Import the updated DeviationProcessor class
import threading

class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor')
        
        # Initialize the DeviationProcessor with camera parameters
        self.processor = DeviationProcessor(
            fx=185.90483944879418,  # Focal length in the x-direction (in pixels)
            fy=185.90483944879418,  # Focal length in the y-direction (in pixels)
            cx=320.5,               # Principal point in the x-direction (in pixels)
            cy=240.5,               # Principal point in the y-direction (in pixels)
            camera_distance=1.0     # Camera distance (Z) in meters
        )

        self.velocity_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 1)

        # Subscribers for laser and image deviation
        self.laser_deviation_subscriber = self.create_subscription(
            Float32, '/laser_deviation', self.laser_deviation_callback, 1)
        
        self.image_deviation_subscriber = self.create_subscription(
            Float32, '/image_deviation', self.image_deviation_callback, 1)

        # Variables to store received deviations
        self.laser_deviation = 0.0
        self.image_deviation = 0.0
        self.angular_correction = 0.0  # Initialize the angular correction to 0

    def laser_deviation_callback(self, msg: Float32):
        """Callback for laser deviation subscription"""
        self.laser_deviation = msg.data
        #self.get_logger().info(f"Laser deviation updated: {self.laser_deviation}")

    def image_deviation_callback(self, msg: Float32):
        """Callback for image deviation subscription"""
        self.image_deviation = msg.data
        #self.get_logger().info(f"Image deviation updated: {self.image_deviation}")

    def velocity_callback(self, msg: Twist):
        # If no message is received, set the deviation to 0
        if self.laser_deviation == 0.0 and self.image_deviation == 0.0:
            self.angular_correction = 0.0  # Reset the correction if no new deviation data is available
        else:
            # Normalize the image and laser deviations
            #normalized_laser_deviation, normalized_image_deviation = self.processor.normalize_deviations(
              #  self.laser_deviation, self.image_deviation
            #)

            # Combine both deviations using RMSE
            combined_deviation = self.processor.combine_deviations_rmse(self.laser_deviation, self.image_deviation)

            # Compute the velocity correction based on the combined deviation
            self.angular_correction = self.processor.compute_velocity_correction(combined_deviation)
            #self.get_logger().info(f"Angular correction computed: {self.angular_correction}")

        # Adjust the Twist message with corrected linear and angular velocities
        corrected_velocity = Twist()
        corrected_velocity.linear = msg.linear

        # If no correction is needed (angular_correction is 0), keep the original angular velocities
        if self.angular_correction == 0.0:
            corrected_velocity.angular = msg.angular
        else:
            corrected_velocity.angular.x = msg.angular.x
            corrected_velocity.angular.y = msg.angular.y
            corrected_velocity.angular.z = msg.angular.z + self.angular_correction  # Apply the correction

        # Publish the adjusted velocity
        self.velocity_pub.publish(corrected_velocity)
        #self.get_logger().info(f"Adjusted velocity: linear.x={corrected_velocity.linear.x}, angular.z={corrected_velocity.angular.z}")

    def read_rosbag_and_publish(self, bag_path):
        try:
            # Read and process the rosbag data
            bag_reader = self.processor.create_bag_reader(bag_path)
            self.processor.process_rosbag(
                bag_reader,
                '/diff_drive_controller/cmd_vel_unstamped',
                self.velocity_callback
            )
            self.get_logger().info("Finished processing rosbag.")
        except Exception as e:
            self.get_logger().error(f"Error reading rosbag: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessorNode()

    # Start a separate thread to read the rosbag and process data
    bag_thread = threading.Thread(
        target=node.read_rosbag_and_publish,
        args=('rosbag_data',),  # Replace with the correct path to the bag
        daemon=True
    )
    bag_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down processor node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
