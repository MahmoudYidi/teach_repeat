from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/diff_drive_controller/cmd_vel_unstamped', description='velocity topic'),
        DeclareLaunchArgument('laser_deviation_topic', default_value='/laser_deviation', description='Laser deviation topic'),
        DeclareLaunchArgument('image_deviation_topic', default_value='/image_deviation', description='Image deviation topic'),
        DeclareLaunchArgument('output_folder', default_value='rosbag_data', description='Folder for saved laser and image data'),
        DeclareLaunchArgument('record_interval', default_value='3.0', description='Interval in seconds for deviation calculation'),
        DeclareLaunchArgument('laser_scan_topic', default_value='/front_lidar_scan', description='Topic for laser scan data'),
        DeclareLaunchArgument('image_topic', default_value='/front_camera/image_raw', description='Topic for camera image data'),
        DeclareLaunchArgument('depth_image_topic', default_value='/front_camera/depth/image_raw', description='Topic for depth image data'),
        DeclareLaunchArgument('odom_topic', default_value='/diff_drive_controller/odom', description='Topic for odometry data'),
        ###################################################################################################################################################

        Node(
            package='teachrepeat', 
            executable='deviation_ros.py',
            name='deviation_node',
            output='screen',
            parameters=[
                {
                    'scan_folder': LaunchConfiguration('output_folder'),
                    'image_folder': LaunchConfiguration('output_folder'),
                    'deviation_interval': LaunchConfiguration('record_interval'),
                    'laser_scan_topic': LaunchConfiguration('laser_scan_topic'),
                    'image_topic': LaunchConfiguration('image_topic'),
                    'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'laser_deviation_topic': LaunchConfiguration('laser_deviation_topic'),
                    'image_deviation_topic': LaunchConfiguration('image_deviation_topic'),
                }
            ]

        ),

        Node(
            package='teachrepeat', 
            executable='process_ros.py',
            name='processor_node',
            output='screen',
            parameters=[
                {
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                    'laser_topic': LaunchConfiguration('laser_deviation_topic'),
                    'image_topic': LaunchConfiguration('image_deviation_topic'),
                }
            ]
        ),

    ])

