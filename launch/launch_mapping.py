from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        #DeclareLaunchArgument('output_folder', default_value='rosbag_data', description='Folder for saving rosbag data and sensor outputs'),
        DeclareLaunchArgument('record_interval', default_value='3.0', description='Time interval (seconds) for saving sensor data'),
        DeclareLaunchArgument('camera_topic', default_value='/front_camera/image_raw', description='Camera topic name'),
        DeclareLaunchArgument('lidar_topic', default_value='/front_lidar_scan', description='Lidar topic name'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/diff_drive_controller/cmd_vel_unstamped', description='Cmd_vel topic name'),
        DeclareLaunchArgument('odom_topic', default_value='/diff_drive_controller/odom', description='Odometry topic name'),
        ######################################################################################################################################


        Node(
            package='teachrepeat',
            executable='mapping.py',
            name='map_recorder',
            output='screen',
            parameters=[
                {
                    #'output_folder': LaunchConfiguration('output_folder'),
                    'record_interval': LaunchConfiguration('record_interval'),
                    'camera_topic': LaunchConfiguration('camera_topic'),
                    'lidar_topic': LaunchConfiguration('lidar_topic'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                }
            ]
        )
    ])
