from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/front_camera/image_raw', description='Camera topic name'),
        DeclareLaunchArgument('lidar_topic', default_value='/front_lidar_scan', description='Lidar topic name'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/diff_drive_controller/cmd_vel_unstamped', description='Cmd_vel topic name'),
        DeclareLaunchArgument('odom_topic', default_value='/diff_drive_controller/odom', description='Odometry topic name'),
        DeclareLaunchArgument('laser_deviation_topic', default_value='/laser_deviation', description='Laser deviation topic'),
        DeclareLaunchArgument('image_deviation_topic', default_value='/image_deviation', description='Image deviation topic'),
        DeclareLaunchArgument('depth_image_topic', default_value='/front_camera/depth/image_raw', description='Topic for depth image data'),
        ######################################################################################################################################


        Node(
            package='teachrepeat',
            executable='mapping.py',
            name='map_recorder',
            output='screen',
            parameters=[
                {
                    
                    'camera_topic': LaunchConfiguration('camera_topic'),
                    'lidar_topic': LaunchConfiguration('lidar_topic'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                }
            ]
        ),

        Node(
            package='teachrepeat', 
            executable='deviation_ros.py',
            name='deviation_node',
            output='screen',
            parameters=[
                {
                    'laser_scan_topic': LaunchConfiguration('lidar_topic'),
                    'image_topic': LaunchConfiguration('camera_topic'),
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
