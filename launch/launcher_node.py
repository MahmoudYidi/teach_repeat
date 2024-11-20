from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch DeviationCalculator node
        Node(
            package='teach_repeat',  # Replace with your package name
            executable='deviation_ros.py',
            name='deviation_calculator',
            output='screen'
        ),
        
        # Launch ProcessorNode
        Node(
            package='teach_repeat',  # Replace with your package name
            executable='process_ros.py',
            name='processor_node',
            output='screen'
        ),
    ])
