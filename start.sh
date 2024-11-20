#!/bin/bash

# Source ROS 2 setup script
source /opt/ros/humble/setup.bash

# Source your custom workspace setup (if applicable)
source ~/ros2_ws/install/setup.bash

# Launch rtabmap with parameters (directly passing arguments as you would in a launch file)
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/front_camera/image_raw \
    depth_topic:=/front_camera/depth/image_raw \
    camera_info_topic:=/front_camera/camera_info \
    frame_id:=base_link \
    approx_sync:=false \
    qos:=1 \
    rviz:=true &

# Wait for the process to continue
wait
