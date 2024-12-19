#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 {start_mapping|stop_mapping|repeat_map} map_name record_interval"
    exit 1
fi

COMMAND=$1
MAP_NAME=$2
RECORD_INTERVAL=$3

# Check the command and run the corresponding ROS 2 action
case "$COMMAND" in
    start_map)
        #ros2 action send_goal /bearnav2/mapmaker bearnav2/action/MapMaker "{map_name: '$MAP_NAME', start: true}"
        ros2 action send_goal /record_map teachrepeat/action/MapRecord "{output_folder: '$MAP_NAME', record_interval: $RECORD_INTERVAL, start_map: true}"
        ;;
    stop_map)
        ros2 action send_goal /record_map teachrepeat/action/MapRecord "{output_folder: '$MAP_NAME', record_interval: $RECORD_INTERVAL, start_map: false}"
        ;;
    repeat_map)
        ros2 action send_goal /repeat_map teachrepeat/action/MapRepeat "{output_folder: '$MAP_NAME', record_interval: $RECORD_INTERVAL}"
        ;;
    *)
        echo "Unknown command: $COMMAND"
        echo "Usage: $0 {start_mapping|stop_mapping|repeat_map} map_name"
        exit 1
        ;;
esac