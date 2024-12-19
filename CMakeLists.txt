cmake_minimum_required(VERSION 3.8)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(teachrepeat)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MapRecord.action"
  "action/MapRepeat.action"
  DEPENDENCIES
    std_msgs
    action_msgs 
)
# Install Python modules
install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/mapping.py
  src/recorder.py
  src/repeater.py
  src/deviation/deviation.py
  src/deviation/deviation_ros.py
  src/process/process.py
  src/process/process_ros.py


  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
