cmake_minimum_required(VERSION 3.5)
project(robot_interfaces)

# Require C++14 (recommended by ROS2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies for message generation
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

###############################################
## Declare the messages in this package
###############################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExcavationPoint.msg"
)

###############################################
## Export the runtime dependencies
###############################################

ament_export_dependencies(rosidl_default_runtime)

###############################################
## Ament package
###############################################

ament_package()
