cmake_minimum_required(VERSION 3.8)
project(space_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PathApproval.srv"
)

# Install Python executables
install(PROGRAMS
  space_project/astar_navigation_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME astar_navigation_node
)

install(PROGRAMS
  space_project/battery_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME battery_manager
)

install(PROGRAMS
  space_project/collision_coordinator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collision_coordinator_node
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
