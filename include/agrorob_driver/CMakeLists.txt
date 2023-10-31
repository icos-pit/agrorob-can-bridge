cmake_minimum_required(VERSION 3.5)
project(agrorob_driver)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(agrorob_interfaces REQUIRED)

# Add include directories
include_directories(include)

# Create an executable for the agrorob_driver_node
add_executable(agrorob_driver_node
  src/agrorob_driver_node.cpp
)

# Link the executable with the required libraries
target_link_libraries(agrorob_driver_node
  rclcpp
  agrorob_interfaces
)

# Install the executable
install(TARGETS
  agrorob_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
