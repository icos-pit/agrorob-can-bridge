cmake_minimum_required(VERSION 3.8)
project(agrorob_can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(can_bridge src/can_bridge.cpp)
target_include_directories(can_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(can_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  can_bridge
  "rclcpp"
  "can_msgs"
  "std_msgs"
  "sensor_msgs"
)

install(TARGETS can_bridge
  DESTINATION lib/${PROJECT_NAME})



ament_package()
