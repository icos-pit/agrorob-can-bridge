cmake_minimum_required(VERSION 3.8)
project(agrorob_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(agrorob_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(agrorob_interface src/main.cpp src/agrorob_interface.cpp)

target_link_libraries(agrorob_interface)

target_include_directories(agrorob_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    agrorob_interface
  "rclcpp"
  "can_msgs"
  "std_msgs"
  "sensor_msgs"
  "agrorob_msgs"
  "geometry_msgs"
)
target_compile_features(agrorob_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS agrorob_interface
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()