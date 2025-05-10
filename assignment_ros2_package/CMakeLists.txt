cmake_minimum_required(VERSION 3.5)
project(assignment_ros2_package)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# Add executables
add_executable(ros2_node_function1_position_control src/ros2_node_function1_position_control.cpp)
ament_target_dependencies(ros2_node_function1_position_control rclcpp px4_msgs)

add_executable(ros2_node_function2_velocity_control src/ros2_node_function2_velocity_control.cpp)
ament_target_dependencies(ros2_node_function2_velocity_control rclcpp px4_msgs)

# Install targets
install(TARGETS
  ros2_node_function1_position_control
  ros2_node_function2_velocity_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
