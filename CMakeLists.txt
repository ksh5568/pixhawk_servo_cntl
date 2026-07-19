cmake_minimum_required(VERSION 3.8)
project(pixhawk_servo_cntl)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(servo_bridge_node src/servo_bridge_node.cpp)
ament_target_dependencies(
  servo_bridge_node
  px4_msgs
  rcl_interfaces
  rclcpp
  std_msgs
)

add_executable(servo_control_node src/servo_control_node.cpp)
ament_target_dependencies(servo_control_node rclcpp std_msgs)

target_compile_features(servo_bridge_node PUBLIC cxx_std_17)
target_compile_features(servo_control_node PUBLIC cxx_std_17)

install(TARGETS
  servo_bridge_node
  servo_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
