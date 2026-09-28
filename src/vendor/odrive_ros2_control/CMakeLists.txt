cmake_minimum_required(VERSION 3.8)
project(odrive_ros2_control)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
add_library(odrive_ros2_control_plugin SHARED
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/odrive_hardware_interface.cpp)
target_include_directories(odrive_ros2_control_plugin PRIVATE ../odrive_base/include)
target_compile_features(odrive_ros2_control_plugin PRIVATE cxx_std_20)
target_compile_options(odrive_ros2_control_plugin PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(odrive_ros2_control_plugin rclcpp hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)
install(TARGETS odrive_ros2_control_plugin LIBRARY DESTINATION lib)
ament_package()
