cmake_minimum_required(VERSION 3.8)
project(hoverbot_nav)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
ament_target_dependencies(scan_to_cloud rclcpp sensor_msgs laser_geometry)

install(TARGETS scan_to_cloud DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
