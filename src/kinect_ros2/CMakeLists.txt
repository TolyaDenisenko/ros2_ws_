cmake_minimum_required(VERSION 3.8)
project(kinect_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREENECT REQUIRED libfreenect)

include_directories(
  include
  ${FREENECT_INCLUDE_DIRS}
)

add_library(kinect_ros2_component SHARED
  src/kinect_ros2_component.cpp
)

ament_target_dependencies(kinect_ros2_component
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  ament_index_cpp
  camera_info_manager
  tf2_ros
)

target_link_libraries(kinect_ros2_component
  ${FREENECT_LIBRARIES}
)

rclcpp_components_register_nodes(kinect_ros2_component "kinect_ros2::KinectRosComponent")

install(TARGETS kinect_ros2_component
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
