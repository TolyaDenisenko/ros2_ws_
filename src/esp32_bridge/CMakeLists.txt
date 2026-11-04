cmake_minimum_required(VERSION 3.8)
project(esp32_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED)            # Asio — header-only
find_package(Threads REQUIRED)

# Разбор протокола — отдельная библиотека без ROS, чтобы тестировать её отдельно
add_library(esp32_protocol STATIC src/protocol.cpp)
target_include_directories(esp32_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# Нода (ros2 run esp32_bridge esp32_bridge), транспорт UDP или USB — параметр transport
add_executable(esp32_bridge src/bridge_node.cpp)
target_link_libraries(esp32_bridge
  esp32_protocol
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  Boost::boost
  Threads::Threads)

install(TARGETS esp32_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol esp32_protocol)
  # protocol.h прошивки — чтобы проверять совместимость форматов ESP32 и ноды
  target_include_directories(test_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/esp32_bridge/include)
endif()

ament_package()
