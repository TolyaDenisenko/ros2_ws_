cmake_minimum_required(VERSION 3.8)
project(twist_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(twist_to_twist_stamped src/twist_to_twist_stamped.cpp)
target_link_libraries(twist_to_twist_stamped rclcpp::rclcpp ${geometry_msgs_TARGETS})

install(TARGETS twist_to_twist_stamped DESTINATION lib/${PROJECT_NAME})

ament_package()
