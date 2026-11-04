cmake_minimum_required(VERSION 3.8)
project(hoverbot_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf worlds launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
