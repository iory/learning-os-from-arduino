cmake_minimum_required(VERSION 3.8)
project(arduino_os_quad_robot)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME})
ament_package()
