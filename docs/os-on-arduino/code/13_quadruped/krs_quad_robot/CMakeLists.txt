cmake_minimum_required(VERSION 3.0.2)
project(krs_quad_robot)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
