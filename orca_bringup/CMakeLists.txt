cmake_minimum_required(VERSION 3.22.1)
project(orca_bringup)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
  config
  launch
  param
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
