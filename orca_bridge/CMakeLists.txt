cmake_minimum_required(VERSION 3.22.1)
project(orca_bridge)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_pose
    scripts/test_pose.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

# Install Python nodes
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/slam_bridge.py
  scripts/camera_info_publisher.py
  scripts/pose_to_path.py
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
install(
  FILES
  scripts/geometry.py
  scripts/slam.py
  scripts/sub.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
