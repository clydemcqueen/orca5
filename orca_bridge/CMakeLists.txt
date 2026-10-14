cmake_minimum_required(VERSION 3.22.1)
project(orca_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python nodes
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/camera_info_publisher.py
  scripts/pose_to_path.py
  scripts/slam_bridge.py
  scripts/tracked_points_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_pose
    test/test_pose.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
