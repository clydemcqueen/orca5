cmake_minimum_required(VERSION 3.22.1)
project(orca_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install mission files
install(
  DIRECTORY missions
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_dive_and_hold.py
    TARGET test_dive_and_hold
    TIMEOUT 180
  )
endif()

ament_package()
