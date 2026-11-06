cmake_minimum_required(VERSION 3.22.1)
project(orca_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install mission and Lua script files
install(
  DIRECTORY missions ../lua_scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_dive_and_hold.py
    TARGET test_dive_and_hold
    TIMEOUT 180
  )

  add_launch_test(
    test/test_dive_and_hold_mono_imu.py
    TARGET test_dive_and_hold_mono_imu
    TIMEOUT 180
  )

  add_launch_test(
    test/test_vpe_hold.py
    TARGET test_vpe_hold
    TIMEOUT 300
  )

  add_launch_test(
    test/test_vpe_hold_mono_imu.py
    TARGET test_vpe_hold_mono_imu
    TIMEOUT 300
  )

  add_launch_test(
    test/test_guided_out_and_back.py
    TARGET test_guided_out_and_back
    TIMEOUT 240
  )

  add_launch_test(
    test/test_guided_out_and_back_mono_imu.py
    TARGET test_guided_out_and_back_mono_imu
    TIMEOUT 240
  )
endif()

ament_package()
