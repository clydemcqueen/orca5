cmake_minimum_required(VERSION 3.22.1)
project(orca_msgs)

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if(_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach(_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach()
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate ROS messages
rosidl_generate_interfaces(
  orca_msgs
  msg/BridgeStatus.msg
  msg/FilterStatus.msg
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_auto_package()