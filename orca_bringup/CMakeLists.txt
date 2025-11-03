cmake_minimum_required(VERSION 3.22.1)
project(orca_bringup)

find_package(ament_cmake REQUIRED)

ament_environment_hooks(hooks/setup.dsv)

# Generate orca5/model.sdf from orca5/model.sdf.in
set(MODEL_NAMES orca5)

foreach(MODEL_NAME ${MODEL_NAMES})
  set(MODEL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/models/${MODEL_NAME})
  set(MODEL_SDF_IN ${MODEL_DIR}/model.sdf.in)
  set(MODEL_SDF ${CMAKE_CURRENT_BINARY_DIR}/models/${MODEL_NAME}/model.sdf)

  # Generate model.sdf from model.sdf.in
  add_custom_command(
    OUTPUT ${MODEL_SDF}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/models/${MODEL_NAME}
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_model.py
            ${MODEL_SDF_IN} ${MODEL_SDF} 0
    DEPENDS ${MODEL_SDF_IN} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_model.py
    COMMENT "Generating ${MODEL_NAME}/model.sdf from model.sdf.in"
  )
  
  # Create a custom target that depends on the generated file
  add_custom_target(
    generate_${MODEL_NAME}_model ALL
    DEPENDS ${MODEL_SDF}
  )
  
  # Install the generated model.sdf
  install(
    FILES ${MODEL_SDF}
    DESTINATION share/${PROJECT_NAME}/models/${MODEL_NAME}
  )
endforeach()

# Install other directories
install(
  DIRECTORY
  config
  launch
  models
  param
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
