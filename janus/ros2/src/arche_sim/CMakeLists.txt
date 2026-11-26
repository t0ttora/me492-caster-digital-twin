cmake_minimum_required(VERSION 3.8)
project(arche_sim)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(marker "${CMAKE_CURRENT_BINARY_DIR}/marker.png")
add_custom_command(OUTPUT "${marker}"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_marker.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/robot_config.json" "${marker}"
  DEPENDS scripts/make_marker.py config/robot_config.json
  VERBATIM
)
add_custom_target(janus_marker ALL DEPENDS "${marker}")
install(DIRECTORY worlds models config DESTINATION share/${PROJECT_NAME})
install(FILES "${marker}" DESTINATION share/${PROJECT_NAME}/models/janus_dock/meshes)
ament_package()
