cmake_minimum_required(VERSION 3.8)
project(arche_experiments)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/qr_observer scripts/command_guard scripts/replay_velocity scripts/export_bag
  DESTINATION lib/${PROJECT_NAME})
ament_package()
