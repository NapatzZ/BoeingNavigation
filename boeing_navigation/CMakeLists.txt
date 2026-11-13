cmake_minimum_required(VERSION 3.8)
project(boeing_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/waypoint_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_manager)
install(PROGRAMS scripts/waypoint_cmd.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_cmd)

ament_package()
