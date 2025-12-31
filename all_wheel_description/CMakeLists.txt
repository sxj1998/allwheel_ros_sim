cmake_minimum_required(VERSION 3.8)
project(all_wheel_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY world/
  DESTINATION share/${PROJECT_NAME}/world
)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  PROGRAMS
    scripts/omni_cmd_vel.py
    scripts/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
