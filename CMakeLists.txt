cmake_minimum_required(VERSION 3.8)
project(compliant_controllers_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(PROGRAMS
  scripts/spacemouse_to_cartesian_command.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
