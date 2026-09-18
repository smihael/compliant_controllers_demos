cmake_minimum_required(VERSION 3.8)
project(surface_following)

find_package(ament_cmake REQUIRED)

install(DIRECTORY fixtures notebooks docker urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/surface_demo.py
  scripts/surface_baseline.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/surface_following.py
  scripts/surface_ros_common.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
