cmake_minimum_required(VERSION 3.8)
project(franka_gz_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

set(DEMO_HELPERS resume_gazebo_when_controller_ready)
include("${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/install_helpers.cmake")

ament_package()
