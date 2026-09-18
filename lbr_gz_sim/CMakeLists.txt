cmake_minimum_required(VERSION 3.8)
project(lbr_gz_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
