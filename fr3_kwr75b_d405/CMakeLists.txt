cmake_minimum_required(VERSION 3.8)
project(fr3_kwr75b_d405)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch urdf meshes config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/zero_ft_sensor.py scripts/republish_tcp_wrench.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
