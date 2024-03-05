cmake_minimum_required(VERSION 3.8)
project(parc_robot_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY config launch worlds 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
