cmake_minimum_required(VERSION 3.8)
project(parc_robot_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(pluginlib REQUIRED)

install(
  DIRECTORY config launch worlds 
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/task1_params.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
