cmake_minimum_required(VERSION 3.8)
project(parc_robot_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CropYield.msg"
 )

ament_package()
