cmake_minimum_required(VERSION 3.8)
project(ps5_odrive_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Encoder.msg"
)

ament_python_install_package(${PROJECT_NAME})
ament_package()