cmake_minimum_required(VERSION 3.16)
project(luxi_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QuestInput.msg"
  "msg/ArmTargets.msg"
  "msg/ArmCommand.msg"
  "msg/ArmState.msg"
  "msg/CameraFrame.msg"
  "msg/RGBDFrame.msg"
  "srv/Command.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
