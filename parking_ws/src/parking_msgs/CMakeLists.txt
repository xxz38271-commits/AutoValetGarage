cmake_minimum_required(VERSION 3.5)
project(parking_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Mission.msg"
  "msg/MissionArray.msg"
  "msg/SlotState.msg"
  "msg/SlotStates.msg"
  "msg/MissionStatus.msg"
  "msg/ParkingCommand.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

