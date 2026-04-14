cmake_minimum_required(VERSION 3.8)
project(lunabot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Excavate.action"
  "action/Deposit.action"
  "msg/DrivetrainCommand.msg"
  "msg/DrivetrainStatus.msg"
  "msg/DrivetrainTelemetry.msg"
  "msg/ExcavationCommand.msg"
  "msg/ExcavationStatus.msg"
  "msg/ExcavationTelemetry.msg"
  "msg/LocalisationStartZoneStatus.msg"
  "srv/ExcavationJog.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
