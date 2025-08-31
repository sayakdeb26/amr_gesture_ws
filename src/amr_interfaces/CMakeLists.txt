cmake_minimum_required(VERSION 3.5)
project(amr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# List ALL your .msg files here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Intent.msg"
  "msg/UnknownGesture.msg"
  "msg/ConfirmRequest.msg"
  "msg/ConfirmReply.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
