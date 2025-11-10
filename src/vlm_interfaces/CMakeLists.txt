cmake_minimum_required(VERSION 3.16)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
project(vlm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClipReady.msg"
  "msg/ConfirmReply.msg"
  "msg/ConfirmRequest.msg"
  "msg/Intent.msg"
  "msg/KeypointsWindow.msg"
  "msg/RecorderRequest.msg"
  "msg/TrainingExample.msg"
  "msg/UnknownGesture.msg"
  "srv/InferClip.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

