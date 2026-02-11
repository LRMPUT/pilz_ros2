cmake_minimum_required(VERSION 3.8)
project(pilz_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IsBrakeTestRequiredResult.msg"
  "msg/OperationModes.msg"
  "srv/BrakeTest.srv"
  "srv/GetOperationMode.srv"
  "srv/GetSpeedOverride.srv"
  "srv/IsBrakeTestRequired.srv"
  "srv/SetSpeedLimit.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
