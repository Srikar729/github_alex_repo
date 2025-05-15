cmake_minimum_required(VERSION 3.8)
project(alex_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs)

set(msg_files
  "msg/CobotAction.msg"
  "msg/CobotActionOption.msg"
  "msg/ActionFileInterface.msg"
  "msg/ActionFileInterfaceReference.msg"
  "msg/CobotUpdates.msg"
  "msg/SocketMessage.msg"
  "msg/MqttMessage.msg"
  "msg/LiquidVolumeStatus.msg"
  "msg/PHMeterStatus.msg"
  "msg/QuantityValue.msg"
  "msg/LiquidDosingStatus.msg"
  "msg/SerialMessage.msg"
  "msg/Weight.msg"
  "msg/NodeItem.msg"
  "msg/ActiveNodes.msg"
  "msg/ActiveNodesStatus.msg"
  "msg/MagneticStirrer.msg"
  "msg/Homogenizer.msg"
  "msg/ModeStatus.msg"
  "msg/CentrifugeStatus.msg"
  "msg/Sonicator.msg"
  "msg/TemperatureStatus.msg"
)

set(srv_files
  "srv/PauseResumeAction.srv"
  "srv/ArmCommand.srv"
  "srv/LiquidDoserCommand.srv"
  "srv/MeasurePhCommand.srv"
  "srv/SerialConnectionStatus.srv"
  "srv/LiquidLevelCommand.srv"
  "srv/PHMeterCommand.srv"
  "srv/WeightCommand.srv"
  "srv/ActivatingNodes.srv"
  "srv/ControlMode.srv"
  "srv/ObjectPoseDetection.srv"
  "srv/MagneticStirrerCommand.srv"
  "srv/HomogenizerCommand.srv"
  "srv/SwitchCamera.srv"
  "srv/MqttAttributes.srv"
  "srv/CentrifugeCommand.srv"
  "srv/SonicatorCommand.srv"
  "srv/DCActuator.srv"
  "srv/MeasureWeights.srv"
  "srv/TemperatureCommand.srv"
  "srv/StepperActuatorCommand.srv"
  "srv/GripperCommand.srv"
  "srv/RobotCommand.srv"
)

set(action_files
  "action/ExecuteActionFile.action"
  "action/ObjectPickUp.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}

  DEPENDENCIES geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
