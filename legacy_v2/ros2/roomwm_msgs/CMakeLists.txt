cmake_minimum_required(VERSION 3.8)
project(roomwm_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrameRef.msg"
  "msg/FrameInfo.msg"
  "msg/ActuationInterval.msg"
  "msg/FramePacket.msg"
  "msg/PerceptionState.msg"
  "msg/ActuationProposal.msg"
  "msg/ActuationAck.msg"
  "msg/TaskRequest.msg"
  "msg/Stop.msg"
  "msg/SafetyStatus.msg"
  "msg/PlannerStatus.msg"
  "action/ApproachPerson.action"
  DEPENDENCIES std_msgs sensor_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
