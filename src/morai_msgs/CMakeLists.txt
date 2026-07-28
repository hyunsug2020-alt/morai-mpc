cmake_minimum_required(VERSION 3.0.2)
project(morai_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

file(GLOB MSG_FILES msg/*.msg)

add_message_files(FILES
  CollisionData.msg
  CtrlCmd.msg
  EgoVehicleStatus.msg
  EventInfo.msg
  GPSMessage.msg
  GhostCmd.msg
  GetTrafficLightStatus.msg
  IntersectionControl.msg
  IntersectionStatus.msg
  Lamps.msg
  MapSpec.msg
  MoraiSimProcHandle.msg
  MoraiSimProcStatus.msg
  MoraiSrvResponse.msg
  MultiEgoSetting.msg
  NpcGhostCmd.msg
  NpcGhostInfo.msg
  ObjectStatus.msg
  ObjectStatusList.msg
  RadarDetection.msg
  RadarDetections.msg
  ReplayInfo.msg
  SaveSensorData.msg
  ScenarioLoad.msg
  SensorPosControl.msg
  SetTrafficLight.msg
  TrafficLightIndex.msg
  TrafficLightInfo.msg
  VehicleCollisionData.msg
  VehicleSpec.msg
  VehicleSpecIndex.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
