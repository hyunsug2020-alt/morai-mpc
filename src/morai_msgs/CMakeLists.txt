cmake_minimum_required(VERSION 3.0.2)
project(morai_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg msg/*.msg)
add_message_files(FILES ${MSG_FILES})

add_service_files(FILES
  MoraiEventCmdSrv.srv
  MoraiMapSpecSrv.srv
  MoraiScenarioLoadSrv.srv
  MoraiSimProcSrv.srv
  MoraiSyncModeAddObjectSrv.srv
  MoraiSyncModeCmdSrv.srv
  MoraiSyncModeCtrlCmdSrv.srv
  MoraiSyncModeRemoveObjectSrv.srv
  MoraiSyncModeSetGearSrv.srv
  MoraiSyncModeSLSrv.srv
  MoraiTLInfoSrv.srv
  MoraiVehicleSpecSrv.srv
  MoraiWaitForTickSrv.srv
  PREventSrv.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
