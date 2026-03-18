# Install script for directory: /home/david/moraimpc_ros1/src/morai_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/david/moraimpc_ros1/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs/msg" TYPE FILE FILES
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/CollisionData.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/CtrlCmd.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/EgoVehicleStatus.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/EventInfo.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/GPSMessage.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/GhostCmd.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/GetTrafficLightStatus.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/IntersectionControl.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/IntersectionStatus.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/Lamps.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/MapSpec.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/MoraiSimProcHandle.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/MoraiSimProcStatus.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/MoraiSrvResponse.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/MultiEgoSetting.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/NpcGhostCmd.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/NpcGhostInfo.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/ObjectStatus.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/ObjectStatusList.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/RadarDetection.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/RadarDetections.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/ReplayInfo.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/SaveSensorData.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/ScenarioLoad.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/SensorPosControl.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/SetTrafficLight.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/TrafficLightIndex.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/TrafficLightInfo.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/VehicleCollisionData.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/VehicleSpec.msg"
    "/home/david/moraimpc_ros1/src/morai_msgs/msg/VehicleSpecIndex.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs/cmake" TYPE FILE FILES "/home/david/moraimpc_ros1/build/morai_msgs/catkin_generated/installspace/morai_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/david/moraimpc_ros1/devel/include/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/david/moraimpc_ros1/devel/share/roseus/ros/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/david/moraimpc_ros1/devel/share/common-lisp/ros/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/david/moraimpc_ros1/devel/share/gennodejs/ros/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/david/moraimpc_ros1/devel/lib/python3/dist-packages/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/david/moraimpc_ros1/devel/lib/python3/dist-packages/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/david/moraimpc_ros1/build/morai_msgs/catkin_generated/installspace/morai_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs/cmake" TYPE FILE FILES "/home/david/moraimpc_ros1/build/morai_msgs/catkin_generated/installspace/morai_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs/cmake" TYPE FILE FILES
    "/home/david/moraimpc_ros1/build/morai_msgs/catkin_generated/installspace/morai_msgsConfig.cmake"
    "/home/david/moraimpc_ros1/build/morai_msgs/catkin_generated/installspace/morai_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs" TYPE FILE FILES "/home/david/moraimpc_ros1/src/morai_msgs/package.xml")
endif()

