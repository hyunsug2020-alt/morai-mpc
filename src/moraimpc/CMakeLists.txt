cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)
project(moraimpc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  morai_msgs
  tf
  tf2_ros
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_library(OSQP_LIBRARY
  NAMES osqp
  PATHS /usr/local/lib /opt/ros/noetic/lib
)
find_path(OSQP_INCLUDE_DIR
  NAMES osqp.h
  PATHS /usr/local/include /opt/ros/noetic/include
  PATH_SUFFIXES osqp
)

if(NOT OSQP_LIBRARY OR NOT OSQP_INCLUDE_DIR)
  message(FATAL_ERROR
    "OSQP not found. Install OSQP or source the ROS Noetic environment.")
endif()

# JSONCPP
find_library(JSONCPP_LIBRARY NAMES jsoncpp)
include_directories(/usr/include/jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moraimpc
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs
                 visualization_msgs morai_msgs tf tf2_ros pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OSQP_INCLUDE_DIR}
)

# MPC Core Library (LTV + NMPC + RTI-NMPC)
add_library(moraimpc_core
  src/ltv_model.cpp
  src/ltv_cost.cpp
  src/ltv_solver.cpp
  src/nmpc_controller.cpp
  src/rti_nmpc_controller.cpp
)
add_dependencies(moraimpc_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(moraimpc_core ${catkin_LIBRARIES} ${OSQP_LIBRARY} ${JSONCPP_LIBRARY})

# Path Follower Node (C++)
add_executable(path_follower_node src/path_follower.cpp)
add_dependencies(path_follower_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower_node moraimpc_core ${catkin_LIBRARIES})

# MPC Dashboard Node (C++ / OpenCV)
find_package(OpenCV REQUIRED)
add_executable(mpc_dashboard_node src/mpc_dashboard_node.cpp)
add_dependencies(mpc_dashboard_node ${catkin_EXPORTED_TARGETS})
target_include_directories(mpc_dashboard_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(mpc_dashboard_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${JSONCPP_LIBRARY})

add_executable(simple_dashboard_node src/simple_dashboard_node.cpp)
add_dependencies(simple_dashboard_node ${catkin_EXPORTED_TARGETS})
target_include_directories(simple_dashboard_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(simple_dashboard_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${JSONCPP_LIBRARY})

# morai_sensor.json-mounted 3D LiDAR cone detector
add_executable(cone_detector_node src/cone_detector_node.cpp)
add_dependencies(cone_detector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cone_detector_node ${catkin_LIBRARIES})
