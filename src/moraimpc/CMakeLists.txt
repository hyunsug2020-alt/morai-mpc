cmake_minimum_required(VERSION 3.0.2)
project(moraimpc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  morai_msgs
  tf
)

find_package(Eigen3 REQUIRED)
find_library(OSQP_LIBRARY NAMES osqp)
find_path(OSQP_INCLUDE_DIR NAMES osqp.h)

# JSONCPP
find_library(JSONCPP_LIBRARY NAMES jsoncpp)
include_directories(/usr/include/jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moraimpc
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs morai_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OSQP_INCLUDE_DIR}
)

# MPC Core Library
add_library(moraimpc_core
  src/ltv_model.cpp
  src/ltv_cost.cpp
  src/ltv_solver.cpp
)
target_link_libraries(moraimpc_core ${catkin_LIBRARIES} ${OSQP_LIBRARY} ${JSONCPP_LIBRARY})

# Main Node (C++)
add_executable(ltv_mpc_node src/ltv_mpc_node.cpp)
target_link_libraries(ltv_mpc_node moraimpc_core ${catkin_LIBRARIES})
