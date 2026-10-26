cmake_minimum_required(VERSION 3.0.2)
project(moraimpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  morai_msgs
)
find_package(Eigen3 REQUIRED)
find_library(OSQP_LIBRARY NAMES osqp PATHS /usr/local/lib /opt/ros/noetic/lib)
find_path(OSQP_INCLUDE_DIR NAMES osqp.h
  PATHS /usr/local/include /usr/include /opt/ros/noetic/include
  PATH_SUFFIXES osqp)
find_library(JSONCPP_LIBRARY NAMES jsoncpp)

if(NOT OSQP_LIBRARY OR NOT OSQP_INCLUDE_DIR)
  message(FATAL_ERROR "OSQP not found. Install OSQP before building moraimpc.")
endif()
if(NOT JSONCPP_LIBRARY)
  message(FATAL_ERROR "jsoncpp not found. Install libjsoncpp-dev.")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs nav_msgs visualization_msgs morai_msgs
)

catkin_install_python(PROGRAMS
  nodes/hdmap_lane_change.py
  nodes/hdmap_lane_change_bspline_test.py
  nodes/local_bypass_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include
  /usr/include/jsoncpp
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OSQP_INCLUDE_DIR}
)

add_library(moraimpc_core
  src/controller_consistency.cpp
  src/command_boundary.cpp
  src/ltv_model.cpp
  src/ltv_cost.cpp
  src/ltv_solver.cpp
  src/nmpc_controller.cpp
  src/rti_nmpc_controller.cpp
)
add_dependencies(moraimpc_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(moraimpc_core ${catkin_LIBRARIES} ${OSQP_LIBRARY} ${JSONCPP_LIBRARY})

add_executable(path_follower_node src/path_follower.cpp)
add_dependencies(path_follower_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower_node moraimpc_core ${catkin_LIBRARIES} ${JSONCPP_LIBRARY})

install(TARGETS moraimpc_core path_follower_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config data launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
