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
  nav_msgs
  morai_msgs
  tf
  pcl_ros
  pcl_conversions
)
find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)
find_library(OSQP_LIBRARY NAMES osqp PATHS /usr/local/lib)
find_path(OSQP_INCLUDE_DIR NAMES osqp.h PATHS /usr/local/include/osqp)

# JSONCPP
find_library(JSONCPP_LIBRARY NAMES jsoncpp)
include_directories(/usr/include/jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moraimpc
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs morai_msgs tf pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OSQP_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# MPC Core Library (LTV + NMPC + RTI-NMPC + Hybrid A*)
add_library(moraimpc_core
  src/ltv_model.cpp
  src/ltv_cost.cpp
  src/ltv_solver.cpp
  src/nmpc_controller.cpp
  src/rti_nmpc_controller.cpp
  src/hybrid_astar.cpp
)
target_link_libraries(moraimpc_core ${catkin_LIBRARIES} ${OSQP_LIBRARY} ${JSONCPP_LIBRARY})

# Path Follower Node (C++)
add_executable(path_follower_node src/path_follower.cpp)
target_link_libraries(path_follower_node moraimpc_core ${catkin_LIBRARIES})

# MPC Dashboard Node (C++ / OpenCV)
find_package(OpenCV REQUIRED)
add_executable(mpc_dashboard_node src/mpc_dashboard_node.cpp)
target_include_directories(mpc_dashboard_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(mpc_dashboard_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${JSONCPP_LIBRARY})

# Cone Detector Node (PCL)
add_executable(cone_detector_node src/cone_detector_node.cpp)
target_link_libraries(cone_detector_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# LiDAR Calibration Visualization (PCL + OpenCV)
add_executable(lidar_calib_viz_node src/lidar_calib_viz_node.cpp)
target_include_directories(lidar_calib_viz_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lidar_calib_viz_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Parking Planner Node (Hybrid A* + 콘 → 박스 → goal)
add_executable(parking_planner_node src/parking_planner_node.cpp)
target_link_libraries(parking_planner_node moraimpc_core ${catkin_LIBRARIES})
