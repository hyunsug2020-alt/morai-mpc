cmake_minimum_required(VERSION 3.0.2)
project(morai_eskf_runtime)

find_package(catkin REQUIRED COMPONENTS
  eskf
  geometry_msgs
  morai_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS eskf geometry_msgs morai_msgs nav_msgs rospy sensor_msgs std_msgs tf tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/pure_odometry_node.py
  scripts/ego_status_from_eskf.py
  scripts/sensor_tf_from_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

