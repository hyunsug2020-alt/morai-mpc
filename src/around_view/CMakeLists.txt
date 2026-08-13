cmake_minimum_required(VERSION 3.0.2)
project(around_view)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS cv_bridge rospy sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/udp_camera_node.py
  scripts/around_view_node.py
  scripts/lidar_camera_calibration_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
