cmake_minimum_required(VERSION 3.0.2)
project(eskf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs morai_msgs nav_msgs rospy sensor_msgs std_msgs tf tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/eskf_node.py
  scripts/eskf_monitor_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_eskf_modules.py)
endif()
