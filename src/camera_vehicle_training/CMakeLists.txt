cmake_minimum_required(VERSION 3.0.2)
project(camera_vehicle_training)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  morai_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/camera_vehicle_detector_node.py
  scripts/camera_vehicle_dataset_recorder.py
  scripts/validate_camera_vehicle_dataset.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_geometry.py)
endif()
