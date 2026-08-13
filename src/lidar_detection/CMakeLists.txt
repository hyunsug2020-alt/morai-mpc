cmake_minimum_required(VERSION 3.0.2)
project(lidar_detection)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  morai_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dataset_recording_guard.py
  scripts/detection_fusion_node.py
  scripts/lidar_dataset_recorder.py
  scripts/object_label_visualizer.py
  scripts/pointpillar_detector.py
  scripts/validate_lidar_dataset.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
