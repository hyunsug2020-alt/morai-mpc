cmake_minimum_required(VERSION 3.0.2)
project(morai_launch)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/morai_topic_init.py
  scripts/morai_clock_bridge.py
  scripts/gps_signal_gate.py
  scripts/competition_udp_bridge.py
  scripts/competition_udp_preflight.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_gps_signal_gate.py)
  catkin_add_nosetests(test/test_competition_udp_protocol.py)
  catkin_add_nosetests(test/test_competition_udp_launch_safety.py)
  catkin_add_nosetests(test/test_competition_udp_preflight.py)
  catkin_add_nosetests(test/test_perception_backends.py)
endif()
