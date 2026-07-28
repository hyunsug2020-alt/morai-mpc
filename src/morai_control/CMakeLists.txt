cmake_minimum_required(VERSION 3.0.2)
project(morai_control)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. ROS 패키지 의존성 탐색
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2
  tf2_ros
  morai_msgs
)

# 2. 수학 및 최적화 라이브러리 탐색 (Eigen3)
find_package(Eigen3 REQUIRED)

# 3. catkin 패키지 선언
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf tf2 tf2_ros morai_msgs
)

# [수정된 부분] 4. 헤더 파일 경로 설정 (OSQP가 설치될 수 있는 모든 경로 강제 지정)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include
  /usr/local/include/osqp
  /usr/include
  /usr/include/osqp
)

# [수정된 부분] 5. 라이브러리 경로 설정 (OSQP 라이브러리 경로 보강)
link_directories(
  /usr/local/lib
  /usr/lib
)

# 6. 파이썬 스크립트 설치 경로 지정
catkin_install_python(PROGRAMS
  scripts/ego_drive.py
  scripts/eskf_node.py
  scripts/ego_status_from_eskf.py
  scripts/odometry_accuracy_validator.py
  scripts/validate_eskf_noise.py
  scripts/validate_eskf_slam.py
  scripts/stress_eskf_slam.py
  scripts/build_eskf_validation_report.py
  scripts/sensor_tf_from_config.py
  scripts/heading_gui.py
  scripts/lio_odometry_experiment.py
  scripts/ltv_mpc_follower_v2.py
  scripts/path_follower.py
  scripts/path_maker.py
  scripts/unified_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_eskf_validation.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 7. C++ MPC 노드 실행 파일 생성
add_executable(ltv_mpc_node 
  src/ltv_mpc_node.cpp 
  src/ltv_model.cpp 
  src/ltv_cost.cpp 
  src/ltv_solver.cpp
)

# 8. 빌드 순서 강제 지정: 메시지 파일들이 먼저 생성되도록 대기시킴
add_dependencies(ltv_mpc_node ${catkin_EXPORTED_TARGETS})

# 9. 라이브러리 링크 (ROS 라이브러리 + OSQP 솔버 연동)
target_link_libraries(ltv_mpc_node
  ${catkin_LIBRARIES}
  osqp
  /usr/lib/x86_64-linux-gnu/libjsoncpp.so
)
