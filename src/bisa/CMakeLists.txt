cmake_minimum_required(VERSION 3.5)
project(bisa)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LapInfo.msg"
  "msg/MPCPerformance.msg"
  "msg/ZoneReservation.msg"
  "msg/ParkingSpot.msg"
  "msg/DriveMode.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# ltv_mpc_core
add_library(ltv_mpc_core SHARED
  src/ltv_mpc/ltv_model.cpp
  src/ltv_mpc/ltv_cost.cpp
  src/ltv_mpc/ltv_solver.cpp
  src/ltv_mpc/ltv_mpc.cpp
)
ament_target_dependencies(ltv_mpc_core rclcpp geometry_msgs nav_msgs)
target_include_directories(ltv_mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${EIGEN3_INCLUDE_DIRS} /usr/include/eigen3)
target_link_libraries(ltv_mpc_core Eigen3::Eigen osqp::osqp)

# rti_nmpc_core
add_library(rti_nmpc_core SHARED
  src/rti_nmpc/rti_nmpc.cpp
)
ament_target_dependencies(rti_nmpc_core rclcpp geometry_msgs nav_msgs)
target_include_directories(rti_nmpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rti_nmpc>
  ${EIGEN3_INCLUDE_DIRS} /usr/include/eigen3)
target_link_libraries(rti_nmpc_core Eigen3::Eigen osqp::osqp)

# mpc_path_tracker_cpp (Ioniq5 파라미터)
add_executable(mpc_path_tracker_cpp src/mpc_path_tracker_cpp.cpp)
ament_target_dependencies(mpc_path_tracker_cpp rclcpp std_msgs geometry_msgs nav_msgs)
target_link_libraries(mpc_path_tracker_cpp ltv_mpc_core rti_nmpc_core)
target_include_directories(mpc_path_tracker_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  /usr/include/eigen3)

if(TARGET ${PROJECT_NAME})
  rosidl_target_interfaces(mpc_path_tracker_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

install(TARGETS ltv_mpc_core rti_nmpc_core mpc_path_tracker_cpp
  EXPORT bisa_targets
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
