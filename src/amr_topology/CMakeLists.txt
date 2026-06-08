cmake_minimum_required(VERSION 3.8)
project(amr_topology)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(topology_follower src/topology_follower.cpp)
target_compile_features(topology_follower PUBLIC cxx_std_17)
ament_target_dependencies(
  topology_follower
  ament_index_cpp
  geometry_msgs
  rclcpp
  std_msgs
  tf2_ros
)
target_link_libraries(topology_follower yaml-cpp)

add_executable(
  mission_loop
  src/mission_loop.cpp
  src/local_astar_pure_pursuit.cpp
)
target_compile_features(mission_loop PUBLIC cxx_std_17)
target_include_directories(mission_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  mission_loop
  ament_index_cpp
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(mission_loop yaml-cpp)

install(TARGETS
  topology_follower
  mission_loop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  include
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
