cmake_minimum_required(VERSION 3.0.2)
project(icp_slam)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  pcl_ros
  roscpp
  tf2
  tf2_sensor_msgs
  fast_gicp
)

add_service_files(
  FILES
  SaveMap.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp tf2 sensor_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(icp_slam_node src/icp_slam_node.cpp src/icp_slam.cpp)
target_link_libraries(icp_slam_node ${catkin_LIBRARIES})
