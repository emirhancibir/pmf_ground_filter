cmake_minimum_required(VERSION 3.0.2)
project(pmf_ground_filter)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pmf_ground_filter
  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_executable(pmf_ground_filter src/pmf_ground_filter.cpp)
target_link_libraries(pmf_ground_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
