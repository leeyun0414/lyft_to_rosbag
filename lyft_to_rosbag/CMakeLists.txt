cmake_minimum_required(VERSION 2.8.3)
project(lyft_to_rosbag)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  std_msgs
  cv_bridge
  #darknet_ros_msgs
  geometry_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL 1.7 REQUIRED )


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lyft
  CATKIN_DEPENDS roscpp rospy std_msgs 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_raw_node src/lyft_to_rosbag_raw.cpp)
add_executable(${PROJECT_NAME}_node src/lyft_to_rosbag.cpp)
link_directories(${PCL_LIBRARY_DIRS})
 target_link_libraries(${PROJECT_NAME}_node
   jsoncpp
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

target_link_libraries(${PROJECT_NAME}_raw_node
   jsoncpp
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )
