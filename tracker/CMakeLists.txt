cmake_minimum_required(VERSION 2.8.3)
project(tracker)

# Set debug flag.
set(ROS_BUILD_TYPE Debug)

# Set c++11 cmake flags.
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  sensor_msgs
	  std_msgs
	  tf
	  cv_bridge
	  image_transport
	  pcl_conversions
	  pcl_ros
	  darknet_ros_msgs
	  octomap_ros
)

include_directories(
	${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
	${OCTOMAP_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs
	INCLUDE_DIRS include
)


add_executable(node_tracker
	src/Strings.cpp
	src/HSVHistogram.cpp
	src/Categories.cpp
	src/Pblob.cpp
	src/MapPlobs.cpp
	src/HSLTool.cpp
	src/OctoMap.cpp
	src/OctoMaps.cpp
	src/node_tracker.cpp
)

target_link_libraries(node_tracker ${catkin_LIBRARIES})
