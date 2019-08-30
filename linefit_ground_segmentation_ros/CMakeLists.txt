cmake_minimum_required(VERSION 2.8.3)
project(linefit_ground_segmentation_ros)

set(ROS_COMPONENTS message_generation message_runtime rosbag roscpp tf linefit_ground_segmentation tf2_ros tf2_eigen utils velodyne_pointcloud)
find_package(catkin COMPONENTS ${ROS_COMPONENTS})

catkin_package()

add_definitions(-std=c++11)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ground_segmentation_node src/ground_segmentation_node.cc)
target_link_libraries(ground_segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#add_doxygen(NOT_AUTOMATIC)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(${PROJECT_NAME}_headers SOURCES ${LibFiles})
