cmake_minimum_required(VERSION 2.8.3)
project(linefit_ground_segmentation_ros)

find_package(catkin_simple 0.1.0 REQUIRED )

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -g)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ground_segmentation_node src/ground_segmentation_node.cc)
add_executable(ground_segmentation_test_node src/ground_segmentation_test_node.cc)
target_link_libraries(ground_segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(ground_segmentation_test_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_doxygen(NOT_AUTOMATIC)

cs_install()
cs_export()

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
