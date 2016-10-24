cmake_minimum_required(VERSION 2.8.3)
project(ground_segmentation)

find_package(catkin_simple 0.1.0 REQUIRED )

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -g)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

cs_add_library(${PROJECT_NAME} src/ground_segmentation.cc src/segment.cc src/bin.cc)

add_executable(ground_segmentation_node src/ground_segmentation_node.cc)
target_link_libraries(ground_segmentation_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBRARIES})

#add_doxygen(NOT_AUTOMATIC)

cs_install()
cs_export()

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
