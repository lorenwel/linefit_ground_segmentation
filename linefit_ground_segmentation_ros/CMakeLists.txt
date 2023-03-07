cmake_minimum_required(VERSION 2.8.3)
project(linefit_ground_segmentation_ros)

find_package(catkin REQUIRED
    eigen_conversions
    linefit_ground_segmentation
    pcl_conversions
    pcl_ros
    roscpp
    tf2_ros
)

catkin_package(
    CATKIN_DEPENDS
        eigen_conversions
        linefit_ground_segmentation
        pcl_conversions
        pcl_ros
        roscpp
        tf2_ros
)
message(catkin_INCLUDE_DIRS="${catkin_INCLUDE_DIRS}")

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(ground_segmentation_node src/ground_segmentation_node.cc)
add_executable(ground_segmentation_test_node src/ground_segmentation_test_node.cc)
target_link_libraries(ground_segmentation_node ${catkin_LIBRARIES})
target_link_libraries(ground_segmentation_test_node ${catkin_LIBRARIES})
target_compile_features(ground_segmentation_node PRIVATE cxx_std_14)
target_compile_features(ground_segmentation_test_node PRIVATE cxx_std_14)

#add_doxygen(NOT_AUTOMATIC)

install(TARGETS ground_segmentation_node ground_segmentation_test_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/segmentation.launch
        launch/test.launch
        launch/segmentation_params.yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
