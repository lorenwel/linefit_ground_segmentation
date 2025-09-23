cmake_minimum_required(VERSION 3.8)
project(linefit_ground_segmentation_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ground_segmentation_node.cc
)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  PCL
)

ament_auto_add_executable(ground_segmentation_test_node
  src/ground_segmentation_test_node.cc
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "SegmentationNode"
  EXECUTABLE ground_segmentation_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    ../doc
    launch
)